"""This script runs on the Raspberry Pi, which is connected to Arduino via USB.
The Arduino is connected to a pair of motors and exports the IMU data over seria.

This script has the following roles
* Receiving commands and data from the Flask client
* Sending the motor commands to the Arduino and reading the IMU
* Sending the IMU and camera data through flask
* Displaying the existing info on the browser

The client will be a Flask/ROS2 interface node. 
It will convert the received data from Flask to ROS2 (IMU and Image).

The original serial interface is implemented in C++ under 
`src/robin_firmware/robin_firmware_cpp/include/robin_firmware_cpp/interface.hpp`.

The twist callback command conversion is originally implemented in C++ under
`src/robin_ros2/robin_ros2_cpp/src/robin_platform_interface_node.cpp` which
goes through the same interface under `robin_firmware_cpp`.

The camera interface is already implemented scripts/stream_pi_cam.py and must use the same configuration file
in `settings.yaml`.

Finally, this script will run everytime the Raspberry Pi boots up.
"""

import threading
import time
import serial
import yaml
from pathlib import Path
from flask import Flask, Response, jsonify, request, make_response
import cv2
from picamera2 import Picamera2

# Load camera settings
settings_path = Path(__file__).parent.parent / "settings.yaml"
with open(settings_path, "r") as f:
    settings = yaml.safe_load(f)
cam_cfg = settings["camera"]

# Serial interface configuration
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600


class SerialPortInterface:
    def __init__(self, port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.lock = threading.Lock()

    def send_motor_command(self, left, right):
        # Example: send two bytes for left/right motor
        with self.lock:
            self.ser.write(bytes([left, right]))

    def read_imu(self):
        # Example: read IMU data from serial
        with self.lock:
            self.ser.write(b"I")  # Request IMU data
            line = self.ser.readline().decode().strip()
            # Parse IMU data (customize as needed)
            try:
                ax, ay, az, gx, gy, gz = map(float, line.split(","))
                return {"accel": [ax, ay, az], "gyro": [gx, gy, gz]}
            except Exception:
                return None


class CameraInterface:
    def __init__(self, width, height, fps, hflip, vflip):
        self.width = width
        self.height = height
        self.fps = fps
        self.hflip = hflip
        self.vflip = vflip
        self.lock = threading.Lock()
        self.frame = None
        self.running = True

        self.cam = Picamera2()
        config = self.cam.create_preview_configuration({"size": (width, height)})
        self.cam.configure(config)
        self.cam.start()
        self._thread = threading.Thread(target=self._picamera_loop, daemon=True)
        self._thread.start()

    def _picamera_loop(self):
        while self.running:
            try:
                frame = self.cam.capture_array()
                with self.lock:
                    self.frame = frame
            except Exception:
                time.sleep(0.05)
            time.sleep(1.0 / self.fps)

    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
            if self.hflip:
                frame = cv2.flip(frame, 1)
            if self.vflip:
                frame = cv2.flip(frame, 0)
            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
            if not ok:
                return None
            return jpg.tobytes()

    def shutdown(self):
        self.running = False
        self._thread.join(timeout=1.0)
        self.cam.stop()


class PiPlatformInterface:
    def __init__(self):
        self.serial = SerialPortInterface()
        self.camera = CameraInterface(
            width=cam_cfg["width"],
            height=cam_cfg["height"],
            fps=cam_cfg["framerate"],
            hflip=cam_cfg["hflip"],
            vflip=cam_cfg["vflip"],
        )
        self.latest_imu = None
        self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self.imu_thread.start()

    def _imu_loop(self):
        while True:
            imu = self.serial.read_imu()
            if imu:
                self.latest_imu = imu
            time.sleep(0.1)

    def send_motor_command(self, left, right):
        self.serial.send_motor_command(left, right)

    def get_imu(self):
        return self.latest_imu

    def get_camera_snapshot(self):
        return self.camera.get_jpeg()

    def shutdown(self):
        self.camera.shutdown()


# Flask app
app = Flask(__name__)
platform = PiPlatformInterface()


@app.route("/cmd", methods=["POST"])
def cmd():
    data = request.json
    left = int(data.get("left", 0))
    right = int(data.get("right", 0))
    platform.send_motor_command(left, right)
    return jsonify({"success": True})


@app.route("/imu")
def imu():
    imu_data = platform.get_imu()
    if imu_data:
        return jsonify(imu_data)
    return jsonify({"error": "No IMU data"}), 503


@app.route("/camera")
def camera():
    img = platform.get_camera_snapshot()
    if img:
        resp = make_response(img)
        resp.headers["Content-Type"] = "image/jpeg"
        return resp
    return "No image", 503


@app.route("/")
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Pi Platform Interface</title>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=700">
        <style>
            body { text-align: center; font-family: sans-serif; background: #f5f5f5; }
            button { font-size: 24px; padding: 10px 20px; margin: 5px; }
            #imuData { font-size: 18px; margin: 10px; }
        </style>
    </head>
    <body>
        <h1>Pi Platform Interface</h1>
        <div>
            <div id="cameraFeed">
                <img id="cameraImage" src="" alt="Camera Feed" style="width: 640px; height: 480px;"><br/>
                <input type="checkbox" id="enableCameraFeed" checked>
                <label for="enableCameraFeed">Enable Camera Feed</label><br>
            </div>
            <div id="imuData">
                <b>IMU:</b>
                <span id="imuText">Loading...</span>
            </div>
            <div id="botControls">
                <button onclick="sendCommand('forward')">↑</button><br/>
                <button onclick="sendCommand('left')">←</button>
                <button onclick="sendCommand('stop')">⏹</button>
                <button onclick="sendCommand('right')">→</button><br/>
                <button onclick="sendCommand('backward')">↓</button><br/>
            </div>
        </div>
        <script>
            function sendCommand(cmd) {
                let left = 0, right = 0;
                if (cmd === "forward") { left = 255; right = 255; }
                else if (cmd === "backward") { left = -255; right = -255; }
                else if (cmd === "left") { left = -255; right = 255; }
                else if (cmd === "right") { left = 255; right = -255; }
                else if (cmd === "stop") { left = 0; right = 0; }
                fetch('/cmd', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({left: left, right: right})
                });
            }
            function fetchCameraSnapshot() {
                const checkbox = document.getElementById('enableCameraFeed');
                const element = document.getElementById('cameraImage');
                if (!checkbox.checked) {
                    element.hidden = true;
                    return;
                }
                element.hidden = false;
                fetch('/camera')
                    .then(response => response.blob())
                    .then(blob => {
                        if (blob.size > 0) {
                            element.src = URL.createObjectURL(blob);
                        }
                    });
            }
            function fetchImu() {
                fetch('/imu')
                    .then(response => response.json())
                    .then(data => {
                        let txt = "No data";
                        if (data.accel && data.gyro) {
                            txt = `Accel: [${data.accel.map(x=>x.toFixed(2)).join(", ")}] Gyro: [${data.gyro.map(x=>x.toFixed(2)).join(", ")}]`;
                        }
                        document.getElementById('imuText').textContent = txt;
                    });
            }
            setInterval(fetchCameraSnapshot, 2000);
            setInterval(fetchImu, 1000);
        </script>
    </body>
    </html>
    """


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8000, threaded=True)
    finally:
        platform.shutdown()
