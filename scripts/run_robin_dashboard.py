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
from pathlib import Path
from flask import Flask, Response, jsonify, request, make_response
from robin_py.platform.platform_interface import PlatformInterface
import subprocess

# Load camera settings
WORKSPACE_DIR = Path(__file__).parent.parent
SETTINGS_PATH = WORKSPACE_DIR / "settings.yaml"
DASHBOARD_HTML_PATH = (WORKSPACE_DIR / "html") / "dashboard.html"

# Flask app
app = Flask(__name__)
platform = PlatformInterface(settings_path=SETTINGS_PATH)

@app.route("/cmd", methods=["POST"])
def cmd():
    data = request.json
    left = int(data.get("left", 0))
    right = int(data.get("right", 0))
    platform.send_motor_command(left, right)
    return jsonify({"success": True})


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
    with open(DASHBOARD_HTML_PATH) as f:
        return f.read()

def get_git_hash():
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=WORKSPACE_DIR, capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()
        else:
            return "Unknown"
    except Exception:
        return "Unknown"

@app.route("/update", methods=["POST"])
def update():
    try:
        result = subprocess.run(["git", "pull", "origin", "main"], cwd=WORKSPACE_DIR, capture_output=True, text=True)
        if result.returncode == 0:
            new_hash = get_git_hash()
            return jsonify({"success": True, "output": result.stdout, "git_hash": new_hash})
        else:
            return jsonify({"success": False, "error": result.stderr}), 500
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/git-hash", methods=["GET"])
def git_hash():
    return jsonify({"git_hash": get_git_hash()})

@app.route("/build", methods=["POST"])
def build():
    try:
        result = subprocess.run(["/bin/bash", str(WORKSPACE_DIR / "scripts/run_robin_deploy.sh")], capture_output=True, text=True)
        if result.returncode == 0:
            return jsonify({"success": True, "output": result.stdout})
        else:
            return jsonify({"success": False, "error": result.stderr}), 500
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8000, threaded=True)
    finally:
        platform.shutdown()
