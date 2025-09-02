#!/usr/bin/env python3
import base64
import time
import datetime
import subprocess
import threading
from pathlib import Path

import rclpy
from flask import Flask, Response, jsonify, send_file
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

OUT_DIR = Path("/home/dev/workspace/external_data/database")

INPUT_CAMERA_TOPIC = "/robin/sensors/image/undistorted/raw"
OUT_CMD_TOPIC = "/robin/controls/command_twist"
OUT_TASK_TOPIC = "/robin/controls/task"
MAX_LINEAR_VELOCITY_M_PER_SEC = 1.0
MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 1.0
ROS_THREAD_SPIN_TIME_MS = 200

app = Flask(__name__)
rclpy.init()

# HTML as a raw string
HTML_PATH = Path(__file__).parent.parent / "html" / "dashboard.html"


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")
        self.latest_image = None
        self.cmd_publisher = self.create_publisher(Twist, OUT_CMD_TOPIC, 100)
        self.task_publisher = self.create_publisher(String, OUT_TASK_TOPIC, 100)
        self.subscription = self.create_subscription(
            CompressedImage, INPUT_CAMERA_TOPIC, self.image_callback, 2
        )
        self.recording_bag = None

    def send_teleop_cmd(self, direction):
        msg = Twist()
        if direction == "forward":
            msg.linear.x = MAX_LINEAR_VELOCITY_M_PER_SEC
        elif direction == "backward":
            msg.linear.x = -MAX_LINEAR_VELOCITY_M_PER_SEC
        elif direction == "left":
            msg.angular.z = MAX_ANGULAR_VELOCITY_RAD_PER_SEC
        elif direction == "right":
            msg.angular.z = -MAX_ANGULAR_VELOCITY_RAD_PER_SEC
        elif direction == "stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_publisher.publish(msg)

    def image_callback(self, msg):
        self.latest_image = msg

    def start_recording(self):
        if not self.recording_bag:
            out_bags_dir = OUT_DIR / "bags"
            out_bags_dir.mkdir(parents=True, exist_ok=True)
            datetime_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.recording_bag = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    f"{out_bags_dir}/web_bag_{datetime_str}",
                    "-a",
                ]
            )
            self.get_logger().info("Recording started")
        else:
            self.get_logger().info("Already recording")

    def stop_recording(self):
        if self.recording_bag:
            self.recording_bag.terminate()
            self.recording_bag.wait()
            self.recording_bag = None
            self.get_logger().info("Recording stopped")
        else:
            self.get_logger().info("Not recording")

    def get_camera_snapshot(self) -> Response:
        if self.latest_image is None:
            return jsonify({"success": False, "message": "No image received yet"})
        encoded = base64.b64encode(self.latest_image.data).decode("utf-8")
        return jsonify({"success": True, "image": f"data:image/jpeg;base64,{encoded}"})

    def send_task(self, task_name: str):
        task_msg = String()
        task_msg.data = task_name
        self.task_publisher.publish(task_msg)


ros_node = DashboardNode()


@app.route("/record/start")
def start_recording():
    if ros_node.recording_bag is None:
        ros_node.start_recording()
        return "Recording started"
    return "Already recording"


@app.route("/record/stop")
def stop_recording():
    if ros_node.recording_bag is not None:
        ros_node.stop_recording()
        return "Recording stopped"
    return "Not recording"


@app.route("/task/<task_name>")
def handle_task(task_name: str):
    ros_node.send_task(task_name)
    return "Task sent"


@app.route("/camera_snapshot")
def camera_snapshot():
    return ros_node.get_camera_snapshot()


@app.route("/")
def index():
    return send_file(str(HTML_PATH), mimetype="text/html")


@app.route("/cmd/<direction>")
def handle_cmd(direction):
    ros_node.send_teleop_cmd(direction)
    return "OK"


@app.route("/record/status")
def recording_status():
    return jsonify({"recording": ros_node.recording_bag is not None})


# Spin ROS in a separate thread
def ros_spin():
    thread_spin_time_s = ROS_THREAD_SPIN_TIME_MS / 1000.0
    while rclpy.ok():
        rclpy.spin_once(ros_node)
        time.sleep(thread_spin_time_s)


threading.Thread(target=ros_spin, daemon=True).start()


def main():
    app.run(host="0.0.0.0", port=5000)


if __name__ == "__main__":
    main()
