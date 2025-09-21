from pathlib import Path

import rclpy
import requests
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # Add this import

# Standard path in devcontainer
SETTINGS_PATH = Path("/home/dev/workspace/settings.yaml")


class DashboardNode(Node):
    def __init__(self, settings_path=SETTINGS_PATH):
        super().__init__("dashboard_node")
        with open(settings_path, "r") as f:
            self.settings_dict = yaml.safe_load(f)

        # ROS2 publishers and subscribers
        self.image_publisher = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 10
        )
        self.twist_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, 10
        )

        # Flask server configuration
        client_settings_dict = self.settings_dict["platform_client"]
        self.flask_server_url = f"http://{client_settings_dict['server_ip']}:{client_settings_dict['server_port']}"
        self.video_feed_url = f"{self.flask_server_url}/video_feed"
        self.cmd_url = f"{self.flask_server_url}/cmd"

        # Timer to fetch video feed periodically
        cam_fps = self.settings_dict["camera"]["framerate"]
        self.create_timer(
            1.0 / cam_fps, self.fetch_video_feed
        )  # Adjusted to fetch at the correct FPS

    def fetch_video_feed(self):
        try:
            response = requests.get(self.video_feed_url, stream=True, timeout=2)
            if response.status_code == 200:
                bytes_data = b""
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b"\xff\xd8")  # Start of JPEG
                    b = bytes_data.find(b"\xff\xd9")  # End of JPEG
                    if a != -1 and b != -1:
                        jpg = bytes_data[a : b + 2]
                        bytes_data = bytes_data[b + 2 :]

                        # Publish the JPEG data as a CompressedImage
                        compressed_image = CompressedImage()
                        compressed_image.header.stamp = self.get_clock().now().to_msg()
                        compressed_image.format = "jpeg"
                        compressed_image.data = jpg
                        self.image_publisher.publish(compressed_image)
                        break
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to fetch video feed: {e}")

    def twist_callback(self, msg: Twist):
        # Convert Twist message to JSON for Flask server
        left_speed = 0.0  # Placeholder for actual conversion logic
        right_speed = 0.0  # Placeholder for actual conversion logic
        data = {"left": left_speed, "right": right_speed}

        try:
            response = requests.post(self.cmd_url, json=data, timeout=2)
            if response.status_code != 200:
                self.get_logger().error(f"Failed to send command: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode(settings_path=SETTINGS_PATH)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
