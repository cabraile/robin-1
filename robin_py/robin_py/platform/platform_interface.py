from pathlib import Path

from robin_py.platform.camera_interface import CameraInterface
from robin_py.platform.motor_interface import MotorInterface

class PlatformInterface:
    def __init__(self, settings_path: Path):
        self.camera_interface = CameraInterface(settings_path=settings_path)
        self.motor_interface = MotorInterface(settings_path=settings_path)

    def send_motor_command(self, left: float, right: float):
        self.motor_interface.activate(left, right)

    def get_camera_snapshot(self):
        return self.camera_interface.get_jpeg()

    def shutdown(self):
        self.camera_interface.shutdown()