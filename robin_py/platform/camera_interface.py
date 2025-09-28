import threading
import time
from pathlib import Path

import cv2
import yaml
from picamera2 import Picamera2

from robin_py.core.core import ImageStamped


class CameraInterface:
    def __init__(self, settings_path: Path):
        with open(settings_path, "r") as f:
            settings = yaml.safe_load(f)
        cam_cfg: dict = settings["camera"]
        self.width: int = cam_cfg["width"]
        self.height: int = cam_cfg["height"]
        self.fps: float = cam_cfg["framerate"]
        self.hflip: bool = cam_cfg["hflip"]
        self.vflip: bool = cam_cfg["vflip"]
        self.lock = threading.Lock()
        self.frame: ImageStamped | None = None
        self.running = True

        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            {"size": (self.width, self.height)}
        )
        self.cam.configure(config)
        self.cam.start()
        self._thread = threading.Thread(target=self._picamera_loop, daemon=True)
        self._thread.start()

    def _picamera_loop(self):
        while self.running:
            try:
                frame = self.cam.capture_array()
                with self.lock:
                    stamp = time.time()
                    self.frame = ImageStamped(image=frame, stamp=stamp)
            except Exception:
                time.sleep(0.05)
            time.sleep(1.0 / self.fps)

    def get_jpeg(self) -> bytes:
        with self.lock:
            if self.frame is None:
                return None
            frame = cv2.cvtColor(self.frame.image, cv2.COLOR_RGB2BGR)
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
