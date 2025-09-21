import time
import yaml
import cv2
import threading
from picamera2 import Picamera2
from pathlib import Path

class CameraInterface:
    def __init__(self, settings_path: Path):
        with open(settings_path, "r") as f:
            settings = yaml.safe_load(f)
        cam_cfg = settings["camera"]
        self.width = cam_cfg["width"]
        self.height = cam_cfg["height"]
        self.fps = cam_cfg["framerate"]
        self.hflip = cam_cfg["hflip"]
        self.vflip = cam_cfg["vflip"]
        self.lock = threading.Lock()
        self.frame = None
        self.running = True

        self.cam = Picamera2()
        config = self.cam.create_preview_configuration({"size": (self.width, self.height)})
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