#!/usr/bin/env python3

import time
import threading
from pathlib import Path
from flask import Flask, Response, make_response
import cv2
import yaml

from picamera2 import Picamera2


def load_settings(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


settings_path = Path(__file__).parent.parent / "settings.yaml"
if not settings_path.exists():
    raise FileNotFoundError(f"Settings file not found at {settings_path}")
settings = load_settings(settings_path)
cam_cfg = settings["camera"]
width = cam_cfg["width"]
height = cam_cfg["height"]
fps = cam_cfg["framerate"]
hflip = cam_cfg["hflip"]
vflip = cam_cfg["vflip"]
port = cam_cfg["stream_port"]


class Camera:
    def __init__(self, width: int, height: int, fps: int, hflip: bool, vflip: bool):
        self.width = width
        self.height = height
        self.fps = fps
        self.hflip = hflip
        self.vflip = vflip
        self.lock = threading.Lock()
        self.frame = None
        self.running = True

        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            {
                "size": (width, height),
            }
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

            ok, jpg = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100]
            )  # Force quality
            if not ok:
                return None
            return jpg.tobytes()

    def shutdown(self):
        self.running = False
        self._thread.join(timeout=1.0)
        self.cam.stop()


app = Flask(__name__)
camera = Camera(width=width, height=height, fps=fps, hflip=hflip, vflip=vflip)


@app.route("/video_feed")
def video_feed():
    def gen():
        while True:
            jpg = camera.get_jpeg()
            if jpg is None:
                time.sleep(0.02)
                continue
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot.jpg")
def snapshot():
    jpg = camera.get_jpeg()
    if jpg is None:
        return ("No frame", 503)
    resp = make_response(jpg)
    resp.headers["Content-Type"] = "image/jpeg"
    return resp


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=port, threaded=True)
