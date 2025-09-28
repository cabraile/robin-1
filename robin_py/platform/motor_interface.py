import logging
from enum import Enum
from pathlib import Path

import yaml
from gpiozero import Motor


class MotorId(Enum):
    LEFT: int = 0
    RIGHT: int = 1


class MotorInterface:
    def __init__(self, settings_path: Path, logger: logging.Logger | None = None):
        with open(settings_path, "r") as settings_file:
            settings_dict = yaml.load(settings_file, Loader=yaml.FullLoader)
        pinout_dict = settings_dict["pinout"]
        self.motors: list[Motor] = [None, None]
        self.motors[MotorId.LEFT.value] = Motor(
            pinout_dict["MOTOR_LEFT_PIN_1"], pinout_dict["MOTOR_LEFT_PIN_2"]
        )
        self.motors[MotorId.RIGHT.value] = Motor(
            pinout_dict["MOTOR_RIGHT_PIN_1"], pinout_dict["MOTOR_RIGHT_PIN_2"]
        )
        self.logger = logger

    def activate_by_id(self, side: MotorId, intensity: float) -> None:
        if intensity >= 0:
            self.motors[side.value].forward(intensity)
        else:
            self.motors[side.value].backward(abs(intensity))
        if self.logger:
            self.logger.debug(
                f"Motor {MotorId(side).name} set to intensity {intensity}"
            )

    def activate(self, intensity_left: float, intensity_right: float):
        self.activate_by_id(MotorId.LEFT, intensity_left)
        self.activate_by_id(MotorId.RIGHT, intensity_right)
