
import yaml
import time
from pathlib import Path
from gpiozero import Motor

WORKSPACE_DIR=Path(__file__).parent.parent 
SETTINGS_PATH= WORKSPACE_DIR / "settings.yaml"

if __name__ =="__main__":
    with open(SETTINGS_PATH, "r") as settings_file:
        settings_dict = yaml.load_all(settings_file)
    pinout_dict = settings_dict["pinout"]
    motor_left = Motor(pinout_dict["MOTOR_LEFT_PIN_1"], pinout_dict["MOTOR_LEFT_PIN_2"])
    motor_right = Motor(pinout_dict["MOTOR_RIGHT_PIN_1"], pinout_dict["MOTOR_RIGHT_PIN_2"])

    for i in range(3):
        motor_left.forward(1)
        motor_right.forward(0)
        time.sleep(2)
        motor_left.forward(0)
        motor_right.forward(1)
        time.sleep(2)