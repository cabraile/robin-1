arduino-config:
    arduino-cli config init
    arduino-cli core update-index
    arduino-cli core install arduino:avr

arduino-compile:
    arduino-cli compile -b arduino:avr:nano /home/dev/workspace/src/robin_firmware/robin_firmware_cpp

arduino-upload:
    arduino-cli upload -p /dev/ttyUSB0 -b arduino:avr:nano /home/dev/workspace/src/robin_firmware/robin_firmware_cpp

arduino-build:
    arduino-cli compile -b arduino:avr:nano /home/dev/workspace/src/robin_firmware/robin_firmware_cpp
    arduino-cli upload -p /dev/ttyUSB0 -b arduino:avr:nano /home/dev/workspace/src/robin_firmware/robin_firmware_cpp

check:
    black --check .
    ruff .
    mypy .

format:
    echo "Formatting Python with black..."
    black .
    echo "Formatting Python imports with isort..."
    isort .