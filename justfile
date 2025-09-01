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
    echo "Checking Python files..."
    black --check .
    mypy .
    echo "Checking cpp files..."
    clang-format --dry-run --Werror $(find src include -name '*.hpp' -o -name '*.cpp')
    clang-tidy $(find src -name '*.cpp') -- -Iinclude
    echo "Done."

format:
    echo "Formatting Python with black..."
    black .
    echo "Formatting Python imports with isort..."
    isort .
    echo "Formatting C++ with clang-format..."
    clang-format -i $(find src include -name '*.hpp' -o -name '*.cpp')
    echo "Applying clang-tidy auto-fixes (if safe)..."
    clang-tidy -fix -fix-errors $(find src -name '*.cpp') -- -Iinclude
    echo "Done."