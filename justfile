arduino-config:
    arduino-cli config init
    arduino-cli core update-index
    arduino-cli core install arduino:avr

check:
    black --check .
    ruff .
    mypy .

format:
    echo "Formatting Python with black..."
    black .
    echo "Formatting Python imports with isort..."
    isort .