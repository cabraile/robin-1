check:
    echo "Checking Python files..."
    black --check .
    mypy .
    echo "Done."

format:
    echo "Formatting Python with black..."
    black .
    echo "Formatting Python imports with isort..."
    isort .
    ruff format .
    echo "Done."