#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Path to the virtual environment
VENV_PATH="/home/caleb/ros2_ws/.venv"

# Activate the virtual environment
source "$VENV_PATH/bin/activate"

# Run the Python script with the virtual environment's Python
exec "$VENV_PATH/bin/python3" "$SCRIPT_DIR/franky_relay.py" "$@" 