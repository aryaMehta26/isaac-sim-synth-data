#!/bin/bash
# Helper script to run Isaac Sim python scripts appropriately
# This assumes the Isaac Sim python environment is set up or aliased to specific paths
# Adjust ISAAC_SIM_PYTHON_PATH to point to your local Isaac Sim python executable

ISAAC_SIM_PYTHON_PATH="/home/isaac-sim/_build/linux-x86_64/release/python.sh"

if [ -f "$ISAAC_SIM_PYTHON_PATH" ]; then
    bash "$ISAAC_SIM_PYTHON_PATH" "$@"
else
    echo "Isaac Sim python.sh not found at $ISAAC_SIM_PYTHON_PATH"
    echo "Assuming 'python3' is configured with Isaac Sim bindings..."
    python3 "$@"
fi
