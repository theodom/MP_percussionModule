#!/bin/bash
# Usage: ./arduino_upload.sh <path/to/sketch.ino> <board FQBN>
# Example: ./arduino_upload.sh firmware/firmware.ino arduino:avr:nano

SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
FQBN="arduino:esp32:nano_nora"
PORT="/dev/ttyACM0"

arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
arduino-cli upload  --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"