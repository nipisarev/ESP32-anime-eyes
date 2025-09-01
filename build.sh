#!/bin/bash

# ESP-IDF Build Script for Anime Eyes Project
# This script sets up the ESP-IDF environment and builds the project

set -e

# ESP-IDF Path - adjust this to match your installation
export IDF_PATH="/Users/npisarev/Projects/ESP32/ESP-IDF/v5.5/esp-idf"

# Check if ESP-IDF exists
if [ ! -d "$IDF_PATH" ]; then
    echo "Error: ESP-IDF not found at $IDF_PATH"
    echo "Please install ESP-IDF or update the IDF_PATH in this script"
    exit 1
fi

echo "Setting up ESP-IDF environment..."

# Source ESP-IDF export script
. $IDF_PATH/export.sh

# Set target to ESP32-C6
echo "Setting target to ESP32-C6..."
idf.py set-target esp32c6

# Build the project
echo "Building project..."
idf.py build

echo "Build complete!"

echo "Flashing project to ESP32-C6..."
idf.py flash

echo "Flash complete!"

idf.py monitor