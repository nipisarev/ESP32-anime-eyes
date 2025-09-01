#!/usr/bin/env python3
import subprocess
import os
import sys

# Set ESP-IDF path
os.environ['IDF_PATH'] = '/Users/npisarev/Projects/ESP32/ESP-IDF/v5.5/esp-idf'

# Change to project directory
os.chdir('/Users/npisarev/Projects/ESP32/ESP32-anime-eyes')

try:
    # Run IDF build
    result = subprocess.run([
        'bash', '-c', 
        'source /Users/npisarev/Projects/ESP32/ESP-IDF/v5.5/esp-idf/export.sh && idf.py build'
    ], capture_output=True, text=True, timeout=60)
    
    print("Return code:", result.returncode)
    print("STDOUT:")
    print(result.stdout)
    print("STDERR:")
    print(result.stderr)
    
except subprocess.TimeoutExpired:
    print("Build timed out")
except Exception as e:
    print(f"Error: {e}")
