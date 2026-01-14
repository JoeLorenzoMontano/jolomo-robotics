#!/bin/bash
set -e

echo "========================================"
echo "Jolomo Robotics System Initialization"
echo "========================================"
echo

# Check if devices exist
if [ ! -e /dev/ttyACM0 ] && [ ! -e /dev/arduino_r4 ]; then
    echo "ERROR: Arduino not found"
    echo "  Expected at: /dev/ttyACM0 or /dev/arduino_r4"
    echo "  Check USB connection"
    exit 1
fi

if [ ! -e /dev/ttyACM1 ] && [ ! -e /dev/odrive_s1 ]; then
    echo "ERROR: ODrive not found"
    echo "  Expected at: /dev/ttyACM1 or /dev/odrive_s1"
    echo "  Check USB connection and power supply"
    exit 1
fi

echo "✓ Devices detected"
echo

# Check ODrive state and set to CLOSED_LOOP if needed
echo "Checking ODrive status..."
python3 << 'PYTHON'
import odrive
from odrive.enums import *
import time
import sys

try:
    odrv = odrive.find_any(timeout=10)
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  Bus Voltage: {odrv.vbus_voltage:.2f}V")
    print(f"  Current State: {odrv.axis0.current_state} (1=IDLE, 8=CLOSED_LOOP)")

    if odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
        print("\n  Setting to CLOSED_LOOP_CONTROL...")

        # Clear errors using v0.6.11 API
        try:
            odrv.clear_errors()
        except AttributeError:
            pass

        time.sleep(0.5)
        odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(2)

        if odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
            print("  ✓ Successfully entered CLOSED_LOOP_CONTROL")
        else:
            print(f"  ✗ Failed to enter CLOSED_LOOP_CONTROL")
            print(f"    State: {odrv.axis0.current_state}")
            print(f"    Disarm reason: {odrv.axis0.disarm_reason}")
            print("\n  SOLUTION: Run calibration")
            print("    python3 calibrate_odrive_s1.py")
            sys.exit(1)
    else:
        print("  ✓ Already in CLOSED_LOOP_CONTROL")

except Exception as e:
    print(f"  ✗ Error connecting to ODrive: {e}")
    sys.exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "Failed to initialize ODrive"
    exit 1
fi

echo
echo "✓ ODrive ready"
echo

# Restart web interface Docker container
echo "Restarting web interface..."
cd /home/jolomoadmin/Internal/development/jolomo-robotics/web_control
docker-compose restart motor-control > /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "✓ Web interface restarted"
else
    echo "⚠ Warning: Failed to restart Docker container"
    echo "  Try manually: cd web_control && docker-compose restart motor-control"
fi

echo
echo "========================================"
echo "✓ System Ready"
echo "========================================"
echo "Web interface: http://localhost:5003"
echo
