#!/usr/bin/env python3
"""
Automatic ODrive S1 Motor Calibration (Non-Interactive)
Runs calibration for Motor 1 (Node ID 2) without user prompts.
"""

import odrive
from odrive.enums import *
import time
import sys

def calibrate():
    print("=" * 80)
    print("Motor 1 Automatic Calibration (Node ID 2)")
    print("=" * 80)

    print("\n[1/5] Connecting to ODrive...")
    odrv = odrive.find_any(timeout=10)

    print(f"✓ Connected: Serial {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  DC Bus: {odrv.vbus_voltage:.2f}V")
    print(f"  CAN Node ID: {odrv.axis0.config.can.node_id}")

    if odrv.axis0.config.can.node_id != 2:
        print(f"\n⚠ WARNING: Expected Node ID 2, found {odrv.axis0.config.can.node_id}")
        print("  This might not be Motor 1. Continuing anyway...")

    print("\n[2/5] Setting motor to IDLE...")
    odrv.axis0.requested_state = AxisState.IDLE
    time.sleep(0.5)

    print("\n[3/5] Running MOTOR_CALIBRATION...")
    print("  ⚠ Motor will beep and move slightly")
    print("  ⚠ Ensure motor can spin freely!")
    time.sleep(1)

    odrv.axis0.requested_state = AxisState.MOTOR_CALIBRATION

    # Wait for calibration to complete
    timeout = 15
    start_time = time.time()
    while odrv.axis0.current_state != AxisState.IDLE:
        if time.time() - start_time > timeout:
            print("\n✗ Motor calibration timed out!")
            print(f"  Current state: {odrv.axis0.current_state}")
            print(f"  Active errors: {hex(odrv.axis0.active_errors)}")
            return 1
        time.sleep(0.1)

    # Check for errors
    if odrv.axis0.active_errors != 0:
        print(f"\n✗ Motor calibration failed!")
        print(f"  Active errors: {hex(odrv.axis0.active_errors)}")
        print(f"  Disarm reason: {odrv.axis0.disarm_reason}")
        return 1

    print("✓ Motor calibration complete")

    # Check if phase resistance/inductance were measured
    try:
        r = odrv.axis0.config.motor.phase_resistance
        l = odrv.axis0.config.motor.phase_inductance
        if r and r > 0:
            print(f"  Phase resistance: {r:.6f} Ω")
        if l and l > 0:
            print(f"  Phase inductance: {l:.6e} H")
    except:
        pass

    print("\n[4/5] Running ENCODER_OFFSET_CALIBRATION...")
    print("  ⚠ Motor will spin slowly in both directions")
    time.sleep(1)

    odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

    # Wait for encoder calibration to complete
    timeout = 30
    start_time = time.time()
    while odrv.axis0.current_state != AxisState.IDLE:
        if time.time() - start_time > timeout:
            print("\n✗ Encoder calibration timed out!")
            print(f"  Current state: {odrv.axis0.current_state}")
            print(f"  Active errors: {hex(odrv.axis0.active_errors)}")
            return 1
        time.sleep(0.1)

    # Check for errors
    if odrv.axis0.active_errors != 0:
        print(f"\n✗ Encoder calibration failed!")
        print(f"  Active errors: {hex(odrv.axis0.active_errors)}")
        print(f"  Disarm reason: {odrv.axis0.disarm_reason}")
        return 1

    print("✓ Encoder offset calibration complete")

    print("\n[5/5] Testing CLOSED_LOOP_CONTROL...")
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(2)

    if odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
        print(f"\n✗ Failed to enter closed loop!")
        print(f"  Current state: {odrv.axis0.current_state}")
        print(f"  Active errors: {hex(odrv.axis0.active_errors)}")
        return 1

    print("✓ Motor entered closed loop control")

    # Quick velocity test
    print("\n  Testing velocity command (0.5 turns/sec for 2 seconds)...")
    odrv.axis0.controller.input_vel = 0.5
    time.sleep(2)
    odrv.axis0.controller.input_vel = 0.0
    time.sleep(0.5)

    # Return to IDLE
    odrv.axis0.requested_state = AxisState.IDLE
    time.sleep(0.5)

    print("\n" + "=" * 80)
    print("✓ CALIBRATION SUCCESSFUL!")
    print("=" * 80)
    print("\nMotor 1 is now fully configured and calibrated.")
    print("You can now:")
    print("  1. Disconnect USB from this ODrive")
    print("  2. Ensure CAN bus is connected (CAN_H and CAN_L)")
    print("  3. Enable Motor 1 from the web interface")
    print("  4. Both motors should now work together")

    return 0

if __name__ == '__main__':
    try:
        sys.exit(calibrate())
    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
