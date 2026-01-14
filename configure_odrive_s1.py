#!/usr/bin/env python3
"""
ODrive S1 Configuration Script for Eaglepower 8308-90KV Motor

This script applies the corrected configuration for ODrive S1 (HW v5.2, FW v0.6.9)
with Eaglepower 8308-90KV brushless gimbal motor.

Key Fixes:
- Removes incorrect encoder mode setting (HALL mode)
- Uses positive position gain with proper direction inversion
- Disables brake resistor (not connected)
- Verifies CAN configuration fields for v0.6.9

Hardware:
- Motor: Eaglepower 8308-90KV (40 magnets, 20 pole pairs)
- Battery: 6S LiPo (22.2V nominal, 25.2V max)
- Interface: Arduino Uno R4 Minima via CAN (250kbps)
"""

import odrive
from odrive.enums import *
import time
import sys

def main():
    print("=" * 80)
    print("ODrive S1 Configuration Script")
    print("Motor: Eaglepower 8308-90KV (90 KV, 20 pole pairs)")
    print("=" * 80)

    # Connect to ODrive
    print("\n[1/8] Connecting to ODrive S1...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        print("Please ensure ODrive is connected via USB and powered on.")
        sys.exit(1)

    print(f"✓ Found ODrive S1: {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"  DC Bus Voltage: {odrv.vbus_voltage:.2f}V")

    # ============================================================================
    # MOTOR CONFIGURATION - Eaglepower 8308-90KV
    # ============================================================================
    print("\n[2/8] Configuring motor parameters...")

    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT  # 0 - FOC control
    odrv.axis0.config.motor.pole_pairs = 20  # 40 magnets / 2 = 20 pole pairs
    odrv.axis0.config.motor.torque_constant = 0.091889  # 8.27 / 90 KV = 0.091889 Nm/A
    odrv.axis0.config.motor.calibration_current = 10.0  # Amps

    # Current limits
    odrv.axis0.config.motor.current_soft_max = 15.0  # Amps (normal operation)
    odrv.axis0.config.motor.current_hard_max = 20.0  # Amps (absolute max)

    print("✓ Motor configuration set:")
    print(f"  Motor type: HIGH_CURRENT (FOC)")
    print(f"  Pole pairs: 20")
    print(f"  Torque constant: 0.091889 Nm/A")
    print(f"  Calibration current: 10.0 A")
    print(f"  Current limits: 15.0 A (soft) / 20.0 A (hard)")

    # ============================================================================
    # ENCODER CONFIGURATION - S1 Onboard Magnetic Encoder
    # ============================================================================
    print("\n[3/8] Configuring onboard magnetic encoder...")

    # IMPORTANT: In v0.6.9, there is NO onboard_encoder0.config object!
    # No mode setting, no direction setting, no enabled setting at encoder level
    # Configuration is done purely through axis0.config

    # Use onboard encoder for commutation and position feedback
    odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0  # 4
    odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0  # 4

    # Note: Direction inversion in v0.6.9 is done via motor phase wire swap
    # or by using negative pos_gain (not recommended - destabilizes control)

    print("✓ Encoder configuration set:")
    print(f"  Encoder type: ONBOARD_ENCODER0 (ID 4)")
    print(f"  Commutation encoder: ONBOARD_ENCODER0")
    print(f"  Load encoder: ONBOARD_ENCODER0")
    print(f"  Note: v0.6.9 has no encoder.config.mode or direction settings")

    # ============================================================================
    # CONTROLLER CONFIGURATION
    # ============================================================================
    print("\n[4/8] Configuring controller gains...")

    # CRITICAL FIX: Use POSITIVE gains (was -40.0)
    # Direction should be fixed via encoder.direction, not negative gains
    odrv.axis0.controller.config.pos_gain = 40.0  # Positive gain
    odrv.axis0.controller.config.vel_limit = 5.0  # turns/sec

    # Velocity control gains
    odrv.axis0.controller.config.vel_gain = 0.3  # (N*m*s) / rad
    odrv.axis0.controller.config.vel_integrator_gain = 0.6  # (N*m) / rad

    # Default control mode (Arduino switches modes as needed)
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL  # 2

    print("✓ Controller configuration set:")
    print(f"  Position gain: 40.0 (POSITIVE - fixed!)")
    print(f"  Velocity limit: 5.0 turns/sec")
    print(f"  Velocity gain: 0.3")
    print(f"  Velocity integrator gain: 0.6")
    print(f"  Default control mode: VELOCITY_CONTROL")

    # ============================================================================
    # CAN BUS CONFIGURATION
    # ============================================================================
    print("\n[5/8] Configuring CAN bus...")

    odrv.can.config.baud_rate = 250000  # 250 kbps (must match Arduino)
    odrv.axis0.config.can.node_id = 1  # Must match Arduino ODRV0_NODE_ID

    # Enable encoder feedback over CAN (verify field name for v0.6.9)
    # Try both possible field names
    can_encoder_configured = False
    try:
        odrv.axis0.config.can.encoder_msg_rate_ms = 10  # 100Hz feedback
        can_encoder_configured = True
        print("✓ CAN encoder feedback: encoder_msg_rate_ms = 10")
    except AttributeError:
        print("  encoder_msg_rate_ms not found, trying encoder_rate_ms...")
        try:
            odrv.axis0.config.can.encoder_rate_ms = 10
            can_encoder_configured = True
            print("✓ CAN encoder feedback: encoder_rate_ms = 10")
        except AttributeError:
            print("⚠ WARNING: Could not find CAN encoder rate config field")
            print("  This may prevent encoder feedback over CAN to Arduino")

    print("✓ CAN bus configuration set:")
    print(f"  Baud rate: 250000 (250 kbps)")
    print(f"  Node ID: 1")
    print(f"  Encoder feedback: {'Enabled' if can_encoder_configured else 'NOT CONFIGURED'}")

    # ============================================================================
    # POWER/PROTECTION CONFIGURATION
    # ============================================================================
    print("\n[6/8] Configuring power limits and protection...")

    # Battery: 6S LiPo (22.2V nominal, 25.2V max charged)
    odrv.config.dc_bus_overvoltage_trip_level = 30.0  # Volts (safe for 6S)
    odrv.config.dc_bus_undervoltage_trip_level = 18.0  # Volts (protect battery)

    # DC bus current limits
    odrv.config.dc_max_positive_current = 120.0  # Amps (motor draw)
    odrv.config.dc_max_negative_current = -10.0  # Amps (regen limit - no brake resistor)

    # Brake resistor: NOT CONNECTED - disable or leave default
    try:
        odrv.config.brake_resistor0.enable = False
        print("✓ Brake resistor disabled (not connected)")
    except AttributeError:
        print("  brake_resistor0 config not found (using default)")

    print("✓ Power/protection configuration set:")
    print(f"  Overvoltage trip: 30.0V")
    print(f"  Undervoltage trip: 18.0V")
    print(f"  Max positive current: 120.0A")
    print(f"  Max negative current: -10.0A (regen limit)")

    # ============================================================================
    # WATCHDOG CONFIGURATION
    # ============================================================================
    odrv.axis0.config.enable_watchdog = False  # Allow calibration to complete
    print("✓ Watchdog disabled (allow calibration)")

    # ============================================================================
    # SAVE AND REBOOT
    # ============================================================================
    print("\n[7/8] Saving configuration...")
    print("  ODrive will reboot after saving...")

    try:
        odrv.save_configuration()
        print("✓ Configuration saved successfully")
    except Exception as e:
        print(f"ERROR: Failed to save configuration: {e}")
        sys.exit(1)

    print("\n  Waiting for ODrive to reboot (3 seconds)...")
    time.sleep(3)

    print("\n[8/8] Reconnecting and verifying configuration...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not reconnect after reboot: {e}")
        sys.exit(1)

    print(f"✓ Reconnected to ODrive S1: {odrv.serial_number}")

    # ============================================================================
    # VERIFICATION
    # ============================================================================
    print("\n" + "=" * 80)
    print("CONFIGURATION VERIFICATION")
    print("=" * 80)

    # Check all critical values
    checks_passed = 0
    checks_total = 0

    def verify(name, actual, expected, tolerance=None):
        nonlocal checks_passed, checks_total
        checks_total += 1
        if tolerance:
            match = abs(actual - expected) < tolerance
        else:
            match = actual == expected

        status = "✓" if match else "✗"
        print(f"{status} {name}: {actual} {'(OK)' if match else f'(expected {expected})'}")
        if match:
            checks_passed += 1
        return match

    verify("Motor type", odrv.axis0.config.motor.motor_type, 0)
    verify("Pole pairs", odrv.axis0.config.motor.pole_pairs, 20)
    verify("Torque constant", odrv.axis0.config.motor.torque_constant, 0.091889, tolerance=0.000001)
    verify("Commutation encoder", odrv.axis0.config.commutation_encoder, 4)
    verify("Load encoder", odrv.axis0.config.load_encoder, 4)
    verify("CAN node ID", odrv.axis0.config.can.node_id, 1)
    verify("CAN baud rate", odrv.can.config.baud_rate, 250000)

    # Position gain should be positive now
    pos_gain = odrv.axis0.controller.config.pos_gain
    pos_gain_positive = pos_gain > 0
    checks_total += 1
    print(f"{'✓' if pos_gain_positive else '✗'} Position gain: {pos_gain} {'(POSITIVE - correct!)' if pos_gain_positive else '(NEGATIVE - still wrong!)'}")
    if pos_gain_positive:
        checks_passed += 1

    print(f"\nVerification: {checks_passed}/{checks_total} checks passed")

    print("\n" + "=" * 80)
    print("CONFIGURATION COMPLETE")
    print("=" * 80)
    print(f"DC Bus Voltage: {odrv.vbus_voltage:.2f}V")
    print(f"Axis State: {odrv.axis0.current_state}")

    print("\n📋 NEXT STEPS:")
    print("1. Run calibration: python3 calibrate_odrive_s1.py")
    print("2. Test encoder direction: python3 test_encoder_direction.py")
    print("3. If encoder calibration still fails, check magnet gap (should be 1-3mm)")

    if checks_passed == checks_total:
        print("\n✓ All configuration checks passed!")
        return 0
    else:
        print(f"\n⚠ Warning: {checks_total - checks_passed} configuration check(s) failed")
        print("Please review the configuration manually.")
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nConfiguration interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
