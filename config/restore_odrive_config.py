#!/usr/bin/env python3
"""
ODrive Configuration Restore Script
Restores configuration after firmware migration
CRITICAL: Sets encoder to ID 4 (onboard) instead of 13 (sensorless)
"""

import odrive
from odrive.enums import *
import time
import sys

def restore_config():
    print("Connecting to ODrive...")
    odrv = odrive.find_any()

    print(f"Connected to ODrive S1: {odrv.serial_number}")
    print(f"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"DC Bus Voltage: {odrv.vbus_voltage:.2f}V")

    # ========================================================================
    # POWER CONFIGURATION
    # ========================================================================
    print("\n[1/6] Configuring power limits...")
    odrv.config.dc_bus_overvoltage_trip_level = 30.0
    odrv.config.dc_bus_undervoltage_trip_level = 20.4
    odrv.config.dc_max_positive_current = 100.0
    odrv.config.dc_max_negative_current = -30.0
    print("✓ Power limits set")

    # ========================================================================
    # MOTOR CONFIGURATION - Eaglepower 8308-90KV
    # ========================================================================
    print("\n[2/6] Configuring motor...")
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.pole_pairs = 20
    odrv.axis0.config.motor.torque_constant = 0.09188888888888888
    odrv.axis0.config.motor.current_soft_max = 10.0
    odrv.axis0.config.motor.current_hard_max = 23.0
    odrv.axis0.config.motor.calibration_current = 10.0
    odrv.axis0.config.motor.resistance_calib_max_voltage = 4.0
    print("✓ Motor configuration set")

    # ========================================================================
    # ENCODER CONFIGURATION - CRITICAL FIX!
    # ========================================================================
    print("\n[3/6] Configuring encoder...")
    print("  ⚠️  CRITICAL: Setting encoder to ID 4 (onboard), not 13 (sensorless)")

    # CORRECT encoder setting for ODrive S1 onboard magnetic encoder
    odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0  # 4
    odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0  # 4

    print("  ✓ Encoder set to ONBOARD_ENCODER0 (ID 4)")

    # ========================================================================
    # CONTROLLER CONFIGURATION
    # ========================================================================
    print("\n[4/6] Configuring controller...")
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    odrv.axis0.controller.config.pos_gain = 20.0
    odrv.axis0.controller.config.vel_gain = 0.1666666716337204
    odrv.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv.axis0.controller.config.vel_limit = 10.0
    odrv.axis0.controller.config.vel_limit_tolerance = 1.2
    odrv.axis0.config.torque_soft_min = -1.5
    odrv.axis0.config.torque_soft_max = 1.5
    print("✓ Controller configuration set")

    # ========================================================================
    # CAN CONFIGURATION
    # ========================================================================
    print("\n[5/6] Configuring CAN bus...")
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 250000
    odrv.axis0.config.can.node_id = 1
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 10
    odrv.axis0.config.can.iq_msg_rate_ms = 10
    odrv.axis0.config.can.torques_msg_rate_ms = 10
    odrv.axis0.config.can.error_msg_rate_ms = 10
    odrv.axis0.config.can.temperature_msg_rate_ms = 10
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 10
    print("✓ CAN configuration set")

    # ========================================================================
    # MISCELLANEOUS
    # ========================================================================
    print("\n[6/6] Setting miscellaneous options...")
    odrv.axis0.config.enable_watchdog = False
    odrv.config.enable_uart_a = False
    print("✓ Miscellaneous options set")

    # ========================================================================
    # SAVE CONFIGURATION
    # ========================================================================
    print("\n" + "="*80)
    print("SAVING CONFIGURATION")
    print("="*80)
    print("ODrive will reboot...")

    odrv.save_configuration()
    time.sleep(4)

    # ========================================================================
    # VERIFY CONFIGURATION
    # ========================================================================
    print("\nReconnecting...")
    odrv = odrive.find_any(timeout=10)

    print("\n" + "="*80)
    print("CONFIGURATION VERIFICATION")
    print("="*80)

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
    verify("Torque constant", odrv.axis0.config.motor.torque_constant, 0.091889, tolerance=0.0001)
    verify("Commutation encoder", odrv.axis0.config.commutation_encoder, 4)  # CRITICAL!
    verify("Load encoder", odrv.axis0.config.load_encoder, 4)  # CRITICAL!
    verify("CAN node ID", odrv.axis0.config.can.node_id, 1)
    verify("CAN baud rate", odrv.can.config.baud_rate, 250000)

    print(f"\n{'='*80}")
    print(f"RESULT: {checks_passed}/{checks_total} checks passed")
    print(f"{'='*80}")

    if checks_passed == checks_total:
        print("\n✓ ALL CHECKS PASSED!")
        print("\nConfiguration restored successfully with CORRECTED encoder settings.")
        print("\n📋 NEXT STEP: Run calibration")
        print("   python3 calibrate_odrive_s1.py")
        return 0
    else:
        print(f"\n⚠️ {checks_total - checks_passed} check(s) failed")
        print("Please review configuration manually.")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(restore_config())
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
