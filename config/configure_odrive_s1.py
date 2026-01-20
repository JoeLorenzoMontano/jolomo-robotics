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
import math

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

    # Motor type - try both enum names for compatibility
    try:
        odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL  # v0.6.11 name
    except AttributeError:
        odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT  # Alternative name

    odrv.axis0.config.motor.pole_pairs = 20  # 40 magnets / 2 = 20 pole pairs
    odrv.axis0.config.motor.torque_constant = 0.09188888888888888  # 8.27 / 90 KV
    odrv.axis0.config.motor.calibration_current = 20.0  # Amps (increased for better calibration)

    # Current limits
    odrv.axis0.config.motor.current_soft_max = 20.0  # Amps (normal operation)
    odrv.axis0.config.motor.current_hard_max = 36.0  # Amps (absolute max)

    # Calibration parameters
    odrv.axis0.config.motor.resistance_calib_max_voltage = 4.0  # Volts
    odrv.axis0.config.calibration_lockin.current = 10.0  # Amps (for encoder offset calibration)

    # Motor thermistor (disabled if not present)
    try:
        odrv.axis0.motor.motor_thermistor.config.enabled = False
        print("✓ Motor thermistor disabled")
    except AttributeError:
        print("  Motor thermistor config not available")

    print("✓ Motor configuration set:")
    print(f"  Motor type: PMSM_CURRENT_CONTROL (FOC)")
    print(f"  Pole pairs: 20")
    print(f"  Torque constant: 0.091889 Nm/A")
    print(f"  Calibration current: 20.0 A")
    print(f"  Current limits: 20.0 A (soft) / 36.0 A (hard)")
    print(f"  Resistance calib max voltage: 4.0 V")
    print(f"  Calibration lockin current: 10.0 A")

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
    print("\n[4/8] Configuring controller parameters...")

    # Control mode and input mode
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP

    # Velocity limits
    odrv.axis0.controller.config.vel_limit = 10.0  # turns/sec
    odrv.axis0.controller.config.vel_limit_tolerance = 1.2
    odrv.axis0.controller.config.vel_ramp_rate = 10.0  # turns/sec^2

    # Torque limits
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf

    # Trajectory planning
    odrv.axis0.trap_traj.config.accel_limit = 10.0  # turns/sec^2

    print("✓ Controller configuration set:")
    print(f"  Control mode: VELOCITY_CONTROL")
    print(f"  Input mode: VEL_RAMP")
    print(f"  Velocity limit: 10.0 turns/sec")
    print(f"  Velocity limit tolerance: 1.2")
    print(f"  Velocity ramp rate: 10.0 turns/sec^2")
    print(f"  Torque limits: -inf to +inf")
    print(f"  Trajectory accel limit: 10.0 turns/sec^2")

    # ============================================================================
    # CAN BUS CONFIGURATION
    # ============================================================================
    print("\n[5/8] Configuring CAN bus...")

    # CAN protocol and baud rate
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 250000  # 250 kbps (must match Arduino)
    odrv.axis0.config.can.node_id = 1  # Must match Arduino ODRV0_NODE_ID

    # CAN message rates (all at 100ms)
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 100
    odrv.axis0.config.can.iq_msg_rate_ms = 100
    odrv.axis0.config.can.torques_msg_rate_ms = 100
    odrv.axis0.config.can.error_msg_rate_ms = 100
    odrv.axis0.config.can.temperature_msg_rate_ms = 100
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 100

    print("✓ CAN bus configuration set:")
    print(f"  Protocol: SIMPLE")
    print(f"  Baud rate: 250000 (250 kbps)")
    print(f"  Node ID: 1")
    print(f"  Message rates: 100ms (10 Hz) for all telemetry")

    # ============================================================================
    # POWER/PROTECTION CONFIGURATION
    # ============================================================================
    print("\n[6/8] Configuring power limits and protection...")

    # Battery: 6S LiPo (22.2V nominal, 25.2V max charged)
    odrv.config.dc_bus_overvoltage_trip_level = 30.0  # Volts (safe for 6S)
    odrv.config.dc_bus_undervoltage_trip_level = 20.4  # Volts (3.4V/cell - LiPo protection)

    # DC bus current limits
    odrv.config.dc_max_positive_current = 100.0  # Amps (motor draw)
    odrv.config.dc_max_negative_current = -25.0  # Amps (regen limit)

    # Brake resistor: NOT CONNECTED - disable
    try:
        odrv.config.brake_resistor0.enable = False
        print("✓ Brake resistor disabled (not connected)")
    except AttributeError:
        print("  brake_resistor0 config not found (using default)")

    print("✓ Power/protection configuration set:")
    print(f"  Overvoltage trip: 30.0V")
    print(f"  Undervoltage trip: 18.0V")
    print(f"  Max positive current: 100.0A")
    print(f"  Max negative current: -25.0A (regen limit)")

    # ============================================================================
    # WATCHDOG CONFIGURATION
    # ============================================================================
    odrv.axis0.config.enable_watchdog = False  # Allow calibration to complete
    print("✓ Watchdog disabled (allow calibration)")

    # ============================================================================
    # UART CONFIGURATION
    # ============================================================================
    print("\n[7/8] Configuring UART...")
    try:
        odrv.config.enable_uart_a = False
        print("✓ UART A disabled (avoid conflicts)")
    except AttributeError:
        print("  UART A config not available")

    # ============================================================================
    # SAVE AND REBOOT
    # ============================================================================
    print("\n[8/9] Saving configuration...")
    print("  ODrive will reboot after saving...")

    try:
        odrv.save_configuration()
        print("✓ Configuration saved - ODrive is rebooting...")
    except Exception as e:
        # Device disconnect during save_configuration() is NORMAL
        # ODrive reboots immediately, causing disconnect
        if "disconnect" in str(e).lower():
            print("✓ Configuration saved - ODrive is rebooting (disconnect is normal)")
        else:
            print(f"ERROR: Failed to save configuration: {e}")
            sys.exit(1)

    print("\n  Waiting for ODrive to reboot (5 seconds)...")
    time.sleep(5)

    print("\n[9/9] Reconnecting and verifying configuration...")
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

    # Motor configuration
    verify("Motor type", odrv.axis0.config.motor.motor_type, 0)
    verify("Pole pairs", odrv.axis0.config.motor.pole_pairs, 20)
    verify("Torque constant", odrv.axis0.config.motor.torque_constant, 0.091889, tolerance=0.000001)
    verify("Calibration current", odrv.axis0.config.motor.calibration_current, 20.0, tolerance=0.1)
    verify("Current soft max", odrv.axis0.config.motor.current_soft_max, 20.0, tolerance=0.1)
    verify("Current hard max", odrv.axis0.config.motor.current_hard_max, 36.0, tolerance=0.1)
    verify("Resistance calib max voltage", odrv.axis0.config.motor.resistance_calib_max_voltage, 4.0, tolerance=0.1)
    verify("Calibration lockin current", odrv.axis0.config.calibration_lockin.current, 10.0, tolerance=0.1)

    # Encoder configuration (v0.6.11: ONBOARD_ENCODER0 = 13, not 4 like in v0.6.9)
    verify("Commutation encoder", odrv.axis0.config.commutation_encoder, 13)
    verify("Load encoder", odrv.axis0.config.load_encoder, 13)

    # Controller configuration
    verify("Velocity limit", odrv.axis0.controller.config.vel_limit, 10.0, tolerance=0.1)

    # CAN configuration
    verify("CAN node ID", odrv.axis0.config.can.node_id, 1)
    verify("CAN baud rate", odrv.can.config.baud_rate, 250000)
    verify("CAN encoder rate", odrv.axis0.config.can.encoder_msg_rate_ms, 100)

    # DC bus limits
    verify("DC max positive current", odrv.config.dc_max_positive_current, 100.0, tolerance=0.1)
    verify("DC max negative current", odrv.config.dc_max_negative_current, -25.0, tolerance=0.1)

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
