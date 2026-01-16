#!/usr/bin/env python3
"""
Apply Working ODrive S1 Configuration
Eaglepower 8308-90KV with 6S LiPo (6200mAh 120C)

This configuration has been tested and verified working.
Run after firmware updates or configuration resets.
"""

import odrive
from odrive.enums import *
import time
import sys
import math

def apply_config():
    print("=" * 80)
    print("Applying Working ODrive S1 Configuration")
    print("=" * 80)

    print("\nConnecting to ODrive...")
    odrv = odrive.find_any(timeout=10)

    print(f"✓ Connected to ODrive S1: {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  DC Bus: {odrv.vbus_voltage:.2f}V")

    print("\n[1/5] Power configuration...")
    odrv.config.dc_bus_overvoltage_trip_level = 30
    odrv.config.dc_bus_undervoltage_trip_level = 18
    odrv.config.dc_max_positive_current = 100
    odrv.config.dc_max_negative_current = -25
    odrv.config.brake_resistor0.enable = False
    print("✓ Power limits set")

    print("\n[2/5] Motor configuration...")
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.pole_pairs = 20
    odrv.axis0.config.motor.torque_constant = 0.09188888888888888
    odrv.axis0.config.motor.current_soft_max = 20
    odrv.axis0.config.motor.current_hard_max = 36
    odrv.axis0.config.motor.calibration_current = 20
    odrv.axis0.config.motor.resistance_calib_max_voltage = 4
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False
    print("✓ Motor parameters set")

    print("\n[3/5] Controller configuration...")
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    odrv.axis0.controller.config.vel_limit = 10
    odrv.axis0.controller.config.vel_limit_tolerance = 1.2
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.axis0.trap_traj.config.accel_limit = 10
    odrv.axis0.controller.config.vel_ramp_rate = 10
    print("✓ Controller settings applied")

    print("\n[4/5] CAN bus configuration...")
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 250000
    odrv.axis0.config.can.node_id = 1
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 100
    odrv.axis0.config.can.iq_msg_rate_ms = 100
    odrv.axis0.config.can.torques_msg_rate_ms = 100
    odrv.axis0.config.can.error_msg_rate_ms = 100
    odrv.axis0.config.can.temperature_msg_rate_ms = 100
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 100
    print("✓ CAN bus configured")

    print("\n[5/5] Encoder and misc settings...")
    odrv.axis0.config.enable_watchdog = False
    odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
    odrv.config.enable_uart_a = False
    print("✓ Encoder set to ID 4 (onboard)")

    print("\n" + "=" * 80)
    print("SAVING CONFIGURATION")
    print("=" * 80)
    print("ODrive will reboot...")

    try:
        odrv.save_configuration()
        time.sleep(4)

        print("\nReconnecting...")
        odrv = odrive.find_any(timeout=10)

        print("\n" + "=" * 80)
        print("✓ CONFIGURATION APPLIED SUCCESSFULLY!")
        print("=" * 80)
        print(f"Serial: {odrv.serial_number}")
        print(f"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"Encoder ID: {odrv.axis0.config.commutation_encoder} (should be 4)")
        print(f"Current limit: {odrv.axis0.config.motor.current_soft_max}A")
        print(f"\n📋 Next: Run motor calibration")
        print("   python3 calibrate_odrive_s1.py")

        return 0

    except Exception as e:
        print(f"\n✗ Error saving configuration: {e}")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(apply_config())
    except KeyboardInterrupt:
        print("\n\nConfiguration cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
