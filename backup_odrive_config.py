#!/usr/bin/env python3
"""
ODrive Configuration Backup Script
Saves current configuration before firmware migration
"""

import odrive
import json
from datetime import datetime

def backup_config():
    print("Connecting to ODrive...")
    odrv = odrive.find_any()

    config = {
        'backup_date': datetime.now().isoformat(),
        'device_info': {
            'serial': odrv.serial_number,
            'hardware_version': f"v{odrv.hw_version_major}.{odrv.hw_version_minor}",
            'firmware_version': f"v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}",
        },
        'power': {
            'dc_bus_overvoltage_trip_level': odrv.config.dc_bus_overvoltage_trip_level,
            'dc_bus_undervoltage_trip_level': odrv.config.dc_bus_undervoltage_trip_level,
            'dc_max_positive_current': odrv.config.dc_max_positive_current,
            'dc_max_negative_current': odrv.config.dc_max_negative_current,
        },
        'motor': {
            'motor_type': odrv.axis0.config.motor.motor_type,
            'pole_pairs': odrv.axis0.config.motor.pole_pairs,
            'torque_constant': odrv.axis0.config.motor.torque_constant,
            'current_soft_max': odrv.axis0.config.motor.current_soft_max,
            'current_hard_max': odrv.axis0.config.motor.current_hard_max,
            'calibration_current': odrv.axis0.config.motor.calibration_current,
        },
        'controller': {
            'pos_gain': odrv.axis0.controller.config.pos_gain,
            'vel_gain': odrv.axis0.controller.config.vel_gain,
            'vel_integrator_gain': odrv.axis0.controller.config.vel_integrator_gain,
            'vel_limit': odrv.axis0.controller.config.vel_limit,
            'control_mode': odrv.axis0.controller.config.control_mode,
        },
        'can': {
            'baud_rate': odrv.can.config.baud_rate,
            'node_id': odrv.axis0.config.can.node_id,
            'encoder_msg_rate_ms': 10,  # Standard value
        },
        'encoder': {
            'commutation_encoder': odrv.axis0.config.commutation_encoder,
            'load_encoder': odrv.axis0.config.load_encoder,
            'note': 'Currently set to 13 (sensorless) - SHOULD BE 4 (onboard)!'
        }
    }

    # Save to JSON file
    backup_file = f"odrive_config_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(backup_file, 'w') as f:
        json.dump(config, f, indent=2)

    print(f"\n✓ Configuration backed up to: {backup_file}")
    print("\nKey settings:")
    print(f"  Serial: {config['device_info']['serial']}")
    print(f"  Firmware: {config['device_info']['firmware_version']}")
    print(f"  Pole pairs: {config['motor']['pole_pairs']}")
    print(f"  Torque constant: {config['motor']['torque_constant']}")
    print(f"  CAN node ID: {config['can']['node_id']}")
    print(f"  ⚠️  Encoder ID: {config['encoder']['commutation_encoder']} (should be 4, not 13!)")

    return backup_file

if __name__ == '__main__':
    backup_config()
