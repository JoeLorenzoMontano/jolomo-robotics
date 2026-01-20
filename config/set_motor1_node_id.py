#!/usr/bin/env python3
"""
Configure second ODrive S1 as Motor 1 (CAN Node ID 2)
This script ONLY sets the CAN node ID - motor configuration should be done separately.
"""

import odrive
import time

def main():
    print("=" * 60)
    print("ODrive S1 Motor 1 - CAN Node ID Configuration")
    print("=" * 60)
    print("\nSearching for ODrive via USB...")
    print("(Make sure ONLY the second motor is connected via USB)\n")

    try:
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Found ODrive S1")
        print(f"  Serial Number: {odrv.serial_number}")
        print(f"  Hardware Version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"  Firmware Version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    except Exception as e:
        print(f"✗ Failed to find ODrive: {e}")
        print("\nTroubleshooting:")
        print("  1. Connect second ODrive via USB")
        print("  2. Disconnect first ODrive USB (if connected)")
        print("  3. Check that ODrive is powered")
        return

    print("\n" + "-" * 60)
    print("CURRENT CONFIGURATION:")
    print("-" * 60)
    print(f"  CAN Node ID: {odrv.axis0.config.can.node_id}")
    print(f"  CAN Baudrate: {odrv.can.config.baud_rate}")
    print(f"  Axis State: {odrv.axis0.current_state}")

    if odrv.axis0.config.can.node_id == 2:
        print("\n✓ This ODrive is already configured as Motor 1 (Node ID 2)")
        print("  No changes needed!")
        return

    print("\n" + "-" * 60)
    print("APPLYING NEW CONFIGURATION:")
    print("-" * 60)
    print("  Setting CAN Node ID: 1 → 2")

    # Set CAN node ID to 2 for Motor 1
    odrv.axis0.config.can.node_id = 2

    # Save configuration
    print("  Saving configuration...")
    try:
        odrv.save_configuration()
        print("  ✓ Configuration saved")
    except Exception as e:
        print(f"  ✗ Failed to save: {e}")
        return

    print("\n" + "=" * 60)
    print("✓ SUCCESS - Motor 1 configured with CAN Node ID 2")
    print("=" * 60)
    print("\nNext steps:")
    print("  1. Disconnect USB from this ODrive")
    print("  2. Ensure both ODrives are:")
    print("     - Powered from 6S battery")
    print("     - Connected to Arduino via CAN bus (CAN_H and CAN_L)")
    print("  3. Refresh the web interface at http://localhost:5003")
    print("  4. Both Motor 0 and Motor 1 should now show live data")
    print("\nNote: ODrive will NOT reboot automatically on S1.")
    print("      The new node ID will take effect immediately.")

if __name__ == "__main__":
    main()
