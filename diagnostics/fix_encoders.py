#!/usr/bin/env python3
"""
Fix encoder configuration - set to onboard encoder (4)
This should be run AFTER configure_odrive_s1.py
"""

import odrive
from odrive.enums import *
import sys
import time

def main():
    print("=" * 80)
    print("Encoder Configuration Fix")
    print("=" * 80)

    # Connect
    print("\nConnecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        return 1

    print(f"✓ Connected: Serial {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    # Check current encoder settings
    print("\nCurrent encoder configuration:")
    print(f"  Commutation encoder: {odrv.axis0.config.commutation_encoder}")
    print(f"  Load encoder: {odrv.axis0.config.load_encoder}")

    if odrv.axis0.config.commutation_encoder == 4 and odrv.axis0.config.load_encoder == 4:
        print("\n✓ Encoders already set correctly to ONBOARD_ENCODER0 (4)")
        return 0

    # Set to onboard encoder
    print("\nSetting encoders to ONBOARD_ENCODER0 (4)...")
    odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0  # 4
    odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0  # 4

    print(f"  Commutation encoder: {odrv.axis0.config.commutation_encoder}")
    print(f"  Load encoder: {odrv.axis0.config.load_encoder}")

    # Save
    print("\nSaving configuration...")
    print("  (ODrive will reboot - disconnect is normal)")
    try:
        odrv.save_configuration()
        print("✓ Configuration saved")
    except Exception as e:
        if "disconnect" in str(e).lower():
            print("✓ Configuration saved (disconnect is normal)")
        else:
            print(f"ERROR: {e}")
            return 1

    # Wait for reboot
    print("\nWaiting for ODrive to reboot (5 seconds)...")
    time.sleep(5)

    # Verify
    print("\nVerifying configuration...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not reconnect: {e}")
        return 1

    comm_enc = odrv.axis0.config.commutation_encoder
    load_enc = odrv.axis0.config.load_encoder

    print(f"  Commutation encoder: {comm_enc}")
    print(f"  Load encoder: {load_enc}")

    if comm_enc == 4 and load_enc == 4:
        print("\n✓ SUCCESS! Encoders configured correctly.")
        print("  Ready for calibration: python3 calibrate_odrive_s1.py")
        return 0
    else:
        print(f"\n✗ FAILED! Encoders still incorrect.")
        print(f"  This may indicate a firmware issue or conflicting configuration.")
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
