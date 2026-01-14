#!/usr/bin/env python3
"""
Quick test to verify motor parameter access works with the new helper function.
This tests the fix without requiring a full calibration run.
"""

import odrive
from odrive.enums import *
import sys

def get_motor_parameters(odrv):
    """
    Get motor phase resistance and inductance from ODrive.
    Tries multiple API paths for firmware version compatibility.
    """
    paths = [
        # v0.6.11 path (tested and confirmed working)
        lambda: (odrv.axis0.config.motor.phase_resistance,
                 odrv.axis0.config.motor.phase_inductance),
        # v0.6.9 path (old firmware)
        lambda: (odrv.axis0.motor.config.phase_resistance,
                 odrv.axis0.motor.config.phase_inductance),
        # Direct on motor object
        lambda: (odrv.axis0.motor.phase_resistance,
                 odrv.axis0.motor.phase_inductance),
    ]

    for path_func in paths:
        try:
            r, l = path_func()
            if r is not None and l is not None and r > 0 and l > 0:
                return (r, l, True)
        except (AttributeError, TypeError):
            continue
        except Exception:
            continue

    return (None, None, False)

def main():
    print("=" * 80)
    print("Testing Motor Parameter Access with Helper Function")
    print("=" * 80)

    # Connect to ODrive
    print("\nConnecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        return 1

    print(f"✓ Connected: Serial {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    # Test the helper function
    print("\nTesting get_motor_parameters() helper function...")
    phase_r, phase_l_h, params_available = get_motor_parameters(odrv)

    if params_available:
        phase_l = phase_l_h * 1e6  # Convert to µH
        print("\n✓ SUCCESS! Motor parameters accessible:")
        print(f"  Phase resistance: {phase_r:.6f} Ω")
        print(f"  Phase inductance: {phase_l:.2f} µH")

        # Validate against expected range
        print("\nValidation:")
        if 0.05 < phase_r < 0.5:
            print(f"  ✓ Resistance in expected range (0.05-0.5 Ω)")
        else:
            print(f"  ⚠ Resistance outside expected range")

        if 20 < phase_l < 200:
            print(f"  ✓ Inductance in expected range (20-200 µH)")
        else:
            print(f"  ⚠ Inductance outside expected range")

        print("\n✓ The fix works correctly!")
        print("  The calibration script should now work without crashing.")
        return 0
    else:
        print("\n✗ FAILED: Motor parameters not accessible")
        print("  This should not happen on firmware v0.6.11")
        print("\nPossible causes:")
        print("  1. Motor has never been calibrated")
        print("  2. API structure changed unexpectedly")
        print("  3. Parameters were cleared/reset")
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
