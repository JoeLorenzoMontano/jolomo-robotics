#!/usr/bin/env python3
"""
ODrive S1 Encoder Direction Test Script

This script tests the encoder direction and provides interactive guidance
to fix direction mismatches between encoder and motor.

The correct behavior is:
- Rotating motor shaft CLOCKWISE (looking at shaft end) → encoder position INCREASES
- Rotating motor shaft COUNTER-CLOCKWISE → encoder position DECREASES

If the opposite occurs, the encoder direction needs to be inverted.
"""

import odrive
from odrive.enums import *
import time
import sys

def main():
    print("=" * 80)
    print("ODrive S1 Encoder Direction Test")
    print("=" * 80)

    # Connect to ODrive
    print("\n[1/4] Connecting to ODrive S1...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        sys.exit(1)

    print(f"✓ Found ODrive S1: {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  DC Bus Voltage: {odrv.vbus_voltage:.2f}V")

    # Check encoder configuration
    print("\n[2/4] Checking encoder configuration...")
    print(f"  Commutation encoder: {odrv.axis0.config.commutation_encoder} (should be 4)")
    print(f"  Load encoder: {odrv.axis0.config.load_encoder} (should be 4)")

    if odrv.axis0.config.commutation_encoder != 4 or odrv.axis0.config.load_encoder != 4:
        print("\n⚠ WARNING: Encoder not configured correctly!")
        print("Please run: python3 configure_odrive_s1.py")
        sys.exit(1)

    # Check encoder status
    try:
        encoder_enabled = odrv.onboard_encoder0.config.enabled
        print(f"  Encoder enabled: {encoder_enabled}")

        if not encoder_enabled:
            print("\n⚠ WARNING: Onboard encoder is not enabled!")
            print("Please run: python3 configure_odrive_s1.py")
            sys.exit(1)

        # Check current direction setting
        try:
            current_direction = odrv.onboard_encoder0.config.direction
            print(f"  Current direction: {current_direction} (0=normal, 1=inverted)")
        except AttributeError:
            current_direction = 0
            print(f"  Current direction: 0 (default)")

    except AttributeError as e:
        print(f"⚠ WARNING: Could not read encoder status: {e}")

    # Test encoder reading
    print("\n[3/4] Testing encoder position reading...")
    print("\n⚠ IMPORTANT: Motor must be in IDLE state for manual rotation test")
    print(f"  Current axis state: {odrv.axis0.current_state}")

    if odrv.axis0.current_state != AxisState.IDLE:
        print("\n  Setting motor to IDLE state...")
        odrv.axis0.requested_state = AxisState.IDLE
        time.sleep(0.5)

    print("\n" + "-" * 80)
    print("ENCODER DIRECTION TEST - Manual Rotation Required")
    print("-" * 80)

    try:
        # Read initial position
        try:
            initial_pos = odrv.onboard_encoder0.pos_estimate
        except AttributeError:
            initial_pos = odrv.encoder_estimator0.pos_estimate

        print(f"\nInitial encoder position: {initial_pos:.6f} turns")

        print("\n📍 INSTRUCTIONS:")
        print("1. Look at the motor shaft from the END (not from motor body)")
        print("2. Slowly rotate the shaft CLOCKWISE by about 1/4 turn (90 degrees)")
        print("3. Press ENTER when done")

        input("\nPress ENTER after rotating clockwise... ")

        # Read position after clockwise rotation
        try:
            cw_pos = odrv.onboard_encoder0.pos_estimate
        except AttributeError:
            cw_pos = odrv.encoder_estimator0.pos_estimate

        cw_change = cw_pos - initial_pos

        print(f"\nPosition after clockwise rotation: {cw_pos:.6f} turns")
        print(f"Change: {cw_change:+.6f} turns ({cw_change * 360:+.1f} degrees)")

        # Analyze direction
        print("\n" + "=" * 80)
        print("ANALYSIS")
        print("=" * 80)

        if abs(cw_change) < 0.05:
            print("\n⚠ WARNING: Very small position change detected")
            print("  Possible causes:")
            print("  1. Motor shaft wasn't rotated enough")
            print("  2. Encoder is not detecting magnet (error 4 likely)")
            print("  3. Encoder offset calibration not completed")
            print("\n  Try rotating more or run calibration first")
            return 1

        direction_correct = cw_change > 0

        if direction_correct:
            print("\n✓ ENCODER DIRECTION CORRECT!")
            print(f"  Clockwise rotation → Position INCREASED by {cw_change:.6f} turns")
            print("  This is the expected behavior.")
            print("\n  Current configuration is correct. No changes needed.")

            # Show current direction setting
            try:
                current_dir = odrv.onboard_encoder0.config.direction
                print(f"\n  Encoder direction setting: {current_dir} (0=normal, 1=inverted)")
            except AttributeError:
                pass

            return 0

        else:
            print("\n✗ ENCODER DIRECTION INVERTED!")
            print(f"  Clockwise rotation → Position DECREASED by {abs(cw_change):.6f} turns")
            print("  This is INCORRECT behavior.")

            print("\n" + "-" * 80)
            print("FIX REQUIRED: Invert Encoder Direction")
            print("-" * 80)

            # Determine what to set direction to
            try:
                current_dir = odrv.onboard_encoder0.config.direction
                new_dir = 1 if current_dir == 0 else 0
                print(f"\n  Current encoder direction: {current_dir}")
                print(f"  Need to set encoder direction to: {new_dir}")
            except AttributeError:
                # Assume current is 0, need to set to 1
                current_dir = 0
                new_dir = 1
                print(f"\n  Need to set encoder direction to: {new_dir}")

            print("\n  Would you like to apply this fix now? (y/n)")
            response = input("  > ").strip().lower()

            if response == 'y' or response == 'yes':
                print("\n  Applying encoder direction fix...")
                try:
                    odrv.onboard_encoder0.config.direction = new_dir
                    print(f"  ✓ Set encoder direction to {new_dir}")

                    print("\n  Saving configuration...")
                    odrv.save_configuration()
                    print("  ✓ Configuration saved")

                    print("\n  ODrive will reboot...")
                    time.sleep(3)

                    print("\n  Reconnecting...")
                    odrv = odrive.find_any(timeout=10)
                    print(f"  ✓ Reconnected to ODrive S1")

                    # Verify direction was applied
                    new_dir_verify = odrv.onboard_encoder0.config.direction
                    print(f"\n  Verification: encoder direction = {new_dir_verify}")

                    if new_dir_verify == new_dir:
                        print("\n✓ Encoder direction fix applied successfully!")
                        print("\n  Next steps:")
                        print("  1. Run this test again to verify direction is now correct")
                        print("  2. Run calibration: python3 calibrate_odrive_s1.py")
                        return 0
                    else:
                        print("\n✗ ERROR: Direction setting did not persist!")
                        return 1

                except Exception as e:
                    print(f"\n✗ ERROR: Failed to apply fix: {e}")
                    return 1

            else:
                print("\n  Fix not applied. You can apply manually:")
                print(f"    odrv.onboard_encoder0.config.direction = {new_dir}")
                print(f"    odrv.save_configuration()")
                print(f"    odrv.reboot()")
                return 1

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
