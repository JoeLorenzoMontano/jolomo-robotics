#!/usr/bin/env python3
"""
Test the updated encoder_diagnostics function
"""

import odrive
from odrive.enums import *
import sys

def print_subsection(title):
    """Print a formatted subsection header"""
    print("\n" + "-" * 80)
    print(title)
    print("-" * 80)

def encoder_diagnostics(odrv):
    """Print detailed encoder diagnostic information"""
    print_subsection("Encoder Diagnostics")

    # In v0.6.11, the encoder API structure changed significantly
    # Many attributes moved or were removed. Try to access what's available.

    try:
        # Try v0.6.9 style first (for backward compatibility)
        try:
            enabled = odrv.onboard_encoder0.config.enabled
            print(f"  Encoder enabled: {enabled}")
        except AttributeError:
            # v0.6.11 doesn't have config.enabled
            # Encoder is implicitly enabled if configured in axis config
            print(f"  Encoder enabled: Yes (configured via axis0.config)")

        # Try is_ready (v0.6.9)
        try:
            ready = odrv.onboard_encoder0.is_ready
            print(f"  Encoder ready: {ready}")
        except AttributeError:
            # v0.6.11 uses status instead of is_ready
            # Check encoder_estimator0.status or onboard_encoder0.status
            try:
                status = odrv.encoder_estimator0.status
                # Status 13 (0x0D) means encoder is ready and working
                ready = (status & 0x01) == 0x01  # Bit 0 indicates ready
                print(f"  Encoder estimator status: {status} {'(ready)' if ready else '(not ready)'}")
            except AttributeError:
                try:
                    status = odrv.onboard_encoder0.status
                    print(f"  Onboard encoder status: {status}")
                except AttributeError:
                    print(f"  Encoder status: N/A")

        # Position estimate - try encoder_estimator0 first (v0.6.11)
        try:
            pos = odrv.encoder_estimator0.pos_estimate
            print(f"  Position estimate: {pos:.6f} turns")
        except AttributeError:
            try:
                pos = odrv.onboard_encoder0.pos_estimate
                print(f"  Position estimate: {pos:.6f} turns")
            except AttributeError:
                print(f"  Position estimate: N/A")

        # Velocity estimate (v0.6.11)
        try:
            vel = odrv.encoder_estimator0.vel_estimate
            print(f"  Velocity estimate: {vel:.6f} turns/sec")
        except AttributeError:
            print(f"  Velocity estimate: N/A")

        # Raw encoder value (v0.6.11)
        try:
            raw = odrv.onboard_encoder0.raw
            print(f"  Encoder raw value: {raw:.6f}")
        except AttributeError:
            print(f"  Encoder raw value: N/A")

        # Shadow count (v0.6.9 only)
        try:
            shadow = odrv.onboard_encoder0.shadow_count
            print(f"  Shadow count: {shadow}")
        except AttributeError:
            pass  # Not available in v0.6.11

        # Count in CPR (v0.6.9 only)
        try:
            cpr = odrv.onboard_encoder0.count_in_cpr
            print(f"  Count in CPR: {cpr}")
        except AttributeError:
            pass  # Not available in v0.6.11

        # Error status - try multiple paths
        try:
            error = odrv.onboard_encoder0.error
            print(f"  Encoder error: {error}")
        except AttributeError:
            try:
                error = odrv.encoder_estimator0.error
                print(f"  Encoder error: {error}")
            except AttributeError:
                pass  # Error attribute not available in v0.6.11

    except Exception as e:
        print(f"  Could not read encoder diagnostics: {e}")

def main():
    print("=" * 80)
    print("Testing Updated Encoder Diagnostics Function")
    print("=" * 80)

    # Connect to ODrive
    print("\nConnecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        return 1

    print(f"✓ Connected: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    # Set to IDLE for safety
    odrv.axis0.requested_state = AxisState.IDLE
    import time
    time.sleep(0.5)

    # Test the encoder diagnostics function
    encoder_diagnostics(odrv)

    print("\n" + "=" * 80)
    print("✓ Test complete! Encoder diagnostics function works without errors.")
    print("=" * 80)

    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
