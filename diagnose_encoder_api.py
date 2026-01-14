#!/usr/bin/env python3
"""
Diagnostic script to explore encoder API structure in firmware v0.6.11
"""

import odrive
from odrive.enums import *
import sys

def main():
    print("=" * 80)
    print("Encoder API Diagnostic Tool")
    print("=" * 80)

    # Connect to ODrive
    print("\nConnecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        return 1

    print(f"✓ Connected: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    # Explore onboard_encoder0
    print("\n" + "=" * 80)
    print("Exploring odrv.onboard_encoder0 attributes")
    print("=" * 80)

    attrs = dir(odrv.onboard_encoder0)
    relevant = []

    for attr in attrs:
        if attr.startswith('_'):
            continue

        try:
            value = getattr(odrv.onboard_encoder0, attr)
            if not callable(value):
                print(f"  {attr}: {value}")
                relevant.append(attr)
        except:
            print(f"  {attr}: (error reading)")

    # Test specific attributes we need
    print("\n" + "=" * 80)
    print("Testing specific encoder attributes")
    print("=" * 80)

    tests = [
        ('odrv.onboard_encoder0.config.enabled', lambda: odrv.onboard_encoder0.config.enabled),
        ('odrv.onboard_encoder0.is_ready', lambda: odrv.onboard_encoder0.is_ready),
        ('odrv.onboard_encoder0.shadow_count', lambda: odrv.onboard_encoder0.shadow_count),
        ('odrv.onboard_encoder0.count_in_cpr', lambda: odrv.onboard_encoder0.count_in_cpr),
        ('odrv.onboard_encoder0.pos_estimate', lambda: odrv.onboard_encoder0.pos_estimate),
        ('odrv.onboard_encoder0.error', lambda: odrv.onboard_encoder0.error),
    ]

    for name, getter in tests:
        try:
            value = getter()
            print(f"✓ {name}: {value}")
        except AttributeError as e:
            print(f"✗ {name}: AttributeError - {e}")
        except Exception as e:
            print(f"⚠ {name}: {type(e).__name__} - {e}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
