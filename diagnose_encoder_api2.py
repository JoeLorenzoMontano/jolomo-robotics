#!/usr/bin/env python3
"""
Diagnostic script to explore encoder-related objects in firmware v0.6.11
"""

import odrive
from odrive.enums import *
import sys

def main():
    print("=" * 80)
    print("Encoder-Related Objects Diagnostic")
    print("=" * 80)

    # Connect to ODrive
    print("\nConnecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        return 1

    print(f"✓ Connected: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    # Test encoder_estimator0
    print("\n" + "=" * 80)
    print("odrv.encoder_estimator0 attributes")
    print("=" * 80)

    try:
        attrs = dir(odrv.encoder_estimator0)
        for attr in attrs:
            if attr.startswith('_'):
                continue
            try:
                value = getattr(odrv.encoder_estimator0, attr)
                if not callable(value):
                    print(f"  {attr}: {value}")
            except:
                print(f"  {attr}: (error)")
    except AttributeError:
        print("  encoder_estimator0 does NOT exist")

    # Test axis0.encoder
    print("\n" + "=" * 80)
    print("odrv.axis0.encoder attributes")
    print("=" * 80)

    try:
        attrs = dir(odrv.axis0.encoder)
        for attr in attrs:
            if attr.startswith('_'):
                continue
            try:
                value = getattr(odrv.axis0.encoder, attr)
                if not callable(value):
                    print(f"  {attr}: {value}")
            except:
                print(f"  {attr}: (error)")
    except AttributeError:
        print("  axis0.encoder does NOT exist")

    # Test specific paths for encoder ready status
    print("\n" + "=" * 80)
    print("Testing encoder ready/status paths")
    print("=" * 80)

    tests = [
        ('odrv.onboard_encoder0.is_ready', lambda: odrv.onboard_encoder0.is_ready),
        ('odrv.encoder_estimator0.is_ready', lambda: odrv.encoder_estimator0.is_ready),
        ('odrv.axis0.encoder.is_ready', lambda: odrv.axis0.encoder.is_ready),
        ('odrv.onboard_encoder0.error', lambda: odrv.onboard_encoder0.error),
        ('odrv.encoder_estimator0.error', lambda: odrv.encoder_estimator0.error),
        ('odrv.axis0.encoder.error', lambda: odrv.axis0.encoder.error),
    ]

    for name, getter in tests:
        try:
            value = getter()
            print(f"✓ {name}: {value}")
        except AttributeError as e:
            print(f"✗ {name}: Does not exist")
        except Exception as e:
            print(f"⚠ {name}: {type(e).__name__}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
