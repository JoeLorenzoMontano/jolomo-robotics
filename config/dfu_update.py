#!/usr/bin/env python3
"""
ODrive DFU Firmware Update Script
Handles firmware update when ODrive is in DFU mode
"""

import subprocess
import sys

print("="*80)
print("ODrive Firmware Update via DFU")
print("="*80)

print("\nThis will update the ODrive firmware to the latest version.")
print("The ODrive is currently in DFU mode and needs firmware to boot normally.")

print("\nStarting firmware update...")
print("This will take 2-5 minutes. Please wait...\n")

try:
    # Run odrivetool new-dfu with automatic yes via stdin
    process = subprocess.Popen(
        ['odrivetool', 'new-dfu'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    stdout, _ = process.communicate(input='y\n', timeout=300)
    result_returncode = process.returncode
    result_stdout = stdout

    print(result_stdout)

    if result_returncode == 0:
        print("\n" + "="*80)
        print("✓ Firmware update completed successfully!")
        print("="*80)
        print("\nThe ODrive should now reboot to normal mode.")
        print("Wait 5 seconds, then verify with: python3 -c 'import odrive; odrv=odrive.find_any(); print(f\"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}\")'")
    else:
        print("\n" + "="*80)
        print("✗ Firmware update failed!")
        print("="*80)
        print(f"Exit code: {result_returncode}")
        sys.exit(1)

except subprocess.TimeoutExpired:
    print("\n✗ Firmware update timed out (>5 minutes)")
    sys.exit(1)
except Exception as e:
    print(f"\n✗ Error during firmware update: {e}")
    sys.exit(1)
