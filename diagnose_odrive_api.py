#!/usr/bin/env python3
"""
ODrive API Diagnostic Tool - Motor Parameter Path Finder

This script safely explores the ODrive S1 API structure to discover the correct
paths for accessing motor phase resistance and inductance measurements.

The API structure changed between firmware versions (v0.6.9 → v0.6.11), causing
calibration scripts to fail when trying to access motor parameters.

This diagnostic tool:
1. Connects to ODrive and sets motor to IDLE (safety)
2. Displays firmware version information
3. Uses introspection to explore the motor object structure
4. Tests multiple candidate API paths
5. Reports which paths work and displays actual values
6. Provides recommendation for fixing calibration scripts
"""

import odrive
from odrive.enums import *
import sys

def print_section(title):
    """Print a formatted section header"""
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)

def print_subsection(title):
    """Print a formatted subsection"""
    print("\n" + "-" * 80)
    print(title)
    print("-" * 80)

def explore_motor_object(odrv):
    """Use introspection to explore the motor object structure"""
    print_subsection("Motor Object Introspection")

    print("\nAvailable attributes on odrv.axis0.motor:")
    motor_attrs = dir(odrv.axis0.motor)

    # Look for attributes that might contain motor parameters
    keywords = ['resistance', 'inductance', 'phase', 'config', 'measured', 'R', 'L']
    relevant_attrs = []

    for attr in motor_attrs:
        # Skip private/magic methods
        if attr.startswith('_'):
            continue

        # Check if attribute name contains any keywords
        attr_lower = attr.lower()
        if any(keyword.lower() in attr_lower for keyword in keywords):
            relevant_attrs.append(attr)

            # Try to read the attribute value
            try:
                value = getattr(odrv.axis0.motor, attr)
                # Format based on type
                if isinstance(value, (int, float)):
                    print(f"  ✓ {attr}: {value}")
                else:
                    print(f"  ✓ {attr}: {type(value).__name__} object")
            except Exception as e:
                print(f"  ⚠ {attr}: {type(e).__name__}")

    if not relevant_attrs:
        print("  No relevant attributes found with keywords:", keywords)

    # Check if motor.config exists
    print("\nChecking for motor.config object:")
    if hasattr(odrv.axis0.motor, 'config'):
        print("  ✓ odrv.axis0.motor.config EXISTS")
        try:
            config_attrs = dir(odrv.axis0.motor.config)
            print("\n  Available attributes on odrv.axis0.motor.config:")
            for attr in config_attrs:
                if attr.startswith('_'):
                    continue
                attr_lower = attr.lower()
                if any(keyword.lower() in attr_lower for keyword in keywords):
                    try:
                        value = getattr(odrv.axis0.motor.config, attr)
                        if isinstance(value, (int, float)):
                            print(f"    ✓ {attr}: {value}")
                        else:
                            print(f"    ✓ {attr}: {type(value).__name__}")
                    except:
                        pass
        except Exception as e:
            print(f"  ⚠ Could not explore motor.config: {e}")
    else:
        print("  ✗ odrv.axis0.motor.config DOES NOT EXIST")

def test_candidate_paths(odrv):
    """Test multiple candidate API paths for phase resistance/inductance"""
    print_subsection("Testing Candidate API Paths")

    # Define candidate paths to test
    candidates = [
        {
            'name': 'v0.6.9 path (motor.config)',
            'resistance': lambda: odrv.axis0.motor.config.phase_resistance,
            'inductance': lambda: odrv.axis0.motor.config.phase_inductance
        },
        {
            'name': 'Direct on motor object',
            'resistance': lambda: odrv.axis0.motor.phase_resistance,
            'inductance': lambda: odrv.axis0.motor.phase_inductance
        },
        {
            'name': 'Abbreviated (R/L)',
            'resistance': lambda: odrv.axis0.motor.R,
            'inductance': lambda: odrv.axis0.motor.L
        },
        {
            'name': 'Abbreviated (phase_R/phase_L)',
            'resistance': lambda: odrv.axis0.motor.phase_R,
            'inductance': lambda: odrv.axis0.motor.phase_L
        },
        {
            'name': 'Measured values',
            'resistance': lambda: odrv.axis0.motor.measured_resistance,
            'inductance': lambda: odrv.axis0.motor.measured_inductance
        },
        {
            'name': 'Reversed structure (config.motor)',
            'resistance': lambda: odrv.axis0.config.motor.phase_resistance,
            'inductance': lambda: odrv.axis0.config.motor.phase_inductance
        },
        {
            'name': 'FOC object',
            'resistance': lambda: odrv.axis0.motor.foc.phase_resistance,
            'inductance': lambda: odrv.axis0.motor.foc.phase_inductance
        },
    ]

    working_paths = []

    for candidate in candidates:
        print(f"\nTesting: {candidate['name']}")
        try:
            r = candidate['resistance']()
            l = candidate['inductance']()

            # Validate values are reasonable
            if r is not None and l is not None and r > 0 and l > 0:
                print(f"  ✓ SUCCESS!")
                print(f"    Phase resistance: {r:.6f} Ω")
                print(f"    Phase inductance: {l*1e6:.2f} µH ({l:.9f} H)")
                working_paths.append({
                    'name': candidate['name'],
                    'resistance': r,
                    'inductance': l
                })
            else:
                print(f"  ⚠ Path exists but values invalid (R={r}, L={l})")

        except AttributeError as e:
            print(f"  ✗ FAILED: {e}")
        except Exception as e:
            print(f"  ✗ ERROR: {type(e).__name__}: {e}")

    return working_paths

def main():
    print_section("ODrive API Diagnostic Tool - Motor Parameter Path Finder")
    print("This tool safely explores the API to find motor parameter paths.")

    # Connect to ODrive
    print("\n[1/5] Connecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        print("Please ensure ODrive is connected via USB and powered on.")
        return 1

    print(f"✓ Connected: Serial {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"  DC Bus Voltage: {odrv.vbus_voltage:.2f}V")

    # Safety: Set motor to IDLE
    print("\n[2/5] Setting motor to IDLE for safety...")
    try:
        odrv.axis0.requested_state = AxisState.IDLE
        import time
        time.sleep(0.5)
        print(f"✓ Motor state: {odrv.axis0.current_state} (IDLE)")
    except Exception as e:
        print(f"⚠ Could not set motor to IDLE: {e}")

    # Explore API structure
    print("\n[3/5] Exploring API structure...")
    explore_motor_object(odrv)

    # Test candidate paths
    print("\n[4/5] Testing candidate API paths...")
    working_paths = test_candidate_paths(odrv)

    # Results and recommendation
    print_section("Results and Recommendation")

    if working_paths:
        print(f"\n✓ Found {len(working_paths)} working path(s):\n")

        for i, path in enumerate(working_paths, 1):
            print(f"{i}. {path['name']}")
            print(f"   Phase resistance: {path['resistance']:.6f} Ω")
            print(f"   Phase inductance: {path['inductance']*1e6:.2f} µH")

        # Provide code recommendation
        best_path = working_paths[0]
        print("\n" + "=" * 80)
        print("RECOMMENDED CODE FOR calibrate_odrive_s1.py:")
        print("=" * 80)

        # Determine the attribute path based on which test passed
        if 'motor.config' in best_path['name']:
            print("\nUse the v0.6.9 path (motor.config):")
            print("```python")
            print("phase_r = odrv.axis0.motor.config.phase_resistance")
            print("phase_l = odrv.axis0.motor.config.phase_inductance")
            print("```")
        elif 'Direct on motor' in best_path['name']:
            print("\nUse the direct motor attribute path:")
            print("```python")
            print("phase_r = odrv.axis0.motor.phase_resistance")
            print("phase_l = odrv.axis0.motor.phase_inductance")
            print("```")
        elif 'Abbreviated (R/L)' in best_path['name']:
            print("\nUse abbreviated attribute names:")
            print("```python")
            print("phase_r = odrv.axis0.motor.R")
            print("phase_l = odrv.axis0.motor.L")
            print("```")
        else:
            print(f"\nPath: {best_path['name']}")
            print("Update calibrate_odrive_s1.py accordingly")

        print("\nDisplay example:")
        print("```python")
        print(f"print(f'Phase resistance: {{phase_r:.6f}} Ω')")
        print(f"print(f'Phase inductance: {{phase_l*1e6:.2f}} µH')")
        print("```")

        # Validate against known good values
        print("\n" + "=" * 80)
        print("VALIDATION AGAINST KNOWN GOOD VALUES:")
        print("=" * 80)
        print("\nFrom ODRIVE_S1_WORKING_CONFIG.md:")
        print("  Expected resistance: 0.091273 Ω")
        print("  Expected inductance: 52.66 µH")

        print(f"\nMeasured values:")
        print(f"  Actual resistance:  {best_path['resistance']:.6f} Ω")
        print(f"  Actual inductance:  {best_path['inductance']*1e6:.2f} µH")

        # Check if values match
        r_diff = abs(best_path['resistance'] - 0.091273)
        l_diff = abs(best_path['inductance']*1e6 - 52.66)

        if r_diff < 0.001 and l_diff < 1.0:
            print("\n✓ Values MATCH expected values!")
        else:
            print(f"\n⚠ Values differ from expected:")
            print(f"  Resistance difference: {r_diff:.6f} Ω")
            print(f"  Inductance difference: {l_diff:.2f} µH")
            print("  This may indicate motor needs recalibration")

    else:
        print("\n✗ No working paths found!")
        print("\nPossible reasons:")
        print("  1. Motor has not been calibrated yet (run motor calibration first)")
        print("  2. API structure has changed significantly in this firmware version")
        print("  3. Parameters are stored internally but not exposed via API")

        print("\nRecommendation:")
        print("  Implement graceful degradation in calibrate_odrive_s1.py")
        print("  Display 'N/A' when parameters are not accessible")

    print("\n" + "=" * 80)
    print("✓ Diagnostic complete!")
    print("=" * 80)

    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
