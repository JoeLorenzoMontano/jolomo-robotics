#!/usr/bin/env python3
"""
ODrive S1 Calibration Script with Comprehensive Diagnostics

This script performs motor calibration and encoder offset calibration
with detailed error reporting and diagnostic information.

Calibration Sequence:
1. Motor Calibration (measures phase resistance and inductance)
2. Encoder Offset Calibration (establishes encoder-to-motor alignment)
3. Enter Closed Loop Control
4. Test velocity command
5. Test position command (optional)
"""

import odrive
from odrive.enums import *
import time
import sys

# API Compatibility Note:
# Motor phase resistance/inductance API paths changed between firmware versions:
# - v0.6.9: odrv.axis0.motor.config.phase_resistance
# - v0.6.11: odrv.axis0.config.motor.phase_resistance (structure reversed)
# The get_motor_parameters() helper function tries multiple paths for compatibility.

def get_motor_parameters(odrv):
    """
    Get motor phase resistance and inductance from ODrive.
    Tries multiple API paths for firmware version compatibility.

    The API structure changed between v0.6.9 and v0.6.11:
    - v0.6.9 used: odrv.axis0.motor.config.*
    - v0.6.11 uses: odrv.axis0.config.motor.* (reversed)

    Returns:
        tuple: (phase_resistance_ohms, phase_inductance_henries, success_flag)
               Returns (None, None, False) if parameters unavailable
    """
    # List of paths to try (in order of preference)
    paths = [
        # v0.6.11 path (tested and confirmed working)
        lambda: (odrv.axis0.config.motor.phase_resistance,
                 odrv.axis0.config.motor.phase_inductance),
        # v0.6.9 path (old firmware)
        lambda: (odrv.axis0.motor.config.phase_resistance,
                 odrv.axis0.motor.config.phase_inductance),
        # Direct on motor object (in case structure changes again)
        lambda: (odrv.axis0.motor.phase_resistance,
                 odrv.axis0.motor.phase_inductance),
    ]

    for path_func in paths:
        try:
            r, l = path_func()
            # Validate values are reasonable (positive and non-zero)
            if r is not None and l is not None and r > 0 and l > 0:
                return (r, l, True)
        except (AttributeError, TypeError):
            continue
        except Exception:
            # Log unexpected errors but continue trying other paths
            continue

    return (None, None, False)

def print_section(title):
    """Print a formatted section header"""
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)

def print_subsection(title):
    """Print a formatted subsection header"""
    print("\n" + "-" * 80)
    print(title)
    print("-" * 80)

def check_errors(odrv, axis_num=0):
    """Check and display any errors on the axis"""
    axis = odrv.axis0 if axis_num == 0 else odrv.axis1

    # Check disarm reason
    if axis.disarm_reason != 0:
        print(f"\n⚠ Disarm Reason: {axis.disarm_reason}")

    # Check procedure result
    if axis.procedure_result != 0:
        print(f"⚠ Procedure Result: {axis.procedure_result}")

    # Try to get error codes
    try:
        if axis.motor.error != 0:
            print(f"⚠ Motor Error: {axis.motor.error}")
    except AttributeError:
        pass

    try:
        if odrv.onboard_encoder0.error != 0:
            print(f"⚠ Encoder Error: {odrv.onboard_encoder0.error}")
    except AttributeError:
        pass

    try:
        if axis.encoder.error != 0:
            print(f"⚠ Axis Encoder Error: {axis.encoder.error}")
    except AttributeError:
        pass

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
    print_section("ODrive S1 Calibration Script")
    print("Motor: Eaglepower 8308-90KV (90 KV, 20 pole pairs)")

    # Connect to ODrive
    print("\n[1/7] Connecting to ODrive S1...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        print("Please ensure ODrive is connected via USB and powered on.")
        sys.exit(1)

    print(f"✓ Found ODrive S1: {odrv.serial_number}")
    print(f"  Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"  Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"  DC Bus Voltage: {odrv.vbus_voltage:.2f}V")

    # Verify voltage is sufficient
    if odrv.vbus_voltage < 18.0:
        print("\n⚠ WARNING: DC bus voltage is low!")
        print(f"  Current: {odrv.vbus_voltage:.2f}V")
        print(f"  Minimum: 18.0V")
        print("  Please ensure battery is charged.")
        response = input("\n  Continue anyway? (y/n): ")
        if response.lower() != 'y':
            sys.exit(1)

    # Verify configuration
    print("\n[2/7] Verifying configuration...")
    config_ok = True

    if odrv.axis0.config.motor.pole_pairs != 20:
        print(f"✗ Pole pairs incorrect: {odrv.axis0.config.motor.pole_pairs} (should be 20)")
        config_ok = False
    else:
        print(f"✓ Pole pairs: 20")

    if abs(odrv.axis0.config.motor.torque_constant - 0.091889) > 0.001:
        print(f"✗ Torque constant incorrect: {odrv.axis0.config.motor.torque_constant:.6f} (should be ~0.091889)")
        config_ok = False
    else:
        print(f"✓ Torque constant: {odrv.axis0.config.motor.torque_constant:.6f}")

    # In v0.6.11, ONBOARD_ENCODER0 = 13 (was 4 in v0.6.9)
    if odrv.axis0.config.commutation_encoder != 13:
        print(f"✗ Commutation encoder incorrect: {odrv.axis0.config.commutation_encoder} (should be 13)")
        config_ok = False
    else:
        print(f"✓ Commutation encoder: 13 (onboard)")

    if odrv.axis0.config.load_encoder != 13:
        print(f"✗ Load encoder incorrect: {odrv.axis0.config.load_encoder} (should be 13)")
        config_ok = False
    else:
        print(f"✓ Load encoder: 13 (onboard)")

    if not config_ok:
        print("\n⚠ Configuration issues detected!")
        print("  Please run: python3 configure_odrive_s1.py")
        sys.exit(1)

    # Set motor to IDLE before calibration
    print("\n  Setting motor to IDLE state...")
    odrv.axis0.requested_state = AxisState.IDLE
    time.sleep(0.5)

    # ============================================================================
    # MOTOR CALIBRATION
    # ============================================================================
    print_section("[3/7] Motor Calibration")
    print("This will measure phase resistance and inductance...")
    print("Motor will beep and move slightly.")

    print("\n⚠ SAFETY CHECK:")
    print("  - Ensure motor can rotate freely")
    print("  - Ensure motor is not mechanically loaded")
    print("  - Ensure wiring is correct")

    response = input("\n  Start motor calibration? (y/n): ")
    if response.lower() != 'y':
        print("Calibration cancelled")
        sys.exit(0)

    print("\nStarting motor calibration...")
    print("(This takes about 6-7 seconds)")

    odrv.axis0.requested_state = AxisState.MOTOR_CALIBRATION

    # Wait for calibration with progress
    for i in range(7):
        time.sleep(1)
        print(f"  {i+1}/7 seconds...")

    # Wait for calibration to complete (poll for completion)
    # In v0.6.11: 0=SUCCESS, 1=BUSY, 2+=ERROR
    print("\nWaiting for motor calibration to complete...")
    timeout = 10  # seconds
    start_time = time.time()

    while True:
        result = odrv.axis0.procedure_result
        if result == 0:  # SUCCESS
            break
        elif result == 1:  # BUSY - still running
            if time.time() - start_time > timeout:
                print(f"\n✗ Motor calibration TIMEOUT!")
                print(f"  Procedure still BUSY after {timeout} seconds")
                check_errors(odrv)
                sys.exit(1)
            time.sleep(0.5)
        else:  # Error (2+)
            print(f"\n✗ Motor calibration FAILED!")
            print(f"  Procedure result: {result}")
            check_errors(odrv)
            print("\n  Common causes:")
            print("  - Incorrect wiring (phase wires swapped or loose)")
            print("  - Motor resistance too high or too low")
            print("  - Insufficient power supply")
            sys.exit(1)

    # Display measured parameters
    print("\n✓ Motor calibration SUCCESSFUL!")

    # Get motor parameters (compatible with multiple firmware versions)
    phase_r, phase_l_h, params_available = get_motor_parameters(odrv)

    if params_available:
        phase_l = phase_l_h * 1e6  # Convert H to µH
        print(f"  Phase resistance: {phase_r:.6f} Ω")
        print(f"  Phase inductance: {phase_l:.2f} µH")
    else:
        print(f"  Phase resistance: N/A (API path changed in firmware v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision})")
        print(f"  Phase inductance: N/A")
        print(f"  Note: Run diagnose_odrive_api.py to find correct API path for this firmware")
        phase_r = None
        phase_l = None

    # Verify measured values are reasonable (if available)
    if params_available:
        if phase_r < 0.05 or phase_r > 0.5:
            print(f"\n⚠ WARNING: Phase resistance {phase_r:.6f} Ω seems unusual")
            print(f"  Expected range: 0.05 - 0.5 Ω for this motor")

        if phase_l < 20 or phase_l > 200:
            print(f"\n⚠ WARNING: Phase inductance {phase_l:.2f} µH seems unusual")
            print(f"  Expected range: 20 - 200 µH for this motor")
    else:
        print(f"\n⚠ Note: Cannot validate motor parameters (not accessible via API)")
        print(f"  Calibration succeeded - motor parameters were measured and stored internally")

    # ============================================================================
    # ENCODER OFFSET CALIBRATION
    # ============================================================================
    print_section("[4/7] Encoder Offset Calibration")
    print("This will establish encoder-to-motor alignment...")
    print("Motor will rotate slowly.")

    # Show encoder diagnostics before calibration
    encoder_diagnostics(odrv)

    response = input("\n  Start encoder offset calibration? (y/n): ")
    if response.lower() != 'y':
        print("Calibration cancelled")
        sys.exit(0)

    print("\nStarting encoder offset calibration...")
    print("(This takes about 3-4 seconds)")

    odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

    # Wait for calibration to complete (poll for completion)
    # In v0.6.11: 0=SUCCESS, 1=BUSY, 2+=ERROR
    print("\nWaiting for encoder offset calibration to complete", end="", flush=True)
    timeout = 15  # seconds - encoder calibration can take longer than motor calibration
    start_time = time.time()

    while True:
        result = odrv.axis0.procedure_result
        if result == 0:  # SUCCESS
            break
        elif result == 1:  # BUSY - still running
            if time.time() - start_time > timeout:
                print(f"\n\n✗ Encoder offset calibration TIMEOUT!")
                print(f"  Procedure still BUSY after {timeout} seconds")
                check_errors(odrv)
                encoder_diagnostics(odrv)

                print("\n  Common causes:")
                print("  1. Motor cannot rotate freely")
                print("     → Check for mechanical binding or obstructions")
                print("  2. Insufficient calibration current")
                print("     → Verify calibration_current is set (20A recommended)")
                print("  3. Encoder not responding")
                print("     → Check magnet gap and alignment")

                sys.exit(1)
            time.sleep(0.5)
            print(".", end="", flush=True)  # Progress indicator
        else:  # Error (2+)
            print(f"\n\n✗ Encoder offset calibration FAILED!")
            print(f"  Procedure result: {result}")
            check_errors(odrv)

            # Show detailed encoder diagnostics
            encoder_diagnostics(odrv)

            print("\n  Common causes:")
            print("  1. Encoder magnet not detected (error 4)")
            print("     → Check magnet gap (should be 1-3mm from PCB)")
            print("     → Verify magnet is diametrically magnetized disc")
            print("     → Ensure magnet is securely attached to motor shaft")
            print("  2. Encoder offset calibration failed (error 5)")
            print("     → Motor may not have rotated properly")
            print("     → Check motor wiring and mechanical freedom")
            print("  3. Configuration issue")
            print("     → Run: python3 configure_odrive_s1.py")

            print("\n  For hardware issues:")
            print("  - Measure magnet-to-PCB gap with calipers")
            print("  - Check if magnet is present on motor shaft")
            print("  - Verify magnet type (6mm x 2.5mm diametrically magnetized)")

            sys.exit(1)

    print("\n✓ Encoder offset calibration SUCCESSFUL!")
    try:
        print(f"  Encoder offset: {odrv.axis0.encoder_offset_float:.6f}")
    except AttributeError:
        print(f"  Encoder offset: N/A")

    # Show encoder diagnostics after successful calibration
    encoder_diagnostics(odrv)

    # ============================================================================
    # ENTER CLOSED LOOP CONTROL
    # ============================================================================
    print_section("[5/7] Entering Closed Loop Control")

    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(1)

    if odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
        print(f"\n✗ Failed to enter closed loop control!")
        print(f"  Current state: {odrv.axis0.current_state}")
        check_errors(odrv)
        sys.exit(1)

    print("✓ Successfully entered closed loop control!")
    print(f"  Axis state: {odrv.axis0.current_state}")

    # ============================================================================
    # TEST VELOCITY CONTROL
    # ============================================================================
    print_section("[6/7] Testing Velocity Control")
    print("Testing motor response to velocity commands...")

    response = input("\n  Test velocity control? (y/n): ")
    if response.lower() != 'y':
        print("Velocity test skipped")
    else:
        print("\n  Setting velocity to 1.0 turns/sec...")
        odrv.axis0.controller.input_vel = 1.0
        time.sleep(2)

        vel = odrv.encoder_estimator0.vel_estimate
        current = odrv.axis0.motor.foc.Iq_measured

        print(f"\n  Velocity: {vel:.3f} turns/sec")
        print(f"  Current: {current:.2f} A")

        if abs(vel - 1.0) > 0.3:
            print(f"\n  ⚠ WARNING: Velocity error is large!")
            print(f"    Target: 1.0 turns/sec")
            print(f"    Actual: {vel:.3f} turns/sec")
            print(f"    Error: {abs(vel - 1.0):.3f} turns/sec")
        else:
            print(f"  ✓ Velocity control working (error: {abs(vel - 1.0):.3f} turns/sec)")

        print("\n  Stopping motor...")
        odrv.axis0.controller.input_vel = 0.0
        time.sleep(1)

        print("  ✓ Velocity control test complete")

    # ============================================================================
    # TEST POSITION CONTROL
    # ============================================================================
    print_section("[7/7] Testing Position Control (Optional)")
    print("Testing motor response to position commands...")

    response = input("\n  Test position control? (y/n): ")
    if response.lower() != 'y':
        print("Position test skipped")
    else:
        # Switch to position control mode
        odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL

        current_pos = odrv.encoder_estimator0.pos_estimate
        target_pos = current_pos + 0.5  # Move 0.5 turns (180 degrees)

        print(f"\n  Current position: {current_pos:.3f} turns")
        print(f"  Target position: {target_pos:.3f} turns (0.5 turn movement)")

        print("\n  Commanding position...")
        odrv.axis0.controller.input_pos = target_pos

        # Monitor for 3 seconds
        print("\n  Monitoring position for 3 seconds...")
        for i in range(30):
            pos = odrv.encoder_estimator0.pos_estimate
            vel = odrv.encoder_estimator0.vel_estimate
            if i % 10 == 0:  # Print every second
                print(f"    Position: {pos:.3f}, Velocity: {vel:.3f} turns/sec")
            time.sleep(0.1)

        final_pos = odrv.encoder_estimator0.pos_estimate
        error = abs(final_pos - target_pos)

        print(f"\n  Final position: {final_pos:.3f} turns")
        print(f"  Position error: {error:.3f} turns ({error*360:.1f} degrees)")

        if error < 0.05:
            print("  ✓ Position control working correctly!")
        elif error < 0.2:
            print("  ⚠ Position control working but may need tuning")
        else:
            print("  ✗ Position control error is large - tuning needed")

            # Check for runaway
            final_vel = odrv.encoder_estimator0.vel_estimate
            if abs(final_vel) > 0.5:
                print(f"\n  ⚠ WARNING: Motor still moving at {final_vel:.3f} turns/sec!")
                print("  This may indicate position control instability.")
                print("\n  Stopping motor...")
                odrv.axis0.controller.input_vel = 0.0
                odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

        # Return to velocity control mode
        odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        print("\n  Returned to velocity control mode")

    # ============================================================================
    # CALIBRATION COMPLETE
    # ============================================================================
    print_section("CALIBRATION COMPLETE")
    print("✓ Motor is calibrated and ready for use!")

    print("\n📋 SUMMARY:")

    # Get motor parameters again (might be available now after full calibration)
    phase_r_final, phase_l_h_final, params_available_final = get_motor_parameters(odrv)

    if params_available_final:
        phase_l_final = phase_l_h_final * 1e6  # Convert H to µH
        print(f"  Phase resistance: {phase_r_final:.6f} Ω")
        print(f"  Phase inductance: {phase_l_final:.2f} µH")
    else:
        print(f"  Phase resistance: N/A")
        print(f"  Phase inductance: N/A")
        print(f"  Note: Parameters measured during calibration but not accessible via API")

    # Check encoder ready status (API changed in v0.6.11)
    try:
        encoder_ready = odrv.onboard_encoder0.is_ready
        print(f"  Encoder ready: {encoder_ready}")
    except AttributeError:
        # v0.6.11 uses status instead of is_ready
        try:
            status = odrv.encoder_estimator0.status
            # Status bit 0 indicates ready
            encoder_ready = (status & 0x01) == 0x01
            print(f"  Encoder ready: {encoder_ready} (status: {status})")
        except AttributeError:
            print(f"  Encoder ready: Unknown")

    print(f"  Axis state: {odrv.axis0.current_state}")

    print("\n📋 NEXT STEPS:")
    print("  1. Test with web interface: docker-compose up -d")
    print("  2. Access at: http://localhost:5003")
    print("  3. Test velocity control via web UI")
    print("  4. Test position control (joystick, rotation knob)")

    print("\n  To set motor to IDLE:")
    print("    odrv.axis0.requested_state = 1")

    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
        print("Setting motor to IDLE...")
        try:
            odrv = odrive.find_any(timeout=5)
            odrv.axis0.requested_state = AxisState.IDLE
            print("Motor set to IDLE")
        except:
            pass
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        print("\nAttempting to set motor to IDLE for safety...")
        try:
            odrv = odrive.find_any(timeout=5)
            odrv.axis0.requested_state = AxisState.IDLE
            print("Motor set to IDLE")
        except:
            pass
        sys.exit(1)
