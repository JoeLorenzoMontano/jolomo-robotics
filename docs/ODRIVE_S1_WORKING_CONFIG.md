# ODrive S1 Working Configuration for Eaglepower 8308-90KV

**Date:** 2026-01-13 (Updated after firmware migration)
**Hardware:** ODrive S1 (HW v5.2, FW v0.6.11) - Migrated from v0.6.9
**Bootloader:** v1.0.0 (new DFU system)
**Motor:** Eaglepower 8308-90KV (40-pole brushless gimbal motor)
**Battery:** 6S LiPo (22.2V nominal, 25.2V fully charged, 6200mAh 120C)
**Interface:** Arduino Uno R4 Minima via CAN bus (250kbps)

---

## Quick Start

Use the automated configuration scripts for easy setup:

1. **Apply Configuration:** `python3 configure_odrive_s1.py`
2. **Run Calibration:** `python3 calibrate_odrive_s1.py`
3. **Test Encoder Direction:** `python3 test_encoder_direction.py` (if needed)

These scripts apply all corrected settings automatically.

---

## Critical Configuration Settings

### Motor Parameters
```python
odrv.axis0.config.motor.motor_type = 0  # MOTOR_TYPE_HIGH_CURRENT (FOC control)
odrv.axis0.config.motor.pole_pairs = 20  # 40 magnets / 2
odrv.axis0.config.motor.torque_constant = 0.091889  # 8.27 / 90 KV
odrv.axis0.config.motor.calibration_current = 10.0  # Amps
```

**CRITICAL:** `motor_type = 0` (HIGH_CURRENT) must be used, NOT `motor_type = 1` (GIMBAL).
GIMBAL mode uses voltage control and does not properly convert torque commands to current.

**CRITICAL:** `torque_constant` must be set correctly. Formula: `8.27 / KV`
For 90KV motor: `8.27 / 90 = 0.091889 Nm/A`
If this is `inf` or wrong, the motor will not produce torque.

### Encoder Configuration
```python
odrv.onboard_encoder0.config.enabled = True

# CRITICAL: Do NOT set mode for onboard encoder!
# The following line is WRONG for S1 onboard encoder:
# odrv.onboard_encoder0.config.mode = 1  # DON'T DO THIS!
# Mode setting is only for Hall sensors (hall_encoder0), not onboard magnetic encoder

odrv.axis0.config.commutation_encoder = 4  # onboard_encoder0 (magnetic encoder)
odrv.axis0.config.load_encoder = 4  # onboard_encoder0 (magnetic encoder)

# Direction inversion (if encoder/motor direction mismatch):
# odrv.onboard_encoder0.config.direction = 1  # 0=normal, 1=inverted
# Use test_encoder_direction.py to check if this is needed
```

**CRITICAL:** Must use encoder ID `4` (onboard magnetic encoder), NOT `13` (sensorless estimator).
The sensorless estimator does not work at low speeds/standstill and will prevent motor control.

**CRITICAL:** Do NOT set `onboard_encoder0.config.mode`. This setting only applies to Hall sensor encoders, not the S1 onboard magnetic encoder. Setting mode incorrectly can cause encoder calibration to fail with error 4.

### Current Limits
```python
odrv.axis0.config.motor.current_soft_max = 20.0  # Amps (increased for more torque)
odrv.axis0.config.motor.current_hard_max = 36.0  # Amps
odrv.axis0.config.motor.calibration_current = 20.0  # Amps (helps with pole detection)
```

### Controller Configuration
```python
odrv.axis0.controller.config.control_mode = 2  # VELOCITY_CONTROL (default)
# Arduino sketch automatically switches to POSITION_CONTROL for POS: commands

# Position Control Gains (for joystick, rotation knob, go-to-position)
odrv.axis0.controller.config.pos_gain = 40.0  # POSITIVE gain (fixed!)
odrv.axis0.controller.config.vel_limit = 5.0  # turns/sec (reduced for smooth motion)

# Velocity Control Gains (for velocity slider)
odrv.axis0.controller.config.vel_gain = 0.3  # (N*m*s) / rad (increased for damping)
odrv.axis0.controller.config.vel_integrator_gain = 0.6  # (N*m) / rad
```

**Important:** Controller gains should be **positive**. If motor direction is inverted, fix it properly using:
- `odrv.onboard_encoder0.config.direction = 1` (preferred method)
- OR swap two motor phase wires (A ↔ B or B ↔ C)

**Never** use negative gains as a workaround - this can destabilize the control loop and make tuning counterintuitive.

### CAN Bus Configuration
```python
odrv.can.config.baud_rate = 250000  # 250 kbps (must match Arduino)
odrv.axis0.config.can.node_id = 1  # Must match Arduino sketch (ODRV0_NODE_ID)

# Enable encoder feedback over CAN - field name may vary by firmware version
# Try both:
odrv.axis0.config.can.encoder_msg_rate_ms = 10  # 100Hz feedback (v0.6.9 uses this)
# OR if above fails:
# odrv.axis0.config.can.encoder_rate_ms = 10
```

**CRITICAL:** `node_id = 1` must match the Arduino sketch.
**CRITICAL:** Encoder feedback rate must be non-zero (10ms = 100Hz) for Arduino to receive position/velocity.
**Note:** Field name may be `encoder_msg_rate_ms` or `encoder_rate_ms` depending on firmware version. The configuration script handles both.

### Battery/Power Limits
```python
odrv.config.dc_bus_overvoltage_trip_level = 30.0  # Volts (safe for 6S @ 25.2V)
odrv.config.dc_bus_undervoltage_trip_level = 20.4  # Volts (3.4V/cell - LiPo protection)
odrv.config.dc_max_positive_current = 120.0  # Amps
odrv.config.dc_max_negative_current = -10.0  # Amps (limit regen without brake resistor)
```

**Important:** Without a brake resistor, `dc_max_negative_current` limits regenerative braking current. Set to -10.0 A to prevent over-voltage from regen.

#### Battery Protection (6S LiPo)

**CRITICAL:** Undervoltage threshold set to **20.4V (3.4V per cell)** to prevent over-discharge damage to LiPo cells.

**Multi-Layer Protection System:**
1. **Arduino Software (Primary):** Real-time monitoring with multi-level warnings
   - Early Warning: 22.2V (3.7V/cell - nominal voltage)
   - Urgent Warning: 21.0V (3.5V/cell - recommended minimum)
   - Critical Shutdown: 20.4V (3.4V/cell - absolute minimum)
   - Auto-recovery at 20.9V (0.5V hysteresis prevents oscillation)

2. **ODrive Hardware (Backup):** Hardware-enforced cutoff at 20.4V
   - Redundant protection if Arduino fails or crashes
   - Cannot be bypassed by software

3. **Web UI (User Awareness):** Visual and audio alerts
   - Color-coded voltage badges (green/yellow/orange/red)
   - Browser notifications for critical states
   - Audio beeps at warning thresholds

**Historical Context:** Previous 18.0V threshold was too low (3.0V/cell). This led to two 6S batteries failing with dead cells due to over-discharge. The new 20.4V threshold prevents cell damage while providing adequate runtime.

### Brake Resistor (Optional)
```python
# If brake resistor is connected (50W 2Ω):
odrv.config.brake_resistor0.enable = True
odrv.config.brake_resistor0.resistance = 2.0  # Ohms

# If no brake resistor connected (default):
# odrv.config.brake_resistor0.enable = False  # Or omit - disabled by default
```

**Note:** ODrive S1 v0.6.9 uses `brake_resistor0` object, not flat config fields like older firmware.

### Watchdog
```python
odrv.axis0.config.enable_watchdog = False  # Disabled to allow calibration to complete
```

---

## Measured Motor Parameters (from calibration)

```python
phase_resistance: 0.091273 Ω
phase_inductance: 0.000052660 H (52.66 µH)
```

These values are measured during motor calibration and stored automatically.

---

## Calibration Procedure

1. **Motor Calibration**
   ```python
   odrv.axis0.requested_state = 4  # AXIS_STATE_MOTOR_CALIBRATION
   # Wait 6-7 seconds
   # Check: odrv.axis0.procedure_result == 0 (success)
   ```

2. **Encoder Offset Calibration**
   ```python
   odrv.axis0.requested_state = 7  # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
   # Wait 3-4 seconds (motor will beep/move slightly)
   # Check: odrv.axis0.procedure_result == 0 or 1 (both work)
   ```

3. **Enter Closed Loop Control**
   ```python
   odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
   # Motor is now ready for velocity/position commands
   ```

---

## Testing Motor Control

### Direct Velocity Command (via USB)
```python
import odrive
odrv = odrive.find_any()

# Command 1 turn/sec
odrv.axis0.controller.input_vel = 1.0

# Check motor is responding
print(f"Velocity: {odrv.encoder_estimator0.vel_estimate:.2f} turns/sec")
print(f"Current: {odrv.axis0.motor.foc.Iq_measured:.2f} A")

# Stop
odrv.axis0.controller.input_vel = 0.0
```

### CAN Bus Control (via Arduino)
Arduino sketch: `/home/jolomoadmin/Internal/development/jolomo-robotics/web_control/s1_web_control/s1_web_control.ino`

Commands over serial:
- `ENABLE` - Run calibration and enter closed loop
- `VEL:1.0` - Set velocity to 1.0 turns/sec
- `STOP` - Set velocity to 0
- `GETPOS` - Read position and velocity

---

## Problem Summary & Solution

### The Problem
- Motor would not physically move despite being in CLOSED_LOOP_CONTROL
- `torque_setpoint` remained at 0.0 even when commanding velocity
- `Iq_setpoint` (motor current) remained at 0.0

### Root Causes Found

1. **Wrong Encoder Configuration (CRITICAL)**
   - `commutation_encoder` was set to `13` (sensorless estimator)
   - Should be `4` (onboard magnetic encoder)
   - Sensorless estimator doesn't work at low speeds → no motor control

2. **Missing Torque Constant**
   - `torque_constant` was set to `inf` (infinity)
   - Controller couldn't convert torque commands to current
   - Formula: `torque_constant = 8.27 / KV = 8.27 / 90 = 0.091889`

3. **Wrong Motor Type (Secondary)**
   - Initially tried `motor_type = 1` (GIMBAL mode)
   - GIMBAL mode uses voltage control, not proper FOC current control
   - Changed to `motor_type = 0` (HIGH_CURRENT) for FOC

### The Fix
```python
# Set correct encoder (MOST IMPORTANT)
odrv.axis0.config.commutation_encoder = 4
odrv.axis0.config.load_encoder = 4

# Set correct torque constant (IMPORTANT)
odrv.axis0.config.motor.torque_constant = 0.091889

# Use HIGH_CURRENT motor type for FOC control
odrv.axis0.config.motor.motor_type = 0

# Save configuration
odrv.save_configuration()
odrv.reboot()

# Recalibrate after changes
odrv.axis0.requested_state = 4  # Motor calibration
# Wait 6s
odrv.axis0.requested_state = 7  # Encoder offset calibration
# Wait 3s
odrv.axis0.requested_state = 8  # Closed loop control
```

---

## Verification Commands

```python
import odrive
odrv = odrive.find_any()

# Verify critical settings
assert odrv.axis0.config.commutation_encoder == 4, "Wrong encoder!"
assert odrv.axis0.config.load_encoder == 4, "Wrong encoder!"
assert odrv.axis0.config.motor.pole_pairs == 20, "Wrong pole pairs!"
assert abs(odrv.axis0.config.motor.torque_constant - 0.0919) < 0.001, "Wrong torque constant!"
assert odrv.axis0.config.can.node_id == 1, "Wrong CAN node ID!"
assert odrv.axis0.config.can.encoder_msg_rate_ms == 10, "CAN encoder feedback disabled!"

print("✓ All critical settings correct!")
```

---

## Arduino CAN Interface

**Sketch:** `s1_web_control.ino`
**CAN Baudrate:** 250000 (must match ODrive)
**Node ID:** 1 (must match ODrive `can.node_id`)

The Arduino communicates with ODrive via CAN bus and provides serial commands for the web interface.

---

## Web Interface

**URL:** http://localhost:5003
**Backend:** Flask/SocketIO (`web_control/docker/app.py`)
**Serial Port:** `/dev/ttyACM1` (Arduino)

The web interface sends serial commands to Arduino, which translates them to CAN messages for the ODrive.

---

## Configuration Issues Fixed (2026-01-13)

### Issue 1: Wrong Encoder Mode Setting (CRITICAL)
**Problem:** Configuration included `odrv.onboard_encoder0.config.mode = 1` (HALL mode)
- This setting is for Hall sensor encoders (`hall_encoder0`), not onboard magnetic encoder
- Caused encoder calibration to fail with error 4 (encoder not detected)

**Fix:** Remove the mode setting entirely - not applicable for onboard encoder

### Issue 2: Negative Position Gain (Poor Practice)
**Problem:** Used `pos_gain = -40.0` to compensate for inverted motor direction
- Negative gains destabilize control loops
- Makes tuning counterintuitive

**Fix:** Use positive gain (`pos_gain = 40.0`) and fix direction via:
- `odrv.onboard_encoder0.config.direction = 1` (preferred)
- OR swap motor phase wires

### Issue 3: Brake Resistor Config Path (Minor)
**Problem:** Used old API paths (`odrv.config.enable_brake_resistor`)
- ODrive S1 firmware uses `brake_resistor0` object

**Fix:** Use `odrv.config.brake_resistor0.enable` or omit if no brake resistor connected

### Issue 4: CAN Encoder Field Name (Version-Specific)
**Problem:** Field name varies by firmware version
- Some use `encoder_msg_rate_ms`, others use `encoder_rate_ms`

**Fix:** Configuration script tries both field names automatically

---

## References

- ODrive S1 Datasheet: https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html
- Eaglepower 8308 Motor: 40-pole (20 pole pairs), 90KV
- Torque Constant Formula: Kt (Nm/A) = 8.27 / KV
- ODrive Encoder IDs:
  - `4` = onboard_encoder0 (magnetic encoder on ODrive S1)
  - `13` = sensorless_estimator0 (doesn't work at low speed)

---

## Important Notes

- Always save configuration after changes: `odrv.save_configuration()`
- ODrive reboots after `save_configuration()` - wait 3-4 seconds to reconnect
- Recalibrate after changing encoder or motor settings
- The motor should draw 0.3-1.0A during normal velocity control
- Position values may wrap around after many rotations (normal behavior)
