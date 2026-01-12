# ODrive S1 Working Configuration for Eaglepower 8308-90KV

**Date:** 2026-01-12
**Hardware:** ODrive S1 (HW v5.2, FW v0.6.9)
**Motor:** Eaglepower 8308-90KV (40-pole brushless gimbal motor)
**Battery:** 6S LiPo (22.2V nominal, 25.2V fully charged)
**Interface:** Arduino Uno R4 Minima via CAN bus (250kbps)

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
odrv.axis0.config.commutation_encoder = 4  # onboard_encoder0 (magnetic encoder)
odrv.axis0.config.load_encoder = 4  # onboard_encoder0 (magnetic encoder)
```

**CRITICAL:** Must use encoder ID `4` (onboard magnetic encoder), NOT `13` (sensorless estimator).
The sensorless estimator does not work at low speeds/standstill and will prevent motor control.

### Current Limits
```python
odrv.axis0.config.motor.current_soft_max = 15.0  # Amps
odrv.axis0.config.motor.current_hard_max = 20.0  # Amps
```

### Controller Configuration
```python
odrv.axis0.controller.config.control_mode = 2  # VELOCITY_CONTROL
odrv.axis0.controller.config.vel_gain = 0.15  # (N*m*s) / rad
odrv.axis0.controller.config.vel_integrator_gain = 0.3  # (N*m) / rad
odrv.axis0.controller.config.vel_limit = 10.0  # turns/sec
```

### CAN Bus Configuration
```python
odrv.axis0.config.can.node_id = 1  # Must match Arduino sketch (ODRV0_NODE_ID)
odrv.axis0.config.can.encoder_msg_rate_ms = 10  # Enable encoder feedback over CAN
```

**CRITICAL:** `node_id = 1` must match the Arduino sketch.
**CRITICAL:** `encoder_msg_rate_ms = 10` must be non-zero for Arduino to receive position/velocity.

### Battery/Power Limits
```python
odrv.config.dc_bus_overvoltage_trip_level = 30.0  # Volts (safe for 6S @ 25.2V)
odrv.config.dc_bus_undervoltage_trip_level = 18.0  # Volts
odrv.config.dc_max_positive_current = 120.0  # Amps
odrv.config.dc_max_negative_current = -10.0  # Amps
```

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
