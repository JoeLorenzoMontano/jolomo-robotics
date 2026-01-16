# Configuration Comparison

## Parameters Comparison: Provided vs Current configure_odrive_s1.py

### ✓ MATCHING Parameters

| Parameter | Value |
|-----------|-------|
| `dc_bus_overvoltage_trip_level` | 30 V |
| `dc_bus_undervoltage_trip_level` | 18 V |
| `brake_resistor0.enable` | False |
| `motor.pole_pairs` | 20 |
| `motor.torque_constant` | 0.091889 Nm/A |
| `controller.config.control_mode` | VELOCITY_CONTROL |
| `can.config.baud_rate` | 250000 |
| `axis0.config.can.node_id` | 1 |
| `axis0.config.enable_watchdog` | False |
| `axis0.config.load_encoder` | ONBOARD_ENCODER0 (4) |
| `axis0.config.commutation_encoder` | ONBOARD_ENCODER0 (4) |

### ✗ DIFFERENT Parameters

| Parameter | Provided Value | Current Value | Notes |
|-----------|----------------|---------------|-------|
| **DC Bus Current** |
| `dc_max_positive_current` | 100 A | **120 A** | Current is MORE permissive |
| `dc_max_negative_current` | -25 A | **-10 A** | Current is LESS permissive (safer without brake resistor) |
| **Motor Type** |
| `motor.motor_type` | PMSM_CURRENT_CONTROL | **HIGH_CURRENT** | Both are FOC control, different enum names |
| **Current Limits** |
| `motor.current_soft_max` | 20 A | **15 A** | Current is lower (conservative) |
| `motor.current_hard_max` | 36 A | **20 A** | Current is MUCH lower (conservative) |
| `motor.calibration_current` | 20 A | **10 A** | Current is lower (conservative) |
| **Controller** |
| `controller.config.vel_limit` | 10 turns/sec | **5 turns/sec** | Current is slower (conservative) |
| **CAN Messaging** |
| `can.encoder_msg_rate_ms` | 100 ms (10 Hz) | **10 ms (100 Hz)** | Current is 10x faster |

### ⚠ MISSING Parameters (in current config)

These parameters are in your provided list but NOT set in `configure_odrive_s1.py`:

#### Motor Configuration
```python
odrv.axis0.config.motor.resistance_calib_max_voltage = 4
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = False
```

#### Controller Configuration
```python
odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 10
odrv.axis0.controller.config.vel_ramp_rate = 10
```

#### CAN Configuration
```python
odrv.can.config.protocol = Protocol.SIMPLE
odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv.axis0.config.can.iq_msg_rate_ms = 100
odrv.axis0.config.can.torques_msg_rate_ms = 100
odrv.axis0.config.can.error_msg_rate_ms = 100
odrv.axis0.config.can.temperature_msg_rate_ms = 100
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 100
```

#### Other
```python
odrv.config.enable_uart_a = False
```

### ✓ EXTRA Parameters (in current config, not in provided list)

These are set in current config but not in your provided list:

```python
# Controller gains (current config)
odrv.axis0.controller.config.pos_gain = 40.0
odrv.axis0.controller.config.vel_gain = 0.3
odrv.axis0.controller.config.vel_integrator_gain = 0.6
```

## Summary of Key Differences

### 1. Motor Current Limits (SIGNIFICANT)
- **Provided:** More aggressive (20A soft, 36A hard, 20A calibration)
- **Current:** Conservative (15A soft, 20A hard, 10A calibration)
- **Impact:** Lower currents = less torque, safer operation

### 2. DC Bus Current (SIGNIFICANT)
- **Provided:** 100A positive, -25A negative (with brake resistor handling)
- **Current:** 120A positive, -10A negative (safer without brake resistor)
- **Impact:** Current config is safer for regen without brake resistor

### 3. Velocity Limits (MODERATE)
- **Provided:** 10 turns/sec
- **Current:** 5 turns/sec
- **Impact:** Current is slower/safer

### 4. CAN Messaging Rates (MODERATE)
- **Provided:** Most messages at 100ms (10 Hz), encoder at 100ms
- **Current:** Only encoder configured at 10ms (100 Hz)
- **Impact:** Current gets faster encoder feedback but missing other telemetry

### 5. Missing Advanced Features (LOW)
- Velocity ramping (input_mode, vel_ramp_rate)
- Torque limits (torque_soft_min/max)
- Trajectory planning (trap_traj.accel_limit)
- Motor thermistor monitoring
- Additional CAN telemetry
- UART disable

## Recommendations

### Critical to Add (May affect encoder calibration):
1. **Motor calibration voltage limit:**
   ```python
   odrv.axis0.config.motor.resistance_calib_max_voltage = 4
   ```
   This limits voltage during calibration to prevent issues.

2. **Calibration lockin current:**
   ```python
   odrv.axis0.config.calibration_lockin.current = 10
   ```
   This sets the current for encoder offset calibration.

### Important to Consider:
1. **Increase current limits** (if encoder calibration is failing due to insufficient torque):
   ```python
   odrv.axis0.config.motor.current_soft_max = 20  # Up from 15
   odrv.axis0.config.motor.current_hard_max = 36  # Up from 20
   odrv.axis0.config.motor.calibration_current = 20  # Up from 10
   ```

2. **Add CAN telemetry** (for better diagnostics):
   ```python
   odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
   odrv.axis0.config.can.error_msg_rate_ms = 100
   odrv.axis0.config.can.temperature_msg_rate_ms = 100
   ```

### Nice to Have:
- Velocity ramping (smoother control)
- Torque limits (safety)
- Motor thermistor monitoring (thermal protection)
- UART disable (avoid conflicts)

## Motor Type Difference

**PMSM_CURRENT_CONTROL vs HIGH_CURRENT:**
- These are likely the same mode in different firmware versions
- Both enable FOC (Field-Oriented Control) with current control
- HIGH_CURRENT is the v0.6.11 name for what was PMSM_CURRENT_CONTROL
- **No action needed** - they're equivalent
