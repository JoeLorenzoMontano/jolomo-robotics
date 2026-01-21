# Dual-Motor System Troubleshooting & Operations Guide

**Last Updated:** 2026-01-21
**System:** Dual ODrive S1 controllers with Arduino Uno R4 Minima CAN bridge
**Motors:** 2x Eaglepower 8308-90KV (40-pole brushless gimbal motors)

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Device Assignments](#device-assignments)
3. [Firmware Version Differences](#firmware-version-differences)
4. [Common Issues & Solutions](#common-issues--solutions)
5. [Calibration Requirements](#calibration-requirements)
6. [USB vs CAN Operation](#usb-vs-can-operation)
7. [Web UI Behavior](#web-ui-behavior)
8. [Diagnostic Commands](#diagnostic-commands)

---

## System Architecture

### Hardware Configuration

```
Battery (6S LiPo)
    ├── ODrive 0 (Motor 0) - CAN Node 1
    │   └── CAN bus connection only (no USB in production)
    │
    ├── ODrive 1 (Motor 1) - CAN Node 2
    │   └── USB for configuration, CAN for operation
    │
    └── Arduino Uno R4 Minima
        ├── USB → Serial → Flask Web Server
        └── CAN bus (250kbps) → Both ODrives
```

### Data Flow

**Control Path:**
```
Web UI → Flask (app.py) → Serial (/dev/ttyACM1) → Arduino → CAN → ODrives
```

**Feedback Path:**
```
ODrives → CAN (heartbeat, encoder, bus voltage) → Arduino → Serial → Flask → Web UI
```

---

## Device Assignments

### Serial Ports (Linux)

| Device | Port | Purpose | Used By |
|--------|------|---------|---------|
| ODrive 0 | `/dev/ttyACM0` | USB config only | Manual config scripts |
| Arduino | `/dev/ttyACM1` | Serial control | Flask app (`app.py`) |
| ODrive 1 | `/dev/ttyACM0` or `/dev/ttyACM2` | USB config only | Manual config scripts |

**Important:** Port assignments can vary based on USB connection order. Use `arduino-cli board list` to verify which device is on which port.

### CAN Node IDs

| Motor | CAN Node ID | ODrive Config |
|-------|-------------|---------------|
| Motor 0 | `1` | `odrv.axis0.config.can.node_id = 1` |
| Motor 1 | `2` | `odrv.axis0.config.can.node_id = 2` |

**Critical:** These must match the Arduino sketch constants or motor commands will go to the wrong motor.

### Flask App Configuration

File: `web_control/docker/app.py`

```python
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
```

**Note:** Changed from `/dev/ttyACM0` to `/dev/ttyACM1` on 2026-01-21 to connect to Arduino (ODrive moved to ttyACM0).

---

## Firmware Version Differences

### ODrive S1 Firmware Variations

| Feature | FW 0.6.9 | FW 0.6.11 |
|---------|----------|-----------|
| **Encoder ID (onboard)** | `13` | `4` |
| **Encoder config path** | No `onboard_encoder0.config` object | Has `onboard_encoder0.config` |
| **Motor type enum** | `MotorType.HIGH_CURRENT` | `MotorType.PMSM_CURRENT_CONTROL` |
| **API structure** | Flatter structure | More nested config objects |

### Encoder ID Reference

Both firmware versions:
- **Onboard magnetic encoder:** ID `13` (FW 0.6.9) or ID `4` (FW 0.6.11)
- **Sensorless estimator:** ID `13` (FW 0.6.11 only)
- **Never use sensorless** for standstill or low-speed control

### Critical Difference for Configuration Scripts

When setting encoder:
```python
# FW 0.6.9:
odrv.axis0.config.commutation_encoder = 13  # ONBOARD_ENCODER0
odrv.axis0.config.load_encoder = 13

# FW 0.6.11:
odrv.axis0.config.commutation_encoder = 4   # ONBOARD_ENCODER0
odrv.axis0.config.load_encoder = 4
```

**Scripts handle this automatically** - see `configure_odrive_s1.py` lines 95-96 and 269-271.

---

## Common Issues & Solutions

### Issue 1: Motor Won't Enable (Stays in IDLE State)

**Symptoms:**
- Motor state transitions to 4 (MOTOR_CALIBRATION) briefly, then back to 1 (IDLE)
- No errors shown (`active_errors = 0x0`)
- Bus voltage shows correctly (~22-23V)
- Configuration appears correct

**Root Cause:**
Motor is missing **encoder offset calibration**. Even if motor parameters are measured (phase resistance/inductance), the ODrive needs to know the alignment between encoder zero position and motor electrical zero.

**Solution:**
```python
import odrive
from odrive.enums import AxisState

odrv = odrive.find_any()

# Run full calibration sequence
odrv.clear_errors()
odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

# Wait ~13-15 seconds (motor will spin/beep)
# Motor calibration: ~4 seconds
# Encoder offset calibration: ~9 seconds

# Check if successful
if odrv.axis0.current_state == AxisState.IDLE:
    # Try entering closed loop
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    # Motor should now stay in state 8

# Save calibration permanently
odrv.save_configuration()
odrv.reboot()
```

**Prevention:**
Always run full calibration after:
- First-time setup
- Firmware updates
- Configuration changes to motor or encoder settings
- If `phase_resistance` or `phase_inductance` is 0

### Issue 2: Motor 1 Shows 0V Bus Voltage in Web UI

**Symptoms:**
- Motor 0 shows correct voltage (~22-23V)
- Motor 1 shows 0.00V
- Motor 1 otherwise works fine (enables, responds to commands)

**Root Cause:**
This is **by design, not a bug**. Both ODrives share the same DC bus (battery), so querying both for bus voltage is redundant.

**Arduino Code:**
File: `s1_web_control_dual.ino` lines 606-629
```cpp
// Only motor 0 reports voltage/current (shared bus)
if (i == 0) {
  if (!odrives[i]->request(vbus, 1)) continue;
}
// ...
if (i == 0) {
  Serial.print(vbus.Bus_Voltage, 2);
} else {
  Serial.print("0.00,0.00");  // Motor 1 always shows 0.00V
}
```

**Solution:**
This is **intentional behavior**. No fix needed. Monitor Motor 0 bus voltage for battery status.

**Optional Fix (if you want both to show voltage):**
Modify Arduino code to query bus voltage from both ODrives:
```cpp
// Remove the "if (i == 0)" check around vbus request
Get_Bus_Voltage_Current_msg_t vbus;
if (!odrives[i]->request(vbus, 1)) continue;

// Remove the conditional in serial output
Serial.print(vbus.Bus_Voltage, 2);
Serial.print(",");
Serial.print(vbus.Bus_Current, 2);
```

### Issue 3: Wrong Motor Responds to Commands

**Symptoms:**
- Commanding Motor 0 moves Motor 1
- Commanding Motor 1 moves Motor 0
- Both motors show heartbeats

**Root Cause:**
CAN node IDs are swapped or incorrect.

**Diagnosis:**
```python
import odrive

# Connect to each ODrive via USB one at a time
odrv = odrive.find_any()
print(f"Serial: {odrv.serial_number}")
print(f"CAN Node ID: {odrv.axis0.config.can.node_id}")
```

**Expected:**
- ODrive 0 (Motor 0): `node_id = 1`
- ODrive 1 (Motor 1): `node_id = 2`

**Solution:**
```python
odrv.axis0.config.can.node_id = 1  # or 2, depending on which motor
odrv.save_configuration()
odrv.reboot()
```

### Issue 4: No Encoder Feedback Over CAN

**Symptoms:**
- Arduino receives heartbeats but no encoder position/velocity
- Web UI shows position as 0.000 or stale data
- `FEEDBACK:` messages show 0.000,0.000

**Root Cause:**
CAN encoder message rate is disabled (set to 0).

**Diagnosis:**
```python
print(f"Encoder msg rate: {odrv.axis0.config.can.encoder_msg_rate_ms}")
# Should be 10-100ms, not 0
```

**Solution:**
```python
odrv.axis0.config.can.encoder_msg_rate_ms = 100  # 100ms = 10Hz
odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 100
odrv.save_configuration()
odrv.reboot()
```

### Issue 5: Serial Port Busy Error

**Error:**
```
SerialException: [Errno 16] Device or resource busy: '/dev/ttyACM1'
```

**Root Cause:**
Flask app (`app.py`) is already using the serial port. Only one process can open a serial port at a time.

**Solutions:**

**Option 1: Stop Flask server temporarily**
```bash
# Find Flask process
ps aux | grep "python.*app.py"
sudo kill <PID>

# Now you can use arduino-cli monitor or Python scripts
# Restart Flask when done:
cd web_control/docker
python3 app.py &
```

**Option 2: Use Flask API for testing**
Send commands via the Flask REST API instead of direct serial access:
```bash
curl -X POST http://localhost:5003/command -d "ENABLE1"
```

**Option 3: Monitor without opening port**
```bash
# This won't work - serial port is exclusive access
# Use Flask's web UI or REST API instead
```

---

## Calibration Requirements

### When Calibration is Needed

**Always Required:**
- ✅ First-time setup
- ✅ After firmware update
- ✅ After changing motor parameters (pole pairs, torque constant)
- ✅ After changing encoder configuration

**Sometimes Required:**
- ⚠️ After power loss (if calibration not saved to NVM)
- ⚠️ If motor disconnected and reconnected (encoder offset may shift)
- ⚠️ If motor physically moved while powered off (some encoder types)

**Never Required:**
- ❌ Normal power cycles (if calibration saved to NVM)
- ❌ Reboots or restarts
- ❌ CAN node ID changes

### Calibration Types

| Type | State | Duration | What it Does | When Needed |
|------|-------|----------|--------------|-------------|
| **Motor Calibration** | `AXIS_STATE_MOTOR_CALIBRATION` (4) | ~4-7 sec | Measures phase resistance and inductance | After motor wiring changes, first setup |
| **Encoder Offset Calibration** | `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` (7 or 9) | ~3-4 sec | Finds encoder-to-motor phase alignment | After encoder config changes, first setup |
| **Full Calibration** | `AXIS_STATE_FULL_CALIBRATION_SEQUENCE` (3) | ~13-15 sec | Both motor + encoder calibration | First time setup, recommended for reliability |

### Full Calibration Workflow

```python
import odrive
from odrive.enums import AxisState
import time

odrv = odrive.find_any()

print("Starting full calibration...")
odrv.clear_errors()
odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

# Monitor progress
while odrv.axis0.current_state != AxisState.IDLE:
    state = odrv.axis0.current_state
    print(f"State: {state}", end="\r")
    time.sleep(0.1)
    if time.time() > start_time + 20:  # Timeout
        print("\nCalibration timeout!")
        break

# Verify calibration succeeded
print(f"\nPhase resistance: {odrv.axis0.config.motor.phase_resistance:.6f} Ω")
print(f"Phase inductance: {odrv.axis0.config.motor.phase_inductance*1e6:.2f} µH")

if odrv.axis0.config.motor.phase_resistance < 0.001:
    print("ERROR: Motor calibration failed (resistance = 0)")
else:
    print("✓ Motor calibration successful")

# Test closed loop
odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(1)

if odrv.axis0.current_state == 8:
    print("✓ Motor entered closed loop control")

    # Save calibration permanently
    print("\nSaving calibration to NVM...")
    odrv.save_configuration()
    print("✓ Calibration saved (ODrive will reboot)")
else:
    print(f"✗ Failed to enter closed loop (state: {odrv.axis0.current_state})")
    print(f"Errors: {hex(odrv.axis0.active_errors)}")
```

### Calibration Warning Signs

**Motor doesn't move during calibration:**
- Check motor phase wires are connected (U, V, W)
- Check current limit is >5A (`calibration_current`)
- Check for motor errors: `odrv.axis0.motor.active_errors`

**Calibration completes but phase_resistance = 0:**
- Motor wiring issue (disconnected or loose)
- Calibration current too low
- Hardware fault

**Encoder offset calibration fails:**
- Wrong encoder ID (sensorless instead of onboard)
- Encoder not connected (if external encoder)
- Magnet gap too large (>3mm for onboard encoder)

---

## USB vs CAN Operation

### USB Connection (Configuration Mode)

**Use Cases:**
- Initial configuration
- Firmware updates
- Detailed diagnostics
- Running calibration
- Reading error codes

**Connection:**
```python
import odrive
odrv = odrive.find_any()

# Full API access via USB
print(odrv.vbus_voltage)
odrv.axis0.requested_state = 8
```

**Limitations:**
- Only one process can access USB at a time
- ODrive disconnects during `save_configuration()` (normal behavior)
- Slower than CAN for real-time control

### CAN Operation (Production Mode)

**Use Cases:**
- Normal operation after configuration
- Real-time motor control
- Multiple ODrives on same bus
- No USB cable needed

**Arduino Integration:**
```
Arduino reads serial commands from Flask → Translates to CAN messages → ODrive
Arduino receives CAN messages from ODrive → Translates to serial → Flask → Web UI
```

**Advantages:**
- No USB cable required in production
- Multiple ODrives share one CAN bus (up to 127 nodes)
- Fast, deterministic communication (250kbps)
- Arduino provides serial abstraction for web control

**Limitations:**
- Limited API subset over CAN (no full configuration access)
- Debugging requires CAN sniffer or Arduino serial output
- Initial setup still requires USB

### Transition from USB to CAN

**Steps:**
1. ✅ Configure ODrive via USB (motor params, encoder, CAN node ID)
2. ✅ Run calibration via USB
3. ✅ Save configuration to NVM
4. ✅ Test motor control via USB to verify
5. ✅ Disconnect USB cable
6. ✅ Power ODrive from battery only
7. ✅ Control via Arduino CAN bus
8. ✅ Monitor via Arduino serial output (`HEALTH_M0`, `HEALTH_M1` messages)

**Verification:**
```bash
# Arduino serial output should show heartbeats
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200 | grep "HEALTH_M"

# Expected output:
# HEALTH_M0:1,0,22.75,0.00,33.59,nan,0.22,0.25,0,1,2,2
# HEALTH_M1:1,0,0.00,0.00,29.15,nan,-0.05,-0.01,0,1,2,2
```

---

## Web UI Behavior

### Health Data Format

Serial message: `HEALTH_M<id>:<fields>`

**Fields:**
1. `axis_state` - 1=IDLE, 8=CLOSED_LOOP_CONTROL
2. `axis_error` - Error code (0 = no error)
3. `vbus_voltage` - DC bus voltage (Motor 0 only, Motor 1 shows 0.00)
4. `ibus_current` - DC bus current (Motor 0 only, Motor 1 shows 0.00)
5. `fet_temp` - FET temperature (°C)
6. `motor_temp` - Motor temperature (°C, often nan if no thermistor)
7. `iq_measured` - Measured current (A)
8. `iq_setpoint` - Commanded current (A)
9. `comm_age` - Age of last CAN message (seconds) - currently always 0
10. `comm_ok` - Communication status (1 = OK) - currently always 1
11. `ctrl_mode` - Control mode (2 = velocity, 3 = position)
12. `input_mode` - Input mode (2 = velocity ramp)

**Example:**
```
HEALTH_M0:8,0,22.75,0.50,33.59,nan,1.25,1.20,0,1,2,2
         ↑ State 8 (CLOSED_LOOP)
            ↑ No errors
               ↑ Bus voltage 22.75V
                     ↑ Bus current 0.50A
                          ↑ FET temp 33.59°C
```

### Motor State Meanings

| State | Name | Description |
|-------|------|-------------|
| 0 | UNDEFINED | Error state or uninitialized |
| 1 | IDLE | Motor disabled, no control |
| 4 | MOTOR_CALIBRATION | Measuring phase resistance/inductance |
| 7 or 9 | ENCODER_OFFSET_CALIBRATION | Finding encoder-motor alignment |
| 8 | CLOSED_LOOP_CONTROL | Motor enabled and controllable |

**Normal Operation:** State should be 1 (IDLE) when disabled, 8 (CLOSED_LOOP) when enabled.

### Battery Voltage Interpretation

| Voltage | Color | Status | Action |
|---------|-------|--------|--------|
| >22.2V | Green | Good | Normal operation |
| 21.0-22.2V | Yellow | Warning | Consider recharging soon |
| 20.4-21.0V | Orange | Urgent | Recharge immediately |
| <20.4V | Red | Critical | Motors disabled, charge now |

**Note:** 6S LiPo fully charged = 25.2V, nominal = 22.2V, empty = 20.4V (3.4V/cell minimum)

---

## Diagnostic Commands

### Check ODrive Configuration (USB)

```python
import odrive

odrv = odrive.find_any()

print(f"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
print(f"Serial: {odrv.serial_number}")
print(f"Bus voltage: {odrv.vbus_voltage:.2f}V")
print(f"Axis state: {odrv.axis0.current_state}")
print(f"Active errors: {hex(odrv.axis0.active_errors)}")

print(f"\n=== Motor Configuration ===")
print(f"Motor type: {odrv.axis0.config.motor.motor_type}")
print(f"Pole pairs: {odrv.axis0.config.motor.pole_pairs}")
print(f"Torque constant: {odrv.axis0.config.motor.torque_constant}")
print(f"Phase resistance: {odrv.axis0.config.motor.phase_resistance:.6f} Ω")
print(f"Phase inductance: {odrv.axis0.config.motor.phase_inductance*1e6:.2f} µH")

print(f"\n=== Encoder Configuration ===")
print(f"Commutation encoder: {odrv.axis0.config.commutation_encoder}")
print(f"Load encoder: {odrv.axis0.config.load_encoder}")

print(f"\n=== CAN Configuration ===")
print(f"Node ID: {odrv.axis0.config.can.node_id}")
print(f"Baud rate: {odrv.can.config.baud_rate}")
print(f"Encoder msg rate: {odrv.axis0.config.can.encoder_msg_rate_ms}ms")
print(f"Heartbeat msg rate: {odrv.axis0.config.can.heartbeat_msg_rate_ms}ms")
```

### Check Arduino CAN Communication

```bash
# Monitor Arduino serial output
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200

# Filter for health messages only
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200 | grep "HEALTH_M"

# Check specific motor
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200 | grep "HEALTH_M1"
```

### Test Motor Enable

```bash
# Via serial command (requires Flask app stopped)
echo "ENABLE0" > /dev/ttyACM1

# Via Flask API (production)
curl -X POST http://localhost:5003/command -d "ENABLE0"
curl -X POST http://localhost:5003/command -d "ENABLE1"
```

### Check Which Port is Which

```bash
arduino-cli board list

# Output example:
# Port         Protocol Type           Board Name    FQBN
# /dev/ttyACM0 serial   Serial Port    ODrive
# /dev/ttyACM1 serial   Serial Port    Arduino Uno R4 Minima arduino:renesas_uno:minima

# Or use dmesg
dmesg | tail -20 | grep tty
```

### Export Current Configuration

```python
import odrive
import json

odrv = odrive.find_any()

config = {
    "serial_number": odrv.serial_number,
    "firmware": f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}",
    "vbus_voltage": odrv.vbus_voltage,
    "motor": {
        "motor_type": odrv.axis0.config.motor.motor_type,
        "pole_pairs": odrv.axis0.config.motor.pole_pairs,
        "torque_constant": odrv.axis0.config.motor.torque_constant,
        "phase_resistance": odrv.axis0.config.motor.phase_resistance,
        "phase_inductance": odrv.axis0.config.motor.phase_inductance,
        "current_soft_max": odrv.axis0.config.motor.current_soft_max,
        "current_hard_max": odrv.axis0.config.motor.current_hard_max,
    },
    "encoder": {
        "commutation_encoder": odrv.axis0.config.commutation_encoder,
        "load_encoder": odrv.axis0.config.load_encoder,
    },
    "can": {
        "node_id": odrv.axis0.config.can.node_id,
        "baud_rate": odrv.can.config.baud_rate,
        "encoder_msg_rate_ms": odrv.axis0.config.can.encoder_msg_rate_ms,
        "heartbeat_msg_rate_ms": odrv.axis0.config.can.heartbeat_msg_rate_ms,
    }
}

print(json.dumps(config, indent=2))

# Save to file
with open(f"odrive_config_{odrv.serial_number}.json", "w") as f:
    json.dump(config, f, indent=2)
```

---

## Quick Reference

### Critical Settings Checklist

Before deployment, verify:

- ✅ Motor type = 0 (PMSM/HIGH_CURRENT, not GIMBAL)
- ✅ Pole pairs = 20 (for 8308 motor)
- ✅ Torque constant = 0.091889 (8.27 / 90 KV)
- ✅ Encoder ID = 13 (FW 0.6.9) or 4 (FW 0.6.11) - **not sensorless!**
- ✅ CAN node ID = 1 (Motor 0) or 2 (Motor 1)
- ✅ CAN baud rate = 250000 (250kbps)
- ✅ CAN encoder msg rate = 10-100ms (not 0)
- ✅ Phase resistance > 0.08Ω (indicates successful calibration)
- ✅ Calibration saved to NVM (`save_configuration()`)
- ✅ Motor enters state 8 (CLOSED_LOOP) when enabled

### Emergency Commands

**Stop all motors immediately:**
```bash
curl -X POST http://localhost:5003/command -d "STOPALL"
# Or via Arduino serial:
echo "STOPALL" > /dev/ttyACM1
```

**Disable motor that won't stop:**
```bash
# Via Flask API
curl -X POST http://localhost:5003/command -d "DISABLE0"
curl -X POST http://localhost:5003/command -d "DISABLE1"
```

**Clear ODrive errors (USB):**
```python
import odrive
odrv = odrive.find_any()
odrv.clear_errors()
odrv.axis0.requested_state = 1  # IDLE
```

**Kill Flask server:**
```bash
ps aux | grep "python.*app.py" | grep -v grep | awk '{print $2}' | xargs sudo kill
```

---

## Additional Resources

- **Main config doc:** `/docs/ODRIVE_S1_WORKING_CONFIG.md`
- **Config scripts:** `/config/configure_odrive_s1.py`, `/config/calibrate_odrive_s1.py`
- **Arduino sketch:** `/web_control/s1_web_control_dual/s1_web_control_dual.ino`
- **Flask app:** `/web_control/docker/app.py`
- **ODrive docs:** https://docs.odriverobotics.com/

---

## Changelog

### 2026-01-21
- Initial document created based on Motor 1 calibration troubleshooting
- Documented encoder offset calibration requirement
- Explained Motor 1 bus voltage 0V display behavior (by design)
- Added firmware version differences (0.6.9 vs 0.6.11 encoder IDs)
- Documented serial port assignments and Flask app configuration
- Added comprehensive diagnostic commands and workflows
