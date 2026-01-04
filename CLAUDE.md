# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino-based robotics control system that integrates:
- **ODrive motor controllers** via CAN bus (250kbps)
- **RC receiver input** for joystick control (4 channels)
- **Arduino Uno R4 Minima** with built-in CAN interface

The system enables real-time motor control using RC joystick input, with velocity control and position hold modes.

## Available Sketches

### 1. bluetooth_can_sandbox.ino
Original sketch using **right joystick** (Channel 3, Pin 11) for motor control. Full-featured with RC control, ODrive CAN communication, and command channel functionality.

### 2. left_joystick_spin.ino
Testing sketch using **left joystick** (Channel 2, Pin 10) for motor spinning control:
- **Joystick up** → Clockwise spin (positive velocity)
- **Joystick down** → Counter-clockwise spin (negative velocity)
- **Centered** → Position hold
- **Command channel** (Channel 4) for configuration and reset
- Speed varies based on joystick position (0 to ±10 turns/sec)

## Hardware Architecture

### CAN Bus Communication
- **Baudrate**: 250000 (must match across all CAN devices)
- **ODrive Node ID**: 1 (ODRV0_NODE_ID)
- **Interface**: Arduino's built-in HardwareCAN using Arduino_CAN library
- **Message Flow**:
  - Incoming: Heartbeat messages (100ms interval), encoder feedback, bus voltage/current
  - Outgoing: setState, setVelocity, setPosition, clearErrors commands

### RC Receiver Integration
- **Channels Used**: 4 (pins 9, 10, 11, 12)
- **Raw Value Range**: ~1400-1600 (typical center around 1500)
- **Calibration**: Uses minMax array (lines 16-26) with values 99-265 for channel mapping
- **Center Values**: Hardcoded per channel (line 247) - channel 3 uses 1480, others use 1495-1500
- **Deadzone**: 50 units around center to prevent jitter

### Control Modes
1. **Velocity Control**: Active when joystick moves from center (line 292)
2. **Position Hold**: Active when joystick is centered (lines 285-288)
3. **Command Channel**: Channel 4 triggers special functions:
   - Value > 1800: Print ODrive configuration
   - Value < 1000: Reset to position hold at current position

## Key Components

### ODrive State Management
- **Initialization Sequence** (lines 160-177):
  1. Wait for heartbeat
  2. Clear errors
  3. Set AXIS_STATE_CLOSED_LOOP_CONTROL
  4. Pump events for 150ms (handles bus congestion and error conditions)
- **User Data**: Tracks last_heartbeat, received_heartbeat, last_feedback, received_feedback

### Callback System
- `onHeartbeat()`: Updates axis state from ODrive heartbeat messages
- `onFeedback()`: Updates encoder position/velocity estimates
- `onCanMessage()`: Routes all CAN messages to registered ODrive instances

### Main Loop Logic (loop() function)
1. Pump CAN events first
2. Read raw RC values from all channels
3. Process command channel for configuration/reset commands (1 second debounce)
4. Apply deadzone to joystick input
5. Calculate target velocity from joystick offset
6. Send position hold or velocity command to ODrive
7. Print debug info every 100ms

## Development Commands

### Prerequisites
```bash
# Ensure you're in the dialout group for serial port access
sudo usermod -a -G dialout $USER
# Then log out and log back in

# Arduino CLI should be installed in ~/.local/bin/
# Required libraries (install via arduino-cli):
arduino-cli lib install "ODriveArduino"
arduino-cli lib install "RC Receiver"
# Arduino_CAN is built-in with the renesas_uno platform
```

### Building and Uploading
The sketch must be in a folder with the same name as the .ino file:
```bash
# Directory structure:
# bluetooth_can_sandbox/
#   └── bluetooth_can_sandbox.ino
# left_joystick_spin/
#   └── left_joystick_spin.ino

# Compile a sketch (example: left joystick spin)
~/.local/bin/arduino-cli compile --fqbn arduino:renesas_uno:minima left_joystick_spin

# Upload to Arduino Uno R4 Minima (typically /dev/ttyACM1, ODrive is on /dev/ttyACM0)
~/.local/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:renesas_uno:minima left_joystick_spin

# Or detect the port automatically
~/.local/bin/arduino-cli board list  # Shows which device is which

# For the original bluetooth_can_sandbox sketch:
~/.local/bin/arduino-cli compile --fqbn arduino:renesas_uno:minima bluetooth_can_sandbox
~/.local/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:renesas_uno:minima bluetooth_can_sandbox
```

### Serial Monitor
```bash
# View debug output (115200 baud) - Arduino is typically on /dev/ttyACM1
~/.local/bin/arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200

# If you need to check which port is which:
# /dev/ttyACM0 = ODrive v3
# /dev/ttyACM1 = Arduino Uno R4 Minima
```

### ODrive Configuration via Python
```bash
# Connect to ODrive via USB for configuration/testing
python3
>>> import odrive
>>> odrv = odrive.find_any()
>>> odrv.vbus_voltage  # Check voltage
>>> odrv.axis0.current_state  # Check state (1=IDLE)
```

## Important Constants and Calibration

### Motor Control Parameters
- `MAX_VELOCITY`: 10.0 turns/sec (line 30)
- `DEADZONE`: 50 units (line 31)
- `RIGHT_JOYSTICK_Y_CHANNEL`: 3 (line 29)

### RC Calibration Process
When recalibrating RC receiver:
1. Run the sketch and observe raw values (printed every loop iteration)
2. Note center values at 5-6 seconds (calibration printout at lines 237-244)
3. Update `centerValues` array (line 247) with observed center points
4. Update `minMax` array (lines 16-26) if using receiver.setMinMax() normalization

### Timing Considerations
- **CAN pump**: Every loop iteration to prevent message loss
- **Feedback print**: 100ms interval (line 297)
- **Command debounce**: 1 second minimum between commands (line 252)
- **Loop delay**: 10ms to prevent CAN bus spam (line 321)

## Critical Dependencies
- `ODriveCAN.h`: ODrive CAN protocol implementation
- `RC_Receiver.h`: RC receiver library for pulse width reading
- `Arduino_CAN.h`: Arduino R4's built-in CAN interface
- `ODriveHardwareCAN.hpp`: Hardware abstraction for CAN interface

## Common Patterns

### Requesting ODrive Data
Always use 1-second timeout and check return value:
```cpp
Get_Bus_Voltage_Current_msg_t vbus;
if (!odrv0.request(vbus, 1)) {
  Serial.println("request failed!");
}
```

### ODrive Control Commands
- **Velocity mode**: `odrv0.setVelocity(targetVelocity)`
- **Position mode**: `odrv0.setPosition(position, velocity_feedforward)`
- **State changes**: `odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)`
- **Error recovery**: `odrv0.clearErrors()`

## Known Limitations

Based on recent commits:
- Only certain CAN message types are guaranteed available (Bus_Voltage_Current, Encoder_Estimates, Heartbeat)
- RC receiver calibration is hardware-specific and requires manual tuning
- The system expects raw values in 1400-1600 range; different RC systems may need recalibration
