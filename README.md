# Jolomo Robotics

Arduino-based robotics control system for real-time motor control via CAN bus communication with ODrive motor controllers.

## Overview

This project provides a complete solution for controlling high-performance motors using:
- **Arduino Uno R4 Minima** as the main controller
- **ODrive S1** motor controllers for brushless motor control
- **RC receiver** input for joystick-based control
- **CAN bus** communication at 250kbps

The system enables real-time velocity control and position hold modes, with configurable control parameters and diagnostic tools for setup and troubleshooting.

## Features

- Real-time motor control with RC joystick input
- Velocity control mode with adjustable speed (0-10 turns/sec)
- Position hold mode when joystick is centered
- CAN bus communication with ODrive motor controllers
- Comprehensive configuration scripts for ODrive S1
- Automated calibration procedures for motors and encoders
- Diagnostic tools for troubleshooting encoder and API issues
- Web interface for monitoring and control

## Hardware Requirements

- Arduino Uno R4 Minima (with built-in CAN interface)
- ODrive S1 motor controller (v0.6.9 or v0.6.11 firmware)
- Brushless motor (configured for 20 pole pairs, 90 KV)
- RC receiver (4+ channels)
- 24V power supply

## Quick Start

### Arduino Setup

```bash
# Compile and upload the main control sketch
~/.local/bin/arduino-cli compile --fqbn arduino:renesas_uno:minima bluetooth_can_sandbox
~/.local/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:renesas_uno:minima bluetooth_can_sandbox

# Monitor serial output
~/.local/bin/arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
```

### ODrive Configuration

```bash
# Configure ODrive S1 parameters
python3 configure_odrive_s1.py

# Run motor and encoder calibration
python3 calibrate_odrive_s1.py

# Test encoder direction
python3 test_encoder_direction.py
```

## Project Structure

- **Arduino sketches**: Motor control code with RC receiver integration
- **Python scripts**: ODrive configuration, calibration, and diagnostics
- **Diagnostic tools**: Encoder API testing and parameter access utilities
- **Documentation**: Configuration guides and firmware compatibility notes

## Control Modes

1. **Velocity Control**: Active when joystick moves from center position
   - Speed varies based on joystick position (0 to ±10 turns/sec)
   - Smooth acceleration/deceleration with configurable ramp rates

2. **Position Hold**: Active when joystick returns to center
   - Maintains current position with configurable hold torque
   - Automatic engagement with deadzone filtering

3. **Command Channel**: Special functions via RC Channel 4
   - Configuration mode and system reset capabilities

## Firmware Compatibility

Scripts support both ODrive firmware v0.6.9 and v0.6.11, with automatic API path detection for cross-version compatibility. See `CLAUDE.md` for detailed API differences.

## Development

This project is designed to work with Claude Code. See `CLAUDE.md` for detailed architecture notes, development commands, and troubleshooting guidance.

## License

This project is provided as-is for educational and development purposes.
