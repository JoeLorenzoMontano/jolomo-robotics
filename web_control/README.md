# ODrive Web Control

Web-based motor control system for ODrive via Arduino and CAN bus.

## Architecture

```
Web Browser → Docker (Flask + WebSocket) → Serial USB → Arduino → CAN → ODrive Motor
```

## Components

- **Arduino Sketch** (`web_control.ino`): Reads commands from serial, controls ODrive via CAN
- **Flask Backend** (`docker/app.py`): WebSocket server for real-time communication
- **Web GUI** (`docker/templates/index.html`): Advanced control interface with graphs
- **Docker**: Containerized deployment

## Setup Instructions

### 1. Upload Arduino Sketch

```bash
cd /home/jolomoadmin/Internal/development/jolomo-robotics
~/.local/bin/arduino-cli compile --fqbn arduino:renesas_uno:minima web_control
~/.local/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:renesas_uno:minima web_control
```

### 2. Verify Arduino Connection

Check that Arduino is on `/dev/ttyACM1`:
```bash
ls -la /dev/ttyACM*
```

If it's on a different port, update `docker/app.py` line with correct device.

### 3. Start Docker Container

```bash
cd web_control
docker-compose up --build
```

### 4. Access Web Interface

Open your browser to:
```
http://localhost:5003
```

## Web Interface Features

### Controls
- **Velocity Slider**: Set motor velocity from -10 to +10 turns/sec
- **Preset Speeds**: Quick buttons for common velocities
- **Fine Control**: Adjust velocity by ±0.1 or ±0.5
- **Position Control**: Go to specific position
- **Emergency Stop**: Immediate motor stop

### Display
- **Real-time Position/Velocity**: Large numeric displays
- **Position Graph**: Real-time position history
- **Velocity Graph**: Real-time velocity history
- **System Log**: Command and error messages
- **Connection Status**: Visual indicator

## Serial Protocol

Commands (PC → Arduino):
- `VEL:<value>` - Set velocity (e.g., `VEL:5.5`)
- `STOP` - Stop motor
- `POS:<value>` - Go to position (e.g., `POS:10.5`)
- `GETPOS` - Request current position/velocity
- `CONFIG` - Get ODrive configuration
- `STATUS` - Get motor state

Responses (Arduino → PC):
- `OK` - Command accepted
- `POS:<pos>,VEL:<vel>` - Position and velocity
- `FEEDBACK:<pos>,<vel>` - Periodic feedback (100Hz)
- `ERROR:<msg>` - Error message
- `READY` - System ready

## Troubleshooting

### Arduino not found
- Check USB connection
- Verify port with `ls /dev/ttyACM*`
- Update port in `docker/app.py`

### Docker permission denied
- Ensure user is in dialout group: `sudo usermod -a -G dialout $USER`
- Log out and back in
- Or run Docker with `--privileged` flag

### WebSocket not connecting
- Check Docker logs: `docker-compose logs -f`
- Verify port 5001 is not in use
- Check firewall settings

### Motor not responding
- Verify ODrive CAN connection
- Check ODrive is powered and calibrated
- Monitor Arduino serial output directly

## Development

### Update Backend
After modifying `docker/app.py`:
```bash
docker-compose down
docker-compose up --build
```

### Update Frontend
After modifying `docker/templates/index.html`:
- Refresh browser (Ctrl+F5)
- No rebuild needed

### Update Arduino
After modifying `web_control.ino`:
```bash
~/.local/bin/arduino-cli compile --fqbn arduino:renesas_uno:minima web_control
~/.local/bin/arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:renesas_uno:minima web_control
```

## Safety Notes

- Emergency stop button immediately stops motor
- Always test in safe environment
- Keep velocity limits reasonable for your application
- Monitor motor temperature and current draw
- Web interface exposed only on localhost by default
