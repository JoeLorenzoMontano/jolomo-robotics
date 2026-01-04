from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import threading
import time
import re

app = Flask(__name__)
app.config['SECRET_KEY'] = 'odrive-motor-control-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Serial connection to Arduino
ser = None
serial_lock = threading.Lock()

def init_serial():
    global ser
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to reset
        print("Serial connection established")
        return True
    except Exception as e:
        print(f"Serial connection failed: {e}")
        return False

def send_command(cmd):
    """Send command to Arduino and get response"""
    with serial_lock:
        try:
            if ser and ser.is_open:
                ser.write(f"{cmd}\n".encode())
                time.sleep(0.05)  # Small delay for response
                response = ser.readline().decode().strip()
                return response
            return "ERROR:Serial not connected"
        except Exception as e:
            return f"ERROR:{str(e)}"

def read_feedback():
    """Read feedback from Arduino (non-blocking)"""
    with serial_lock:
        try:
            if ser and ser.is_open and ser.in_waiting:
                line = ser.readline().decode().strip()
                return line
            return None
        except Exception as e:
            print(f"Read error: {e}")
            return None

def feedback_thread():
    """Background thread to read feedback and emit to clients"""
    while True:
        feedback = read_feedback()
        if feedback:
            if feedback.startswith("FEEDBACK:"):
                # Parse: FEEDBACK:pos,vel
                data = feedback[9:].split(',')
                if len(data) == 2:
                    try:
                        pos = float(data[0])
                        vel = float(data[1])
                        socketio.emit('feedback', {
                            'position': pos,
                            'velocity': vel,
                            'timestamp': time.time()
                        })
                    except ValueError:
                        pass
            elif feedback.startswith("ERROR"):
                socketio.emit('error', {'message': feedback})
            elif feedback.startswith("READY"):
                socketio.emit('status', {'state': 'ready'})

        time.sleep(0.01)  # 100Hz polling

# Flask routes
@app.route('/')
def index():
    return render_template('index.html')

# WebSocket events
@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status', {'state': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('set_velocity')
def handle_set_velocity(data):
    vel = float(data.get('velocity', 0))
    response = send_command(f"VEL:{vel}")
    emit('command_response', {'command': 'velocity', 'response': response})

@socketio.on('set_position')
def handle_set_position(data):
    pos = float(data.get('position', 0))
    response = send_command(f"POS:{pos}")
    emit('command_response', {'command': 'position', 'response': response})

@socketio.on('stop')
def handle_stop():
    response = send_command("STOP")
    emit('command_response', {'command': 'stop', 'response': response})

@socketio.on('get_position')
def handle_get_position():
    response = send_command("GETPOS")
    if response.startswith("POS:"):
        # Parse: POS:value,VEL:value
        parts = response.split(',')
        if len(parts) == 2:
            try:
                pos = float(parts[0].split(':')[1])
                vel = float(parts[1].split(':')[1])
                emit('feedback', {
                    'position': pos,
                    'velocity': vel,
                    'timestamp': time.time()
                })
            except (ValueError, IndexError):
                emit('error', {'message': 'Failed to parse position'})
    else:
        emit('error', {'message': response})

@socketio.on('get_config')
def handle_get_config():
    response = send_command("CONFIG")
    emit('config', {'data': response})

if __name__ == '__main__':
    # Initialize serial connection
    if init_serial():
        # Start feedback thread
        feedback_task = threading.Thread(target=feedback_thread, daemon=True)
        feedback_task.start()

        # Run Flask-SocketIO server
        socketio.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False, allow_unsafe_werkzeug=True)
    else:
        print("Failed to connect to Arduino. Please check the connection.")
