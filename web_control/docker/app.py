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
                # Parse feedback - supports both single and multi-motor formats
                # Single motor: FEEDBACK:pos,vel
                # Multi-motor: FEEDBACK:<j0_p>,<j0_v>;<j1_p>,<j1_v>
                feedback_data = feedback[9:]

                if ';' in feedback_data:
                    # Multi-motor format
                    try:
                        joints = []
                        joint_parts = feedback_data.split(';')
                        for i, joint_data in enumerate(joint_parts):
                            values = joint_data.split(',')
                            if len(values) == 2:
                                joints.append({
                                    'id': i,
                                    'position': float(values[0]),
                                    'velocity': float(values[1])
                                })

                        # Emit multi-joint feedback
                        socketio.emit('feedback', {
                            'joints': joints,
                            'timestamp': time.time()
                        })

                        # For backward compatibility, also emit joint 0 as single motor feedback
                        if joints:
                            socketio.emit('feedback', {
                                'position': joints[0]['position'],
                                'velocity': joints[0]['velocity'],
                                'timestamp': time.time()
                            })
                    except ValueError:
                        pass
                else:
                    # Single motor format (backward compatible)
                    data = feedback_data.split(',')
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

@socketio.on('set_joint_angles')
def handle_set_joint_angles(data):
    """Handle multi-motor joint angle commands"""
    joints = data.get('joints', [])
    if not joints:
        emit('error', {'message': 'No joint angles provided'})
        return

    # Format: SETALL:<j0>,<j1>,...
    joint_str = ','.join(str(float(j)) for j in joints)
    response = send_command(f"SETALL:{joint_str}")
    emit('command_response', {'command': 'set_joint_angles', 'response': response})

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
