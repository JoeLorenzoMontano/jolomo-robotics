from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import threading
import time
import re
import os
import numpy as np
from kinematics.ik_solver import RobotIKSolver

app = Flask(__name__)
app.config['SECRET_KEY'] = 'odrive-motor-control-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Serial connection to Arduino
ser = None
serial_lock = threading.Lock()

# IK Solver instances (loaded on demand)
ik_solvers = {}  # Cache IK solvers by config name
current_joint_angles = np.zeros(6)  # Current joint state

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

@socketio.on('set_velocity_ramp')
def handle_set_velocity_ramp(data):
    """Set velocity ramping parameters"""
    acceleration = float(data.get('acceleration', 5.0))
    deceleration = float(data.get('deceleration', 5.0))
    response = send_command(f"SETRAMP:{acceleration},{deceleration}")
    emit('command_response', {'command': 'set_velocity_ramp', 'response': response})

@socketio.on('set_velocity_ramp_enabled')
def handle_set_velocity_ramp_enabled(data):
    """Enable or disable velocity ramping"""
    enabled = 1 if data.get('enabled', True) else 0
    response = send_command(f"RAMPENABLE:{enabled}")
    emit('command_response', {'command': 'set_velocity_ramp_enabled', 'response': response})

@socketio.on('get_velocity_ramp')
def handle_get_velocity_ramp():
    """Get current velocity ramping parameters"""
    response = send_command("GETRAMP")
    emit('velocity_ramp_config', {'data': response})

@socketio.on('set_cartesian_target')
def handle_set_cartesian_target(data):
    """
    Handle Cartesian position target using inverse kinematics
    Computes joint angles and sends to Arduino
    """
    global current_joint_angles, ik_solvers

    try:
        # Extract target position
        x = float(data.get('x', 0))
        y = float(data.get('y', 0))
        z = float(data.get('z', 0))
        config = data.get('config', '3dof')

        # Map config to file path
        config_map = {
            '2dof': 'configs/robot_config_2dof_test.json',
            '3dof': 'configs/robot_config_3dof_test.json',
            '6dof': 'configs/robot_config_6dof_default.json'
        }

        config_path = config_map.get(config)
        if not config_path:
            emit('error', {'message': f'Unknown configuration: {config}'})
            return

        # Load or get cached IK solver
        if config not in ik_solvers:
            try:
                print(f"Loading IK solver for {config}...")
                ik_solvers[config] = RobotIKSolver(config_path)
                print(f"IK solver loaded successfully")
            except Exception as e:
                emit('error', {'message': f'Failed to load IK solver: {str(e)}'})
                return

        ik_solver = ik_solvers[config]

        # Compute IK (use current joint angles as initial guess)
        target_position = [x, y, z]
        initial_guess = current_joint_angles[:ik_solver.num_joints]

        print(f"Computing IK for target: {target_position}")
        joint_angles, success, error = ik_solver.compute_ik(
            target_position,
            initial_guess=initial_guess
        )

        if not success:
            emit('error', {
                'message': f'IK failed: position unreachable (error: {error:.4f}m)'
            })
            emit('ik_result', {
                'success': False,
                'error': error,
                'target': target_position
            })
            return

        # Update current joint angles
        for i in range(len(joint_angles)):
            current_joint_angles[i] = joint_angles[i]

        # Format joint angles for SETALL command
        joint_str = ','.join(str(float(angle)) for angle in joint_angles)
        response = send_command(f"SETALL:{joint_str}")

        # Send success response
        emit('ik_result', {
            'success': True,
            'joint_angles': joint_angles.tolist(),
            'error': error,
            'target': target_position
        })
        emit('command_response', {
            'command': 'set_cartesian_target',
            'response': response
        })

        print(f"IK success: angles={joint_angles}, error={error:.4f}m")

    except Exception as e:
        print(f"Cartesian target error: {e}")
        import traceback
        traceback.print_exc()
        emit('error', {'message': f'Cartesian control error: {str(e)}'})

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
