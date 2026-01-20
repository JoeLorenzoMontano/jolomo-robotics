import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import threading
import time
import re
import os
import math
import numpy as np
from kinematics.ik_solver import RobotIKSolver

app = Flask(__name__)
app.config['SECRET_KEY'] = 'odrive-motor-control-secret'
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    ping_interval=10,      # Send ping every 10 seconds
    ping_timeout=30,       # Wait 30 seconds for pong before disconnect
    async_mode='eventlet',
    logger=False,          # Disable verbose Socket.IO logging
    engineio_logger=False  # Disable verbose Engine.IO logging
)

# Serial connection to Arduino
ser = None
serial_lock = threading.Lock()

# Client connection tracking
connected_clients = 0
clients_lock = threading.Lock()
feedback_thread_started = False

# IK Solver instances (loaded on demand)
ik_solvers = {}  # Cache IK solvers by config name
current_joint_angles = np.zeros(6)  # Current joint state

def init_serial():
    global ser
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
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
        # Only process feedback if there are connected clients
        if connected_clients == 0:
            time.sleep(0.1)  # Sleep longer when no clients
            continue

        feedback = read_feedback()
        if feedback:
            # Debug: Print raw Arduino messages (only HEALTH messages)
            if feedback.startswith("HEALTH_M"):
                print(f"[ARDUINO] {feedback}")
            # Skip POS responses (from get_position polling) - we use FEEDBACK instead
            if feedback.startswith("POS:"):
                continue

            if feedback.startswith("FEEDBACK:"):
                # Parse feedback - supports both single and multi-motor formats
                # Single motor: FEEDBACK:pos,vel
                # Multi-motor: FEEDBACK:<j0_p>,<j0_v>;<j1_p>,<j1_v>
                feedback_data = feedback[9:]
                # Track feedback count
                if not hasattr(feedback_thread, 'feedback_count'):
                    feedback_thread.feedback_count = 0
                feedback_thread.feedback_count += 1

                if ';' in feedback_data:
                    # Multi-motor format
                    try:
                        joints = []
                        joint_parts = feedback_data.split(';')
                        for i, joint_data in enumerate(joint_parts):
                            values = joint_data.split(',')
                            if len(values) == 2:
                                pos = float(values[0])
                                vel = float(values[1])
                                # Convert NaN to None for valid JSON serialization
                                if math.isnan(pos):
                                    pos = None
                                if math.isnan(vel):
                                    vel = None
                                joints.append({
                                    'id': i,
                                    'position': pos,
                                    'velocity': vel
                                })

                        # Emit multi-joint feedback
                        try:
                            socketio.emit('feedback', {
                                'joints': joints,
                                'timestamp': time.time()
                            }, )

                            # For backward compatibility, also emit joint 0 as single motor feedback
                            if joints:
                                socketio.emit('feedback', {
                                    'position': joints[0]['position'],
                                    'velocity': joints[0]['velocity'],
                                    'timestamp': time.time()
                                }, )
                        except Exception:
                            pass  # Silently ignore emit errors when no clients connected
                    except ValueError:
                        pass
                else:
                    # Single motor format (backward compatible)
                    data = feedback_data.split(',')
                    if len(data) == 2:
                        try:
                            pos = float(data[0])
                            vel = float(data[1])

                            # Convert NaN to None (null in JSON) for valid serialization
                            if math.isnan(pos):
                                pos = None
                            if math.isnan(vel):
                                vel = None

                            try:
                                socketio.emit('feedback', {
                                    'position': pos,
                                    'velocity': vel,
                                    'timestamp': time.time()
                                })
                            except Exception as e:
                                pass  # Silently ignore emit errors
                        except ValueError as e:
                            # Silently ignore parse errors
                            pass
            elif feedback.startswith("HEALTH:"):
                # Parse: HEALTH:<state>,<errors>,<vbus>,<ibus>,<t_fet>,<t_motor>,<iq_meas>,<iq_set>,<comm_age>,<comm_ok>,<ctrl_mode>,<input_mode>
                try:
                    parts = feedback[7:].split(',')
                    if len(parts) >= 12:
                        health_data = {
                            'axis_state': int(parts[0]),
                            'errors': int(parts[1]),
                            'bus_voltage': float(parts[2]),
                            'bus_current': float(parts[3]),
                            'fet_temp': float(parts[4]),
                            'motor_temp': float(parts[5]),
                            'iq_measured': float(parts[6]),
                            'iq_setpoint': float(parts[7]),
                            'comm_age': int(parts[8]),
                            'comm_ok': parts[9] == '1',
                            'control_mode': int(parts[10]),
                            'input_mode': int(parts[11])
                        }

                        # Sanitize NaN/Infinity values to None for valid JSON serialization
                        for key in ['bus_voltage', 'bus_current', 'fet_temp', 'motor_temp',
                                    'iq_measured', 'iq_setpoint']:
                            if key in health_data:
                                val = health_data[key]
                                if math.isnan(val) or math.isinf(val):
                                    health_data[key] = None

                        try:
                            socketio.emit('health', health_data, )
                        except Exception:
                            pass

                        # Battery protection alerts
                        bus_voltage = health_data.get('bus_voltage')
                        if bus_voltage is not None:
                            alert_level = None
                            alert_message = None

                            if bus_voltage < 20.4:
                                alert_level = 'critical'
                                alert_message = f'CRITICAL: Battery {bus_voltage:.2f}V - Motors shutting down!'
                            elif bus_voltage < 21.0:
                                alert_level = 'urgent'
                                alert_message = f'URGENT: Battery {bus_voltage:.2f}V - Shutdown imminent!'
                            elif bus_voltage < 22.2:
                                alert_level = 'warning'
                                alert_message = f'WARNING: Battery {bus_voltage:.2f}V - Please land soon'

                            if alert_level:
                                try:
                                    socketio.emit('battery_alert', {
                                        'level': alert_level,
                                        'voltage': bus_voltage,
                                        'message': alert_message
                                    })
                                except Exception:
                                    pass
                except (ValueError, IndexError) as e:
                    # Silently ignore parse errors
                    pass
            elif feedback.startswith("BATTERY_STATE:"):
                # Parse: BATTERY_STATE:<state>:<voltage>V
                try:
                    parts = feedback[14:].split(':')
                    if len(parts) >= 2:
                        state = parts[0]
                        voltage = float(parts[1].replace('V', ''))
                        socketio.emit('battery_state', {
                            'state': state,
                            'voltage': voltage
                        })
                except Exception:
                    pass
            elif feedback.startswith("HEALTH_M"):
                # Parse per-motor HEALTH: HEALTH_M0:... or HEALTH_M1:...
                try:
                    # Extract motor ID
                    motor_id_end = feedback.find(':', 8)
                    motor_id = int(feedback[8:motor_id_end])

                    parts = feedback[motor_id_end+1:].split(',')
                    if len(parts) >= 12:
                        health_data = {
                            'motor_id': motor_id,
                            'axis_state': int(parts[0]) if parts[0] else None,
                            'errors': int(parts[1]) if parts[1] else None,
                            'bus_voltage': float(parts[2]) if parts[2] else None,
                            'bus_current': float(parts[3]) if parts[3] else None,
                            'fet_temp': float(parts[4]) if parts[4] else None,
                            'motor_temp': float(parts[5]) if parts[5] else None,
                            'iq_measured': float(parts[6]) if parts[6] else None,
                            'iq_setpoint': float(parts[7]) if parts[7] else None,
                            'comm_age': int(parts[8]) if parts[8] else None,
                            'comm_ok': parts[9] == '1' if parts[9] else None,
                            'control_mode': int(parts[10]) if parts[10] else None,
                            'input_mode': int(parts[11]) if parts[11] else None
                        }

                        # Sanitize NaN/Infinity values
                        for key in ['bus_voltage', 'bus_current', 'fet_temp', 'motor_temp',
                                    'iq_measured', 'iq_setpoint']:
                            if key in health_data and health_data[key] is not None:
                                val = health_data[key]
                                if math.isnan(val) or math.isinf(val):
                                    health_data[key] = None

                        # Debug: Log before emission
                        print(f"[DEBUG] About to emit health_motor for motor {motor_id}: control_mode={health_data.get('control_mode')}, voltage={health_data.get('bus_voltage')}")

                        # Emit per-motor health event
                        socketio.emit('health_motor', health_data)

                        print(f"[DEBUG] Emitted health_motor event for motor {motor_id}")

                        # Battery alerts only from motor 0 (has actual voltage reading)
                        if motor_id == 0:
                            bus_voltage = health_data.get('bus_voltage')
                            if bus_voltage is not None:
                                alert_level = None
                                alert_message = None

                                if bus_voltage < 20.4:
                                    alert_level = 'critical'
                                    alert_message = f'CRITICAL: Battery {bus_voltage:.2f}V - Motors shutting down!'
                                elif bus_voltage < 21.0:
                                    alert_level = 'urgent'
                                    alert_message = f'URGENT: Battery {bus_voltage:.2f}V - Shutdown imminent!'
                                elif bus_voltage < 22.2:
                                    alert_level = 'warning'
                                    alert_message = f'WARNING: Battery {bus_voltage:.2f}V - Please land soon'

                                if alert_level:
                                    try:
                                        socketio.emit('battery_alert', {
                                            'level': alert_level,
                                            'voltage': bus_voltage,
                                            'message': alert_message
                                        })
                                    except Exception:
                                        pass
                except (ValueError, IndexError) as e:
                    print(f"[ERROR] Failed to parse health_motor: {e}")
                    print(f"[DEBUG] Feedback line: {feedback}")
                except Exception as e:
                    print(f"[ERROR] Unexpected error in health_motor handler: {e}")
                    print(f"[DEBUG] Feedback line: {feedback}")
                    import traceback
                    traceback.print_exc()
            elif feedback.startswith("MOTOR_SHUTDOWN:"):
                # Parse: MOTOR_SHUTDOWN:<reason>:<voltage>V
                try:
                    parts = feedback[15:].split(':')
                    if len(parts) >= 2:
                        reason = parts[0]
                        voltage = float(parts[1].replace('V', ''))
                        socketio.emit('motor_shutdown', {
                            'reason': reason,
                            'voltage': voltage
                        })
                except Exception:
                    pass
            elif feedback.startswith("MOTOR_BLOCKED:"):
                # Motors blocked due to protection
                try:
                    message = feedback[14:]
                    socketio.emit('motor_blocked', {'message': message})
                except Exception:
                    pass
            elif feedback.startswith("ERROR"):
                try:
                    socketio.emit('error', {'message': feedback}, )
                except Exception:
                    pass
            elif feedback.startswith("READY"):
                try:
                    socketio.emit('status', {'state': 'ready'}, )
                except Exception:
                    pass

        time.sleep(0.01)  # 100Hz polling

# Flask routes
@app.route('/')
def index():
    return render_template('index.html')

# WebSocket events
@socketio.on('connect')
def handle_connect():
    global connected_clients, feedback_thread_started
    with clients_lock:
        connected_clients += 1

        # Start feedback thread on first connection (with delay to ensure handshake completes)
        if not feedback_thread_started:
            feedback_thread_started = True
            def delayed_start():
                time.sleep(3)  # Wait longer for handshake to complete
                feedback_task = threading.Thread(target=feedback_thread, daemon=True)
                feedback_task.start()
                print("Feedback thread started after 3-second delay")
            threading.Thread(target=delayed_start, daemon=True).start()
            print("Scheduled feedback thread to start in 3 seconds...")

    print(f'Client connected (total: {connected_clients})')
    emit('status', {'state': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    global connected_clients
    with clients_lock:
        connected_clients = max(0, connected_clients - 1)
    print(f'Client disconnected (remaining: {connected_clients})')

@socketio.on('set_velocity')
def handle_set_velocity(data):
    vel = float(data.get('velocity', 0))
    response = send_command(f"VEL:{vel}")
    emit('command_response', {'command': 'velocity', 'response': response})

@socketio.on('set_position')
def handle_set_position(data):
    pos = float(data.get('position', 0))
    response = send_command(f"POS:{pos}")

    # Check if motor is already near target (from Arduino deadband)
    if response.startswith("NEAR_TARGET:"):
        try:
            current = response.split(':')[1].strip()
            emit('command_response', {
                'command': 'position',
                'response': 'NEAR_TARGET',
                'message': f'Already at target position ({current} turns)',
                'current_position': float(current)
            })
        except (IndexError, ValueError):
            # Fallback if parsing fails
            emit('command_response', {'command': 'position', 'response': response})
    else:
        emit('command_response', {'command': 'position', 'response': response})

@socketio.on('stop')
def handle_stop():
    response = send_command("STOP")
    emit('command_response', {'command': 'stop', 'response': response})

@socketio.on('enable_motor')
def handle_enable_motor():
    response = send_command("ENABLE")
    emit('command_response', {'command': 'enable', 'response': response})

@socketio.on('disable_motor')
def handle_disable_motor():
    response = send_command("DISABLE")
    emit('command_response', {'command': 'disable', 'response': response})

# Per-motor control handlers for dual-motor system
@socketio.on('set_velocity_motor')
def handle_set_velocity_motor(data):
    motor_id = int(data.get('motor_id', 0))
    vel = float(data.get('velocity', 0))
    response = send_command(f"VEL{motor_id}:{vel}")
    emit('command_response', {
        'command': f'velocity_motor_{motor_id}',
        'response': response,
        'motor_id': motor_id
    })

@socketio.on('set_position_motor')
def handle_set_position_motor(data):
    motor_id = int(data.get('motor_id', 0))
    pos = float(data.get('position', 0))
    response = send_command(f"POS{motor_id}:{pos}")

    if response.startswith("NEAR_TARGET:"):
        try:
            current = response.split(':')[1].strip()
            emit('command_response', {
                'command': f'position_motor_{motor_id}',
                'response': 'NEAR_TARGET',
                'message': f'Already at target position ({current} turns)',
                'current_position': float(current),
                'motor_id': motor_id
            })
        except (IndexError, ValueError):
            emit('command_response', {
                'command': f'position_motor_{motor_id}',
                'response': response,
                'motor_id': motor_id
            })
    else:
        emit('command_response', {
            'command': f'position_motor_{motor_id}',
            'response': response,
            'motor_id': motor_id
        })

@socketio.on('enable_motor_id')
def handle_enable_motor_id(data):
    motor_id = int(data.get('motor_id', 0))
    response = send_command(f"ENABLE{motor_id}")
    emit('command_response', {
        'command': f'enable_motor_{motor_id}',
        'response': response,
        'motor_id': motor_id
    })

@socketio.on('disable_motor_id')
def handle_disable_motor_id(data):
    motor_id = int(data.get('motor_id', 0))
    response = send_command(f"DISABLE{motor_id}")
    emit('command_response', {
        'command': f'disable_motor_{motor_id}',
        'response': response,
        'motor_id': motor_id
    })

@socketio.on('stop_all')
def handle_stop_all():
    response = send_command("STOPALL")
    emit('command_response', {'command': 'stop_all', 'response': response})

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

@socketio.on('set_control_mode')
def handle_set_control_mode(data):
    """Set ODrive control mode and input mode"""
    control_mode = int(data.get('control_mode', 2))
    input_mode = int(data.get('input_mode', 2))
    response = send_command(f"SETMODE:{control_mode},{input_mode}")
    emit('command_response', {'command': 'set_control_mode', 'response': response})

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
        # Feedback thread will start automatically on first client connection
        print("Serial initialized. Waiting for client connection to start feedback thread...")

        # Run Flask-SocketIO server
        socketio.run(app, host='0.0.0.0', port=5001, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    else:
        print("Failed to connect to Arduino. Please check the connection.")

