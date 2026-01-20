#include <Arduino.h>
#include "ODriveCAN.h"
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

// ODrive S1 Dual Motor Web Control
// Multi-motor commands: VEL0/VEL1, POS0/POS1, ENABLE0/ENABLE1, DISABLE0/DISABLE1, SETALL, STOPALL
// Legacy commands: VEL, POS, STOP, GETPOS, ENABLE, DISABLE (backward compatible)

#define NUM_MOTORS 2
#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 1
#define ODRV1_NODE_ID 2

// Battery protection thresholds (6S LiPo)
#define VOLTAGE_SHUTDOWN      20.4   // 3.4V per cell - absolute minimum
#define VOLTAGE_RECOVERY      20.9   // Recovery threshold (hysteresis)
#define VOLTAGE_URGENT        21.0   // Urgent warning threshold
#define VOLTAGE_EARLY         22.2   // Early warning (nominal voltage)
#define VOLTAGE_CHECK_INTERVAL 500   // Check every 500ms

enum BatteryState {
  BATTERY_NORMAL,           // > 22.2V - All systems operational
  BATTERY_WARNING_EARLY,    // 21.0-22.2V - Yellow alert
  BATTERY_WARNING_URGENT,   // 20.4-21.0V - Orange alert
  BATTERY_SHUTDOWN          // < 20.4V - Motors disabled
};

BatteryState batteryState = BATTERY_NORMAL;
unsigned long lastVoltageCheck = 0;
float lastVoltageReading = 0.0;
bool motorsDisabledByProtection = false;

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct MotorUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

MotorUserData motor_states[NUM_MOTORS];

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  MotorUserData* motor_user_data = static_cast<MotorUserData*>(user_data);
  motor_user_data->last_heartbeat = msg;
  motor_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  MotorUserData* motor_user_data = static_cast<MotorUserData*>(user_data);
  motor_user_data->last_feedback = msg;
  motor_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

// Battery protection functions
const char* getBatteryStateString(BatteryState state) {
  switch (state) {
    case BATTERY_NORMAL: return "NORMAL";
    case BATTERY_WARNING_EARLY: return "WARNING_EARLY";
    case BATTERY_WARNING_URGENT: return "WARNING_URGENT";
    case BATTERY_SHUTDOWN: return "SHUTDOWN";
    default: return "UNKNOWN";
  }
}

void shutdownMotors(const char* reason) {
  motorsDisabledByProtection = true;

  // Shutdown ALL motors simultaneously
  for (int i = 0; i < NUM_MOTORS; i++) {
    odrives[i]->setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrives[i]->setVelocity(0);
  }

  Serial.print("MOTOR_SHUTDOWN:");
  Serial.print(reason);
  Serial.print(":");
  Serial.print(lastVoltageReading, 2);
  Serial.println("V");
}

bool canSendMotorCommand() {
  if (batteryState == BATTERY_SHUTDOWN || motorsDisabledByProtection) {
    static unsigned long lastBlockLog = 0;
    if (millis() - lastBlockLog > 2000) {
      Serial.println("MOTOR_BLOCKED:Battery protection active");
      lastBlockLog = millis();
    }
    return false;
  }
  return true;
}

void checkBatteryVoltage() {
  unsigned long now = millis();

  // Rate limiting: check every 500ms
  if (now - lastVoltageCheck < VOLTAGE_CHECK_INTERVAL) {
    return;
  }
  lastVoltageCheck = now;

  // Request voltage from ODrive
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    // CAN request failed - log warning but maintain current state
    static unsigned long lastFailWarning = 0;
    if (now - lastFailWarning > 5000) {
      Serial.println("WARNING:Failed to read bus voltage");
      lastFailWarning = now;
    }
    return;
  }

  lastVoltageReading = vbus.Bus_Voltage;
  BatteryState previousState = batteryState;

  // State machine with hysteresis
  switch (batteryState) {
    case BATTERY_NORMAL:
      if (lastVoltageReading < VOLTAGE_EARLY) {
        batteryState = BATTERY_WARNING_EARLY;
      }
      break;

    case BATTERY_WARNING_EARLY:
      if (lastVoltageReading >= VOLTAGE_EARLY) {
        batteryState = BATTERY_NORMAL;
      } else if (lastVoltageReading < VOLTAGE_URGENT) {
        batteryState = BATTERY_WARNING_URGENT;
      }
      break;

    case BATTERY_WARNING_URGENT:
      if (lastVoltageReading >= VOLTAGE_URGENT + 0.2) {
        batteryState = BATTERY_WARNING_EARLY;
      } else if (lastVoltageReading < VOLTAGE_SHUTDOWN) {
        batteryState = BATTERY_SHUTDOWN;
        shutdownMotors("Battery voltage critical");
      }
      break;

    case BATTERY_SHUTDOWN:
      // Only recover if voltage rises above recovery threshold
      if (lastVoltageReading >= VOLTAGE_RECOVERY) {
        batteryState = BATTERY_WARNING_URGENT;
        motorsDisabledByProtection = false;
        Serial.print("BATTERY_RECOVERED:");
        Serial.println(lastVoltageReading, 2);
      }
      break;
  }

  // Log state transitions
  if (batteryState != previousState) {
    Serial.print("BATTERY_STATE:");
    Serial.print(getBatteryStateString(batteryState));
    Serial.print(":");
    Serial.print(lastVoltageReading, 2);
    Serial.println("V");
  }
}

// Helper function to parse comma-separated float array
bool parseFloatArray(String str, float* values, int count) {
  int idx = 0;
  int start = 0;

  for (int i = 0; i < count; i++) {
    int comma = str.indexOf(',', start);
    String token;

    if (comma == -1) {
      // Last value
      token = str.substring(start);
    } else {
      token = str.substring(start, comma);
      start = comma + 1;
    }

    values[i] = token.toFloat();
  }

  return true;
}

void processCommand(String cmd) {
  cmd.trim();

  // Multi-motor commands (VEL0/VEL1, POS0/POS1, ENABLE0/ENABLE1, DISABLE0/DISABLE1)
  if (cmd.startsWith("VEL0:") || cmd.startsWith("VEL1:")) {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }
    int motor_id = (cmd.charAt(3) == '0') ? 0 : 1;
    float vel = cmd.substring(5).toFloat();

    odrives[motor_id]->setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    delay(10);
    odrives[motor_id]->setVelocity(vel);

    Serial.print("OK:MOTOR");
    Serial.println(motor_id);
  }
  else if (cmd.startsWith("POS0:") || cmd.startsWith("POS1:")) {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }
    int motor_id = (cmd.charAt(3) == '0') ? 0 : 1;
    float pos = cmd.substring(5).toFloat();

    // Safety check: verify we have feedback for this motor
    if (!motor_states[motor_id].received_feedback) {
      Serial.print("ERROR:No feedback for motor ");
      Serial.println(motor_id);
      return;
    }

    float current_pos = motor_states[motor_id].last_feedback.Pos_Estimate;
    float delta = pos - current_pos;

    // Position deadband
    const float POSITION_DEADBAND = 0.05;
    if (abs(delta) < POSITION_DEADBAND) {
      Serial.print("NEAR_TARGET:");
      Serial.println(current_pos, 3);
      return;
    }

    // Limit max movement
    if (abs(delta) > 100.0) {
      Serial.print("ERROR:Position change too large: ");
      Serial.print(delta, 3);
      Serial.println(" turns");
      return;
    }

    odrives[motor_id]->setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER);
    delay(10);
    odrives[motor_id]->setPosition(pos, 0, 0);

    Serial.print("OK:MOTOR");
    Serial.println(motor_id);
  }
  else if (cmd.startsWith("ENABLE0") || cmd.startsWith("ENABLE1")) {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }
    int motor_id = (cmd.charAt(6) == '0') ? 0 : 1;
    ODriveCAN* odrv = odrives[motor_id];

    odrv->clearErrors();
    delay(10);
    odrv->setState(ODriveAxisState::AXIS_STATE_MOTOR_CALIBRATION);
    delay(6000);  // Wait for calibration
    odrv->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    Serial.print("OK:MOTOR");
    Serial.println(motor_id);
  }
  else if (cmd.startsWith("DISABLE0") || cmd.startsWith("DISABLE1")) {
    int motor_id = (cmd.charAt(7) == '0') ? 0 : 1;
    odrives[motor_id]->setState(ODriveAxisState::AXIS_STATE_IDLE);

    Serial.print("OK:MOTOR");
    Serial.println(motor_id);
  }
  else if (cmd.startsWith("SETALL:")) {
    // Coordinated position control: SETALL:<j0>,<j1>
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }

    float positions[NUM_MOTORS];
    if (parseFloatArray(cmd.substring(7), positions, NUM_MOTORS)) {
      // Set all motors to position control mode
      for (int i = 0; i < NUM_MOTORS; i++) {
        odrives[i]->setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER);
        delay(5);
        odrives[i]->setPosition(positions[i], 0, 0);
      }
      Serial.println("OK:ALL");
    } else {
      Serial.println("ERROR:Invalid SETALL format");
    }
  }
  else if (cmd == "STOPALL") {
    // Emergency stop all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
      odrives[i]->setVelocity(0);
    }
    Serial.println("OK:ALL");
  }
  // Legacy single-motor commands (backward compatible - operate on motor 0)
  else if (cmd.startsWith("VEL:")) {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }
    float vel = cmd.substring(4).toFloat();
    // Switch to velocity control mode
    odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    delay(10);  // Allow mode switch to settle
    odrv0.setVelocity(vel);
    Serial.println("OK");
  }
  else if (cmd == "STOP") {
    odrv0.setVelocity(0);
    Serial.println("OK");
  }
  else if (cmd.startsWith("POS:")) {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active");
      return;
    }
    float pos = cmd.substring(4).toFloat();

    // Safety: Limit position commands to small movements (legacy - uses motor 0)
    if (!motor_states[0].received_feedback) {
      Serial.println("ERROR:No feedback");
      return;
    }

    float current_pos = motor_states[0].last_feedback.Pos_Estimate;
    float delta = pos - current_pos;

    // Position deadband to prevent oscillation from repeated commands
    const float POSITION_DEADBAND = 0.05;  // 0.05 turns = ~18 degrees
    if (abs(delta) < POSITION_DEADBAND) {
      Serial.print("NEAR_TARGET:");
      Serial.println(current_pos, 3);
      return;  // Already close enough, don't send new command
    }

    // Only allow moves <= 100 turns at a time (temporary - for testing)
    // TODO: Reduce to 2.0 turns once position control is verified working
    if (abs(delta) > 100.0) {
      Serial.print("ERROR:Position change too large: ");
      Serial.print(delta, 3);
      Serial.println(" turns (max 100.0)");
      return;
    }

    // Switch to position control mode with filtered input (smoother motion)
    odrv0.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER);
    delay(10);  // Allow mode switch to settle

    // Send position command (position, velocity_feedforward, torque_feedforward)
    odrv0.setPosition(pos, 0, 0);
    Serial.println("OK");
  }
  else if (cmd.startsWith("RAMPENABLE:")) {
    // Enable/disable velocity ramping: RAMPENABLE:<0|1>
    // Note: s1_web_control.ino currently doesn't implement velocity ramping
    // This command is acknowledged for UI compatibility but doesn't affect behavior
    // Velocity ramping would need to be implemented in the main loop
    int enable = cmd.substring(11).toInt();
    Serial.println("OK");
    // TODO: Implement actual velocity ramping if needed
  }
  else if (cmd.startsWith("SETMODE:")) {
    // Set ODrive control mode and input mode
    // Format: SETMODE:<control_mode>,<input_mode>
    // Example: SETMODE:3,3 (position control with POS_FILTER)
    String params = cmd.substring(8);
    int commaIndex = params.indexOf(',');

    if (commaIndex > 0) {
      uint8_t ctrl_mode = params.substring(0, commaIndex).toInt();
      uint8_t inp_mode = params.substring(commaIndex + 1).toInt();

      // Validate mode values (ODrive S1 standard ranges)
      // Control modes: 0=IDLE, 1=CLOSED_LOOP, 2=VELOCITY, 3=POSITION
      // Input modes: 0-8 various input modes
      if (ctrl_mode <= 3 && inp_mode <= 8) {
        // Apply mode change to ODrive
        odrv0.setControllerMode(ctrl_mode, inp_mode);
        delay(10);  // Allow mode switch to settle
        Serial.println("OK");
      } else {
        Serial.println("ERROR:Invalid mode values");
      }
    } else {
      Serial.println("ERROR:Invalid SETMODE format");
    }
  }
  else if (cmd == "GETPOS") {
    // Legacy command - returns motor 0 position
    if (motor_states[0].received_feedback) {
      Serial.print("POS:");
      Serial.print(motor_states[0].last_feedback.Pos_Estimate, 3);
      Serial.print(",VEL:");
      Serial.println(motor_states[0].last_feedback.Vel_Estimate, 3);
    } else {
      Serial.println("ERROR:No feedback");
    }
  }
  else if (cmd == "ENABLE") {
    if (!canSendMotorCommand()) {
      Serial.println("ERROR:Battery protection active - charge battery above 20.9V");
      return;
    }
    // Enable closed loop control
    odrv0.clearErrors();
    delay(10);
    odrv0.setState(ODriveAxisState::AXIS_STATE_MOTOR_CALIBRATION);

    // Wait for motor calibration
    delay(6000);

    // Enter closed loop
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.println("OK");
  }
  else if (cmd == "DISABLE") {
    odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
    Serial.println("OK");
  }
  else if (cmd.startsWith("SETRAMP:")) {
    // Web interface sends velocity ramp settings
    // ODrive handles ramping internally, so just acknowledge
    Serial.println("OK");
  }
  else if (cmd == "CONFIG") {
    // Return basic configuration info
    Serial.println("OK:ODrive S1 Dual Motor, Nodes 1+2");
  }
  else if (cmd.length() == 0) {
    // Ignore empty commands
    return;
  }
  else {
    // Only print error for truly unknown commands
    Serial.print("ERROR:Unknown command: ");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("=== ODrive S1 Dual Motor Web Control v2.1 ===");

  // Register callbacks for both motors
  odrv0.onFeedback(onFeedback, &motor_states[0]);
  odrv0.onStatus(onHeartbeat, &motor_states[0]);
  odrv1.onFeedback(onFeedback, &motor_states[1]);
  odrv1.onStatus(onHeartbeat, &motor_states[1]);

  if (!setupCan()) {
    Serial.println("ERROR:CAN failed to initialize");
    while (true);
  }

  Serial.println("CAN initialized");

  // Wait for both ODrives heartbeat
  Serial.println("Waiting for ODrives...");
  unsigned long start = millis();
  bool allConnected = false;

  while (!allConnected && millis() - start < 5000) {
    pumpEvents(can_intf);
    delay(50);

    allConnected = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!motor_states[i].received_heartbeat) {
        allConnected = false;
        break;
      }
    }
  }

  // Report connection status
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_states[i].received_heartbeat) {
      Serial.print("ODrive ");
      Serial.print(i);
      Serial.print(" found - State: ");
      Serial.println(motor_states[i].last_heartbeat.Axis_State);
    } else {
      Serial.print("WARNING: ODrive ");
      Serial.print(i);
      Serial.println(" not found");
    }
  }

  if (motor_states[0].received_heartbeat) {

    // Check initial battery voltage
    Get_Bus_Voltage_Current_msg_t vbus;
    if (odrv0.request(vbus, 1)) {
      Serial.print("DC Bus Voltage: ");
      Serial.print(vbus.Bus_Voltage, 2);
      Serial.println("V");

      // Halt if voltage too low for safe operation
      if (vbus.Bus_Voltage < VOLTAGE_SHUTDOWN) {
        Serial.println("FATAL:Battery voltage too low for operation");
        Serial.print("  Current: ");
        Serial.print(vbus.Bus_Voltage, 2);
        Serial.println("V");
        Serial.print("  Required: >");
        Serial.print(VOLTAGE_RECOVERY, 1);
        Serial.println("V");
        Serial.println("Charge battery and restart");
        while (true) {
          delay(1000);  // Halt system
        }
      }

      lastVoltageReading = vbus.Bus_Voltage;

      // Set initial battery state based on voltage
      if (vbus.Bus_Voltage >= VOLTAGE_EARLY) {
        batteryState = BATTERY_NORMAL;
        Serial.println("Battery: NORMAL");
      } else if (vbus.Bus_Voltage >= VOLTAGE_URGENT) {
        batteryState = BATTERY_WARNING_EARLY;
        Serial.println("Battery: WARNING_EARLY");
      } else if (vbus.Bus_Voltage >= VOLTAGE_SHUTDOWN) {
        batteryState = BATTERY_WARNING_URGENT;
        Serial.println("Battery: WARNING_URGENT");
      }
    } else {
      Serial.println("WARNING: Could not read bus voltage");
    }
  } else {
    Serial.println("WARNING: ODrive not found");
  }

  Serial.println("READY");
  Serial.println("Send 'ENABLE' command to start motor control");
}

void loop() {
  pumpEvents(can_intf);

  // Check battery voltage continuously
  checkBatteryVoltage();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Send feedback every 100ms - Multi-motor format: FEEDBACK:<j0_p>,<j0_v>;<j1_p>,<j1_v>
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime > 100) {
    lastFeedbackTime = millis();

    Serial.print("FEEDBACK:");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (motor_states[i].received_feedback) {
        Serial.print(motor_states[i].last_feedback.Pos_Estimate, 3);
        Serial.print(",");
        Serial.print(motor_states[i].last_feedback.Vel_Estimate, 3);
      } else {
        Serial.print("0.000,0.000");
      }

      if (i < NUM_MOTORS - 1) {
        Serial.print(";");  // Semicolon separator between motors
      }
    }
    Serial.println();
  }

  // Send per-motor health data every 2 seconds - Format: HEALTH_M0:... and HEALTH_M1:...
  static unsigned long lastHealthUpdate = 0;
  const unsigned long HEALTH_UPDATE_INTERVAL = 2000;  // 2 seconds

  if (millis() - lastHealthUpdate > HEALTH_UPDATE_INTERVAL) {
    lastHealthUpdate = millis();

    // Send health for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!motor_states[i].received_heartbeat) continue;

      Get_Bus_Voltage_Current_msg_t vbus;
      Get_Temperature_msg_t temp;
      Get_Iq_msg_t current;

      // Only motor 0 reports voltage/current (shared bus)
      if (i == 0) {
        if (!odrives[i]->request(vbus, 1)) continue;
      }
      if (!odrives[i]->request(temp, 1)) continue;
      if (!odrives[i]->request(current, 1)) continue;

      // Format: HEALTH_M<id>:<state>,<errors>,<vbus>,<ibus>,<t_fet>,<t_motor>,<iq_meas>,<iq_set>,<comm_age>,<comm_ok>,<ctrl_mode>,<input_mode>
      Serial.print("HEALTH_M");
      Serial.print(i);
      Serial.print(":");
      Serial.print(motor_states[i].last_heartbeat.Axis_State);
      Serial.print(",");
      Serial.print(motor_states[i].last_heartbeat.Axis_Error);
      Serial.print(",");

      // Voltage/current only from motor 0 (shared), others send 0
      if (i == 0) {
        Serial.print(vbus.Bus_Voltage, 2);
        Serial.print(",");
        Serial.print(vbus.Bus_Current, 2);
      } else {
        Serial.print("0.00,0.00");
      }
      Serial.print(",");

      Serial.print(temp.FET_Temperature, 2);
      Serial.print(",");
      Serial.print(temp.Motor_Temperature, 2);
      Serial.print(",");
      Serial.print(current.Iq_Measured, 2);
      Serial.print(",");
      Serial.print(current.Iq_Setpoint, 2);
      Serial.print(",");
      Serial.print("0,1,2,2");  // comm_age, comm_ok, ctrl_mode, input_mode
      Serial.println();
    }
  }

  delay(10);
}
