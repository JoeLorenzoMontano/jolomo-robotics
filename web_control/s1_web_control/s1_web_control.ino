#include <Arduino.h>
#include "ODriveCAN.h"
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

// ODrive S1 Web Control - Fresh Start
// Commands: VEL:<value>, POS:<value>, STOP, GETPOS, ENABLE, DISABLE

#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 1

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = {&odrv0};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("VEL:")) {
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
    float pos = cmd.substring(4).toFloat();

    // Safety: Limit position commands to small movements
    if (!odrv0_user_data.received_feedback) {
      Serial.println("ERROR:No feedback");
      return;
    }

    float current_pos = odrv0_user_data.last_feedback.Pos_Estimate;
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
    if (odrv0_user_data.received_feedback) {
      Serial.print("POS:");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 3);
      Serial.print(",VEL:");
      Serial.println(odrv0_user_data.last_feedback.Vel_Estimate, 3);
    } else {
      Serial.println("ERROR:No feedback");
    }
  }
  else if (cmd == "ENABLE") {
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
    Serial.println("OK:ODrive S1, Node 1");
  }
  else if (cmd.startsWith("SETALL:")) {
    // Multi-motor command (not supported with single motor)
    Serial.println("ERROR:Single motor only");
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

  Serial.println("=== ODrive S1 Web Control v2.0 - UPDATED 2026-01-13 ===");

  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  if (!setupCan()) {
    Serial.println("ERROR:CAN failed to initialize");
    while (true);
  }

  Serial.println("CAN initialized");

  // Wait for ODrive heartbeat
  Serial.println("Waiting for ODrive...");
  unsigned long start = millis();
  while (!odrv0_user_data.received_heartbeat && millis() - start < 5000) {
    pumpEvents(can_intf);
    delay(50);
  }

  if (odrv0_user_data.received_heartbeat) {
    Serial.println("ODrive found");
    Serial.print("Axis State: ");
    Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
  } else {
    Serial.println("WARNING: ODrive not found");
  }

  Serial.println("READY");
  Serial.println("Send 'ENABLE' command to start motor control");
}

void loop() {
  pumpEvents(can_intf);

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Send feedback every 100ms
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime > 100) {
    lastFeedbackTime = millis();

    if (odrv0_user_data.received_feedback) {
      Serial.print("FEEDBACK:");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 3);
      Serial.print(",");
      Serial.println(odrv0_user_data.last_feedback.Vel_Estimate, 3);
    }
  }

  // Send health data every 2 seconds
  static unsigned long lastHealthUpdate = 0;
  const unsigned long HEALTH_UPDATE_INTERVAL = 2000;  // 2 seconds

  if (millis() - lastHealthUpdate > HEALTH_UPDATE_INTERVAL) {
    lastHealthUpdate = millis();

    if (odrv0_user_data.received_heartbeat) {
      // Request bus voltage/current
      Get_Bus_Voltage_Current_msg_t vbus;
      if (!odrv0.request(vbus, 1)) {
        return;  // Skip this health update if voltage request fails
      }

      // Request temperature data
      Get_Temperature_msg_t temp;
      if (!odrv0.request(temp, 1)) {
        return;  // Skip this health update if temperature request fails
      }

      // Request motor current (Iq)
      Get_Iq_msg_t current;
      if (!odrv0.request(current, 1)) {
        return;  // Skip this health update if current request fails
      }

      // Send HEALTH message: <state>,<errors>,<vbus>,<ibus>,<t_fet>,<t_motor>,<iq_meas>,<iq_set>,<comm_age>,<comm_ok>,<ctrl_mode>,<input_mode>
      Serial.print("HEALTH:");
      Serial.print(odrv0_user_data.last_heartbeat.Axis_State);
      Serial.print(",");
      Serial.print(odrv0_user_data.last_heartbeat.Axis_Error);
      Serial.print(",");
      Serial.print(vbus.Bus_Voltage, 2);
      Serial.print(",");
      Serial.print(vbus.Bus_Current, 2);
      Serial.print(",");
      Serial.print(temp.FET_Temperature, 2);
      Serial.print(",");
      Serial.print(temp.Motor_Temperature, 2);
      Serial.print(",");
      Serial.print(current.Iq_Measured, 2);
      Serial.print(",");
      Serial.print(current.Iq_Setpoint, 2);
      Serial.print(",");

      Serial.print("0,1,");  // comm_age (0ms), comm_ok (true)

      // Add control mode and input mode (default to velocity control with vel_ramp for now)
      Serial.print("2,2");  // control_mode=2 (VELOCITY), input_mode=2 (VEL_RAMP)

      Serial.println();
    }
  }

  delay(10);
}
