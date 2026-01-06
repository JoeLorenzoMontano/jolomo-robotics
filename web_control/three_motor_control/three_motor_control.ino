#include <Arduino.h>
#include "ODriveCAN.h"

// Multi-Motor Serial command protocol
// Commands:
//   SETALL:<j0>,<j1>,<j2> - Set all joint positions
//   GETALL                - Query all joint states
//   STOPALL               - Emergency stop all motors
//   VEL:<val>             - Set velocity (joint 0 only, backward compat)
//   POS:<val>             - Set position (joint 0 only, backward compat)
//
// Responses:
//   FEEDBACK:<j0_p>,<j0_v>;<j1_p>,<j1_v>;<j2_p>,<j2_v>  - All joint feedback (100ms interval)
//   OK:ALL                - Batch command accepted
//   OK                    - Single command accepted
//   ERROR:<msg>           - Error message

/* Configuration -----------------------------------------------------------*/

#define NUM_JOINTS 3
#define CAN_BAUDRATE 250000

// ODrive node IDs for each joint
const uint8_t ODRIVE_NODE_IDS[NUM_JOINTS] = {1, 2, 3};

#define IS_ARDUINO_BUILTIN // Arduino Uno R4 Minima with built-in CAN

/* Board-specific includes -------------------------------------------------*/

#if defined(IS_ARDUINO_BUILTIN)
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif

/* Board-specific settings -------------------------------------------------*/

#ifdef IS_ARDUINO_BUILTIN
HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}
#endif

/* Multi-Motor Setup -------------------------------------------------------*/

// Instantiate ODrive objects for each joint
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRIVE_NODE_IDS[0]);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRIVE_NODE_IDS[1]);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRIVE_NODE_IDS[2]);

// Array of pointers to ODrive objects
ODriveCAN* odrives[NUM_JOINTS] = {&odrv0, &odrv1, &odrv2};

// Joint state structure
struct JointState {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Per-joint state tracking
JointState joint_states[NUM_JOINTS];

/* Callbacks ---------------------------------------------------------------*/

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  JointState* state = static_cast<JointState*>(user_data);
  state->last_heartbeat = msg;
  state->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  JointState* state = static_cast<JointState*>(user_data);
  state->last_feedback = msg;
  state->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  // Route CAN messages to all ODrive instances
  for (int i = 0; i < NUM_JOINTS; i++) {
    onReceive(msg, *odrives[i]);
  }
}

/* Command Processing ------------------------------------------------------*/

// Helper: Parse comma-separated float array
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

// Process serial commands
void processCommand(String cmd) {
  cmd.trim();

  // Multi-motor commands
  if (cmd.startsWith("SETALL:")) {
    // Parse: SETALL:<j0>,<j1>,<j2>
    float positions[NUM_JOINTS];
    String posStr = cmd.substring(7);

    if (parseFloatArray(posStr, positions, NUM_JOINTS)) {
      // Send position commands to all motors
      for (int i = 0; i < NUM_JOINTS; i++) {
        odrives[i]->setPosition(positions[i], 0);
      }
      Serial.println("OK:ALL");
    } else {
      Serial.println("ERROR:Parse failed");
    }
  }
  else if (cmd == "GETALL") {
    // Return all joint feedback
    Serial.print("FEEDBACK:");
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (joint_states[i].received_feedback) {
        Serial.print(joint_states[i].last_feedback.Pos_Estimate, 3);
        Serial.print(",");
        Serial.print(joint_states[i].last_feedback.Vel_Estimate, 3);
      } else {
        Serial.print("0.000,0.000");
      }

      if (i < NUM_JOINTS - 1) {
        Serial.print(";");
      }
    }
    Serial.println();
  }
  else if (cmd == "STOPALL") {
    // Emergency stop all motors
    for (int i = 0; i < NUM_JOINTS; i++) {
      odrives[i]->setVelocity(0);
    }
    Serial.println("OK:ALL");
  }

  // Backward-compatible single-motor commands (joint 0 only)
  else if (cmd.startsWith("VEL:")) {
    float vel = cmd.substring(4).toFloat();
    odrives[0]->setVelocity(vel);
    Serial.println("OK");
  }
  else if (cmd == "STOP") {
    odrives[0]->setVelocity(0);
    Serial.println("OK");
  }
  else if (cmd.startsWith("POS:")) {
    float pos = cmd.substring(4).toFloat();
    odrives[0]->setPosition(pos, 0);
    Serial.println("OK");
  }
  else if (cmd == "GETPOS") {
    if (joint_states[0].received_feedback) {
      Serial.print("POS:");
      Serial.print(joint_states[0].last_feedback.Pos_Estimate, 3);
      Serial.print(",VEL:");
      Serial.println(joint_states[0].last_feedback.Vel_Estimate, 3);
    } else {
      Serial.println("ERROR:No feedback");
    }
  }
  else if (cmd == "CONFIG") {
    // Print bus voltage/current (from joint 0)
    Get_Bus_Voltage_Current_msg_t vbus;
    if (odrives[0]->request(vbus, 1)) {
      Serial.print("VBUS:");
      Serial.print(vbus.Bus_Voltage, 2);
      Serial.print(",IBUS:");
      Serial.println(vbus.Bus_Current, 2);
    } else {
      Serial.println("ERROR:Config request failed");
    }
  }
  else if (cmd == "STATUS") {
    // Get axis states for all joints
    Serial.print("STATES:");
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (joint_states[i].received_heartbeat) {
        Serial.print(joint_states[i].last_heartbeat.Axis_State);
      } else {
        Serial.print("0");
      }
      if (i < NUM_JOINTS - 1) Serial.print(",");
    }
    Serial.println();
  }
  else {
    Serial.println("ERROR:Unknown command");
  }
}

/* Setup -------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);

  // Wait for serial connection
  while (!Serial && millis() < 3000);

  Serial.println("Three-Motor Control - 3-DOF Arm with IK");
  Serial.print("Number of joints: ");
  Serial.println(NUM_JOINTS);

  // Register callbacks for all motors
  for (int i = 0; i < NUM_JOINTS; i++) {
    odrives[i]->onFeedback(onFeedback, &joint_states[i]);
    odrives[i]->onStatus(onHeartbeat, &joint_states[i]);
  }

  // Configure and initialize CAN bus
  if (!setupCan()) {
    Serial.println("ERROR:CAN failed to initialize");
    while (true); // Halt
  }

  Serial.println("Waiting for ODrives...");

  // Wait for heartbeat from all motors
  unsigned long startTime = millis();
  bool allConnected = false;

  while (!allConnected && (millis() - startTime < 5000)) {
    pumpEvents(can_intf);

    allConnected = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (!joint_states[i].received_heartbeat) {
        allConnected = false;
        break;
      }
    }

    delay(100);
  }

  if (!allConnected) {
    Serial.println("ERROR:Not all ODrives found");
    for (int i = 0; i < NUM_JOINTS; i++) {
      Serial.print("Joint ");
      Serial.print(i);
      Serial.print(" (node ");
      Serial.print(ODRIVE_NODE_IDS[i]);
      Serial.print("): ");
      Serial.println(joint_states[i].received_heartbeat ? "OK" : "MISSING");
    }
  } else {
    Serial.println("All ODrives found");
  }

  // Request bus voltage (from joint 0)
  Get_Bus_Voltage_Current_msg_t vbus;
  if (odrives[0]->request(vbus, 1)) {
    Serial.print("VBUS:");
    Serial.println(vbus.Bus_Voltage, 2);
  }

  // Enable closed loop control on all motors
  Serial.println("Enabling closed loop control...");
  for (int i = 0; i < NUM_JOINTS; i++) {
    odrives[i]->clearErrors();
    delay(1);
    odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events to handle responses
    for (int j = 0; j < 15; j++) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("READY");
}

/* Main Loop ---------------------------------------------------------------*/

void loop() {
  // Handle CAN messages
  pumpEvents(can_intf);

  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Send periodic feedback (every 100ms = 10Hz)
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime > 100) {
    lastFeedbackTime = millis();

    // Send aggregated feedback for all joints
    Serial.print("FEEDBACK:");
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (joint_states[i].received_feedback) {
        Serial.print(joint_states[i].last_feedback.Pos_Estimate, 3);
        Serial.print(",");
        Serial.print(joint_states[i].last_feedback.Vel_Estimate, 3);
      } else {
        Serial.print("0.000,0.000");
      }

      if (i < NUM_JOINTS - 1) {
        Serial.print(";");
      }
    }
    Serial.println();
  }

  delay(10);
}
