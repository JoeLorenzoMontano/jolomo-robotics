#include <Arduino.h>
#include "ODriveCAN.h"

// Serial command protocol for web control
// Commands: VEL:<value>, STOP, POS:<value>, GETPOS, CONFIG
// Responses: OK, POS:<pos>,VEL:<vel>, ERROR:<msg>

/* Configuration of example sketch -------------------------------------------*/

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 1

#define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_ARDUINO_BUILTIN)
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

/* Example sketch ------------------------------------------------------------*/

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = {&odrv0};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

// Process serial commands
void processCommand(String cmd) {
  cmd.trim(); // Remove whitespace

  if (cmd.startsWith("VEL:")) {
    // Set velocity command
    float vel = cmd.substring(4).toFloat();
    odrv0.setVelocity(vel);
    Serial.println("OK");
  }
  else if (cmd == "STOP") {
    // Stop motor
    odrv0.setVelocity(0);
    Serial.println("OK");
  }
  else if (cmd.startsWith("POS:")) {
    // Go to position
    float pos = cmd.substring(4).toFloat();
    odrv0.setPosition(pos, 0);
    Serial.println("OK");
  }
  else if (cmd == "GETPOS") {
    // Get current position and velocity
    if (odrv0_user_data.received_feedback) {
      Serial.print("POS:");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 3);
      Serial.print(",VEL:");
      Serial.println(odrv0_user_data.last_feedback.Vel_Estimate, 3);
    } else {
      Serial.println("ERROR:No feedback");
    }
  }
  else if (cmd == "CONFIG") {
    // Print configuration
    Get_Bus_Voltage_Current_msg_t vbus;
    if (odrv0.request(vbus, 1)) {
      Serial.print("VBUS:");
      Serial.print(vbus.Bus_Voltage, 2);
      Serial.print(",IBUS:");
      Serial.println(vbus.Bus_Current, 2);
    } else {
      Serial.println("ERROR:Config request failed");
    }
  }
  else if (cmd == "STATUS") {
    // Get axis state
    if (odrv0_user_data.received_heartbeat) {
      Serial.print("STATE:");
      Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
    } else {
      Serial.println("ERROR:No heartbeat");
    }
  }
  else {
    Serial.println("ERROR:Unknown command");
  }
}

void setup() {
  Serial.begin(115200);

  // Wait for serial connection
  while (!Serial && millis() < 3000);

  Serial.println("ODrive Web Control Ready");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface
  if (!setupCan()) {
    Serial.println("ERROR:CAN failed to initialize");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("ODrive found");

  // request bus voltage and current
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("ERROR:VBus request failed");
    while (true);
  }

  Serial.print("VBUS:");
  Serial.println(vbus.Bus_Voltage, 2);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("READY");
}

void loop() {
  pumpEvents(can_intf); // Handle incoming CAN messages

  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Send periodic feedback (every 100ms)
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime > 100) {
    lastFeedbackTime = millis();

    if (odrv0_user_data.received_feedback) {
      Serial.print("FEEDBACK:");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 3);
      Serial.print(",");
      Serial.println(odrv0_user_data.last_feedback.Vel_Estimate, 3);
      // Don't reset received_feedback - keep sending last known values
    }
  }

  delay(10);
}
