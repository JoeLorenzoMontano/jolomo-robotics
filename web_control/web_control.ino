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

// Velocity ramping state
struct VelocityRampState {
  bool enabled = true;
  float acceleration_limit = 5.0;  // turns/sec²
  float deceleration_limit = 5.0;  // turns/sec²
  float current_velocity = 0.0;
  float target_velocity = 0.0;
  unsigned long last_update_time = 0;
};

VelocityRampState ramp_state;

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

// Update velocity ramping
void updateVelocityRamp() {
  if (!ramp_state.enabled) {
    // Ramping disabled - use target velocity directly
    odrv0.setVelocity(ramp_state.target_velocity);
    ramp_state.current_velocity = ramp_state.target_velocity;
    return;
  }

  unsigned long current_time = millis();

  // Initialize on first call
  if (ramp_state.last_update_time == 0) {
    ramp_state.last_update_time = current_time;
    return;
  }

  // Calculate time delta in seconds
  float dt = (current_time - ramp_state.last_update_time) / 1000.0;
  ramp_state.last_update_time = current_time;

  // Skip if time delta is too large (> 100ms indicates missed cycles)
  if (dt > 0.1 || dt <= 0) {
    return;
  }

  // Calculate velocity error
  float error = ramp_state.target_velocity - ramp_state.current_velocity;

  // Choose acceleration or deceleration limit based on direction of change
  float accel_limit = (error > 0) ? ramp_state.acceleration_limit : ramp_state.deceleration_limit;

  // Maximum velocity change this timestep
  float max_delta = accel_limit * dt;

  // Apply ramp
  if (abs(error) <= max_delta) {
    // Snap to target if close enough
    ramp_state.current_velocity = ramp_state.target_velocity;
  } else {
    // Ramp towards target
    ramp_state.current_velocity += (error > 0) ? max_delta : -max_delta;
  }

  // Send ramped velocity to ODrive
  odrv0.setVelocity(ramp_state.current_velocity);
}

// Process serial commands
void processCommand(String cmd) {
  cmd.trim(); // Remove whitespace

  if (cmd.startsWith("VEL:")) {
    // Set velocity command (with ramping if enabled)
    float vel = cmd.substring(4).toFloat();
    ramp_state.target_velocity = vel;
    Serial.println("OK");
  }
  else if (cmd == "STOP") {
    // Stop motor (with ramping if enabled)
    ramp_state.target_velocity = 0;
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
  else if (cmd.startsWith("SETRAMP:")) {
    // Set velocity ramping parameters: SETRAMP:<accel>,<decel>
    String params = cmd.substring(8);
    int commaIndex = params.indexOf(',');
    if (commaIndex > 0) {
      float accel = params.substring(0, commaIndex).toFloat();
      float decel = params.substring(commaIndex + 1).toFloat();
      ramp_state.acceleration_limit = accel;
      ramp_state.deceleration_limit = decel;
      Serial.println("OK");
    } else {
      Serial.println("ERROR:Invalid SETRAMP format");
    }
  }
  else if (cmd.startsWith("RAMPENABLE:")) {
    // Enable/disable velocity ramping: RAMPENABLE:<0|1>
    int enable = cmd.substring(11).toInt();
    ramp_state.enabled = (enable != 0);
    if (!ramp_state.enabled) {
      // Reset ramping state when disabled
      ramp_state.current_velocity = ramp_state.target_velocity;
    }
    Serial.println("OK");
  }
  else if (cmd == "GETRAMP") {
    // Get current velocity ramping parameters
    Serial.print("RAMP:");
    Serial.print(ramp_state.enabled ? "1" : "0");
    Serial.print(",");
    Serial.print(ramp_state.acceleration_limit, 1);
    Serial.print(",");
    Serial.println(ramp_state.deceleration_limit, 1);
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

  // Update velocity ramping
  updateVelocityRamp();

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
