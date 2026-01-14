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

// Health monitoring state
struct ODriveHealth {
  // Communication metrics (tracked automatically)
  unsigned long last_heartbeat_time = 0;
  unsigned long last_feedback_time = 0;
  bool communication_healthy = false;

  // Polled telemetry
  float bus_voltage = 0.0;
  float bus_current = 0.0;
  float fet_temperature = 0.0;
  float motor_temperature = 0.0;
  float iq_measured = 0.0;
  float iq_setpoint = 0.0;

  // Error tracking
  uint32_t active_errors = 0;
  uint8_t axis_state = 0;

  // Statistics
  uint16_t can_success_count = 0;
  uint16_t can_timeout_count = 0;
};

struct ArduinoHealth {
  unsigned long loop_time_us = 0;
  unsigned long max_loop_time_us = 0;
  unsigned long loop_count = 0;
};

ODriveHealth odrv0_health;
ArduinoHealth arduino_health;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;

  // Update health tracking
  odrv0_health.last_heartbeat_time = millis();
  odrv0_health.active_errors = msg.Axis_Error;
  odrv0_health.axis_state = msg.Axis_State;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;

  // Update health tracking
  odrv0_health.last_feedback_time = millis();
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

// Poll ODrive health telemetry (called every 5 seconds)
void pollODriveHealth() {
  // Request bus voltage and current
  Get_Bus_Voltage_Current_msg_t vbus;
  if (odrv0.request(vbus, 10)) {
    odrv0_health.bus_voltage = vbus.Bus_Voltage;
    odrv0_health.bus_current = vbus.Bus_Current;
    odrv0_health.can_success_count++;
  } else {
    odrv0_health.can_timeout_count++;
  }

  // Request temperature data
  Get_Temperature_msg_t temp;
  if (odrv0.request(temp, 10)) {
    odrv0_health.fet_temperature = temp.FET_Temperature;
    odrv0_health.motor_temperature = temp.Motor_Temperature;
    odrv0_health.can_success_count++;
  } else {
    odrv0_health.can_timeout_count++;
  }

  // Request motor current
  Get_Iq_msg_t current;
  if (odrv0.request(current, 10)) {
    odrv0_health.iq_measured = current.Iq_Measured;
    odrv0_health.iq_setpoint = current.Iq_Setpoint;
    odrv0_health.can_success_count++;
  } else {
    odrv0_health.can_timeout_count++;
  }

  // Update communication health flag
  unsigned long now = millis();
  odrv0_health.communication_healthy =
    (now - odrv0_health.last_heartbeat_time < 1000) &&
    (now - odrv0_health.last_feedback_time < 1000);
}

// Send health data via serial
void sendHealthData() {
  // Format: HEALTH:<state>,<errors>,<vbus>,<ibus>,<t_fet>,<t_motor>,<iq_meas>,<iq_set>,<comm_age>,<comm_ok>
  unsigned long comm_age = millis() - odrv0_health.last_heartbeat_time;

  Serial.print("HEALTH:");
  Serial.print(odrv0_health.axis_state); Serial.print(",");
  Serial.print(odrv0_health.active_errors); Serial.print(",");
  Serial.print(odrv0_health.bus_voltage, 2); Serial.print(",");
  Serial.print(odrv0_health.bus_current, 2); Serial.print(",");
  Serial.print(odrv0_health.fet_temperature, 1); Serial.print(",");
  Serial.print(odrv0_health.motor_temperature, 1); Serial.print(",");
  Serial.print(odrv0_health.iq_measured, 2); Serial.print(",");
  Serial.print(odrv0_health.iq_setpoint, 2); Serial.print(",");
  Serial.print(comm_age); Serial.print(",");
  Serial.print(odrv0_health.communication_healthy ? "1" : "0");
  Serial.println();
}

// Send Arduino health data
void sendArduinoHealth() {
  // Format: ARDUINO_HEALTH:<loop_us>,<max_loop_us>,<can_success_rate>
  float success_rate = 100.0;
  uint16_t total = odrv0_health.can_success_count + odrv0_health.can_timeout_count;
  if (total > 0) {
    success_rate = (odrv0_health.can_success_count * 100.0) / total;
  }

  Serial.print("ARDUINO_HEALTH:");
  Serial.print(arduino_health.loop_time_us); Serial.print(",");
  Serial.print(arduino_health.max_loop_time_us); Serial.print(",");
  Serial.print(success_rate, 1);
  Serial.println();
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
  else if (cmd == "GETHEALTH") {
    // Get ODrive health data
    sendHealthData();
    Serial.println("OK");
  }
  else if (cmd == "GETARDUINOHEALTH") {
    // Get Arduino health data
    sendArduinoHealth();
    Serial.println("OK");
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
  unsigned long loop_start = micros();

  pumpEvents(can_intf); // Handle incoming CAN messages

  // Update velocity ramping
  updateVelocityRamp();

  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Health monitoring - poll every 5 seconds
  static unsigned long last_health_poll = 0;
  if (millis() - last_health_poll >= 5000) {
    pollODriveHealth();
    last_health_poll = millis();
  }

  // Send health update every 2 seconds
  static unsigned long last_health_send = 0;
  if (millis() - last_health_send >= 2000) {
    sendHealthData();
    sendArduinoHealth();
    last_health_send = millis();
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

  // Track loop timing
  unsigned long loop_time = micros() - loop_start;
  arduino_health.loop_time_us = loop_time;
  if (loop_time > arduino_health.max_loop_time_us) {
    arduino_health.max_loop_time_us = loop_time;
  }
  arduino_health.loop_count++;

  delay(10);
}
