#include <Arduino.h>
#include "ODriveCAN.h"
#include <RC_Receiver.h>

//Create an instance of a receiver
//You can put up to 8 channels pin with one receiver instance
//ex : RC_Receiver receiver('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8');
RC_Receiver receiver(9, 10, 11, 12);

//Channel min and max value
//Use the RC_raw script to get the min max val by moving your joystick up and down
//This need to be combined with the receiver.setMinMax(minMax); call
//Leave the default value for the un used channels
//First val is the min and the second is the max
//Invert the min and max val to reverse
int minMax[8][2] = 
{
	{-100,100}, 
	{-100,100}, 
	{-100,100}, 
	{-100,100}, 
	{-100,100}, 
	{-100,100}, 
	{-100,100}, 
	{-100,100}
};

// Joystick control parameters
#define RIGHT_JOYSTICK_Y_CHANNEL 3  // Assuming channel 3 is the right joystick Y-axis
#define MAX_VELOCITY 10.0           // Maximum velocity in turns/sec
#define DEADZONE 10                 // Joystick deadzone (-10 to 10 considered as zero)

// Commands for retrieving ODrive configuration
#define CMD_SAVE_CONFIG 0            // Print current config and save to Serial output
#define CMD_RESET_VELOCITY 1         // Reset to default velocity (hold position)
#define COMMAND_CHANNEL 4            // Use channel 4 for command input

// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

/* Configuration of example sketch -------------------------------------------*/

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 1

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_ARDUINO_BUILTIN)
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif


/* Example sketch ------------------------------------------------------------*/

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
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

void setup() {
  Serial.begin(115200);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  receiver.setMinMax(minMax); //set the min and max value for each channels
  Serial.println("Receiver Ready");

  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive running!");
  
  // Print ODrive configuration
  printODriveConfig();
}

void printODriveConfig() {
  Serial.println("\n----- ODrive Configuration -----");
  
  // Fetch bus voltage and current
  Get_Bus_Voltage_Current_msg_t vbus;
  if (odrv0.request(vbus, 1)) {
    Serial.println("Bus Measurements:");
    Serial.print("  DC voltage [V]: "); Serial.println(vbus.Bus_Voltage);
    Serial.print("  DC current [A]: "); Serial.println(vbus.Bus_Current);
  } else {
    Serial.println("Failed to get bus voltage and current");
  }
  
  // Fetch motor current
  Get_Iq_setpoint_measured_msg_t current;
  if (odrv0.request(current, 1)) {
    Serial.println("Motor Current:");
    Serial.print("  Iq setpoint [A]: "); Serial.println(current.Iq_setpoint);  
    Serial.print("  Iq measured [A]: "); Serial.println(current.Iq_measured);
  } else {
    Serial.println("Failed to get motor current");
  }
  
  // Fetch encoder estimates
  Get_Encoder_Estimates_msg_t encoder;
  if (odrv0.request(encoder, 1)) {
    Serial.println("Encoder Estimates:");
    Serial.print("  Position: "); Serial.println(encoder.Pos_Estimate);
    Serial.print("  Velocity: "); Serial.println(encoder.Vel_Estimate);
  } else {
    Serial.println("Failed to get encoder estimates");
  }
  
  // Fetch error states
  Get_Error_msg_t errors;
  if (odrv0.request(errors, 1)) {
    Serial.println("Error States:");
    Serial.print("  Active Errors: 0x"); Serial.println(errors.Active_Errors, HEX);
    Serial.print("  Disarm Reason: 0x"); Serial.println(errors.Disarm_Reason, HEX);
  } else {
    Serial.println("Failed to get error states");
  }
  
  // Get temperature
  Get_Temperature_msg_t temp;
  if (odrv0.request(temp, 1)) {
    Serial.println("Temperature:");
    Serial.print("  Inverter Temperature [°C]: "); Serial.println(temp.Inverter_Temp);
    Serial.print("  Motor Temperature [°C]: "); Serial.println(temp.Motor_Temp);
  } else {
    Serial.println("Failed to get temperature");
  }
  
  // Get controller status
  Heartbeat_msg_t status;
  if (odrv0.request(status, 1)) {
    Serial.println("Controller Status:");
    Serial.print("  Axis State: "); Serial.println(status.Axis_State);
    Serial.print("  Axis Flags: 0x"); Serial.println(status.Axis_Flags, HEX);
  } else {
    Serial.println("Failed to get controller status");
  }
  
  Serial.println("----- End Configuration -----\n");
}

void loop() {
  pumpEvents(can_intf); // This is required to handle incoming CAN messages
  
  // Get joystick input from right joystick (vertical axis)
  int joystickValue = receiver.getMap(RIGHT_JOYSTICK_Y_CHANNEL);
  
  // Get command channel value
  int commandValue = receiver.getMap(COMMAND_CHANNEL);
  
  // Check if command is being issued
  static int lastCommandValue = 0;
  static unsigned long lastCommandTime = 0;
  
  if (abs(commandValue - lastCommandValue) > 50 && (millis() - lastCommandTime) > 1000) {
    lastCommandTime = millis();
    lastCommandValue = commandValue;
    
    // Command near 100 = print config
    if (commandValue > 80) {
      Serial.println("Command received: Print configuration");
      printODriveConfig();
    }
    // Command near -100 = reset to position hold
    else if (commandValue < -80) {
      Serial.println("Command received: Reset to position hold");
      if (odrv0_user_data.received_feedback) {
        float currentPosition = odrv0_user_data.last_feedback.Pos_Estimate;
        odrv0.setPosition(currentPosition, 0);
        Serial.print("Holding position at: "); Serial.println(currentPosition);
      }
    }
  }
  
  // Apply deadzone to prevent jitter at center position
  if (abs(joystickValue) < DEADZONE) {
    joystickValue = 0;
  }
  
  // Calculate velocity based on joystick position
  // Map from -100 to 100 to -MAX_VELOCITY to MAX_VELOCITY
  float targetVelocity = (float)joystickValue / 100.0 * MAX_VELOCITY;
  
  // Maintain current position when joystick is centered
  if (joystickValue == 0) {
    // Get current position from encoder feedback
    if (odrv0_user_data.received_feedback) {
      float currentPosition = odrv0_user_data.last_feedback.Pos_Estimate;
      // Hold position with zero velocity
      odrv0.setPosition(currentPosition, 0);
    }
  } else {
    // Use velocity control mode when joystick is not centered
    odrv0.setVelocity(targetVelocity);
  }
  
  // Print debug information
  static unsigned long lastPrintTime = 0;
  if (odrv0_user_data.received_feedback && (millis() - lastPrintTime > 100)) {
    lastPrintTime = millis();
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    Serial.print("odrv0-pos:");
    Serial.print(feedback.Pos_Estimate);
    Serial.print(",");
    Serial.print("odrv0-vel:");
    Serial.print(feedback.Vel_Estimate);
    Serial.print(",");
    Serial.print("target-vel:");
    Serial.println(targetVelocity);
    
    // Print receiver values
    Serial.print("RC values: ");
    Serial.print(receiver.getMap(1));
    Serial.print("\t");  
    Serial.print(receiver.getMap(2));
    Serial.print("\t");  
    Serial.print(receiver.getMap(3));
    Serial.print("\t");  
    Serial.print(receiver.getMap(4));
    Serial.println();
  }
  
  delay(10); // Small delay to prevent spamming the CAN bus
}
