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
	{99,265}, 
	{99,265}, 
	{99,265}, 
	{99,265}, 
	{99,265}, 
	{99,265}, 
	{99,265}, 
	{99,265}
};

// Joystick control parameters
#define RIGHT_JOYSTICK_Y_CHANNEL 3  // Assuming channel 3 is the right joystick Y-axis
#define MAX_VELOCITY 10.0           // Maximum velocity in turns/sec
#define DEADZONE 50                 // Joystick deadzone (center position noise threshold)

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
  
  // Get bus voltage and current - this is known to work
  Get_Bus_Voltage_Current_msg_t vbus;
  if (odrv0.request(vbus, 1)) {
    Serial.println("Bus Measurements:");
    Serial.print("  DC voltage [V]: "); Serial.println(vbus.Bus_Voltage);
    Serial.print("  DC current [A]: "); Serial.println(vbus.Bus_Current);
  } else {
    Serial.println("Failed to get bus voltage and current");
  }
  
  // Display current encoder position and velocity from feedback
  if (odrv0_user_data.received_feedback) {
    Serial.println("Encoder Estimates (from feedback):");
    Serial.print("  Position: "); Serial.println(odrv0_user_data.last_feedback.Pos_Estimate);
    Serial.print("  Velocity: "); Serial.println(odrv0_user_data.last_feedback.Vel_Estimate);
  }
  
  // Display current axis state from heartbeat
  if (odrv0_user_data.received_heartbeat) {
    Serial.println("Controller Status (from heartbeat):");
    Serial.print("  Axis State: "); Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
  }
  
  Serial.println("----- End Configuration -----\n");
}

void loop() {
  pumpEvents(can_intf); // This is required to handle incoming CAN messages
  
  // Print raw values first to debug
  Serial.print("Raw values: ");
  for (int i = 1; i <= 4; i++) {
    int rawValue = receiver.getRaw(i);
    Serial.print(rawValue);
    Serial.print("\t");
  }
  Serial.println();
  
  // Get joystick input from right joystick (vertical axis)
  int joystickValue = receiver.getRaw(RIGHT_JOYSTICK_Y_CHANNEL);
  
  // Get command channel value
  int commandValue = receiver.getRaw(COMMAND_CHANNEL);
  
  // Check if command is being issued
  static int lastCommandValue = 0;
  static unsigned long lastCommandTime = 0;
  
  // Reset code if nothing is being controlled
  if (millis() > 5000 && millis() < 6000) {
    // This will run once to show what the center values should be
    Serial.println("CALIBRATION VALUES:");
    Serial.print("Channel 1 center: "); Serial.println((receiver.getRaw(1) + 200) / 2);
    Serial.print("Channel 2 center: "); Serial.println((receiver.getRaw(2) + 200) / 2);
    Serial.print("Channel 3 center: "); Serial.println((receiver.getRaw(3) + 200) / 2);
    Serial.print("Channel 4 center: "); Serial.println((receiver.getRaw(4) + 200) / 2);
  }
  
  // Fixed center values based on your new output
  int centerValues[4] = {1500, 1495, 1480, 1500}; // Manually determined from your logs
  int joystickCenter = centerValues[RIGHT_JOYSTICK_Y_CHANNEL-1];
  int commandCenter = centerValues[COMMAND_CHANNEL-1];
  
  // Check for commands
  if (abs(commandValue - lastCommandValue) > 100 && (millis() - lastCommandTime) > 1000) {
    lastCommandTime = millis();
    lastCommandValue = commandValue;
    
    // Command far right = print config (> 1800)
    if (commandValue > 1800) {
      Serial.println("Command received: Print configuration");
      printODriveConfig();
    }
    // Command far left = reset to position hold (< 1000)
    else if (commandValue < 1000) {
      Serial.println("Command received: Reset to position hold");
      if (odrv0_user_data.received_feedback) {
        float currentPosition = odrv0_user_data.last_feedback.Pos_Estimate;
        odrv0.setPosition(currentPosition, 0);
        Serial.print("Holding position at: "); Serial.println(currentPosition);
      }
    }
  }
  
  // Apply deadzone to prevent jitter at center position
  if (abs(joystickValue - joystickCenter) < 50) {  // Wider deadzone for 1500-range values
    joystickValue = joystickCenter;
  }
  
  // Calculate velocity based on joystick position
  float maxRange = 500.0; // From center to extreme (1500±500 is typical range)
  float joystickOffset = joystickValue - joystickCenter;
  float targetVelocity = (joystickOffset / maxRange) * MAX_VELOCITY;
  
  // Maintain current position when joystick is centered
  if (abs(joystickValue - joystickCenter) < 50) {  // Match deadzone above
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
    
    // Print joystick info
    Serial.print("Joystick: value=");
    Serial.print(joystickValue);
    Serial.print(", offset=");
    Serial.print(joystickOffset);
    Serial.print(", deadzone=");
    Serial.print(abs(joystickValue - joystickCenter) < 15 ? "YES" : "NO");
    Serial.println();
  }
  
  delay(10); // Small delay to prevent spamming the CAN bus
}
