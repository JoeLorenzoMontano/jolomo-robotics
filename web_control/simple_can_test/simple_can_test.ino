#include <Arduino.h>
#include "ODriveCAN.h"
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

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
  unsigned long heartbeat_count = 0;
};

ODriveUserData odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
  odrv_user_data->heartbeat_count++;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("=== Simple CAN Communication Test ===");

  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  if (!setupCan()) {
    Serial.println("ERROR: CAN init failed");
    while (true);
  }

  Serial.println("CAN initialized");
  Serial.println("Waiting for ODrive heartbeat...");

  unsigned long start = millis();
  while (!odrv0_user_data.received_heartbeat && millis() - start < 5000) {
    pumpEvents(can_intf);
    delay(50);
  }

  if (odrv0_user_data.received_heartbeat) {
    Serial.println("✓ ODrive found via CAN!");
    Serial.print("  Axis State: ");
    Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
    Serial.print("  Axis Error: ");
    Serial.println(odrv0_user_data.last_heartbeat.Axis_Error);
  } else {
    Serial.println("✗ No ODrive heartbeat received");
  }

  Serial.println("\nREADY - Type commands:");
  Serial.println("  'v' - Request VBus voltage");
  Serial.println("  's' - Show ODrive status");
  Serial.println("  'h' - Show heartbeat count");
}

void loop() {
  pumpEvents(can_intf);

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'v') {
      Get_Bus_Voltage_Current_msg_t vbus;
      if (odrv0.request(vbus, 1)) {
        Serial.print("VBus: ");
        Serial.print(vbus.Bus_Voltage, 2);
        Serial.println("V");
      } else {
        Serial.println("VBus request failed");
      }
    }
    else if (cmd == 's') {
      Serial.print("State: ");
      Serial.print(odrv0_user_data.last_heartbeat.Axis_State);
      Serial.print(", Error: 0x");
      Serial.println(odrv0_user_data.last_heartbeat.Axis_Error, HEX);
    }
    else if (cmd == 'h') {
      Serial.print("Heartbeats received: ");
      Serial.println(odrv0_user_data.heartbeat_count);
    }
  }

  // Print status every 2 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 2000) {
    lastStatus = millis();
    Serial.print("Heartbeats: ");
    Serial.print(odrv0_user_data.heartbeat_count);
    Serial.print(", State: ");
    Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
  }

  delay(10);
}
