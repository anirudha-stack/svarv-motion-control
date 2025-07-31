/**
 * @file Current_Sensor_Test.ino
 * @brief Simple current sensor test - motor holds position, display current readings
 * @author Svarv Robotics
 * 
 * This example sets the motor to hold position 0 and displays current consumption.
 * User can manually move the motor to see current changes.
 */

#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor* motor = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Current Sensor Test");
  Serial.println("==================");
  
  // Initialize CAN
  if (!svarv.begin(1000000)) {
    Serial.println("CAN init failed!");
    while (1) delay(1000);
  }
  
  // Find motor
  auto discovered = svarv.scanForMotors(3000);
  if (discovered.empty()) {
    Serial.println("No motors found!");
    while (1) delay(1000);
  }
  
  motor = &svarv.addMotor(discovered[0]);
  Serial.println("Motor found: " + String(motor->getNodeId()));
  
  // Wait for connection
  while (!motor->isConnected()) {
    svarv.update();
    delay(100);
  }
  
  // Set position mode with gentle PID
  motor->setControlMode(SVARV_MODE_POSITION);
  motor->setPID(SVARV_PID_POSITION, 10.0, 0.0, 0.0);  // Very gentle P-only
  motor->setLimits(5.0, 1.0, 5.0);  // Low current limit
  
  // Hold at position 0
  motor->moveTo(0.0);
  
  Serial.println("Motor holding at 0 position");
  Serial.println("Manually move motor to see current");
  Serial.println("Format: Position(rad) | Current_Q(A) | Current_D(A) | Current_Total(A)");
  Serial.println("Commands: 's X' = set scaling factor");
}

void loop() {
  svarv.update();
  
  // Handle scaling command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("s ")) {
      float scale = cmd.substring(2).toFloat();
      Serial.println("Scaling factor set to: " + String(scale));
      // Note: This is just for display, actual scaling would need motor controller support
    }
  }
  
  // Display current every 200ms
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200) {
    lastDisplay = millis();
    
    if (motor->isConnected()) {
      const auto& status = motor->getStatus();
      
      float current_total = sqrt(status.current_q * status.current_q + 
                               status.current_d * status.current_d);
      
      Serial.print("Pos: ");
      Serial.print(status.position, 3);
      Serial.print(" | I_q: ");
      Serial.print(status.current_q, 3);
      Serial.print(" | I_d: ");
      Serial.print(status.current_d, 3);
      Serial.print(" | I_total: ");
      Serial.print(current_total, 3);
      Serial.print(" | Enabled: ");
      Serial.println(status.enabled ? "Yes" : "No");
      
      if (status.error_code != SVARV_ERROR_NONE) {
        Serial.println("ERROR: " + motor->getErrorString());
      }
    }
  }
}