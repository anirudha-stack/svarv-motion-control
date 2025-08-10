/**
 * @file Physical_Limits_Example.ino
 * @brief Demonstrates physical position limits with Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example shows how to set up and use physical position limits to constrain
 * motor movement within safe boundaries.
 * 
 * Hardware Required:
 * - ESP32 with built-in CAN
 * - CAN transceiver
 * - Svarv BLDC Motor Controller
 * 
 * @section USAGE
 * 
 * The motor will be configured with position limits and then demonstrate:
 * - Setting minimum and maximum position limits
 * - Automatic position constraint
 * - Enabling/disabling limits
 * - Attempting to move beyond limits
 */

#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor motor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Svarv Motion Control - Physical Limits Example");
  Serial.println("==============================================");
  Serial.println();
  
  // Initialize CAN bus
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    while (1) delay(1000);
  }
  
  Serial.println("Scanning for motors...");
  auto discovered = svarv.scanForMotors(3000);
  
  if (discovered.empty()) {
    Serial.println("ERROR: No motors found!");
    while (1) delay(1000);
  }
  
  // Use first discovered motor
  motor = svarv.addMotor(discovered[0]);
  Serial.print("Connected to motor with Node ID: ");
  Serial.println(motor.getNodeId());
  
  // Wait for motor to be ready
  Serial.print("Waiting for motor initialization");
  while (!motor.isConnected() || motor.getStatus().initializing) {
    svarv.update();
    Serial.print(".");
    delay(100);
  }
  Serial.println(" Ready!");
  
  // Configure motor for position control
  motor.setControlMode(SVARV_MODE_POSITION);
  
  // Set up physical limits: -2π to +2π radians (4 full rotations range)
  float min_limit = -2.0 * PI;  // -2π radians
  float max_limit = 2.0 * PI;   // +2π radians
  
  Serial.println();
  Serial.println("=== SETTING PHYSICAL LIMITS ===");
  Serial.println("Min limit: " + String(min_limit, 3) + " rad (" + String(min_limit * 180.0 / PI, 1) + " degrees)");
  Serial.println("Max limit: " + String(max_limit, 3) + " rad (" + String(max_limit * 180.0 / PI, 1) + " degrees)");
  
  if (motor.setPositionLimits(min_limit, max_limit, true)) {
    Serial.println("✅ Position limits set successfully!");
    
    // Save configuration to motor's EEPROM
    if (motor.saveConfig()) {
      Serial.println("✅ Configuration saved to EEPROM");
    }
  } else {
    Serial.println("❌ Failed to set position limits");
  }
  
  // Home motor to center position
  Serial.println();
  Serial.println("Homing motor to center position (0 rad)...");
  motor.moveTo(0.0);
  delay(3000);
  
  Serial.println();
  Serial.println("Starting demonstration...");
  Serial.println("Watch how the motor constrains movement to within limits!");
  Serial.println();
}

void loop() {
  svarv.update();
  
  static unsigned long lastMoveTime = 0;
  static int testStep = 0;
  
  if (millis() - lastMoveTime > 5000) { // Every 5 seconds
    lastMoveTime = millis();
    
    switch (testStep) {
      case 0:
        Serial.println("=== TEST 1: Normal movement within limits ===");
        Serial.println("Moving to π radians (180 degrees)...");
        motor.moveTo(PI);
        break;
        
      case 1:
        Serial.println("Moving to -π radians (-180 degrees)...");
        motor.moveTo(-PI);
        break;
        
      case 2:
        Serial.println();
        Serial.println("=== TEST 2: Attempting to exceed limits ===");
        Serial.println("Trying to move to 4π radians (720 degrees)...");
        Serial.println("Should be constrained to 2π radians (360 degrees)");
        motor.moveTo(4.0 * PI); // Will be constrained to max limit
        break;
        
      case 3:
        Serial.println("Trying to move to -4π radians (-720 degrees)...");
        Serial.println("Should be constrained to -2π radians (-360 degrees)");
        motor.moveTo(-4.0 * PI); // Will be constrained to min limit
        break;
        
      case 4:
        Serial.println();
        Serial.println("=== TEST 3: Disabling limits ===");
        Serial.println("Disabling position limits...");
        if (motor.disablePositionLimits()) {
          Serial.println("✅ Position limits disabled");
        }
        break;
        
      case 5:
        Serial.println("Now trying to move to 3π radians (should work)...");
        motor.moveTo(3.0 * PI); // Should work with limits disabled
        break;
        
      case 6:
        Serial.println();
        Serial.println("=== TEST 4: Re-enabling limits ===");
        Serial.println("Re-enabling position limits...");
        if (motor.enablePositionLimits()) {
          Serial.println("✅ Position limits re-enabled");
        }
        break;
        
      case 7:
        Serial.println("Moving back to center (0 rad)...");
        motor.moveTo(0.0);
        break;
        
      case 8:
        Serial.println();
        Serial.println("=== DEMONSTRATION COMPLETE ===");
        Serial.println("Physical limits are now active and saved to EEPROM");
        Serial.println("The motor will retain these limits after power cycling");
        Serial.println();
        printLimitStatus();
        testStep = -1; // Will wrap to 0
        break;
    }
    
    testStep++;
  }
  
  // Print motor status every 2 seconds
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 2000) {
    lastStatusTime = millis();
    
    if (motor.isConnected()) {
      const auto& status = motor.getStatus();
      Serial.print("Motor Status: Pos=");
      Serial.print(status.position, 3);
      Serial.print(" rad (");
      Serial.print(status.position * 180.0 / PI, 1);
      Serial.print("°), Limits=");
      Serial.print(motor.arePositionLimitsEnabled() ? "ON" : "OFF");
      
      if (status.error_code != SVARV_ERROR_NONE) {
        Serial.print(", ERROR: ");
        Serial.print(motor.getErrorString());
      }
      
      Serial.println();
    }
  }
}

void printLimitStatus() {
  Serial.println("=== CURRENT LIMIT STATUS ===");
  Serial.println("Limits Enabled: " + String(motor.arePositionLimitsEnabled() ? "YES" : "NO"));
  
  if (motor.arePositionLimitsEnabled()) {
    Serial.println("Min Limit: " + String(motor.getMinPositionLimit(), 3) + " rad (" + 
                  String(motor.getMinPositionLimit() * 180.0 / PI, 1) + "°)");
    Serial.println("Max Limit: " + String(motor.getMaxPositionLimit(), 3) + " rad (" + 
                  String(motor.getMaxPositionLimit() * 180.0 / PI, 1) + "°)");
    
    float current_pos = motor.getStatus().position;
    Serial.println("Current Position: " + String(current_pos, 3) + " rad (" + 
                  String(current_pos * 180.0 / PI, 1) + "°)");
    Serial.println("Within Limits: " + String(motor.isPositionWithinLimits(current_pos) ? "YES" : "NO"));
  }
  Serial.println("=============================");
}