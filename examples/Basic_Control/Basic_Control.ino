/**
 * @file Basic_Control.ino
 * @brief Basic motor control example using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example demonstrates the basic usage of the Svarv Motion Control library
 * for controlling a single BLDC motor via CAN bus.
 * 
 * Hardware Required:
 * - ESP32 with built-in CAN (ESP32-C3, ESP32-S3, etc.)
 * - CAN transceiver (SN65HVD230 or similar)
 * - Svarv BLDC Motor Controller with firmware v1.2+
 * 
 * Connections:
 * - CAN TX: GPIO 21
 * - CAN RX: GPIO 20
 * - 120Ω termination resistors on both ends of CAN bus
 * 
 * @section BEHAVIOR
 * 
 * This example will:
 * 1. Initialize the CAN bus
 * 2. Connect to a motor with node ID 1
 * 3. Demonstrate position, velocity, and torque control
 * 4. Show real-time status monitoring
 */

#include "SvarvMotionControl.h"

// Create motion control instance
SvarvMotionControl svarv;
SvarvMotor motor1;

// Control variables
unsigned long lastMoveTime = 0;
int moveStep = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Svarv Motion Control - Basic Example");
  Serial.println("====================================");
  
  // Enable debug output
  svarv.enableDebug(true);
  
  // Initialize CAN bus at 1 Mbps
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    Serial.println("Check your connections and try again.");
    while (1) delay(1000);
  }
  
  Serial.println("CAN bus initialized successfully!");
  
  // Add motor with node ID 1
  motor1 = svarv.addMotor(1);
  
  // Set up callbacks for motor events
  motor1.onStatusUpdate([](SvarvMotor& motor, const SvarvMotorStatus& status) {
    Serial.print("Motor Status - Mode: ");
    Serial.print(svarvModeToString(status.control_mode));
    Serial.print(", Position: ");
    Serial.print(status.position, 3);
    Serial.print(" rad, Velocity: ");
    Serial.print(status.velocity, 2);
    Serial.print(" rad/s, Enabled: ");
    Serial.println(status.enabled ? "Yes" : "No");
  });
  
  motor1.onError([](SvarvMotor& motor, SvarvErrorCode error, const String& message) {
    Serial.print("ERROR on Motor ");
    Serial.print(motor.getNodeId());
    Serial.print(": ");
    Serial.println(message);
  });
  
  motor1.onConnectionChange([](SvarvMotor& motor, SvarvConnectionState oldState, SvarvConnectionState newState) {
    Serial.print("Motor ");
    Serial.print(motor.getNodeId());
    Serial.print(" connection: ");
    Serial.print(svarvConnectionStateToString(oldState));
    Serial.print(" -> ");
    Serial.println(svarvConnectionStateToString(newState));
  });
  
  Serial.println("Waiting for motor connection...");
  Serial.println("Make sure your motor controller is powered and on the CAN bus.");
  
  // Wait for motor to connect
  unsigned long connectStart = millis();
  while (!motor1.isConnected() && millis() - connectStart < 10000) {
    svarv.update();
    delay(100);
  }
  
  if (!motor1.isConnected()) {
    Serial.println("ERROR: Motor not responding!");
    Serial.println("Check:");
    Serial.println("- Motor controller power");
    Serial.println("- CAN bus connections");
    Serial.println("- Node ID configuration");
    while (1) delay(1000);
  }
  
  Serial.println("Motor connected successfully!");
  Serial.println("");
  Serial.println("Starting movement demo...");
  Serial.println("The motor will demonstrate different control modes:");
  Serial.println("1. Position control");
  Serial.println("2. Velocity control");
  Serial.println("3. Torque control");
  Serial.println("");
}

void loop() {
  // IMPORTANT: Always call update() to process CAN messages
  svarv.update();
  
  // Run movement demo every 5 seconds
  if (millis() - lastMoveTime > 5000) {
    lastMoveTime = millis();
    
    if (!motor1.isConnected()) {
      Serial.println("Motor disconnected - skipping movement");
      return;
    }
    
    switch (moveStep) {
      case 0:
        Serial.println("=== Position Control Demo ===");
        motor1.setControlMode(SVARV_MODE_POSITION);
        motor1.moveTo(0.0); // Move to 0 radians
        Serial.println("Moving to 0 radians...");
        break;
        
      case 1:
        motor1.moveTo(3.14159); // Move to π radians (180 degrees)
        Serial.println("Moving to π radians (180 degrees)...");
        break;
        
      case 2:
        motor1.moveTo(-3.14159); // Move to -π radians (-180 degrees)
        Serial.println("Moving to -π radians (-180 degrees)...");
        break;
        
      case 3:
        motor1.moveTo(0.0); // Return to 0
        Serial.println("Returning to 0 radians...");
        break;
        
      case 4:
        Serial.println("=== Velocity Control Demo ===");
        motor1.setControlMode(SVARV_MODE_VELOCITY);
        motor1.setVelocity(2.0); // 2 rad/s clockwise
        Serial.println("Spinning at 2 rad/s...");
        break;
        
      case 5:
        motor1.setVelocity(-2.0); // 2 rad/s counter-clockwise
        Serial.println("Spinning at -2 rad/s...");
        break;
        
      case 6:
        motor1.setVelocity(0.0); // Stop
        Serial.println("Stopping...");
        break;
        
      case 7:
        Serial.println("=== Torque Control Demo ===");
        motor1.setControlMode(SVARV_MODE_TORQUE);
        motor1.setTorque(0.1); // Small positive torque
        Serial.println("Applying 0.1 Nm torque...");
        break;
        
      case 8:
        motor1.setTorque(-0.1); // Small negative torque
        Serial.println("Applying -0.1 Nm torque...");
        break;
        
      case 9:
        motor1.setTorque(0.0); // No torque
        Serial.println("Removing torque...");
        break;
        
      case 10:
        Serial.println("=== Demo Complete ===");
        motor1.disable(); // Return to idle mode
        Serial.println("Motor disabled. Demo will restart in 5 seconds.");
        Serial.println("");
        moveStep = -1; // Will be incremented to 0
        break;
    }
    
    moveStep++;
  }
  
  // Print motor status every 2 seconds
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 2000) {
    lastStatusTime = millis();
    
    if (motor1.isConnected()) {
      const SvarvMotorStatus& status = motor1.getStatus();
      
      Serial.print("Status: Mode=");
      Serial.print(svarvModeToString(status.control_mode));
      Serial.print(", Pos=");
      Serial.print(status.position, 2);
      Serial.print(" rad, Vel=");
      Serial.print(status.velocity, 2);
      Serial.print(" rad/s, Current=");
      Serial.print(status.current_q, 2);
      Serial.print(" A");
      
      if (status.error_code != SVARV_ERROR_NONE) {
        Serial.print(", ERROR: ");
        Serial.print(motor1.getErrorString());
      }
      
      Serial.println();
    }
  }
}