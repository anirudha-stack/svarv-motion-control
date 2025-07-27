/**
 * @file Multi_Motor_Control.ino
 * @brief Advanced example controlling multiple motors simultaneously
 * @author Svarv Robotics
 * 
 * This example demonstrates advanced features of the Svarv Motion Control library
 * including multiple motor coordination, synchronized movements, and system monitoring.
 * 
 * Hardware Required:
 * - ESP32 with built-in CAN (ESP32-C3, ESP32-S3, etc.)
 * - CAN transceiver (SN65HVD230 or similar)
 * - Multiple Svarv BLDC Motor Controllers (2-4 recommended)
 * - 120Ω termination resistors on CAN bus
 * 
 * @section FEATURES
 * 
 * This example demonstrates:
 * - Multi-motor initialization and management
 * - Synchronized coordinated movements
 * - Real-time system monitoring
 * - Error handling across multiple motors
 * - Advanced movement patterns
 */

#include "SvarvMotionControl.h"

// Motion control system
SvarvMotionControl svarv;

// Motor objects
SvarvMotor motor1, motor2, motor3, motor4;
std::vector<SvarvMotor*> motors;

// Control variables
unsigned long lastCoordinatedMove = 0;
int coordinationStep = 0;
bool systemReady = false;

// Movement patterns
struct MovementPattern {
  float positions[4];  // Target positions for each motor
  String description;
  unsigned long duration; // Time to hold position (ms)
};

MovementPattern patterns[] = {
  {{0.0, 0.0, 0.0, 0.0}, "Home Position", 2000},
  {{1.57, 0.0, -1.57, 0.0}, "Pattern 1: Alternating", 3000},
  {{3.14, 3.14, 3.14, 3.14}, "Pattern 2: All Forward", 3000},
  {{1.57, 3.14, 4.71, 6.28}, "Pattern 3: Progressive", 4000},
  {{0.0, 1.57, 0.0, 1.57}, "Pattern 4: Checkerboard", 3000},
  {{-1.57, -1.57, -1.57, -1.57}, "Pattern 5: All Reverse", 3000}
};

const int numPatterns = sizeof(patterns) / sizeof(patterns[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Svarv Motion Control - Multi-Motor Example");
  Serial.println("==========================================");
  
  // Enable debug output
  svarv.enableDebug(true);
  
  // Initialize CAN bus
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    while (1) delay(1000);
  }
  
  Serial.println("CAN bus initialized at 1 Mbps");
  
  // Auto-discover and configure motors
  Serial.println("Scanning for motors...");
  auto discovered = svarv.scanForMotors(5000);
  
  Serial.print("Found ");
  Serial.print(discovered.size());
  Serial.println(" motors:");
  
  for (uint8_t nodeId : discovered) {
    Serial.print("  - Motor ");
    Serial.println(nodeId);
  }
  
  // Add motors to our system
  if (discovered.size() >= 1) {
    motor1 = svarv.addMotor(discovered[0]);
    motors.push_back(&motor1);
  }
  if (discovered.size() >= 2) {
    motor2 = svarv.addMotor(discovered[1]);
    motors.push_back(&motor2);
  }
  if (discovered.size() >= 3) {
    motor3 = svarv.addMotor(discovered[2]);
    motors.push_back(&motor3);
  }
  if (discovered.size() >= 4) {
    motor4 = svarv.addMotor(discovered[3]);
    motors.push_back(&motor4);
  }
  
  if (motors.empty()) {
    Serial.println("ERROR: No motors found!");
    Serial.println("Please check connections and motor configuration.");
    while (1) delay(1000);
  }
  
  // Configure all motors for position control
  Serial.println("Configuring motors...");
  for (auto* motor : motors) {
    // Set up error handling
    motor->onError([](SvarvMotor& motor, SvarvErrorCode error, const String& message) {
      Serial.print("ERROR Motor ");
      Serial.print(motor.getNodeId());
      Serial.print(": ");
      Serial.println(message);
      
      // Emergency stop all motors on any error
      Serial.println("Emergency stopping all motors due to error!");
      // Note: We can't call svarv.emergencyStopAll() here due to callback context
    });
    
    // Set up connection monitoring
    motor->onConnectionChange([](SvarvMotor& motor, SvarvConnectionState oldState, SvarvConnectionState newState) {
      Serial.print("Motor ");
      Serial.print(motor.getNodeId());
      Serial.print(" connection: ");
      Serial.println(svarvConnectionStateToString(newState));
    });
    
    // Configure for position control
    motor->setControlMode(SVARV_MODE_POSITION);
    
    // Set conservative PID values for smooth coordinated motion
    motor->setPID(SVARV_PID_POSITION, 15.0, 0.0, 0.1);
    
    // Set reasonable limits
    motor->setLimits(10.0, 3.0, 5.0); // vel_limit, current_limit, voltage_limit
    
    delay(100); // Small delay between motor configurations
  }
  
  // Wait for all motors to be ready
  Serial.println("Waiting for all motors to be ready...");
  unsigned long waitStart = millis();
  
  while (millis() - waitStart < 10000) {
    svarv.update();
    
    bool allReady = true;
    for (auto* motor : motors) {
      if (!motor->isConnected() || motor->getStatus().initializing) {
        allReady = false;
        break;
      }
    }
    
    if (allReady) {
      systemReady = true;
      break;
    }
    
    delay(100);
  }
  
  if (!systemReady) {
    Serial.println("ERROR: Not all motors are ready!");
    while (1) delay(1000);
  }
  
  Serial.println("All motors ready!");
  Serial.print("System has ");
  Serial.print(motors.size());
  Serial.println(" motors configured for coordinated motion.");
  Serial.println();
  
  // Save configuration to all motors
  Serial.println("Saving configuration to motors...");
  for (auto* motor : motors) {
    motor->saveConfig();
    delay(200);
  }
  
  Serial.println("Starting coordinated movement patterns...");
  Serial.println();
}

void loop() {
  // CRITICAL: Always update the system
  svarv.update();
  
  if (!systemReady) {
    return;
  }
  
  // Execute coordinated movement patterns
  executeCoordinatedMovement();
  
  // Monitor system health
  monitorSystemHealth();
  
  // Handle emergency stop on Serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "stop" || command == "STOP") {
      Serial.println("EMERGENCY STOP TRIGGERED!");
      svarv.emergencyStopAll();
    } else if (command == "home") {
      Serial.println("Homing all motors...");
      homeAllMotors();
    } else if (command == "enable") {
      Serial.println("Enabling all motors...");
      svarv.enableAll();
    } else if (command == "status") {
      printDetailedStatus();
    }
  }
}

void executeCoordinatedMovement() {
  static unsigned long patternStartTime = 0;
  static int currentPattern = 0;
  
  unsigned long currentTime = millis();
  
  // Check if it's time to start a new pattern
  if (currentTime - patternStartTime >= patterns[currentPattern].duration) {
    // Move to next pattern
    currentPattern = (currentPattern + 1) % numPatterns;
    patternStartTime = currentTime;
    
    Serial.println("=== " + patterns[currentPattern].description + " ===");
    
    // Send commands to all motors simultaneously
    for (int i = 0; i < motors.size() && i < 4; i++) {
      float targetPos = patterns[currentPattern].positions[i];
      motors[i]->moveTo(targetPos);
      
      Serial.print("Motor ");
      Serial.print(motors[i]->getNodeId());
      Serial.print(" -> ");
      Serial.print(targetPos, 2);
      Serial.println(" rad");
    }
    
    Serial.println();
  }
}

void homeAllMotors() {
  Serial.println("Homing all motors to position 0...");
  
  for (auto* motor : motors) {
    motor->setControlMode(SVARV_MODE_POSITION);
    motor->moveTo(0.0);
  }
  
  // Wait for all motors to reach home position
  Serial.print("Waiting for motors to reach home position");
  unsigned long homeStart = millis();
  
  while (millis() - homeStart < 10000) {
    svarv.update();
    
    bool allAtHome = true;
    for (auto* motor : motors) {
      if (!motor->isAtTarget(0.1)) { // 0.1 rad tolerance
        allAtHome = false;
        break;
      }
    }
    
    if (allAtHome) {
      Serial.println(" Done!");
      return;
    }
    
    // Print progress indicator
    if ((millis() - homeStart) % 500 == 0) {
      Serial.print(".");
    }
    
    delay(50);
  }
  
  Serial.println(" Timeout!");
}

void monitorSystemHealth() {
  static unsigned long lastHealthCheck = 0;
  
  if (millis() - lastHealthCheck > 5000) {
    lastHealthCheck = millis();
    
    // Check CAN bus health
    if (!svarv.isCANHealthy()) {
      Serial.println("WARNING: CAN bus health issues detected!");
    }
    
    // Check motor connections
    int connectedCount = 0;
    int errorCount = 0;
    
    for (auto* motor : motors) {
      if (motor->isConnected()) {
        connectedCount++;
      }
      if (motor->hasError()) {
        errorCount++;
      }
    }
    
    Serial.print("Health Check - Connected: ");
    Serial.print(connectedCount);
    Serial.print("/");
    Serial.print(motors.size());
    Serial.print(", Errors: ");
    Serial.println(errorCount);
    
    // Get CAN statistics
    uint32_t sent, received, errors;
    svarv.getCANStatistics(sent, received, errors);
    
    Serial.print("CAN Stats - Sent: ");
    Serial.print(sent);
    Serial.print(", Received: ");
    Serial.print(received);
    Serial.print(", Errors: ");
    Serial.println(errors);
    Serial.println();
  }
}

void printDetailedStatus() {
  Serial.println("=== DETAILED SYSTEM STATUS ===");
  
  for (int i = 0; i < motors.size(); i++) {
    auto* motor = motors[i];
    const auto& status = motor->getStatus();
    
    Serial.print("Motor ");
    Serial.print(motor->getNodeId());
    Serial.println(":");
    Serial.print("  Connection: ");
    Serial.println(svarvConnectionStateToString(status.connection_state));
    Serial.print("  Mode: ");
    Serial.println(svarvModeToString(status.control_mode));
    Serial.print("  Position: ");
    Serial.print(status.position, 3);
    Serial.println(" rad");
    Serial.print("  Velocity: ");
    Serial.print(status.velocity, 2);
    Serial.println(" rad/s");
    Serial.print("  Current: ");
    Serial.print(status.current_q, 2);
    Serial.println(" A");
    Serial.print("  Enabled: ");
    Serial.println(status.enabled ? "Yes" : "No");
    
    if (status.error_code != SVARV_ERROR_NONE) {
      Serial.print("  ERROR: ");
      Serial.println(motor->getErrorString());
    }
    
    Serial.println();
  }
  
  Serial.println("Commands:");
  Serial.println("  'stop' - Emergency stop all motors");
  Serial.println("  'home' - Home all motors to position 0");
  Serial.println("  'enable' - Enable all motors");
  Serial.println("  'status' - Print this status");
  Serial.println("=============================");
  Serial.println();
}