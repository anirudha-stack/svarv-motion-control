/**
 * @file Cross_Platform_Example.ino
 * @brief Cross-platform motor control example using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example demonstrates how to use the Svarv Motion Control library across
 * different hardware platforms with automatic platform detection and configuration.
 * 
 * Supported Platforms:
 * - ESP32 (ESP32-C3, ESP32-S3, ESP32 Classic) - Built-in CAN
 * - STM32 (F1, F4, F7, H7 series) - Built-in CAN
 * - Arduino AVR (Uno, Mega, Nano) + MCP2515 CAN shield
 * - Other platforms with MCP2515 support
 * 
 * Hardware Connections:
 * 
 * ESP32 (Built-in CAN):
 * - CAN TX: GPIO 21 (default)
 * - CAN RX: GPIO 20 (default)
 * - 120Ω termination resistors on CAN bus
 * 
 * STM32 (Built-in CAN):
 * - Uses built-in CAN peripheral (CAN1)
 * - Pin configuration handled automatically
 * - 120Ω termination resistors on CAN bus
 * 
 * Arduino + MCP2515:
 * - CS (Chip Select): Pin 10 (default)
 * - INT (Interrupt): Pin 2 (default)
 * - SCK, MOSI, MISO: Standard SPI pins
 * - 120Ω termination resistors on CAN bus
 * 
 * @section BEHAVIOR
 * 
 * This example will:
 * 1. Auto-detect the hardware platform
 * 2. Initialize CAN with platform-appropriate settings
 * 3. Scan for and connect to available motors
 * 4. Demonstrate basic motor control operations
 * 5. Show platform-specific information
 */

#include "SvarvMotionControl.h"

// Create motion control instance
SvarvMotionControl svarv;
SvarvMotor motor1;

// Control variables
unsigned long lastMoveTime = 0;
int moveStep = 0;
bool systemInitialized = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10); // Wait up to 5 seconds for Serial
  
  Serial.println("===========================================");
  Serial.println("Svarv Motion Control - Cross-Platform Demo");
  Serial.println("===========================================");
  Serial.println();
  
  // Display library and platform information
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println();
  
  // Enable debug output
  svarv.enableDebug(true);
  
  Serial.println("Detecting platform and initializing CAN...");
  
  // Initialize with auto-platform detection
  bool canInitialized = false;
  
#if defined(ESP32)
  // ESP32 - Try built-in CAN with default pins
  Serial.println("Platform: ESP32 detected");
  Serial.println("Attempting ESP32 built-in CAN initialization...");
  canInitialized = svarv.begin(1000000); // 1 Mbps, default pins (21, 20)
  
  if (!canInitialized) {
    // Try with alternative pin configuration
    Serial.println("Retrying with alternative pins (GPIO 4, 5)...");
    canInitialized = svarv.begin(1000000, 4, 5);
  }
  
#elif defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
  // STM32 - Try built-in CAN
  Serial.println("Platform: STM32 detected");
  Serial.println("Attempting STM32 built-in CAN initialization...");
  canInitialized = svarv.begin(1000000); // Uses CAN1 by default
  
#else
  // Arduino AVR or other platform - Use MCP2515
  Serial.println("Platform: Arduino/Generic detected");
  Serial.println("Attempting MCP2515 CAN initialization...");
  
  // Try different common pin configurations
  int csPins[] = {10, 9, 8, 7};
  int intPins[] = {2, 3};
  
  for (int cs : csPins) {
    for (int intPin : intPins) {
      Serial.println("Trying CS pin " + String(cs) + ", INT pin " + String(intPin));
      canInitialized = svarv.begin(1000000, cs, intPin, MCP_8MHZ);
      if (canInitialized) {
        Serial.println("Success with CS=" + String(cs) + ", INT=" + String(intPin));
        break;
      }
    }
    if (canInitialized) break;
  }
  
  if (!canInitialized) {
    // Try with 16MHz crystal
    Serial.println("Retrying with 16MHz crystal...");
    canInitialized = svarv.begin(1000000, 10, 2, MCP_16MHZ);
  }
  
#endif

  if (!canInitialized) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    Serial.println();
    Serial.println("Troubleshooting tips:");
    Serial.println("- Check CAN transceiver connections");
    Serial.println("- Verify 120Ω termination resistors");
    Serial.println("- Check power supply to CAN transceiver");
    Serial.println("- For MCP2515: verify CS and INT pin connections");
    Serial.println("- For MCP2515: check crystal frequency (8MHz/16MHz)");
    
    while (1) {
      delay(1000);
      Serial.print(".");
    }
  }
  
  Serial.println();
  Serial.println("CAN bus initialized successfully!");
  Serial.println("Platform: " + svarv.getPlatformInfo());
  Serial.println();
  
  // Scan for motors
  Serial.println("Scanning for motors...");
  auto discovered = svarv.scanForMotors(5000);
  
  if (discovered.empty()) {
    Serial.println("WARNING: No motors found!");
    Serial.println();
    Serial.println("Please check:");
    Serial.println("- Motor controller power");
    Serial.println("- CAN bus connections");
    Serial.println("- Motor controller CAN configuration");
    Serial.println();
    Serial.println("Continuing with demo anyway...");
    
    // Add a motor anyway for demonstration
    motor1 = svarv.addMotor(1);
  } else {
    Serial.print("Found " + String(discovered.size()) + " motor(s): ");
    for (size_t i = 0; i < discovered.size(); i++) {
      Serial.print(discovered[i]);
      if (i < discovered.size() - 1) Serial.print(", ");
    }
    Serial.println();
    
    // Use the first discovered motor
    motor1 = svarv.addMotor(discovered[0]);
  }
  
  // Set up motor callbacks
  motor1.onStatusUpdate([](SvarvMotor& motor, const SvarvMotorStatus& status) {
    // Only print status updates occasionally to avoid spam
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      lastPrint = millis();
      Serial.print("Motor " + String(motor.getNodeId()) + " Status: ");
      Serial.print(svarvModeToString(status.control_mode) + ", ");
      Serial.print("Pos: " + String(status.position, 2) + " rad, ");
      Serial.print("Vel: " + String(status.velocity, 2) + " rad/s");
      if (status.error_code != SVARV_ERROR_NONE) {
        Serial.print(", ERROR: " + motor.getErrorString());
      }
      Serial.println();
    }
  });
  
  motor1.onError([](SvarvMotor& motor, SvarvErrorCode error, const String& message) {
    Serial.println("ERROR on Motor " + String(motor.getNodeId()) + ": " + message);
  });
  
  motor1.onConnectionChange([](SvarvMotor& motor, SvarvConnectionState oldState, SvarvConnectionState newState) {
    Serial.println("Motor " + String(motor.getNodeId()) + " connection: " + 
                  svarvConnectionStateToString(oldState) + " -> " + 
                  svarvConnectionStateToString(newState));
  });
  
  // Wait for motor to be ready
  Serial.print("Waiting for motor to initialize");
  unsigned long waitStart = millis();
  while (!motor1.isConnected() && millis() - waitStart < 10000) {
    svarv.update();
    Serial.print(".");
    delay(100);
  }
  
  if (motor1.isConnected()) {
    Serial.println(" Connected!");
    
    // Configure motor for position control
    motor1.setControlMode(SVARV_MODE_POSITION);
    
    // Set conservative PID values for smooth operation
    motor1.setPID(SVARV_PID_POSITION, 15.0, 0.0, 0.1);
    
    Serial.println();
    Serial.println("Starting movement demo...");
    systemInitialized = true;
  } else {
    Serial.println(" Timeout!");
    Serial.println("Motor not responding - continuing with demo anyway");
  }
  
  Serial.println();
  Serial.println("Demo Commands (send via Serial Monitor):");
  Serial.println("  'info'  - Show platform and system information");
  Serial.println("  'scan'  - Scan for motors");
  Serial.println("  'home'  - Move motor to home position (0 rad)");
  Serial.println("  'stop'  - Emergency stop");
  Serial.println("  'stats' - Show CAN bus statistics");
  Serial.println("  'pos<value>' - Move to position (e.g., 'pos3.14')");
  Serial.println();
}

void loop() {
  // IMPORTANT: Always call update() to process CAN messages
  svarv.update();
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    handleSerialCommand(command);
  }
  
  // Run automated movement demo if system is initialized
  if (systemInitialized && motor1.isConnected()) {
    runMovementDemo();
  }
  
  // Print system status periodically
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 10000) { // Every 10 seconds
    lastStatusTime = millis();
    printSystemStatus();
  }
}

void handleSerialCommand(const String& command) {
  if (command == "info") {
    printPlatformInfo();
  } else if (command == "scan") {
    Serial.println("Scanning for motors...");
    auto discovered = svarv.scanForMotors(3000);
    Serial.println("Found " + String(discovered.size()) + " motor(s)");
    for (uint8_t nodeId : discovered) {
      Serial.println("  - Motor " + String(nodeId));
    }
  } else if (command == "home") {
    Serial.println("Homing motor...");
    motor1.moveTo(0.0);
  } else if (command == "stop") {
    Serial.println("Emergency stop!");
    svarv.emergencyStopAll();
  } else if (command == "stats") {
    printCANStatistics();
  } else if (command.startsWith("pos")) {
    float pos = command.substring(3).toFloat();
    Serial.println("Moving to position: " + String(pos) + " radians");
    motor1.moveTo(pos);
  } else if (command == "help") {
    Serial.println("Available commands: info, scan, home, stop, stats, pos<value>, help");
  } else if (command.length() > 0) {
    Serial.println("Unknown command: '" + command + "'. Type 'help' for available commands.");
  }
}

void runMovementDemo() {
  // Run movement demo every 8 seconds
  if (millis() - lastMoveTime > 8000) {
    lastMoveTime = millis();
    
    switch (moveStep) {
      case 0:
        Serial.println("=== Demo: Moving to π/2 radians (90 degrees) ===");
        motor1.moveTo(1.5708); // π/2 radians
        break;
        
      case 1:
        Serial.println("=== Demo: Moving to π radians (180 degrees) ===");
        motor1.moveTo(3.14159); // π radians
        break;
        
      case 2:
        Serial.println("=== Demo: Moving to -π/2 radians (-90 degrees) ===");
        motor1.moveTo(-1.5708); // -π/2 radians
        break;
        
      case 3:
        Serial.println("=== Demo: Returning to home (0 radians) ===");
        motor1.moveTo(0.0);
        break;
        
      case 4:
        Serial.println("=== Demo: Testing velocity control ===");
        motor1.setControlMode(SVARV_MODE_VELOCITY);
        motor1.setVelocity(2.0); // 2 rad/s
        break;
        
      case 5:
        Serial.println("=== Demo: Stopping velocity control ===");
        motor1.setVelocity(0.0);
        break;
        
      case 6:
        Serial.println("=== Demo: Returning to position control ===");
        motor1.setControlMode(SVARV_MODE_POSITION);
        motor1.moveTo(0.0);
        moveStep = -1; // Will be incremented to 0
        break;
    }
    
    moveStep++;
  }
}

void printPlatformInfo() {
  Serial.println("=== PLATFORM INFORMATION ===");
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println("CAN Interface: " + svarv.getPlatformInfo());
  
#if defined(ESP32)
  Serial.println("Platform: ESP32");
  Serial.println("Chip Model: " + String(ESP.getChipModel()));
  Serial.println("Chip Revision: " + String(ESP.getChipRevision()));
  Serial.println("CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
  Serial.println("Flash Size: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB");
#elif defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
  Serial.println("Platform: STM32");
  Serial.println("Core Version: " + String(STM32_CORE_VERSION));
#elif defined(ARDUINO_ARCH_AVR)
  Serial.println("Platform: Arduino AVR");
  Serial.println("Board: " + String(ARDUINO_BOARD));
#else
  Serial.println("Platform: Generic/Other");
#endif

  Serial.println("Compile Date: " + String(__DATE__) + " " + String(__TIME__));
  Serial.println("Connected Motors: " + String(svarv.getConnectedMotorCount()));
  Serial.println("CAN Healthy: " + String(svarv.isCANHealthy() ? "Yes" : "No"));
  Serial.println("=============================");
}

void printCANStatistics() {
  uint32_t sent, received, errors;
  svarv.getCANStatistics(sent, received, errors);
  
  Serial.println("=== CAN BUS STATISTICS ===");
  Serial.println("Messages Sent: " + String(sent));
  Serial.println("Messages Received: " + String(received));
  Serial.println("Errors: " + String(errors));
  
  if (sent + received > 0) {
    float errorRate = (float)errors / (sent + received) * 100.0;
    Serial.println("Error Rate: " + String(errorRate, 2) + "%");
  }
  
  Serial.println("Bus Health: " + String(svarv.isCANHealthy() ? "Good" : "Poor"));
  Serial.println("===========================");
}

void printSystemStatus() {
  Serial.println("--- System Status ---");
  Serial.println("Uptime: " + String(millis() / 1000) + " seconds");
  Serial.println("Connected Motors: " + String(svarv.getConnectedMotorCount()));
  
  if (motor1.isConnected()) {
    const auto& status = motor1.getStatus();
    Serial.println("Motor 1: " + svarvModeToString(status.control_mode) + 
                  ", Pos: " + String(status.position, 2) + " rad" +
                  ", Enabled: " + String(status.enabled ? "Yes" : "No"));
    
    if (status.error_code != SVARV_ERROR_NONE) {
      Serial.println("Motor 1 Error: " + motor1.getErrorString());
    }
  } else {
    Serial.println("Motor 1: Disconnected");
  }
  
  Serial.println("CAN Health: " + String(svarv.isCANHealthy() ? "Good" : "Poor"));
  Serial.println("--------------------");
}