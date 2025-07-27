/**
 * @file Node_ID_Setup_Example.ino
 * @brief Node ID configuration example using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example demonstrates how to configure Node IDs for Svarv motor controllers
 * using the built-in library functions. This is essential when setting up new motors
 * or when you have multiple motors that need unique identifiers on the CAN bus.
 * 
 * Hardware Required:
 * - ESP32/STM32/Arduino with CAN capability
 * - CAN transceiver or MCP2515 shield
 * - One or more Svarv BLDC Motor Controllers
 * 
 * @section SCENARIOS
 * 
 * This example covers Node ID setup scenarios:
 * 1. Discovering motors and their current Node IDs
 * 2. Auto-configuring multiple new motors (Node ID = 0)
 * 3. Testing communication with configured motors
 * 4. Factory reset and reconfiguration
 * 
 * @section BEHAVIOR
 * 
 * The example provides an interactive menu system that allows you to:
 * - Scan for motors and display their current Node IDs
 * - Auto-assign sequential Node IDs to unconfigured motors
 * - Test communication with specific motors
 * - Reset motors to factory defaults
 * 
 * @section USAGE
 * 
 * 1. Connect your motor controller(s) to the CAN bus
 * 2. Upload this example to your microcontroller
 * 3. Open Serial Monitor (115200 baud)
 * 4. Follow the interactive menu to configure Node IDs
 * 
 * @note New motors typically have Node ID = 0 (unconfigured)
 * @note Each motor on the CAN bus must have a unique Node ID (1-255)
 */

#include "SvarvMotionControl.h"

// Create motion control instance
SvarvMotionControl svarv;

// Menu system state
bool showMenu = true;
unsigned long lastScan = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10); // Wait for Serial
  
  Serial.println("========================================");
  Serial.println("Svarv Motion Control - Node ID Setup");
  Serial.println("========================================");
  Serial.println();
  
  // Enable debug output for detailed information
  svarv.enableDebug(true);
  
  Serial.println("Initializing CAN bus...");
  
  // Initialize CAN with auto-platform detection
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    Serial.println("Please check your connections and try again.");
    while (1) delay(1000);
  }
  
  Serial.println("CAN bus initialized successfully!");
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println();
  
  // Initial scan for motors
  performMotorScan();
  
  // Show main menu
  printMainMenu();
}

void loop() {
  // Always update the CAN system
  svarv.update();
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    processCommand(command);
  }
  
  // Auto-scan periodically if no motors found
  if (svarv.getConnectedMotorCount() == 0 && millis() - lastScan > 10000) {
    Serial.println("No motors detected. Performing automatic scan...");
    performMotorScan();
    lastScan = millis();
  }
  
  // Show menu periodically
  static unsigned long lastMenuTime = 0;
  if (showMenu && millis() - lastMenuTime > 30000) {
    lastMenuTime = millis();
    Serial.println("\n--- Type 'help' to see the menu again ---");
  }
}

void processCommand(const String& command) {
  if (command == "1" || command == "scan") {
    performMotorScan();
    
  } else if (command == "2" || command == "auto") {
    autoConfigureMotors();
    
  } else if (command == "3" || command == "test") {
    testMotorCommunication();
    
  } else if (command == "4" || command == "reset") {
    factoryResetMotor();
    
  } else if (command == "help" || command == "menu") {
    printMainMenu();
    
  } else if (command == "info") {
    showSystemInfo();
    
  } else if (command == "ref") {
    printQuickReference();
    
  } else if (command == "demo") {
    demonstrateAdvancedFeatures();
    
  } else if (command.length() > 0) {
    Serial.println("Unknown command: '" + command + "'");
    Serial.println("Type 'help' to see available commands.");
  }
}

void printMainMenu() {
  Serial.println("=== NODE ID CONFIGURATION MENU ===");
  Serial.println("1. scan  - Scan for motors and show Node IDs");
  Serial.println("2. auto  - Auto-configure unconfigured motors");
  Serial.println("3. test  - Test communication with motor");
  Serial.println("4. reset - Factory reset motor");
  Serial.println();
  Serial.println("Additional: help, info, ref, demo");
  Serial.println("===================================");
  Serial.println();
  showMenu = false;
}

void performMotorScan() {
  Serial.println("=== SCANNING FOR MOTORS ===");
  Serial.println("Scanning CAN bus for motors (this may take a few seconds)...");
  
  // Clear existing motors to get fresh scan
  auto existing = svarv.getAllMotors();
  for (auto* motor : existing) {
    svarv.removeMotor(motor->getNodeId());
  }
  
  // Scan for motors with longer timeout
  auto discovered = svarv.scanForMotors(8000);
  
  Serial.println();
  if (discovered.empty()) {
    Serial.println("❌ No configured motors found on the CAN bus.");
    Serial.println();
    Serial.println("This could mean:");
    Serial.println("- No motors are connected or powered");
    Serial.println("- All motors have Node ID = 0 (unconfigured)");
    Serial.println("- CAN bus connection issues");
    Serial.println();
    Serial.println("Troubleshooting tips:");
    Serial.println("- Check motor controller power");
    Serial.println("- Verify CAN bus connections (CANH, CANL, GND)");
    Serial.println("- Ensure 120Ω termination resistors are installed");
    Serial.println("- Try option 2 to auto-configure unconfigured motors");
    
  } else {
    Serial.println("✅ Found " + String(discovered.size()) + " configured motor(s):");
    Serial.println();
    
    // Display detailed information for each motor
    for (uint8_t nodeId : discovered) {
      SvarvMotor& motor = svarv.addMotor(nodeId);
      
      // Wait a moment for status update
      unsigned long startTime = millis();
      while (!motor.isConnected() && millis() - startTime < 2000) {
        svarv.update();
        delay(10);
      }
      
      Serial.print("  📡 Motor Node ID: " + String(nodeId));
      
      if (motor.isConnected()) {
        const auto& status = motor.getStatus();
        Serial.print(" - Status: Connected");
        Serial.print(" - Mode: " + svarvModeToString(status.control_mode));
        Serial.print(" - Position: " + String(status.position, 2) + " rad");
        
        if (status.error_code != SVARV_ERROR_NONE) {
          Serial.print(" - ERROR: " + motor.getErrorString());
        }
      } else {
        Serial.print(" - Status: Not responding");
      }
      Serial.println();
    }
    
    Serial.println();
    Serial.println("All motors are configured and responding!");
  }
  
  Serial.println("============================");
  Serial.println();
}

void autoConfigureMotors() {
  Serial.println("=== AUTO-CONFIGURE MOTORS ===");
  Serial.println();
  Serial.println("This will automatically assign sequential Node IDs to all");
  Serial.println("unconfigured motors (currently Node ID = 0)");
  Serial.println();
  
  // Get starting Node ID
  Serial.print("Enter starting Node ID (1-255): ");
  String input = waitForSerialInput();
  
  uint8_t startId = input.toInt();
  if (startId < 1 || startId > 255) {
    Serial.println("❌ Invalid starting Node ID. Must be between 1 and 255.");
    return;
  }
  
  // Get maximum number of motors to configure
  Serial.print("Enter maximum number of motors to configure (1-20): ");
  input = waitForSerialInput();
  
  uint8_t maxMotors = input.toInt();
  if (maxMotors < 1 || maxMotors > 20) {
    Serial.println("❌ Invalid number. Must be between 1 and 20.");
    return;
  }
  
  Serial.println();
  Serial.println("Starting auto-configuration...");
  Serial.println("Starting Node ID: " + String(startId));
  Serial.println("Max motors: " + String(maxMotors));
  Serial.println();
  
  // Perform auto-configuration using the library's built-in method
  int configuredCount = svarv.autoConfigureMotors(startId, maxMotors);
  
  Serial.println();
  if (configuredCount > 0) {
    Serial.println("✅ Successfully configured " + String(configuredCount) + " motor(s)!");
    Serial.println("Node IDs assigned: " + String(startId) + " to " + String(startId + configuredCount - 1));
    
    // Verify all configured motors
    Serial.println();
    Serial.println("Verifying configured motors...");
    delay(2000);
    
    for (int i = 0; i < configuredCount; i++) {
      uint8_t nodeId = startId + i;
      SvarvMotor& motor = svarv.addMotor(nodeId);
      
      // Brief connection check
      unsigned long checkStart = millis();
      while (!motor.isConnected() && millis() - checkStart < 1000) {
        svarv.update();
        delay(50);
      }
      
      if (motor.isConnected()) {
        Serial.println("  ✅ Motor " + String(nodeId) + ": Connected and responding");
        motor.saveConfig(); // Save configuration
      } else {
        Serial.println("  ❌ Motor " + String(nodeId) + ": Not responding");
      }
    }
    
  } else {
    Serial.println("❌ No motors were configured.");
    Serial.println("Possible reasons:");
    Serial.println("- No unconfigured motors found (all motors may already have Node IDs)");
    Serial.println("- Communication issues with motors");
    Serial.println("- Motors not powered or connected properly");
    Serial.println();
    Serial.println("Try:");
    Serial.println("- Check that motors are powered and connected");
    Serial.println("- Use 'scan' to see if any motors are already configured");
    Serial.println("- Check CAN bus connections and termination");
  }
  
  Serial.println("=====================================");
  Serial.println();
}

void testMotorCommunication() {
  Serial.println("=== TEST MOTOR COMMUNICATION ===");
  Serial.println();
  
  // Get Node ID to test
  Serial.print("Enter Node ID to test (1-255): ");
  String input = waitForSerialInput();
  
  uint8_t nodeId = input.toInt();
  if (nodeId < 1 || nodeId > 255) {
    Serial.println("❌ Invalid Node ID. Must be between 1 and 255.");
    return;
  }
  
  Serial.println();
  Serial.println("Testing communication with motor " + String(nodeId) + "...");
  
  // Add motor and test connection
  SvarvMotor& motor = svarv.addMotor(nodeId);
  
  // Test connection with timeout
  unsigned long startTime = millis();
  bool connected = false;
  
  while (millis() - startTime < 5000) { // 5 second timeout
    svarv.update();
    
    if (motor.isConnected()) {
      connected = true;
      break;
    }
    
    // Request status update
    motor.requestStatusUpdate();
    delay(100);
  }
  
  if (connected) {
    Serial.println("✅ Motor " + String(nodeId) + " is responding!");
    
    const auto& status = motor.getStatus();
    Serial.println();
    Serial.println("Motor Status:");
    Serial.println("  Control Mode: " + svarvModeToString(status.control_mode));
    Serial.println("  Position: " + String(status.position, 3) + " rad");
    Serial.println("  Velocity: " + String(status.velocity, 2) + " rad/s");
    Serial.println("  Current: " + String(status.current_q, 2) + " A");
    Serial.println("  Enabled: " + String(status.enabled ? "Yes" : "No"));
    Serial.println("  Calibrated: " + String(status.calibrated ? "Yes" : "No"));
    
    if (status.error_code != SVARV_ERROR_NONE) {
      Serial.println("  Error: " + motor.getErrorString());
    } else {
      Serial.println("  Error: None");
    }
    
    // Test basic movement
    Serial.println();
    Serial.print("Test basic movement (y/n)? ");
    input = waitForSerialInput();
    
    if (input.toLowerCase() == "y" || input.toLowerCase() == "yes") {
      Serial.println("Testing position control...");
      motor.setControlMode(SVARV_MODE_POSITION);
      delay(500);
      
      float currentPos = motor.getPosition();
      Serial.println("Current position: " + String(currentPos, 3) + " rad");
      Serial.println("Moving +0.5 radians...");
      motor.moveTo(currentPos + 0.5);
      
      delay(3000);
      Serial.println("Returning to original position...");
      motor.moveTo(currentPos);
      delay(2000);
      
      Serial.println("✅ Movement test complete!");
    }
    
  } else {
    Serial.println("❌ Motor " + String(nodeId) + " is not responding!");
    Serial.println();
    Serial.println("Possible issues:");
    Serial.println("- Motor not powered");
    Serial.println("- Wrong Node ID");
    Serial.println("- CAN bus connection problems");
    Serial.println("- Motor controller fault");
  }
  
  Serial.println("=================================");
  Serial.println();
}

void factoryResetMotor() {
  Serial.println("=== FACTORY RESET MOTOR ===");
  Serial.println();
  Serial.println("⚠️  WARNING: This will reset the motor to factory defaults!");
  Serial.println("All configuration will be lost including Node ID, PID settings, etc.");
  Serial.println();
  
  // Get Node ID to reset
  Serial.print("Enter Node ID of motor to reset (1-255): ");
  String input = waitForSerialInput();
  
  uint8_t nodeId = input.toInt();
  if (nodeId < 1 || nodeId > 255) {
    Serial.println("❌ Invalid Node ID. Must be between 1 and 255.");
    return;
  }
  
  // Confirmation
  Serial.println();
  Serial.println("⚠️  Are you sure you want to factory reset motor " + String(nodeId) + "?");
  Serial.print("Type 'YES' to confirm: ");
  input = waitForSerialInput();
  
  if (input != "YES") {
    Serial.println("❌ Factory reset cancelled.");
    return;
  }
  
  Serial.println();
  Serial.println("Performing factory reset on motor " + String(nodeId) + "...");
  
  // Get motor object
  SvarvMotor& motor = svarv.addMotor(nodeId);
  
  // Wait for connection
  unsigned long startTime = millis();
  while (!motor.isConnected() && millis() - startTime < 3000) {
    svarv.update();
    delay(100);
  }
  
  if (!motor.isConnected()) {
    Serial.println("❌ Cannot connect to motor " + String(nodeId));
    return;
  }
  
  // Perform factory reset
  if (motor.factoryReset()) {
    Serial.println("✅ Factory reset command sent successfully!");
    Serial.println();
    Serial.println("⚠️  Note: Motor will now have Node ID = 0 (unconfigured)");
    Serial.println("Use option 2 to reconfigure the Node ID.");
    
    // Remove motor from our system since it's now unconfigured
    svarv.removeMotor(nodeId);
    
  } else {
    Serial.println("❌ Factory reset failed. Please try again.");
  }
  
  Serial.println("============================");
  Serial.println();
}

void showSystemInfo() {
  Serial.println("=== SYSTEM INFORMATION ===");
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println("Connected Motors: " + String(svarv.getConnectedMotorCount()));
  
  uint32_t sent, received, errors;
  svarv.getCANStatistics(sent, received, errors);
  Serial.println("CAN Messages Sent: " + String(sent));
  Serial.println("CAN Messages Received: " + String(received));
  Serial.println("CAN Errors: " + String(errors));
  Serial.println("CAN Health: " + String(svarv.isCANHealthy() ? "Good" : "Poor"));
  
  // Show all connected motors
  auto motors = svarv.getAllMotors();
  if (!motors.empty()) {
    Serial.println();
    Serial.println("Connected Motors:");
    for (auto* motor : motors) {
      if (motor->isConnected()) {
        Serial.println("  - Motor " + String(motor->getNodeId()) + 
                      " (" + svarvModeToString(motor->getStatus().control_mode) + ")");
      }
    }
  }
  
  Serial.println("===========================");
  Serial.println();
}

void printQuickReference() {
  Serial.println("=== QUICK REFERENCE ===");
  Serial.println();
  Serial.println("Node ID Setup Process:");
  Serial.println("1. Connect motors to CAN bus");
  Serial.println("2. Power on motor controllers");
  Serial.println("3. Run 'scan' to see current motors");
  Serial.println("4. Run 'auto' to configure unconfigured motors");
  Serial.println("5. Run 'test' to verify communication");
  Serial.println();
  Serial.println("Node ID Rules:");
  Serial.println("- New motors have Node ID = 0 (unconfigured)");
  Serial.println("- Each motor needs unique Node ID (1-255)");
  Serial.println("- Node IDs are saved in motor's EEPROM");
  Serial.println("- Factory reset sets Node ID back to 0");
  Serial.println();
  Serial.println("Troubleshooting:");
  Serial.println("- Check CAN bus termination (120Ω resistors)");
  Serial.println("- Verify power to all motor controllers");
  Serial.println("- Ensure proper CAN H/L connections");
  Serial.println("- Use 'info' to check CAN bus health");
  Serial.println();
  Serial.println("Common Commands:");
  Serial.println("- scan: Find all motors");
  Serial.println("- auto: Configure unconfigured motors");
  Serial.println("- test: Test specific motor");
  Serial.println("- reset: Factory reset motor");
  Serial.println("- info: System information");
  Serial.println("- help: Show main menu");
  Serial.println("========================");
  Serial.println();
}

void demonstrateAdvancedFeatures() {
  Serial.println("=== ADVANCED FEATURES DEMO ===");
  
  auto motors = svarv.getAllMotors();
  if (motors.empty()) {
    Serial.println("No motors available for demo. Configure motors first.");
    return;
  }
  
  SvarvMotor* motor = motors[0]; // Use first available motor
  
  Serial.println("Demonstrating advanced features with Motor " + String(motor->getNodeId()));
  
  // 1. PID Tuning Example
  Serial.println();
  Serial.println("1. Setting custom PID parameters...");
  motor->setControlMode(SVARV_MODE_POSITION);
  motor->setPID(SVARV_PID_POSITION, 20.0, 0.1, 0.05);
  Serial.println("   Position PID: P=20.0, I=0.1, D=0.05");
  
  // 2. Safety Limits
  Serial.println();
  Serial.println("2. Setting safety limits...");
  motor->setLimits(10.0, 3.0, 12.0); // vel, current, voltage
  Serial.println("   Limits: 10 rad/s, 3 A, 12 V");
  
  // 3. Configuration Management
  Serial.println();
  Serial.println("3. Saving configuration to EEPROM...");
  if (motor->saveConfig()) {
    Serial.println("   ✅ Configuration saved successfully!");
  } else {
    Serial.println("   ❌ Configuration save failed");
  }
  
  // 4. Real-time Monitoring
  Serial.println();
  Serial.println("4. Real-time monitoring (5 seconds)...");
  unsigned long monitorStart = millis();
  while (millis() - monitorStart < 5000) {
    svarv.update();
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      const auto& status = motor->getStatus();
      Serial.println("   Pos: " + String(status.position, 2) + 
                    " rad, Vel: " + String(status.velocity, 2) + 
                    " rad/s, Current: " + String(status.current_q, 2) + " A");
    }
  }
  
  // 5. Error Handling Demo
  Serial.println();
  Serial.println("5. Error handling...");
  if (motor->hasError()) {
    Serial.println("   Current error: " + motor->getErrorString());
    Serial.println("   Clearing errors...");
    motor->clearErrors();
  } else {
    Serial.println("   No errors detected");
  }
  
  Serial.println();
  Serial.println("Advanced features demo complete!");
  Serial.println("===============================");
  Serial.println();
}

// Helper function to wait for serial input
String waitForSerialInput() {
  while (!Serial.available()) {
    svarv.update(); // Keep CAN system running
    delay(10);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  Serial.println(input); // Echo the input
  return input;
}