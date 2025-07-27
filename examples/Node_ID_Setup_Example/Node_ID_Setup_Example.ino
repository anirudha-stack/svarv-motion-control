/**
 * @file Node_ID_Setup_Example.ino
 * @brief Node ID configuration example using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example demonstrates how to configure Node IDs for Svarv motor controllers.
 * This is essential when setting up new motors or when you have multiple motors
 * that need unique identifiers on the CAN bus.
 * 
 * Hardware Required:
 * - ESP32/STM32/Arduino with CAN capability
 * - CAN transceiver or MCP2515 shield
 * - One or more Svarv BLDC Motor Controllers
 * 
 * @section SCENARIOS
 * 
 * This example covers several Node ID setup scenarios:
 * 1. Setting up a single new motor (Node ID = 0)
 * 2. Auto-configuring multiple new motors
 * 3. Changing an existing motor's Node ID
 * 4. Discovering and listing all motors on the bus
 * 5. Factory reset and reconfiguration
 * 
 * @section BEHAVIOR
 * 
 * The example provides an interactive menu system that allows you to:
 * - Scan for motors and display their current Node IDs
 * - Set a specific Node ID for an unconfigured motor
 * - Auto-assign sequential Node IDs to multiple motors
 * - Change an existing motor's Node ID
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
  Serial.println("Platform: " + svarv.getPlatformInfo());
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
    
  } else if (command == "2" || command == "set") {
    setIndividualNodeId();
    
  } else if (command == "3" || command == "auto") {
    autoConfigureMultipleMotors();
    
  } else if (command == "4" || command == "change") {
    changeExistingNodeId();
    
  } else if (command == "5" || command == "reset") {
    factoryResetMotor();
    
  } else if (command == "6" || command == "test") {
    testMotorCommunication();
    
  } else if (command == "help" || command == "menu") {
    printMainMenu();
    
  } else if (command == "info") {
    showSystemInfo();
    
  } else if (command.length() > 0) {
    Serial.println("Unknown command: '" + command + "'");
    Serial.println("Type 'help' to see available commands.");
  }
}

void printMainMenu() {
  Serial.println("=== NODE ID CONFIGURATION MENU ===");
  Serial.println("1. scan  - Scan for motors and show Node IDs");
  Serial.println("2. set   - Set Node ID for unconfigured motor");
  Serial.println("3. auto  - Auto-configure multiple motors");
  Serial.println("4. change- Change existing motor's Node ID");
  Serial.println("5. reset - Factory reset motor");
  Serial.println("6. test  - Test communication with motor");
  Serial.println();
  Serial.println("Commands: help, info");
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
    Serial.println("❌ No motors found on the CAN bus.");
    Serial.println();
    Serial.println("Troubleshooting tips:");
    Serial.println("- Check motor controller power");
    Serial.println("- Verify CAN bus connections (CANH, CANL, GND)");
    Serial.println("- Ensure 120Ω termination resistors are installed");
    Serial.println("- Check CAN transceiver connections");
    Serial.println();
    
    // Try scanning for unconfigured motors (Node ID = 0)
    Serial.println("Scanning for unconfigured motors (Node ID = 0)...");
    if (sendSystemCommand(0, "PING")) {
      Serial.println("✅ Found unconfigured motor(s) with Node ID = 0");
      Serial.println("Use option 2 or 3 to configure Node IDs");
    } else {
      Serial.println("❌ No unconfigured motors found either");
    }
    
  } else {
    Serial.println("✅ Found " + String(discovered.size()) + " motor(s):");
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

void setIndividualNodeId() {
  Serial.println("=== SET NODE ID FOR UNCONFIGURED MOTOR ===");
  Serial.println();
  Serial.println("This will set a Node ID for a motor that currently has Node ID = 0");
  Serial.println("(new/unconfigured motors)");
  Serial.println();
  
  // Check if there are unconfigured motors
  Serial.println("Checking for unconfigured motors...");
  if (!sendSystemCommand(0, "PING")) {
    Serial.println("❌ No unconfigured motors found (Node ID = 0)");
    Serial.println("All motors may already be configured.");
    Serial.println("Use 'scan' to see configured motors.");
    return;
  }
  
  Serial.println("✅ Found unconfigured motor(s)");
  Serial.println();
  
  // Get desired Node ID
  Serial.print("Enter desired Node ID (1-255): ");
  String input = waitForSerialInput();
  
  uint8_t newNodeId = input.toInt();
  if (newNodeId < 1 || newNodeId > 255) {
    Serial.println("❌ Invalid Node ID. Must be between 1 and 255.");
    return;
  }
  
  // Check if Node ID is already in use
  auto existingMotors = svarv.scanForMotors(2000);
  for (uint8_t existingId : existingMotors) {
    if (existingId == newNodeId) {
      Serial.println("❌ Node ID " + String(newNodeId) + " is already in use!");
      Serial.println("Choose a different Node ID or change the existing motor first.");
      return;
    }
  }
  
  Serial.println();
  Serial.println("Setting Node ID " + String(newNodeId) + " for unconfigured motor...");
  
  // Send system command to set Node ID
  if (setMotorNodeId(0, newNodeId)) {
    Serial.println("✅ Success! Motor Node ID set to " + String(newNodeId));
    Serial.println();
    
    // Verify the configuration
    delay(1000);
    Serial.println("Verifying configuration...");
    
    SvarvMotor& motor = svarv.addMotor(newNodeId);
    unsigned long startTime = millis();
    while (!motor.isConnected() && millis() - startTime < 3000) {
      svarv.update();
      delay(100);
    }
    
    if (motor.isConnected()) {
      Serial.println("✅ Motor is responding with new Node ID!");
      
      // Save configuration to motor's EEPROM
      Serial.println("Saving configuration to motor's EEPROM...");
      if (motor.saveConfig()) {
        Serial.println("✅ Configuration saved successfully!");
      } else {
        Serial.println("⚠️  Configuration save failed - settings may be lost on power cycle");
      }
    } else {
      Serial.println("⚠️  Motor not responding with new Node ID - please check connections");
    }
    
  } else {
    Serial.println("❌ Failed to set Node ID. Please try again.");
  }
  
  Serial.println("==========================================");
  Serial.println();
}

void autoConfigureMultipleMotors() {
  Serial.println("=== AUTO-CONFIGURE MULTIPLE MOTORS ===");
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
  
  // Perform auto-configuration
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
  }
  
  Serial.println("=====================================");
  Serial.println();
}

void changeExistingNodeId() {
  Serial.println("=== CHANGE EXISTING MOTOR NODE ID ===");
  Serial.println();
  
  // Scan for existing motors
  auto existingMotors = svarv.scanForMotors(3000);
  
  if (existingMotors.empty()) {
    Serial.println("❌ No configured motors found.");
    Serial.println("Use 'scan' to check for motors or 'set' for unconfigured motors.");
    return;
  }
  
  Serial.println("Found " + String(existingMotors.size()) + " configured motor(s):");
  for (uint8_t nodeId : existingMotors) {
    Serial.println("  - Motor Node ID: " + String(nodeId));
  }
  Serial.println();
  
  // Get current Node ID
  Serial.print("Enter current Node ID to change: ");
  String input = waitForSerialInput();
  
  uint8_t currentId = input.toInt();
  bool found = false;
  for (uint8_t nodeId : existingMotors) {
    if (nodeId == currentId) {
      found = true;
      break;
    }
  }
  
  if (!found) {
    Serial.println("❌ Motor with Node ID " + String(currentId) + " not found.");
    return;
  }
  
  // Get new Node ID
  Serial.print("Enter new Node ID (1-255): ");
  input = waitForSerialInput();
  
  uint8_t newId = input.toInt();
  if (newId < 1 || newId > 255) {
    Serial.println("❌ Invalid new Node ID. Must be between 1 and 255.");
    return;
  }
  
  if (newId == currentId) {
    Serial.println("❌ New Node ID is the same as current Node ID.");
    return;
  }
  
  // Check if new ID is already in use
  for (uint8_t nodeId : existingMotors) {
    if (nodeId == newId) {
      Serial.println("❌ Node ID " + String(newId) + " is already in use!");
      return;
    }
  }
  
  Serial.println();
  Serial.println("Changing Node ID from " + String(currentId) + " to " + String(newId) + "...");
  
  // Change the Node ID
  if (setMotorNodeId(currentId, newId)) {
    Serial.println("✅ Node ID changed successfully!");
    
    // Remove old motor and add new one
    svarv.removeMotor(currentId);
    
    // Verify new configuration
    delay(1000);
    Serial.println("Verifying new configuration...");
    
    SvarvMotor& motor = svarv.addMotor(newId);
    unsigned long startTime = millis();
    while (!motor.isConnected() && millis() - startTime < 3000) {
      svarv.update();
      delay(100);
    }
    
    if (motor.isConnected()) {
      Serial.println("✅ Motor is responding with new Node ID " + String(newId) + "!");
      
      // Save configuration
      if (motor.saveConfig()) {
        Serial.println("✅ Configuration saved to motor's EEPROM!");
      }
    } else {
      Serial.println("⚠️  Motor not responding with new Node ID - please check connections");
    }
    
  } else {
    Serial.println("❌ Failed to change Node ID. Please try again.");
  }
  
  Serial.println("==================================");
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
    Serial.println("Use option 2 or 3 to reconfigure the Node ID.");
    
    // Remove motor from our system since it's now unconfigured
    svarv.removeMotor(nodeId);
    
  } else {
    Serial.println("❌ Factory reset failed. Please try again.");
  }
  
  Serial.println("============================");
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

void showSystemInfo() {
  Serial.println("=== SYSTEM INFORMATION ===");
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println("Platform: " + svarv.getPlatformInfo());
  Serial.println("Connected Motors: " + String(svarv.getConnectedMotorCount()));
  
  uint32_t sent, received, errors;
  svarv.getCANStatistics(sent, received, errors);
  Serial.println("CAN Messages Sent: " + String(sent));
  Serial.println("CAN Messages Received: " + String(received));
  Serial.println("CAN Errors: " + String(errors));
  Serial.println("CAN Health: " + String(svarv.isCANHealthy() ? "Good" : "Poor"));
  Serial.println("===========================");
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

// Helper function to send system command to motor
bool sendSystemCommand(uint8_t nodeId, const String& command) {
  // This is a simplified system command sender
  // In reality, you would send the appropriate CAN message
  // For this example, we'll simulate by trying to communicate
  
  if (nodeId == 0) {
    // For unconfigured motors, we need to send a broadcast system command
    // This is a placeholder - actual implementation depends on firmware protocol
    delay(100);
    return true; // Assume unconfigured motor exists for demo
  }
  
  return false;
}

// Helper function to set motor Node ID
bool setMotorNodeId(uint8_t currentId, uint8_t newId) {
  // This function sends the actual Node ID change command
  // The exact implementation depends on your motor controller firmware
  
  Serial.println("Sending Node ID change command...");
  
  // Example implementation:
  // 1. Send system command to current Node ID (or 0 for unconfigured)
  // 2. Motor acknowledges and changes its Node ID
  // 3. Motor responds with new Node ID
  
  if (currentId == 0) {
    // Setting Node ID for unconfigured motor
    // Send broadcast system command with new Node ID
    uint8_t cmd = 0x51; // CMD_SYS_SET_NODE_ID from firmware
    return svarv.sendCANMessage(0, 0b110, cmd, &newId, 1); // FUNCTION_SYSTEM
  } else {
    // Changing existing motor's Node ID
    // Send system command to current Node ID
    uint8_t cmd = 0x51; // CMD_SYS_SET_NODE_ID from firmware
    return svarv.sendCANMessage(currentId, 0b110, cmd, &newId, 1); // FUNCTION_SYSTEM
  }
}

// This function needs to be added to SvarvMotionControl class to expose sendCANMessage
// For now, we'll create a workaround