#include <Arduino.h>
#include <algorithm>               // for std::find
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
bool canInitialized = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("========================================");
  Serial.println("🔧 Svarv Node ID Configuration Tool");
  Serial.println("========================================");
  Serial.println();
  
  printHelp();
}

void loop() {
  // Always update CAN system if initialized
  if (canInitialized) {
    svarv.update();
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    processCommand(command);
  }
}

void processCommand(const String& command) {
  if (command == "help" || command == "h") {
    printHelp();
    
  } else if (command == "init" || command == "i") {
    initializeCAN();
    
  } else if (command == "scan" || command == "s") {
    if (!checkCANInit()) return;
    scanForMotors();
    
  } else if (command.startsWith("set ")) {
    if (!checkCANInit()) return;
    String idStr = command.substring(4);
    int nodeId = idStr.toInt();
    setNodeId(nodeId);
    
  } else if (command.startsWith("save ")) {
    if (!checkCANInit()) return;
    String idStr = command.substring(5);
    int nodeId = idStr.toInt();
    saveMotorConfig(nodeId);
    
  } else if (command == "verify" || command == "v") {
    if (!checkCANInit()) return;
    verifyConfiguration();
    
  } else if (command == "status") {
    showSystemStatus();
    
  } else if (command.length() > 0) {
    Serial.println("❌ Unknown command: '" + command + "'");
    Serial.println("   Type 'help' to see available commands");
  }
}

void printHelp() {
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println();
  Serial.println("📋 Setup Commands:");
  Serial.println("  init      - Initialize CAN bus");
  Serial.println("  scan      - Scan for existing motors");
  Serial.println();
  Serial.println("🔧 Configuration Commands:");
  Serial.println("  set <id>  - Set node ID for unconfigured motor");
  Serial.println("            Example: 'set 2' to assign node ID 2");
  Serial.println("  save <id> - Save configuration to motor's EEPROM");
  Serial.println("            Example: 'save 2' to save config for motor 2");
  Serial.println();
  Serial.println("✅ Verification Commands:");
  Serial.println("  verify    - Verify all configured motors");
  Serial.println("  status    - Show system status");
  Serial.println();
  Serial.println("💡 Help:");
  Serial.println("  help      - Show this help menu");
  Serial.println();
  Serial.println("=== TYPICAL WORKFLOW ===");
  Serial.println("1. Connect ONLY ONE unconfigured motor to CAN bus");
  Serial.println("2. Type 'init' to initialize CAN communication");
  Serial.println("3. Type 'scan' to check for existing motors");
  Serial.println("4. Type 'set X' where X is desired node ID (1-255)");
  Serial.println("5. Type 'save X' to save the configuration");
  Serial.println("6. Type 'verify' to confirm the motor responds");
  Serial.println("7. Disconnect motor and repeat for next motor");
  Serial.println("=============================");
  Serial.println();
}

void initializeCAN() {
  Serial.println("🔄 Initializing CAN bus...");
 
  
  if (svarv.begin()) {
    canInitialized = true;
    Serial.println("✅ CAN bus initialized successfully!");
    Serial.println();
    Serial.println("💡 Next step: Type 'scan' to check for existing motors");
  } else {
    Serial.println("❌ CAN initialization failed!");
    Serial.println();
    Serial.println("🔍 Troubleshooting:");
    Serial.println("   - Check CAN transceiver connections");
    Serial.println("   - Verify 120Ω termination resistors");
    Serial.println("   - Check power supply to CAN transceiver");
    Serial.println("   - Ensure GPIO pins are not used by other peripherals");
  }
  Serial.println();
}

bool checkCANInit() {
  if (!canInitialized) {
    Serial.println("❌ CAN bus not initialized!");
    Serial.println("   Type 'init' first to initialize CAN communication");
    Serial.println();
    return false;
  }
  return true;
}

void scanForMotors() {
  Serial.println("🔍 Scanning for motors on CAN bus...");
  Serial.println("   Timeout: 3 seconds");
  
  // Clear existing motor list to get fresh scan
  auto currentMotors = svarv.getAllMotors();
  for (auto* motor : currentMotors) {
    if (motor) {
      svarv.removeMotor(motor->getNodeId());
    }
  }
  
  auto discovered = svarv.scanForMotors(3000);
  
  Serial.println();
  if (discovered.empty()) {
    Serial.println("📡 No configured motors found");
    Serial.println();
    Serial.println("💡 This could mean:");
    Serial.println("   - No motors are connected");
    Serial.println("   - Motors are unconfigured (node ID = 0)");
    Serial.println("   - CAN bus communication issues");
    Serial.println();
    Serial.println("💡 Next step: Connect one unconfigured motor and type 'set X'");
  } else {
    Serial.println("📡 Found " + String(discovered.size()) + " configured motor(s):");
    
    // Add discovered motors back to the system with fresh status
    for (uint8_t id : discovered) {
      SvarvMotor& motor = svarv.addMotor(id);
      
      // Give a moment for status update
      delay(100);
      svarv.update();
      
      Serial.print("   ✅ Motor at Node ID: " + String(id));
      if (motor.isConnected()) {
        const auto& status = motor.getStatus();
        Serial.print(" - Mode: " + svarvModeToString(status.control_mode));
        if (status.error_code != SVARV_ERROR_NONE) {
          Serial.print(" - ERROR: " + motor.getErrorString());
        }
      }
      Serial.println();
    }
    Serial.println();
    Serial.println("💡 These motors are configured and updated in system");
  }
  Serial.println();
}

void setNodeId(int nodeId) {
  if (nodeId < 1 || nodeId > 255) {
    Serial.println("❌ Invalid node ID: " + String(nodeId));
    Serial.println("   Node ID must be between 1 and 255");
    Serial.println();
    return;
  }
  
  // Check if node ID is already in use
  auto existing = svarv.scanForMotors(1000);
  if (std::find(existing.begin(), existing.end(), nodeId) != existing.end()) {
    Serial.println("⚠️  Node ID " + String(nodeId) + " is already in use!");
    Serial.println("   Choose a different node ID or disconnect the existing motor");
    Serial.println();
    return;
  }
  
  Serial.println("🔧 Setting Node ID to " + String(nodeId) + "...");
  Serial.println("⚠️  IMPORTANT: Ensure only ONE unconfigured motor is connected!");
  Serial.println();
  
  bool success = svarv.configureSingleMotor(nodeId);
  
  if (success) {
    Serial.println("✅ Node ID " + String(nodeId) + " configured successfully!");
    Serial.println();
    Serial.println("💡 Next step: Type 'save " + String(nodeId) + "' to save configuration");
  } else {
    Serial.println("❌ Failed to configure Node ID " + String(nodeId));
    Serial.println();
    Serial.println("🔍 Possible issues:");
    Serial.println("   - No unconfigured motor connected (node ID = 0)");
    Serial.println("   - Multiple unconfigured motors causing conflicts");
    Serial.println("   - CAN communication problems");
    Serial.println("   - Motor controller not responding");
  }
  Serial.println();
}

void saveMotorConfig(int nodeId) {
  if (nodeId < 1 || nodeId > 255) {
    Serial.println("❌ Invalid node ID: " + String(nodeId));
    Serial.println("   Node ID must be between 1 and 255");
    Serial.println();
    return;
  }
  
  Serial.println("💾 Saving configuration for motor " + String(nodeId) + "...");
  
  SvarvMotor* motor = svarv.getMotor(nodeId);
  if (!motor) {
    Serial.println("❌ Motor " + String(nodeId) + " not found!");
    Serial.println("   Use 'set " + String(nodeId) + "' first to configure the motor");
    Serial.println();
    return;
  }
  
  if (!motor->isConnected()) {
    Serial.println("❌ Motor " + String(nodeId) + " is not connected!");
    Serial.println("   Check CAN bus connections and power");
    Serial.println();
    return;
  }
  
  if (motor->saveConfig()) {
    Serial.println("✅ Configuration saved to motor " + String(nodeId) + " EEPROM!");
    Serial.println();
    Serial.println("💡 Motor will retain this node ID after power cycling");
    Serial.println("💡 Next step: Type 'verify' to confirm everything works");
  } else {
    Serial.println("❌ Failed to save configuration for motor " + String(nodeId));
    Serial.println("   Motor may not be responding properly");
  }
  Serial.println();
}

void verifyConfiguration() {
  Serial.println("🔍 Verifying motor configuration...");
  
  auto motors = svarv.scanForMotors(3000);
  
  if (motors.empty()) {
    Serial.println("❌ No motors found during verification!");
    Serial.println("   Check power and CAN connections");
  } else {
    Serial.println("✅ Verification complete:");
    Serial.println("   Found " + String(motors.size()) + " motor(s):");
    
    for (uint8_t id : motors) {
      SvarvMotor* motor = svarv.getMotor(id);
      Serial.print("   📡 Motor " + String(id) + ": ");
      
      if (motor && motor->isConnected()) {
        const auto& status = motor->getStatus();
        Serial.print("Connected, Mode: " + svarvModeToString(status.control_mode));
        if (status.error_code != SVARV_ERROR_NONE) {
          Serial.print(", ERROR: " + motor->getErrorString());
        }
        Serial.println();
      } else {
        Serial.println("Detected but not responding");
      }
    }
  }
  Serial.println();
}

void showSystemStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.println("CAN Initialized: " + String(canInitialized ? "Yes" : "No"));
  
  if (canInitialized) {
    Serial.println("Platform: " + svarv.getPlatformInfo());
    Serial.println("Connected Motors: " + String(svarv.getConnectedMotorCount()));
    Serial.println("CAN Health: " + String(svarv.isCANHealthy() ? "Good" : "Poor"));
    
    uint32_t sent, received, errors;
    svarv.getCANStatistics(sent, received, errors);
    Serial.println("CAN Messages - Sent: " + String(sent) + ", Received: " + String(received) + ", Errors: " + String(errors));
    
    auto motors = svarv.getAllMotors();
    if (!motors.empty()) {
      Serial.println();
      Serial.println("Motor List:");
      for (auto* motor : motors) {
        if (motor) {
          Serial.println("  Node " + String(motor->getNodeId()) + ": " + 
                        String(motor->isConnected() ? "Connected" : "Disconnected"));
        }
      }
    }
  }
  Serial.println("=====================");
  Serial.println();
}