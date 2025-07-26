/**
 * @file PID_Tuning.ino
 * @brief Interactive PID tuning tool using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example provides an interactive tool for tuning PID parameters on Svarv motors.
 * It demonstrates advanced features like real-time parameter adjustment, step response
 * testing, and performance analysis.
 * 
 * Hardware Required:
 * - ESP32 with built-in CAN
 * - CAN transceiver
 * - Svarv BLDC Motor Controller
 * 
 * @section USAGE
 * 
 * Use the Serial monitor to send commands:
 * - 'p<value>' - Set position P gain (e.g., p20)
 * - 'i<value>' - Set position I gain (e.g., i0.1)
 * - 'd<value>' - Set position D gain (e.g., d0.05)
 * - 't<pos>' - Test step response to position (e.g., t3.14)
 * - 'step' - Automated step response test
 * - 'sine' - Sine wave tracking test
 * - 'save' - Save current PID values to motor
 * - 'reset' - Reset to default PID values
 * - 'help' - Show all commands
 */

#include "SvarvMotionControl.h"

// Motion control system
SvarvMotionControl svarv;
SvarvMotor motor;

// PID tuning state
struct PIDTuningState {
  float current_p = 20.0;
  float current_i = 0.0;
  float current_d = 0.0;
  
  bool testing_active = false;
  unsigned long test_start_time = 0;
  float test_target = 0.0;
  float test_start_position = 0.0;
  
  // Performance metrics
  float max_overshoot = 0.0;
  float settling_time = 0.0;
  float steady_state_error = 0.0;
  unsigned long rise_time = 0;
  
  // Data logging
  struct DataPoint {
    unsigned long time;
    float target;
    float position;
    float velocity;
    float error;
  };
  
  std::vector<DataPoint> test_data;
} tuning;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Svarv Motion Control - PID Tuning Tool");
  Serial.println("======================================");
  Serial.println();
  
  // Initialize CAN bus
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    while (1) delay(1000);
  }
  
  Serial.println("CAN bus initialized. Scanning for motors...");
  
  // Auto-discover motors
  auto discovered = svarv.scanForMotors(3000);
  
  if (discovered.empty()) {
    Serial.println("ERROR: No motors found!");
    Serial.println("Please check connections and try again.");
    while (1) delay(1000);
  }
  
  // Use the first discovered motor
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
  
  // Set initial PID values
  motor.setPID(SVARV_PID_POSITION, tuning.current_p, tuning.current_i, tuning.current_d);
  
  // Set up real-time monitoring
  motor.onStatusUpdate([](SvarvMotor& m, const SvarvMotorStatus& status) {
    // Log data during testing
    if (tuning.testing_active) {
      PIDTuningState::DataPoint point;
      point.time = millis() - tuning.test_start_time;
      point.target = tuning.test_target;
      point.position = status.position;
      point.velocity = status.velocity;
      point.error = tuning.test_target - status.position;
      
      tuning.test_data.push_back(point);
      
      // Update performance metrics
      updatePerformanceMetrics(point);
    }
  });
  
  motor.onError([](SvarvMotor& m, SvarvErrorCode error, const String& message) {
    Serial.println("MOTOR ERROR: " + message);
    tuning.testing_active = false;
  });
  
  // Move to home position
  Serial.println("Homing motor to position 0...");
  motor.moveTo(0.0);
  delay(2000);
  
  Serial.println();
  printHelp();
  printCurrentPID();
  Serial.println();
  Serial.println("Ready for PID tuning! Enter commands:");
}

void loop() {
  // Always update the system
  svarv.update();
  
  // Process serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    processCommand(command);
  }
  
  // Monitor active tests
  if (tuning.testing_active) {
    monitorStepTest();
  }
  
  // Print real-time status
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    printRealTimeStatus();
  }
}

void processCommand(const String& command) {
  if (command.startsWith("p")) {
    float p_value = command.substring(1).toFloat();
    if (p_value > 0 && p_value <= 1000) {
      tuning.current_p = p_value;
      updatePID();
      Serial.println("Position P gain set to: " + String(p_value));
    } else {
      Serial.println("ERROR: P gain must be between 0 and 1000");
    }
  }
  else if (command.startsWith("i")) {
    float i_value = command.substring(1).toFloat();
    if (i_value >= 0 && i_value <= 100) {
      tuning.current_i = i_value;
      updatePID();
      Serial.println("Position I gain set to: " + String(i_value));
    } else {
      Serial.println("ERROR: I gain must be between 0 and 100");
    }
  }
  else if (command.startsWith("d")) {
    float d_value = command.substring(1).toFloat();
    if (d_value >= 0 && d_value <= 10) {
      tuning.current_d = d_value;
      updatePID();
      Serial.println("Position D gain set to: " + String(d_value));
    } else {
      Serial.println("ERROR: D gain must be between 0 and 10");
    }
  }
  else if (command.startsWith("t")) {
    float target = command.substring(1).toFloat();
    if (fabs(target) <= 10) {
      startStepTest(target);
    } else {
      Serial.println("ERROR: Target must be between -10 and +10 radians");
    }
  }
  else if (command == "step") {
    startStepTest(3.14159); // π radians step
  }
  else if (command == "sine") {
    startSineTest();
  }
  else if (command == "save") {
    motor.saveConfig();
    Serial.println("PID values saved to motor EEPROM");
  }
  else if (command == "reset") {
    tuning.current_p = 20.0;
    tuning.current_i = 0.0;
    tuning.current_d = 0.0;
    updatePID();
    Serial.println("PID values reset to defaults");
  }
  else if (command == "home") {
    motor.moveTo(0.0);
    Serial.println("Homing motor to position 0");
  }
  else if (command == "help") {
    printHelp();
  }
  else