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
  else if (command == "status") {
    printCurrentPID();
    printPerformanceMetrics();
  }
  else if (command == "stop") {
    tuning.testing_active = false;
    motor.emergencyStop();
    Serial.println("Emergency stop - all tests cancelled");
  }
  else {
    Serial.println("Unknown command: " + command);
    Serial.println("Type 'help' for available commands");
  }
}

void updatePID() {
  motor.setPID(SVARV_PID_POSITION, tuning.current_p, tuning.current_i, tuning.current_d);
  printCurrentPID();
}

void startStepTest(float target) {
  Serial.println("=== STEP RESPONSE TEST ===");
  Serial.println("Target: " + String(target, 3) + " rad");
  Serial.println("Current PID: P=" + String(tuning.current_p) + ", I=" + String(tuning.current_i) + ", D=" + String(tuning.current_d));
  
  // Clear previous test data
  tuning.test_data.clear();
  resetPerformanceMetrics();
  
  // Record test parameters
  tuning.test_target = target;
  tuning.test_start_position = motor.getStatus().position;
  tuning.test_start_time = millis();
  tuning.testing_active = true;
  
  // Send move command
  motor.moveTo(target);
  
  Serial.println("Test started. Monitoring for 5 seconds...");
  Serial.println("Time(s)\tTarget\tPosition\tVelocity\tError");
}

void startSineTest() {
  Serial.println("=== SINE WAVE TRACKING TEST ===");
  Serial.println("Frequency: 0.1 Hz, Amplitude: 2 rad");
  Serial.println("Duration: 20 seconds");
  
  tuning.test_data.clear();
  tuning.testing_active = true;
  tuning.test_start_time = millis();
  
  Serial.println("Test started...");
}

void monitorStepTest() {
  unsigned long elapsed = millis() - tuning.test_start_time;
  
  // For sine wave test
  if (tuning.test_target == 0.0 && elapsed < 20000) {
    // Generate sine wave target
    float t = elapsed / 1000.0; // time in seconds
    float sine_target = 2.0 * sin(2.0 * PI * 0.1 * t); // 0.1 Hz, 2 rad amplitude
    motor.moveTo(sine_target);
    tuning.test_target = sine_target;
    return;
  }
  
  // End test after duration
  if (elapsed > 5000) {
    tuning.testing_active = false;
    analyzeTestResults();
  }
}

void updatePerformanceMetrics(const PIDTuningState::DataPoint& point) {
  float error_abs = fabs(point.error);
  
  // Calculate overshoot
  if (tuning.test_target > tuning.test_start_position) {
    // Positive step
    if (point.position > tuning.test_target) {
      float overshoot = ((point.position - tuning.test_target) / (tuning.test_target - tuning.test_start_position)) * 100.0;
      tuning.max_overshoot = max(tuning.max_overshoot, overshoot);
    }
  } else {
    // Negative step
    if (point.position < tuning.test_target) {
      float overshoot = ((tuning.test_target - point.position) / (tuning.test_start_position - tuning.test_target)) * 100.0;
      tuning.max_overshoot = max(tuning.max_overshoot, overshoot);
    }
  }
  
  // Calculate rise time (time to reach 90% of target)
  float step_size = fabs(tuning.test_target - tuning.test_start_position);
  float ninety_percent = step_size * 0.9;
  
  if (tuning.rise_time == 0 && fabs(point.position - tuning.test_start_position) >= ninety_percent) {
    tuning.rise_time = point.time;
  }
  
  // Calculate settling time (when error stays within 5% for 100ms)
  static unsigned long last_settle_check = 0;
  if (error_abs <= step_size * 0.05) {
    if (last_settle_check == 0) {
      last_settle_check = point.time;
    } else if (point.time - last_settle_check >= 100) {
      if (tuning.settling_time == 0) {
        tuning.settling_time = point.time / 1000.0; // Convert to seconds
      }
    }
  } else {
    last_settle_check = 0;
  }
  
  // Update steady state error (average of last 500ms)
  if (point.time > 4500) { // Last 500ms of 5-second test
    tuning.steady_state_error = (tuning.steady_state_error + error_abs) / 2.0;
  }
}

void resetPerformanceMetrics() {
  tuning.max_overshoot = 0.0;
  tuning.settling_time = 0.0;
  tuning.steady_state_error = 0.0;
  tuning.rise_time = 0;
}

void analyzeTestResults() {
  Serial.println();
  Serial.println("=== TEST RESULTS ===");
  
  if (tuning.test_data.empty()) {
    Serial.println("No test data collected");
    return;
  }
  
  Serial.print("Max Overshoot: ");
  Serial.print(tuning.max_overshoot, 1);
  Serial.println("%");
  
  Serial.print("Rise Time: ");
  Serial.print(tuning.rise_time);
  Serial.println(" ms");
  
  Serial.print("Settling Time: ");
  if (tuning.settling_time > 0) {
    Serial.print(tuning.settling_time, 2);
    Serial.println(" s");
  } else {
    Serial.println("Did not settle within test duration");
  }
  
  Serial.print("Steady State Error: ");
  Serial.print(tuning.steady_state_error, 4);
  Serial.println(" rad");
  
  // Performance assessment
  Serial.println();
  Serial.println("=== PERFORMANCE ASSESSMENT ===");
  
  if (tuning.max_overshoot > 50) {
    Serial.println("⚠️  High overshoot - Consider reducing P gain or adding D gain");
  } else if (tuning.max_overshoot < 5) {
    Serial.println("✅ Low overshoot - Good stability");
  }
  
  if (tuning.rise_time > 1000) {
    Serial.println("⚠️  Slow response - Consider increasing P gain");
  } else if (tuning.rise_time < 200) {
    Serial.println("✅ Fast response");
  }
  
  if (tuning.settling_time == 0) {
    Serial.println("❌ Did not settle - System may be unstable");
  } else if (tuning.settling_time > 3.0) {
    Serial.println("⚠️  Slow settling - Consider tuning I or D gains");
  } else {
    Serial.println("✅ Good settling time");
  }
  
  if (tuning.steady_state_error > 0.1) {
    Serial.println("⚠️  High steady-state error - Consider adding I gain");
  } else {
    Serial.println("✅ Low steady-state error");
  }
  
  // Tuning suggestions
  Serial.println();
  Serial.println("=== TUNING SUGGESTIONS ===");
  
  if (tuning.max_overshoot > 20 && tuning.current_d < 0.01) {
    Serial.println("💡 Try adding D gain (start with d0.01)");
  }
  
  if (tuning.steady_state_error > 0.05 && tuning.current_i == 0) {
    Serial.println("💡 Try adding I gain (start with i0.1)");
  }
  
  if (tuning.rise_time > 500 && tuning.max_overshoot < 10) {
    Serial.println("💡 Try increasing P gain by 20-50%");
  }
  
  if (tuning.max_overshoot > 30) {
    Serial.println("💡 Try reducing P gain by 20-30%");
  }
  
  Serial.println();
}

void printRealTimeStatus() {
  if (!tuning.testing_active) return;
  
  const auto& status = motor.getStatus();
  float error = tuning.test_target - status.position;
  float elapsed = (millis() - tuning.test_start_time) / 1000.0;
  
  Serial.print(elapsed, 2);
  Serial.print("\t");
  Serial.print(tuning.test_target, 3);
  Serial.print("\t");
  Serial.print(status.position, 3);
  Serial.print("\t");
  Serial.print(status.velocity, 2);
  Serial.print("\t");
  Serial.println(error, 4);
}

void printCurrentPID() {
  Serial.println("Current PID Values:");
  Serial.println("  P = " + String(tuning.current_p));
  Serial.println("  I = " + String(tuning.current_i));
  Serial.println("  D = " + String(tuning.current_d));
}

void printPerformanceMetrics() {
  Serial.println("Last Test Performance:");
  Serial.println("  Overshoot: " + String(tuning.max_overshoot, 1) + "%");
  Serial.println("  Rise Time: " + String(tuning.rise_time) + " ms");
  Serial.println("  Settling Time: " + String(tuning.settling_time, 2) + " s");
  Serial.println("  SS Error: " + String(tuning.steady_state_error, 4) + " rad");
}

void printHelp() {
  Serial.println("=== PID TUNING COMMANDS ===");
  Serial.println("Basic PID Control:");
  Serial.println("  p<value>  - Set P gain (e.g., p20, p50)");
  Serial.println("  i<value>  - Set I gain (e.g., i0.1, i0.5)");
  Serial.println("  d<value>  - Set D gain (e.g., d0.01, d0.05)");
  Serial.println();
  Serial.println("Testing:");
  Serial.println("  t<pos>    - Step test to position (e.g., t3.14)");
  Serial.println("  step      - Standard step test (π radians)");
  Serial.println("  sine      - Sine wave tracking test");
  Serial.println();
  Serial.println("Configuration:");
  Serial.println("  save      - Save PID values to motor EEPROM");
  Serial.println("  reset     - Reset to default PID values");
  Serial.println("  home      - Move motor to position 0");
  Serial.println();
  Serial.println("Information:");
  Serial.println("  status    - Show current PID and performance");
  Serial.println("  help      - Show this help");
  Serial.println("  stop      - Emergency stop");
  Serial.println();
  Serial.println("=== TUNING TIPS ===");
  Serial.println("1. Start with P-only control (I=0, D=0)");
  Serial.println("2. Increase P until slight overshoot appears");
  Serial.println("3. Add D gain to reduce overshoot");
  Serial.println("4. Add I gain only if steady-state error exists");
  Serial.println("5. Use step tests to evaluate each change");
  Serial.println("6. Save configuration when satisfied");
  Serial.println("===========================");
}