/**
 * @file Advanced_Control_Patterns.ino
 * @brief Demonstrates advanced motion control patterns and techniques
 * @author Svarv Robotics
 * 
 * This example showcases sophisticated motion control techniques including:
 * - Trajectory generation and following
 * - Velocity profiling with acceleration limits
 * - Force control applications
 * - Impedance control for compliant motion
 * - Real-time motion adaptation
 * 
 * Hardware Required:
 * - ESP32 with CAN capability
 * - CAN transceiver
 * - 1-4 Svarv BLDC Motor Controllers
 * 
 * @section APPLICATIONS
 * 
 * This example demonstrates techniques useful for:
 * - Robotic arm control
 * - CNC machining operations
 * - Force-feedback systems
 * - Compliant manipulation
 * - High-precision positioning
 */

#include "SvarvMotionControl.h"
#include <math.h>

// Motion control system
SvarvMotionControl svarv;
SvarvMotor motor1, motor2;

// Motion control modes
enum MotionMode {
  MODE_TRAJECTORY_FOLLOWING,
  MODE_VELOCITY_PROFILING,
  MODE_FORCE_CONTROL,
  MODE_IMPEDANCE_CONTROL,
  MODE_SINE_TRACKING,
  MODE_MANUAL_CONTROL
};

MotionMode currentMode = MODE_TRAJECTORY_FOLLOWING;
unsigned long modeStartTime = 0;
int currentStep = 0;

// Trajectory generation parameters
struct TrajectoryPoint {
  float position;
  float velocity;
  float acceleration;
  unsigned long time_ms;
};

std::vector<TrajectoryPoint> trajectory;
int trajectoryIndex = 0;

// Velocity profiling parameters
struct VelocityProfile {
  float start_pos;
  float end_pos;
  float max_velocity;
  float acceleration;
  float deceleration;
  unsigned long start_time;
  
  // Calculated parameters
  float accel_time;
  float const_vel_time;
  float decel_time;
  float total_time;
};

VelocityProfile currentProfile;

// Force control parameters
struct ForceController {
  float target_force;           // Desired force [N]
  float current_force;          // Estimated current force [N]
  float force_gain;             // Force control gain
  float compliance_position;    // Compliant position reference
  float stiffness;              // Virtual stiffness [N/m]
  bool force_control_active;
};

ForceController forceCtrl = {0.0, 0.0, 0.1, 0.0, 100.0, false};

// Impedance control parameters
struct ImpedanceController {
  float desired_position;       // Desired position [rad]
  float virtual_mass;           // Virtual mass [kg⋅m²]
  float virtual_damping;        // Virtual damping [N⋅m⋅s/rad]
  float virtual_stiffness;      // Virtual stiffness [N⋅m/rad]
  float external_torque;        // Estimated external torque [N⋅m]
  
  // State variables
  float last_velocity;
  float last_acceleration;
  unsigned long last_update_time;
};

ImpedanceController impedanceCtrl = {0.0, 0.01, 0.1, 50.0, 0.0, 0.0, 0.0, 0};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Svarv Motion Control - Advanced Control Patterns");
  Serial.println("===============================================");
  Serial.println();
  
  // Initialize CAN bus
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: CAN initialization failed!");
    while (1) delay(1000);
  }
  
  Serial.println("Scanning for motors...");
  auto discovered = svarv.scanForMotors(3000);
  
  if (discovered.empty()) {
    Serial.println("ERROR: No motors found!");
    while (1) delay(1000);
  }
  
  // Setup motors
  motor1 = svarv.addMotor(discovered[0]);
  if (discovered.size() > 1) {
    motor2 = svarv.addMotor(discovered[1]);
  }
  
  Serial.println("Configuring motors for advanced control...");
  
  // Configure motor 1 for high-performance position control
  motor1.setControlMode(SVARV_MODE_POSITION);
  motor1.setPID(SVARV_PID_POSITION, 50.0, 0.5, 0.1);  // High-gain PID
  motor1.setLimits(20.0, 5.0, 12.0);  // High speed limits
  
  if (motor2.isConnected()) {
    motor2.setControlMode(SVARV_MODE_POSITION);
    motor2.setPID(SVARV_PID_POSITION, 40.0, 0.3, 0.08);
    motor2.setLimits(15.0, 4.0, 10.0);
  }
  
  // Wait for motors to be ready
  Serial.print("Waiting for motors to initialize");
  while (!motor1.isConnected() || motor1.getStatus().initializing) {
    svarv.update();
    Serial.print(".");
    delay(100);
  }
  Serial.println(" Ready!");
  
  // Initialize control modes
  generateSmoothTrajectory(0.0, 6.28, 5000); // 0 to 2π in 5 seconds
  setupVelocityProfile(0.0, 3.14, 2.0, 1.0, 1.5); // π rad move
  
  Serial.println();
  printModeHelp();
  
  modeStartTime = millis();
}

void loop() {
  svarv.update();
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Execute current control mode
  switch (currentMode) {
    case MODE_TRAJECTORY_FOLLOWING:
      executeTrajectoryFollowing();
      break;
      
    case MODE_VELOCITY_PROFILING:
      executeVelocityProfiling();
      break;
      
    case MODE_FORCE_CONTROL:
      executeForceControl();
      break;
      
    case MODE_IMPEDANCE_CONTROL:
      executeImpedanceControl();
      break;
      
    case MODE_SINE_TRACKING:
      executeSineTracking();
      break;
      
    case MODE_MANUAL_CONTROL:
      // Manual control via serial commands
      break;
  }
  
  // Print status periodically
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    printControlStatus();
  }
}

void executeTrajectoryFollowing() {
  if (trajectory.empty()) return;
  
  unsigned long elapsed = millis() - modeStartTime;
  
  // Find current trajectory point
  while (trajectoryIndex < trajectory.size() - 1 && 
         elapsed >= trajectory[trajectoryIndex + 1].time_ms) {
    trajectoryIndex++;
  }
  
  if (trajectoryIndex < trajectory.size()) {
    TrajectoryPoint& point = trajectory[trajectoryIndex];
    
    // Send position command
    motor1.moveTo(point.position);
    
    // Optional: Use velocity feedforward for smoother tracking
    // This would require direct velocity control capability
    
    // Check if trajectory is complete
    if (trajectoryIndex >= trajectory.size() - 1 && elapsed > point.time_ms + 1000) {
      Serial.println("Trajectory following complete!");
      currentMode = MODE_VELOCITY_PROFILING;
      modeStartTime = millis();
      currentStep = 0;
    }
  }
}

void executeVelocityProfiling() {
  unsigned long elapsed = millis() - modeStartTime;
  float t = elapsed / 1000.0; // Time in seconds
  
  float target_position = calculateVelocityProfilePosition(t);
  
  motor1.moveTo(target_position);
  
  // Check if profile is complete
  if (t > currentProfile.total_time + 1.0) {
    Serial.println("Velocity profiling complete!");
    currentMode = MODE_FORCE_CONTROL;
    modeStartTime = millis();
    forceCtrl.force_control_active = true;
    forceCtrl.compliance_position = motor1.getPosition();
  }
}

void executeForceControl() {
  // Estimate current force from motor current
  // Force = Kt * Current (simplified)
  const float torque_constant = 0.1; // [N⋅m/A]
  const float lever_arm = 0.1;       // [m] - assuming 10cm lever arm
  
  float motor_torque = torque_constant * motor1.getStatus().current_q;
  forceCtrl.current_force = motor_torque / lever_arm;
  
  // Force control law: adjust position based on force error
  float force_error = forceCtrl.target_force - forceCtrl.current_force;
  float position_adjustment = force_error * forceCtrl.force_gain;
  
  forceCtrl.compliance_position += position_adjustment * 0.001; // Small increments
  
  motor1.moveTo(forceCtrl.compliance_position);
  
  // Cycle through different target forces
  unsigned long elapsed = millis() - modeStartTime;
  if (elapsed > 3000) {
    static int force_step = 0;
    force_step = (force_step + 1) % 4;
    
    switch (force_step) {
      case 0: forceCtrl.target_force = 0.0; break;
      case 1: forceCtrl.target_force = 1.0; break;
      case 2: forceCtrl.target_force = -1.0; break;
      case 3: forceCtrl.target_force = 0.5; break;
    }
    
    modeStartTime = millis();
    
    if (force_step == 0) {
      Serial.println("Force control complete!");
      currentMode = MODE_IMPEDANCE_CONTROL;
      modeStartTime = millis();
      impedanceCtrl.last_update_time = millis();
    }
  }
}

void executeImpedanceControl() {
  unsigned long current_time = millis();
  float dt = (current_time - impedanceCtrl.last_update_time) / 1000.0;
  
  if (dt > 0.001) { // Update at most every 1ms
    const auto& status = motor1.getStatus();
    
    // Estimate external torque from tracking error and dynamics
    float position_error = impedanceCtrl.desired_position - status.position;
    float velocity_error = 0.0 - status.velocity; // Desired velocity = 0
    
    // Simple external torque estimation
    float expected_torque = impedanceCtrl.virtual_stiffness * position_error + 
                           impedanceCtrl.virtual_damping * velocity_error;
    
    // Current torque from motor
    const float torque_constant = 0.1;
    float actual_torque = torque_constant * status.current_q;
    
    impedanceCtrl.external_torque = actual_torque - expected_torque;
    
    // Impedance control law
    float acceleration = (impedanceCtrl.virtual_stiffness * position_error + 
                         impedanceCtrl.virtual_damping * velocity_error) / 
                         impedanceCtrl.virtual_mass;
    
    // Integrate to get desired velocity and position
    float desired_velocity = impedanceCtrl.last_velocity + acceleration * dt;
    float desired_position = impedanceCtrl.desired_position + desired_velocity * dt;
    
    motor1.moveTo(desired_position);
    
    impedanceCtrl.last_velocity = desired_velocity;
    impedanceCtrl.last_acceleration = acceleration;
    impedanceCtrl.last_update_time = current_time;
  }
  
  // Change desired position periodically
  unsigned long elapsed = millis() - modeStartTime;
  if (elapsed > 4000) {
    static int impedance_step = 0;
    impedance_step = (impedance_step + 1) % 3;
    
    switch (impedance_step) {
      case 0: impedanceCtrl.desired_position = 0.0; break;
      case 1: impedanceCtrl.desired_position = 2.0; break;
      case 2: impedanceCtrl.desired_position = -1.0; break;
    }
    
    modeStartTime = millis();
    
    if (impedance_step == 0) {
      Serial.println("Impedance control complete!");
      currentMode = MODE_SINE_TRACKING;
      modeStartTime = millis();
    }
  }
}

void executeSineTracking() {
  unsigned long elapsed = millis() - modeStartTime;
  float t = elapsed / 1000.0;
  
  // Multi-frequency sine wave
  float sine1 = 1.0 * sin(2.0 * PI * 0.2 * t);  // 0.2 Hz, 1 rad amplitude
  float sine2 = 0.5 * sin(2.0 * PI * 0.7 * t);  // 0.7 Hz, 0.5 rad amplitude
  float target = sine1 + sine2;
  
  motor1.moveTo(target);
  
  // If we have a second motor, make it track with phase offset
  if (motor2.isConnected()) {
    float target2 = 1.5 * sin(2.0 * PI * 0.3 * t + PI/2); // Phase shifted
    motor2.moveTo(target2);
  }
  
  if (elapsed > 15000) { // 15 second test
    Serial.println("Sine tracking complete!");
    currentMode = MODE_MANUAL_CONTROL;
    motor1.moveTo(0.0);
    if (motor2.isConnected()) motor2.moveTo(0.0);
  }
}

void generateSmoothTrajectory(float start_pos, float end_pos, unsigned long duration_ms) {
  trajectory.clear();
  
  const int num_points = 100;
  const float total_distance = end_pos - start_pos;
  
  for (int i = 0; i <= num_points; i++) {
    float t = (float)i / num_points; // 0 to 1
    unsigned long time_ms = (t * duration_ms);
    
    // Use S-curve (sigmoid) for smooth acceleration profile
    float s = smootherstep(t); // Smooth interpolation
    
    TrajectoryPoint point;
    point.time_ms = time_ms;
    point.position = start_pos + s * total_distance;
    
    // Calculate velocity (derivative of position)
    if (i > 0) {
      float dt = (time_ms - trajectory[i-1].time_ms) / 1000.0;
      point.velocity = (point.position - trajectory[i-1].position) / dt;
    } else {
      point.velocity = 0.0;
    }
    
    // Calculate acceleration (derivative of velocity)
    if (i > 0) {
      float dt = (time_ms - trajectory[i-1].time_ms) / 1000.0;
      point.acceleration = (point.velocity - trajectory[i-1].velocity) / dt;
    } else {
      point.acceleration = 0.0;
    }
    
    trajectory.push_back(point);
  }
  
  trajectoryIndex = 0;
  Serial.println("Generated smooth trajectory with " + String(trajectory.size()) + " points");
}

float smootherstep(float t) {
  // Smootherstep function: 6t^5 - 15t^4 + 10t^3
  // Provides smooth acceleration and deceleration
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

void setupVelocityProfile(float start_pos, float end_pos, float max_vel, float accel, float decel) {
  currentProfile.start_pos = start_pos;
  currentProfile.end_pos = end_pos;
  currentProfile.max_velocity = max_vel;
  currentProfile.acceleration = accel;
  currentProfile.deceleration = decel;
  currentProfile.start_time = millis();
  
  float distance = fabs(end_pos - start_pos);
  
  // Calculate time to reach max velocity
  currentProfile.accel_time = max_vel / accel;
  currentProfile.decel_time = max_vel / decel;
  
  // Distance covered during acceleration and deceleration
  float accel_distance = 0.5 * accel * currentProfile.accel_time * currentProfile.accel_time;
  float decel_distance = 0.5 * decel * currentProfile.decel_time * currentProfile.decel_time;
  
  if (accel_distance + decel_distance >= distance) {
    // Triangular profile (no constant velocity phase)
    float peak_vel = sqrt(distance * accel * decel / (accel + decel));
    currentProfile.accel_time = peak_vel / accel;
    currentProfile.decel_time = peak_vel / decel;
    currentProfile.const_vel_time = 0.0;
  } else {
    // Trapezoidal profile
    float const_vel_distance = distance - accel_distance - decel_distance;
    currentProfile.const_vel_time = const_vel_distance / max_vel;
  }
  
  currentProfile.total_time = currentProfile.accel_time + currentProfile.const_vel_time + currentProfile.decel_time;
  
  Serial.println("Velocity profile: Accel=" + String(currentProfile.accel_time, 2) + 
                "s, Const=" + String(currentProfile.const_vel_time, 2) + 
                "s, Decel=" + String(currentProfile.decel_time, 2) + "s");
}

float calculateVelocityProfilePosition(float t) {
  float position = currentProfile.start_pos;
  float direction = (currentProfile.end_pos > currentProfile.start_pos) ? 1.0 : -1.0;
  
  if (t <= currentProfile.accel_time) {
    // Acceleration phase
    position += direction * 0.5 * currentProfile.acceleration * t * t;
  } else if (t <= currentProfile.accel_time + currentProfile.const_vel_time) {
    // Constant velocity phase
    float accel_distance = 0.5 * currentProfile.acceleration * currentProfile.accel_time * currentProfile.accel_time;
    float const_time = t - currentProfile.accel_time;
    position += direction * (accel_distance + currentProfile.max_velocity * const_time);
  } else if (t <= currentProfile.total_time) {
    // Deceleration phase
    float accel_distance = 0.5 * currentProfile.acceleration * currentProfile.accel_time * currentProfile.accel_time;
    float const_distance = currentProfile.max_velocity * currentProfile.const_vel_time;
    float decel_time = t - currentProfile.accel_time - currentProfile.const_vel_time;
    float decel_distance = currentProfile.max_velocity * decel_time - 
                          0.5 * currentProfile.deceleration * decel_time * decel_time;
    position += direction * (accel_distance + const_distance + decel_distance);
  } else {
    // Move complete
    position = currentProfile.end_pos;
  }
  
  return position;
}

void processCommand(const String& command) {
  if (command == "1") {
    currentMode = MODE_TRAJECTORY_FOLLOWING;
    modeStartTime = millis();
    trajectoryIndex = 0;
    Serial.println("Mode: Trajectory Following");
  } else if (command == "2") {
    currentMode = MODE_VELOCITY_PROFILING;
    modeStartTime = millis();
    Serial.println("Mode: Velocity Profiling");
  } else if (command == "3") {
    currentMode = MODE_FORCE_CONTROL;
    modeStartTime = millis();
    forceCtrl.force_control_active = true;
    Serial.println("Mode: Force Control");
  } else if (command == "4") {
    currentMode = MODE_IMPEDANCE_CONTROL;
    modeStartTime = millis();
    impedanceCtrl.last_update_time = millis();
    Serial.println("Mode: Impedance Control");
  } else if (command == "5") {
    currentMode = MODE_SINE_TRACKING;
    modeStartTime = millis();
    Serial.println("Mode: Sine Tracking");
  } else if (command == "6") {
    currentMode = MODE_MANUAL_CONTROL;
    Serial.println("Mode: Manual Control");
  } else if (command == "stop") {
    motor1.emergencyStop();
    if (motor2.isConnected()) motor2.emergencyStop();
    Serial.println("Emergency stop!");
  } else if (command == "home") {
    motor1.moveTo(0.0);
    if (motor2.isConnected()) motor2.moveTo(0.0);
    Serial.println("Homing motors...");
  } else if (command == "help") {
    printModeHelp();
  } else if (command.startsWith("pos")) {
    float pos = command.substring(3).toFloat();
    motor1.moveTo(pos);
    Serial.println("Moving to position: " + String(pos));
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void printModeHelp() {
  Serial.println("=== ADVANCED CONTROL MODES ===");
  Serial.println("1 - Trajectory Following");
  Serial.println("2 - Velocity Profiling");
  Serial.println("3 - Force Control");
  Serial.println("4 - Impedance Control");
  Serial.println("5 - Sine Wave Tracking");
  Serial.println("6 - Manual Control");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  pos<value> - Move to position (manual mode)");
  Serial.println("  home       - Home all motors");
  Serial.println("  stop       - Emergency stop");
  Serial.println("  help       - Show this help");
  Serial.println("===============================");
}

void printControlStatus() {
  String mode_name;
  switch (currentMode) {
    case MODE_TRAJECTORY_FOLLOWING: mode_name = "Trajectory"; break;
    case MODE_VELOCITY_PROFILING: mode_name = "Velocity"; break;
    case MODE_FORCE_CONTROL: mode_name = "Force"; break;
    case MODE_IMPEDANCE_CONTROL: mode_name = "Impedance"; break;
    case MODE_SINE_TRACKING: mode_name = "Sine"; break;
    case MODE_MANUAL_CONTROL: mode_name = "Manual"; break;
  }
  
  Serial.print("Mode: " + mode_name);
  Serial.print(" | M1: " + String(motor1.getPosition(), 3) + " rad");
  Serial.print(" (" + String(motor1.getVelocity(), 2) + " rad/s)");
  
  if (motor2.isConnected()) {
    Serial.print(" | M2: " + String(motor2.getPosition(), 3) + " rad");
  }
  
  if (currentMode == MODE_FORCE_CONTROL) {
    Serial.print(" | Force: " + String(forceCtrl.current_force, 2) + "N");
    Serial.print(" (Target: " + String(forceCtrl.target_force, 1) + "N)");
  }
  
  Serial.println();
}