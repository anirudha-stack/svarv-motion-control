/**
 * @file Basic_Control_Two_Motors.ino
 * @brief Basic motor control example using Svarv Motion Control Library (2 motors)
 * @author Svarv Robotics
 *
 * This example demonstrates the basic usage of the Svarv Motion Control library
 * for controlling two BLDC motors via CAN bus, sending opposite commands to each.
 *
 * Hardware Required:
 * - ESP32 with built-in CAN (ESP32-C3, ESP32-S3, etc.)
 * - CAN transceiver (SN65HVD230 or similar)
 * - Two Svarv BLDC Motor Controllers with firmware v1.2+ (node IDs 1 and 2)
 *
 * Connections:
 * - CAN TX: GPIO 21
 * - CAN RX: GPIO 20
 * - 120Ω termination resistors on both ends of CAN bus
 *
 * BEHAVIOR:
 * 1. Initialize the CAN bus
 * 2. Connect to motors with node IDs 1 and 2
 * 3. Demonstrate position, velocity, and torque control on both motors,
 *    sending opposite targets to motor 2
 * 4. Show real-time status monitoring for both
 */

#include "SvarvMotionControl.h"

// Create motion control instance
SvarvMotionControl svarv;
SvarvMotor* motor1 = nullptr;
SvarvMotor* motor2 = nullptr;

// Control variables
unsigned long lastMoveTime = 0;
int moveStep = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Svarv Motion Control - Two Motor Example");
  Serial.println("========================================");

  // Enable debug output
  svarv.enableDebug(true);

  // Initialize CAN bus at 1 Mbps
  if (!svarv.begin(1000000)) {
    Serial.println("ERROR: Failed to initialize CAN bus!");
    Serial.println("Check your connections and try again.");
    while (1) delay(1000);
  }

  Serial.println("CAN bus initialized successfully!");

  // Add motors with node IDs 1 and 2
  motor1 = &svarv.addMotor(1);
  motor2 = &svarv.addMotor(2);

  // Configure limits and save for both motors
  motor1->setLimits(25, 15, 3);
  motor2->setLimits(25, 15, 3);
  delay(100);
  motor1->saveConfig();
  motor2->saveConfig();

  // Set up callbacks for both motors
  auto statusCb = [](SvarvMotor& m, const SvarvMotorStatus& s) {
    Serial.print("Motor ");
    Serial.print(m.getNodeId());
    Serial.print(" Status - Mode: ");
    Serial.print(svarvModeToString(s.control_mode));
    Serial.print(", Pos: ");
    Serial.print(s.position, 3);
    Serial.print(" rad, Vel: ");
    Serial.print(s.velocity, 2);
    Serial.print(" rad/s, En: ");
    Serial.println(s.enabled ? "Yes" : "No");
  };
  auto errorCb = [](SvarvMotor& m, SvarvErrorCode e, const String& msg) {
    Serial.print("ERROR on Motor ");
    Serial.print(m.getNodeId());
    Serial.print(": ");
    Serial.println(msg);
  };
  auto connCb = [](SvarvMotor& m, SvarvConnectionState o, SvarvConnectionState n) {
    Serial.print("Motor ");
    Serial.print(m.getNodeId());
    Serial.print(" connection: ");
    Serial.print(svarvConnectionStateToString(o));
    Serial.print(" -> ");
    Serial.println(svarvConnectionStateToString(n));
  };

  motor1->onStatusUpdate(statusCb);
  motor2->onStatusUpdate(statusCb);
  motor1->onError(errorCb);
  motor2->onError(errorCb);
  motor1->onConnectionChange(connCb);
  motor2->onConnectionChange(connCb);

  Serial.println("Waiting for both motors to connect...");
  unsigned long connectStart = millis();
  while ((!motor1->isConnected() || !motor2->isConnected()) && (millis() - connectStart < 10000)) {
    svarv.update();
    delay(100);
  }

  if (!motor1->isConnected() || !motor2->isConnected()) {
    Serial.println("ERROR: One or both motors not responding!");
    Serial.println("- Check power, CAN wiring, and node IDs (1 & 2).");
    while (1) delay(1000);
  }

  Serial.println("Both motors connected successfully!");
  Serial.println("");
  Serial.println("Starting movement demo (opposite commands)...");
  Serial.println("Steps: Position, Velocity, Torque (motor2 gets negative of motor1)");
  Serial.println("");
}

void loop() {
  // Process incoming CAN messages
  svarv.update();

  // Movement demo every 5 seconds
  if (millis() - lastMoveTime > 5000) {
    lastMoveTime = millis();

    if (!motor1->isConnected() || !motor2->isConnected()) {
      Serial.println("Motor disconnect detected - skipping movement");
      return;
    }

    float target = 0.0f;
    switch (moveStep) {
      case 0:
        Serial.println("=== Position Control Demo ===");
        motor1->setControlMode(SVARV_MODE_POSITION);
        motor2->setControlMode(SVARV_MODE_POSITION);
        target = 0.0f;
        Serial.println("Moving both to 0 rad");
        break;
      case 1:
        target = 3.14159f;
        Serial.println("Moving to ±π radians");
        break;
      case 2:
        target = -3.14159f;
        Serial.println("Moving to ∓π radians");
        break;
      case 3:
        target = 0.0f;
        Serial.println("Returning both to 0 rad");
        break;
      case 4:
        Serial.println("=== Velocity Control Demo ===");
        motor1->setControlMode(SVARV_MODE_VELOCITY);
        motor2->setControlMode(SVARV_MODE_VELOCITY);
        target = 2.0f;
        Serial.println("Spinning at ±2 rad/s");
        break;
      case 5:
        target = -2.0f;
        Serial.println("Reversing spin at ∓2 rad/s");
        break;
      case 6:
        target = 0.0f;
        Serial.println("Stopping both motors");
        break;
      case 7:
        Serial.println("=== Torque Control Demo ===");
        motor1->setControlMode(SVARV_MODE_TORQUE);
        motor2->setControlMode(SVARV_MODE_TORQUE);
        target = 0.1f;
        Serial.println("Applying ±0.1 Nm torque");
        break;
      case 8:
        target = -0.1f;
        Serial.println("Applying ∓0.1 Nm torque");
        break;
      case 9:
        target = 0.0f;
        Serial.println("Removing torque");
        break;
      case 10:
        Serial.println("=== Demo Complete ===");
        motor1->disable();
        motor2->disable();
        Serial.println("Motors disabled. Demo will restart in 5 seconds.");
        moveStep = -1;  // reset
        break;
    }

    // Issue commands with opposite values
    switch (moveStep) {
      case 0: case 3:
        motor1->moveTo(target);
        motor2->moveTo(-target);
        break;
      case 1: case 2:
        motor1->moveTo(target);
        motor2->moveTo(-target);
        break;
      case 4: case 5: case 6:
        motor1->setVelocity(target);
        motor2->setVelocity(-target);
        break;
      case 7: case 8: case 9:
        motor1->setTorque(target);
        motor2->setTorque(-target);
        break;
      default:
        // already handled disable
        break;
    }

    moveStep++;
  }

  // Print status every 2 seconds
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 2000) {
    lastStatusTime = millis();
    if (motor1->isConnected()) {
      auto& s1 = motor1->getStatus();
      Serial.print("M1 Status: ");
      Serial.print(svarvModeToString(s1.control_mode));
      Serial.print(", Pos=");
      Serial.print(s1.position, 2);
      Serial.print(" rad, Vel=");
      Serial.print(s1.velocity, 2);
      Serial.print(" rad/s");
      Serial.print(", Iq=");
      Serial.print(s1.current_q, 2);
      Serial.println(" A");
    }
    if (motor2->isConnected()) {
      auto& s2 = motor2->getStatus();
      Serial.print("M2 Status: ");
      Serial.print(svarvModeToString(s2.control_mode));
      Serial.print(", Pos=");
      Serial.print(s2.position, 2);
      Serial.print(" rad, Vel=");
      Serial.print(s2.velocity, 2);
      Serial.print(" rad/s");
      Serial.print(", Iq=");
      Serial.print(s2.current_q, 2);
      Serial.println(" A");
    }
  }
}
