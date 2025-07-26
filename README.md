# Svarv Motion Control Library

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/)
[![CAN Bus](https://img.shields.io/badge/Interface-CAN%20Bus-orange.svg)](https://en.wikipedia.org/wiki/CAN_bus)
[![Version](https://img.shields.io/badge/Version-1.0.0-brightgreen.svg)](https://github.com/svarv-robotics/svarv-motion-control)

A comprehensive Arduino library for controlling CAN-based BLDC motor controllers. Inspired by industry-leading libraries like SimpleFOC, ODrive, and VESC, this library provides a simple yet powerful interface for advanced motor control applications.

## 🚀 Features

### **Multi-Motor Control**
- Control up to 255 motors on a single CAN bus
- Automatic motor discovery and configuration
- Synchronized coordinated movements
- Individual motor status monitoring

### **Advanced Control Modes**
- **Position Control** - Precise angular positioning with configurable PID
- **Velocity Control** - Speed regulation with smooth acceleration profiles  
- **Torque Control** - Direct torque/current control for force applications
- **Calibration Mode** - Automatic sensor and motor parameter calibration

### **Real-Time Monitoring**
- Live telemetry data (position, velocity, current, voltage)
- Connection state monitoring with automatic reconnection
- Comprehensive error handling and reporting
- CAN bus health monitoring and statistics

### **Advanced Features**
- **Interactive PID Tuning** - Real-time parameter adjustment with performance analysis
- **Non-blocking Operations** - Asynchronous communication with callback support
- **Configuration Management** - Save/load parameters to motor EEPROM
- **Safety Systems** - Emergency stops, timeout protection, limit enforcement

## 📋 Requirements

### Hardware
- **ESP32** with built-in CAN controller (ESP32-C3, ESP32-S3, ESP32 Classic)
- **CAN Transceiver** (SN65HVD230, TJA1050, or similar)
- **Svarv BLDC Motor Controller(s)** with firmware v1.2 or later
- **120Ω termination resistors** at both ends of CAN bus

### Software
- **Arduino IDE** 1.8.0+ or **PlatformIO**
- **ESP32-TWAI-CAN library** (automatically installed)

## 🔧 Installation

### Method 1: Arduino Library Manager
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "Svarv Motion Control"
4. Click **Install**

### Method 2: Manual Installation
1. Download the latest release from [GitHub](https://github.com/svarv-robotics/svarv-motion-control)
2. Extract to your Arduino `libraries` folder
3. Restart Arduino IDE

### Method 3: PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    svarv-robotics/Svarv Motion Control@^1.0.0
```

## 🚀 Quick Start

### Basic Single Motor Control

```cpp
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor motor1;

void setup() {
  Serial.begin(115200);
  
  // Initialize CAN bus at 1 Mbps
  if (!svarv.begin(1000000)) {
    Serial.println("CAN initialization failed!");
    while(1);
  }
  
  // Add motor with node ID 1
  motor1 = svarv.addMotor(1);
  
  // Set position control mode
  motor1.setControlMode(SVARV_MODE_POSITION);
}

void loop() {
  // IMPORTANT: Call update() frequently!
  svarv.update();
  
  // Move to different positions
  motor1.moveTo(3.14159); // Move to π radians
  delay(3000);
  
  motor1.moveTo(0.0);     // Return to home
  delay(3000);
}
```

### Multi-Motor Coordination

```cpp
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor motor1, motor2, motor3;

void setup() {
  Serial.begin(115200);
  svarv.begin(1000000);
  
  // Auto-discover motors
  auto discovered = svarv.scanForMotors(5000);
  
  // Add discovered motors
  for (uint8_t nodeId : discovered) {
    Serial.println("Found motor: " + String(nodeId));
  }
  
  motor1 = svarv.addMotor(1);
  motor2 = svarv.addMotor(2);
  motor3 = svarv.addMotor(3);
  
  // Configure all for position control
  motor1.setControlMode(SVARV_MODE_POSITION);
  motor2.setControlMode(SVARV_MODE_POSITION);
  motor3.setControlMode(SVARV_MODE_POSITION);
}

void loop() {
  svarv.update();
  
  // Synchronized movement
  motor1.moveTo(1.0);
  motor2.moveTo(2.0);
  motor3.moveTo(3.0);
  
  delay(5000);
  
  // Return all to home
  motor1.moveTo(0.0);
  motor2.moveTo(0.0);
  motor3.moveTo(0.0);
  
  delay(5000);
}
```

## 📖 Detailed Examples

The library includes comprehensive examples demonstrating various features:

### [Basic_Control.ino](examples/Basic_Control/Basic_Control.ino)
- Single motor control
- Different control modes (position, velocity, torque)
- Real-time status monitoring
- Error handling

### [Multi_Motor_Control.ino](examples/Multi_Motor_Control/Multi_Motor_Control.ino)
- Multiple motor coordination
- Synchronized movements
- System health monitoring
- Advanced movement patterns

### [PID_Tuning.ino](examples/PID_Tuning/PID_Tuning.ino)
- Interactive PID parameter tuning
- Step response testing
- Performance analysis
- Automated tuning suggestions

## 🎛️ Control Modes

### Position Control
Precise angular positioning with PID feedback control.

```cpp
motor.setControlMode(SVARV_MODE_POSITION);
motor.moveTo(3.14159);  // Move to π radians
motor.moveBy(1.0);      // Move 1 radian relative

// Check if target reached
if (motor.isAtTarget(0.1)) {
  Serial.println("Target reached!");
}

// Wait for target (blocking)
motor.waitForTarget(5000, 0.05); // 5s timeout, 0.05 rad tolerance
```

### Velocity Control
Continuous speed regulation with smooth acceleration.

```cpp
motor.setControlMode(SVARV_MODE_VELOCITY);
motor.setVelocity(5.0);   // 5 rad/s clockwise
motor.setVelocity(-2.0);  // 2 rad/s counter-clockwise
motor.setVelocity(0.0);   // Stop
```

### Torque Control
Direct torque/current control for force applications.

```cpp
motor.setControlMode(SVARV_MODE_TORQUE);
motor.setTorque(0.5);   // 0.5 Nm torque
motor.setTorque(-0.2);  // -0.2 Nm reverse torque
motor.setTorque(0.0);   // No torque
```

## ⚙️ Configuration & Tuning

### PID Controller Tuning

```cpp
// Set position PID parameters
motor.setPID(SVARV_PID_POSITION, 20.0, 0.1, 0.05);
//           P gain ^^^^  ^^^I  ^^^D

// Set individual gains
motor.setPID(SVARV_PID_POSITION, 25.0, 0.0, 0.0); // P-only control

// Different controllers
motor.setPID(SVARV_PID_VELOCITY, 0.5, 10.0, 0.0); // Velocity PID
motor.setPID(SVARV_PID_CURRENT, 3.0, 300.0, 0.0); // Current PID
```

### Motor Limits

```cpp
// Set safety limits
motor.setLimits(15.0,  // Max velocity [rad/s]
                5.0,   // Max current [A]
                12.0); // Max voltage [V]
```

### Configuration Persistence

```cpp
// Save current settings to motor's EEPROM
motor.saveConfig();

// Load settings from EEPROM
motor.loadConfig();

// Reset to factory defaults
motor.factoryReset();
```

## 📊 Monitoring & Callbacks

### Real-Time Status Monitoring

```cpp
// Get current status
const SvarvMotorStatus& status = motor.getStatus();
Serial.println("Position: " + String(status.position));
Serial.println("Velocity: " + String(status.velocity));
Serial.println("Current: " + String(status.current_q));

// Quick status checks
bool connected = motor.isConnected();
bool enabled = motor.isEnabled();
bool hasError = motor.hasError();
```

### Event Callbacks

```cpp
// Status update callback
motor.onStatusUpdate([](SvarvMotor& m, const SvarvMotorStatus& status) {
  Serial.println("Motor " + String(m.getNodeId()) + " updated");
});

// Error callback
motor.onError([](SvarvMotor& m, SvarvErrorCode error, const String& message) {
  Serial.println("ERROR: " + message);
  m.emergencyStop(); // Auto-stop on error
});

// Connection state callback
motor.onConnectionChange([](SvarvMotor& m, SvarvConnectionState oldState, SvarvConnectionState newState) {
  Serial.println("Connection: " + svarvConnectionStateToString(newState));
});
```

### System-Wide Monitoring

```cpp
// Global error handling
svarv.onAnyError([](SvarvMotor& motor, SvarvErrorCode error, const String& message) {
  Serial.println("System error on motor " + String(motor.getNodeId()) + ": " + message);
  svarv.emergencyStopAll(); // Stop all motors on any error
});

// CAN bus health monitoring
if (!svarv.isCANHealthy()) {
  Serial.println("CAN bus issues detected!");
}

// Get statistics
uint32_t sent, received, errors;
svarv.getCANStatistics(sent, received, errors);
Serial.println("CAN Stats - Sent: " + String(sent) + ", Errors: " + String(errors));
```

## 🔧 Advanced Features

### Motor Discovery & Auto-Configuration

```cpp
// Scan for motors on the CAN bus
auto discovered = svarv.scanForMotors(5000); // 5 second timeout

Serial.println("Found " + String(discovered.size()) + " motors:");
for (uint8_t nodeId : discovered) {
  Serial.println("  - Motor " + String(nodeId));
}

// Auto-configure unconfigured motors (node ID = 0)
int configured = svarv.autoConfigureMotors(1, 10); // Start from ID 1, max 10 motors
Serial.println("Configured " + String(configured) + " motors");
```

### Coordinated Multi-Motor Operations

```cpp
// Get all motors
auto allMotors = svarv.getAllMotors();

// Emergency stop all
int stopped = svarv.emergencyStopAll();

// Enable/disable all
int enabled = svarv.enableAll();
int disabled = svarv.disableAll();

// Synchronized movements
for (auto* motor : allMotors) {
  motor->moveTo(calculateTargetPosition(motor->getNodeId()));
}
```

### Performance Optimization

```cpp
void loop() {
  // CRITICAL: Call update() frequently for optimal performance
  svarv.update(); // Aim for 10ms or faster
  
  // Your application code here
  // ...
  
  // Avoid blocking delays - use non-blocking timing instead
  static unsigned long lastAction = 0;
  if (millis() - lastAction > 1000) {
    lastAction = millis();
    // Periodic actions
  }
}
```

## 🛠️ Hardware Setup

### CAN Bus Wiring

```
ESP32          CAN Transceiver          Motor Controller
-----          ---------------          ----------------
GPIO 21  ----> CTX (TX)
GPIO 20  ----> CRX (RX)
3.3V     ----> VCC
GND      ----> GND
                    CANH  ============== CANH
                    CANL  ============== CANL
                    GND   ============== GND

Add 120Ω resistors between CANH and CANL at both ends of the bus
```

### Multi-Motor Setup

```
     ESP32
       |
  CAN Transceiver
       |
  -----+-----+-----+-----
  |    |     |     |    |
Motor1 Motor2 Motor3 ... MotorN
(ID=1) (ID=2) (ID=3)    (ID=N)

Each motor needs unique node ID (1-255)
```

## 🚨 Safety & Error Handling

### Emergency Stops

```cpp
// Individual motor emergency stop
motor.emergencyStop();

// System-wide emergency stop
svarv.emergencyStopAll();

// Error-triggered emergency stop
motor.onError([](SvarvMotor& m, SvarvErrorCode error, const String& message) {
  if (error == SVARV_ERROR_OVER_CURRENT) {
    m.emergencyStop();
  }
});
```

### Error Codes

| Error Code | Description | Typical Cause | Action |
|------------|-------------|---------------|---------|
| `SVARV_ERROR_NONE` | No error | Normal operation | Continue |
| `SVARV_ERROR_COMMAND_TIMEOUT` | No commands received | Communication loss | Check connections |
| `SVARV_ERROR_OVER_CURRENT` | Current limit exceeded | Mechanical binding | Emergency stop |
| `SVARV_ERROR_OVER_VOLTAGE` | Voltage limit exceeded | Power supply issue | Check power |
| `SVARV_ERROR_ENCODER_FAILURE` | Encoder not responding | Sensor malfunction | Replace encoder |

### Timeout Protection

```cpp
// Motors automatically enter safe mode if no commands received
// Position mode: 60 second timeout
// Velocity mode: 5 second timeout  
// Torque mode: 1 second timeout

// Send periodic commands to keep motor active
motor.moveTo(currentTarget); // Refresh target
```

## 🐛 Troubleshooting

### Common Issues

**Motor not responding:**
```cpp
// Check connection
if (!motor.isConnected()) {
  Serial.println("Motor not connected - check CAN bus");
}

// Check node ID
Serial.println("Motor node ID: " + String(motor.getNodeId()));

// Request status update
motor.requestStatusUpdate();
```

**CAN bus errors:**
```cpp
// Check CAN health
if (!svarv.isCANHealthy()) {
  uint32_t sent, received, errors;
  svarv.getCANStatistics(sent, received, errors);
  
  float errorRate = (float)errors / (sent + received);
  if (errorRate > 0.05) {
    Serial.println("High CAN error rate: " + String(errorRate * 100) + "%");
  }
}
```

**Poor position control:**
```cpp
// Check PID tuning
const auto& status = motor.getStatus();
float error = fabs(status.target_value - status.position);

if (error > 0.1) {
  Serial.println("Large position error - check PID gains");
  // Use PID_Tuning.ino example for systematic tuning
}
```

### Debug Output

```cpp
// Enable debug output
svarv.enableDebug(true);

// This will print detailed CAN communication info:
// [Svarv] CAN bus initialized @ 1000000 bps
// [Svarv] Added motor with node ID: 1
// [Svarv] Motor 1 connection: Disconnected -> Connected
```

## 📚 API Reference

### SvarvMotionControl Class

| Method | Description |
|--------|-------------|
| `begin(speed, tx, rx)` | Initialize CAN bus |
| `addMotor(nodeId)` | Add motor to system |
| `update()` | Process CAN messages (call frequently!) |
| `scanForMotors(timeout)` | Discover motors on bus |
| `emergencyStopAll()` | Stop all motors |
| `enableDebug(enable)` | Enable/disable debug output |

### SvarvMotor Class

| Method | Description |
|--------|-------------|
| `setControlMode(mode)` | Set control mode |
| `moveTo(position)` | Move to absolute position |
| `setVelocity(velocity)` | Set velocity target |
| `setTorque(torque)` | Set torque target |
| `setPID(type, p, i, d)` | Configure PID controller |
| `setLimits(vel, cur, volt)` | Set safety limits |
| `isConnected()` | Check connection status |
| `getStatus()` | Get current status |
| `saveConfig()` | Save to EEPROM |

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### Development Setup
1. Clone the repository
2. Install dependencies
3. Run examples with hardware
4. Submit pull requests

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **Documentation:** [GitHub Wiki](https://github.com/svarv-robotics/svarv-motion-control/wiki)
- **Issues:** [GitHub Issues](https://github.com/svarv-robotics/svarv-motion-control/issues)
- **Discussions:** [GitHub Discussions](https://github.com/svarv-robotics/svarv-motion-control/discussions)
- **Email:** support@svarv.com

## 🙏 Acknowledgments

Inspired by excellent motor control libraries:
- [SimpleFOC](https://simplefoc.com/) - Open-source FOC library
- [ODrive](https://odriverobotics.com/) - High-performance motor controller
- [VESC](https://vesc-project.com/) - Open-source motor controller
- [MJBots](https://mjbots.com/) - Precision servo controllers

## 🔗 Related Projects

- [Svarv Motor Controller Firmware](https://github.com/svarv-robotics/svarv-motor-controller) - BLDC motor controller firmware
- [Svarv Hardware Designs](https://github.com/svarv-robotics/svarv-hardware) - Open-source motor controller hardware
- [CAN Bus Examples](https://github.com/svarv-robotics/can-examples) - Additional CAN communication examples

---

**Built with ❤️ by [Svarv Robotics](https://svarv.com)**