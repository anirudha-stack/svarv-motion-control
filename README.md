# Svarv Motion Control Library

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/)
[![Cross Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20STM32%20%7C%20AVR-green.svg)](https://github.com/svarv-robotics/svarv-motion-control)
[![CAN Bus](https://img.shields.io/badge/Interface-CAN%20Bus-orange.svg)](https://en.wikipedia.org/wiki/CAN_bus)
[![Version](https://img.shields.io/badge/Version-2.0.0-brightgreen.svg)](https://github.com/svarv-robotics/svarv-motion-control)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A comprehensive cross-platform Arduino library for controlling CAN-based BLDC motor controllers. Inspired by industry-leading libraries like SimpleFOC, ODrive, and VESC, this library provides a simple yet powerful interface for advanced motor control applications across multiple hardware platforms.

## 🚀 Key Features

### **Cross-Platform Compatibility** *(New in v2.0)*
- **ESP32**: Built-in CAN via ESP32-TWAI-CAN library (ESP32-C3, ESP32-S3, ESP32 Classic)
- **STM32**: Built-in CAN via STM32duino library (F1, F4, F7, H7 series)
- **Arduino AVR**: MCP2515 external CAN controller support (Uno, Mega, Nano)
- **Generic Platforms**: Universal MCP2515 support for other Arduino-compatible boards
- **Automatic Platform Detection** with intelligent configuration

### **Advanced Motor Control**
- **Multi-Motor Control**: Up to 255 motors on single CAN bus (8 on AVR due to memory)
- **Control Modes**: Position, velocity, torque, and calibration modes
- **Real-Time Monitoring**: Live telemetry, status updates, and performance metrics
- **Interactive PID Tuning**: Real-time parameter adjustment with step response analysis
- **Physical Position Limits**: Configurable min/max position constraints with automatic enforcement
- **Node ID Management**: Interactive configuration tool for setting up motor IDs

### **Professional Features**
- **Non-blocking Operations**: Asynchronous communication with callback support
- **Comprehensive Error Handling**: Detailed error codes, automatic recovery, and safety systems
- **Configuration Management**: Save/load parameters to motor EEPROM with factory reset
- **Advanced Motion Patterns**: Trajectory generation, velocity profiling, force control, impedance control
- **System Diagnostics**: CAN bus health monitoring, performance analysis, and debugging tools

### **Developer-Friendly**
- **Platform Abstraction**: Write once, run on any supported platform
- **Memory Optimization**: Efficient data structures for resource-constrained devices
- **Rich Examples**: 8 comprehensive examples covering all features
- **Extensive Documentation**: Detailed API reference and troubleshooting guides

## 📋 Hardware Requirements

### Microcontroller Platforms
| Platform | CAN Interface | Memory Usage | Max Motors | Performance |
|----------|---------------|--------------|------------|-------------|
| **ESP32** | Built-in CAN (TWAI) | ~8KB | 255 | Excellent |
| **STM32** | Built-in CAN peripheral | ~6KB | 255 | Excellent |
| **Arduino AVR** | MCP2515 (SPI) | ~3KB | 8* | Good |
| **Generic** | MCP2515 (SPI) | ~4KB | 255 | Good |

*Memory-optimized for Arduino Uno/Nano

### Additional Hardware
- **CAN Transceiver**: SN65HVD230, TJA1050, or similar
- **Svarv BLDC Motor Controller(s)**: With firmware v1.2 or later
- **120Ω Termination Resistors**: At both ends of CAN bus
- **Power Supply**: Appropriate for your motor controllers

### Software Dependencies
- **Arduino IDE** 1.8.0+ or **PlatformIO**
- Platform-specific libraries (automatically installed):
  - ESP32: ESP32-TWAI-CAN library
  - STM32: STM32duino (built-in CAN)
  - AVR/Generic: mcp_can library

## 🔧 Installation

### Method 1: Arduino Library Manager (Recommended)
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
    svarv-robotics/Svarv Motion Control@^2.0.0
```

## 🚀 Quick Start

### Basic Single Motor Control

```cpp
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor* motor1 = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize CAN bus (auto-detects platform)
  if (!svarv.begin(1000000)) {
    Serial.println("CAN initialization failed!");
    while(1);
  }
  
  // Add motor with node ID 1
  motor1 = &svarv.addMotor(1);
  
  // Wait for connection
  while (!motor1->isConnected()) {
    svarv.update();
    delay(100);
  }
  
  // Configure for position control
  motor1->setControlMode(SVARV_MODE_POSITION);
  motor1->setLimits(25.0, 5.0, 12.0); // vel, current, voltage limits
}

void loop() {
  // IMPORTANT: Call update() frequently!
  svarv.update();
  
  // Move to different positions
  motor1->moveTo(3.14159); // Move to π radians
  delay(3000);
  
  motor1->moveTo(0.0);     // Return to home
  delay(3000);
}
```

### Multi-Motor Coordination

```cpp
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
SvarvMotor* motor1 = nullptr;
SvarvMotor* motor2 = nullptr;

void setup() {
  Serial.begin(115200);
  svarv.begin(1000000);
  
  // Scan for available motors
  auto discovered = svarv.scanForMotors(5000);
  Serial.println("Found " + String(discovered.size()) + " motors");
  
  // Add discovered motors
  motor1 = &svarv.addMotor(discovered[0]);
  motor2 = &svarv.addMotor(discovered[1]);
  
  // Configure both for position control
  motor1->setControlMode(SVARV_MODE_POSITION);
  motor2->setControlMode(SVARV_MODE_POSITION);
}

void loop() {
  svarv.update();
  
  // Synchronized movement - opposite directions
  motor1->moveTo(2.0);
  motor2->moveTo(-2.0);
  delay(3000);
  
  // Return both to home
  motor1->moveTo(0.0);
  motor2->moveTo(0.0);
  delay(3000);
}
```

## 🎯 Platform-Specific Setup

The library automatically detects your platform, but you can specify platform-specific settings:

### ESP32 Built-in CAN
```cpp
// Auto-detection (default pins: TX=21, RX=20)
svarv.begin(1000000);

// Custom pins
svarv.begin(1000000, 4, 5); // TX=GPIO4, RX=GPIO5
```

### STM32 Built-in CAN
```cpp
// Auto-detection (uses CAN1)
svarv.begin(1000000);

// Specify CAN instance
svarv.begin(1000000, 2); // Use CAN2
```

### Arduino + MCP2515
```cpp
// Basic setup (CS=10, INT=2, 8MHz crystal)
svarv.begin(1000000, 10);

// Full configuration
svarv.begin(1000000, 10, 2, MCP_8MHZ); // CS, INT, crystal
```

## 🛠️ Hardware Wiring

### ESP32 CAN Bus Wiring

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

### STM32 CAN Bus Wiring

```
STM32F4        CAN Transceiver          Motor Controller
-------        ---------------          ----------------
PB9      ----> CTX (TX)                 (pins vary by variant)
PB8      ----> CRX (RX)
3.3V     ----> VCC
GND      ----> GND
                    CANH  ============== CANH
                    CANL  ============== CANL
                    GND   ============== GND
```

### Arduino + MCP2515 Wiring

```
Arduino Uno    MCP2515 Module           Motor Controller
-----------    --------------           ----------------
Pin 13   ----> SCK
Pin 11   ----> MOSI (SI)
Pin 12   ----> MISO (SO)
Pin 10   ----> CS
Pin 2    ----> INT
5V       ----> VCC
GND      ----> GND
                    CANH  ============== CANH
                    CANL  ============== CANL
                    GND   ============== GND
```

## 📖 Comprehensive Examples

The library includes 8 detailed examples demonstrating various features:

### 1. [Basic_Control.ino](examples/Basic_Control/Basic_Control.ino)
**Purpose**: Introduction to single motor control  
**Features**: Position, velocity, and torque modes with real-time monitoring  
**Best for**: Getting started, understanding basic concepts

### 2. [Multi_Motor_Control.ino](examples/Multi_Motor_Control/Multi_Motor_Control.ino)
**Purpose**: Coordinated control of multiple motors  
**Features**: Two-motor synchronization with opposite commands  
**Best for**: Multi-axis systems, coordinated movements

### 3. [Cross_Platform_Example.ino](examples/Cross_Platform_Example/Cross_Platform_Example.ino)
**Purpose**: Platform compatibility and auto-detection demonstration  
**Features**: Auto-detection, platform-specific configuration, troubleshooting  
**Best for**: Testing platform compatibility, debugging setup issues

### 4. [PID_Tuning.ino](examples/PID_Tuning/PID_Tuning.ino)
**Purpose**: Interactive PID parameter tuning tool  
**Features**: Real-time gain adjustment, step response analysis, performance metrics  
**Best for**: Optimizing control performance, system tuning  
**Commands**: `p20` (P gain), `i0.1` (I gain), `step` (test), `save` (store settings)

### 5. [Advanced_Control_Patterns.ino](examples/Advanced_Control_Patterns/Advanced_Control_Patterns.ino)
**Purpose**: Sophisticated motion control techniques  
**Features**: Trajectory generation, velocity profiling, force control, impedance control  
**Best for**: Robotics applications, advanced motion patterns

### 6. [Node_ID_Setup_Example.ino](examples/Node_ID_Setup_Example/Node_ID_Setup_Example.ino)
**Purpose**: Interactive motor Node ID configuration  
**Features**: Configure unconfigured motors, change existing IDs, bulk setup  
**Best for**: Initial motor setup, system configuration  
**Commands**: `init`, `scan`, `set 2`, `save 2`, `verify`

### 7. [Current_Sense_Test.ino](examples/Current_Sense_Test/Current_Sense_Test.ino)
**Purpose**: Current sensor testing and monitoring  
**Features**: Real-time current display, manual motor loading test  
**Best for**: Hardware validation, current sensor calibration

### 8. [Physical_Limits_Example.ino](examples/Physical_Limits_Example/Physical_Limits_Example.ino)
**Purpose**: Position limits and safety constraints  
**Features**: Set min/max limits, automatic position clamping, enable/disable limits  
**Best for**: Safety-critical applications, preventing mechanical damage

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

// Different controllers
motor.setPID(SVARV_PID_VELOCITY, 0.5, 10.0, 0.0); // Velocity PID
motor.setPID(SVARV_PID_CURRENT, 3.0, 300.0, 0.0); // Current PID
```

### Physical Position Limits

```cpp
// Set position limits: -2π to +2π radians
motor.setPositionLimits(-2*PI, 2*PI, true); // min, max, enable

// Commands beyond limits are automatically constrained
motor.moveTo(10.0); // Automatically limited to 2π

// Enable/disable limits
motor.enablePositionLimits();
motor.disablePositionLimits();

// Check if position is within limits
if (motor.isPositionWithinLimits(target_pos)) {
  motor.moveTo(target_pos);
}
```

### Motor Limits and Safety

```cpp
// Set safety limits
motor.setLimits(15.0,  // Max velocity [rad/s]
                5.0,   // Max current [A]
                12.0); // Max voltage [V]

// Save configuration to EEPROM
motor.saveConfig();

// Emergency stop
motor.emergencyStop();
svarv.emergencyStopAll(); // Stop all motors
```

## 🔧 Node ID Configuration

### Setting Up New Motors

Use the interactive Node ID configuration tool:

```cpp
// 1. Connect ONLY ONE unconfigured motor to CAN bus
// 2. Use Node_ID_Setup_Example.ino
// 3. Follow the interactive commands:

// Initialize CAN
// Command: init

// Scan for existing motors
// Command: scan

// Configure unconfigured motor to ID 2
// Command: set 2

// Save configuration to EEPROM
// Command: save 2

// Verify motor responds
// Command: verify
```

### Bulk Configuration

```cpp
// Configure multiple motors with sequential IDs
int configured = svarv.configureMotorRange(1, 5); // IDs 1-5
Serial.println("Configured " + String(configured) + " motors");
```

## 📊 Monitoring & Callbacks

### Real-Time Status Monitoring

```cpp
// Get current status
const SvarvMotorStatus& status = motor.getStatus();
Serial.println("Position: " + String(status.position, 3) + " rad");
Serial.println("Velocity: " + String(status.velocity, 2) + " rad/s");
Serial.println("Current: " + String(status.current_q, 2) + " A");

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

## 🚨 Safety & Error Handling

### Error Codes and Recovery

| Error Code | Description | Typical Cause | Recommended Action |
|------------|-------------|---------------|-------------------|
| `SVARV_ERROR_NONE` | No error | Normal operation | Continue |
| `SVARV_ERROR_COMMAND_TIMEOUT` | No commands received | Communication loss | Check connections |
| `SVARV_ERROR_OVER_CURRENT` | Current limit exceeded | Mechanical binding | Emergency stop |
| `SVARV_ERROR_OVER_VOLTAGE` | Voltage limit exceeded | Power supply issue | Check power |
| `SVARV_ERROR_POSITION_LIMIT` | Position limit hit | Motion beyond limits | Adjust target |
| `SVARV_ERROR_ENCODER_FAILURE` | Encoder not responding | Sensor malfunction | Replace encoder |

### Timeout Protection

```cpp
// Motors automatically enter safe mode if no commands received:
// - Position mode: 60 second timeout
// - Velocity mode: 5 second timeout  
// - Torque mode: 1 second timeout

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

// Verify CAN health
if (!svarv.isCANHealthy()) {
  Serial.println("CAN bus issues detected");
}

// Request status update
motor.requestStatusUpdate();
```

**Platform-specific issues:**
- **ESP32**: Check if CAN pins conflict with other peripherals
- **STM32**: Ensure STM32duino core is installed and CAN peripheral exists
- **Arduino AVR**: Use lower CAN speeds (500k/250k), check memory usage

### Debug Output

```cpp
// Enable debug output for detailed diagnostics
svarv.enableDebug(true);

// This prints detailed information:
// [Svarv] CAN bus initialized @ 1000000 bps
// [Svarv] Platform: ESP32 Built-in CAN
// [Svarv] Added motor with node ID: 1
// [Svarv] Motor 1 connection: Disconnected -> Connected
```

## 📚 API Reference

### SvarvMotionControl Class

| Method | Description |
|--------|-------------|
| `begin(speed)` | Initialize with auto-platform detection |
| `begin(speed, tx, rx)` | Initialize ESP32 with custom pins |
| `begin(speed, can_instance)` | Initialize STM32 with CAN1/CAN2 |
| `begin(speed, cs, int, crystal)` | Initialize MCP2515 with configuration |
| `addMotor(nodeId)` | Add motor to system |
| `removeMotor(nodeId)` | Remove motor from system |
| `scanForMotors(timeout)` | Discover motors on bus |
| `configureSingleMotor(targetId)` | Configure unconfigured motor to specific ID |
| `update()` | Process CAN messages (call frequently!) |
| `emergencyStopAll()` | Stop all motors |
| `getPlatformInfo()` | Get platform information |
| `enableDebug(enable)` | Enable/disable debug output |

### SvarvMotor Class

| Method | Description |
|--------|-------------|
| `setControlMode(mode)` | Set control mode (IDLE, POSITION, VELOCITY, TORQUE) |
| `moveTo(position)` | Move to absolute position [rad] |
| `moveBy(delta)` | Move relative to current position [rad] |
| `setVelocity(velocity)` | Set velocity target [rad/s] |
| `setTorque(torque)` | Set torque target [Nm] |
| `setPID(type, p, i, d)` | Configure PID controller |
| `setLimits(vel, cur, volt)` | Set safety limits |
| `setPositionLimits(min, max, enable)` | Set physical position limits |
| `isConnected()` | Check connection status |
| `getStatus()` | Get current status |
| `saveConfig()` | Save configuration to EEPROM |
| `isAtTarget(tolerance)` | Check if at target position |
| `waitForTarget(timeout, tolerance)` | Wait for target (blocking) |

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### Development Setup
1. Clone the repository
2. Install dependencies for your target platform
3. Test with real hardware
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

---

**Built with ❤️ by [Svarv Robotics]**