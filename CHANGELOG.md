# Changelog

All notable changes to the Svarv Motion Control Library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2025-07-27

### 🚀 Major Release - Cross-Platform Support

This is a major release that completely refactors the library to support multiple hardware platforms while maintaining backward compatibility.

### Added

#### Cross-Platform Support
- **ESP32 Support**: Built-in CAN via ESP32-TWAI-CAN library
  - ESP32, ESP32-C3, ESP32-S3 with configurable pins
  - High-performance operation up to 1 Mbps
  - Full feature set with 255 motor support

- **STM32 Support**: Built-in CAN via STM32duino
  - STM32F1, STM32F4, STM32F7, STM32H7 series
  - CAN1/CAN2 peripheral support
  - Robust operation with hardware CAN

- **Arduino AVR Support**: MCP2515 external CAN controller
  - Arduino Uno, Mega, Nano compatibility
  - Memory-optimized for limited AVR resources
  - Configurable CS/INT pins and crystal frequency
  - Support for 8MHz and 16MHz crystals

- **Generic Platform Support**: MCP2515 for other Arduino-compatible boards
  - Universal compatibility layer
  - Automatic SPI configuration

#### Platform Abstraction Layer
- `SvarvCANInterface` abstract base class for CAN communication
- Platform-specific implementations:
  - `SvarvCANInterface_ESP32` for ESP32 built-in CAN
  - `SvarvCANInterface_STM32` for STM32 built-in CAN
  - `SvarvCANInterface_MCP2515` for external MCP2515 controllers
- Unified `SvarvCANMessage` structure across all platforms

#### Automatic Platform Detection
- `svarvDetectPlatform()` function for automatic hardware detection
- `SvarvPlatformConfig` structure for platform-specific settings
- Intelligent fallback to MCP2515 for unknown platforms

#### Memory Optimization
- `SimpleMap` template for AVR platforms with limited memory
- Conditional compilation for platform-specific features
- Reduced memory footprint on resource-constrained devices
- Configurable maximum motor count (`SVARV_MAX_MOTORS` for AVR)

#### Enhanced API
- Multiple `begin()` method overloads for different platforms:
  - `begin(speed)` - Auto-detection with default settings
  - `begin(speed, tx, rx)` - ESP32 with custom pins
  - `begin(speed, can_instance)` - STM32 with CAN1/CAN2 selection
  - `begin(speed, cs, int, crystal)` - MCP2515 with full configuration
  - `begin(config)` - Explicit platform configuration
- `getPlatformInfo()` method for runtime platform identification
- Enhanced error reporting with platform-specific details

#### New Examples
- `Cross_Platform_Example.ino` - Demonstrates auto-detection and platform-specific features
- Platform-specific configuration examples
- Memory-optimized examples for AVR platforms
- Troubleshooting and diagnostic examples

#### Utility Functions
- `svarvPlatformToString()` - Convert platform enum to readable string
- `svarvDetectPlatform()` - Hardware platform detection
- Enhanced debug output with platform information

### Changed

#### Breaking Changes
- **Library version updated to 2.0.0**
- **Header includes**: Platform-specific CAN libraries now included automatically
- **Namespace changes**: Some internal structures reorganized for cross-platform support
- **Memory management**: Different approaches for AVR vs other platforms

#### API Improvements
- More robust error handling across platforms
- Improved timeout handling for different CAN interfaces
- Better resource management and cleanup
- Enhanced callback system with platform awareness

#### Performance Optimizations
- Platform-specific optimizations for message processing
- Reduced overhead on resource-constrained platforms
- Optimized data structures for different memory constraints
- Improved CAN message throughput on high-performance platforms

#### Documentation
- Comprehensive cross-platform documentation
- Platform-specific wiring diagrams
- Troubleshooting guides for each platform
- Performance comparison tables
- Memory usage guidelines

### Dependencies

#### New Dependencies
- **ESP32**: `ESP32-TWAI-CAN` library (auto-installed)
- **STM32**: `STM32duino` core with built-in CAN support
- **AVR/Generic**: `mcp_can` library for MCP2515 support

#### Dependency Management
- Automatic dependency resolution in Arduino Library Manager
- Platform-specific dependency inclusion
- Graceful handling of missing dependencies

### Platform-Specific Features

#### ESP32
- Configurable CAN pins (default: TX=21, RX=20)
- High-speed operation up to 1 Mbps
- Large memory capacity for complex applications
- Real-time performance with hardware CAN

#### STM32
- Built-in CAN peripheral support
- CAN1/CAN2 selection capability
- Excellent real-time performance
- Robust industrial-grade operation

#### Arduino AVR
- Memory-optimized data structures
- Limited to 8 motors on Uno/Nano (memory constraint)
- Support for common MCP2515 shield configurations
- Automatic crystal frequency detection

### Backward Compatibility

#### API Compatibility
- Existing code using `svarv.begin(speed)` continues to work
- All motor control methods remain unchanged
- Callback system maintains same interface
- Configuration methods unchanged

#### Migration Path
- No code changes required for basic ESP32 usage
- STM32 users benefit from automatic platform detection
- Arduino users need to add MCP2515 connections
- Detailed migration guide provided in documentation

### Testing

#### Platform Testing
- Tested on ESP32-C3, ESP32-S3, ESP32 Classic
- Tested on STM32F4, STM32F1 development boards
- Tested on Arduino Uno, Mega, Nano with MCP2515 shields
- Cross-platform compatibility verified

#### Integration Testing
- Multi-motor coordination across platforms
- CAN bus health monitoring on all platforms
- Error handling and recovery testing
- Performance benchmarking per platform

### Known Issues

#### Platform Limitations
- **Arduino AVR**: Limited to 8 motors due to memory constraints
- **MCP2515**: Maximum CAN speed may be limited by SPI performance
- **STM32**: Some variants may not have CAN peripheral (compile-time detection)

#### Workarounds Provided
- Memory optimization guidelines for AVR
- Alternative pin configurations for conflicting peripherals
- Fallback options for unsupported platforms

### Future Roadmap

#### Planned Platforms
- **ESP8266**: Support planned for v2.1 with SoftwareCAN
- **Raspberry Pi Pico**: RP2040 support with PIO-based CAN
- **Teensy**: Native CAN support for Teensy 3.x/4.x

#### Planned Features
- **CAN-FD Support**: For high-speed modern platforms
- **Wireless CAN**: ESP-NOW and WiFi CAN bridging
- **Real-time Extensions**: RTOS integration for critical applications

---

## [1.0.0] - 2025-07-26

### Added

#### Initial Release - ESP32 Only
- **ESP32 CAN Support**: Built-in CAN via ESP32-TWAI-CAN library
- **Multi-Motor Control**: Support for up to 255 motors on single CAN bus
- **Advanced Control Modes**:
  - Position control with configurable PID
  - Velocity control with smooth acceleration
  - Torque control for force applications
  - Calibration mode for automatic setup

#### Core Features
- **Real-Time Monitoring**: Live telemetry and status updates
- **Comprehensive Error Handling**: Detailed error codes and recovery
- **Non-Blocking Operations**: Asynchronous communication with callbacks
- **Configuration Management**: Save/load parameters to motor EEPROM
- **Safety Systems**: Emergency stops, timeout protection, limit enforcement

#### Examples
- `Basic_Control.ino` - Single motor control demonstration
- `Multi_Motor_Control.ino` - Coordinated multi-motor operations
- `PID_Tuning.ino` - Interactive PID parameter tuning
- `Advanced_Control_Patterns.ino` - Sophisticated motion control

#### API Features
- **Motor Management**: Add, remove, and configure motors
- **Control Methods**: moveTo(), setVelocity(), setTorque()
- **Configuration**: setPID(), setLimits(), saveConfig()
- **Monitoring**: Real-time status, error reporting, connection management
- **Callbacks**: Status updates, error handling, connection changes

#### Documentation
- Comprehensive README with examples
- Detailed API reference
- Hardware setup guides
- Troubleshooting documentation
- Contributing guidelines

### Dependencies
- **ESP32-TWAI-CAN**: For ESP32 built-in CAN communication
- **Arduino Core for ESP32**: Version 2.0.0+

### Limitations
- **ESP32 Only**: Limited to ESP32 platforms with built-in CAN
- **Single Platform**: No support for other microcontrollers
- **Hardware Dependency**: Required ESP32 with CAN capability

---

## Version History Summary

- **v2.0.0 (2025-07-27)**: Major cross-platform release
  - Added STM32 and Arduino AVR support
  - Platform abstraction layer
  - Memory optimizations
  - Enhanced examples and documentation

- **v1.0.0 (2025-07-26)**: Initial ESP32-only release
  - Core motor control functionality
  - Multi-motor coordination
  - Advanced control features
  - Comprehensive documentation

## Migration Guide

### From v1.0.0 to v2.0.0

#### ESP32 Users (No Changes Required)
```cpp
// This code works unchanged in v2.0.0
SvarvMotionControl svarv;
void setup() {
  svarv.begin(1000000); // Still works!
  // ... rest of code unchanged
}
```

#### New Platform Users

##### STM32 Users
```cpp
// Old ESP32-specific code:
// #include <ESP32-TWAI-CAN.hpp>  // Remove this

// New cross-platform code:
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
void setup() {
  svarv.begin(1000000); // Auto-detects STM32
  // ... same API as before
}
```

##### Arduino + MCP2515 Users
```cpp
#include "SvarvMotionControl.h"

SvarvMotionControl svarv;
void setup() {
  // Specify MCP2515 configuration
  svarv.begin(1000000, 10, 2, MCP_8MHZ); // speed, CS, INT, crystal
  // ... same API as before
}
```

#### Memory-Constrained Platforms
```cpp
// For Arduino AVR - memory optimizations
void setup() {
  Serial.begin(9600);        // Lower baud rate
  svarv.enableDebug(false);  // Disable debug output
  svarv.begin(500000);       // Lower CAN speed
  
  // Limit to fewer motors on AVR
  motor1 = svarv.addMotor(1);
  // Don't add too many motors on Uno/Nano
}
```

### API Compatibility Matrix

| Feature | v1.0.0 (ESP32) | v2.0.0 (All Platforms) | Notes |
|---------|----------------|-------------------------|-------|
| `svarv.begin(speed)` | ✅ ESP32 only | ✅ All platforms | Auto-detection added |
| Motor control API | ✅ Full support | ✅ Full support | Unchanged |
| Callbacks | ✅ Full support | ✅ Full support | Unchanged |
| Multi-motor | ✅ 255 motors | ✅ Platform dependent | AVR limited to 8 |
| Debug output | ✅ Full | ✅ Platform optimized | Reduced on AVR |
| Memory usage | ~8KB | 3-8KB | Platform optimized |

## Support and Help

For migration assistance or platform-specific questions:
- **GitHub Issues**: [Report platform-specific issues](https://github.com/svarv-robotics/svarv-motion-control/issues)
- **GitHub Discussions**: [Platform-specific help](https://github.com/svarv-robotics/svarv-motion-control/discussions)
- **Email Support**: support@svarv.com