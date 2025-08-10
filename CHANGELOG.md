# Changelog

All notable changes to the Svarv Motion Control Library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2025-07-27

### Added
- **Cross-Platform Support**: ESP32, STM32, Arduino AVR, Generic platforms
- **Platform Abstraction Layer**: `SvarvCANInterface` with platform-specific implementations
- **Automatic Platform Detection**: `svarvDetectPlatform()` function
- **Physical Position Limits**: `setPositionLimits()`, `enablePositionLimits()`, `disablePositionLimits()`
- **Node ID Configuration**: `configureSingleMotor()`, interactive setup tools
- **Memory Optimization**: `SimpleMap` for AVR, reduced footprint
- **New Examples**: 
  - `Cross_Platform_Example.ino`
  - `Node_ID_Setup_Example.ino` 
  - `Physical_Limits_Example.ino`
  - `Current_Sense_Test.ino`
- **Enhanced API**: Multiple `begin()` overloads for different platforms
- **Platform Info**: `getPlatformInfo()` method

### Changed
- **Motor Object Management**: Now requires pointer/reference usage
- **Memory Management**: Platform-specific optimizations
- **Error Handling**: Enhanced cross-platform error reporting
- **Documentation**: Updated for multi-platform support

### Fixed
- **AVR Memory Issues**: Optimized data structures for limited RAM
- **Connection Handling**: Improved timeout and reconnection logic
- **CAN Message Processing**: Better platform-specific handling

### Platform Support
| Platform | Interface | Max Motors | Memory |
|----------|-----------|------------|---------|
| ESP32 | Built-in CAN | 255 | ~8KB |
| STM32 | Built-in CAN | 255 | ~6KB |
| Arduino AVR | MCP2515 | 8 | ~3KB |
| Generic | MCP2515 | 255 | ~4KB |

### Dependencies
- **ESP32**: ESP32-TWAI-CAN library
- **STM32**: STM32duino core
- **AVR/Generic**: mcp_can library

### Breaking Changes
- Motor objects must now use pointers: `SvarvMotor* motor = &svarv.addMotor(1)`
- Some internal structures reorganized for cross-platform compatibility

---

## [1.0.0] - 2025-07-26

### Added
- **Initial Release**: ESP32-only support via ESP32-TWAI-CAN
- **Multi-Motor Control**: Up to 255 motors on single CAN bus
- **Control Modes**: Position, velocity, torque, calibration
- **Real-Time Monitoring**: Live telemetry and status updates
- **PID Tuning**: Configurable position, velocity, current controllers
- **Safety Features**: Emergency stops, timeout protection, error handling
- **Configuration Management**: Save/load to motor EEPROM
- **Callback System**: Status, error, and connection callbacks
- **Examples**:
  - `Basic_Control.ino`
  - `Multi_Motor_Control.ino`
  - `PID_Tuning.ino`
  - `Advanced_Control_Patterns.ino`

### Dependencies
- ESP32-TWAI-CAN library
- Arduino Core for ESP32 v2.0.0+

### Limitations
- ESP32 platforms only
- Built-in CAN requirement

---

## Migration Guide

### v1.0.0 → v2.0.0

#### Minimal Changes (ESP32)
```cpp
// Old (v1.0.0)
SvarvMotor motor1;
motor1 = svarv.addMotor(1);

// New (v2.0.0)  
SvarvMotor* motor1 = &svarv.addMotor(1);
```

#### Platform-Specific
- **ESP32**: `svarv.begin(1000000)` unchanged
- **STM32**: `svarv.begin(1000000)` auto-detects
- **Arduino+MCP2515**: `svarv.begin(1000000, 10, 2, MCP_8MHZ)`