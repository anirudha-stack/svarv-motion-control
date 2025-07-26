# Contributing to Svarv Motion Control Library

Thank you for your interest in contributing to the Svarv Motion Control Library! This document provides guidelines and information for contributors.

## 🤝 How to Contribute

### Reporting Bugs

Before submitting a bug report, please:

1. **Check existing issues** to avoid duplicates
2. **Test with the latest version** to ensure the bug still exists
3. **Provide detailed information** including:
   - Library version
   - Hardware configuration (ESP32 variant, CAN transceiver, motor controllers)
   - Arduino IDE or PlatformIO version
   - Complete error messages
   - Minimal code example that reproduces the issue
   - Serial output logs

**Bug Report Template:**
```markdown
**Library Version:** 1.0.0
**Hardware:** ESP32-C3 + SN65HVD230 + Svarv Motor Controller v1.2
**IDE:** Arduino IDE 2.1.0

**Description:**
Brief description of the bug

**Steps to Reproduce:**
1. Step 1
2. Step 2
3. ...

**Expected Behavior:**
What should happen

**Actual Behavior:**
What actually happens

**Code Example:**
```cpp
// Minimal code that reproduces the issue
```

**Serial Output:**
```
// Complete serial output showing the error
```
```

### Suggesting Enhancements

Enhancement suggestions are welcome! Please provide:

1. **Clear description** of the proposed feature
2. **Use case** explaining why it would be valuable
3. **Implementation ideas** if you have them
4. **Backward compatibility** considerations

**Enhancement Template:**
```markdown
**Feature Description:**
Brief summary of the proposed feature

**Use Case:**
Explain the problem this feature would solve

**Proposed Implementation:**
If you have ideas on how to implement this

**Alternatives Considered:**
Other approaches you considered

**Additional Context:**
Any other relevant information
```

### Pull Requests

We welcome code contributions! Please follow these guidelines:

#### Before You Start

1. **Open an issue** to discuss major changes
2. **Check the roadmap** to avoid conflicts with planned features
3. **Read the code style guide** below

#### Pull Request Process

1. **Fork the repository**
2. **Create a feature branch** from `main`
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** following our coding standards
4. **Test thoroughly** with real hardware
5. **Update documentation** if needed
6. **Commit with clear messages**
   ```bash
   git commit -m "Add feature: brief description
   
   Detailed explanation of what this commit does.
   Include any breaking changes or special notes."
   ```
7. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```
8. **Open a pull request** with:
   - Clear title and description
   - Reference to related issues
   - Test results with hardware configuration
   - Screenshots/videos if applicable

#### Pull Request Checklist

- [ ] Code follows the project's style guidelines
- [ ] Self-review of code completed
- [ ] Code is well-commented, especially complex areas
- [ ] Documentation updated for API changes
- [ ] Examples updated if needed
- [ ] Tested with real hardware
- [ ] No new compiler warnings introduced
- [ ] Backward compatibility maintained (or breaking changes documented)

## 📝 Code Style Guidelines

### General Principles

- **Clarity over cleverness** - Write code that's easy to understand
- **Consistent style** - Follow existing patterns in the codebase
- **Meaningful names** - Use descriptive variable and function names
- **Comment complex logic** - Explain the "why", not just the "what"

### C++ Style Guidelines

#### Naming Conventions

```cpp
// Classes: PascalCase
class SvarvMotor {
    // Public methods: camelCase
    bool setControlMode(SvarvControlMode mode);
    
    // Private members: camelCase with trailing underscore
    SvarvMotionControl* controller_;
    SvarvMotorConfig config_;
    
    // Constants: UPPER_CASE
    static const int MAX_MOTORS = 255;
};

// Enums: PascalCase with prefix
enum SvarvControlMode {
    SVARV_MODE_IDLE,
    SVARV_MODE_POSITION
};

// Functions: camelCase
bool sendCANMessage(uint8_t nodeId, uint8_t cmd);

// Variables: camelCase
float targetPosition = 0.0f;
unsigned long lastUpdateTime = 0;
```

#### Code Formatting

```cpp
// Indentation: 2 spaces (no tabs)
if (condition) {
  doSomething();
  if (nestedCondition) {
    doSomethingElse();
  }
}

// Braces: Always use braces, even for single statements
if (motor.isConnected()) {
  motor.moveTo(target);
}

// Function declarations: One parameter per line for long lists
bool sendCANMessage(uint8_t nodeId, 
                   uint8_t functionId, 
                   uint8_t command,
                   const uint8_t* data = nullptr, 
                   uint8_t length = 0);

// Member initialization: Use initializer lists
SvarvMotor::SvarvMotor(uint8_t nodeId, SvarvMotionControl* controller)
    : controller_(controller), 
      config_(),
      status_(),
      lastCommandTime_(0) {
  // Constructor body
}
```

#### Documentation

```cpp
/**
 * @brief Set the motor control mode
 * 
 * Changes the motor control mode and automatically resets the target value
 * for safety. The motor must be connected before calling this method.
 * 
 * @param mode New control mode (IDLE, TORQUE, VELOCITY, POSITION, CALIBRATION)
 * @return true if command sent successfully, false if motor not connected
 * 
 * @note Switching modes resets the target value to prevent unexpected movement
 * @warning Always ensure motor is in a safe position before mode changes
 * 
 * @see SvarvControlMode for available modes
 * @see setTargetValue() to set new target after mode change
 */
bool setControlMode(SvarvControlMode mode);
```

#### Error Handling

```cpp
// Always check return values
if (!svarv.begin(1000000)) {
  Serial.println("ERROR: CAN initialization failed!");
  return false;
}

// Use meaningful error messages
if (!motor.isConnected()) {
  Serial.println("ERROR: Motor " + String(nodeId) + " not responding");
  Serial.println("Check CAN bus connections and motor power");
  return false;
}

// Validate input parameters
bool SvarvMotor::setPID(SvarvPIDType pidType, float P, float I, float D) {
  if (P < 0 || P > 1000) {
    Serial.println("ERROR: P gain must be between 0 and 1000");
    return false;
  }
  // ... continue with valid parameters
}
```

### Arduino-Specific Guidelines

```cpp
// Use appropriate data types
uint8_t nodeId = 1;           // For small integers (0-255)
uint16_t intervalMs = 1000;   // For medium integers (0-65535)
uint32_t timestamp = millis(); // For large integers
float position = 3.14159f;    // Use 'f' suffix for floats

// Non-blocking code patterns
void loop() {
  // Always call update() frequently
  svarv.update();
  
  // Use timing instead of delay()
  static unsigned long lastAction = 0;
  if (millis() - lastAction > 1000) {
    lastAction = millis();
    // Periodic action
  }
  
  // Never use delay() in library code
  // delay(1000); // ❌ DON'T DO THIS
}

// Memory management
// Prefer stack allocation
SvarvMotorStatus status = motor.getStatus();

// Use references for large objects
void processStatus(const SvarvMotorStatus& status) {
  // Process status without copying
}
```

## 🧪 Testing Guidelines

### Hardware Testing Requirements

All contributions that affect motor control behavior must be tested with real hardware:

1. **Minimum test setup:**
   - ESP32 with CAN transceiver
   - At least one Svarv motor controller
   - CAN bus with proper termination

2. **Test scenarios:**
   - Basic connectivity and communication
   - All control modes (position, velocity, torque)
   - Error handling and recovery
   - Multiple motor coordination (if applicable)

3. **Test documentation:**
   - Hardware configuration used
   - Test procedure followed
   - Results and any issues found

### Code Testing

```cpp
// Include test cases in examples when adding new features
void testNewFeature() {
  Serial.println("Testing new feature...");
  
  // Test normal operation
  bool result = motor.newFeature(validParameter);
  if (result) {
    Serial.println("✅ Normal operation test passed");
  } else {
    Serial.println("❌ Normal operation test failed");
  }
  
  // Test error conditions
  result = motor.newFeature(invalidParameter);
  if (!result) {
    Serial.println("✅ Error handling test passed");
  } else {
    Serial.println("❌ Error handling test failed");
  }
}
```

## 📚 Documentation Standards

### README Updates

When adding new features, update the README.md:

1. **Quick Start section** if the basic usage changes
2. **API Reference** for new methods
3. **Examples section** if new examples are added

### Code Comments

```cpp
// Good comments explain WHY, not WHAT
float error = targetPosition - currentPosition; // Calculate position error

// Bad comment (obvious what the code does)
targetPosition = 5.0; // Set target position to 5.0

// Good comment (explains the reasoning)
targetPosition = 5.0; // Move to known safe position before calibration
```

### Example Documentation

Each example should include:

```cpp
/**
 * @file ExampleName.ino
 * @brief Brief description of what this example demonstrates
 * @author Svarv Robotics
 * 
 * Detailed description of the example, including:
 * - What it demonstrates
 * - Hardware requirements
 * - Expected behavior
 * - Learning objectives
 * 
 * Hardware Required:
 * - ESP32 with CAN capability
 * - CAN transceiver (SN65HVD230)
 * - Svarv BLDC Motor Controller
 * 
 * Connections:
 * - CAN TX: GPIO 21
 * - CAN RX: GPIO 20
 * - Add 120Ω termination resistors
 */
```

## 🚀 Development Setup

### Prerequisites

1. **Arduino IDE** 2.0+ or **PlatformIO**
2. **ESP32 Arduino Core** 2.0.0+
3. **Hardware setup** for testing
4. **Git** for version control

### Development Workflow

```bash
# Clone the repository
git clone https://github.com/svarv-robotics/svarv-motion-control.git
cd svarv-motion-control

# Create a feature branch
git checkout -b feature/your-feature

# Make changes and test
# ... edit code ...

# Commit changes
git add .
git commit -m "Add feature: description"

# Push and create pull request
git push origin feature/your-feature
```

### Building and Testing

```bash
# Using PlatformIO
pio run
pio run --target upload
pio device monitor

# Using Arduino IDE
# Open any example in the IDE
# Select your ESP32 board
# Compile and upload
```

## 🎯 Areas for Contribution

We especially welcome contributions in these areas:

### High Priority
- **Bug fixes** - Always appreciated!
- **Documentation improvements** - Help make the library more accessible
- **Hardware testing** - Test with different ESP32 variants and motor controllers
- **Performance optimizations** - Improve timing and reduce overhead

### Medium Priority
- **Additional examples** - Show specific use cases
- **Advanced control algorithms** - Trajectory planning, advanced PID tuning
- **Diagnostic tools** - Better debugging and monitoring capabilities
- **Platform support** - Support for additional microcontrollers

### Future Features
- **ROS/ROS2 integration** - Bridge to robot operating systems
- **Machine learning** - Auto-tuning and predictive maintenance
- **Web interface** - Browser-based configuration and monitoring
- **Safety certifications** - Industrial safety standards compliance

## 🏆 Recognition

Contributors will be recognized in:

- **CONTRIBUTORS.md** file
- **Release notes** for their contributions
- **GitHub contributors** section
- **Special thanks** in documentation

## 📞 Getting Help

If you need help with contributing:

1. **Check the documentation** first
2. **Search existing issues** for similar questions
3. **Join the discussion** on GitHub Discussions
4. **Contact us** via email: support@svarv.com

## 📜 Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](https://www.contributor-covenant.org/). By participating, you agree to uphold this code. Please report unacceptable behavior to support@svarv.com.

---

Thank you for contributing to the Svarv Motion Control Library! Your contributions help make advanced motor control accessible to everyone.