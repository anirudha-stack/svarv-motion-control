/**
 * @file SvarvMotionControl.h
 * @brief Svarv Motion Control Library - Cross-Platform CAN-based BLDC Motor Control
 * @version 2.0.0
 * @date 2025-07-26
 * @author Svarv Robotics
 *
 * @section DESCRIPTION
 *
 * The Svarv Motion Control Library provides a simple, intuitive interface for controlling
 * multiple CAN-based BLDC motor controllers across different microcontroller platforms.
 * Inspired by industry-leading libraries like SimpleFOC, ODrive, and VESC, this library
 * abstracts both CAN communication and platform differences while providing full access
 * to advanced motor control features.
 *
 * @section SUPPORTED_PLATFORMS
 *
 * - ESP32 (ESP32-C3, ESP32-S3, ESP32 Classic) - Built-in CAN via ESP32-TWAI-CAN
 * - STM32 (STM32F4, STM32F1, etc.) - Built-in CAN via STM32duino
 * - Arduino AVR with MCP2515 CAN Controller - Via MCP_CAN library
 * - Other platforms with MCP2515 shield support
 *
 * @section FEATURES
 *
 * - Multi-motor control with automatic node management
 * - Position, velocity, and torque control modes
 * - Real-time telemetry and status monitoring
 * - Advanced PID tuning with live feedback
 * - Comprehensive error handling and safety monitoring
 * - Non-blocking asynchronous operations
 * - Built-in diagnostics and performance monitoring
 * - Cross-platform compatibility
 *
 * @section QUICK_START
 *
 * @code
 * #include "SvarvMotionControl.h"
 *
 * SvarvMotionControl svarv;
 * SvarvMotor motor1;
 *
 * void setup() {
 *   // ESP32 example
 *   svarv.begin(1000000); // 1 Mbps CAN, default pins
 *
 *   // STM32 example
 *   svarv.begin(1000000); // Uses built-in CAN
 *
 *   // Arduino + MCP2515 example
 *   svarv.begin(1000000, 10); // 1 Mbps, CS pin 10
 *
 *   motor1 = svarv.addMotor(1); // Node ID 1
 *   motor1.setControlMode(SVARV_MODE_POSITION);
 * }
 *
 * void loop() {
 *   motor1.moveTo(5.0); // Move to 5 radians
 *   svarv.update(); // Process CAN messages
 * }
 * @endcode
 *
 * @section LICENSE
 *
 * MIT License - See LICENSE file for details
 *
 * @section COMPATIBILITY
 *
 * - ESP32 (all variants with built-in CAN)
 * - STM32 (F1, F4, F7, H7 series with built-in CAN)
 * - Arduino AVR (Uno, Mega, Nano) with MCP2515 CAN shield
 * - Arduino IDE 1.8.0+
 * - PlatformIO
 * - Svarv Motor Controller Firmware v1.2+
 */

#ifndef SVARV_MOTION_CONTROL_H
#define SVARV_MOTION_CONTROL_H

#include <Arduino.h>
#include <functional>
#include <vector>

 // Platform detection and CAN library includes
#if defined(ESP32)
#include <ESP32-TWAI-CAN.hpp>
#define SVARV_PLATFORM_ESP32
#elif defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
#include <STM32_CAN.h>
#define SVARV_PLATFORM_STM32
#elif defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
  #include <mcp_can.h>
  #include <mcp_can_dfs.h>   // <<— add this so MCP_8MHZ, MCP_16MHZ, etc. are defined
  #include <SPI.h>
#define SVARV_PLATFORM_AVR
#else
    // Generic platform - assume MCP2515 support
  #include <mcp_can.h>
  #include <mcp_can_dfs.h>   // <<— add this so MCP_8MHZ, MCP_16MHZ, etc. are defined
  #include <SPI.h>
#define SVARV_PLATFORM_GENERIC
#endif

// Use std::map equivalent for different platforms
#ifdef ARDUINO_ARCH_AVR
    // AVR has limited memory - use simple array-based map
#define SVARV_MAX_MOTORS 8
template<typename K, typename V>
struct SimpleMap {
    struct Pair { K key; V value; bool used; };
    Pair data[SVARV_MAX_MOTORS];
    int count;

    SimpleMap() : count(0) {
        for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
            data[i].used = false;
        }
    }

    V& operator[](const K& key) {
        for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
            if (data[i].used && data[i].key == key) {
                return data[i].value;
            }
        }
        // Find empty slot
        for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
            if (!data[i].used) {
                data[i].key = key;
                data[i].used = true;
                count++;
                return data[i].value;
            }
        }
        // Return first slot if full (error case)
        return data[0].value;
    }

    bool find(const K& key) const {
        for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
            if (data[i].used && data[i].key == key) {
                return true;
            }
        }
        return false;
    }

    bool erase(const K& key) {
        for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
            if (data[i].used && data[i].key == key) {
                data[i].used = false;
                count--;
                return true;
            }
        }
        return false;
    }

    bool empty() const { return count == 0; }
    int size() const { return count; }
};
#else
#include <map>
template<typename K, typename V>
using SimpleMap = std::map<K, V>;
#endif

// Library version
#define SVARV_VERSION_MAJOR 2
#define SVARV_VERSION_MINOR 0
#define SVARV_VERSION_PATCH 0
#define SVARV_VERSION "2.0.0"

// Forward declarations
class SvarvMotor;
class SvarvMotionControl;

// Universal sections
// If we didn’t pull in mcp_can_dfs.h, define these so the constant is always available
#ifndef MCP_8MHZ
  #define MCP_8MHZ  2   // 8 MHz crystal
  #define MCP_16MHZ 1   // 16 MHz crystal
  #define MCP_20MHZ 0   // 20 MHz crystal
#endif


// ============================================================================
// ENUMERATIONS AND CONSTANTS
// ============================================================================

/**
 * @brief Motor control modes
 */
typedef enum {
    SVARV_MODE_IDLE = 0,        ///< Motor disabled, no control
    SVARV_MODE_TORQUE = 1,      ///< Direct torque control (current control)
    SVARV_MODE_VELOCITY = 2,    ///< Velocity control
    SVARV_MODE_POSITION = 3,    ///< Position control
    SVARV_MODE_CALIBRATION = 4  ///< Sensor and motor calibration
} SvarvControlMode;

/**
 * @brief Motor connection states
 */
typedef enum {
    SVARV_STATE_DISCONNECTED = 0,  ///< Motor not responding
    SVARV_STATE_CONNECTING = 1,    ///< Attempting connection
    SVARV_STATE_CONNECTED = 2,     ///< Connected and operational
    SVARV_STATE_ERROR = 3          ///< Connected but in error state
} SvarvConnectionState;

/**
 * @brief Error codes (matches firmware error_code_t)
 */
typedef enum {
    SVARV_ERROR_NONE = 0,
    SVARV_ERROR_COMMAND_TIMEOUT = 1,
    SVARV_ERROR_OVER_CURRENT = 2,
    SVARV_ERROR_OVER_VOLTAGE = 3,
    SVARV_ERROR_UNDER_VOLTAGE = 4,
    SVARV_ERROR_ENCODER_FAILURE = 5,
    SVARV_ERROR_DRIVER_FAILURE = 6,
    SVARV_ERROR_CALIBRATION_FAILED = 7,
    SVARV_ERROR_POSITION_LIMIT = 8,
    SVARV_ERROR_VELOCITY_LIMIT = 9,
    SVARV_ERROR_CURRENT_LIMIT = 10,
    SVARV_ERROR_COMMUNICATION = 11,
    SVARV_ERROR_WATCHDOG = 12,
    SVARV_ERROR_INVALID_PARAMETER = 13,
    SVARV_ERROR_MEMORY_CORRUPTION = 14
} SvarvErrorCode;

/**
 * @brief PID controller types
 */
typedef enum {
    SVARV_PID_POSITION = 0,
    SVARV_PID_VELOCITY = 1,
    SVARV_PID_CURRENT = 2
} SvarvPIDType;

/**
 * @brief Supported hardware platforms
 */
typedef enum {
    SVARV_PLATFORM_AUTO = 0,    ///< Auto-detect platform
    SVARV_PLATFORM_ESP32_BUILTIN = 1,  ///< ESP32 built-in CAN
    SVARV_PLATFORM_STM32_BUILTIN = 2,  ///< STM32 built-in CAN
    SVARV_PLATFORM_MCP2515 = 3         ///< MCP2515 external CAN controller
} SvarvPlatformType;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief PID controller parameters
 */
struct SvarvPID {
    float P = 0.0f;           ///< Proportional gain
    float I = 0.0f;           ///< Integral gain
    float D = 0.0f;           ///< Derivative gain
    float output_limit = 0.0f; ///< Output limit
    float integral_limit = 0.0f; ///< Integral windup limit

    SvarvPID() = default;
    SvarvPID(float p, float i, float d, float out_lim = 100.0f)
        : P(p), I(i), D(d), output_limit(out_lim), integral_limit(out_lim) {
    }
};

/**
 * @brief Motor status information
 */
struct SvarvMotorStatus {
    // Control state
    SvarvControlMode control_mode = SVARV_MODE_IDLE;
    bool enabled = false;
    bool calibrated = false;
    bool initializing = false;
    float target_value = 0.0f;

    // Measured values
    float position = 0.0f;        ///< Current position [rad]
    float velocity = 0.0f;        ///< Current velocity [rad/s]
    float current_q = 0.0f;       ///< Q-axis current [A]
    float current_d = 0.0f;       ///< D-axis current [A]
    float voltage_q = 0.0f;       ///< Q-axis voltage [V]
    float voltage_d = 0.0f;       ///< D-axis voltage [V]

    // Diagnostics
    uint16_t loop_time_us = 0;    ///< Control loop time [μs]
    SvarvErrorCode error_code = SVARV_ERROR_NONE;
    uint16_t error_count = 0;

    // Connection info
    SvarvConnectionState connection_state = SVARV_STATE_DISCONNECTED;
    unsigned long last_response_time = 0;
    uint8_t node_id = 0;
};

/**
 * @brief Motor configuration parameters
 */
struct SvarvMotorConfig {
    // System settings
    uint8_t node_id = 0;
    SvarvControlMode default_mode = SVARV_MODE_IDLE;

    // Motor parameters
    uint8_t pole_pairs = 7;
    float phase_resistance = 0.1f;    ///< [Ohm]
    float phase_inductance = 0.0001f; ///< [H]

    // Encoder settings
    uint16_t encoder_resolution = 4096;
    float encoder_offset = 0.0f;      ///< [rad]
    bool encoder_reversed = false;

    // PID controllers
    SvarvPID position_pid = SvarvPID(20.0f, 0.0f, 0.0f, 20.0f);
    SvarvPID velocity_pid = SvarvPID(0.5f, 10.0f, 0.0f, 12.0f);
    SvarvPID current_pid = SvarvPID(3.0f, 300.0f, 0.0f, 12.0f);

    // Safety limits
    float velocity_limit = 15.0f;     ///< [rad/s]
    float current_limit = 5.0f;       ///< [A]
    float voltage_limit = 5.0f;       ///< [V]
    uint16_t command_timeout = 1000;  ///< [ms]

    // Telemetry settings
    uint16_t telemetry_interval = 100; ///< [ms]
    uint8_t telemetry_channels = 0x03; ///< Enabled channels bitmask
};

/**
 * @brief Platform-specific configuration
 */
struct SvarvPlatformConfig {
    SvarvPlatformType platform = SVARV_PLATFORM_AUTO;

    // ESP32 specific
    int esp32_tx_pin = 21;
    int esp32_rx_pin = 22;

    // STM32 specific
    int stm32_can_instance = 1;  // CAN1 or CAN2

    // MCP2515 specific
    int mcp2515_cs_pin = 10;
    int mcp2515_int_pin = 2;
    uint32_t mcp2515_crystal_freq = MCP_8MHZ; // 8MHz, 16MHz, 20MHz

    // Common settings
    uint32_t can_speed = 1000000; // 1 Mbps default
};

// ============================================================================
// CALLBACK FUNCTION TYPES
// ============================================================================

/**
 * @brief Motor status update callback
 * @param motor Reference to the motor that was updated
 * @param status New status information
 */
typedef std::function<void(SvarvMotor& motor, const SvarvMotorStatus& status)> SvarvStatusCallback;

/**
 * @brief Error callback
 * @param motor Reference to the motor with error
 * @param error_code Error code
 * @param error_message Human-readable error description
 */
typedef std::function<void(SvarvMotor& motor, SvarvErrorCode error_code, const String& error_message)> SvarvErrorCallback;

/**
 * @brief Connection state change callback
 * @param motor Reference to the motor
 * @param old_state Previous connection state
 * @param new_state New connection state
 */
typedef std::function<void(SvarvMotor& motor, SvarvConnectionState old_state, SvarvConnectionState new_state)> SvarvConnectionCallback;

// ============================================================================
// PLATFORM ABSTRACTION LAYER
// ============================================================================

/**
 * @brief CAN message structure (platform-independent)
 */
struct SvarvCANMessage {
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
    bool extended;
    bool rtr;
};

/**
 * @brief Platform abstraction layer for CAN communication
 */
class SvarvCANInterface {
public:
    virtual ~SvarvCANInterface() {}

    /**
     * @brief Initialize CAN interface
     * @param config Platform configuration
     * @return true if successful
     */
    virtual bool begin(const SvarvPlatformConfig& config) = 0;

    /**
     * @brief Send CAN message
     * @param msg Message to send
     * @return true if successful
     */
    virtual bool sendMessage(const SvarvCANMessage& msg) = 0;

    /**
     * @brief Receive CAN message
     * @param msg Output message buffer
     * @return true if message received
     */
    virtual bool receiveMessage(SvarvCANMessage& msg) = 0;

    /**
     * @brief Check if CAN interface is available/healthy
     * @return true if operational
     */
    virtual bool isAvailable() = 0;

    /**
     * @brief Get platform name
     * @return Platform description string
     */
    virtual String getPlatformName() = 0;
};

// Platform-specific CAN implementations
#ifdef SVARV_PLATFORM_ESP32
class SvarvCANInterface_ESP32 : public SvarvCANInterface {
private:
    SvarvPlatformConfig config_;

public:
    bool begin(const SvarvPlatformConfig& config) override;
    bool sendMessage(const SvarvCANMessage& msg) override;
    bool receiveMessage(SvarvCANMessage& msg) override;
    bool isAvailable() override;
    String getPlatformName() override { return "ESP32 Built-in CAN"; }
};
#endif

#ifdef SVARV_PLATFORM_STM32
class SvarvCANInterface_STM32 : public SvarvCANInterface {
private:
    SvarvPlatformConfig config_;
    STM32_CAN* can_;

public:
    SvarvCANInterface_STM32();
    ~SvarvCANInterface_STM32();
    bool begin(const SvarvPlatformConfig& config) override;
    bool sendMessage(const SvarvCANMessage& msg) override;
    bool receiveMessage(SvarvCANMessage& msg) override;
    bool isAvailable() override;
    String getPlatformName() override { return "STM32 Built-in CAN"; }
};
#endif

#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
class SvarvCANInterface_MCP2515 : public SvarvCANInterface {
private:
    SvarvPlatformConfig config_;
    MCP_CAN* can_;

public:
    SvarvCANInterface_MCP2515();
    ~SvarvCANInterface_MCP2515();
    bool begin(const SvarvPlatformConfig& config) override;
    bool sendMessage(const SvarvCANMessage& msg) override;
    bool receiveMessage(SvarvCANMessage& msg) override;
    bool isAvailable() override;
    String getPlatformName() override { return "MCP2515 External CAN"; }
};
#endif

// ============================================================================
// SVARV MOTOR CLASS
// ============================================================================

/**
 * @brief Individual motor controller interface
 *
 * This class represents a single motor in the CAN network. It provides a clean,
 * object-oriented interface for motor control operations while handling all the
 * underlying CAN communication automatically.
 *
 * @note Motors are created and managed by SvarvMotionControl. Do not instantiate directly.
 */
class SvarvMotor {
    friend class SvarvMotionControl;

public:
    // ========================================================================
    // BASIC CONTROL METHODS
    // ========================================================================

    /**
     * @brief Set the control mode
     * @param mode New control mode
     * @return true if command sent successfully
     */
    bool setControlMode(SvarvControlMode mode);

    /**
     * @brief Move to absolute position (position mode only)
     * @param position Target position in radians
     * @return true if command sent successfully
     */
    bool moveTo(float position);

    /**
     * @brief Move relative to current position (position mode only)
     * @param delta Relative movement in radians
     * @return true if command sent successfully
     */
    bool moveBy(float delta);

    /**
     * @brief Set velocity target (velocity mode only)
     * @param velocity Target velocity in rad/s
     * @return true if command sent successfully
     */
    bool setVelocity(float velocity);

    /**
     * @brief Set torque target (torque mode only)
     * @param torque Target torque in Nm
     * @return true if command sent successfully
     */
    bool setTorque(float torque);

    /**
     * @brief Emergency stop - immediately disable motor
     * @return true if command sent successfully
     */
    bool emergencyStop();

    /**
     * @brief Enable motor (switch from IDLE to last active mode)
     * @return true if command sent successfully
     */
    bool enable();

    /**
     * @brief Disable motor (switch to IDLE mode)
     * @return true if command sent successfully
     */
    bool disable();

    // ========================================================================
    // CONFIGURATION METHODS
    // ========================================================================

    /**
     * @brief Set PID controller parameters
     * @param pid_type Which PID controller to configure
     * @param pid PID parameters
     * @return true if command sent successfully
     */
    bool setPID(SvarvPIDType pid_type, const SvarvPID& pid);

    /**
     * @brief Set individual PID gain
     * @param pid_type Which PID controller
     * @param P Proportional gain
     * @param I Integral gain
     * @param D Derivative gain
     * @return true if command sent successfully
     */
    bool setPID(SvarvPIDType pid_type, float P, float I, float D);

    /**
     * @brief Set motor limits
     * @param velocity_limit Maximum velocity [rad/s]
     * @param current_limit Maximum current [A]
     * @param voltage_limit Maximum voltage [V]
     * @return true if command sent successfully
     */
    bool setLimits(float velocity_limit, float current_limit, float voltage_limit);

    /**
     * @brief Save current configuration to motor's EEPROM
     * @return true if command sent successfully
     */
    bool saveConfig();

    /**
     * @brief Load configuration from motor's EEPROM
     * @return true if command sent successfully
     */
    bool loadConfig();

    /**
     * @brief Reset motor to factory defaults
     * @return true if command sent successfully
     */
    bool factoryReset();

    // ========================================================================
    // STATUS AND MONITORING
    // ========================================================================

    /**
     * @brief Get current motor status
     * @return Motor status structure
     */
    const SvarvMotorStatus& getStatus() const { return status_; }

    /**
     * @brief Get current position
     * @return Position in radians
     */
    float getPosition() const { return status_.position; }

    /**
     * @brief Get current velocity
     * @return Velocity in rad/s
     */
    float getVelocity() const { return status_.velocity; }

    /**
     * @brief Get current torque (estimated from current)
     * @return Torque in Nm
     */
    float getTorque() const;

    /**
     * @brief Check if motor is connected and responding
     * @return true if connected
     */
    bool isConnected() const { return status_.connection_state == SVARV_STATE_CONNECTED; }

    /**
     * @brief Check if motor is enabled
     * @return true if enabled
     */
    bool isEnabled() const { return status_.enabled; }

    /**
     * @brief Check if motor has errors
     * @return true if error present
     */
    bool hasError() const { return status_.error_code != SVARV_ERROR_NONE; }

    /**
     * @brief Get last error code
     * @return Error code
     */
    SvarvErrorCode getLastError() const { return status_.error_code; }

    /**
     * @brief Get human-readable error description
     * @return Error description string
     */
    String getErrorString() const;

    /**
     * @brief Clear error flags
     * @return true if command sent successfully
     */
    bool clearErrors();

    // ========================================================================
    // ADVANCED FEATURES
    // ========================================================================

    /**
     * @brief Check if position target has been reached
     * @param tolerance Position tolerance in radians (default: 0.05)
     * @return true if at target position
     */
    bool isAtTarget(float tolerance = 0.05f) const;

    /**
     * @brief Wait for position target to be reached (blocking)
     * @param timeout_ms Maximum wait time in milliseconds
     * @param tolerance Position tolerance in radians
     * @return true if target reached within timeout
     */
    bool waitForTarget(unsigned long timeout_ms = 5000, float tolerance = 0.05f);

    /**
     * @brief Request fresh status update from motor
     * @return true if request sent successfully
     */
    bool requestStatusUpdate();

    /**
     * @brief Get motor node ID
     * @return CAN node ID
     */
    uint8_t getNodeId() const { return config_.node_id; }

    /**
     * @brief Get motor configuration
     * @return Configuration structure
     */
    const SvarvMotorConfig& getConfig() const { return config_; }

    // ========================================================================
    // CALLBACK REGISTRATION
    // ========================================================================

    /**
     * @brief Set status update callback
     * @param callback Function to call when status updates
     */
    void onStatusUpdate(SvarvStatusCallback callback) { status_callback_ = callback; }

    /**
     * @brief Set error callback
     * @param callback Function to call when errors occur
     */
    void onError(SvarvErrorCallback callback) { error_callback_ = callback; }

    /**
     * @brief Set connection state change callback
     * @param callback Function to call when connection state changes
     */
    void onConnectionChange(SvarvConnectionCallback callback) { connection_callback_ = callback; }

private:
    // Private constructor - only SvarvMotionControl can create motors
    SvarvMotor(uint8_t node_id, SvarvMotionControl* controller);

    // Internal methods
    bool sendCommand(uint8_t cmd, const uint8_t* data = nullptr, uint8_t len = 0);
    bool sendConfig(uint8_t cmd, const uint8_t* data, uint8_t len);
    void updateStatus(const SvarvMotorStatus& new_status);
    void setConnectionState(SvarvConnectionState state);
    void handleTimeout();

    // Member variables
    SvarvMotionControl* controller_;
    SvarvMotorConfig config_;
    SvarvMotorStatus status_;
    SvarvControlMode last_active_mode_;
    unsigned long last_command_time_;

    // Callbacks
    SvarvStatusCallback status_callback_;
    SvarvErrorCallback error_callback_;
    SvarvConnectionCallback connection_callback_;
};

// ============================================================================
// SVARV MOTION CONTROL CLASS
// ============================================================================

/**
 * @brief Main motion control system interface
 *
 * This is the primary class for managing multiple CAN-based motor controllers
 * across different hardware platforms. It handles CAN bus initialization,
 * message routing, and provides methods for adding and managing multiple motors.
 *
 * @section PLATFORM_USAGE
 *
 * ESP32 Usage:
 * @code
 * SvarvMotionControl svarv;
 * svarv.begin(1000000); // Auto-detect ESP32, use default pins
 * // OR
 * svarv.begin(1000000, 21, 20); // Specify TX/RX pins
 * @endcode
 *
 * STM32 Usage:
 * @code
 * SvarvMotionControl svarv;
 * svarv.begin(1000000); // Auto-detect STM32, use CAN1
 * @endcode
 *
 * Arduino + MCP2515 Usage:
 * @code
 * SvarvMotionControl svarv;
 * svarv.begin(1000000, 10); // CS pin 10, default settings
 * // OR
 * svarv.begin(1000000, 10, 2, MCP_8MHZ); // CS pin 10, INT pin 2, 8MHz crystal
 * @endcode
 */
class SvarvMotionControl {
    friend class SvarvMotor;

public:
    // ========================================================================
    // INITIALIZATION AND SETUP
    // ========================================================================

    /**
     * @brief Constructor
     */
    SvarvMotionControl();

    /**
     * @brief Destructor
     */
    ~SvarvMotionControl();

    // Platform-specific begin methods

    /**
     * @brief Initialize with auto-platform detection
     * @param can_speed CAN bus speed in bps (default: 1000000 = 1 Mbps)
     * @return true if initialization successful
     */
    bool begin(uint32_t can_speed = 1000000);

#ifdef SVARV_PLATFORM_ESP32
    /**
     * @brief Initialize ESP32 with custom pins
     * @param can_speed CAN bus speed in bps
     * @param tx_pin CAN TX pin
     * @param rx_pin CAN RX pin
     * @return true if initialization successful
     */
    bool begin(uint32_t can_speed, int tx_pin, int rx_pin);
#endif

#ifdef SVARV_PLATFORM_STM32
    /**
     * @brief Initialize STM32 with specific CAN instance
     * @param can_speed CAN bus speed in bps
     * @param can_instance CAN instance (1 for CAN1, 2 for CAN2)
     * @return true if initialization successful
     */
    bool begin(uint32_t can_speed, int can_instance);
#endif

#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
    /**
     * @brief Initialize MCP2515 with custom pins
     * @param can_speed CAN bus speed in bps
     * @param cs_pin MCP2515 chip select pin
     * @param int_pin MCP2515 interrupt pin (optional, default: 2)
     * @param crystal_freq MCP2515 crystal frequency (default: MCP_8MHZ)
     * @return true if initialization successful
     */
    bool begin(uint32_t can_speed, int cs_pin, int int_pin = 2, uint32_t crystal_freq = MCP_8MHZ);
#endif

    /**
     * @brief Initialize with explicit platform configuration
     * @param config Platform-specific configuration
     * @return true if initialization successful
     */
    bool begin(const SvarvPlatformConfig& config);

    /**
     * @brief Add a motor to the control system
     * @param node_id CAN node ID of the motor (1-255)
     * @return Motor object for control operations
     * @note If motor with this node_id already exists, returns existing motor
     */
    SvarvMotor& addMotor(uint8_t node_id);

    /**
     * @brief Remove a motor from the control system
     * @param node_id CAN node ID to remove
     * @return true if motor was found and removed
     */
    bool removeMotor(uint8_t node_id);

    /**
     * @brief check if a motor from the control system is available
     * @param node_id CAN node ID to chcek
     * @return true if motor was found and checked
     */
    bool isNodeIdAvailable(uint8_t node_id);

    
    int configureMotorRange(uint8_t start_id = 1, uint8_t end_id = 10, bool force_reconfigure = false);


    /**
     * @brief Get motor by node ID
     * @param node_id CAN node ID
     * @return Pointer to motor object, or nullptr if not found
     */
    SvarvMotor* getMotor(uint8_t node_id);

    /**
     * @brief Get all motors
     * @return Vector of pointers to all motors
     */
    std::vector<SvarvMotor*> getAllMotors();

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    /**
     * @brief Update the motion control system
     *
     * This method MUST be called frequently in your main loop. It handles:
     * - Processing incoming CAN messages
     * - Updating motor status
     * - Managing timeouts and connections
     * - Triggering callbacks
     *
     * @note Call this at least every 10ms for optimal performance
     */
    void update();

    // ========================================================================
    // SYSTEM-WIDE OPERATIONS
    // ========================================================================

    /**
     * @brief Emergency stop all motors
     * @return Number of motors successfully stopped
     */
    int emergencyStopAll();

    /**
     * @brief Enable all motors
     * @return Number of motors successfully enabled
     */
    int enableAll();

    /**
     * @brief Disable all motors
     * @return Number of motors successfully disabled
     */
    int disableAll();

    /**
     * @brief Scan for motors on the CAN bus
     * @param timeout_ms Scan timeout in milliseconds
     * @return Vector of discovered node IDs
     */
    std::vector<uint8_t> scanForMotors(unsigned long timeout_ms = 5000);

    /**
     * @brief Auto-configure motor node IDs
     *
     * This function will scan for unconfigured motors (node ID = 0) and assign
     * sequential node IDs starting from start_id.
     *
     * @param start_id Starting node ID for assignment
     * @param max_motors Maximum number of motors to configure
     * @return Number of motors successfully configured
     */
    bool configureSingleMotor(uint8_t target_id);


    std::vector<uint8_t> scanForUnconfiguredMotors(unsigned long timeout_ms = 8000);



    // ========================================================================
    // SYSTEM STATUS AND DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get number of connected motors
     * @return Count of connected motors
     */
    int getConnectedMotorCount() const;

    /**
     * @brief Get CAN bus statistics
     * @param messages_sent Output: Total messages sent
     * @param messages_received Output: Total messages received
     * @param errors Output: Total CAN errors
     */
    void getCANStatistics(uint32_t& messages_sent, uint32_t& messages_received, uint32_t& errors) const;

    /**
     * @brief Check if CAN bus is healthy
     * @return true if CAN bus is operating normally
     */
    bool isCANHealthy() const;

    /**
     * @brief Get detected platform information
     * @return Platform description string
     */
    String getPlatformInfo() const;

    /**
     * @brief Get library version
     * @return Version string (e.g., "2.0.0")
     */
    static String getVersion() { return SVARV_VERSION; }

    // ========================================================================
    // GLOBAL CALLBACKS
    // ========================================================================

    /**
     * @brief Set global error callback (called for any motor error)
     * @param callback Function to call when any motor has an error
     */
    void onAnyError(SvarvErrorCallback callback) { global_error_callback_ = callback; }

    /**
     * @brief Set global connection callback (called for any connection change)
     * @param callback Function to call when any motor connection changes
     */
    void onAnyConnectionChange(SvarvConnectionCallback callback) { global_connection_callback_ = callback; }

    /**
     * @brief Enable debug output
     * @param enable true to enable debug output to Serial
     */
    void enableDebug(bool enable = true) { debug_enabled_ = enable; }

private:
    // Internal CAN communication
    bool sendCANMessage(uint8_t node_id, uint8_t function_id, uint8_t cmd,
        const uint8_t* data = nullptr, uint8_t len = 0);
    void processIncomingMessages();
    void handleResponseMessage(uint8_t node_id, const SvarvCANMessage& msg);
    void handleTelemetryMessage(uint8_t node_id, const SvarvCANMessage& msg);
    void handleErrorMessage(uint8_t node_id, const SvarvCANMessage& msg);

    // Platform detection and setup
    SvarvPlatformType detectPlatform();
    SvarvCANInterface* createCANInterface(SvarvPlatformType platform);

    // Internal motor management
    void updateMotorTimeouts();
    void debugPrint(const String& message);

    // Member variables
    SimpleMap<uint8_t, SvarvMotor*> motors_;
    SvarvCANInterface* can_interface_;
    SvarvPlatformConfig platform_config_;
    bool initialized_;
    bool debug_enabled_;

    // CAN statistics
    uint32_t messages_sent_;
    uint32_t messages_received_;
    uint32_t can_errors_;

    // Global callbacks
    SvarvErrorCallback global_error_callback_;
    SvarvConnectionCallback global_connection_callback_;

    // Timing
    unsigned long last_update_time_;
    unsigned long last_scan_time_;
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Convert control mode enum to human-readable string
 * @param mode Control mode
 * @return Mode name string
 */
String svarvModeToString(SvarvControlMode mode);

/**
 * @brief Convert error code to human-readable string
 * @param error Error code
 * @return Error description string
 */
String svarvErrorToString(SvarvErrorCode error);

/**
 * @brief Convert connection state to human-readable string
 * @param state Connection state
 * @return State name string
 */
String svarvConnectionStateToString(SvarvConnectionState state);

/**
 * @brief Convert platform type to human-readable string
 * @param platform Platform type
 * @return Platform name string
 */
String svarvPlatformToString(SvarvPlatformType platform);

/**
 * @brief Auto-detect the current hardware platform
 * @return Detected platform type
 */
SvarvPlatformType svarvDetectPlatform();

#endif // SVARV_MOTION_CONTROL_H