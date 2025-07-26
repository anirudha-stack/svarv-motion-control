/**
 * @file SvarvMotionControl.h
 * @brief Svarv Motion Control Library - Arduino CAN-based BLDC Motor Control
 * @version 1.0.0
 * @date 2025-07-26
 * @author Svarv Robotics
 * 
 * @section DESCRIPTION
 * 
 * The Svarv Motion Control Library provides a simple, intuitive interface for controlling
 * multiple CAN-based BLDC motor controllers. Inspired by industry-leading libraries like
 * SimpleFOC, ODrive, and VESC, this library abstracts the complexity of CAN communication
 * while providing full access to advanced motor control features.
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
 *   svarv.begin(1000000); // 1 Mbps CAN
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
 * - ESP32 (ESP32-C3, ESP32-S3, ESP32 Classic)
 * - Arduino IDE 1.8.0+
 * - PlatformIO
 * - Svarv Motor Controller Firmware v1.2+
 */

#ifndef SVARV_MOTION_CONTROL_H
#define SVARV_MOTION_CONTROL_H

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <functional>
#include <vector>
#include <map>

// Library version
#define SVARV_VERSION_MAJOR 1
#define SVARV_VERSION_MINOR 0
#define SVARV_VERSION_PATCH 0
#define SVARV_VERSION "1.0.0"

// Forward declarations
class SvarvMotor;
class SvarvMotionControl;

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
        : P(p), I(i), D(d), output_limit(out_lim), integral_limit(out_lim) {}
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
 * This is the primary class for managing multiple CAN-based motor controllers.
 * It handles CAN bus initialization, message routing, and provides methods for
 * adding and managing multiple motors.
 * 
 * @section USAGE
 * 
 * @code
 * SvarvMotionControl svarv;
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   
 *   // Initialize CAN bus
 *   if (!svarv.begin(1000000)) {
 *     Serial.println("CAN initialization failed!");
 *     while(1);
 *   }
 *   
 *   // Add motors
 *   SvarvMotor motor1 = svarv.addMotor(1);
 *   SvarvMotor motor2 = svarv.addMotor(2);
 *   
 *   // Configure motors
 *   motor1.setControlMode(SVARV_MODE_POSITION);
 *   motor2.setControlMode(SVARV_MODE_VELOCITY);
 * }
 * 
 * void loop() {
 *   svarv.update(); // Essential - call this frequently!
 * }
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
    
    /**
     * @brief Initialize CAN bus and motion control system
     * @param can_speed CAN bus speed in bps (default: 1000000 = 1 Mbps)
     * @param tx_pin CAN TX pin (default: 21)
     * @param rx_pin CAN RX pin (default: 20)
     * @return true if initialization successful
     */
    bool begin(uint32_t can_speed = 1000000, int tx_pin = 21, int rx_pin = 20);
    
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
    int autoConfigureMotors(uint8_t start_id = 1, uint8_t max_motors = 10);
    
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
     * @brief Get library version
     * @return Version string (e.g., "1.0.0")
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
    void handleResponseMessage(uint8_t node_id, const CanFrame& frame);
    void handleTelemetryMessage(uint8_t node_id, const CanFrame& frame);
    void handleErrorMessage(uint8_t node_id, const CanFrame& frame);
    
    // Internal motor management
    void updateMotorTimeouts();
    void debugPrint(const String& message);
    
    // Member variables
    std::map<uint8_t, SvarvMotor*> motors_;
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

#endif // SVARV_MOTION_CONTROL_H