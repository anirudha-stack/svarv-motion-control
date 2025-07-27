/**
 * @file SvarvMotionControl.cpp
 * @brief Svarv Motion Control Library Implementation - Cross-Platform Support
 * @version 2.0.0
 * @date 2025-07-26
 */

#include "SvarvMotionControl.h"

// CAN Protocol Constants (from firmware)
#define CAN_FUNCTION_COMMAND      0b000
#define CAN_FUNCTION_CONFIG       0b001
#define CAN_FUNCTION_QUERY        0b010
#define CAN_FUNCTION_RESPONSE     0b011
#define CAN_FUNCTION_TELEMETRY    0b100
#define CAN_FUNCTION_ERROR        0b101
#define CAN_FUNCTION_SYSTEM       0b110

// Command IDs (from firmware)
#define CMD_SET_MODE               0x01
#define CMD_SET_TARGET             0x02
#define CMD_EMERGENCY_STOP         0x03
#define CMD_CLEAR_ERRORS           0x04

// Split PID configuration commands
#define CMD_CONFIG_VEL_PID_P       0x10
#define CMD_CONFIG_VEL_PID_I       0x11
#define CMD_CONFIG_VEL_PID_D       0x12
#define CMD_CONFIG_POS_PID_P       0x13
#define CMD_CONFIG_POS_PID_I       0x14
#define CMD_CONFIG_POS_PID_D       0x15
#define CMD_CONFIG_CUR_PID_P       0x16
#define CMD_CONFIG_CUR_PID_I       0x17
#define CMD_CONFIG_CUR_PID_D       0x18

// Limit configuration
#define CMD_CONFIG_VEL_LIMIT       0x19
#define CMD_CONFIG_CUR_LIMIT       0x1A
#define CMD_CONFIG_VOLT_LIMIT      0x1B

// Configuration management
#define CMD_SAVE_CONFIG            0x30
#define CMD_LOAD_CONFIG            0x31
#define CMD_FACTORY_RESET          0x32

// Query commands
#define CMD_QUERY_STATUS           0x40
#define CMD_QUERY_CONFIG           0x41
#define CMD_QUERY_PID_VALUES       0x43

// System commands
#define CMD_SYS_REBOOT             0x50
#define CMD_SYS_SET_NODE_ID        0x51
#define CMD_SYS_GET_VERSION        0x52

// Telemetry IDs
#define TELEMETRY_HEARTBEAT        0x01
#define TELEMETRY_MOTOR_STATE      0x02
#define TELEMETRY_MOTOR_POWER      0x03
#define TELEMETRY_DIAGNOSTICS      0x04

// Timeouts and intervals
#define CONNECTION_TIMEOUT_MS      3000
#define STATUS_REQUEST_INTERVAL_MS 1000
#define MAX_RESPONSE_WAIT_MS       500

// ============================================================================
// PLATFORM-SPECIFIC CAN INTERFACE IMPLEMENTATIONS
// ============================================================================

#ifdef SVARV_PLATFORM_ESP32
bool SvarvCANInterface_ESP32::begin(const SvarvPlatformConfig& config) {
    config_ = config;
    
    // Configure CAN pins and settings
    ESP32Can.setPins(config_.esp32_tx_pin, config_.esp32_rx_pin);
    ESP32Can.setRxQueueSize(32);
    ESP32Can.setTxQueueSize(16);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(config_.can_speed));
    
    // Initialize CAN bus
    return ESP32Can.begin();
}

bool SvarvCANInterface_ESP32::sendMessage(const SvarvCANMessage& msg) {
    CanFrame frame = {0};
    frame.identifier = msg.id;
    frame.extd = msg.extended ? 1 : 0;
    frame.rtr = msg.rtr ? 1 : 0;
    frame.data_length_code = msg.length;
    
    for (int i = 0; i < msg.length && i < 8; i++) {
        frame.data[i] = msg.data[i];
    }
    
    return ESP32Can.writeFrame(frame);
}

bool SvarvCANInterface_ESP32::receiveMessage(SvarvCANMessage& msg) {
    CanFrame frame;
    if (ESP32Can.readFrame(frame, 1)) {
        msg.id = frame.identifier;
        msg.extended = frame.extd != 0;
        msg.rtr = frame.rtr != 0;
        msg.length = frame.data_length_code;
        
        for (int i = 0; i < msg.length && i < 8; i++) {
            msg.data[i] = frame.data[i];
        }
        
        return true;
    }
    return false;
}

bool SvarvCANInterface_ESP32::isAvailable() {
    return true; // TO BE FIXED
}
#endif // SVARV_PLATFORM_ESP32

#ifdef SVARV_PLATFORM_STM32
SvarvCANInterface_STM32::SvarvCANInterface_STM32() : can_(nullptr) {}

SvarvCANInterface_STM32::~SvarvCANInterface_STM32() {
    delete can_;
}

bool SvarvCANInterface_STM32::begin(const SvarvPlatformConfig& config) {
    config_ = config;
    
    // Create CAN instance based on configuration
    if (config_.stm32_can_instance == 1) {
        can_ = new STM32_CAN(CAN1);
    } else if (config_.stm32_can_instance == 2) {
        can_ = new STM32_CAN(CAN2);
    } else {
        return false;
    }
    
    // Configure CAN speed
    can_->begin();
    can_->setBaudRate(config_.can_speed);
    
    return true;
}

bool SvarvCANInterface_STM32::sendMessage(const SvarvCANMessage& msg) {
    if (!can_) return false;
    
    CAN_message_t stm32_msg;
    stm32_msg.id = msg.id;
    stm32_msg.ext = msg.extended;
    stm32_msg.rtr = msg.rtr;
    stm32_msg.len = msg.length;
    
    for (int i = 0; i < msg.length && i < 8; i++) {
        stm32_msg.buf[i] = msg.data[i];
    }
    
    return can_->write(stm32_msg) > 0;
}

bool SvarvCANInterface_STM32::receiveMessage(SvarvCANMessage& msg) {
    if (!can_) return false;
    
    CAN_message_t stm32_msg;
    if (can_->read(stm32_msg)) {
        msg.id = stm32_msg.id;
        msg.extended = stm32_msg.ext;
        msg.rtr = stm32_msg.rtr;
        msg.length = stm32_msg.len;
        
        for (int i = 0; i < msg.length && i < 8; i++) {
            msg.data[i] = stm32_msg.buf[i];
        }
        
        return true;
    }
    return false;
}

bool SvarvCANInterface_STM32::isAvailable() {
    return can_ != nullptr;
}
#endif // SVARV_PLATFORM_STM32

#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
SvarvCANInterface_MCP2515::SvarvCANInterface_MCP2515() : can_(nullptr) {}

SvarvCANInterface_MCP2515::~SvarvCANInterface_MCP2515() {
    delete can_;
}

bool SvarvCANInterface_MCP2515::begin(const SvarvPlatformConfig& config) {
    config_ = config;
    
    // Initialize SPI
    SPI.begin();
    
    // Create MCP_CAN instance
    can_ = new MCP_CAN(config_.mcp2515_cs_pin);
    
    // Configure interrupt pin if specified
    if (config_.mcp2515_int_pin >= 0) {
        pinMode(config_.mcp2515_int_pin, INPUT);
    }
    
    // Convert CAN speed to MCP2515 constants
    uint8_t can_speed_cfg;
    switch (config_.can_speed) {
        case 1000000: can_speed_cfg = CAN_1000KBPS; break;
        case 500000:  can_speed_cfg = CAN_500KBPS; break;
        case 250000:  can_speed_cfg = CAN_250KBPS; break;
        case 125000:  can_speed_cfg = CAN_125KBPS; break;
        case 100000:  can_speed_cfg = CAN_100KBPS; break;
        case 50000:   can_speed_cfg = CAN_50KBPS; break;
        case 20000:   can_speed_cfg = CAN_20KBPS; break;
        case 10000:   can_speed_cfg = CAN_10KBPS; break;
        case 5000:    can_speed_cfg = CAN_5KBPS; break;
        default:      can_speed_cfg = CAN_500KBPS; break;
    }
    
    // Initialize MCP2515
    return can_->begin(can_speed_cfg, config_.mcp2515_crystal_freq) == CAN_OK;
}

bool SvarvCANInterface_MCP2515::sendMessage(const SvarvCANMessage& msg) {
    if (!can_) return false;
    
    uint8_t result = can_->sendMsgBuf(msg.id, msg.extended ? 1 : 0, msg.length, msg.data);
    return result == CAN_OK;
}

bool SvarvCANInterface_MCP2515::receiveMessage(SvarvCANMessage& msg) {
    if (!can_) return false;
    
    if (can_->checkReceive() == CAN_MSGAVAIL) {
        uint32_t id;
        uint8_t len;
        uint8_t buf[8];
        
        if (can_->readMsgBuf(&id, &len, buf) == CAN_OK) {
            msg.id = id;
            msg.extended = (id & 0x80000000) != 0;
            msg.rtr = false; // MCP2515 library handles RTR differently
            msg.length = len;
            
            for (int i = 0; i < len && i < 8; i++) {
                msg.data[i] = buf[i];
            }
            
            return true;
        }
    }
    return false;
}

bool SvarvCANInterface_MCP2515::isAvailable() {
    return can_ != nullptr;
}
#endif // SVARV_PLATFORM_AVR || SVARV_PLATFORM_GENERIC

// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

String svarvModeToString(SvarvControlMode mode) {
    switch (mode) {
        case SVARV_MODE_IDLE: return "IDLE";
        case SVARV_MODE_TORQUE: return "TORQUE";
        case SVARV_MODE_VELOCITY: return "VELOCITY";
        case SVARV_MODE_POSITION: return "POSITION";
        case SVARV_MODE_CALIBRATION: return "CALIBRATION";
        default: return "UNKNOWN";
    }
}

String svarvErrorToString(SvarvErrorCode error) {
    switch (error) {
        case SVARV_ERROR_NONE: return "No Error";
        case SVARV_ERROR_COMMAND_TIMEOUT: return "Command Timeout";
        case SVARV_ERROR_OVER_CURRENT: return "Over Current";
        case SVARV_ERROR_OVER_VOLTAGE: return "Over Voltage";
        case SVARV_ERROR_UNDER_VOLTAGE: return "Under Voltage";
        case SVARV_ERROR_ENCODER_FAILURE: return "Encoder Failure";
        case SVARV_ERROR_DRIVER_FAILURE: return "Driver Failure";
        case SVARV_ERROR_CALIBRATION_FAILED: return "Calibration Failed";
        case SVARV_ERROR_POSITION_LIMIT: return "Position Limit";
        case SVARV_ERROR_VELOCITY_LIMIT: return "Velocity Limit";
        case SVARV_ERROR_CURRENT_LIMIT: return "Current Limit";
        case SVARV_ERROR_COMMUNICATION: return "Communication Error";
        case SVARV_ERROR_WATCHDOG: return "Watchdog Timeout";
        case SVARV_ERROR_INVALID_PARAMETER: return "Invalid Parameter";
        case SVARV_ERROR_MEMORY_CORRUPTION: return "Memory Corruption";
        default: return "Unknown Error";
    }
}

String svarvConnectionStateToString(SvarvConnectionState state) {
    switch (state) {
        case SVARV_STATE_DISCONNECTED: return "Disconnected";
        case SVARV_STATE_CONNECTING: return "Connecting";
        case SVARV_STATE_CONNECTED: return "Connected";
        case SVARV_STATE_ERROR: return "Error";
        default: return "Unknown";
    }
}

String svarvPlatformToString(SvarvPlatformType platform) {
    switch (platform) {
        case SVARV_PLATFORM_AUTO: return "Auto-Detect";
        case SVARV_PLATFORM_ESP32_BUILTIN: return "ESP32 Built-in CAN";
        case SVARV_PLATFORM_STM32_BUILTIN: return "STM32 Built-in CAN";
        case SVARV_PLATFORM_MCP2515: return "MCP2515 External CAN";
        default: return "Unknown Platform";
    }
}

SvarvPlatformType svarvDetectPlatform() {
#if defined(ESP32)
    return SVARV_PLATFORM_ESP32_BUILTIN;
#elif defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
    return SVARV_PLATFORM_STM32_BUILTIN;
#else
    return SVARV_PLATFORM_MCP2515;
#endif
}

// ============================================================================
// SVARV MOTOR CLASS IMPLEMENTATION
// ============================================================================

SvarvMotor::SvarvMotor(uint8_t node_id, SvarvMotionControl* controller)
    : controller_(controller), last_active_mode_(SVARV_MODE_POSITION), last_command_time_(0) {
    config_.node_id = node_id;
    status_.node_id = node_id;
    status_.connection_state = SVARV_STATE_DISCONNECTED;
}

bool SvarvMotor::setControlMode(SvarvControlMode mode) {
    if (!isConnected()) {
        return false;
    }
    
    uint8_t mode_data = static_cast<uint8_t>(mode);
    bool result = sendCommand(CMD_SET_MODE, &mode_data, 1);
    
    if (result) {
        if (mode != SVARV_MODE_IDLE) {
            last_active_mode_ = mode;
        }
        status_.control_mode = mode;
    }
    
    return result;
}

bool SvarvMotor::moveTo(float position) {
    if (status_.control_mode != SVARV_MODE_POSITION) {
        if (!setControlMode(SVARV_MODE_POSITION)) {
            return false;
        }
    }
    
    return sendCommand(CMD_SET_TARGET, reinterpret_cast<const uint8_t*>(&position), sizeof(float));
}

bool SvarvMotor::moveBy(float delta) {
    return moveTo(status_.position + delta);
}

bool SvarvMotor::setVelocity(float velocity) {
    if (status_.control_mode != SVARV_MODE_VELOCITY) {
        if (!setControlMode(SVARV_MODE_VELOCITY)) {
            return false;
        }
    }
    
    return sendCommand(CMD_SET_TARGET, reinterpret_cast<const uint8_t*>(&velocity), sizeof(float));
}

bool SvarvMotor::setTorque(float torque) {
    if (status_.control_mode != SVARV_MODE_TORQUE) {
        if (!setControlMode(SVARV_MODE_TORQUE)) {
            return false;
        }
    }
    
    return sendCommand(CMD_SET_TARGET, reinterpret_cast<const uint8_t*>(&torque), sizeof(float));
}

bool SvarvMotor::emergencyStop() {
    bool result = sendCommand(CMD_EMERGENCY_STOP);
    if (result) {
        status_.control_mode = SVARV_MODE_IDLE;
        status_.enabled = false;
        status_.target_value = 0.0f;
    }
    return result;
}

bool SvarvMotor::enable() {
    return setControlMode(last_active_mode_);
}

bool SvarvMotor::disable() {
    return setControlMode(SVARV_MODE_IDLE);
}

bool SvarvMotor::setPID(SvarvPIDType pid_type, const SvarvPID& pid) {
    return setPID(pid_type, pid.P, pid.I, pid.D);
}

bool SvarvMotor::setPID(SvarvPIDType pid_type, float P, float I, float D) {
    if (!isConnected()) {
        return false;
    }
    
    uint8_t p_cmd, i_cmd, d_cmd;
    
    // Determine which PID commands to use
    switch (pid_type) {
        case SVARV_PID_POSITION:
            p_cmd = CMD_CONFIG_POS_PID_P;
            i_cmd = CMD_CONFIG_POS_PID_I;
            d_cmd = CMD_CONFIG_POS_PID_D;
            config_.position_pid.P = P;
            config_.position_pid.I = I;
            config_.position_pid.D = D;
            break;
            
        case SVARV_PID_VELOCITY:
            p_cmd = CMD_CONFIG_VEL_PID_P;
            i_cmd = CMD_CONFIG_VEL_PID_I;
            d_cmd = CMD_CONFIG_VEL_PID_D;
            config_.velocity_pid.P = P;
            config_.velocity_pid.I = I;
            config_.velocity_pid.D = D;
            break;
            
        case SVARV_PID_CURRENT:
            p_cmd = CMD_CONFIG_CUR_PID_P;
            i_cmd = CMD_CONFIG_CUR_PID_I;
            d_cmd = CMD_CONFIG_CUR_PID_D;
            config_.current_pid.P = P;
            config_.current_pid.I = I;
            config_.current_pid.D = D;
            break;
            
        default:
            return false;
    }
    
    // Send the three PID parameters separately
    bool success = true;
    success &= sendConfig(p_cmd, reinterpret_cast<const uint8_t*>(&P), sizeof(float));
    delay(50); // Small delay between commands
    success &= sendConfig(i_cmd, reinterpret_cast<const uint8_t*>(&I), sizeof(float));
    delay(50);
    success &= sendConfig(d_cmd, reinterpret_cast<const uint8_t*>(&D), sizeof(float));
    
    return success;
}

bool SvarvMotor::setLimits(float velocity_limit, float current_limit, float voltage_limit) {
    if (!isConnected()) {
        return false;
    }
    
    bool success = true;
    success &= sendConfig(CMD_CONFIG_VEL_LIMIT, reinterpret_cast<const uint8_t*>(&velocity_limit), sizeof(float));
    delay(50);
    success &= sendConfig(CMD_CONFIG_CUR_LIMIT, reinterpret_cast<const uint8_t*>(&current_limit), sizeof(float));
    delay(50);
    success &= sendConfig(CMD_CONFIG_VOLT_LIMIT, reinterpret_cast<const uint8_t*>(&voltage_limit), sizeof(float));
    
    if (success) {
        config_.velocity_limit = velocity_limit;
        config_.current_limit = current_limit;
        config_.voltage_limit = voltage_limit;
    }
    
    return success;
}

bool SvarvMotor::saveConfig() {
    return sendConfig(CMD_SAVE_CONFIG, nullptr, 0);
}

bool SvarvMotor::loadConfig() {
    return sendConfig(CMD_LOAD_CONFIG, nullptr, 0);
}

bool SvarvMotor::factoryReset() {
    return sendConfig(CMD_FACTORY_RESET, nullptr, 0);
}

float SvarvMotor::getTorque() const {
    // Estimate torque from current (simplified calculation)
    // In reality, torque = Kt * I_q, where Kt is the torque constant
    const float estimated_kt = 0.1f; // Estimated torque constant [Nm/A]
    return estimated_kt * status_.current_q;
}

String SvarvMotor::getErrorString() const {
    return svarvErrorToString(status_.error_code);
}

bool SvarvMotor::clearErrors() {
    bool result = sendCommand(CMD_CLEAR_ERRORS);
    if (result) {
        status_.error_code = SVARV_ERROR_NONE;
    }
    return result;
}

bool SvarvMotor::isAtTarget(float tolerance) const {
    if (status_.control_mode != SVARV_MODE_POSITION) {
        return false;
    }
    
    return fabs(status_.position - status_.target_value) <= tolerance;
}

bool SvarvMotor::waitForTarget(unsigned long timeout_ms, float tolerance) {
    unsigned long start_time = millis();
    
    while (millis() - start_time < timeout_ms) {
        controller_->update(); // Keep processing messages
        
        if (isAtTarget(tolerance)) {
            return true;
        }
        
        delay(10); // Small delay to prevent busy waiting
    }
    
    return false;
}

bool SvarvMotor::requestStatusUpdate() {
    uint8_t query_cmd = CMD_QUERY_STATUS;
    return controller_->sendCANMessage(config_.node_id, CAN_FUNCTION_QUERY, query_cmd);
}

bool SvarvMotor::sendCommand(uint8_t cmd, const uint8_t* data, uint8_t len) {
    last_command_time_ = millis();
    return controller_->sendCANMessage(config_.node_id, CAN_FUNCTION_COMMAND, cmd, data, len);
}

bool SvarvMotor::sendConfig(uint8_t cmd, const uint8_t* data, uint8_t len) {
    return controller_->sendCANMessage(config_.node_id, CAN_FUNCTION_CONFIG, cmd, data, len);
}

void SvarvMotor::updateStatus(const SvarvMotorStatus& new_status) {
    SvarvMotorStatus old_status = status_;
    status_ = new_status;
    status_.node_id = config_.node_id; // Preserve node ID
    status_.last_response_time = millis();
    
    // Update connection state based on successful communication
    if (status_.connection_state == SVARV_STATE_DISCONNECTED || 
        status_.connection_state == SVARV_STATE_CONNECTING) {
        setConnectionState(SVARV_STATE_CONNECTED);
    }
    
    // Trigger status callback if registered
    if (status_callback_) {
        status_callback_(*this, status_);
    }
    
    // Check for new errors
    if (status_.error_code != SVARV_ERROR_NONE && 
        status_.error_code != old_status.error_code && 
        error_callback_) {
        error_callback_(*this, status_.error_code, getErrorString());
    }
}

void SvarvMotor::setConnectionState(SvarvConnectionState state) {
    SvarvConnectionState old_state = status_.connection_state;
    status_.connection_state = state;
    
    if (old_state != state && connection_callback_) {
        connection_callback_(*this, old_state, state);
    }
}

void SvarvMotor::handleTimeout() {
    if (millis() - status_.last_response_time > CONNECTION_TIMEOUT_MS) {
        if (status_.connection_state == SVARV_STATE_CONNECTED) {
            setConnectionState(SVARV_STATE_DISCONNECTED);
        }
    }
}

// ============================================================================
// SVARV MOTION CONTROL CLASS IMPLEMENTATION
// ============================================================================

SvarvMotionControl::SvarvMotionControl()
    : can_interface_(nullptr), initialized_(false), debug_enabled_(false), 
      messages_sent_(0), messages_received_(0), can_errors_(0), 
      last_update_time_(0), last_scan_time_(0) {
}

SvarvMotionControl::~SvarvMotionControl() {
    // Clean up motors
#ifdef ARDUINO_ARCH_AVR
    for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
        if (motors_.data[i].used) {
            delete motors_.data[i].value;
        }
    }
#else
    for (auto& pair : motors_) {
        delete pair.second;
    }
#endif
    motors_.clear();
    
    delete can_interface_;
}

bool SvarvMotionControl::begin(uint32_t can_speed) {
    // Auto-detect platform and use default settings
    SvarvPlatformConfig config;
    config.platform = detectPlatform();
    config.can_speed = can_speed;
    
    return begin(config);
}

#ifdef SVARV_PLATFORM_ESP32
bool SvarvMotionControl::begin(uint32_t can_speed, int tx_pin, int rx_pin) {
    SvarvPlatformConfig config;
    config.platform = SVARV_PLATFORM_ESP32_BUILTIN;
    config.can_speed = can_speed;
    config.esp32_tx_pin = tx_pin;
    config.esp32_rx_pin = rx_pin;
    
    return begin(config);
}
#endif

#ifdef SVARV_PLATFORM_STM32
bool SvarvMotionControl::begin(uint32_t can_speed, int can_instance) {
    SvarvPlatformConfig config;
    config.platform = SVARV_PLATFORM_STM32_BUILTIN;
    config.can_speed = can_speed;
    config.stm32_can_instance = can_instance;
    
    return begin(config);
}
#endif

#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
bool SvarvMotionControl::begin(uint32_t can_speed, int cs_pin, int int_pin, uint32_t crystal_freq) {
    SvarvPlatformConfig config;
    config.platform = SVARV_PLATFORM_MCP2515;
    config.can_speed = can_speed;
    config.mcp2515_cs_pin = cs_pin;
    config.mcp2515_int_pin = int_pin;
    config.mcp2515_crystal_freq = crystal_freq;
    
    return begin(config);
}
#endif

bool SvarvMotionControl::begin(const SvarvPlatformConfig& config) {
    debugPrint("Svarv Motion Control v" + String(SVARV_VERSION) + " Initializing...");
    debugPrint("Platform: " + svarvPlatformToString(config.platform));
    
    platform_config_ = config;
    
    // Create appropriate CAN interface
    can_interface_ = createCANInterface(config.platform);
    if (!can_interface_) {
        debugPrint("ERROR: Failed to create CAN interface for platform!");
        return false;
    }
    
    // Initialize CAN interface
    if (!can_interface_->begin(config)) {
        debugPrint("ERROR: CAN interface initialization failed!");
        delete can_interface_;
        can_interface_ = nullptr;
        return false;
    }
    
    debugPrint("CAN interface initialized: " + can_interface_->getPlatformName());
    debugPrint("CAN Speed: " + String(config.can_speed) + " bps");
    
    initialized_ = true;
    last_update_time_ = millis();
    
    debugPrint("Svarv Motion Control ready!");
    return true;
}

SvarvPlatformType SvarvMotionControl::detectPlatform() {
    return svarvDetectPlatform();
}

SvarvCANInterface* SvarvMotionControl::createCANInterface(SvarvPlatformType platform) {
    switch (platform) {
#ifdef SVARV_PLATFORM_ESP32
        case SVARV_PLATFORM_ESP32_BUILTIN:
        case SVARV_PLATFORM_AUTO:
            #if defined(ESP32)
            return new SvarvCANInterface_ESP32();
            #endif
            break;
#endif

#ifdef SVARV_PLATFORM_STM32
        case SVARV_PLATFORM_STM32_BUILTIN:
            #if defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
            return new SvarvCANInterface_STM32();
            #endif
            break;
#endif

#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
        case SVARV_PLATFORM_MCP2515:
            return new SvarvCANInterface_MCP2515();
            break;
#endif

        default:
            // Fall back to MCP2515 for unknown platforms
#if defined(SVARV_PLATFORM_AVR) || defined(SVARV_PLATFORM_GENERIC)
            return new SvarvCANInterface_MCP2515();
#else
            return nullptr;
#endif
    }
    
    return nullptr;
}

SvarvMotor& SvarvMotionControl::addMotor(uint8_t node_id) {
    if (node_id == 0 || node_id > 255) {
        debugPrint("ERROR: Invalid node ID: " + String(node_id));
        // Return a reference to a static dummy motor for error cases
        static SvarvMotor dummy_motor(0, this);
        return dummy_motor;
    }
    
    // Check if motor already exists
    auto it = motors_.find(node_id);
    if (it != motors_.end()) {
        debugPrint("WARNING: Motor with node ID " + String(node_id) + " already exists");
        
        // Check if the existing motor is still connected
        if (it->second->isConnected()) {
            debugPrint("Existing motor is still connected - returning existing instance");
            return *(it->second);
        } else {
            debugPrint("Existing motor is disconnected - replacing with new instance");
            delete it->second;
            motors_.erase(it);
        }
    }
    
    // Create new motor
    SvarvMotor* motor = new SvarvMotor(node_id, this);
    motors_[node_id] = motor;
    
    debugPrint("Added motor with node ID: " + String(node_id));
    
    // Request initial status (but don't assume it will respond immediately)
    motor->requestStatusUpdate();
    
    return *motor;
}

// Enhanced removeMotor with better cleanup
bool SvarvMotionControl::removeMotor(uint8_t node_id) {
    auto it = motors_.find(node_id);
    if (it != motors_.end()) {
        debugPrint("Removing motor with node ID: " + String(node_id));
        
        // Safely stop the motor before removal
        if (it->second->isConnected()) {
            it->second->emergencyStop();
            delay(100); // Allow time for stop command
        }
        
        delete it->second;
        motors_.erase(it);
        return true;
    }
    
    debugPrint("WARNING: Attempted to remove non-existent motor ID: " + String(node_id));
    return false;
}

// Helper function to check for node ID conflicts
bool SvarvMotionControl::isNodeIdAvailable(uint8_t node_id) {
    if (node_id == 0 || node_id > 255) {
        return false;
    }
    
    auto it = motors_.find(node_id);
    if (it != motors_.end() && it->second->isConnected()) {
        return false; // ID is in use by connected motor
    }
    
    return true;
}

// Safe bulk configuration function
int SvarvMotionControl::configureMotorRange(uint8_t start_id, uint8_t end_id, bool force_reconfigure) {
    int configured_count = 0;
    
    if (start_id == 0 || end_id > 255 || start_id > end_id) {
        debugPrint("ERROR: Invalid ID range");
        return 0;
    }
    
    for (uint8_t id = start_id; id <= end_id; id++) {
        if (!force_reconfigure && !isNodeIdAvailable(id)) {
            debugPrint("Skipping ID " + String(id) + " - already in use");
            continue;
        }
        
        if (force_reconfigure) {
            removeMotor(id); // Force removal if requested
        }
        
        // Attempt to configure unconfigured device to this ID
        if (sendCANMessage(0, CAN_FUNCTION_SYSTEM, CMD_SYS_SET_NODE_ID, &id, 1)) {
            delay(1000);
            
            SvarvMotor& motor = addMotor(id);
            motor.requestStatusUpdate();
            
            // Brief verification period
            for (int i = 0; i < 5; i++) {
                processIncomingMessages();
                delay(100);
            }
            
            if (motor.isConnected()) {
                configured_count++;
                debugPrint("Successfully configured motor ID " + String(id));
            } else {
                removeMotor(id);
            }
        }
    }
    
    return configured_count;
}

SvarvMotor* SvarvMotionControl::getMotor(uint8_t node_id) {
#ifdef ARDUINO_ARCH_AVR
    return motors_.find(node_id) ? &motors_[node_id] : nullptr;
#else
    auto it = motors_.find(node_id);
    return (it != motors_.end()) ? it->second : nullptr;
#endif
}

std::vector<SvarvMotor*> SvarvMotionControl::getAllMotors() {
    std::vector<SvarvMotor*> result;
#ifdef ARDUINO_ARCH_AVR
    for (int i = 0; i < SVARV_MAX_MOTORS; i++) {
        if (motors_.data[i].used) {
            result.push_back(motors_.data[i].value);
        }
    }
#else
    for (auto& pair : motors_) {
        result.push_back(pair.second);
    }
#endif
    return result;
}

void SvarvMotionControl::update() {
    if (!initialized_ || !can_interface_) {
        return;
    }
    
    unsigned long current_time = millis();
    
    // Process incoming CAN messages
    processIncomingMessages();
    
    // Update motor timeouts
    updateMotorTimeouts();
    
    // Request status updates periodically
    if (current_time - last_update_time_ > STATUS_REQUEST_INTERVAL_MS) {
        auto motors = getAllMotors();
        for (auto* motor : motors) {
            if (motor && motor->isConnected()) {
                motor->requestStatusUpdate();
            }
        }
        last_update_time_ = current_time;
    }
}

int SvarvMotionControl::emergencyStopAll() {
    int count = 0;
    auto motors = getAllMotors();
    for (auto* motor : motors) {
        if (motor && motor->emergencyStop()) {
            count++;
        }
    }
    debugPrint("Emergency stopped " + String(count) + " motors");
    return count;
}

int SvarvMotionControl::enableAll() {
    int count = 0;
    auto motors = getAllMotors();
    for (auto* motor : motors) {
        if (motor && motor->enable()) {
            count++;
        }
    }
    debugPrint("Enabled " + String(count) + " motors");
    return count;
}

int SvarvMotionControl::disableAll() {
    int count = 0;
    auto motors = getAllMotors();
    for (auto* motor : motors) {
        if (motor && motor->disable()) {
            count++;
        }
    }
    debugPrint("Disabled " + String(count) + " motors");
    return count;
}

/**
 * @brief Configure a single unconfigured motor's node ID
 *
 * This function will attempt to configure ONE unconfigured motor (node ID = 0) 
 * to the specified target_id. 
 *
 * IMPORTANT: Only connect ONE unconfigured motor to the CAN bus when using this function.
 * If multiple unconfigured motors are connected, they will all receive the same node ID,
 * causing conflicts.
 *
 * @param target_id Desired node ID for the single unconfigured motor (1-255)
 * @return true if motor was successfully configured and responds, false otherwise
 */
bool SvarvMotionControl::configureSingleMotor(uint8_t target_id) {
    if (target_id == 0 || target_id > 255) {
        debugPrint("ERROR: Invalid target node ID: " + String(target_id));
        return false;
    }
    
    // Check if target ID is already in use
    if (!isNodeIdAvailable(target_id)) {
        debugPrint("ERROR: Node ID " + String(target_id) + " is already in use");
        return false;
    }
    
    debugPrint("Configuring single unconfigured motor to node ID " + String(target_id));
    debugPrint("WARNING: Ensure only ONE unconfigured motor is connected to CAN bus!");
    
    // Send node ID configuration to unconfigured device (node 0)
    if (!sendCANMessage(0, CAN_FUNCTION_SYSTEM, CMD_SYS_SET_NODE_ID, &target_id, 1)) {
        debugPrint("ERROR: Failed to send node ID configuration command");
        return false;
    }
    
    // Wait for device to restart and apply new configuration
    delay(2000);
    
    // Remove any existing motor object for this ID to prevent conflicts
    removeMotor(target_id);
    
    // Try to communicate with the newly configured motor
    SvarvMotor& motor = addMotor(target_id);
    
    // Request status to verify the motor is responding
    if (!motor.requestStatusUpdate()) {
        debugPrint("ERROR: Failed to send status request to newly configured motor");
        removeMotor(target_id);
        return false;
    }
    
    // Wait for response and process messages
    unsigned long start_time = millis();
    while (millis() - start_time < 3000) { // 3 second timeout
        processIncomingMessages();
        
        if (motor.isConnected()) {
            debugPrint("SUCCESS: Motor configured and responding with node ID " + String(target_id));
            return true;
        }
        
        delay(50);
    }
    
    debugPrint("ERROR: Newly configured motor not responding");
    removeMotor(target_id);
    return false;
}

// Also add a helper function to safely scan for unconfigured devices
std::vector<uint8_t> SvarvMotionControl::scanForUnconfiguredMotors(unsigned long timeout_ms) {
    std::vector<uint8_t> unconfigured_responses;
    unsigned long start_time = millis();
    
    debugPrint("Scanning for unconfigured motors (node ID = 0)...");
    
    // Send status query to node 0 (unconfigured devices)
    sendCANMessage(0, CAN_FUNCTION_QUERY, CMD_QUERY_STATUS);
    
    // Also send version request to see if any unconfigured devices respond
    sendCANMessage(0, CAN_FUNCTION_SYSTEM, CMD_SYS_GET_VERSION);
    
    // Listen for responses
    while (millis() - start_time < timeout_ms) {
        processIncomingMessages();
        delay(10);
    }
    
    // Count how many unconfigured devices responded
    // This is tricky because unconfigured devices all have node ID 0
    // We can infer from the responses we received
    
    debugPrint("Unconfigured device scan complete");
    return unconfigured_responses; // May need more sophisticated logic here
}

// Enhanced motor scanning that respects existing configurations
std::vector<uint8_t> SvarvMotionControl::scanForMotors(unsigned long timeout_ms) {
    std::vector<uint8_t> discovered_nodes;
    unsigned long start_time = millis();
    
    debugPrint("Scanning for motors (timeout: " + String(timeout_ms) + "ms)...");
    
    // First scan existing motors in our list
    auto existing_motors = getAllMotors();
    for (auto* motor : existing_motors) {
        if (motor) {
            debugPrint("Checking existing motor ID " + String(motor->getNodeId()));
            motor->requestStatusUpdate();
        }
    }
    
    // Send status queries to potential node IDs (skip 0 for now)
    for (uint8_t node_id = 1; node_id <= 20; node_id++) {
        // Skip if we already have this motor
        if (getMotor(node_id) != nullptr) {
            continue;
        }
        
        sendCANMessage(node_id, CAN_FUNCTION_QUERY, CMD_QUERY_STATUS);
        delay(10); // Small delay between queries
    }
    
    // Listen for responses
    while (millis() - start_time < timeout_ms) {
        processIncomingMessages();
        delay(10);
    }
    
    // Check which motors responded
    auto all_motors = getAllMotors();
    for (auto* motor : all_motors) {
        if (motor && motor->isConnected()) {
            discovered_nodes.push_back(motor->getNodeId());
        }
    }
    
    debugPrint("Scan complete. Found " + String(discovered_nodes.size()) + " motors");
    return discovered_nodes;
}


int SvarvMotionControl::getConnectedMotorCount() const {
    int count = 0;
    auto motors = const_cast<SvarvMotionControl*>(this)->getAllMotors();
    for (auto* motor : motors) {
        if (motor && motor->isConnected()) {
            count++;
        }
    }
    return count;
}

void SvarvMotionControl::getCANStatistics(uint32_t& messages_sent, uint32_t& messages_received, uint32_t& errors) const {
    messages_sent = messages_sent_;
    messages_received = messages_received_;
    errors = can_errors_;
}

bool SvarvMotionControl::isCANHealthy() const {
    if (!can_interface_ || !can_interface_->isAvailable()) {
        return false;
    }
    
    // Consider CAN healthy if error rate is low
    uint32_t total_messages = messages_sent_ + messages_received_;
    if (total_messages == 0) return true;
    
    float error_rate = static_cast<float>(can_errors_) / total_messages;
    return error_rate < 0.05f; // Less than 5% error rate
}

String SvarvMotionControl::getPlatformInfo() const {
    if (can_interface_) {
        return can_interface_->getPlatformName();
    }
    return "No CAN Interface";
}

bool SvarvMotionControl::sendCANMessage(uint8_t node_id, uint8_t function_id, uint8_t cmd, 
                                       const uint8_t* data, uint8_t len) {
    if (!initialized_ || !can_interface_) {
        return false;
    }
    
    // Check frame size limits
    if (len + 1 > 8) {
        debugPrint("ERROR: CAN frame too large (" + String(len + 1) + " bytes)");
        return false;
    }
    
    SvarvCANMessage msg;
    msg.id = (function_id << 8) | node_id;
    msg.extended = false;
    msg.rtr = false;
    msg.length = len + 1;
    msg.data[0] = cmd;
    
    if (data && len > 0) {
        for (int i = 0; i < len && i < 7; i++) {
            msg.data[i + 1] = data[i];
        }
    }
    
    bool success = can_interface_->sendMessage(msg);
    
    if (success) {
        messages_sent_++;
    } else {
        can_errors_++;
        debugPrint("ERROR: Failed to send CAN message to node " + String(node_id));
    }
    
    return success;
}

void SvarvMotionControl::processIncomingMessages() {
    SvarvCANMessage msg;
    
    while (can_interface_ && can_interface_->receiveMessage(msg)) {
        messages_received_++;
        
        uint32_t msgID = msg.id;
        uint8_t function_id = (msgID >> 8) & 0x07;
        uint8_t source_node = msgID & 0xFF;
        
        // Route message based on function type
        switch (function_id) {
            case CAN_FUNCTION_RESPONSE:
                handleResponseMessage(source_node, msg);
                break;
                
            case CAN_FUNCTION_TELEMETRY:
                handleTelemetryMessage(source_node, msg);
                break;
                
            case CAN_FUNCTION_ERROR:
                handleErrorMessage(source_node, msg);
                break;
                
            default:
                // Ignore other message types
                break;
        }
    }
}

void SvarvMotionControl::handleResponseMessage(uint8_t node_id, const SvarvCANMessage& msg) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor) {
        // Auto-create motor if we receive a response from unknown node
        motor = &addMotor(node_id);
    }
    
    if (msg.length >= 1) {
        uint8_t cmd = msg.data[0];
        
        if (cmd == CMD_QUERY_STATUS && msg.length >= 7) {
            // Parse status response
            SvarvMotorStatus new_status = motor->getStatus();
            
            uint8_t msg_num = msg.data[1];
            if (msg_num == 1) {
                // First status message
                new_status.control_mode = static_cast<SvarvControlMode>(msg.data[2]);
                new_status.enabled = msg.data[3] != 0;
                new_status.calibrated = msg.data[4] != 0;
                new_status.initializing = msg.data[5] != 0;
                new_status.error_code = static_cast<SvarvErrorCode>(msg.data[6]);
                if (msg.length >= 8) {
                    new_status.error_count = msg.data[7];
                }
                
                motor->updateStatus(new_status);
            } else if (msg_num == 2 && msg.length >= 8) {
                // Second status message with position and velocity
                memcpy(&new_status.position, &msg.data[2], 4);
                int16_t scaled_velocity;
                memcpy(&scaled_velocity, &msg.data[6], 2);
                new_status.velocity = scaled_velocity / 100.0f;
                
                motor->updateStatus(new_status);
            }
        }
    }
}

void SvarvMotionControl::handleTelemetryMessage(uint8_t node_id, const SvarvCANMessage& msg) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor || msg.length < 1) {
        return;
    }
    
    uint8_t telemetry_type = msg.data[0];
    SvarvMotorStatus new_status = motor->getStatus();
    
    switch (telemetry_type) {
        case TELEMETRY_HEARTBEAT:
            if (msg.length >= 3) {
                new_status.control_mode = static_cast<SvarvControlMode>(msg.data[1]);
                new_status.error_code = static_cast<SvarvErrorCode>(msg.data[2]);
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_MOTOR_STATE:
            if (msg.length >= 8) {
                memcpy(&new_status.position, &msg.data[1], 4);
                int16_t scaled_velocity;
                memcpy(&scaled_velocity, &msg.data[5], 2);
                new_status.velocity = scaled_velocity / 100.0f;
                
                uint8_t flags = msg.data[7];
                new_status.calibrated = (flags & 0x01) != 0;
                new_status.enabled = (flags & 0x02) != 0;
                
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_MOTOR_POWER:
            if (msg.length >= 7) {
                int16_t scaled_current;
                memcpy(&scaled_current, &msg.data[1], 2);
                new_status.current_q = scaled_current / 100.0f;
                
                uint16_t scaled_voltage;
                memcpy(&scaled_voltage, &msg.data[3], 2);
                new_status.voltage_q = scaled_voltage / 100.0f;
                
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_DIAGNOSTICS:
            if (msg.length >= 5) {
                memcpy(&new_status.loop_time_us, &msg.data[1], 2);
                memcpy(&new_status.error_count, &msg.data[3], 2);
                motor->updateStatus(new_status);
            }
            break;
    }
}

void SvarvMotionControl::handleErrorMessage(uint8_t node_id, const SvarvCANMessage& msg) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor || msg.length < 1) {
        return;
    }
    
    SvarvErrorCode error_code = static_cast<SvarvErrorCode>(msg.data[0]);
    
    SvarvMotorStatus new_status = motor->getStatus();
    new_status.error_code = error_code;
    if (msg.length >= 3) {
        new_status.error_count = (msg.data[1] << 8) | msg.data[2];
    }
    
    motor->updateStatus(new_status);
    
    // Trigger global error callback
    if (global_error_callback_) {
        global_error_callback_(*motor, error_code, svarvErrorToString(error_code));
    }
}

void SvarvMotionControl::updateMotorTimeouts() {
    auto motors = getAllMotors();
    for (auto* motor : motors) {
        if (motor) {
            motor->handleTimeout();
        }
    }
}

void SvarvMotionControl::debugPrint(const String& message) {
    if (debug_enabled_) {
        Serial.println("[Svarv] " + message);
    }
}