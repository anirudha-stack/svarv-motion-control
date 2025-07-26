/**
 * @file SvarvMotionControl.cpp
 * @brief Svarv Motion Control Library Implementation
 * @version 1.0.0
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
    : initialized_(false), debug_enabled_(false), messages_sent_(0), 
      messages_received_(0), can_errors_(0), last_update_time_(0), last_scan_time_(0) {
}

SvarvMotionControl::~SvarvMotionControl() {
    // Clean up motors
    for (auto& pair : motors_) {
        delete pair.second;
    }
    motors_.clear();
}

bool SvarvMotionControl::begin(uint32_t can_speed, int tx_pin, int rx_pin) {
    debugPrint("Svarv Motion Control v" + String(SVARV_VERSION) + " Initializing...");
    
    // Configure CAN pins and settings
    ESP32Can.setPins(tx_pin, rx_pin);
    ESP32Can.setRxQueueSize(32);
    ESP32Can.setTxQueueSize(16);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(can_speed));
    
    // Initialize CAN bus
    if (!ESP32Can.begin()) {
        debugPrint("ERROR: CAN bus initialization failed!");
        return false;
    }
    
    debugPrint("CAN bus initialized @ " + String(can_speed) + " bps");
    debugPrint("TX Pin: " + String(tx_pin) + ", RX Pin: " + String(rx_pin));
    
    initialized_ = true;
    last_update_time_ = millis();
    
    debugPrint("Svarv Motion Control ready!");
    return true;
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
        debugPrint("Motor with node ID " + String(node_id) + " already exists");
        return *(it->second);
    }
    
    // Create new motor
    SvarvMotor* motor = new SvarvMotor(node_id, this);
    motors_[node_id] = motor;
    
    debugPrint("Added motor with node ID: " + String(node_id));
    
    // Request initial status
    motor->requestStatusUpdate();
    
    return *motor;
}

bool SvarvMotionControl::removeMotor(uint8_t node_id) {
    auto it = motors_.find(node_id);
    if (it != motors_.end()) {
        delete it->second;
        motors_.erase(it);
        debugPrint("Removed motor with node ID: " + String(node_id));
        return true;
    }
    return false;
}

SvarvMotor* SvarvMotionControl::getMotor(uint8_t node_id) {
    auto it = motors_.find(node_id);
    return (it != motors_.end()) ? it->second : nullptr;
}

std::vector<SvarvMotor*> SvarvMotionControl::getAllMotors() {
    std::vector<SvarvMotor*> result;
    for (auto& pair : motors_) {
        result.push_back(pair.second);
    }
    return result;
}

void SvarvMotionControl::update() {
    if (!initialized_) {
        return;
    }
    
    unsigned long current_time = millis();
    
    // Process incoming CAN messages
    processIncomingMessages();
    
    // Update motor timeouts
    updateMotorTimeouts();
    
    // Request status updates periodically
    if (current_time - last_update_time_ > STATUS_REQUEST_INTERVAL_MS) {
        for (auto& pair : motors_) {
            if (pair.second->isConnected()) {
                pair.second->requestStatusUpdate();
            }
        }
        last_update_time_ = current_time;
    }
}

int SvarvMotionControl::emergencyStopAll() {
    int count = 0;
    for (auto& pair : motors_) {
        if (pair.second->emergencyStop()) {
            count++;
        }
    }
    debugPrint("Emergency stopped " + String(count) + " motors");
    return count;
}

int SvarvMotionControl::enableAll() {
    int count = 0;
    for (auto& pair : motors_) {
        if (pair.second->enable()) {
            count++;
        }
    }
    debugPrint("Enabled " + String(count) + " motors");
    return count;
}

int SvarvMotionControl::disableAll() {
    int count = 0;
    for (auto& pair : motors_) {
        if (pair.second->disable()) {
            count++;
        }
    }
    debugPrint("Disabled " + String(count) + " motors");
    return count;
}

std::vector<uint8_t> SvarvMotionControl::scanForMotors(unsigned long timeout_ms) {
    std::vector<uint8_t> discovered_nodes;
    unsigned long start_time = millis();
    
    debugPrint("Scanning for motors (timeout: " + String(timeout_ms) + "ms)...");
    
    // Send status queries to potential node IDs
    for (uint8_t node_id = 1; node_id <= 20; node_id++) {
        sendCANMessage(node_id, CAN_FUNCTION_QUERY, CMD_QUERY_STATUS);
        delay(10); // Small delay between queries
    }
    
    // Listen for responses
    while (millis() - start_time < timeout_ms) {
        processIncomingMessages();
        delay(10);
    }
    
    // Check which motors responded
    for (auto& pair : motors_) {
        if (pair.second->isConnected()) {
            discovered_nodes.push_back(pair.first);
        }
    }
    
    debugPrint("Scan complete. Found " + String(discovered_nodes.size()) + " motors");
    return discovered_nodes;
}

int SvarvMotionControl::autoConfigureMotors(uint8_t start_id, uint8_t max_motors) {
    debugPrint("Auto-configuring motors starting from ID " + String(start_id));
    
    int configured_count = 0;
    uint8_t current_id = start_id;
    
    // Look for unconfigured motors (node ID = 0)
    for (int attempt = 0; attempt < max_motors && configured_count < max_motors; attempt++) {
        // Send node ID setup command to unconfigured device
        if (sendCANMessage(0, CAN_FUNCTION_SYSTEM, CMD_SYS_SET_NODE_ID, &current_id, 1)) {
            delay(500); // Wait for configuration to complete
            
            // Try to communicate with the newly configured motor
            if (sendCANMessage(current_id, CAN_FUNCTION_QUERY, CMD_QUERY_STATUS)) {
                debugPrint("Configured motor with node ID: " + String(current_id));
                configured_count++;
                current_id++;
            }
        }
        
        delay(100); // Delay between attempts
    }
    
    debugPrint("Auto-configuration complete. Configured " + String(configured_count) + " motors");
    return configured_count;
}

int SvarvMotionControl::getConnectedMotorCount() const {
    int count = 0;
    for (const auto& pair : motors_) {
        if (pair.second->isConnected()) {
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
    // Consider CAN healthy if error rate is low
    uint32_t total_messages = messages_sent_ + messages_received_;
    if (total_messages == 0) return true;
    
    float error_rate = static_cast<float>(can_errors_) / total_messages;
    return error_rate < 0.05f; // Less than 5% error rate
}

bool SvarvMotionControl::sendCANMessage(uint8_t node_id, uint8_t function_id, uint8_t cmd, 
                                       const uint8_t* data, uint8_t len) {
    if (!initialized_) {
        return false;
    }
    
    // Check frame size limits
    if (len + 1 > 8) {
        debugPrint("ERROR: CAN frame too large (" + String(len + 1) + " bytes)");
        return false;
    }
    
    CanFrame frame = {0};
    frame.identifier = (function_id << 8) | node_id;
    frame.extd = 0;
    frame.data_length_code = len + 1;
    frame.data[0] = cmd;
    
    if (data && len > 0) {
        memcpy(&frame.data[1], data, len);
    }
    
    bool success = ESP32Can.writeFrame(frame);
    
    if (success) {
        messages_sent_++;
    } else {
        can_errors_++;
        debugPrint("ERROR: Failed to send CAN message to node " + String(node_id));
    }
    
    return success;
}

void SvarvMotionControl::processIncomingMessages() {
    CanFrame rxFrame;
    
    while (ESP32Can.readFrame(rxFrame, 1)) {
        messages_received_++;
        
        uint32_t msgID = rxFrame.identifier;
        uint8_t function_id = (msgID >> 8) & 0x07;
        uint8_t source_node = msgID & 0xFF;
        
        // Route message based on function type
        switch (function_id) {
            case CAN_FUNCTION_RESPONSE:
                handleResponseMessage(source_node, rxFrame);
                break;
                
            case CAN_FUNCTION_TELEMETRY:
                handleTelemetryMessage(source_node, rxFrame);
                break;
                
            case CAN_FUNCTION_ERROR:
                handleErrorMessage(source_node, rxFrame);
                break;
                
            default:
                // Ignore other message types
                break;
        }
    }
}

void SvarvMotionControl::handleResponseMessage(uint8_t node_id, const CanFrame& frame) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor) {
        // Auto-create motor if we receive a response from unknown node
        motor = &addMotor(node_id);
    }
    
    if (frame.data_length_code >= 1) {
        uint8_t cmd = frame.data[0];
        
        if (cmd == CMD_QUERY_STATUS && frame.data_length_code >= 7) {
            // Parse status response
            SvarvMotorStatus new_status = motor->getStatus();
            
            uint8_t msg_num = frame.data[1];
            if (msg_num == 1) {
                // First status message
                new_status.control_mode = static_cast<SvarvControlMode>(frame.data[2]);
                new_status.enabled = frame.data[3] != 0;
                new_status.calibrated = frame.data[4] != 0;
                new_status.initializing = frame.data[5] != 0;
                new_status.error_code = static_cast<SvarvErrorCode>(frame.data[6]);
                new_status.error_count = frame.data[7];
                
                motor->updateStatus(new_status);
            } else if (msg_num == 2 && frame.data_length_code >= 8) {
                // Second status message with position and velocity
                memcpy(&new_status.position, &frame.data[2], 4);
                int16_t scaled_velocity;
                memcpy(&scaled_velocity, &frame.data[6], 2);
                new_status.velocity = scaled_velocity / 100.0f;
                
                motor->updateStatus(new_status);
            }
        }
    }
}

void SvarvMotionControl::handleTelemetryMessage(uint8_t node_id, const CanFrame& frame) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor || frame.data_length_code < 1) {
        return;
    }
    
    uint8_t telemetry_type = frame.data[0];
    SvarvMotorStatus new_status = motor->getStatus();
    
    switch (telemetry_type) {
        case TELEMETRY_HEARTBEAT:
            if (frame.data_length_code >= 3) {
                new_status.control_mode = static_cast<SvarvControlMode>(frame.data[1]);
                new_status.error_code = static_cast<SvarvErrorCode>(frame.data[2]);
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_MOTOR_STATE:
            if (frame.data_length_code >= 8) {
                memcpy(&new_status.position, &frame.data[1], 4);
                int16_t scaled_velocity;
                memcpy(&scaled_velocity, &frame.data[5], 2);
                new_status.velocity = scaled_velocity / 100.0f;
                
                uint8_t flags = frame.data[7];
                new_status.calibrated = (flags & 0x01) != 0;
                new_status.enabled = (flags & 0x02) != 0;
                
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_MOTOR_POWER:
            if (frame.data_length_code >= 7) {
                int16_t scaled_current;
                memcpy(&scaled_current, &frame.data[1], 2);
                new_status.current_q = scaled_current / 100.0f;
                
                uint16_t scaled_voltage;
                memcpy(&scaled_voltage, &frame.data[3], 2);
                new_status.voltage_q = scaled_voltage / 100.0f;
                
                motor->updateStatus(new_status);
            }
            break;
            
        case TELEMETRY_DIAGNOSTICS:
            if (frame.data_length_code >= 5) {
                memcpy(&new_status.loop_time_us, &frame.data[1], 2);
                memcpy(&new_status.error_count, &frame.data[3], 2);
                motor->updateStatus(new_status);
            }
            break;
    }
}

void SvarvMotionControl::handleErrorMessage(uint8_t node_id, const CanFrame& frame) {
    SvarvMotor* motor = getMotor(node_id);
    if (!motor || frame.data_length_code < 1) {
        return;
    }
    
    SvarvErrorCode error_code = static_cast<SvarvErrorCode>(frame.data[0]);
    
    SvarvMotorStatus new_status = motor->getStatus();
    new_status.error_code = error_code;
    if (frame.data_length_code >= 3) {
        new_status.error_count = (frame.data[1] << 8) | frame.data[2];
    }
    
    motor->updateStatus(new_status);
    
    // Trigger global error callback
    if (global_error_callback_) {
        global_error_callback_(*motor, error_code, svarvErrorToString(error_code));
    }
}

void SvarvMotionControl::updateMotorTimeouts() {
    for (auto& pair : motors_) {
        pair.second->handleTimeout();
    }
}

void SvarvMotionControl::debugPrint(const String& message) {
    if (debug_enabled_) {
        Serial.println("[Svarv] " + message);
    }
}