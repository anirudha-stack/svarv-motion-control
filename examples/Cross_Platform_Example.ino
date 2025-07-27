/**
 * @file Cross_Platform_Example.ino
 * @brief Cross-platform motor control example using Svarv Motion Control Library
 * @author Svarv Robotics
 * 
 * This example demonstrates how to use the Svarv Motion Control library across
 * different hardware platforms with automatic platform detection and configuration.
 * 
 * Supported Platforms:
 * - ESP32 (ESP32-C3, ESP32-S3, ESP32 Classic) - Built-in CAN
 * - STM32 (F1, F4, F7, H7 series) - Built-in CAN
 * - Arduino AVR (Uno, Mega, Nano) + MCP2515 CAN shield
 * - Other platforms with MCP2515 support
 * 
 * Hardware Connections:
 * 
 * ESP32 (Built-in CAN):
 * - CAN TX: GPIO 21 (default)
 * - CAN RX: GPIO 20 (default)
 * - 120Ω termination resistors on CAN bus
 * 
 * STM32 (Built-in CAN):
 * - Uses built-in CAN peripheral (CAN1)
 * - Pin configuration handled automatically
 * - 120Ω termination resistors on CAN bus
 * 
 * Arduino + MCP2515:
 * - CS (Chip Select): Pin 10 (default)
 * - INT (Interrupt): Pin 2 (default)
 * - SCK, MOSI, MISO: Standard SPI pins
 * - 120Ω termination resistors on CAN bus
 * 
 * @section BEHAVIOR
 * 
 * This example will:
 * 1. Auto-detect the hardware platform
 * 2. Initialize CAN with platform-appropriate settings
 * 3. Scan for and connect to available motors
 * 4. Demonstrate basic motor control operations
 * 5. Show platform-specific information
 */

#include "SvarvMotionControl.h"

// Create motion control instance
SvarvMotionControl svarv;
SvarvMotor motor1;

// Control variables
unsigned long lastMoveTime = 0;
int moveStep = 0;
bool systemInitialized = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10); // Wait up to 5 seconds for Serial
  
  Serial.println("===========================================");
  Serial.println("Svarv Motion Control - Cross-Platform Demo");
  Serial.println("===========================================");
  Serial.println();
  
  // Display library and platform information
  Serial.println("Library Version: " + SvarvMotionControl::getVersion());
  Serial.println();
  
  // Enable debug output
  svarv.enableDebug(true);
  
  Serial.println("Detecting platform and initializing CAN...");
  
  // Initialize with auto-platform detection
  bool canInitialized = false;
  
#if defined(ESP32)
  // ESP32 - Try built-in CAN with default pins
  Serial.println("Platform: ESP32 detected");
  Serial.println("Attempting ESP32 built-in CAN initialization...");
  canInitialized = svarv.begin(1000000); // 1 Mbps, default pins (21, 20)
  
  if (!canInitialized) {
    // Try with alternative pin configuration
    Serial.println("Retrying with alternative pins (GPIO 4, 5)...");
    canInitialized = svarv.begin(1000000, 4, 5);
  }
  
#elif defined(STM32_CORE_VERSION) || defined(ARDUINO_ARCH_STM32)
  // STM32 - Try built-in CAN
  Serial.println("Platform: STM32 detected");
  Serial.println("Attempting STM32 built-in CAN initialization...");
  canInitialized = svarv.begin(1000000); // Uses CAN1 by default
  
#else
  // Arduino AVR or other platform - Use MCP2515
  Serial.println("Platform: Arduino/Generic detected");
  Serial.println("Attempting MCP2515 CAN initialization...");
  
  // Try different common pin configurations
  int csPins[] = {10, 9, 8,