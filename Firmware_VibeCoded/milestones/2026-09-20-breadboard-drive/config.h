/**
 * @file config.h
 * @brief Rainbow Six: Siege Recon Drone Configuration
 * @author Peter
 * @version 1.0
 * 
 * Hardware configuration for ESP32-S3 CAM N16R8 (GOOUUU v1.5)
 * Features: Self-balancing camera drone with web interface
 * Memory: 16MB Flash, 8MB PSRAM
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================
// HARDWARE BOARD CONFIGURATION
// =====================================
#define BOARD_MODEL "ESP32-S3 CAM N16R8"
#define BOARD_VERSION "1.5"
#define BOARD_MANUFACTURER "GOOUUU"
#define FLASH_SIZE_MB 16
#define PSRAM_SIZE_MB 8

// =====================================
// FREERTOS TASK CONFIGURATION
// =====================================
#define CORE_0_TASKS 0  // High-speed control loop
#define CORE_1_TASKS 1  // Web server and peripherals

// Task priorities (higher number = higher priority)
#define PRIORITY_CRITICAL 3  // IMU + PID + Motor control
#define PRIORITY_HIGH 2      // Camera streaming
#define PRIORITY_MEDIUM 1    // Web server, LED animations
#define PRIORITY_LOW 0       // Background tasks

// Task timing
#define PID_LOOP_FREQUENCY_HZ 100
#define PID_LOOP_PERIOD_MS (1000 / PID_LOOP_FREQUENCY_HZ)
#define CAMERA_STREAM_FPS 30
#define LED_ANIMATION_UPDATE_MS LED_ANIMATION_SPEED
#define ENCODER_READ_INTERVAL_MS 10

// =====================================
// BMI270 IMU CONFIGURATION (I2C)
// =====================================
#define IMU_ENABLED true
#define IMU_SDA_PIN 1
#define IMU_SCL_PIN 2
#define IMU_I2C_FREQ 400000  // 400kHz
#define IMU_I2C_PORT 0
#define IMU_ADDRESS 0x68     // BMI270 default address
#define IMU_INT_PIN -1       // Not used for polling mode

// IMU calibration and filtering
#define GYRO_RANGE 2000      // ±2000°/s
#define ACCEL_RANGE 16       // ±16g
#define IMU_FILTER_ALPHA 0.98f  // Complementary filter coefficient
#define GYRO_DEADBAND 2.0f   // Degrees/second deadband

// =====================================
// DRV8833 MOTOR DRIVER CONFIGURATION
// =====================================
#define MOTORS_ENABLED true
// Camera owns GPIO 8-13 and 16-18. Do not reuse those for motors.
#define MOTOR_A_PIN1 41      // Motor A Forward  (DRV8833 AIN1)
#define MOTOR_A_PIN2 42      // Motor A Reverse  (DRV8833 AIN2)
#define MOTOR_B_PIN1 39      // Motor B Forward  (DRV8833 BIN1)
#define MOTOR_B_PIN2 40      // Motor B Reverse  (DRV8833 BIN2)

// Motor PWM settings
#define MOTOR_PWM_FREQ 5000     // 5kHz — 20kHz can trip cheap DRV8833 modules
#define MOTOR_PWM_RESOLUTION 8  // 8-bit resolution (0-255)
// Camera XCLK uses LEDC 0 (timer 0). Spotlight uses LEDC 7 (timer 3).
// Channels 0/1 share timer 0, 6/7 share timer 3 — motors must avoid those.
#define MOTOR_PWM_CHANNEL_A1 2
#define MOTOR_PWM_CHANNEL_A2 3
#define MOTOR_PWM_CHANNEL_B1 4
#define MOTOR_PWM_CHANNEL_B2 5

// Motor control limits
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 0
#define MOTOR_DEADBAND 10    // Minimum PWM to overcome friction

// =====================================
// ENCODER CONFIGURATION
// =====================================
#define ENCODERS_ENABLED true
#define ENCODER_LEFT_A_PIN 19   // Left encoder A  (USB_D+ silkscreen; TTL USB only)
#define ENCODER_LEFT_B_PIN 20   // Left encoder B  (USB_D- silkscreen; TTL USB only)
#define ENCODER_RIGHT_A_PIN 21  // Right encoder A
#define ENCODER_RIGHT_B_PIN 47  // Right encoder B

// Encoder specifications
#define ENCODER_PPR 20          // Pulses per revolution
#define WHEEL_DIAMETER_MM 65    // Wheel diameter in millimeters
#define ENCODER_FILTER_SAMPLES 3 // Moving average filter

// =====================================
// PID CONTROLLER CONFIGURATION
// =====================================
#define PID_ENABLED true

// Balance PID (pitch control)
#define PID_BALANCE_KP 30.0f
#define PID_BALANCE_KI 0.5f
#define PID_BALANCE_KD 0.8f
#define PID_BALANCE_SETPOINT 0.0f  // Target angle (degrees)

// Velocity PID (speed control)  
#define PID_VELOCITY_KP 2.0f
#define PID_VELOCITY_KI 0.1f
#define PID_VELOCITY_KD 0.05f

// Position PID (turning control)
#define PID_POSITION_KP 1.5f
#define PID_POSITION_KI 0.0f
#define PID_POSITION_KD 0.1f

// PID limits
#define PID_OUTPUT_LIMIT 200
#define PID_INTEGRAL_LIMIT 100
#define BALANCE_ANGLE_LIMIT 45.0f  // Degrees - safety cutoff

// =====================================
// OV3660 CAMERA CONFIGURATION (DVP)
// =====================================
#define CAMERA_ENABLED true
#define CAMERA_MODEL_OV3660

// DVP interface pins
#define CAM_PIN_SIOD 4    // SDA
#define CAM_PIN_SIOC 5    // SCL
#define CAM_PIN_VSYNC 6   // Vertical sync
#define CAM_PIN_HREF 7    // Horizontal reference
#define CAM_PIN_PCLK 13   // Pixel clock
#define CAM_PIN_XCLK 15   // External clock

// Data pins D0-D7
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9  
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

// Camera settings
#define CAMERA_FRAME_SIZE FRAMESIZE_HD    // 1280x720 default
#define CAMERA_PIXEL_FORMAT PIXFORMAT_JPEG
#define CAMERA_QUALITY 10    // JPEG quality (0-63, lower = better)
#define CAMERA_FB_COUNT 2    // Frame buffer count
#define CAMERA_XCLK_FREQ_HZ 20000000  // 20MHz

// =====================================
// MICROPHONE REMOVED
// =====================================
// INMP441 microphone permanently removed from project

// =====================================
// WS2812B LED STRIP CONFIGURATION
// =====================================
#define LED_STRIP_ENABLED true
#define LED_DATA_PIN 3
#define LED_COUNT 6          // Number of LEDs in strip (0-5)
#define LED_BRIGHTNESS 64    // Default brightness (0-255) = 25%
#define LED_TYPE NEO_GRB     // Color order

// LED animations
#define LED_ANIMATION_SPEED 30   // milliseconds between animation steps
#define LED_FADE_STEPS 20
#define LED_RAINBOW_SPEED 5

// Default colors
#define LED_COLOR_OFF 0x000000
#define LED_COLOR_RED 0xFF0000
#define LED_COLOR_GREEN 0x00FF00      // Default startup color
#define LED_COLOR_BLUE 0x0000FF
#define LED_COLOR_WHITE 0xFFFFFF
#define LED_COLOR_R6_ORANGE 0xFF8C00  // Rainbow Six orange theme

// =====================================
// SPOTLIGHT CONFIGURATION
// =====================================
#define SPOTLIGHT_ENABLED true
#define SPOTLIGHT_PIN 38     // Gate/Base driver pin
#define SPOTLIGHT_PWM_CHANNEL 7
#define SPOTLIGHT_PWM_FREQ 1000
#define SPOTLIGHT_PWM_RESOLUTION 8
#define SPOTLIGHT_MAX_BRIGHTNESS 255
#define SPOTLIGHT_DEFAULT_BRIGHTNESS 128

// =====================================
// WIFI CONFIGURATION
// =====================================
#define WIFI_ENABLED true
#define WIFI_CONNECTION_TIMEOUT_MS 10000
#define WIFI_RECONNECT_INTERVAL_MS 5000
#define WIFI_MAX_RECONNECT_ATTEMPTS 10

// Default credentials (can be changed via web interface)
#define DEFAULT_WIFI_SSID "R6_Drone_Network"
#define DEFAULT_WIFI_PASSWORD "Rainbow6Siege"

// Access Point mode (fallback)
#define AP_MODE_ENABLED true
#define AP_SSID "R6_Recon_Drone"
#define AP_PASSWORD "ReconDrone123"
#define AP_CHANNEL 6
#define AP_MAX_CONNECTIONS 4

// =====================================
// WEB SERVER CONFIGURATION
// =====================================
#define WEBSERVER_ENABLED true
#define WEBSERVER_PORT 80
#define WEBSOCKET_PORT 81
#define MAX_WEBSOCKET_CLIENTS 4
#define STREAM_PART_BOUNDARY "123456789000000000000987654321"

// Web interface settings
#define WEB_UPDATE_INTERVAL_MS 100
#define CONTROL_TIMEOUT_MS 2000  // Stop motors if no command received
#define DRIVE_HOLD_TIMEOUT_MS 800  // Stop if no drive request arrives (click or hold)
#define BUTTON_DEBOUNCE_MS 50

// =====================================
// CONTROL SYSTEM CONFIGURATION  
// =====================================
#define CONTROL_ENABLED true

// Movement speeds (0-255)
#define SPEED_FORWARD 150
#define SPEED_BACKWARD 120
#define SPEED_TURN_LEFT 100
#define SPEED_TURN_RIGHT 100
#define SPEED_STOP 0

// Control smoothing
#define ACCELERATION_RATE 5    // PWM units per update
#define DECELERATION_RATE 8    // PWM units per update
#define TURN_SENSITIVITY 0.7f

// Safety features
#define TILT_SAFETY_ANGLE 60.0f      // Degrees - emergency stop
#define LOW_BATTERY_VOLTAGE 3.2f     // Volts per cell
#define EMERGENCY_STOP_ENABLED true
#define WATCHDOG_TIMEOUT_MS 5000

// =====================================
// SYSTEM MONITORING
// =====================================
#define MONITORING_ENABLED true
#define BATTERY_MONITOR_PIN A0  // If available
#define SYSTEM_STATUS_UPDATE_MS 1000
#define TEMPERATURE_MONITOR_ENABLED false  // ESP32-S3 internal temp

// Debug and logging
#define DEBUG_ENABLED true
#define SERIAL_BAUD_RATE 115200
#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 3
#define CURRENT_LOG_LEVEL LOG_LEVEL_INFO

// Performance monitoring
#define PERFORMANCE_MONITORING true
#define TASK_MONITOR_INTERVAL_MS 5000
#define MEMORY_MONITOR_ENABLED true

// =====================================
// FEATURE FLAGS
// =====================================
#define FEATURE_AUDIO_STREAMING true
#define FEATURE_OTA_UPDATE true
#define FEATURE_TELEMETRY true
#define FEATURE_GESTURE_CONTROL false  // Future feature
#define FEATURE_AUTONOMOUS_MODE false  // Future feature
#define FEATURE_NIGHT_VISION false     // Future feature

// =====================================
// MEMORY CONFIGURATION
// =====================================
#define STACK_SIZE_CONTROL_TASK 4096
#define STACK_SIZE_CAMERA_TASK 8192
#define STACK_SIZE_WEB_TASK 6144
#define STACK_SIZE_LED_TASK 2048
#define STACK_SIZE_AUDIO_TASK 4096

// Buffer sizes
#define COMMAND_BUFFER_SIZE 256
#define RESPONSE_BUFFER_SIZE 512
#define SENSOR_DATA_BUFFER_SIZE 128

// =====================================
// VERSION INFORMATION
// =====================================
#define FIRMWARE_VERSION "1.1.0"
#define FIRMWARE_BUILD_DATE __DATE__ " " __TIME__
#define PROJECT_NAME "R6 Recon Drone"
#define AUTHOR "Peter"

// =====================================
// COMPILE-TIME VALIDATION
// =====================================
#if !defined(ESP32S3)
#error "This firmware is designed for ESP32-S3 only"
#endif

#if PSRAM_SIZE_MB < 8
#warning "Insufficient PSRAM for optimal camera performance"
#endif

#if FLASH_SIZE_MB < 16
#warning "Limited flash storage may affect OTA and web assets"
#endif

// =====================================
// HELPER MACROS
// =====================================
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define PIN_VALID(pin) ((pin) >= 0 && (pin) <= 48)
#define CONSTRAIN(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define MAP_VALUE(x, in_min, in_max, out_min, out_max) \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// Debug macros
#if DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(fmt, ...)
#endif

#endif // CONFIG_H