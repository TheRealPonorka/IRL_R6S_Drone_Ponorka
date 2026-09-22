/**
 * @file main_simple_new.cpp
 * @brief BMI270 Library Implementation for R6 Drone
 * @author Peter
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "esp_camera.h"
#include "driver/i2s.h"
#include "config.h"
#include "SparkFun_BMI270_Arduino_Library.h"

// Simple LED strip for testing
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);

// Web server
WebServer server(80);

// SparkFun BMI270 IMU
BMI270 imu;
bool imuInitialized = false;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
uint32_t lastIMUUpdate = 0;

// IMU calibration offsets
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
bool imuCalibrated = false;
bool calibrationInProgress = false;

// Other variables (keep your existing ones)
bool ledEnabled = false;
uint32_t lastLedUpdate = 0;
int ledAnimationStep = 0;
bool cameraInitialized = false;
bool microphoneInitialized = false;
bool microphoneEnabled = false;
bool spotlightEnabled = false;
int spotlightBrightness = 50;
float audioLevel = 0.0;
uint8_t ledR = 64, ledG = 64, ledB = 0;

bool initializeIMU() {
    Serial.println("=== SPARKFUN BMI270 LIBRARY INITIALIZATION ===");
    
    // Initialize I2C with correct pins
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);  // 400kHz I2C speed
    Serial.printf("I2C Config: SDA=GPIO%d, SCL=GPIO%d, Freq=400kHz\n", IMU_SDA_PIN, IMU_SCL_PIN);
    delay(100);
    
    // First, scan I2C bus for any devices
    Serial.println("🔍 Scanning I2C bus for devices...");
    bool deviceFound = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("   📍 I2C device found at address 0x%02X\n", addr);
            deviceFound = true;
        }
    }
    
    if (!deviceFound) {
        Serial.println("❌ No I2C devices found! Check wiring:");
        Serial.println("   - SDA wire to GPIO1");
        Serial.println("   - SCL wire to GPIO2"); 
        Serial.println("   - VCC to 3.3V (NOT 5V for BMI270!)");
        Serial.println("   - GND to GND");
        return false;
    }
    
    // Check specifically for BMI270 at common addresses
    Serial.println("🎯 Checking BMI270 specific addresses...");
    uint8_t bmi270_addresses[] = {0x68, 0x69, 0x6A, 0x6B};
    uint8_t detectedAddress = 0;
    
    for (int i = 0; i < 4; i++) {
        Wire.beginTransmission(bmi270_addresses[i]);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("   ✅ BMI270-compatible device found at 0x%02X\n", bmi270_addresses[i]);
            detectedAddress = bmi270_addresses[i];
            break;
        }
    }
    
    if (detectedAddress == 0) {
        Serial.println("❌ No BMI270 device detected at expected addresses");
        Serial.println("💡 If device shows at different address, check:");
        Serial.println("   - SDO/SA0 pin connection (affects address)");
        Serial.println("   - Correct BMI270 module (not BMI160/BMI088/etc)");
        return false;
    }
    
    Serial.println("🎯 Initializing BMI270 with SparkFun library...");
    Serial.println("   (This library handles config file upload automatically!)");
    
    // Initialize BMI270 using SparkFun library at detected address
    int8_t result = imu.beginI2C(detectedAddress);
    if (result != BMI2_OK) {
        Serial.printf("❌ SparkFun library initialization failed at 0x%02X (error: %d)\n", detectedAddress, result);
        Serial.println("💡 Possible causes:");
        Serial.println("   - Library version incompatibility");
        Serial.println("   - Hardware communication issue");
        Serial.println("   - BMI270 configuration file problem");
        return false;
    }
    
    Serial.printf("✅ BMI270 initialized successfully at address 0x%02X!\n", detectedAddress);
    
    Serial.println("✅ BMI270 initialized successfully with SparkFun library!");
    Serial.println("🎉 Config file uploaded automatically!");
    
    // Test reading sensor data immediately
    Serial.println("🔍 Testing sensor data...");
    result = imu.getSensorData();
    if (result != BMI2_OK) {
        Serial.printf("❌ Sensor data read failed (error: %d)\n", result);
        return false;
    }
    
    // Test accelerometer
    float accelXTest = imu.data.accelX;
    float accelYTest = imu.data.accelY;
    float accelZTest = imu.data.accelZ;
    
    Serial.printf("📐 Accelerometer test: %.3f, %.3f, %.3f g\n", accelXTest, accelYTest, accelZTest);
    
    // Test gyroscope
    float gyroXTest = imu.data.gyroX;
    float gyroYTest = imu.data.gyroY;
    float gyroZTest = imu.data.gyroZ;
    
    Serial.printf("🔄 Gyroscope test: %.3f, %.3f, %.3f °/s\n", gyroXTest, gyroYTest, gyroZTest);
    
    // Check if we're getting reasonable data
    float totalAccel = abs(accelXTest) + abs(accelYTest) + abs(accelZTest);
    float totalGyro = abs(gyroXTest) + abs(gyroYTest) + abs(gyroZTest);
    
    Serial.printf("📊 Data validation: Total accel=%.3f g, Total gyro=%.3f °/s\n", totalAccel, totalGyro);
    
    if (totalAccel > 0.1) {  // Should have at least gravity
        Serial.println("✅ ACCELEROMETER IS WORKING! Data detected!");
    } else {
        Serial.println("⚠️  Accelerometer returning zero data");
        Serial.println("💡 This may indicate:");
        Serial.println("   - Config file not loaded properly");
        Serial.println("   - Power supply issue");
        Serial.println("   - Hardware failure");
    }
    
    if (totalGyro < 100) {  // Reasonable range when stationary
        Serial.println("✅ GYROSCOPE IS WORKING! Data in reasonable range!");
    } else {
        Serial.println("⚠️  Gyroscope showing very high values (may indicate noise)");
    }
    
    imuInitialized = true;
    Serial.println("=== SPARKFUN BMI270 INITIALIZATION COMPLETE! ===");
    Serial.println("📐 Both accelerometer and gyroscope should now work!");
    
    return true;
}

void calibrateIMU() {
    if (!imuInitialized) {
        Serial.println("❌ Cannot calibrate - IMU not initialized");
        return;
    }
    
    Serial.println("🔧 Starting BMI270 gyroscope calibration...");
    Serial.println("📍 Keep the drone completely still during calibration!");
    
    calibrationInProgress = true;
    float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int samples = 0;
    uint32_t startTime = millis();
    
    // Collect 100 samples over 1 second
    while (samples < 100 && (millis() - startTime) < 2000) {
        if (imu.getSensorData() == BMI2_OK) {
            gyroXSum += imu.data.gyroX;
            gyroYSum += imu.data.gyroY;
            gyroZSum += imu.data.gyroZ;
            samples++;
            
            // Show progress
            if (samples % 20 == 0) {
                Serial.printf("📊 Calibration progress: %d/100 samples\n", samples);
            }
        }
        delay(10);
    }
    
    if (samples >= 50) {  // Need at least 50 good samples
        gyroOffsetX = gyroXSum / samples;
        gyroOffsetY = gyroYSum / samples;
        gyroOffsetZ = gyroZSum / samples;
        
        Serial.printf("✅ Calibration complete! Offsets: X=%.3f, Y=%.3f, Z=%.3f °/s\n", 
                     gyroOffsetX, gyroOffsetY, gyroOffsetZ);
        imuCalibrated = true;
    } else {
        Serial.printf("❌ Calibration failed - only got %d samples\n", samples);
    }
    
    calibrationInProgress = false;
}

void updateIMU() {
    if (!imuInitialized) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastIMUUpdate < 10) return; // 100Hz update rate for production
    lastIMUUpdate = currentTime;
    
    // Read sensor data using SparkFun library
    int8_t result = imu.getSensorData();
    if (result == BMI2_OK) {
        // Get accelerometer data directly from library
        accelX = imu.data.accelX;  // Already in g units
        accelY = imu.data.accelY;
        accelZ = imu.data.accelZ;
        
        // Get gyroscope data and apply calibration
        gyroX = imu.data.gyroX - gyroOffsetX;  // Already in °/s units
        gyroY = imu.data.gyroY - gyroOffsetY;
        gyroZ = imu.data.gyroZ - gyroOffsetZ;
        
        // Calculate pitch and roll from accelerometer
        float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
        float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
        
        // Apply complementary filter (98% gyro, 2% accel for smooth response)
        float dt = 0.01; // 10ms = 0.01s for 100Hz updates
        pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accelPitch;
        roll = 0.98 * (roll + gyroY * dt) + 0.02 * accelRoll;
        yaw += gyroZ * dt; // Integrate gyro for yaw
        
        // Debug output every 2 seconds only
        static uint32_t lastDebug = 0;
        if (currentTime - lastDebug > 2000) {
            lastDebug = currentTime;
            Serial.printf("📊 BMI270: Accel(%.3f,%.3f,%.3f) Gyro(%.1f,%.1f,%.1f) | Pitch: %.1f°, Roll: %.1f°, Yaw: %.1f°\n", 
                         accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitch, roll, yaw);
        }
    } else {
        Serial.printf("❌ BMI270 data read failed (error: %d)\n", result);
        
        // Count read failures
        static int failureCount = 0;
        failureCount++;
        if (failureCount % 10 == 1) {  // Report every 10th failure
            Serial.printf("⚠️ BMI270 read failures: %d\n", failureCount);
        }
    }
}

// Minimal setup for testing BMI270
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=================================");
    Serial.println("🚁 R6 Drone - BMI270 Library Test");
    Serial.println("=================================");
    
    // Initialize LED strip first
    Serial.println("Initializing LED strip...");
    strip.begin();
    strip.show();
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
    strip.show();
    Serial.println("LED strip initialized");
    
    // Initialize BMI270
    Serial.println("Initializing BMI270 IMU...");
    if (initializeIMU()) {
        Serial.println("✅ BMI270 IMU initialized successfully");
        
        // Auto-calibrate on startup
        delay(1000);
        calibrateIMU();
        
        Serial.println("📐 Gyroscope and accelerometer ready for balance control");
    } else {
        Serial.println("❌ BMI270 initialization failed");
    }
    
    // Start WiFi and web server for IMU monitoring
    Serial.println("Starting WiFi Access Point...");
    WiFi.softAP("R6_Drone_IMU", "12345678");
    Serial.printf("WiFi AP started. Connect to 'R6_Drone_IMU' and visit http://%s\n", WiFi.softAPIP().toString().c_str());
    
    // Setup web server routes
    server.on("/", []() {
        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>BMI270 IMU Monitor</title>
    <meta charset='utf-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body { font-family: Arial; margin: 20px; background: #1a1a1a; color: #fff; }
        .container { max-width: 800px; margin: 0 auto; }
        .data-box { background: #2a2a2a; padding: 20px; margin: 10px 0; border-radius: 10px; }
        .value { font-size: 24px; font-weight: bold; color: #4CAF50; }
        .label { font-size: 14px; color: #ccc; }
        .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
        .online { background: #4CAF50; }
        .calibrate-btn { background: #2196F3; color: white; border: none; padding: 15px 30px; font-size: 16px; border-radius: 5px; cursor: pointer; }
        .refresh-info { color: #888; font-size: 12px; }
    </style>
    <script>
        function updateData() {
            fetch('/imu-data').then(r => r.json()).then(data => {
                document.getElementById('accel-x').textContent = data.accel.x.toFixed(3);
                document.getElementById('accel-y').textContent = data.accel.y.toFixed(3);
                document.getElementById('accel-z').textContent = data.accel.z.toFixed(3);
                document.getElementById('gyro-x').textContent = data.gyro.x.toFixed(1);
                document.getElementById('gyro-y').textContent = data.gyro.y.toFixed(1);
                document.getElementById('gyro-z').textContent = data.gyro.z.toFixed(1);
                document.getElementById('pitch').textContent = data.orientation.pitch.toFixed(1);
                document.getElementById('roll').textContent = data.orientation.roll.toFixed(1);
                document.getElementById('yaw').textContent = data.orientation.yaw.toFixed(1);
                document.getElementById('calibrated').textContent = data.calibrated ? 'Calibrated' : 'Not Calibrated';
                document.getElementById('status').className = 'status ' + (data.online ? 'online' : 'offline');
                document.getElementById('status').textContent = data.online ? 'IMU Online' : 'IMU Offline';
            });
        }
        function calibrate() {
            fetch('/calibrate', {method: 'POST'}).then(() => {
                alert('Calibration started! Keep drone still.');
                setTimeout(updateData, 2000);
            });
        }
        setInterval(updateData, 500);
        updateData();
    </script>
</head>
<body>
    <div class='container'>
        <h1>🚁 R6 Drone - BMI270 IMU Monitor</h1>
        <div id='status' class='status'>Loading...</div>
        
        <div class='data-box'>
            <h3>📐 Accelerometer (g)</h3>
            <div>X: <span id='accel-x' class='value'>--</span> <span class='label'>g</span></div>
            <div>Y: <span id='accel-y' class='value'>--</span> <span class='label'>g</span></div>
            <div>Z: <span id='accel-z' class='value'>--</span> <span class='label'>g</span></div>
        </div>
        
        <div class='data-box'>
            <h3>🔄 Gyroscope (°/s)</h3>
            <div>X: <span id='gyro-x' class='value'>--</span> <span class='label'>°/s</span></div>
            <div>Y: <span id='gyro-y' class='value'>--</span> <span class='label'>°/s</span></div>
            <div>Z: <span id='gyro-z' class='value'>--</span> <span class='label'>°/s</span></div>
        </div>
        
        <div class='data-box'>
            <h3>🎯 Orientation</h3>
            <div>Pitch: <span id='pitch' class='value'>--</span> <span class='label'>°</span></div>
            <div>Roll: <span id='roll' class='value'>--</span> <span class='label'>°</span></div>
            <div>Yaw: <span id='yaw' class='value'>--</span> <span class='label'>°</span></div>
        </div>
        
        <div class='data-box'>
            <button class='calibrate-btn' onclick='calibrate()'>🔧 Calibrate Gyroscope</button>
            <div>Status: <span id='calibrated'>--</span></div>
        </div>
        
        <p class='refresh-info'>Data refreshes every 500ms automatically</p>
    </div>
</body>
</html>
        )";
        server.send(200, "text/html", html);
    });
    
    server.on("/imu-data", []() {
        String json = "{";
        json += "\"online\":" + String(imuInitialized ? "true" : "false") + ",";
        json += "\"calibrated\":" + String(imuCalibrated ? "true" : "false") + ",";
        json += "\"accel\":{\"x\":" + String(accelX, 3) + ",\"y\":" + String(accelY, 3) + ",\"z\":" + String(accelZ, 3) + "},";
        json += "\"gyro\":{\"x\":" + String(gyroX, 1) + ",\"y\":" + String(gyroY, 1) + ",\"z\":" + String(gyroZ, 1) + "},";
        json += "\"orientation\":{\"pitch\":" + String(pitch, 1) + ",\"roll\":" + String(roll, 1) + ",\"yaw\":" + String(yaw, 1) + "}";
        json += "}";
        server.send(200, "application/json", json);
    });
    
    server.on("/calibrate", HTTP_POST, []() {
        calibrateIMU();
        server.send(200, "text/plain", "Calibration started");
    });
    
    server.begin();
    
    Serial.println("=================================");
    Serial.println("BMI270 Library Test Ready!");
    Serial.println("Watch serial monitor for IMU data...");
    Serial.println("Or visit the web interface for live monitoring!");
    Serial.println("=================================");
}

void loop() {
    updateIMU();
    server.handleClient();  // Handle web requests
    
    // Simple status every 5 seconds
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        lastStatus = millis();
        Serial.printf("[STATUS] IMU: %s | Free heap: %d bytes | Uptime: %d seconds\n",
                     imuInitialized ? "Online" : "Offline", ESP.getFreeHeap(), millis() / 1000);
    }
    
    delay(10); // Small delay
}