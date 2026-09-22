/**
 * @file main_integrated.cpp
 * @brief R6 Drone - Complete Integrated System
 * @author Peter
 * @version 2.0.0
 * 
 * Features:
 * - BMI270 IMU with SparkFun library
 * - OV3660 Camera streaming
 * - WS2812B LED strip animations
 * - Spotlight control
 * - Complete responsive web interface
 * - Real-time system monitoring
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "esp_camera.h"
#include "config.h"
#include "SparkFun_BMI270_Arduino_Library.h"

// ==========================================
// GLOBAL OBJECTS
// ==========================================
WebServer server(80);
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);
BMI270 imu;

// ==========================================
// SYSTEM STATE VARIABLES
// ==========================================
// IMU state
bool imuInitialized = false;
bool imuCalibrated = false;
bool calibrationInProgress = false;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
uint32_t lastIMUUpdate = 0;

// Camera state
bool cameraInitialized = false;

// LED state
bool ledEnabled = true;
uint32_t lastLedUpdate = 0;
int ledAnimationStep = 0;
uint8_t ledR = 64, ledG = 64, ledB = 0;  // Default green

// Spotlight state
bool spotlightEnabled = false;
int spotlightBrightness = 128;

// System state
uint32_t lastStatusUpdate = 0;

// ==========================================
// CAMERA INITIALIZATION
// ==========================================
bool initializeCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
    config.pixel_format = CAMERA_PIXEL_FORMAT;
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_QUALITY;
    config.fb_count = CAMERA_FB_COUNT;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Apply camera settings for optimal performance
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, 0);     // -2 to 2
        s->set_special_effect(s, 0); // 0 to 6 (0-No Effect, 1-Negative, 2-Grayscale, 3-Red Tint, 4-Green Tint, 5-Blue Tint, 6-Sepia)
        s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_aec_value(s, 300);    // 0 to 1200
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
        s->set_agc_gain(s, 0);       // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);            // 0 = disable , 1 = enable
        s->set_wpc(s, 1);            // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
        s->set_lenc(s, 1);           // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
        s->set_vflip(s, 0);          // 0 = disable , 1 = enable
        s->set_dcw(s, 1);            // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    }

    return true;
}

// ==========================================
// BMI270 IMU FUNCTIONS
// ==========================================
bool initializeIMU() {
    Serial.println("=== BMI270 IMU INITIALIZATION ===");
    Serial.printf("I2C Config: SDA=GPIO%d, SCL=GPIO%d, Freq=400kHz\n", IMU_SDA_PIN, IMU_SCL_PIN);
    
    // Initialize I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    delay(100);
    
    // Scan for I2C devices
    Serial.println("🔍 Scanning I2C bus...");
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
        Serial.println("❌ No I2C devices found! Check BMI270 wiring");
        return false;
    }
    
    // Initialize BMI270 with SparkFun library
    Serial.println("🎯 Initializing BMI270 with SparkFun library...");
    int8_t result = imu.beginI2C(0x69);  // Try default address first
    if (result != BMI2_OK) {
        result = imu.beginI2C(0x68);     // Try alternate address
        if (result != BMI2_OK) {
            Serial.printf("❌ BMI270 initialization failed (error: %d)\n", result);
            return false;
        }
    }
    
    Serial.println("✅ BMI270 initialized successfully!");
    
    // Test sensor data
    result = imu.getSensorData();
    if (result == BMI2_OK) {
        Serial.printf("📊 Initial readings - Accel: %.3f,%.3f,%.3f g | Gyro: %.1f,%.1f,%.1f °/s\n",
                     imu.data.accelX, imu.data.accelY, imu.data.accelZ,
                     imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ);
    }
    
    return true;
}

void calibrateIMU() {
    if (!imuInitialized) return;
    
    Serial.println("🔧 Calibrating BMI270 gyroscope...");
    Serial.println("📍 Keep drone completely still!");
    
    calibrationInProgress = true;
    float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int samples = 0;
    
    // Collect 100 samples
    while (samples < 100) {
        if (imu.getSensorData() == BMI2_OK) {
            gyroXSum += imu.data.gyroX;
            gyroYSum += imu.data.gyroY;
            gyroZSum += imu.data.gyroZ;
            samples++;
            
            if (samples % 25 == 0) {
                Serial.printf("📊 Calibration: %d/100 samples\n", samples);
            }
        }
        delay(10);
    }
    
    // Calculate offsets
    gyroOffsetX = gyroXSum / samples;
    gyroOffsetY = gyroYSum / samples;
    gyroOffsetZ = gyroZSum / samples;
    
    Serial.printf("✅ Calibration complete! Offsets: X=%.3f, Y=%.3f, Z=%.3f °/s\n", 
                 gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    
    imuCalibrated = true;
    calibrationInProgress = false;
}

void updateIMU() {
    if (!imuInitialized) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastIMUUpdate < 10) return; // 100Hz update
    lastIMUUpdate = currentTime;
    
    // Read sensor data
    if (imu.getSensorData() == BMI2_OK) {
        // Get raw data
        accelX = imu.data.accelX;
        accelY = imu.data.accelY;
        accelZ = imu.data.accelZ;
        
        // Apply calibration to gyro
        gyroX = imu.data.gyroX - gyroOffsetX;
        gyroY = imu.data.gyroY - gyroOffsetY;
        gyroZ = imu.data.gyroZ - gyroOffsetZ;
        
        // Calculate orientation using complementary filter
        float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
        float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
        
        float dt = 0.01; // 10ms
        pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accelPitch;
        roll = 0.98 * (roll + gyroY * dt) + 0.02 * accelRoll;
        yaw += gyroZ * dt;
    }
}

// ==========================================
// LED ANIMATION FUNCTIONS
// ==========================================
void updateLEDAnimation() {
    if (!ledEnabled) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastLedUpdate < LED_ANIMATION_SPEED) return;
    lastLedUpdate = currentTime;
    
    // Clear all LEDs
    strip.clear();
    
    // Middle-out animation pattern
    switch (ledAnimationStep) {
        case 0: // All off
            break;
        case 1: // LEDs 2,3 (middle)
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
        case 2: // LEDs 1,2,3,4
            strip.setPixelColor(1, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(4, strip.Color(ledR, ledG, ledB));
            break;
        case 3: // All LEDs 0,1,2,3,4,5
            for (int i = 0; i < LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
            }
            break;
        case 4: // LEDs 1,2,3,4
            strip.setPixelColor(1, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(4, strip.Color(ledR, ledG, ledB));
            break;
        case 5: // LEDs 2,3 (middle)
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
    }
    
    strip.show();
    
    // Advance animation
    ledAnimationStep++;
    if (ledAnimationStep > 5) {
        ledAnimationStep = 0;
    }
}

// ==========================================
// SPOTLIGHT CONTROL
// ==========================================
void initializeSpotlight() {
    ledcSetup(SPOTLIGHT_PWM_CHANNEL, SPOTLIGHT_PWM_FREQ, SPOTLIGHT_PWM_RESOLUTION);
    ledcAttachPin(SPOTLIGHT_PIN, SPOTLIGHT_PWM_CHANNEL);
    ledcWrite(SPOTLIGHT_PWM_CHANNEL, 0); // Start with spotlight off
}

void updateSpotlight() {
    if (spotlightEnabled) {
        ledcWrite(SPOTLIGHT_PWM_CHANNEL, spotlightBrightness);
    } else {
        ledcWrite(SPOTLIGHT_PWM_CHANNEL, 0);
    }
}

// ==========================================
// CAMERA STREAMING HANDLER (NON-BLOCKING)
// ==========================================
void handleCameraStream() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        server.send(503, "text/plain", "Camera fail");
        return;
    }
    
    WiFiClient client = server.client();
    client.print("HTTP/1.1 200 OK\r\n"
                 "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                 "\r\n");
    
    // Non-blocking approach - only send a few frames then exit
    for (int frameCount = 0; frameCount < 5 && client.connected(); frameCount++) {
        esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        if (!fb) break;
        
        client.printf("--frame\r\n"
                     "Content-Type: image/jpeg\r\n"
                     "Content-Length: %u\r\n\r\n", fb->len);
        
        if (client.write(fb->buf, fb->len) != fb->len) break;
        client.print("\r\n");
        
        // Very short delay to allow other tasks
        delay(10);
    }
    
    esp_camera_fb_return(fb);
}

// ==========================================
// WEB SERVER ROUTES
// ==========================================
void setupWebServer() {
    // Main page - Simple version without emojis
    server.on("/", []() {
        String html = "<!DOCTYPE html><html><head><title>R6 Drone</title>";
        html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
        html += "<style>body{font-family:Arial;margin:10px;background:#111;color:#fff}";
        html += ".container{max-width:800px;margin:0 auto}h1{color:#ff8c00;text-align:center}";
        html += ".section{background:#222;padding:15px;margin:10px 0;border-radius:8px}";
        html += ".btn{background:#ff8c00;color:#000;border:none;padding:10px 15px;border-radius:5px;cursor:pointer;margin:5px}";
        html += ".btn.active{background:#4CAF50}.slider{width:100%;margin:5px 0}";
        html += ".data{font-weight:bold;color:#4CAF50}";
        html += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}";
        html += "@media(max-width:600px){.grid{grid-template-columns:1fr}}";
        html += "</style></head><body><div class=\"container\">";
        html += "<h1>R6 Drone Control</h1>";
        
        // Camera section
        html += "<div class=\"section\"><h3>Camera</h3>";
        html += "<img id=\"cam\" style=\"width:100%;border-radius:5px;background:#333;min-height:200px\" alt=\"Click button to enable camera\">";
        html += "<br><button class=\"btn\" onclick=\"enableCamera()\">Enable Camera</button>";
        html += "<button class=\"btn\" onclick=\"toggleCameraMode()\" style=\"display:none\" id=\"mode-btn\">Stream Mode</button></div>";
        
        // IMU section
        html += "<div class=\"section\"><h3>IMU Data</h3><div class=\"grid\">";
        html += "<div>Pitch: <span id=\"pitch\" class=\"data\">--</span></div>";
        html += "<div>Roll: <span id=\"roll\" class=\"data\">--</span></div>";
        html += "<div>Yaw: <span id=\"yaw\" class=\"data\">--</span></div>";
        html += "<div>Status: <span id=\"imu-status\" class=\"data\">--</span></div></div>";
        html += "<button class=\"btn\" onclick=\"calibrateIMU()\">Calibrate</button></div>";
        
        // LED section
        html += "<div class=\"section\"><h3>LEDs</h3>";
        html += "<button id=\"led-btn\" class=\"btn active\" onclick=\"toggleLED()\">Toggle</button><br>";
        html += "R: <input type=\"range\" min=\"0\" max=\"255\" value=\"64\" id=\"r\" class=\"slider\" oninput=\"updateColor()\"><br>";
        html += "G: <input type=\"range\" min=\"0\" max=\"255\" value=\"64\" id=\"g\" class=\"slider\" oninput=\"updateColor()\"><br>";
        html += "B: <input type=\"range\" min=\"0\" max=\"255\" value=\"0\" id=\"b\" class=\"slider\" oninput=\"updateColor()\"></div>";
        
        // Spotlight section
        html += "<div class=\"section\"><h3>Spotlight</h3>";
        html += "<button id=\"spot-btn\" class=\"btn\" onclick=\"toggleSpot()\">Toggle</button><br>";
        html += "Brightness: <input type=\"range\" min=\"0\" max=\"255\" value=\"128\" id=\"spot\" class=\"slider\" oninput=\"updateSpot()\"></div>";
        
        // Status section
        html += "<div class=\"section\"><h3>Status</h3>";
        html += "<div>Memory: <span id=\"mem\" class=\"data\">--</span></div>";
        html += "<div>Uptime: <span id=\"up\" class=\"data\">--</span></div></div>";
        
        html += "</div>";
        
        // JavaScript
        html += "<script>";
        html += "let streamMode=false;";
        html += "function updateData(){fetch('/api/status').then(r=>r.json()).then(d=>{";
        html += "document.getElementById('pitch').textContent=d.imu.pitch.toFixed(1)+'°';";
        html += "document.getElementById('roll').textContent=d.imu.roll.toFixed(1)+'°';";
        html += "document.getElementById('yaw').textContent=d.imu.yaw.toFixed(1)+'°';";
        html += "document.getElementById('imu-status').textContent=d.imu.calibrated?'Calibrated':'Raw';";
        html += "document.getElementById('mem').textContent=Math.floor(d.memory/1024)+'KB';";
        html += "document.getElementById('up').textContent=Math.floor(d.uptime/1000)+'s';";
        html += "}).catch(e=>console.log('Failed:',e));}";
        html += "let cameraEnabled=false,streamMode=false,cameraError=0;";
        html += "function updateCamera(){if(cameraEnabled&&!streamMode&&cameraError<3){";
        html += "document.getElementById('cam').src='/snapshot?t='+Date.now();}}";
        html += "function enableCamera(){cameraEnabled=true;";
        html += "document.getElementById('mode-btn').style.display='inline';";
        html += "document.getElementById('cam').src='/snapshot';";
        html += "setInterval(updateCamera,3000);}";
        html += "function toggleCameraMode(){streamMode=!streamMode;cameraError=0;";
        html += "if(streamMode){document.getElementById('cam').src='/stream';}";
        html += "else{document.getElementById('cam').src='/snapshot';}}";
        html += "document.getElementById('cam').onerror=function(){cameraError++;console.log('Camera error',cameraError);};";
        html += "document.getElementById('cam').onload=function(){cameraError=0;};";
        html += "function toggleLED(){fetch('/api/led/toggle',{method:'POST'});";
        html += "document.getElementById('led-btn').classList.toggle('active');}";
        html += "function updateColor(){const r=document.getElementById('r').value;";
        html += "const g=document.getElementById('g').value;const b=document.getElementById('b').value;";
        html += "fetch('/api/led/color',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({r:+r,g:+g,b:+b})});}";
        html += "function toggleSpot(){fetch('/api/spotlight/toggle',{method:'POST'});";
        html += "document.getElementById('spot-btn').classList.toggle('active');}";
        html += "function updateSpot(){const v=document.getElementById('spot').value;";
        html += "fetch('/api/spotlight/brightness',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({brightness:+v})});}";
        html += "function calibrateIMU(){fetch('/api/imu/calibrate',{method:'POST'});}";
        html += "setInterval(updateData,1000);updateData();";
        html += "</script></body></html>";
        
        server.send(200, "text/html", html);
    });
    
    // Camera stream
    server.on("/stream", handleCameraStream);
    
    // Single camera snapshot (lighter alternative)
    server.on("/snapshot", []() {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            server.send(503, "text/plain", "No camera data");
            return;
        }
        
        // Send headers first
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "-1");
        
        // Send the image
        server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
    });
    
    // API endpoints
    server.on("/api/status", []() {
        // Use more memory-efficient approach
        char json[300];
        snprintf(json, sizeof(json), 
                "{\"status\":\"Online\",\"uptime\":%lu,\"memory\":%u,\"camera\":%s,"
                "\"imu\":{\"online\":%s,\"calibrated\":%s,\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f}}",
                millis(), ESP.getFreeHeap(), 
                cameraInitialized ? "true" : "false",
                imuInitialized ? "true" : "false",
                imuCalibrated ? "true" : "false",
                pitch, roll, yaw);
        server.send(200, "application/json", json);
    });
    
    server.on("/api/led/toggle", HTTP_POST, []() {
        ledEnabled = !ledEnabled;
        if (!ledEnabled) {
            strip.clear();
            strip.show();
        }
        server.send(200, "text/plain", ledEnabled ? "ON" : "OFF");
    });
    
    server.on("/api/led/color", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            // Parse JSON manually (simple approach)
            String body = server.arg("plain");
            int rPos = body.indexOf("\"r\":") + 4;
            int gPos = body.indexOf("\"g\":") + 4;
            int bPos = body.indexOf("\"b\":") + 4;
            
            if (rPos > 3 && gPos > 3 && bPos > 3) {
                ledR = body.substring(rPos, body.indexOf(",", rPos)).toInt();
                ledG = body.substring(gPos, body.indexOf(",", gPos)).toInt();
                ledB = body.substring(bPos, body.indexOf("}", bPos)).toInt();
            }
        }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/api/spotlight/toggle", HTTP_POST, []() {
        spotlightEnabled = !spotlightEnabled;
        updateSpotlight();
        server.send(200, "text/plain", spotlightEnabled ? "ON" : "OFF");
    });
    
    server.on("/api/spotlight/brightness", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            int pos = body.indexOf("\"brightness\":") + 13;
            if (pos > 12) {
                spotlightBrightness = body.substring(pos, body.indexOf("}", pos)).toInt();
                updateSpotlight();
            }
        }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/api/imu/calibrate", HTTP_POST, []() {
        calibrateIMU();
        server.send(200, "text/plain", "Calibration started");
    });
    
    server.begin();
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    
    Serial.println("=================================");
    Serial.println("🚁 R6 Drone - Integrated System v2.0");
    Serial.println("=================================");
    
    // Initialize LED strip
    Serial.println("🔹 Initializing LED strip...");
    strip.begin();
    strip.show();
    Serial.println("✅ LED strip ready");
    
    // Initialize spotlight
    Serial.println("🔹 Initializing spotlight...");
    initializeSpotlight();
    Serial.println("✅ Spotlight ready");
    
    // Initialize BMI270 IMU
    Serial.println("🔹 Initializing BMI270 IMU...");
    if (initializeIMU()) {
        imuInitialized = true;
        Serial.println("✅ BMI270 IMU ready");
        
        // Auto-calibrate
        delay(1000);
        calibrateIMU();
    } else {
        Serial.println("❌ BMI270 IMU failed to initialize");
    }
    
    // Initialize camera
    Serial.println("🔹 Initializing OV3660 camera...");
    if (initializeCamera()) {
        cameraInitialized = true;
        Serial.println("✅ Camera ready");
    } else {
        Serial.println("❌ Camera failed to initialize");
    }
    
    // Start WiFi Access Point
    Serial.println("🔹 Starting WiFi Access Point...");
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("✅ WiFi AP: %s (Password: %s)\n", AP_SSID, AP_PASSWORD);
    Serial.printf("🌐 Web interface: http://%s\n", WiFi.softAPIP().toString().c_str());
    
    // Setup web server
    Serial.println("🔹 Starting web server...");
    setupWebServer();
    Serial.println("✅ Web server ready");
    
    Serial.println("=================================");
    Serial.println("🎯 R6 Drone - ALL SYSTEMS READY!");
    Serial.println("=================================");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Handle web requests first (most important)
    server.handleClient();
    
    // Update components with time slicing
    static uint8_t taskCounter = 0;
    taskCounter++;
    
    // Always update IMU (critical for stability)
    updateIMU();
    
    // Time-slice other tasks to prevent blocking
    switch (taskCounter % 4) {
        case 0:
            updateLEDAnimation();
            break;
        case 1:
            updateSpotlight();
            break;
        case 2:
            server.handleClient(); // Handle more web requests
            break;
        case 3:
            // System status (every ~20 loops = ~200ms * 20 = 4 seconds)
            static uint32_t statusCounter = 0;
            if (++statusCounter >= 200) { // 200 * 10ms = 2 seconds
                statusCounter = 0;
                uint32_t freeHeap = ESP.getFreeHeap();
                Serial.printf("[STATUS] IMU: %s | Camera: %s | Memory: %dKB | Uptime: %ds\n",
                             imuInitialized ? (imuCalibrated ? "CAL" : "ON") : "OFF",
                             cameraInitialized ? "ON" : "OFF",
                             freeHeap / 1024,
                             millis() / 1000);
                
                // Memory warning
                if (freeHeap < 100000) {
                    Serial.printf("⚠️  LOW MEMORY WARNING: %d bytes free\n", freeHeap);
                }
            }
            break;
    }
    
    delay(5); // Shorter delay for better responsiveness
}