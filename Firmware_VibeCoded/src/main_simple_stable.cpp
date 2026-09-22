/**
 * @file main_simple_stable.cpp
 * @brief R6 Drone - Simple Stable Version
 * @author Peter
 * @version 2.1.0
 * 
 * Back to basics - stable BMI270 + LEDs + Spotlight + Simple Web Interface
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
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
uint32_t lastIMUUpdate = 0;

// LED state
bool ledEnabled = true;
uint32_t lastLedUpdate = 0;
int ledAnimationStep = 0;
uint8_t ledR = 64, ledG = 64, ledB = 0;  // Default green

// Spotlight state
bool spotlightEnabled = false;
int spotlightBrightness = 128;

// Camera state
bool cameraInitialized = false;

// System state
uint32_t lastStatusUpdate = 0;

// ==========================================
// BMI270 IMU FUNCTIONS
// ==========================================
bool initializeIMU() {
    Serial.println("=== BMI270 IMU INITIALIZATION ===");
    Serial.printf("I2C Config: SDA=GPIO%d, SCL=GPIO%d, Freq=400kHz\n", IMU_SDA_PIN, IMU_SCL_PIN);
    
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    delay(100);
    
    Serial.println("Initializing BMI270 with SparkFun library...");
    int8_t result = imu.beginI2C(0x69);
    if (result != BMI2_OK) {
        result = imu.beginI2C(0x68);
        if (result != BMI2_OK) {
            Serial.printf("BMI270 initialization failed (error: %d)\n", result);
            return false;
        }
    }
    
    Serial.println("BMI270 initialized successfully!");
    return true;
}

void calibrateIMU() {
    if (!imuInitialized) return;
    
    Serial.println("Calibrating BMI270 gyroscope...");
    float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int samples = 0;
    
    while (samples < 100) {
        if (imu.getSensorData() == BMI2_OK) {
            gyroXSum += imu.data.gyroX;
            gyroYSum += imu.data.gyroY;
            gyroZSum += imu.data.gyroZ;
            samples++;
            
            if (samples % 25 == 0) {
                Serial.printf("Calibration: %d/100 samples\n", samples);
            }
        }
        delay(10);
    }
    
    gyroOffsetX = gyroXSum / samples;
    gyroOffsetY = gyroYSum / samples;
    gyroOffsetZ = gyroZSum / samples;
    
    Serial.printf("Calibration complete! Offsets: X=%.3f, Y=%.3f, Z=%.3f\n", 
                 gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    imuCalibrated = true;
}

void updateIMU() {
    if (!imuInitialized) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastIMUUpdate < 10) return;
    lastIMUUpdate = currentTime;
    
    if (imu.getSensorData() == BMI2_OK) {
        accelX = imu.data.accelX;
        accelY = imu.data.accelY;
        accelZ = imu.data.accelZ;
        
        gyroX = imu.data.gyroX - gyroOffsetX;
        gyroY = imu.data.gyroY - gyroOffsetY;
        gyroZ = imu.data.gyroZ - gyroOffsetZ;
        
        // Calculate orientation
        float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
        float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
        
        float dt = 0.01;
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
    
    strip.clear();
    
    switch (ledAnimationStep) {
        case 0: break;
        case 1:
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
        case 2:
            for (int i = 1; i <= 4; i++) {
                strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
            }
            break;
        case 3:
            for (int i = 0; i < LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
            }
            break;
        case 4:
            for (int i = 1; i <= 4; i++) {
                strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
            }
            break;
        case 5:
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
    }
    
    strip.show();
    
    ledAnimationStep++;
    if (ledAnimationStep > 5) {
        ledAnimationStep = 0;
    }
}

// ==========================================
// CAMERA FUNCTIONS
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
    config.xclk_freq_hz = 8000000; // Even lower clock for stability
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA; // 800x600 (smaller for faster transfer)
    config.jpeg_quality = 35; // More compression for smaller files (~8-12KB)
    config.fb_count = 1; // Single frame buffer to save memory
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Disable all automatic adjustments for minimal processing
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_brightness(s, 0);     // 0 = neutral
        s->set_contrast(s, 0);       // 0 = neutral
        s->set_saturation(s, 0);     // 0 = neutral
        s->set_special_effect(s, 0); // 0 = No Effect
        s->set_whitebal(s, 0);       // 0 = disable auto white balance
        s->set_awb_gain(s, 0);       // 0 = disable auto white balance gain
        s->set_wb_mode(s, 0);        // Manual white balance
        s->set_exposure_ctrl(s, 0);  // 0 = disable auto exposure
        s->set_aec2(s, 0);           // 0 = disable auto exposure control 2
        s->set_ae_level(s, 0);       // Manual exposure level
        s->set_aec_value(s, 400);    // Fixed exposure value
        s->set_gain_ctrl(s, 0);      // 0 = disable auto gain
        s->set_agc_gain(s, 5);       // Fixed gain value (0-30)
        s->set_gainceiling(s, (gainceiling_t)2);  // Gain ceiling
        s->set_bpc(s, 1);            // Black pixel correction
        s->set_wpc(s, 1);            // White pixel correction
        s->set_raw_gma(s, 1);        // Gamma correction
        s->set_lenc(s, 0);           // 0 = disable lens correction
        s->set_hmirror(s, 0);        // No horizontal mirror
        s->set_vflip(s, 0);          // No vertical flip
        s->set_dcw(s, 0);            // 0 = disable downsize cropping
        s->set_colorbar(s, 0);       // 0 = disable color bar test pattern
    }

    return true;
}

// ==========================================
// SPOTLIGHT CONTROL
// ==========================================
void initializeSpotlight() {
    ledcSetup(SPOTLIGHT_PWM_CHANNEL, SPOTLIGHT_PWM_FREQ, SPOTLIGHT_PWM_RESOLUTION);
    ledcAttachPin(SPOTLIGHT_PIN, SPOTLIGHT_PWM_CHANNEL);
    ledcWrite(SPOTLIGHT_PWM_CHANNEL, 0);
}

void updateSpotlight() {
    ledcWrite(SPOTLIGHT_PWM_CHANNEL, spotlightEnabled ? spotlightBrightness : 0);
}

// ==========================================
// WEB SERVER SETUP
// ==========================================
void setupWebServer() {
    // Simple main page
    server.on("/", []() {
        String html = "<!DOCTYPE html><html><head><title>R6 Drone</title>";
        html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
        html += "<style>";
        html += "body{font-family:Arial;margin:20px;background:#111;color:#fff}";
        html += ".container{max-width:600px;margin:0 auto}";
        html += "h1{color:#ff8c00;text-align:center}";
        html += ".section{background:#222;padding:20px;margin:15px 0;border-radius:10px}";
        html += ".btn{background:#ff8c00;color:#000;border:none;padding:12px 20px;border-radius:5px;cursor:pointer;margin:5px;font-weight:bold}";
        html += ".btn.active{background:#4CAF50}";
        html += ".slider{width:100%;margin:10px 0}";
        html += ".data{font-weight:bold;color:#4CAF50;font-size:18px}";
        html += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:15px;margin:15px 0}";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h1>R6 Drone Control</h1>";
        
        // Camera Section
        if (cameraInitialized) {
            html += "<div class='section'>";
            html += "<h3>Camera View (SVGA 800x600)</h3>";
            html += "<div id='camera-status' style='color:#ff8c00;margin:10px 0'>Camera ready - click buttons to test</div>";
            html += "<img id='camera' style='width:100%;max-width:400px;border-radius:8px;background:#333;border:2px solid #555' alt='No image loaded'>";
            html += "<br><button class='btn' onclick='testCamera()'>Test Endpoint</button>";
            html += "<button class='btn' onclick='takePreview()'>Small Preview</button>";
            html += "<button class='btn' onclick='takeSnapshot()'>Full Image</button>";
            html += "<button class='btn' onclick='toggleCamera()' id='cam-btn'>Auto Mode</button>";
            html += "</div>";
        }
        
        // IMU Data
        html += "<div class='section'>";
        html += "<h3>IMU Data</h3>";
        html += "<div class='grid'>";
        html += "<div>Pitch: <span id='pitch' class='data'>--°</span></div>";
        html += "<div>Roll: <span id='roll' class='data'>--°</span></div>";
        html += "<div>Yaw: <span id='yaw' class='data'>--°</span></div>";
        html += "<div>Status: <span id='imu-status' class='data'>--</span></div>";
        html += "</div>";
        html += "<button class='btn' onclick='calibrateIMU()'>Calibrate IMU</button>";
        html += "</div>";
        
        // LED Controls
        html += "<div class='section'>";
        html += "<h3>LED Strip Control</h3>";
        html += "<button id='led-btn' class='btn active' onclick='toggleLED()'>Toggle LEDs</button><br>";
        html += "<div class='grid'>";
        html += "<div>Red: <input type='range' min='0' max='255' value='64' id='r' class='slider' onchange='updateColor()'></div>";
        html += "<div>Green: <input type='range' min='0' max='255' value='64' id='g' class='slider' onchange='updateColor()'></div>";
        html += "<div>Blue: <input type='range' min='0' max='255' value='0' id='b' class='slider' onchange='updateColor()'></div>";
        html += "</div>";
        html += "</div>";
        
        // Spotlight Controls  
        html += "<div class='section'>";
        html += "<h3>Spotlight Control</h3>";
        html += "<button id='spot-btn' class='btn' onclick='toggleSpotlight()'>Toggle Spotlight</button><br>";
        html += "Brightness: <input type='range' min='0' max='255' value='128' id='spot' class='slider' onchange='updateSpotlight()'>";
        html += "</div>";
        
        // System Status
        html += "<div class='section'>";
        html += "<h3>System Status</h3>";
        html += "<div>Memory: <span id='memory' class='data'>--</span></div>";
        html += "<div>Uptime: <span id='uptime' class='data'>--</span></div>";
        html += "</div>";
        
        html += "</div>";
        
        // JavaScript
        html += "<script>";
        html += "let cameraActive=false,cameraInterval=null;";
        html += "function updateData(){";
        html += "fetch('/api/status').then(r=>r.json()).then(d=>{";
        html += "document.getElementById('pitch').textContent=d.pitch.toFixed(1)+'°';";
        html += "document.getElementById('roll').textContent=d.roll.toFixed(1)+'°';";
        html += "document.getElementById('yaw').textContent=d.yaw.toFixed(1)+'°';";
        html += "document.getElementById('imu-status').textContent=d.calibrated?'Calibrated':'Raw';";
        html += "document.getElementById('memory').textContent=Math.floor(d.memory/1024)+'KB';";
        html += "document.getElementById('uptime').textContent=Math.floor(d.uptime/1000)+'s';";
        html += "}).catch(e=>console.log('Update failed'));";
        html += "}";
        html += "function updateStatus(msg){document.getElementById('camera-status').textContent=msg;}";
        html += "function testCamera(){";
        html += "updateStatus('Testing camera endpoint...');";
        html += "fetch('/camera/test').then(r=>r.text()).then(d=>{";
        html += "updateStatus('Camera endpoint: '+d);";
        html += "}).catch(e=>updateStatus('Camera test failed: '+e));";
        html += "}";
        html += "function updateCamera(){";
        html += "if(cameraActive){";
        html += "updateStatus('Auto refresh...');";
        html += "const img=document.getElementById('camera');";
        html += "img.onload=()=>updateStatus('Auto image updated');";
        html += "img.onerror=()=>updateStatus('Auto refresh failed');";
        html += "img.src='/camera/preview?t='+Date.now();";
        html += "}}";
        html += "function toggleCamera(){";
        html += "cameraActive=!cameraActive;";
        html += "const btn=document.getElementById('cam-btn');";
        html += "if(cameraActive){";
        html += "btn.textContent='Stop Auto';btn.classList.add('active');";
        html += "cameraInterval=setInterval(updateCamera,3000);updateCamera();";
        html += "}else{";
        html += "btn.textContent='Auto Refresh';btn.classList.remove('active');";
        html += "clearInterval(cameraInterval);updateStatus('Auto refresh stopped');}}";
        html += "function takePreview(){";
        html += "updateStatus('Loading small preview...');";
        html += "const img=document.getElementById('camera');";
        html += "img.onload=()=>updateStatus('Small preview loaded (160x120)');";
        html += "img.onerror=()=>updateStatus('Preview failed');";
        html += "img.src='/camera/preview?t='+Date.now();";
        html += "}";
        html += "function takeSnapshot(){";
        html += "updateStatus('Loading full image...');";
        html += "const img=document.getElementById('camera');";
        html += "img.onload=()=>updateStatus('Full image loaded (800x600)');";
        html += "img.onerror=()=>updateStatus('Full image failed');";
        html += "img.src='/camera/snapshot?t='+Date.now();";
        html += "}";
        html += "function toggleLED(){";
        html += "fetch('/api/led/toggle',{method:'POST'});";
        html += "document.getElementById('led-btn').classList.toggle('active');";
        html += "}";
        html += "function updateColor(){";
        html += "const r=document.getElementById('r').value;";
        html += "const g=document.getElementById('g').value;";
        html += "const b=document.getElementById('b').value;";
        html += "fetch('/api/led/color?r='+r+'&g='+g+'&b='+b,{method:'POST'});";
        html += "}";
        html += "function toggleSpotlight(){";
        html += "fetch('/api/spotlight/toggle',{method:'POST'});";
        html += "document.getElementById('spot-btn').classList.toggle('active');";
        html += "}";
        html += "function updateSpotlight(){";
        html += "const v=document.getElementById('spot').value;";
        html += "fetch('/api/spotlight/brightness?v='+v,{method:'POST'});";
        html += "}";
        html += "function calibrateIMU(){";
        html += "fetch('/api/imu/calibrate',{method:'POST'});";
        html += "}";
        html += "setInterval(updateData,1000);";
        html += "updateData();";
        html += "</script>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
    });
    
    // API endpoints
    server.on("/api/status", []() {
        char json[200];
        snprintf(json, sizeof(json), 
                "{\"uptime\":%lu,\"memory\":%u,\"calibrated\":%s,"
                "\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f}",
                millis(), ESP.getFreeHeap(), 
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
        if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
            ledR = server.arg("r").toInt();
            ledG = server.arg("g").toInt();
            ledB = server.arg("b").toInt();
        }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/api/spotlight/toggle", HTTP_POST, []() {
        spotlightEnabled = !spotlightEnabled;
        updateSpotlight();
        server.send(200, "text/plain", spotlightEnabled ? "ON" : "OFF");
    });
    
    server.on("/api/spotlight/brightness", HTTP_POST, []() {
        if (server.hasArg("v")) {
            spotlightBrightness = server.arg("v").toInt();
            updateSpotlight();
        }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/api/imu/calibrate", HTTP_POST, []() {
        calibrateIMU();
        server.send(200, "text/plain", "OK");
    });
    
    // Camera endpoints (only if camera is initialized)
    if (cameraInitialized) {
        server.on("/camera/snapshot", []() {
            Serial.println("Camera snapshot requested");
            
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed - no frame buffer");
                server.send(503, "text/plain", "No image");
                return;
            }
            
            if (fb->len == 0) {
                Serial.println("Camera capture failed - empty buffer");
                esp_camera_fb_return(fb);
                server.send(503, "text/plain", "Empty image");
                return;
            }
            
            Serial.printf("Camera: Captured %d bytes\n", fb->len);
            
            // Use server.send_P for more reliable transfer
            server.sendHeader("Content-Type", "image/jpeg");
            server.sendHeader("Cache-Control", "no-cache, no-store");
            server.sendHeader("Connection", "close");
            
            // Send image in one go using send_P
            server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
            
            esp_camera_fb_return(fb);
            Serial.println("Camera: Image sent successfully");
        });
        
        // Simple test endpoint
        server.on("/camera/test", []() {
            server.send(200, "text/plain", "Camera endpoint working");
        });
        
        // Tiny preview endpoint (very small image)
        server.on("/camera/preview", []() {
            // Temporarily set to lowest resolution for reliable transfer
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                framesize_t original_size = s->status.framesize;
                s->set_framesize(s, FRAMESIZE_QQVGA); // 160x120 - tiny!
                
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb && fb->len > 0) {
                    Serial.printf("Preview: %d bytes\n", fb->len);
                    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
                    esp_camera_fb_return(fb);
                } else {
                    server.send(503, "text/plain", "Preview failed");
                    if (fb) esp_camera_fb_return(fb);
                }
                
                // Restore original resolution
                s->set_framesize(s, original_size);
            } else {
                server.send(503, "text/plain", "Camera not available");
            }
        });
    }
    
    server.begin();
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    
    Serial.println("=================================");
    Serial.println("R6 Drone - Simple Stable System");
    Serial.println("=================================");
    
    // Initialize LED strip
    Serial.println("Initializing LED strip...");
    strip.begin();
    strip.show();
    Serial.println("LED strip ready");
    
    // Initialize spotlight
    Serial.println("Initializing spotlight...");
    initializeSpotlight();
    Serial.println("Spotlight ready");
    
    // Initialize BMI270 IMU
    Serial.println("Initializing BMI270 IMU...");
    if (initializeIMU()) {
        imuInitialized = true;
        Serial.println("BMI270 IMU ready");
        delay(1000);
        calibrateIMU();
    } else {
        Serial.println("BMI270 IMU failed to initialize");
    }
    
    // Initialize Camera (lightweight)
    Serial.println("Initializing OV3660 camera...");
    if (initializeCamera()) {
        cameraInitialized = true;
        Serial.println("Camera ready (XGA 1024x768, manual settings)");
        
        // Test camera immediately
        Serial.println("Testing camera capture...");
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            Serial.printf("Camera test: Got frame %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
            esp_camera_fb_return(fb);
        } else {
            Serial.println("Camera test: Failed to get frame");
        }
    } else {
        Serial.println("Camera failed to initialize (continuing without camera)");
    }
    
    // Start WiFi Access Point
    Serial.println("Starting WiFi Access Point...");
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("WiFi AP: %s (Password: %s)\n", AP_SSID, AP_PASSWORD);
    Serial.printf("Web interface: http://%s\n", WiFi.softAPIP().toString().c_str());
    
    // Setup web server
    Serial.println("Starting web server...");
    setupWebServer();
    Serial.println("Web server ready");
    
    Serial.println("=================================");
    Serial.println("R6 Drone - ALL SYSTEMS READY!");
    Serial.println("=================================");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    updateIMU();
    updateLEDAnimation();
    updateSpotlight();
    server.handleClient();
    
    // System status every 5 seconds
    uint32_t currentTime = millis();
    if (currentTime - lastStatusUpdate > 5000) {
        lastStatusUpdate = currentTime;
        Serial.printf("[STATUS] IMU: %s | Camera: %s | Memory: %dKB | Uptime: %ds\n",
                     imuInitialized ? (imuCalibrated ? "CAL" : "ON") : "OFF",
                     cameraInitialized ? "ON" : "OFF",
                     ESP.getFreeHeap() / 1024,
                     millis() / 1000);
    }
    
    delay(10);
}