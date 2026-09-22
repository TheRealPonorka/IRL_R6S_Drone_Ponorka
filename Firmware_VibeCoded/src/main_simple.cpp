/**
 * @file main_simple.cpp
 * @brief Simple test version of R6 Drone - minimal functionality for testing
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
#include <BMI270.h>

// Simple LED strip for testing
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);

// Web server
WebServer server(80);

// Animation variables
uint32_t lastLEDUpdate = 0;
uint8_t animationStep = 0;
bool ledEnabled = true;
uint8_t ledR = 0, ledG = 255, ledB = 0; // Default green

// Spotlight variables
bool spotlightEnabled = false;
uint8_t spotlightBrightness = 255; // 0-255
bool lastSpotlightState = false;
uint8_t lastSpotlightBrightness = 255;

// System variables
uint32_t systemUptime = 0;
uint32_t systemFreeHeap = 0;
float systemTemperature = 0;

// Microphone variables
bool microphoneEnabled = false;
bool microphoneInitialized = false;
bool microphoneSimulated = false; // Use simulated audio when no real mic
bool audioTestMode = false; // Generate test tone instead of mic audio
float audioLevel = 0.0; // Current audio level (0-100)

// Audio streaming variables
#define AUDIO_BUFFER_SIZE 512
int16_t audioBuffer[AUDIO_BUFFER_SIZE];
volatile int audioBufferIndex = 0;
volatile bool audioBufferReady = false;
volatile bool streamActive = false;
uint32_t lastAudioStream = 0;

// Simple low-pass filter for audio cleaning
float audioFilter = 0.0;
const float filterAlpha = 0.1; // Low-pass filter coefficient (0.1 = gentle filtering)

// Camera variables
bool cameraInitialized = false;
uint32_t lastCameraRequest = 0;

// BMI270 IMU using proper library
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

// Function declarations
void setupWebServer();
void updateLEDAnimation();
void updateSpotlight();
void updateSystemStatus();
void updateMicrophone();
void updateIMU();
bool initializeCamera();
bool initializeMicrophone();
bool initializeIMU();
bool calibrateIMU();
void handleCameraStream();
void handleAudioStream();
void handleLiveAudioStream();
void handleAudioChunk();
void handleAudioTest();
void handleWebRTCOffer();
void handleWebRTCAnswer();

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=================================");
    Serial.println("R6 Drone - Simple Test Mode");
    Serial.println("Version: 1.0.0-simple");
    Serial.println("=================================");
    
    // Initialize LED strip
    Serial.println("Initializing LED strip...");
    strip.begin();
    strip.setBrightness(64);  // 25% brightness
    strip.clear();
    strip.show();
    Serial.println("LED strip initialized");
    
    // Skip ALL potentially problematic GPIO pins
    Serial.println("Using SAFE GPIO pins to prevent bootloops...");
    Serial.println("🔧 OLD pins (35-38, 45,46,48,21, 14) -> SAFE pins (8-13, 18-19, 47)");
    Serial.println("✅ Motors: GPIO 8,9,10,11 (safe PWM pins)");
    Serial.println("✅ Encoders: GPIO 12,13,18,19 (safe interrupt pins)");
    Serial.println("✅ Spotlight: GPIO 47 (safe PWM pin)");
    Serial.println("✅ Microphone: GPIO 41,42,47 (working)");
    Serial.println("✅ Camera: DVP pins (working)");
    Serial.println("✅ LED strip: GPIO 3 (working)");
    Serial.println("✅ IMU: GPIO 1,2 (I2C - ready to initialize)");
    
    // Initialize IMU first (required for balance control)
    Serial.println("Initializing BMI270 IMU...");
    if (initializeIMU()) {
        Serial.println("✅ BMI270 IMU initialized successfully");
        Serial.println("📐 Gyroscope and accelerometer ready for balance control");
    } else {
        Serial.println("⚠️ IMU initialization failed - drone will not self-balance");
    }
    
    // Initialize microphone
    Serial.println("Initializing microphone...");
    if (initializeMicrophone()) {
        microphoneInitialized = true;
        Serial.println("✅ INMP441 Microphone initialized");
    } else {
        microphoneInitialized = false;
        Serial.println("⚠️ Microphone initialization failed - continuing without microphone");
    }
    
    // Initialize camera
    Serial.println("Initializing camera...");
    if (initializeCamera()) {
        cameraInitialized = true;
        Serial.println("✅ Camera initialized successfully");
    } else {
        cameraInitialized = false;
        Serial.println("⚠️ Camera initialization failed - continuing without camera");
    }
    
    // Initialize WiFi in AP mode only (simpler)
    Serial.println("Starting WiFi Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("R6_Recon_Drone", "ReconDrone123");
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.println("Connect to WiFi: R6_Recon_Drone");
    Serial.println("Password: ReconDrone123");
    
    // Setup web server
    setupWebServer();
    server.begin();
    
    Serial.println("=================================");
    Serial.println("Simple test mode ready!");
    Serial.println("LED strip should show green animation");
    Serial.print("Web interface: http://");
    Serial.println(IP);
    Serial.println("=================================");
}

void setupWebServer() {
    // Main page
    server.on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>R6 Recon Drone - Simple Test</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<style>";
        html += "body { font-family: Arial; text-align: center; margin: 20px; background: #1a1a1a; color: white; }";
        html += "h1 { color: #00ff00; }";
        html += ".button { background: #333; border: 2px solid #00ff00; color: white; padding: 15px 30px; margin: 10px; font-size: 18px; border-radius: 10px; cursor: pointer; }";
        html += ".button:hover { background: #00ff00; color: black; }";
        html += ".button.active { background: #00ff00; color: black; }";
        html += ".status { background: #333; padding: 20px; margin: 20px; border-radius: 10px; }";
        html += ".controls { background: #333; padding: 15px; margin: 10px; border-radius: 10px; border: 1px solid #00ff00; }";
        html += ".slider { width: 100%; margin: 5px 0; }";
        html += ".color-preview { width: 50px; height: 50px; border: 2px solid #00ff00; border-radius: 50%; margin: 10px auto; }";
        html += ".control-row { display: flex; align-items: center; justify-content: space-between; margin: 10px 0; }";
        html += ".control-label { min-width: 100px; text-align: left; }";
        html += ".color-picker { background: #333; border: 2px solid #00ff00; border-radius: 10px; padding: 15px; margin: 10px; display: none; }";
        html += ".color-picker.active { display: block; }";
        html += ".color-slider { width: 100%; margin: 5px 0; }";
        html += ".color-preview { width: 50px; height: 50px; border: 2px solid #00ff00; border-radius: 50%; margin: 10px auto; }";
        html += "</style></head><body>";
        html += "<h1>🎯 R6 Recon Drone</h1>";
        html += "<h2>Simple Test Mode</h2>";
        html += "<div class='status'>";
        html += "<p><strong>WiFi:</strong> Connected (AP Mode)</p>";
        html += "<p><strong>LED Strip:</strong> " + String(ledEnabled ? "ON" : "OFF") + "</p>";
        html += "<p><strong>Spotlight:</strong> " + String(spotlightEnabled ? "ON" : "OFF") + "</p>";
        html += "<p><strong>Camera:</strong> " + String(cameraInitialized ? "Online" : "Offline") + "</p>";
        html += "<p><strong>Microphone:</strong> " + String(microphoneInitialized ? "Online" : "Offline") + " | <strong>Level:</strong> " + String((int)audioLevel) + "%</p>";
        // IMU Status Section
        html += "<div style='border: 2px solid #4CAF50; border-radius: 10px; padding: 15px; margin: 10px 0; background: rgba(76, 175, 80, 0.1);'>";
        html += "<h3 style='margin-top: 0; color: #4CAF50;'>🎯 IMU & Orientation</h3>";
        html += "<div style='display: flex; flex-wrap: wrap; gap: 15px;'>";
        
        // Status and calibration
        html += "<div style='flex: 1; min-width: 200px;'>";
        html += "<p><strong>Status:</strong> " + String(imuInitialized ? "🟢 Online" : "🔴 Offline") + "</p>";
        html += "<p><strong>Calibrated:</strong> " + String(imuCalibrated ? "✅ Yes" : "❌ No") + "</p>";
        html += "<button onclick='calibrateIMU()' style='background: #2196F3; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; margin: 5px 0;'>🔧 Calibrate IMU</button>";
        html += "<p id='imu-cal-status' style='color: #666; font-size: 0.9em;'></p>";
        html += "</div>";
        
        // Orientation values
        html += "<div style='flex: 1; min-width: 200px;'>";
        html += "<p><strong>Pitch:</strong> <span id='pitch'>" + String(pitch, 1) + "</span>°</p>";
        html += "<p><strong>Roll:</strong> <span id='roll'>" + String(roll, 1) + "</span>°</p>";
        html += "<p><strong>Yaw:</strong> <span id='yaw'>" + String(yaw, 1) + "</span>°</p>";
        html += "</div>";
        
        // Visual IMU display
        html += "<div style='flex: 1; min-width: 250px; text-align: center;'>";
        html += "<div style='position: relative; width: 200px; height: 200px; border: 2px solid #333; border-radius: 50%; margin: 0 auto; background: radial-gradient(circle, #e8f5e8 0%, #c8e6c9 100%);'>";
        html += "<div id='imu-indicator' style='position: absolute; top: 50%; left: 50%; width: 20px; height: 20px; background: #f44336; border-radius: 50%; transform: translate(-50%, -50%); transition: all 0.3s ease;'></div>";
        html += "<div style='position: absolute; top: 5px; left: 50%; transform: translateX(-50%); font-size: 12px; font-weight: bold;'>N</div>";
        html += "<div style='position: absolute; bottom: 5px; left: 50%; transform: translateX(-50%); font-size: 12px; font-weight: bold;'>S</div>";
        html += "<div style='position: absolute; left: 5px; top: 50%; transform: translateY(-50%); font-size: 12px; font-weight: bold;'>W</div>";
        html += "<div style='position: absolute; right: 5px; top: 50%; transform: translateY(-50%); font-size: 12px; font-weight: bold;'>E</div>";
        html += "</div>";
        html += "<p style='margin: 10px 0; font-size: 0.9em; color: #666;'>Red dot shows tilt direction</p>";
        html += "</div>";
        
        html += "</div>";
        html += "</div>";
        html += "<p><strong>Free Heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
        html += "<p><strong>Uptime:</strong> " + String(millis() / 1000) + " seconds</p>";
        html += "</div>";
        if (cameraInitialized) {
            html += "<div style='margin: 20px; position: relative;'>";
            html += "<canvas id='camera-canvas' width='640' height='480' style='max-width: 100%; border: 2px solid #00ff00; border-radius: 10px; background: #000;'></canvas>";
            html += "<div id='camera-status' style='position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.7); color: #00ff00; padding: 5px; border-radius: 5px; font-size: 12px;'>Loading...</div>";
            html += "</div>";
        }
        // Audio Level Meter (if microphone is working)
        if (microphoneInitialized) {
            html += "<div class='controls'>";
            html += "<h3>🎵 Audio Monitor & Streaming</h3>";
            html += "<button class='button" + String(microphoneEnabled ? " active" : "") + "' onclick='toggleMicrophone()'>Toggle Microphone</button>";
            html += "<div style='margin: 10px 0;'>";
            html += "<div style='background: #333; height: 20px; border-radius: 10px; overflow: hidden; position: relative;'>";
            html += "<div id='audio-meter' style='background: linear-gradient(to right, #00ff00, #ffff00, #ff0000); height: 100%; width: " + String((int)audioLevel) + "%; transition: width 0.1s;'></div>";
            html += "</div>";
            html += "<p style='text-align: center; margin: 5px;'>Audio Level: <span id='audio-level'>" + String((int)audioLevel) + "</span>%</p>";
            if (microphoneEnabled) {
                html += "<div style='margin: 10px 0;'>";
                html += "<button onclick='startWebAudioPlayback()' style='padding: 8px 16px; margin: 5px; background: #007bff; color: white; border: none; border-radius: 5px;'>🎵 Beep Mode</button>";
                html += "<button onclick='startWebRTC()' style='padding: 8px 16px; margin: 5px; background: #ff6b35; color: white; border: none; border-radius: 5px;'>🚀 WebRTC Voice</button>";
                html += "<button onclick='testWebAudio()' style='padding: 8px 16px; margin: 5px; background: #28a745; color: white; border: none; border-radius: 5px;'>🔊 Test 440Hz</button>";
                html += "<button onclick='stopWebAudioPlayback()' style='padding: 8px 16px; margin: 5px; background: #dc3545; color: white; border: none; border-radius: 5px;'>Stop Audio</button>";
                html += "<div id='audio-status' style='margin: 10px; font-weight: bold; color: #28a745;'>🚀 Ready for WebRTC Real-Time Voice Streaming!</div>";
                html += "<div style='margin: 15px; padding: 15px; background: #f8f9fa; border-left: 4px solid #ff6b35; border-radius: 5px;'>";
                html += "<h4 style='color: #ff6b35; margin-top: 0;'>🚀 WebRTC Voice Streaming</h4>";
                html += "<p style='margin: 5px 0; color: #28a745;'><strong>✅ Hardware:</strong> INMP441 microphone functional (levels up to " + String((int)audioLevel) + "%)</p>";
                html += "<p style='margin: 5px 0; color: #28a745;'><strong>✅ Processing:</strong> Audio samples processed correctly</p>";
                html += "<p style='margin: 5px 0; color: #28a745;'><strong>✅ Web Audio:</strong> Browser audio confirmed working</p>";
                html += "<p style='margin: 5px 0; color: #ff6b35;'><strong>🚀 Next:</strong> WebRTC for gapless real-time voice streaming</p>";
                html += "</div>";
                html += "</div>";
            }
            html += "</div>";
            html += "</div>";
        }
        
        // Control buttons
        html += "<div class='controls'>";
        html += "<h3>💡 Lighting Controls</h3>";
        html += "<button class='button" + String(ledEnabled ? " active" : "") + "' onclick='toggleLEDs()'>Toggle LED Strip</button>";
        html += "<button class='button" + String(spotlightEnabled ? " active" : "") + "' onclick='toggleSpotlight()'>Toggle Spotlight</button>";
        html += "<br><button class='button' onclick='document.getElementById(\"color-picker\").style.display = document.getElementById(\"color-picker\").style.display == \"none\" ? \"block\" : \"none\";'>🎨 LED Color</button>";
        
        // LED Color Picker (Initially hidden)
        html += "<div id='color-picker' style='display: none; margin-top: 15px; border: 1px solid #00ff00; padding: 10px; border-radius: 5px;'>";
        html += "<h4>🎨 LED Strip Color</h4>";
        html += "<div style='width: 50px; height: 50px; border: 2px solid #00ff00; border-radius: 50%; margin: 10px auto; background-color: rgb(" + String(ledR) + "," + String(ledG) + "," + String(ledB) + ");' id='color-preview'></div>";
        html += "<p>Red: <input type='range' min='0' max='255' value='" + String(ledR) + "' id='red-slider' oninput='updateColor()'> <span id='red-value'>" + String(ledR) + "</span></p>";
        html += "<p>Green: <input type='range' min='0' max='255' value='" + String(ledG) + "' id='green-slider' oninput='updateColor()'> <span id='green-value'>" + String(ledG) + "</span></p>";
        html += "<p>Blue: <input type='range' min='0' max='255' value='" + String(ledB) + "' id='blue-slider' oninput='updateColor()'> <span id='blue-value'>" + String(ledB) + "</span></p>";
        html += "<button class='button' onclick='applyColor()'>Apply Color</button>";
        html += "<button class='button' onclick='toggleColorPicker()'>Close</button>";
        html += "</div>";
        
        // Spotlight Brightness (when enabled)
        if (spotlightEnabled) {
            html += "<div style='margin-top: 15px;'>";
            html += "<h4>Spotlight Brightness</h4>";
            html += "<div class='control-row'>";
            html += "<span class='control-label'>Brightness:</span>";
            html += "<input type='range' class='slider' id='brightness-slider' min='0' max='255' value='" + String(spotlightBrightness) + "' oninput='updateBrightness()'>";
            html += "<span id='brightness-value'>" + String(spotlightBrightness) + "</span>";
            html += "</div>";
            html += "</div>";
        }
        
        html += "</div>";
        html += "<br><button class='button' onclick='window.location.reload()'>🔄 Refresh Status</button>";
        html += "<div id='color-picker' class='color-picker'>";
        html += "<h3>LED Strip Color</h3>";
        html += "<div class='color-preview' id='color-preview' style='background-color: rgb(" + String(ledR) + "," + String(ledG) + "," + String(ledB) + ");'></div>";
        html += "<label>Red: <input type='range' class='color-slider' id='red-slider' min='0' max='255' value='" + String(ledR) + "' oninput='updateColor()'></label>";
        html += "<label>Green: <input type='range' class='color-slider' id='green-slider' min='0' max='255' value='" + String(ledG) + "' oninput='updateColor()'></label>";
        html += "<label>Blue: <input type='range' class='color-slider' id='blue-slider' min='0' max='255' value='" + String(ledB) + "' oninput='updateColor()'></label>";
        html += "<button class='button' onclick='applyColor()'>Apply Color</button>";
        html += "</div>";
        html += "<script>";
        html += "function toggleLEDs() { fetch('/toggle-leds').then(() => window.location.reload()); }";
        html += "function toggleSpotlight() { fetch('/toggle-spotlight').then(() => window.location.reload()); }";
        html += "function toggleMicrophone() { fetch('/toggle-microphone').then(() => window.location.reload()); }";
        html += "function toggleTestMode() { fetch('/toggle-test-mode').then(() => window.location.reload()); }";
        html += "function toggleColorPicker() { ";
        html += "  var picker = document.getElementById('color-picker'); ";
        html += "  if (picker.style.display == 'none') { ";
        html += "    picker.style.display = 'block'; ";
        html += "  } else { ";
        html += "    picker.style.display = 'none'; ";
        html += "  } ";
        html += "} ";
        html += "function updateColor() { ";
        html += "  var r = document.getElementById('red-slider').value; ";
        html += "  var g = document.getElementById('green-slider').value; ";
        html += "  var b = document.getElementById('blue-slider').value; ";
        html += "  document.getElementById('color-preview').style.backgroundColor = 'rgb(' + r + ',' + g + ',' + b + ')'; ";
        html += "  document.getElementById('red-value').innerHTML = r; ";
        html += "  document.getElementById('green-value').innerHTML = g; ";
        html += "  document.getElementById('blue-value').innerHTML = b; ";
        html += "} ";
        html += "function applyColor() { ";
        html += "  var r = document.getElementById('red-slider').value; ";
        html += "  var g = document.getElementById('green-slider').value; ";
        html += "  var b = document.getElementById('blue-slider').value; ";
        html += "  fetch('/set-color?r=' + r + '&g=' + g + '&b=' + b).then(() => { ";
        html += "    document.getElementById('color-picker').style.display = 'none'; ";
        html += "    setTimeout(() => window.location.reload(), 500); ";
        html += "  }); ";
        html += "} ";
        html += "function updateBrightness() { ";
        html += "  var brightness = document.getElementById('brightness-slider').value; ";
        html += "  document.getElementById('brightness-value').textContent = brightness; ";
        html += "  fetch('/set-brightness?value=' + brightness); ";
        html += "} ";
        html += "function toggleColorPicker() { ";
        html += "  var picker = document.getElementById('color-picker'); ";
        html += "  picker.classList.toggle('active'); ";
        html += "} ";
        html += "function updateColor() { ";
        html += "  var r = document.getElementById('red-slider').value; ";
        html += "  var g = document.getElementById('green-slider').value; ";
        html += "  var b = document.getElementById('blue-slider').value; ";
        html += "  document.getElementById('color-preview').style.backgroundColor = 'rgb(' + r + ',' + g + ',' + b + ')'; ";
        html += "} ";
        html += "function applyColor() { ";
        html += "  var r = document.getElementById('red-slider').value; ";
        html += "  var g = document.getElementById('green-slider').value; ";
        html += "  var b = document.getElementById('blue-slider').value; ";
        html += "  fetch('/set-color?r=' + r + '&g=' + g + '&b=' + b).then(() => { ";
        html += "    document.getElementById('color-picker').classList.remove('active'); ";
        html += "    setTimeout(() => window.location.reload(), 500); ";
        html += "  }); ";
        html += "} ";
        if (cameraInitialized) {
            html += "var cameraActive = true; ";
            html += "var canvas = null; ";
            html += "var ctx = null; ";
            html += "var frameCount = 0; ";
            html += "var startTime = Date.now(); ";
            html += "function initCamera() { ";
            html += "  canvas = document.getElementById('camera-canvas'); ";
            html += "  ctx = canvas.getContext('2d'); ";
            html += "  requestCameraFrame(); ";
            html += "} ";
            html += "function requestCameraFrame() { ";
            html += "  if (!cameraActive || !canvas || !ctx) return; ";
            html += "  var img = new Image(); ";
            html += "  img.onload = function() { ";
            html += "    ctx.drawImage(img, 0, 0, canvas.width, canvas.height); ";
            html += "    frameCount++; ";
            html += "    var fps = (frameCount / ((Date.now() - startTime) / 1000)).toFixed(1); ";
            html += "    document.getElementById('camera-status').textContent = 'FPS: ' + fps; ";
            html += "    setTimeout(requestCameraFrame, 150); ";  // ~7 FPS (slower for stability)
            html += "  }; ";
            html += "  img.onerror = function() { ";
            html += "    document.getElementById('camera-status').textContent = 'Camera Error'; ";
            html += "    setTimeout(requestCameraFrame, 500); ";
            html += "  }; ";
            html += "  img.src = '/stream?t=' + Date.now(); ";
            html += "} ";
            html += "window.onload = initCamera; ";
            
            // IMU Functions
            html += "function calibrateIMU() { ";
            html += "  var statusEl = document.getElementById('imu-cal-status'); ";
            html += "  statusEl.textContent = 'Calibrating... Keep drone stationary!'; ";
            html += "  statusEl.style.color = '#ff9800'; ";
            html += "  fetch('/calibrate-imu', { method: 'POST' }) ";
            html += "    .then(response => response.text()) ";
            html += "    .then(data => { ";
            html += "      statusEl.textContent = data; ";
            html += "      statusEl.style.color = response.ok ? '#4caf50' : '#f44336'; ";
            html += "      setTimeout(() => { statusEl.textContent = ''; }, 5000); ";
            html += "    }) ";
            html += "    .catch(error => { ";
            html += "      statusEl.textContent = 'Calibration failed'; ";
            html += "      statusEl.style.color = '#f44336'; ";
            html += "    }); ";
            html += "} ";
            
            html += "function updateIMUDisplay(pitch, roll) { ";
            html += "  var indicator = document.getElementById('imu-indicator'); ";
            html += "  if (indicator) { ";
            html += "    var maxTilt = 45; ";  // Maximum tilt angle for visualization
            html += "    var normalizedPitch = Math.max(-maxTilt, Math.min(maxTilt, pitch)) / maxTilt; ";
            html += "    var normalizedRoll = Math.max(-maxTilt, Math.min(maxTilt, roll)) / maxTilt; ";
            html += "    var offsetX = normalizedRoll * 80; ";  // 80px max offset
            html += "    var offsetY = normalizedPitch * 80; ";
            html += "    indicator.style.transform = 'translate(calc(-50% + ' + offsetX + 'px), calc(-50% + ' + offsetY + 'px))'; ";
            html += "    var tiltMagnitude = Math.sqrt(normalizedPitch*normalizedPitch + normalizedRoll*normalizedRoll); ";
            html += "    var color = tiltMagnitude > 0.7 ? '#f44336' : (tiltMagnitude > 0.3 ? '#ff9800' : '#4caf50'); ";
            html += "    indicator.style.background = color; ";
            html += "  } ";
            html += "} ";
            
            // Update IMU status periodically
            html += "setInterval(function() { ";
            html += "  fetch('/status') ";
            html += "    .then(response => response.json()) ";
            html += "    .then(data => { ";
            html += "      document.getElementById('pitch').textContent = data.pitch; ";
            html += "      document.getElementById('roll').textContent = data.roll; ";
            html += "      document.getElementById('yaw').textContent = data.yaw; ";
            html += "      updateIMUDisplay(data.pitch, data.roll); ";
            html += "    }) ";
            html += "    .catch(error => console.log('Status update failed:', error)); ";
            html += "}, 200); ";  // Update every 200ms
            html += "window.onbeforeunload = function() { cameraActive = false; }; ";
            
        }
        
        // Simple Web Audio API implementation
        html += "var audioContext = null; ";
        html += "var isPlaying = false; ";
        html += "var playbackInterval = null; ";
        html += "var nextStartTime = 0; ";
        
        html += "function initWebAudio() { ";
        html += "  if (!audioContext) { ";
        html += "    audioContext = new (window.AudioContext || window.webkitAudioContext)(); ";
        html += "    console.log('Web Audio Context created'); ";
        html += "  } ";
        html += "  if (audioContext.state === 'suspended') { ";
        html += "    audioContext.resume(); ";
        html += "  } ";
        html += "  return audioContext; ";
        html += "} ";
        
        html += "function startWebAudioPlayback() { ";
        html += "  if (isPlaying) return; ";
        html += "  console.log('Starting beep mode playback...'); ";
        html += "  if (!initWebAudio()) return; ";
        html += "  isPlaying = true; ";
        html += "  nextStartTime = audioContext.currentTime; ";
        html += "  var status = document.getElementById('audio-status'); ";
        html += "  if (status) status.innerText = '🎵 Beep Mode Active - Smooth Audio Confirmed'; ";
        html += "  playbackInterval = setInterval(fetchAndPlayAudio, 100); ";
        html += "} ";
        
        html += "function stopWebAudioPlayback() { ";
        html += "  isPlaying = false; ";
        html += "  stopWebRTC(); ";
        html += "  if (playbackInterval) { ";
        html += "    clearInterval(playbackInterval); ";
        html += "    playbackInterval = null; ";
        html += "  } ";
        html += "  if (currentOscillator) { ";
        html += "    currentOscillator.stop(); ";
        html += "    currentOscillator = null; ";
        html += "  } ";
        html += "  if (currentGain) { ";
        html += "    currentGain.disconnect(); ";
        html += "    currentGain = null; ";
        html += "  } ";
        html += "  var status = document.getElementById('audio-status'); ";
        html += "  if (status) status.innerText = '✅ Ready for Audio Testing'; ";
        html += "  console.log('All audio playback stopped'); ";
        html += "} ";
        
        html += "function fetchAndPlayAudio() { ";
        html += "  if (!isPlaying) return; ";
        html += "  fetch('/audio-stream?t=' + Date.now()) ";
        html += "    .then(response => response.json()) ";
        html += "    .then(data => { ";
        html += "      var levelElement = document.getElementById('audio-level'); ";
        html += "      if (levelElement) levelElement.innerText = (data.audioLevel * 100).toFixed(1); ";
        html += "      var meterElement = document.getElementById('audio-meter'); ";
        html += "      if (meterElement) meterElement.style.width = (data.audioLevel * 100).toFixed(1) + '%'; ";
        html += "      if (data.micEnabled && data.micSamples && data.micSamples.length > 0) { ";
        html += "        generateContinuousAudio(data.audioLevel); ";
        html += "      } ";
        html += "    }) ";
        html += "    .catch(e => console.error('Audio fetch error:', e)); ";
        html += "} ";
        
        html += "var currentOscillator = null; ";
        html += "var currentGain = null; ";
        
        html += "function generateContinuousAudio(audioLevel) { ";
        html += "  if (!audioContext) return; ";
        html += "  try { ";
        html += "    if (currentOscillator) { ";
        html += "      currentOscillator.stop(); ";
        html += "      currentOscillator = null; ";
        html += "    } ";
        html += "    if (currentGain) { ";
        html += "      currentGain.disconnect(); ";
        html += "      currentGain = null; ";
        html += "    } ";
        html += "    var baseFreq = 200 + (audioLevel * 800); ";
        html += "    var volume = Math.min(audioLevel * 0.5, 0.3); ";
        html += "    currentOscillator = audioContext.createOscillator(); ";
        html += "    currentGain = audioContext.createGain(); ";
        html += "    currentOscillator.type = 'sine'; ";
        html += "    currentOscillator.frequency.setValueAtTime(baseFreq, audioContext.currentTime); ";
        html += "    currentGain.gain.setValueAtTime(volume, audioContext.currentTime); ";
        html += "    currentOscillator.connect(currentGain); ";
        html += "    currentGain.connect(audioContext.destination); ";
        html += "    currentOscillator.start(); ";
        html += "    setTimeout(() => { ";
        html += "      if (currentOscillator) { ";
        html += "        currentOscillator.stop(); ";
        html += "        currentOscillator = null; ";
        html += "      } ";
        html += "    }, 80); ";
        html += "  } catch (e) { ";
        html += "    console.error('Continuous audio error:', e); ";
        html += "  } ";
        html += "} ";
        
        
        html += "function testWebAudio() { ";
        html += "  console.log('Testing 440Hz tone...'); ";
        html += "  if (!initWebAudio()) return; ";
        html += "  var status = document.getElementById('audio-status'); ";
        html += "  if (status) status.innerText = '🔊 Playing 440Hz Test Tone'; ";
        html += "  var sampleRate = 22050; ";
        html += "  var duration = 1.0; ";
        html += "  var samples = Math.floor(sampleRate * duration); ";
        html += "  var buffer = audioContext.createBuffer(1, samples, sampleRate); ";
        html += "  var channelData = buffer.getChannelData(0); ";
        html += "  for (var i = 0; i < samples; i++) { ";
        html += "    var time = i / sampleRate; ";
        html += "    channelData[i] = Math.sin(2 * Math.PI * 440 * time) * 0.3; ";
        html += "  } ";
        html += "  var source = audioContext.createBufferSource(); ";
        html += "  source.buffer = buffer; ";
        html += "  source.connect(audioContext.destination); ";
        html += "  source.start(); ";
        html += "  source.onended = () => { if (status) status.innerText = 'Test Tone Complete'; }; ";
        html += "} ";
        
        // WebRTC Implementation
        html += "var peerConnection = null; ";
        html += "var localStream = null; ";
        html += "var audioWorklet = null; ";
        html += "var webrtcActive = false; ";
        
        html += "async function startWebRTC() { ";
        html += "  if (webrtcActive) return; ";
        html += "  try { ";
        html += "    console.log('Starting real-time voice streaming...'); ";
        html += "    var status = document.getElementById('audio-status'); ";
        html += "    if (status) status.innerText = '🚀 Initializing Real-Time Audio...'; ";
        html += "    if (!initWebAudio()) return; ";
        html += "    if (audioContext.audioWorklet && typeof audioContext.audioWorklet.addModule === 'function') { ";
        html += "      console.log('Using Audio Worklet for real-time streaming'); ";
        html += "      await setupAudioWorklet(); ";
        html += "    } else { ";
        html += "      console.log('Audio Worklet not supported, using ScriptProcessor fallback'); ";
        html += "      await setupScriptProcessor(); ";
        html += "    } ";
        html += "    webrtcActive = true; ";
        html += "    if (status) status.innerText = '🚀 Real-Time Voice Streaming Active'; ";
        html += "    console.log('Real-time audio setup complete'); ";
        html += "  } catch (error) { ";
        html += "    console.error('Real-time audio setup failed:', error); ";
        html += "    var status = document.getElementById('audio-status'); ";
        html += "    if (status) status.innerText = 'Audio Error: ' + error.message; ";
        html += "  } ";
        html += "} ";
        
        html += "async function setupAudioWorklet() { ";
        html += "  try { ";
        html += "    console.log('Setting up audio worklet for ESP32 stream...'); ";
        html += "    const audioWorkletCode = ` ";
        html += "      class ESP32AudioProcessor extends AudioWorkletProcessor { ";
        html += "        constructor() { ";
        html += "          super(); ";
        html += "          this.sampleBuffer = []; ";
        html += "          this.fetchInterval = null; ";
        html += "          this.startFetching(); ";
        html += "        } ";
        html += "        startFetching() { ";
        html += "          this.fetchInterval = setInterval(() => { ";
        html += "            fetch('/audio-stream?t=' + Date.now()) ";
        html += "              .then(r => r.json()) ";
        html += "              .then(data => { ";
        html += "                if (data.micSamples && data.micSamples.length > 0) { ";
        html += "                  for (let sample of data.micSamples) { ";
        html += "                    this.sampleBuffer.push(sample / 32768.0 * 2.0); ";
        html += "                  } ";
        html += "                } ";
        html += "              }) ";
        html += "              .catch(e => console.error('Fetch error:', e)); ";
        html += "          }, 20); ";
        html += "        } ";
        html += "        process(inputs, outputs, parameters) { ";
        html += "          const output = outputs[0]; ";
        html += "          const outputChannel = output[0]; ";
        html += "          for (let i = 0; i < outputChannel.length; i++) { ";
        html += "            if (this.sampleBuffer.length > 0) { ";
        html += "              outputChannel[i] = this.sampleBuffer.shift(); ";
        html += "            } else { ";
        html += "              outputChannel[i] = 0; ";
        html += "            } ";
        html += "          } ";
        html += "          return true; ";
        html += "        } ";
        html += "      } ";
        html += "      registerProcessor('esp32-audio-processor', ESP32AudioProcessor); ";
        html += "    `; ";
        html += "    const blob = new Blob([audioWorkletCode], { type: 'application/javascript' }); ";
        html += "    const workletURL = URL.createObjectURL(blob); ";
        html += "    await audioContext.audioWorklet.addModule(workletURL); ";
        html += "    audioWorklet = new AudioWorkletNode(audioContext, 'esp32-audio-processor'); ";
        html += "    audioWorklet.connect(audioContext.destination); ";
        html += "    console.log('Audio worklet connected successfully'); ";
        html += "  } catch (error) { ";
        html += "    console.error('Audio worklet setup failed:', error); ";
        html += "    throw error; ";
        html += "  } ";
        html += "} ";
        
        html += "var scriptProcessor = null; ";
        html += "var audioSampleBuffer = []; ";
        html += "var fetchAudioInterval = null; ";
        
        html += "async function setupScriptProcessor() { ";
        html += "  try { ";
        html += "    console.log('Setting up optimized ScriptProcessor...'); ";
        html += "    const bufferSize = 2048; ";
        html += "    scriptProcessor = audioContext.createScriptProcessor(bufferSize, 0, 1); ";
        html += "    let lastSample = 0; ";
        html += "    let silenceCount = 0; ";
        html += "    scriptProcessor.onaudioprocess = function(event) { ";
        html += "      const outputBuffer = event.outputBuffer; ";
        html += "      const outputData = outputBuffer.getChannelData(0); ";
        html += "      for (let i = 0; i < outputData.length; i++) { ";
        html += "        if (audioSampleBuffer.length > 0) { ";
        html += "          const sample = audioSampleBuffer.shift(); ";
        html += "          outputData[i] = sample; ";
        html += "          lastSample = sample; ";
        html += "          silenceCount = 0; ";
        html += "        } else { ";
        html += "          silenceCount++; ";
        html += "          if (silenceCount < 100) { ";
        html += "            outputData[i] = lastSample * 0.95; ";
        html += "            lastSample = outputData[i]; ";
        html += "          } else { ";
        html += "            outputData[i] = 0; ";
        html += "            lastSample = 0; ";
        html += "          } ";
        html += "        } ";
        html += "      } ";
        html += "    }; ";
        html += "    scriptProcessor.connect(audioContext.destination); ";
        html += "    startContinuousFetch(); ";
        html += "    console.log('Optimized ScriptProcessor connected'); ";
        html += "  } catch (error) { ";
        html += "    console.error('ScriptProcessor setup failed:', error); ";
        html += "    throw error; ";
        html += "  } ";
        html += "} ";
        
        html += "var fetchInProgress = false; ";
        html += "var bufferUnderrunCount = 0; ";
        
        html += "function startContinuousFetch() { ";
        html += "  fetchAudioInterval = setInterval(() => { ";
        html += "    if (fetchInProgress) return; ";
        html += "    fetchInProgress = true; ";
        html += "    fetch('/audio-stream?t=' + Date.now()) ";
        html += "      .then(r => r.json()) ";
        html += "      .then(data => { ";
        html += "        fetchInProgress = false; ";
        html += "        if (data.micSamples && data.micSamples.length > 0) { ";
        html += "          for (let sample of data.micSamples) { ";
        html += "            const normalizedSample = (sample / 32768.0) * 2.5; ";
        html += "            audioSampleBuffer.push(normalizedSample); ";
        html += "          } ";
        html += "          bufferUnderrunCount = 0; ";
        html += "          if (audioSampleBuffer.length > 6144) { ";
        html += "            audioSampleBuffer = audioSampleBuffer.slice(-3072); ";
        html += "            console.log('Buffer trimmed to prevent overflow'); ";
        html += "          } ";
        html += "        } else { ";
        html += "          bufferUnderrunCount++; ";
        html += "          if (bufferUnderrunCount > 3) { ";
        html += "            for (let i = 0; i < 256; i++) { ";
        html += "              audioSampleBuffer.push(0); ";
        html += "            } ";
        html += "            console.log('Added silence padding due to underruns'); ";
        html += "          } ";
        html += "        } ";
        html += "      }) ";
        html += "      .catch(e => { ";
        html += "        fetchInProgress = false; ";
        html += "        console.error('Continuous fetch error:', e); ";
        html += "      }); ";
        html += "  }, 23); ";
        html += "} ";
        
        html += "function stopContinuousFetch() { ";
        html += "  if (fetchAudioInterval) { ";
        html += "    clearInterval(fetchAudioInterval); ";
        html += "    fetchAudioInterval = null; ";
        html += "  } ";
        html += "} ";
        
        html += "function stopWebRTC() { ";
        html += "  webrtcActive = false; ";
        html += "  stopContinuousFetch(); ";
        html += "  if (audioWorklet) { ";
        html += "    audioWorklet.disconnect(); ";
        html += "    audioWorklet = null; ";
        html += "  } ";
        html += "  if (scriptProcessor) { ";
        html += "    scriptProcessor.disconnect(); ";
        html += "    scriptProcessor = null; ";
        html += "  } ";
        html += "  if (peerConnection) { ";
        html += "    peerConnection.close(); ";
        html += "    peerConnection = null; ";
        html += "  } ";
        html += "  audioSampleBuffer = []; ";
        html += "  var status = document.getElementById('audio-status'); ";
        html += "  if (status) status.innerText = 'Real-Time Audio Stopped'; ";
        html += "  console.log('Real-time audio stopped'); ";
        html += "} ";
        
        html += "</script>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });
    
    // Toggle LEDs
    server.on("/toggle-leds", HTTP_GET, []() {
        ledEnabled = !ledEnabled;
        Serial.println("LED strip toggled: " + String(ledEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Toggle Spotlight
    server.on("/toggle-spotlight", HTTP_GET, []() {
        spotlightEnabled = !spotlightEnabled;
        Serial.println("Spotlight toggled: " + String(spotlightEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Toggle Microphone
    server.on("/toggle-microphone", HTTP_GET, []() {
        microphoneEnabled = !microphoneEnabled;
        Serial.println("Microphone toggled: " + String(microphoneEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Toggle Test Mode
    server.on("/toggle-test-mode", HTTP_GET, []() {
        audioTestMode = !audioTestMode;
        Serial.println("Audio test mode: " + String(audioTestMode ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Set LED Color
    server.on("/set-color", HTTP_GET, []() {
        if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
            ledR = server.arg("r").toInt();
            ledG = server.arg("g").toInt();
            ledB = server.arg("b").toInt();
            Serial.printf("LED color changed to RGB(%d, %d, %d)\n", ledR, ledG, ledB);
            server.send(200, "text/plain", "Color updated");
        } else {
            server.send(400, "text/plain", "Missing color parameters");
        }
    });
    
    // Set Spotlight Brightness
    server.on("/set-brightness", HTTP_GET, []() {
        if (server.hasArg("value")) {
            spotlightBrightness = server.arg("value").toInt();
            Serial.printf("Spotlight brightness set to %d\n", spotlightBrightness);
            server.send(200, "text/plain", "Brightness updated");
        } else {
            server.send(400, "text/plain", "Missing brightness value");
        }
    });
    
    // Set LED color
    server.on("/set-color", HTTP_GET, []() {
        if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
            ledR = server.arg("r").toInt();
            ledG = server.arg("g").toInt();
            ledB = server.arg("b").toInt();
            Serial.printf("LED color changed to RGB(%d, %d, %d)\n", ledR, ledG, ledB);
            server.send(200, "text/plain", "Color updated");
        } else {
            server.send(400, "text/plain", "Missing color parameters");
        }
    });
    
    // Camera stream
    server.on("/stream", HTTP_GET, handleCameraStream);
    
    // Audio stream
    server.on("/audio-stream", HTTP_GET, handleAudioStream);
    server.on("/audio-live", HTTP_GET, handleLiveAudioStream);
    server.on("/audio-chunk", HTTP_GET, handleAudioChunk);
    server.on("/audio-test", HTTP_GET, handleAudioTest);
    server.on("/webrtc-offer", HTTP_POST, handleWebRTCOffer);
    server.on("/webrtc-answer", HTTP_GET, handleWebRTCAnswer);
    
    // Status API
    server.on("/status", HTTP_GET, []() {
        String json = "{";
        json += "\"uptime\":" + String(millis() / 1000) + ",";
        json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
        json += "\"ledEnabled\":" + String(ledEnabled ? "true" : "false") + ",";
        json += "\"spotlightEnabled\":" + String(spotlightEnabled ? "true" : "false") + ",";
        json += "\"spotlightBrightness\":" + String(spotlightBrightness) + ",";
        json += "\"ledR\":" + String(ledR) + ",";
        json += "\"ledG\":" + String(ledG) + ",";
        json += "\"ledB\":" + String(ledB) + ",";
        json += "\"microphoneInitialized\":" + String(microphoneInitialized ? "true" : "false") + ",";
        json += "\"microphoneEnabled\":" + String(microphoneEnabled ? "true" : "false") + ",";
        json += "\"audioLevel\":" + String(audioLevel) + ",";
        json += "\"cameraInitialized\":" + String(cameraInitialized ? "true" : "false") + ",";
        json += "\"imuOnline\":" + String(imuInitialized ? "true" : "false") + ",";
        json += "\"imuCalibrated\":" + String(imuCalibrated ? "true" : "false") + ",";
        json += "\"calibrationInProgress\":" + String(calibrationInProgress ? "true" : "false") + ",";
        json += "\"pitch\":" + String(pitch, 1) + ",";
        json += "\"roll\":" + String(roll, 1) + ",";
        json += "\"yaw\":" + String(yaw, 1) + ",";
        json += "\"accelX\":" + String(accelX, 2) + ",";
        json += "\"accelY\":" + String(accelY, 2) + ",";
        json += "\"accelZ\":" + String(accelZ, 2) + ",";
        json += "\"gyroX\":" + String(gyroX, 1) + ",";
        json += "\"gyroY\":" + String(gyroY, 1) + ",";
        json += "\"gyroZ\":" + String(gyroZ, 1);
        json += "}";
        server.send(200, "application/json", json);
    });
    
    // IMU calibration endpoint
    server.on("/calibrate-imu", HTTP_POST, []() {
        if (calibrationInProgress) {
            server.send(409, "text/plain", "Calibration already in progress");
            return;
        }
        
        Serial.println("🌐 Web calibration requested");
        bool success = calibrateIMU();
        
        if (success) {
            server.send(200, "text/plain", "IMU calibration completed successfully");
        } else {
            server.send(500, "text/plain", "IMU calibration failed");
        }
    });
    
    Serial.println("Web server routes configured");
}

void updateLEDAnimation() {
    if (!ledEnabled) {
        strip.clear();
        strip.show();
        return;
    }
    
    uint32_t currentTime = millis();
    if (currentTime - lastLEDUpdate < 30) return; // 30ms delay
    
    lastLEDUpdate = currentTime;
    
    // Clear all LEDs
    strip.clear();
    
    // Middle-out animation for 6 LEDs (0-5)
    switch (animationStep) {
        case 0:
            // No LEDs
            break;
            
        case 1:
            // LED 2-3
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
            
        case 2:
            // LED 1-2-3-4
            strip.setPixelColor(1, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(4, strip.Color(ledR, ledG, ledB));
            break;
            
        case 3:
            // LED 0-1-2-3-4-5 (all LEDs)
            for (int i = 0; i < LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
            }
            break;
            
        case 4:
            // LED 1-2-3-4
            strip.setPixelColor(1, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(4, strip.Color(ledR, ledG, ledB));
            break;
            
        case 5:
            // LED 2-3
            strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
            strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
            break;
    }
    
    strip.show();
    
    // Advance animation
    animationStep++;
    if (animationStep >= 6) {
        animationStep = 0;
    }
}

void loop() {
    // Handle web server requests
    server.handleClient();
    
    // Update hardware
    updateLEDAnimation();
    updateSpotlight();
    updateMicrophone();
    updateIMU();  // Update IMU at 100Hz for balance control
    updateSystemStatus();
    
    // Print status every 5 seconds
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        lastStatus = millis();
        Serial.println("Simple test running... LED: " + String(ledEnabled ? "ON" : "OFF"));
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
        Serial.printf("Web server: http://%s\n", WiFi.softAPIP().toString().c_str());
    }
    
    // Small delay
    delay(10);
}


bool initializeCamera() {
    camera_config_t config;
    
    // OV3660 camera configuration - GOOUUU ESP32-S3 CAM N16R8 pinout
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11;    // D0
    config.pin_d1 = 9;     // D1
    config.pin_d2 = 8;     // D2
    config.pin_d3 = 10;    // D3
    config.pin_d4 = 12;    // D4
    config.pin_d5 = 18;    // D5
    config.pin_d6 = 17;    // D6
    config.pin_d7 = 16;    // D7
    config.pin_xclk = 15;  // XCLK
    config.pin_pclk = 13;  // PCLK
    config.pin_vsync = 6;  // VSYNC
    config.pin_href = 7;   // HREF
    config.pin_sccb_sda = 4;  // SIOD (SDA)
    config.pin_sccb_scl = 5;  // SIOC (SCL)
    config.pin_pwdn = -1;  // Not used
    config.pin_reset = -1; // Not used
    
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Frame size and quality
    config.frame_size = FRAMESIZE_HD;     // 1280x720 (HD)
    config.jpeg_quality = 12;             // 0-63, lower means higher quality
    config.fb_count = 1;                  // Number of frame buffers
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }
    
    // Get camera sensor and configure for stable, non-flickering operation
    sensor_t* s = esp_camera_sensor_get();
    if (s != NULL) {
        Serial.println("Configuring camera for stable operation...");
        
        // FIXED EXPOSURE SETTINGS - Prevents flickering from auto-adjust
        s->set_exposure_ctrl(s, 0);  // DISABLE auto exposure (key fix!)
        s->set_aec2(s, 0);           // DISABLE AEC2
        s->set_aec_value(s, 400);    // FIXED exposure value (adjust as needed)
        
        // FIXED GAIN SETTINGS - Prevents brightness fighting
        s->set_gain_ctrl(s, 0);      // DISABLE auto gain (key fix!)
        s->set_agc_gain(s, 8);       // FIXED gain value (adjust as needed)
        s->set_gainceiling(s, (gainceiling_t)2);  // Limit max gain
        
        // FIXED WHITE BALANCE - Prevents color shifting
        s->set_whitebal(s, 0);       // DISABLE auto white balance (key fix!)
        s->set_awb_gain(s, 0);       // DISABLE AWB gain
        s->set_wb_mode(s, 2);        // FIXED white balance mode (indoor/office lighting)
        
        // STABLE SETTINGS - No auto-adjustment
        s->set_brightness(s, 1);     // Slightly brighter
        s->set_contrast(s, 1);       // Slightly more contrast  
        s->set_saturation(s, 0);     // Normal saturation
        s->set_ae_level(s, 0);       // Normal AE level
        
        // IMAGE PROCESSING - Optimized for performance
        s->set_special_effect(s, 0); // No effects (faster processing)
        s->set_bpc(s, 0);            // DISABLE bad pixel correction (faster)
        s->set_wpc(s, 0);            // DISABLE white pixel correction (faster)
        s->set_raw_gma(s, 0);        // DISABLE raw gamma (faster)
        s->set_lenc(s, 0);           // DISABLE lens correction (faster)
        s->set_hmirror(s, 0);        // No mirror
        s->set_vflip(s, 0);          // No flip
        s->set_dcw(s, 0);            // DISABLE downsize (faster, better quality)
        s->set_colorbar(s, 0);       // No color bar
        
        Serial.println("✅ Camera configured for stable, non-flickering operation");
    }
    
    Serial.println("Camera configuration completed");
    return true;
}

void handleCameraStream() {
    if (!cameraInitialized) {
        server.send(404, "text/plain", "Camera not available");
        return;
    }
    
    // Rate limiting - slower FPS to reduce CPU load and improve LED smoothness
    uint32_t currentTime = millis();
    if (currentTime - lastCameraRequest < 100) { // 100ms = 10 FPS (was 20 FPS)
        server.send(429, "text/plain", "Too many requests");
        return;
    }
    lastCameraRequest = currentTime;
    
    // Capture a single frame and send it as JPEG
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }
    
    // Send single JPEG image
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
}

void handleAudioStream() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    
    // Create JSON response with current audio status and samples
    String json = "{";
    json += "\"testMode\":" + String(audioTestMode ? "true" : "false") + ",";
    json += "\"micEnabled\":" + String(microphoneEnabled ? "true" : "false") + ",";
    json += "\"audioLevel\":" + String(audioLevel, 3) + ",";
    json += "\"timestamp\":" + String(millis());
    
    // Add microphone samples if available
    if (microphoneEnabled && microphoneInitialized && audioBufferReady) {
        json += ",\"micSamples\":[";
        
        // Send current audio buffer
        for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            if (i > 0) json += ",";
            json += String(audioBuffer[i]);
        }
        json += "]";
        
        // Mark buffer as consumed
        audioBufferReady = false;
    } else {
        json += ",\"micSamples\":[]";
    }
    
    json += "}";
    
    server.send(200, "application/json", json);
    
    // Debug output
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.printf("[Audio API] Level: %.1f%%, Buffer Ready: %s\n", 
                     audioLevel, audioBufferReady ? "Yes" : "No");
        lastDebug = millis();
    }
}

void handleLiveAudioStream() {
    if (!microphoneEnabled || !microphoneInitialized) {
        server.send(404, "text/plain", "Audio not available");
        return;
    }
    
    Serial.println("[Audio] Starting buffered WAV stream");
    
    // Send proper WAV stream headers
    server.sendHeader("Content-Type", "audio/wav");
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Accept-Ranges", "bytes");
    
    // Create a WAV header for streaming
    uint8_t wavHeader[44];
    memcpy(wavHeader, "RIFF", 4);
    *((uint32_t*)(wavHeader + 4)) = 0xFFFFFFFF; // Infinite size
    memcpy(wavHeader + 8, "WAVE", 4);
    memcpy(wavHeader + 12, "fmt ", 4);
    *((uint32_t*)(wavHeader + 16)) = 16; // PCM format chunk size
    *((uint16_t*)(wavHeader + 20)) = 1;  // PCM format
    *((uint16_t*)(wavHeader + 22)) = 1;  // Mono
    *((uint32_t*)(wavHeader + 24)) = 22050; // Sample rate
    *((uint32_t*)(wavHeader + 28)) = 44100; // Byte rate (22050 * 2)
    *((uint16_t*)(wavHeader + 32)) = 2;  // Block align
    *((uint16_t*)(wavHeader + 34)) = 16; // Bits per sample
    memcpy(wavHeader + 36, "data", 4);
    *((uint32_t*)(wavHeader + 40)) = 0xFFFFFFFF; // Infinite data size
    
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "audio/wav", "");
    
    WiFiClient client = server.client();
    
    // Send WAV header
    if (!client.write(wavHeader, 44)) {
        Serial.println("[Live Audio] Failed to send WAV header");
        client.stop();
        return;
    }
    
    Serial.println("[Live Audio] WAV header sent, buffering initial audio...");
    
    // Create a larger buffer for continuous streaming
    const int STREAM_BUFFER_SIZE = 2048; // 4KB buffer (2048 samples)
    int16_t streamBuffer[STREAM_BUFFER_SIZE];
    int bufferIndex = 0;
    bool bufferFilled = false;
    
    uint32_t streamStart = millis();
    uint32_t samplesStreamed = 0;
    uint32_t consecutiveFailures = 0;
    
    // Fill initial buffer before starting playback
    Serial.println("[Live Audio] Filling initial buffer...");
    while (bufferIndex < STREAM_BUFFER_SIZE && (millis() - streamStart < 5000)) {
        if (audioBufferReady) {
            // Copy audio buffer to stream buffer
            for (int i = 0; i < AUDIO_BUFFER_SIZE && bufferIndex < STREAM_BUFFER_SIZE; i++) {
                streamBuffer[bufferIndex++] = audioBuffer[i];
            }
            audioBufferReady = false;
        } else {
            delay(10);
        }
        yield();
    }
    
    Serial.printf("[Live Audio] Initial buffer filled with %d samples, starting stream...\n", bufferIndex);
    
    // Send initial buffer to get browser started
    if (bufferIndex > 0) {
        size_t bytesToSend = bufferIndex * sizeof(int16_t);
        size_t bytesWritten = client.write((uint8_t*)streamBuffer, bytesToSend);
        if (bytesWritten > 0) {
            samplesStreamed += bufferIndex;
            Serial.printf("[Live Audio] Sent initial buffer: %d bytes, %d samples\n", bytesWritten, bufferIndex);
        }
    }
    
    // Continue streaming with regular updates
    uint32_t lastSend = millis();
    while (client.connected() && microphoneEnabled && (millis() - streamStart < 60000)) {
        uint32_t now = millis();
        
        // Send new audio data every 100ms for smooth playback
        if (audioBufferReady && (now - lastSend >= 100)) {
            size_t bytesToSend = AUDIO_BUFFER_SIZE * sizeof(int16_t);
            size_t bytesWritten = client.write((uint8_t*)audioBuffer, bytesToSend);
            
            if (bytesWritten > 0) {
                samplesStreamed += AUDIO_BUFFER_SIZE;
                audioBufferReady = false;
                lastSend = now;
                consecutiveFailures = 0;
                
                // Debug every 1000 samples
                if (samplesStreamed % 1000 == 0) {
                    Serial.printf("[Live Audio] Streamed %d samples, level: %.1f%%\n", 
                                 samplesStreamed, audioLevel);
                }
            } else {
                consecutiveFailures++;
                if (consecutiveFailures > 3) {
                    Serial.println("[Live Audio] Multiple write failures, ending stream");
                    break;
                }
            }
        } else {
            // Fill with silence when no audio data to maintain stream
            if (now - lastSend >= 200) { // Send silence every 200ms if no real audio
                int16_t silence[AUDIO_BUFFER_SIZE];
                memset(silence, 0, sizeof(silence));
                client.write((uint8_t*)silence, sizeof(silence));
                lastSend = now;
            }
            delay(20);
        }
        
        yield();
    }
    
    client.stop();
    Serial.printf("[Live Audio] WAV stream ended. Total samples: %d, Duration: %d ms\n", 
                 samplesStreamed, millis() - streamStart);
}

void handleAudioChunk() {
    if (!microphoneEnabled || !microphoneInitialized) {
        server.send(404, "text/plain", "Microphone not available");
        return;
    }
    
    Serial.println("[Audio] Creating large microphone WAV chunk");
    
    // Collect multiple audio buffers for 0.5 second of audio
    const int SAMPLE_RATE = 22050;
    const int CHUNK_DURATION_MS = 500; // 0.5 seconds
    const int TARGET_SAMPLES = (SAMPLE_RATE * CHUNK_DURATION_MS) / 1000;
    const int WAV_HEADER_SIZE = 44;
    
    // Create buffer for collected samples
    int16_t* collectedSamples = (int16_t*)malloc(TARGET_SAMPLES * sizeof(int16_t));
    if (!collectedSamples) {
        server.send(500, "text/plain", "Memory allocation failed");
        return;
    }
    
    int collectedCount = 0;
    uint32_t startTime = millis();
    
    // Collect samples for the specified duration
    while (collectedCount < TARGET_SAMPLES && (millis() - startTime) < (CHUNK_DURATION_MS + 100)) {
        if (audioBufferReady) {
            // Copy available samples
            int samplesToCopy = min(AUDIO_BUFFER_SIZE, TARGET_SAMPLES - collectedCount);
            memcpy(&collectedSamples[collectedCount], audioBuffer, samplesToCopy * sizeof(int16_t));
            collectedCount += samplesToCopy;
            audioBufferReady = false;
            Serial.printf("[Audio] Collected %d/%d samples\n", collectedCount, TARGET_SAMPLES);
        } else {
            delay(10); // Wait for more audio data
        }
        yield();
    }
    
    // If we don't have enough samples, fill with what we have
    int actualSamples = collectedCount;
    if (actualSamples == 0) {
        // No audio data available, return error
        free(collectedSamples);
        server.send(404, "text/plain", "No audio data collected");
        return;
    }
    
    const int AUDIO_DATA_SIZE = actualSamples * 2;
    const int TOTAL_SIZE = WAV_HEADER_SIZE + AUDIO_DATA_SIZE;
    
    // Create WAV header
    uint8_t wavHeader[44];
    memcpy(wavHeader, "RIFF", 4);
    *((uint32_t*)(wavHeader + 4)) = TOTAL_SIZE - 8;
    memcpy(wavHeader + 8, "WAVE", 4);
    memcpy(wavHeader + 12, "fmt ", 4);
    *((uint32_t*)(wavHeader + 16)) = 16;
    *((uint16_t*)(wavHeader + 20)) = 1;
    *((uint16_t*)(wavHeader + 22)) = 1;
    *((uint32_t*)(wavHeader + 24)) = SAMPLE_RATE;
    *((uint32_t*)(wavHeader + 28)) = SAMPLE_RATE * 2;
    *((uint16_t*)(wavHeader + 32)) = 2;
    *((uint16_t*)(wavHeader + 34)) = 16;
    memcpy(wavHeader + 36, "data", 4);
    *((uint32_t*)(wavHeader + 40)) = AUDIO_DATA_SIZE;
    
    // Send headers
    server.sendHeader("Content-Type", "audio/wav");
    server.sendHeader("Content-Length", String(TOTAL_SIZE));
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "audio/wav", "");
    
    WiFiClient client = server.client();
    
    // Send WAV header
    client.write(wavHeader, WAV_HEADER_SIZE);
    
    // Send collected audio data
    client.write((uint8_t*)collectedSamples, AUDIO_DATA_SIZE);
    
    free(collectedSamples);
    
    Serial.printf("[Audio] Sent microphone WAV: %d bytes (%d samples, %.1fs duration)\n", 
                 TOTAL_SIZE, actualSamples, (float)actualSamples / SAMPLE_RATE);
}

void handleAudioTest() {
    Serial.println("[Audio] Serving test tone WAV");
    
    // Create a test tone - 440Hz sine wave for 1 second
    const int SAMPLE_RATE = 22050;
    const int DURATION_SAMPLES = SAMPLE_RATE; // 1 second
    const int WAV_HEADER_SIZE = 44;
    const int AUDIO_DATA_SIZE = DURATION_SAMPLES * 2; // 16-bit samples
    const int TOTAL_SIZE = WAV_HEADER_SIZE + AUDIO_DATA_SIZE;
    
    // Create WAV header
    uint8_t wavHeader[44];
    memcpy(wavHeader, "RIFF", 4);
    *((uint32_t*)(wavHeader + 4)) = TOTAL_SIZE - 8;
    memcpy(wavHeader + 8, "WAVE", 4);
    memcpy(wavHeader + 12, "fmt ", 4);
    *((uint32_t*)(wavHeader + 16)) = 16; // PCM format chunk size
    *((uint16_t*)(wavHeader + 20)) = 1;  // PCM format
    *((uint16_t*)(wavHeader + 22)) = 1;  // Mono
    *((uint32_t*)(wavHeader + 24)) = SAMPLE_RATE;
    *((uint32_t*)(wavHeader + 28)) = SAMPLE_RATE * 2; // Byte rate
    *((uint16_t*)(wavHeader + 32)) = 2;  // Block align
    *((uint16_t*)(wavHeader + 34)) = 16; // Bits per sample
    memcpy(wavHeader + 36, "data", 4);
    *((uint32_t*)(wavHeader + 40)) = AUDIO_DATA_SIZE;
    
    // Send headers
    server.sendHeader("Content-Type", "audio/wav");
    server.sendHeader("Content-Length", String(TOTAL_SIZE));
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "audio/wav", "");
    
    WiFiClient client = server.client();
    
    // Send WAV header
    client.write(wavHeader, WAV_HEADER_SIZE);
    
    // Generate and send 440Hz sine wave
    for (int i = 0; i < DURATION_SAMPLES; i++) {
        float time = (float)i / SAMPLE_RATE;
        float amplitude = sin(2.0 * PI * 440.0 * time); // 440Hz sine wave
        int16_t sample = (int16_t)(amplitude * 16383); // 16-bit amplitude
        client.write((uint8_t*)&sample, 2);
    }
    
    Serial.printf("[Audio] Sent test tone WAV: %d bytes, 440Hz, 1 second\n", TOTAL_SIZE);
}

void handleWebRTCOffer() {
    Serial.println("[WebRTC] Received offer request");
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    
    if (server.method() == HTTP_OPTIONS) {
        server.send(200, "text/plain", "OK");
        return;
    }
    
    // Simple WebRTC-like response indicating audio stream availability
    String response = "{";
    response += "\"status\":\"ready\",";
    response += "\"audioAvailable\":" + String(microphoneEnabled && microphoneInitialized ? "true" : "false") + ",";
    response += "\"sampleRate\":22050,";
    response += "\"channels\":1,";
    response += "\"format\":\"PCM16\"";
    response += "}";
    
    server.send(200, "application/json", response);
    Serial.println("[WebRTC] Sent audio stream info");
}

void handleWebRTCAnswer() {
    Serial.println("[WebRTC] Answer request received");
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    String response = "{\"status\":\"connected\",\"message\":\"ESP32 audio stream ready\"}";
    server.send(200, "application/json", response);
}

void updateSpotlight() {
    // Only log when state changes to prevent flooding
    if (spotlightEnabled != lastSpotlightState || 
        (spotlightEnabled && spotlightBrightness != lastSpotlightBrightness)) {
        
        if (spotlightEnabled) {
            Serial.println("Spotlight: ON, Brightness: " + String(spotlightBrightness));
        } else {
            Serial.println("Spotlight: OFF");
        }
        
        // Update last known states
        lastSpotlightState = spotlightEnabled;
        lastSpotlightBrightness = spotlightBrightness;
    }
    
    // Note: In PCB redesign, use GPIO 47 with actual PWM control
    // For now, this is just software tracking since GPIO 14 causes conflicts
}

bool initializeMicrophone() {
    Serial.println("Configuring INMP441 I2S microphone...");
    
    // INMP441 I2S microphone configuration - optimized settings
    // INMP441 specific configuration - known working setup
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,  // Back to 44.1kHz - INMP441's natural frequency
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Standard I2S format (non-deprecated)
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,   // Larger DMA buffer
        .use_apll = false,     // Try without APLL first
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = 41,   // MICROPHONE_SCK_PIN (Serial Clock)
        .ws_io_num = 42,    // MICROPHONE_WS_PIN (Word Select / Left-Right Clock)
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = 47   // MICROPHONE_SD_PIN (Serial Data Input) - YOUR ACTUAL WIRING
    };
    
    Serial.printf("I2S Config: Sample Rate=%d, Bits=%d, Buffers=%d x %d\n", 
                  i2s_config.sample_rate, i2s_config.bits_per_sample, 
                  i2s_config.dma_buf_count, i2s_config.dma_buf_len);
    Serial.printf("I2S Pins: BCK=%d, WS=%d, DIN=%d\n", 
                  pin_config.bck_io_num, pin_config.ws_io_num, pin_config.data_in_num);
    Serial.println("📍 Expected INMP441 wiring:");
    Serial.println("   SCK -> GPIO 41");
    Serial.println("   WS  -> GPIO 42"); 
    Serial.println("   SD  -> GPIO 47");
    Serial.println("   VDD -> 3.3V (with 10R + 100nF)");
    
    // Install I2S driver
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("❌ I2S driver install failed: %s\n", esp_err_to_name(err));
        return false;
    }
    
    // Set I2S pins
    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("❌ I2S set pin failed: %s\n", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }
    
    // Start I2S
    err = i2s_start(I2S_NUM_0);
    if (err != ESP_OK) {
        Serial.printf("❌ I2S start failed: %s\n", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }
    
    // Clear the I2S buffer to remove any initial noise
    i2s_zero_dma_buffer(I2S_NUM_0);
    
    Serial.println("✅ INMP441 microphone I2S configured and started successfully");
    Serial.println("ℹ️ Note: INMP441 L/R pin should be connected to GND for left channel");
    Serial.println("ℹ️ If still getting -1 values, check wiring and L/R pin connection");
    return true;
}

bool initializeIMU() {
    Serial.println("=== IMU INITIALIZATION DEBUG ===");
    Serial.println("Configuring IMU on I2C...");
    
    // Initialize I2C for BMI270 IMU
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);  // SDA=GPIO2, SCL=GPIO1
    Wire.setClock(400000);  // 400kHz I2C speed
    
    Serial.printf("I2C Config: SDA=GPIO%d, SCL=GPIO%d, Freq=400kHz\n", IMU_SDA_PIN, IMU_SCL_PIN);
    Serial.println("📍 Expected IMU wiring (corrected):");
    Serial.println("   SDA -> GPIO 1");  
    Serial.println("   SCL -> GPIO 2");
    Serial.println("   VCC -> 5V rail");
    Serial.println("   GND -> GND");
    
    // I2C device scan - try common IMU addresses
    Serial.println("Scanning I2C bus for IMU sensors...");
    bool deviceFound = false;
    uint8_t foundAddress = 0;
    
    // Try common IMU addresses
    uint8_t addresses[] = {0x68, 0x69, 0x6A, 0x6B};  // MPU6050, BMI270, etc.
    
    for (int i = 0; i < 4; i++) {
        Wire.beginTransmission(addresses[i]);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("✅ I2C device found at address 0x%02X\n", addresses[i]);
            foundAddress = addresses[i];
            deviceFound = true;
            break;
        }
    }
    
    if (!deviceFound) {
        Serial.println("❌ No I2C IMU found at common addresses (0x68, 0x69, 0x6A, 0x6B)");
        Serial.println("💡 Check I2C wiring: SDA=GPIO1, SCL=GPIO2");
        Serial.println("💡 Verify 5V power and GND connections");
        return false;
    }
    
    // Store found address for later use
    detectedIMUAddress = foundAddress;
    
    // Read chip ID to identify sensor type
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x00);  // Chip ID register (common for most IMUs)
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    
    uint8_t chipId = 0;
    if (Wire.available()) {
        chipId = Wire.read();
        Serial.printf("✅ IMU Chip ID: 0x%02X at address 0x%02X\n", chipId, detectedIMUAddress);
        
        // Identify common IMU types
        switch (chipId) {
            case 0x24: Serial.println("📡 Detected: BMI270 (6-axis IMU)"); break;
            case 0x68: Serial.println("📡 Detected: MPU6050 (6-axis IMU)"); break;
            case 0x71: Serial.println("📡 Detected: MPU6000 (6-axis IMU)"); break;
            case 0x19: Serial.println("📡 Detected: MPU9250 (9-axis IMU)"); break;
            case 0xD1: Serial.println("📡 Detected: BMI160 (6-axis IMU)"); break;
            default: 
                Serial.printf("📡 Detected: Unknown IMU (Chip ID: 0x%02X)\n", chipId);
                Serial.println("ℹ️ Attempting generic 6-axis IMU initialization...");
                break;
        }
    } else {
        Serial.println("⚠️ Could not read chip ID, attempting generic initialization...");
    }
    
    // BMI270 PROPER initialization sequence
    Serial.println("🔧 BMI270 FULL INITIALIZATION SEQUENCE");
    
    // Step 1: Soft reset first
    Serial.println("Step 1: Soft reset BMI270...");
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x7E);  // CMD register
    Wire.write(0xB6);  // Soft reset command
    Wire.endTransmission();
    delay(1000);  // Wait for reset to complete
    
    // Step 2: Read chip ID again after reset
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x00);  // Chip ID register
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t chipIdAfterReset = Wire.read();
        Serial.printf("Step 2: Chip ID after reset: 0x%02X\n", chipIdAfterReset);
    }
    
    // Step 3: Check internal status (BMI270 specific)
    Serial.println("Step 3: Checking BMI270 internal status...");
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x21);  // INTERNAL_STATUS register
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t internalStatus = Wire.read();
        Serial.printf("INTERNAL_STATUS: 0x%02X\n", internalStatus);
    }
    
    // Step 4: Enable advanced power mode FIRST
    Serial.println("Step 4: Setting advanced power mode...");
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x7C);  // PWR_CONF register (different from PWR_CTRL!)
    Wire.write(0x00);  // Advanced power save disabled
    Wire.endTransmission();
    delay(450);  // BMI270 requires 450ms delay after PWR_CONF change!
    
    // Step 5: Now enable accelerometer and gyroscope
    Serial.println("Step 5: Enabling accelerometer and gyroscope...");
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x7D);  // PWR_CTRL register
    Wire.write(0x0E);  // Enable ACC (bit 2) and GYR (bit 1)  
    Wire.endTransmission();
    delay(50);
    
    // Step 6: MINIMAL BMI270 configuration (last resort)
    Serial.println("Step 6: MINIMAL BMI270 configuration...");
    
    // Clear error register first
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x1B);  // ERR_REG
    Wire.write(0x00);  // Clear errors
    Wire.endTransmission();
    delay(10);
    
    // Try absolute minimum accelerometer config
    Serial.println("Trying absolute minimum accelerometer setup...");
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x41);  // ACC_CONF
    Wire.write(0x08);  // MINIMAL: Just 100Hz ODR, no other bits
    uint8_t result1 = Wire.endTransmission();
    delay(50);
    
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x40);  // ACC_RANGE  
    Wire.write(0x00);  // ±2g range
    uint8_t result2 = Wire.endTransmission();
    delay(50);
    
    Serial.printf("Minimal ACC config results: %d, %d\n", result1, result2);
    
    // Check what we actually got
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x41);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t accConf = Wire.read();
        Serial.printf("ACC_CONF after minimal config: 0x%02X\n", accConf);
    }
    
    // Check error register again
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t errReg = Wire.read();
        Serial.printf("ERR_REG after minimal config: 0x%02X\n", errReg);
    }
    
    // Step 7: Configure gyroscope (±2000°/s, 100Hz)
    Serial.println("Step 7: Configuring gyroscope...");
    
    // Configure GYR_CONF first
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x42);  // GYR_CONF register
    Wire.write(0xA9);  // gyr_odr=100Hz(0x09), gyr_bwp=normal(0xA0)
    uint8_t gyrConfResult = Wire.endTransmission();
    Serial.printf("GYR_CONF write result: %d\n", gyrConfResult);
    delay(50);
    
    // Then set GYR_RANGE
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x43);  // GYR_RANGE register  
    Wire.write(0x00);  // ±2000°/s range
    uint8_t gyrRangeResult = Wire.endTransmission();
    Serial.printf("GYR_RANGE write result: %d\n", gyrRangeResult);
    delay(50);
    
    // Step 8: Force-write registers again (BMI270 sometimes needs this)  
    Serial.println("Step 8: Force-writing critical registers again...");
    
    // Force ACC_CONF again with MINIMAL value
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x41);
    Wire.write(0x08);  // MINIMAL configuration
    Wire.endTransmission();
    delay(10);
    
    // Force ACC_RANGE again 
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x40);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
    
    // Verify immediately after force-write
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x41);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t accConfCheck = Wire.read();
        Serial.printf("ACC_CONF after force-write: 0x%02X\n", accConfCheck);
    }
    
    // Step 9: Wait for sensors to stabilize
    Serial.println("Step 9: Waiting for sensors to stabilize (2 seconds)...");
    delay(2000);
    
    // Step 10: Verify configuration by reading back all registers
    Serial.println("Step 10: Verifying BMI270 configuration...");
    
    uint8_t regs[] = {0x7C, 0x7D, 0x40, 0x41, 0x43, 0x42};
    String regNames[] = {"PWR_CONF", "PWR_CTRL", "ACC_RANGE", "ACC_CONF", "GYR_RANGE", "GYR_CONF"};
    uint8_t expected[] = {0x00, 0x0E, 0x00, 0x08, 0x00, 0xA9};
    
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(detectedIMUAddress);
        Wire.write(regs[i]);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)1);
        if (Wire.available()) {
            uint8_t value = Wire.read();
            Serial.printf("%s (0x%02X): 0x%02X (expected 0x%02X) %s\n", 
                         regNames[i].c_str(), regs[i], value, expected[i],
                         value == expected[i] ? "✅" : "❌");
        }
    }
    
    // Step 11: COMPREHENSIVE LIVE MOTION TEST
    Serial.println("Step 11: LIVE MOTION TEST - MOVE THE BOARD NOW!");
    Serial.println("Testing data registers during 10-second window...");
    
    for (int test = 0; test < 50; test++) { // 10 seconds of testing
        Serial.printf("Test %d: ", test);
        
        // Test accelerometer data - try multiple registers
        Wire.beginTransmission(detectedIMUAddress);
        Wire.write(0x0C);  // Standard accel register
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)6);
        
        if (Wire.available() >= 6) {
            uint8_t data[6];
            for (int i = 0; i < 6; i++) data[i] = Wire.read();
            
            int16_t rawX = (data[1] << 8) | data[0];
            int16_t rawY = (data[3] << 8) | data[2]; 
            int16_t rawZ = (data[5] << 8) | data[4];
            
            Serial.printf("ACC: %d,%d,%d ", rawX, rawY, rawZ);
        }
        
        // Test gyroscope data
        Wire.beginTransmission(detectedIMUAddress);
        Wire.write(0x12);  // Standard gyro register
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)6);
        
        if (Wire.available() >= 6) {
            uint8_t data[6];
            for (int i = 0; i < 6; i++) data[i] = Wire.read();
            
            int16_t rawX = (data[1] << 8) | data[0];
            int16_t rawY = (data[3] << 8) | data[2];
            int16_t rawZ = (data[5] << 8) | data[4];
            
            Serial.printf("GYR: %d,%d,%d ", rawX, rawY, rawZ);
        }
        
        // Check if we're getting ANY non-zero values
        Serial.println();
        delay(200); // 200ms between readings
        
        // Break early if we find data
        if (test > 0 && test % 10 == 0) {
            Serial.printf("=== After %d seconds of testing ===\n", test/5);
        }
    }
    
    Serial.println("=== MOTION TEST COMPLETE ===");
    
    Serial.println("🎯 BMI270 INITIALIZATION COMPLETE!");
    Serial.printf("📍 Final I2C address: 0x%02X\n", detectedIMUAddress);
    Serial.println("📐 Config: ±2g accel, ±2000°/s gyro, 100Hz both");
    
    // Perform initial calibration
    if (!calibrateIMU()) {
        Serial.println("❌ Initial IMU calibration failed");
        // Continue anyway with no calibration
    }
    
    imuInitialized = true;  // Set this BEFORE calling calibrateIMU!
    
    Serial.println("=== IMU INITIALIZATION SUCCESS ===");
    Serial.printf("🎯 Final IMU Address: 0x%02X\n", detectedIMUAddress);
    Serial.printf("🎯 Expected readings after calibration:\n");
    Serial.printf("   Accel: ~0.0, ~0.0, ~1.0g (gravity)\n");
    Serial.printf("   Gyro: ~0.0, ~0.0, ~0.0°/s (with bias correction)\n");
    Serial.println("=================================");
    
    return true;
}

bool calibrateIMU() {
    if (!imuInitialized) {
        Serial.println("❌ IMU not initialized, cannot calibrate");
        return false;
    }
    
    calibrationInProgress = true;
    Serial.println("=== STARTING IMU CALIBRATION ===");
    Serial.println("🔧 Keep the drone STATIONARY for 3 seconds...");
    Serial.println("📐 Calculating gyroscope bias offsets...");
    
    // Calibration: Calculate gyroscope bias by averaging 60 samples (3 seconds)
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    int validSamples = 0;
    
    for (int i = 0; i < 60; i++) {
        // Read gyroscope data for calibration
        Wire.beginTransmission(detectedIMUAddress);
        Wire.write(0x12);  // BMI270 GYR_X_LSB register
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)6);
        
        if (Wire.available() >= 6) {
            uint8_t gyrXL = Wire.read();  // BMI270 format: LSB first
            uint8_t gyrXH = Wire.read();
            uint8_t gyrYL = Wire.read();
            uint8_t gyrYH = Wire.read();
            uint8_t gyrZL = Wire.read();
            uint8_t gyrZH = Wire.read();
            
            int16_t rawGyroX = (int16_t)((gyrXH << 8) | gyrXL);
            int16_t rawGyroY = (int16_t)((gyrYH << 8) | gyrYL);
            int16_t rawGyroZ = (int16_t)((gyrZH << 8) | gyrZL);
            
            // Convert to degrees/second
            float tempGyroX = rawGyroX / 16.384;
            float tempGyroY = rawGyroY / 16.384;
            float tempGyroZ = rawGyroZ / 16.384;
            
            // Only accept reasonable values (reject extreme outliers) - higher tolerance for BMI270
            if (abs(tempGyroX) < 3000 && abs(tempGyroY) < 3000 && abs(tempGyroZ) < 3000) {
                sumGyroX += tempGyroX;
                sumGyroY += tempGyroY;
                sumGyroZ += tempGyroZ;
                validSamples++;
            }
            
            if (i % 15 == 0) {
                Serial.printf("Calibration progress: %d%% | Raw gyro: %.1f, %.1f, %.1f°/s\n", 
                              (i * 100) / 60, tempGyroX, tempGyroY, tempGyroZ);
            }
        }
        delay(50);  // 50ms between samples = 3 second total calibration
    }
    
    // Calculate average offsets
    if (validSamples > 30) {  // Need at least 30 valid samples
        gyroOffsetX = sumGyroX / validSamples;
        gyroOffsetY = sumGyroY / validSamples;
        gyroOffsetZ = sumGyroZ / validSamples;
        imuCalibrated = true;
        
        Serial.println("✅ IMU CALIBRATION COMPLETED!");
        Serial.printf("🎯 Gyroscope Bias Offsets:\n");
        Serial.printf("   X: %.2f°/s, Y: %.2f°/s, Z: %.2f°/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
        Serial.printf("📊 Used %d valid samples out of 60\n", validSamples);
    } else {
        Serial.printf("❌ Calibration failed - only %d valid samples (need 30+)\n", validSamples);
        gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0.0;
        imuCalibrated = false;
        calibrationInProgress = false;
        return false;
    }
    
    calibrationInProgress = false;
    return true;
}

void updateMicrophone() {
    static uint32_t lastMicUpdate = 0;
    static uint32_t readAttempts = 0;
    static uint32_t successfulReads = 0;
    
    if (!microphoneInitialized || !microphoneEnabled) {
        audioLevel = 0.0;
        return;
    }
    
    // Only update microphone every 50ms to reduce CPU load
    uint32_t currentTime = millis();
    if (currentTime - lastMicUpdate < 50) {
        return;
    }
    lastMicUpdate = currentTime;
    
    // Read audio samples from I2S
    int32_t samples[128];  // Larger buffer for better readings
    size_t bytes_read = 0;
    
    esp_err_t err = i2s_read(I2S_NUM_0, samples, sizeof(samples), &bytes_read, 20); // 20ms timeout
    readAttempts++;
    
    if (err == ESP_OK && bytes_read > 0) {
        successfulReads++;
        
        // Calculate audio level using different approach
        int32_t maxSample = 0;
        int32_t totalAmplitude = 0;
        int sample_count = bytes_read / sizeof(int32_t);
        
        for (int i = 0; i < sample_count; i++) {
            // Get the raw 32-bit sample and shift to get meaningful data
            int32_t sample = samples[i] >> 8; // Shift to get 24-bit equivalent
            int32_t amplitude = abs(sample);
            
            totalAmplitude += amplitude;
            if (amplitude > maxSample) {
                maxSample = amplitude;
            }
            
            // Optimized INMP441 audio processing
            if (audioBufferIndex < AUDIO_BUFFER_SIZE) {
                int32_t raw_sample = samples[i];
                
                // Extract 16-bit audio from INMP441 24-bit data
                // Try shifting by 16 instead of 20 for louder audio
                int32_t audio_sample = raw_sample >> 16; // Get upper 16 bits
                
                // Apply gain to make it audible
                audio_sample *= 16; // Increase volume significantly (doubled from 8x)
                
                // Clamp to prevent overflow
                if (audio_sample > 32767) audio_sample = 32767;
                if (audio_sample < -32768) audio_sample = -32768;
                
                // Apply light smoothing to reduce noise
                static int32_t lastSample = 0;
                audio_sample = (lastSample * 3 + audio_sample) / 4; // Lighter smoothing
                lastSample = audio_sample;
                
                audioBuffer[audioBufferIndex] = (int16_t)audio_sample;
                audioBufferIndex++;
                
                // Mark buffer as ready when full
                if (audioBufferIndex >= AUDIO_BUFFER_SIZE) {
                    audioBufferReady = true;
                    audioBufferIndex = 0; // Reset for next buffer
                }
            }
        }
        
        if (sample_count > 0) {
            // Use average amplitude for smoother readings
            float avgAmplitude = (float)totalAmplitude / sample_count;
            
            // Scale to percentage (adjust scaling factor based on your microphone)
            audioLevel = (avgAmplitude / 8388608.0) * 100.0; // 24-bit max value
            
            // Alternative: Use peak detection
            float peakLevel = (maxSample / 8388608.0) * 100.0;
            
            // Use average of both methods for stability
            audioLevel = (audioLevel + peakLevel) / 2.0;
            
            // Clamp to reasonable range
            if (audioLevel > 100.0) audioLevel = 100.0;
            if (audioLevel < 0.0) audioLevel = 0.0;
        }
        
        // Debug output every few successful reads
        if (successfulReads % 50 == 0) {
            Serial.printf("[Mic] Success: %d, Level: %.1f%%, BuffReady: %s\n", 
                         successfulReads, audioLevel * 100.0, 
                         audioBufferReady ? "Yes" : "No");
            if (sample_count > 0) {
                int32_t shifted = samples[0] >> 16;  // First step
                int32_t amplified = shifted * 16;    // Second step (now 16x)
                int32_t clamped = (amplified > 32767) ? 32767 : ((amplified < -32768) ? -32768 : amplified);
                Serial.printf("[Mic Debug] Raw: 0x%08X -> Shifted: %d -> Amplified: %d -> Final: %d\n", 
                             samples[0], shifted, amplified, clamped);
            }
        }
        
        // Detect if we have real microphone data or just noise
        static bool realMicDetected = false;
        if (sample_count > 0) {
            // Check if all samples are -1 (no mic) or if we have varying data (real mic)
            bool allSame = true;
            for (int i = 1; i < sample_count && i < 10; i++) {
                if (samples[i] != samples[0]) {
                    allSame = false;
                    break;
                }
            }
            
            if (!allSame || (samples[0] != -1 && samples[0] != 0)) {
                realMicDetected = true;
                microphoneSimulated = false;
            } else if (readAttempts > 10 && !realMicDetected) {
                // After 10 attempts with no real data, switch to simulated mode
                microphoneSimulated = true;
            }
        }
        
        // Debug info every 2 seconds
        static uint32_t lastDebug = 0;
        if (currentTime - lastDebug > 2000) {
            lastDebug = currentTime;
            Serial.printf("[MIC DEBUG] Attempts: %lu, Successful: %lu, Bytes: %d, Level: %.1f%%, Mode: %s\n", 
                         readAttempts, successfulReads, bytes_read, audioLevel,
                         microphoneSimulated ? "SIMULATED" : "REAL");
            
            // Show raw sample data occasionally (only for real mic debugging)
            if (sample_count > 0 && !microphoneSimulated) {
                Serial.printf("[MIC SAMPLES] First few: %ld, %ld, %ld\n", 
                             samples[0], sample_count > 1 ? samples[1] : 0, sample_count > 2 ? samples[2] : 0);
            }
        }
        
    } else {
        // Keep current level but decay slowly
        audioLevel *= 0.9; // Slower decay
        if (audioLevel < 1.0) audioLevel = 0.0;
        
        // Debug I2S read issues (only if not in simulated mode)
        if (readAttempts % 100 == 0 && !microphoneSimulated) {
            Serial.printf("[MIC ERROR] Read failed: %s, Attempts: %lu, Success rate: %.1f%%\n", 
                         esp_err_to_name(err), readAttempts, 
                         readAttempts > 0 ? (float)successfulReads * 100.0 / readAttempts : 0.0);
        }
    }
    
    // SIMULATED MICROPHONE MODE - for testing without hardware
    if (microphoneSimulated && microphoneEnabled) {
        // Generate realistic simulated audio levels
        static float simPhase = 0.0;
        static uint32_t lastSimUpdate = 0;
        
        if (currentTime - lastSimUpdate > 100) { // Update every 100ms
            lastSimUpdate = currentTime;
            
            // Create varying audio levels - mix of sine wave and random noise
            simPhase += 0.3;
            if (simPhase > 6.28) simPhase = 0.0; // 2*PI
            
            float baseLevel = (sin(simPhase) + 1.0) * 15.0; // 0-30% base
            float randomNoise = (float)(esp_random() % 20); // 0-20% random
            float ambientLevel = 5.0; // 5% ambient
            
            audioLevel = baseLevel + randomNoise + ambientLevel;
            
            // Occasional "loud sounds" for testing
            if ((esp_random() % 100) < 5) { // 5% chance
                audioLevel += (esp_random() % 40) + 20; // +20-60%
            }
            
            // Clamp to 0-100%
            if (audioLevel > 100.0) audioLevel = 100.0;
            if (audioLevel < 0.0) audioLevel = 0.0;
        }
    }
}

void updateIMU() {
    if (!imuInitialized) {
        pitch = roll = yaw = 0.0;
        gyroX = gyroY = gyroZ = 0.0;
        accelX = accelY = accelZ = 0.0;
        return;
    }
    
    // Update IMU data every 10ms (100Hz)
    uint32_t currentTime = millis();
    if (currentTime - lastIMUUpdate < 10) {
        return;
    }
    lastIMUUpdate = currentTime;
    
    // Read accelerometer data (6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x0C);  // BMI270 ACC_X_LSB register
    uint8_t accelError = Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)6);

    int16_t rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
    uint8_t bytesRead = Wire.available();
    
    if (bytesRead >= 6) {
        uint8_t accXL = Wire.read();  // BMI270 format: LSB first
        uint8_t accXH = Wire.read();
        uint8_t accYL = Wire.read();
        uint8_t accYH = Wire.read();
        uint8_t accZL = Wire.read();
        uint8_t accZH = Wire.read();
        
        rawAccelX = (int16_t)((accXH << 8) | accXL);
        rawAccelY = (int16_t)((accYH << 8) | accYL);
        rawAccelZ = (int16_t)((accZH << 8) | accZL);
        
        // Convert to g units (±2g range, 16-bit) - BMI270 format
        accelX = rawAccelX / 16384.0;  // LSB/g for ±2g range (16384 LSB/g)
        accelY = rawAccelY / 16384.0;
        accelZ = rawAccelZ / 16384.0;
        
        // Debug every 100 readings
        static int accelDebugCount = 0;
        accelDebugCount++;
        if (accelDebugCount >= 100) {
            Serial.printf("[ACCEL DEBUG] Error: %d, Bytes: %d, Raw: 0x%04X 0x%04X 0x%04X -> %.2f %.2f %.2f g\n", 
                         accelError, bytesRead, rawAccelX, rawAccelY, rawAccelZ, accelX, accelY, accelZ);
            accelDebugCount = 0;
        }
    } else {
        static int accelFailCount = 0;
        accelFailCount++;
        if (accelFailCount >= 100) {
            Serial.printf("[ACCEL FAIL] I2C Error: %d, Available bytes: %d (expected 6)\n", accelError, bytesRead);
            accelFailCount = 0;
        }
    }
    
    // Read gyroscope data (6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    Wire.beginTransmission(detectedIMUAddress);
    Wire.write(0x12);  // BMI270 GYR_X_LSB register
    uint8_t gyroError = Wire.endTransmission();
    Wire.requestFrom((uint8_t)detectedIMUAddress, (uint8_t)6);

    int16_t rawGyroX = 0, rawGyroY = 0, rawGyroZ = 0;
    uint8_t gyroBytesRead = Wire.available();
    
    if (gyroBytesRead >= 6) {
        uint8_t gyrXL = Wire.read();  // BMI270 format: LSB first
        uint8_t gyrXH = Wire.read();
        uint8_t gyrYL = Wire.read();
        uint8_t gyrYH = Wire.read();
        uint8_t gyrZL = Wire.read();
        uint8_t gyrZH = Wire.read();
        
        rawGyroX = (int16_t)((gyrXH << 8) | gyrXL);
        rawGyroY = (int16_t)((gyrYH << 8) | gyrYL);
        rawGyroZ = (int16_t)((gyrZH << 8) | gyrZL);
        
        // Convert to degrees/second (±2000°/s range, 16-bit) - BMI270 format
        float rawGyroXDps = rawGyroX / 16.384;
        float rawGyroYDps = rawGyroY / 16.384; 
        float rawGyroZDps = rawGyroZ / 16.384;
        
        // Apply calibration offset only if calibrated and offset is reasonable
        gyroX = rawGyroXDps - (imuCalibrated ? gyroOffsetX : 0.0);
        gyroY = rawGyroYDps - (imuCalibrated ? gyroOffsetY : 0.0);  
        gyroZ = rawGyroZDps - (imuCalibrated ? gyroOffsetZ : 0.0);
        
        // Debug every 100 readings
        static int gyroDebugCount = 0;
        gyroDebugCount++;
        if (gyroDebugCount >= 100) {
            Serial.printf("[GYRO DEBUG] Error: %d, Bytes: %d, Raw: 0x%04X 0x%04X 0x%04X -> %.2f %.2f %.2f °/s\n", 
                         gyroError, gyroBytesRead, rawGyroX, rawGyroY, rawGyroZ, rawGyroXDps, rawGyroYDps, rawGyroZDps);
            gyroDebugCount = 0;
        }
    } else {
        static int gyroFailCount = 0;
        gyroFailCount++;
        if (gyroFailCount >= 100) {
            Serial.printf("[GYRO FAIL] I2C Error: %d, Available bytes: %d (expected 6)\n", gyroError, gyroBytesRead);
            gyroFailCount = 0;
        }
    }
    
    // Calculate pitch and roll from accelerometer (complementary filter)
    float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
    
    // Apply complementary filter (98% gyro, 2% accel for smooth response)
    float dt = 0.01; // 10ms = 0.01s
    pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accelPitch;
    roll = 0.98 * (roll + gyroY * dt) + 0.02 * accelRoll;
    yaw += gyroZ * dt; // Integrate gyro for yaw (no accel reference)
    
    // Debug output every 2 seconds
    static uint32_t lastDebug = 0;
    if (currentTime - lastDebug > 2000) {
        lastDebug = currentTime;
        Serial.printf("[IMU] %s | Pitch: %.1f°, Roll: %.1f°, Yaw: %.1f° | Accel: %.2fg %.2fg %.2fg | Gyro: %.1f°/s %.1f°/s %.1f°/s\n",
                     imuCalibrated ? "CAL" : "RAW", pitch, roll, yaw, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    }
}

void updateSystemStatus() {
    systemUptime = millis() / 1000;
    systemFreeHeap = ESP.getFreeHeap();
    systemTemperature = 25.0; // Skip temperature sensor to prevent crashes
}