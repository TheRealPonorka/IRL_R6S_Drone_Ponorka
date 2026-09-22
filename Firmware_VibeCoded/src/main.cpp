/**
 * @file main.cpp
 * @brief R6 Recon Drone - Clean WiFi Test Version
 * @author Peter
 * @version 2.1.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>
#include "config.h"

// ===== CORE SYSTEM =====
const char* ap_ssid = "R6_Recon_Drone";
const char* ap_password = "ReconDrone123";

// Web server (standard, proven stable)
WebServer server(80);

// LED Strip
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);

// ===== SYSTEM STATE =====
struct SystemState {
    // LED Strip
    bool ledEnabled = true;
    uint8_t ledR = 0, ledG = 255, ledB = 0; // Default green
    uint8_t ledBrightness = 64;
    
    // Spotlight
    bool spotlightEnabled = false;
    
    // System
    uint32_t uptime = 0;
    uint32_t freeHeap = 0;
    
    // Hardware status
    bool imuInitialized = false;
    bool motorsInitialized = false;
    bool encodersInitialized = false;
    bool spotlightInitialized = false;
} systemState;

// ===== ANIMATION VARIABLES =====
uint32_t lastLEDUpdate = 0;
uint8_t animationStep = 0;

// ===== FUNCTION DECLARATIONS =====
void initializeHardware();
void initializeLEDs();
void initializeSpotlight();
void initializeMotors();
void initializeEncoders();
bool initializeIMU();
void updateLEDs();
void updateSpotlight();
void setupWebServer();

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("==========================================");
    Serial.println("🎯 R6 Recon Drone v2.1.0 - Clean WiFi Test");
    Serial.println("==========================================");
    
    // Initialize hardware
    initializeHardware();
    
    // Initialize WiFi
    Serial.println("Starting WiFi AP...");
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_AP);
    delay(1000);
    
    if (WiFi.softAP(ap_ssid, ap_password)) {
        IPAddress IP = WiFi.softAPIP();
        Serial.println("✅ WiFi AP started successfully!");
        Serial.println("📍 IP: " + IP.toString());
        Serial.println("📱 SSID: " + String(ap_ssid));
        
        // Setup web server
        setupWebServer();
        server.begin();
        Serial.println("✅ Web server started");
    } else {
        Serial.println("❌ WiFi failed to start");
    }
    
    Serial.println("==========================================");
    Serial.println("✅ R6 Drone operational!");
    Serial.println("LED strip + WiFi + WebServer working");
    Serial.println("==========================================");
}

void loop() {
    // Handle web server
    server.handleClient();
    
    // Update hardware
    updateLEDs();
    updateSpotlight();
    
    // System status
    systemState.uptime = millis() / 1000;
    systemState.freeHeap = ESP.getFreeHeap();
    
    // Print status every 5 seconds
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        lastStatus = millis();
        Serial.printf("[STATUS] Uptime: %lu s, Heap: %d, LED: %s, WiFi: %s\n", 
                     systemState.uptime, systemState.freeHeap,
                     systemState.ledEnabled ? "ON" : "OFF",
                     WiFi.status() == WL_CONNECTED ? "STA" : "AP");
    }
    
    delay(10);
    yield();
}

// ===== HARDWARE INITIALIZATION =====
void initializeHardware() {
    Serial.println("Initializing hardware...");
    
    // LittleFS
    if (LittleFS.begin(true)) {
        Serial.println("✅ LittleFS mounted");
    } else {
        Serial.println("⚠️ LittleFS failed");
    }
    
    // Hardware components
    initializeLEDs();
    initializeSpotlight();
    
    if (initializeIMU()) {
        systemState.imuInitialized = true;
        Serial.println("✅ IMU initialized");
    } else {
        Serial.println("⚠️ IMU not found");
    }
    
    initializeMotors();
    initializeEncoders();
    
    Serial.println("Hardware initialization complete");
}

void initializeLEDs() {
    strip.begin();
    strip.setBrightness(systemState.ledBrightness);
    strip.clear();
    strip.show();
    Serial.println("✅ LED Strip initialized");
}

void initializeSpotlight() {
    pinMode(SPOTLIGHT_PIN, OUTPUT);
    ledcSetup(4, 5000, 8);
    ledcAttachPin(SPOTLIGHT_PIN, 4);
    ledcWrite(4, 0);
    systemState.spotlightInitialized = true;
    Serial.println("✅ Spotlight initialized");
}

void initializeMotors() {
    pinMode(MOTOR_A_PIN1, OUTPUT);
    pinMode(MOTOR_A_PIN2, OUTPUT);
    pinMode(MOTOR_B_PIN1, OUTPUT);
    pinMode(MOTOR_B_PIN2, OUTPUT);
    
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);
    ledcSetup(2, 20000, 8);
    ledcSetup(3, 20000, 8);
    
    ledcAttachPin(MOTOR_A_PIN1, 0);
    ledcAttachPin(MOTOR_A_PIN2, 1);
    ledcAttachPin(MOTOR_B_PIN1, 2);
    ledcAttachPin(MOTOR_B_PIN2, 3);
    
    systemState.motorsInitialized = true;
    Serial.println("✅ Motors initialized");
}

void initializeEncoders() {
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);
    systemState.encodersInitialized = true;
    Serial.println("✅ Encoders initialized");
}

bool initializeIMU() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    
    Wire.beginTransmission(0x68); // BMI270 address
    if (Wire.endTransmission() == 0) {
        return true;
    }
    return false;
}

// ===== UPDATE FUNCTIONS =====
void updateLEDs() {
    if (!systemState.ledEnabled) {
        strip.clear();
        strip.show();
        return;
    }
    
    uint32_t currentTime = millis();
    if (currentTime - lastLEDUpdate < LED_ANIMATION_SPEED) return;
    
    lastLEDUpdate = currentTime;
    strip.clear();
    
    // Middle-out animation
    switch (animationStep) {
        case 0: break; // No LEDs
        case 1: // LED 2-3
            strip.setPixelColor(2, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            strip.setPixelColor(3, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            break;
        case 2: // LED 1-2-3-4
            for (int i = 1; i <= 4; i++) {
                strip.setPixelColor(i, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            }
            break;
        case 3: // All LEDs
            for (int i = 0; i < LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            }
            break;
        case 4: // LED 1-2-3-4
            for (int i = 1; i <= 4; i++) {
                strip.setPixelColor(i, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            }
            break;
        case 5: // LED 2-3
            strip.setPixelColor(2, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            strip.setPixelColor(3, strip.Color(systemState.ledR, systemState.ledG, systemState.ledB));
            break;
    }
    
    strip.show();
    animationStep++;
    if (animationStep >= 6) animationStep = 0;
}

void updateSpotlight() {
    if (systemState.spotlightEnabled) {
        ledcWrite(4, 255); // Full brightness
    } else {
        ledcWrite(4, 0);
    }
}

// ===== WEB SERVER =====
void setupWebServer() {
    // Main page
    server.on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>R6 Recon Drone</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<style>";
        html += "body { font-family: Arial; text-align: center; margin: 20px; background: #1a1a1a; color: white; }";
        html += "h1 { color: #00ff00; }";
        html += ".button { background: #333; border: 2px solid #00ff00; color: white; padding: 15px 30px; margin: 10px; font-size: 18px; border-radius: 10px; cursor: pointer; }";
        html += ".button:hover { background: #00ff00; color: black; }";
        html += ".button.active { background: #00ff00; color: black; }";
        html += ".status { background: #333; padding: 20px; margin: 20px; border-radius: 10px; }";
        html += "</style></head><body>";
        html += "<h1>🎯 R6 Recon Drone v2.1.0</h1>";
        html += "<div class='status'>";
        html += "<p><strong>WiFi:</strong> Connected (AP Mode)</p>";
        html += "<p><strong>LED Strip:</strong> " + String(systemState.ledEnabled ? "ON" : "OFF") + "</p>";
        html += "<p><strong>Spotlight:</strong> " + String(systemState.spotlightEnabled ? "ON" : "OFF") + "</p>";
        html += "<p><strong>Uptime:</strong> " + String(systemState.uptime) + " seconds</p>";
        html += "<p><strong>Free Heap:</strong> " + String(systemState.freeHeap) + " bytes</p>";
        html += "</div>";
        html += "<button class='button" + String(systemState.ledEnabled ? " active" : "") + "' onclick='toggleLED()'>Toggle LED Strip</button>";
        html += "<button class='button" + String(systemState.spotlightEnabled ? " active" : "") + "' onclick='toggleSpotlight()'>Toggle Spotlight</button>";
        html += "<br><button class='button' onclick='window.location.reload()'>Refresh Status</button>";
        html += "<script>";
        html += "function toggleLED() { fetch('/toggle-led').then(() => window.location.reload()); }";
        html += "function toggleSpotlight() { fetch('/toggle-spotlight').then(() => window.location.reload()); }";
        html += "</script>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });
    
    // Toggle LED
    server.on("/toggle-led", HTTP_GET, []() {
        systemState.ledEnabled = !systemState.ledEnabled;
        Serial.println("LED toggled: " + String(systemState.ledEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Toggle Spotlight
    server.on("/toggle-spotlight", HTTP_GET, []() {
        systemState.spotlightEnabled = !systemState.spotlightEnabled;
        Serial.println("Spotlight toggled: " + String(systemState.spotlightEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    // Status API
    server.on("/status", HTTP_GET, []() {
        String json = "{";
        json += "\"uptime\":" + String(systemState.uptime) + ",";
        json += "\"freeHeap\":" + String(systemState.freeHeap) + ",";
        json += "\"ledEnabled\":" + String(systemState.ledEnabled ? "true" : "false") + ",";
        json += "\"spotlightEnabled\":" + String(systemState.spotlightEnabled ? "true" : "false");
        json += "}";
        server.send(200, "application/json", json);
    });
    
    Serial.println("Web server routes configured");
}