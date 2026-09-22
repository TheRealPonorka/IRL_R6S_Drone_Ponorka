/**
 * @file main_wifi_test.cpp
 * @brief R6 Drone - Exact copy of working simple test + hardware init
 * @author Peter
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

// Simple LED strip for testing (EXACT same as working simple test)
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);

// Web server (EXACT same as working simple test)
WebServer server(80);

// Animation variables (EXACT same as working simple test)
uint32_t lastLEDUpdate = 0;
uint8_t animationStep = 0;
bool ledEnabled = true;
uint8_t ledR = 0, ledG = 255, ledB = 0; // Default green

// Hardware status
bool spotlightInitialized = false;
bool motorsInitialized = false;
bool encodersInitialized = false;

// Function declarations
void setupWebServer();
void updateLEDAnimation();
void initializeExtraHardware();

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=================================");
    Serial.println("R6 Drone - WiFi Test (Based on Working Simple Test)");
    Serial.println("Version: 2.2.0-wifi-test");
    Serial.println("=================================");
    
    // Initialize LED strip (EXACT same as working simple test)
    Serial.println("Initializing LED strip...");
    strip.begin();
    strip.setBrightness(64);  // 25% brightness
    strip.clear();
    strip.show();
    Serial.println("LED strip initialized");
    
    // Initialize extra hardware (NEW - but safe)
    Serial.println("Initializing extra hardware...");
    initializeExtraHardware();
    
    // Initialize WiFi in AP mode (EXACT same as working simple test)
    Serial.println("Starting WiFi Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("R6_Recon_Drone", "ReconDrone123");
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.println("Connect to WiFi: R6_Recon_Drone");
    Serial.println("Password: ReconDrone123");
    
    // Setup web server (EXACT same as working simple test)
    setupWebServer();
    server.begin();
    
    Serial.println("=================================");
    Serial.println("WiFi test mode ready!");
    Serial.println("LED strip should show green animation");
    Serial.print("Web interface: http://");
    Serial.println(IP);
    Serial.println("=================================");
}

void initializeExtraHardware() {
    // Spotlight
    pinMode(SPOTLIGHT_PIN, OUTPUT);
    digitalWrite(SPOTLIGHT_PIN, LOW);
    spotlightInitialized = true;
    Serial.println("✅ Spotlight initialized");
    
    // Motors (just pin setup, no PWM to avoid conflicts)
    pinMode(MOTOR_A_PIN1, OUTPUT);
    pinMode(MOTOR_A_PIN2, OUTPUT);
    pinMode(MOTOR_B_PIN1, OUTPUT);
    pinMode(MOTOR_B_PIN2, OUTPUT);
    digitalWrite(MOTOR_A_PIN1, LOW);
    digitalWrite(MOTOR_A_PIN2, LOW);
    digitalWrite(MOTOR_B_PIN1, LOW);
    digitalWrite(MOTOR_B_PIN2, LOW);
    motorsInitialized = true;
    Serial.println("✅ Motor pins initialized");
    
    // Encoders
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);
    encodersInitialized = true;
    Serial.println("✅ Encoder pins initialized");
}

void setupWebServer() {
    // Main page (EXACT same structure as working simple test)
    server.on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>R6 Recon Drone - WiFi Test</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<style>";
        html += "body { font-family: Arial; text-align: center; margin: 20px; background: #1a1a1a; color: white; }";
        html += "h1 { color: #00ff00; }";
        html += ".button { background: #333; border: 2px solid #00ff00; color: white; padding: 15px 30px; margin: 10px; font-size: 18px; border-radius: 10px; cursor: pointer; }";
        html += ".button:hover { background: #00ff00; color: black; }";
        html += ".button.active { background: #00ff00; color: black; }";
        html += ".status { background: #333; padding: 20px; margin: 20px; border-radius: 10px; }";
        html += "</style></head><body>";
        html += "<h1>🎯 R6 Recon Drone</h1>";
        html += "<h2>WiFi Test Mode</h2>";
        html += "<div class='status'>";
        html += "<p><strong>WiFi:</strong> Connected (AP Mode)</p>";
        html += "<p><strong>LED Strip:</strong> " + String(ledEnabled ? "ON" : "OFF") + "</p>";
        html += "<p><strong>Animation:</strong> Middle-Out Pattern</p>";
        html += "<p><strong>Extra Hardware:</strong> Initialized</p>";
        html += "<p><strong>Uptime:</strong> " + String(millis() / 1000) + " seconds</p>";
        html += "</div>";
        html += "<button class='button" + String(ledEnabled ? " active" : "") + "' onclick='toggleLEDs()'>Toggle LED Strip</button>";
        html += "<br><button class='button' onclick='window.location.reload()'>Refresh Status</button>";
        html += "<script>";
        html += "function toggleLEDs() { fetch('/toggle-leds').then(() => window.location.reload()); }";
        html += "</script>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });
    
    // Toggle LEDs (EXACT same as working simple test)
    server.on("/toggle-leds", HTTP_GET, []() {
        ledEnabled = !ledEnabled;
        Serial.println("LED strip toggled: " + String(ledEnabled ? "ON" : "OFF"));
        server.send(200, "text/plain", "OK");
    });
    
    Serial.println("Web server routes configured");
}

void updateLEDAnimation() {
    // EXACT same LED animation as working simple test
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
    // EXACT same loop as working simple test
    // Handle web server requests
    server.handleClient();
    
    // Update LED animation
    updateLEDAnimation();
    
    // Print status every 5 seconds
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        lastStatus = millis();
        Serial.println("WiFi test running... LED: " + String(ledEnabled ? "ON" : "OFF"));
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
        Serial.printf("Web server: http://%s\n", WiFi.softAPIP().toString().c_str());
    }
    
    // Small delay
    delay(10);
}