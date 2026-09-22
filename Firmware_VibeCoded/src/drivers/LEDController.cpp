/**
 * @file LEDController.cpp
 * @brief WS2812B LED Strip Controller Implementation
 * @author Peter
 */

#include "LEDController.h"
#include <math.h>

#ifndef TWO_PI
#define TWO_PI (2.0 * PI)
#endif

// Predefined colors
const LEDColor LEDController::COLOR_OFF(0, 0, 0);
const LEDColor LEDController::COLOR_WHITE(255, 255, 255);
const LEDColor LEDController::COLOR_RED(255, 0, 0);
const LEDColor LEDController::COLOR_GREEN(0, 255, 0);
const LEDColor LEDController::COLOR_BLUE(0, 0, 255);
const LEDColor LEDController::COLOR_YELLOW(255, 255, 0);
const LEDColor LEDController::COLOR_CYAN(0, 255, 255);
const LEDColor LEDController::COLOR_MAGENTA(255, 0, 255);
const LEDColor LEDController::COLOR_ORANGE(255, 165, 0);
const LEDColor LEDController::COLOR_R6_ORANGE(255, 140, 0);
const LEDColor LEDController::COLOR_R6_BLUE(0, 120, 255);
const LEDColor LEDController::COLOR_WARM_WHITE(255, 220, 180);

LEDController::LEDController()
    : strip(nullptr), currentPattern(LED_PATTERN_OFF),
      primaryColor(COLOR_R6_ORANGE), secondaryColor(COLOR_OFF),
      brightness(LED_BRIGHTNESS), enabled(false), initialized(false),
      lastUpdate(0), animationSpeed(LED_ANIMATION_SPEED),
      animationStep(0), animationDirection(1),
      breathingPhase(0), rainbowOffset(0), chasePosition(0),
      emergencyState(false) {
}

LEDController::~LEDController() {
    if (strip) {
        delete strip;
    }
}

bool LEDController::begin() {
    DEBUG_PRINTLN("Initializing WS2812B LED Controller...");
    
    // Create NeoPixel strip object
    strip = new Adafruit_NeoPixel(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);
    
    if (!strip) {
        DEBUG_PRINTLN("Failed to create NeoPixel object");
        return false;
    }
    
    // Initialize the strip
    strip->begin();
    strip->setBrightness(brightness);
    strip->clear();
    strip->show();
    
    initialized = true;
    enabled = true;
    lastUpdate = millis();
    
    // Set default startup pattern to middle-out animation with green color at 25% brightness
    primaryColor = LEDColor(0, 255, 0);  // Green
    setBrightness(LED_BRIGHTNESS);       // 64 (25%)
    setMiddleOutPattern();
    
    DEBUG_PRINTLN("WS2812B LED Controller initialized");
    return true;
}

void LEDController::reset() {
    if (!initialized) return;
    
    currentPattern = LED_PATTERN_OFF;
    animationStep = 0;
    breathingPhase = 0;
    rainbowOffset = 0;
    chasePosition = 0;
    emergencyState = false;
    
    strip->clear();
    strip->show();
}

void LEDController::turnOn() {
    enabled = true;
    if (currentPattern == LED_PATTERN_OFF) {
        setPattern(LED_PATTERN_SOLID);
    }
}

void LEDController::turnOff() {
    enabled = false;
    currentPattern = LED_PATTERN_OFF;
    if (initialized) {
        strip->clear();
        strip->show();
    }
}

void LEDController::setBrightness(uint8_t newBrightness) {
    brightness = constrain(newBrightness, 0, 255);
    if (initialized) {
        strip->setBrightness(brightness);
        strip->show();
    }
}

void LEDController::setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
    setSolidColor(LEDColor(r, g, b));
}

void LEDController::setSolidColor(LEDColor color) {
    primaryColor = color;
    setPattern(LED_PATTERN_SOLID);
}

void LEDController::setSolidColor(uint32_t color) {
    setSolidColor(LEDColor::fromUint32(color));
}

void LEDController::setPattern(LEDPattern pattern) {
    currentPattern = pattern;
    animationStep = 0;
    breathingPhase = 0;
    rainbowOffset = 0;
    chasePosition = 0;
    lastUpdate = millis();
}

void LEDController::setPatternColors(LEDColor primary, LEDColor secondary) {
    primaryColor = primary;
    secondaryColor = secondary;
}

void LEDController::setAnimationSpeed(uint32_t speedMs) {
    animationSpeed = speedMs;
}

void LEDController::setEmergencyPattern() {
    emergencyState = true;
    setPattern(LED_PATTERN_EMERGENCY);
}

void LEDController::setBalancingPattern(uint8_t r, uint8_t g, uint8_t b) {
    primaryColor = LEDColor(r, g, b);
    setPattern(LED_PATTERN_BALANCING);
}

void LEDController::setStartupSequence() {
    setPattern(LED_PATTERN_STARTUP);
}

void LEDController::setR6ThemePattern() {
    primaryColor = COLOR_R6_ORANGE;
    secondaryColor = COLOR_R6_BLUE;
    setPattern(LED_PATTERN_R6_THEME);
}

void LEDController::setRainbowPattern() {
    setPattern(LED_PATTERN_RAINBOW);
}

void LEDController::setBreathingPattern(LEDColor color) {
    primaryColor = color;
    setPattern(LED_PATTERN_BREATHING);
}

void LEDController::setChasePattern(LEDColor color) {
    primaryColor = color;
    setPattern(LED_PATTERN_CHASE);
}

void LEDController::setMiddleOutPattern(LEDColor color) {
    primaryColor = color;
    setPattern(LED_PATTERN_MIDDLE_OUT_ANIMATION);
}

void LEDController::update() {
    if (!initialized || !enabled) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastUpdate < animationSpeed) return;
    
    lastUpdate = currentTime;
    
    switch (currentPattern) {
        case LED_PATTERN_OFF:
            strip->clear();
            break;
            
        case LED_PATTERN_SOLID:
            updateSolid();
            break;
            
        case LED_PATTERN_BREATHING:
            updateBreathing();
            break;
            
        case LED_PATTERN_RAINBOW:
            updateRainbow();
            break;
            
        case LED_PATTERN_CHASE:
            updateChase();
            break;
            
        case LED_PATTERN_EMERGENCY:
            updateEmergency();
            break;
            
        case LED_PATTERN_BALANCING:
            updateBalancing();
            break;
            
        case LED_PATTERN_STARTUP:
            updateStartup();
            break;
            
        case LED_PATTERN_R6_THEME:
            updateR6Theme();
            break;
            
        case LED_PATTERN_MIDDLE_OUT_ANIMATION:
            updateMiddleOutAnimation();
            break;
    }
    
    strip->show();
}

void LEDController::updateSolid() {
    setAllPixels(primaryColor);
}

void LEDController::updateBreathing() {
    breathingPhase += 0.1f;
    if (breathingPhase >= TWO_PI) breathingPhase = 0;
    
    float breathFactor = (sin(breathingPhase) + 1.0f) / 2.0f;
    LEDColor breathColor(
        (uint8_t)(primaryColor.r * breathFactor),
        (uint8_t)(primaryColor.g * breathFactor),
        (uint8_t)(primaryColor.b * breathFactor)
    );
    
    setAllPixels(breathColor);
}

void LEDController::updateRainbow() {
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        uint8_t wheelPos = ((i * 256 / LED_COUNT) + rainbowOffset) & 255;
        setPixelColor(i, wheel(wheelPos));
    }
    
    rainbowOffset += LED_RAINBOW_SPEED;
    if (rainbowOffset >= 256) rainbowOffset = 0;
}

void LEDController::updateChase() {
    strip->clear();
    
    // Create chase pattern with multiple dots
    for (int i = 0; i < 3; i++) {
        uint16_t pos = (chasePosition + i * (LED_COUNT / 3)) % LED_COUNT;
        setPixelColor(pos, primaryColor);
    }
    
    chasePosition++;
    if (chasePosition >= LED_COUNT) chasePosition = 0;
}

void LEDController::updateEmergency() {
    // Fast red strobe
    if (animationStep % 2 == 0) {
        setAllPixels(COLOR_RED);
    } else {
        setAllPixels(COLOR_OFF);
    }
    
    animationStep++;
    if (animationStep >= 10) {
        animationStep = 0;
        emergencyState = false;
        setPattern(LED_PATTERN_SOLID);
    }
}

void LEDController::updateBalancing() {
    // Gentle pulsing with primary color
    float pulseFactor = (sin(breathingPhase * 2.0f) + 1.0f) / 2.0f;
    pulseFactor = pulseFactor * 0.5f + 0.5f; // Keep minimum brightness at 50%
    
    LEDColor pulseColor(
        (uint8_t)(primaryColor.r * pulseFactor),
        (uint8_t)(primaryColor.g * pulseFactor),
        (uint8_t)(primaryColor.b * pulseFactor)
    );
    
    setAllPixels(pulseColor);
    breathingPhase += 0.05f;
    if (breathingPhase >= TWO_PI) breathingPhase = 0;
}

void LEDController::updateStartup() {
    // Rainbow wipe effect
    if (animationStep < LED_COUNT) {
        uint8_t wheelPos = (animationStep * 256 / LED_COUNT) & 255;
        setPixelColor(animationStep, wheel(wheelPos));
        animationStep++;
    } else {
        // Startup complete, switch to R6 theme
        setR6ThemePattern();
    }
}

void LEDController::updateR6Theme() {
    // Alternating R6 orange and blue chase
    strip->clear();
    
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        if ((i + chasePosition) % 4 < 2) {
            setPixelColor(i, primaryColor);  // R6 Orange
        } else {
            setPixelColor(i, secondaryColor); // R6 Blue
        }
    }
    
    chasePosition++;
    if (chasePosition >= 4) chasePosition = 0;
}

void LEDController::updateMiddleOutAnimation() {
    // Middle-out animation for 6 LEDs (0-5)
    // Animation sequence (30ms between each step):
    // Step 0: no LEDs
    // Step 1: LED 2-3
    // Step 2: LED 1-2-3-4  
    // Step 3: LED 0-1-2-3-4-5
    // Step 4: LED 1-2-3-4
    // Step 5: LED 2-3
    // Step 6: no LEDs (back to step 0)
    
    strip->clear();  // Start with all LEDs off
    
    switch (animationStep) {
        case 0:
            // No LEDs - already cleared above
            break;
            
        case 1:
            // LED 2-3
            setPixelColor(2, primaryColor);
            setPixelColor(3, primaryColor);
            break;
            
        case 2:
            // LED 1-2-3-4
            setPixelColor(1, primaryColor);
            setPixelColor(2, primaryColor);
            setPixelColor(3, primaryColor);
            setPixelColor(4, primaryColor);
            break;
            
        case 3:
            // LED 0-1-2-3-4-5 (all LEDs)
            for (uint16_t i = 0; i < LED_COUNT; i++) {
                setPixelColor(i, primaryColor);
            }
            break;
            
        case 4:
            // LED 1-2-3-4
            setPixelColor(1, primaryColor);
            setPixelColor(2, primaryColor);
            setPixelColor(3, primaryColor);
            setPixelColor(4, primaryColor);
            break;
            
        case 5:
            // LED 2-3
            setPixelColor(2, primaryColor);
            setPixelColor(3, primaryColor);
            break;
    }
    
    // Advance to next animation step
    animationStep++;
    if (animationStep >= 6) {
        animationStep = 0;  // Loop back to start (no extra delay)
    }
}

void LEDController::strobeEffect(LEDColor color, uint8_t flashes) {
    if (!initialized) return;
    
    for (uint8_t i = 0; i < flashes; i++) {
        setAllPixels(color);
        strip->show();
        delay(50);
        
        setAllPixels(COLOR_OFF);
        strip->show();
        delay(50);
    }
}

void LEDController::wipeEffect(LEDColor color, bool reverse) {
    if (!initialized) return;
    
    strip->clear();
    
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        uint16_t pixel = reverse ? (LED_COUNT - 1 - i) : i;
        setPixelColor(pixel, color);
        strip->show();
        delay(50);
    }
}

void LEDController::sparkleEffect(LEDColor color, uint8_t sparkles) {
    if (!initialized) return;
    
    strip->clear();
    
    for (uint8_t i = 0; i < sparkles; i++) {
        uint16_t pixel = random(LED_COUNT);
        setPixelColor(pixel, color);
    }
    
    strip->show();
}

void LEDController::fireEffect() {
    // Simple fire simulation
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        uint8_t heat = random(160, 255);
        LEDColor fireColor(heat, heat * 0.4f, 0);
        setPixelColor(i, fireColor);
    }
}

void LEDController::policeEffect() {
    // Red and blue alternating
    uint16_t half = LED_COUNT / 2;
    
    if (animationStep % 2 == 0) {
        for (uint16_t i = 0; i < half; i++) {
            setPixelColor(i, COLOR_RED);
        }
        for (uint16_t i = half; i < LED_COUNT; i++) {
            setPixelColor(i, COLOR_OFF);
        }
    } else {
        for (uint16_t i = 0; i < half; i++) {
            setPixelColor(i, COLOR_OFF);
        }
        for (uint16_t i = half; i < LED_COUNT; i++) {
            setPixelColor(i, COLOR_BLUE);
        }
    }
}

void LEDController::showBatteryLevel(float percentage) {
    percentage = constrain(percentage, 0.0f, 100.0f);
    uint16_t litPixels = (uint16_t)((percentage / 100.0f) * LED_COUNT);
    
    strip->clear();
    
    for (uint16_t i = 0; i < litPixels; i++) {
        LEDColor color;
        if (percentage > 50) {
            color = COLOR_GREEN;
        } else if (percentage > 25) {
            color = COLOR_YELLOW;
        } else {
            color = COLOR_RED;
        }
        
        setPixelColor(i, color);
    }
    
    strip->show();
}

void LEDController::showWiFiStatus(bool connected) {
    if (connected) {
        setAllPixels(COLOR_GREEN);
    } else {
        setBreathingPattern(COLOR_RED);
    }
}

void LEDController::showSystemStatus(bool balanced, bool emergency) {
    if (emergency) {
        setEmergencyPattern();
    } else if (balanced) {
        setSolidColor(COLOR_GREEN);
    } else {
        setBalancingPattern(primaryColor.r, primaryColor.g, primaryColor.b);
    }
}

LEDColor LEDController::wheel(uint8_t pos) {
    pos = 255 - pos;
    if (pos < 85) {
        return LEDColor(255 - pos * 3, 0, pos * 3);
    } else if (pos < 170) {
        pos -= 85;
        return LEDColor(0, pos * 3, 255 - pos * 3);
    } else {
        pos -= 170;
        return LEDColor(pos * 3, 255 - pos * 3, 0);
    }
}

LEDColor LEDController::interpolateColor(LEDColor color1, LEDColor color2, float ratio) {
    ratio = constrain(ratio, 0.0f, 1.0f);
    
    return LEDColor(
        (uint8_t)(color1.r + (color2.r - color1.r) * ratio),
        (uint8_t)(color1.g + (color2.g - color1.g) * ratio),
        (uint8_t)(color1.b + (color2.b - color1.b) * ratio)
    );
}

uint8_t LEDController::gamma8(uint8_t x) {
    // Gamma correction for better color perception
    // Simple gamma correction without lookup table for now
    return (uint8_t)(pow(x / 255.0, 2.2) * 255 + 0.5);
}

void LEDController::setPixelColor(uint16_t pixel, LEDColor color) {
    if (initialized && pixel < LED_COUNT) {
        strip->setPixelColor(pixel, color.r, color.g, color.b);
    }
}

void LEDController::setAllPixels(LEDColor color) {
    if (!initialized) return;
    
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        setPixelColor(i, color);
    }
}

void LEDController::fadePixel(uint16_t pixel, LEDColor color, float fadeFactor) {
    LEDColor fadedColor(
        (uint8_t)(color.r * fadeFactor),
        (uint8_t)(color.g * fadeFactor),
        (uint8_t)(color.b * fadeFactor)
    );
    setPixelColor(pixel, fadedColor);
}

uint16_t LEDController::getPixelCount() const {
    return LED_COUNT;
}

LEDColor LEDController::HSVtoRGB(float h, float s, float v) {
    float c = v * s;
    float x = c * (1 - abs(fmod(h / 60.0f, 2) - 1));
    float m = v - c;
    
    float r, g, b;
    
    if (h >= 0 && h < 60) {
        r = c; g = x; b = 0;
    } else if (h >= 60 && h < 120) {
        r = x; g = c; b = 0;
    } else if (h >= 120 && h < 180) {
        r = 0; g = c; b = x;
    } else if (h >= 180 && h < 240) {
        r = 0; g = x; b = c;
    } else if (h >= 240 && h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    return LEDColor(
        (uint8_t)((r + m) * 255),
        (uint8_t)((g + m) * 255),
        (uint8_t)((b + m) * 255)
    );
}

void LEDController::RGBtoHSV(LEDColor color, float& h, float& s, float& v) {
    float r = color.r / 255.0f;
    float g = color.g / 255.0f;
    float b = color.b / 255.0f;
    
    float cmax = max(r, max(g, b));
    float cmin = min(r, min(g, b));
    float diff = cmax - cmin;
    
    // Hue calculation
    if (cmax == cmin) {
        h = 0;
    } else if (cmax == r) {
        h = fmod(60 * ((g - b) / diff) + 360, 360);
    } else if (cmax == g) {
        h = fmod(60 * ((b - r) / diff) + 120, 360);
    } else if (cmax == b) {
        h = fmod(60 * ((r - g) / diff) + 240, 360);
    }
    
    // Saturation calculation
    if (cmax == 0) {
        s = 0;
    } else {
        s = diff / cmax;
    }
    
    // Value calculation
    v = cmax;
}
