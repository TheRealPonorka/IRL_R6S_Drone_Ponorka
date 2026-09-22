/**
 * @file LEDController.h
 * @brief WS2812B LED Strip Controller for R6 Recon Drone
 * @author Peter
 * 
 * Controls WS2812B NeoPixel LED strip with various animation patterns
 * Optimized for non-blocking animations using millis() timing
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

// LED animation patterns
enum LEDPattern {
    LED_PATTERN_OFF,
    LED_PATTERN_SOLID,
    LED_PATTERN_BREATHING,
    LED_PATTERN_RAINBOW,
    LED_PATTERN_CHASE,
    LED_PATTERN_EMERGENCY,
    LED_PATTERN_BALANCING,
    LED_PATTERN_STARTUP,
    LED_PATTERN_R6_THEME,
    LED_PATTERN_MIDDLE_OUT_ANIMATION  // Custom middle-out animation
};

// LED color structure
struct LEDColor {
    uint8_t r, g, b;
    
    LEDColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) 
        : r(red), g(green), b(blue) {}
    
    uint32_t toUint32() const {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    
    static LEDColor fromUint32(uint32_t color) {
        return LEDColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
    }
};

class LEDController {
private:
    Adafruit_NeoPixel* strip;
    
    // Current state
    LEDPattern currentPattern;
    LEDColor primaryColor;
    LEDColor secondaryColor;
    uint8_t brightness;
    bool enabled;
    bool initialized;
    
    // Animation state
    uint32_t lastUpdate;
    uint32_t animationSpeed;
    uint16_t animationStep;
    uint16_t animationDirection;
    
    // Pattern-specific variables
    float breathingPhase;
    uint16_t rainbowOffset;
    uint8_t chasePosition;
    bool emergencyState;
    
    // Internal methods
    void updateSolid();
    void updateBreathing();
    void updateRainbow();
    void updateChase();
    void updateEmergency();
    void updateBalancing();
    void updateStartup();
    void updateR6Theme();
    void updateMiddleOutAnimation();
    
    // Utility methods
    LEDColor wheel(uint8_t pos);
    LEDColor interpolateColor(LEDColor color1, LEDColor color2, float ratio);
    uint8_t gamma8(uint8_t x);
    void setPixelColor(uint16_t pixel, LEDColor color);
    void setAllPixels(LEDColor color);
    void fadePixel(uint16_t pixel, LEDColor color, float fadeFactor);
    
public:
    LEDController();
    ~LEDController();
    
    // Initialization
    bool begin();
    void reset();
    
    // Basic control
    void turnOn();
    void turnOff();
    void setBrightness(uint8_t brightness);
    void setSolidColor(uint8_t r, uint8_t g, uint8_t b);
    void setSolidColor(LEDColor color);
    void setSolidColor(uint32_t color);
    
    // Pattern control
    void setPattern(LEDPattern pattern);
    void setPatternColors(LEDColor primary, LEDColor secondary = LEDColor(0, 0, 0));
    void setAnimationSpeed(uint32_t speedMs);
    
    // Predefined patterns
    void setEmergencyPattern();
    void setBalancingPattern(uint8_t r, uint8_t g, uint8_t b);
    void setStartupSequence();
    void setR6ThemePattern();
    void setRainbowPattern();
    void setBreathingPattern(LEDColor color);
    void setChasePattern(LEDColor color);
    void setMiddleOutPattern(LEDColor color = LEDColor(0, 255, 0));
    
    // Advanced effects
    void strobeEffect(LEDColor color, uint8_t flashes = 3);
    void wipeEffect(LEDColor color, bool reverse = false);
    void sparkleEffect(LEDColor color, uint8_t sparkles = 3);
    void fireEffect();
    void policeEffect();
    
    // Status indication
    void showBatteryLevel(float percentage);
    void showWiFiStatus(bool connected);
    void showSystemStatus(bool balanced, bool emergency);
    
    // Animation update (call in main loop)
    void update();
    
    // Status getters
    bool isEnabled() const { return enabled; }
    bool isInitialized() const { return initialized; }
    LEDPattern getCurrentPattern() const { return currentPattern; }
    uint8_t getBrightness() const { return brightness; }
    LEDColor getPrimaryColor() const { return primaryColor; }
    uint16_t getPixelCount() const;
    
    // Configuration
    void setGammaCorrection(bool enable);
    void setPixelOrder(neoPixelType order);
    
    // Utility functions
    static LEDColor HSVtoRGB(float h, float s, float v);
    static void RGBtoHSV(LEDColor color, float& h, float& s, float& v);
    
    // Predefined colors
    static const LEDColor COLOR_OFF;
    static const LEDColor COLOR_WHITE;
    static const LEDColor COLOR_RED;
    static const LEDColor COLOR_GREEN;
    static const LEDColor COLOR_BLUE;
    static const LEDColor COLOR_YELLOW;
    static const LEDColor COLOR_CYAN;
    static const LEDColor COLOR_MAGENTA;
    static const LEDColor COLOR_ORANGE;
    static const LEDColor COLOR_R6_ORANGE;
    static const LEDColor COLOR_R6_BLUE;
    static const LEDColor COLOR_WARM_WHITE;
};

#endif // LED_CONTROLLER_H