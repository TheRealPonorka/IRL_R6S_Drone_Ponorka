/**
 * @file SpotlightController.cpp
 * @brief Spotlight LED Controller Implementation
 * @author Peter
 */

#include "SpotlightController.h"

SpotlightController::SpotlightController()
    : initialized(false), enabled(false), 
      currentBrightness(0), targetBrightness(0),
      pwmChannel(SPOTLIGHT_PWM_CHANNEL) {
}

bool SpotlightController::begin() {
    DEBUG_PRINTLN("Initializing Spotlight Controller...");
    
    // Configure PWM channel
    if (!ledcSetup(pwmChannel, SPOTLIGHT_PWM_FREQ, SPOTLIGHT_PWM_RESOLUTION)) {
        DEBUG_PRINTLN("Failed to setup spotlight PWM channel");
        return false;
    }
    
    // Attach pin to PWM channel
    ledcAttachPin(SPOTLIGHT_PIN, pwmChannel);
    
    // Initialize to off state
    currentBrightness = 0;
    targetBrightness = 0;
    updatePWM();
    
    initialized = true;
    
    DEBUG_PRINTLN("Spotlight Controller initialized");
    return true;
}

void SpotlightController::updatePWM() {
    if (initialized) {
        ledcWrite(pwmChannel, currentBrightness);
    }
}

void SpotlightController::turnOn() {
    if (!initialized) return;
    
    enabled = true;
    targetBrightness = SPOTLIGHT_DEFAULT_BRIGHTNESS;
    currentBrightness = targetBrightness;
    updatePWM();
    
    DEBUG_PRINTLN("Spotlight turned on");
}

void SpotlightController::turnOff() {
    if (!initialized) return;
    
    enabled = false;
    targetBrightness = 0;
    currentBrightness = 0;
    updatePWM();
    
    DEBUG_PRINTLN("Spotlight turned off");
}

void SpotlightController::toggle() {
    if (enabled) {
        turnOff();
    } else {
        turnOn();
    }
}

void SpotlightController::setState(bool state) {
    if (state) {
        turnOn();
    } else {
        turnOff();
    }
}

void SpotlightController::setBrightness(uint8_t brightness) {
    if (!initialized) return;
    
    brightness = constrain(brightness, 0, SPOTLIGHT_MAX_BRIGHTNESS);
    targetBrightness = brightness;
    currentBrightness = brightness;
    
    enabled = (brightness > 0);
    updatePWM();
    
    DEBUG_PRINTF("Spotlight brightness set to: %d\n", brightness);
}

void SpotlightController::fadeIn(uint16_t duration) {
    if (!initialized) return;
    
    enabled = true;
    targetBrightness = SPOTLIGHT_DEFAULT_BRIGHTNESS;
    
    // Simple fade - could be improved with timing
    for (uint8_t i = 0; i <= targetBrightness; i += 5) {
        currentBrightness = i;
        updatePWM();
        delay(duration / (targetBrightness / 5));
    }
    
    currentBrightness = targetBrightness;
    updatePWM();
}

void SpotlightController::fadeOut(uint16_t duration) {
    if (!initialized) return;
    
    uint8_t startBrightness = currentBrightness;
    
    // Simple fade - could be improved with timing
    for (int16_t i = startBrightness; i >= 0; i -= 5) {
        currentBrightness = (uint8_t)max(0, (int)i);
        updatePWM();
        delay(duration / (startBrightness / 5));
    }
    
    currentBrightness = 0;
    targetBrightness = 0;
    enabled = false;
    updatePWM();
}

void SpotlightController::update() {
    // For future smooth fading implementation
    if (!initialized) return;
    
    // Smooth brightness transitions could be implemented here
    if (currentBrightness != targetBrightness) {
        // Implement smooth transitions
        if (currentBrightness < targetBrightness) {
            currentBrightness++;
        } else if (currentBrightness > targetBrightness) {
            currentBrightness--;
        }
        updatePWM();
    }
}