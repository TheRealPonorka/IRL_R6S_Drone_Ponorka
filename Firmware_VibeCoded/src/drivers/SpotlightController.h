/**
 * @file SpotlightController.h
 * @brief Spotlight LED Controller for R6 Recon Drone
 * @author Peter
 */

#ifndef SPOTLIGHT_CONTROLLER_H
#define SPOTLIGHT_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class SpotlightController {
private:
    bool initialized;
    bool enabled;
    uint8_t currentBrightness;
    uint8_t targetBrightness;
    
    // PWM configuration
    uint8_t pwmChannel;
    
    void updatePWM();

public:
    SpotlightController();
    
    // Initialization
    bool begin();
    
    // Control
    void turnOn();
    void turnOff();
    void toggle();
    void setState(bool state);
    
    // Brightness control
    void setBrightness(uint8_t brightness);
    void fadeIn(uint16_t duration = 500);
    void fadeOut(uint16_t duration = 500);
    
    // Status
    bool isEnabled() const { return enabled; }
    bool isInitialized() const { return initialized; }
    uint8_t getBrightness() const { return currentBrightness; }
    
    // Update (call in loop for smooth fading)
    void update();
};

#endif // SPOTLIGHT_CONTROLLER_H