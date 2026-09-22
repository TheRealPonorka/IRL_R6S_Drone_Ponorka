/**
 * @file EncoderReader.h
 * @brief Quadrature Encoder Reader for R6 Recon Drone
 * @author Peter
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>
#include "config.h"

class EncoderReader {
private:
    // Encoder pins
    int leftPinA, leftPinB;
    int rightPinA, rightPinB;
    
    // Encoder counts
    volatile int32_t leftCount;
    volatile int32_t rightCount;
    
    // Previous states
    volatile int lastLeftA, lastLeftB;
    volatile int lastRightA, lastRightB;
    
    // Timing
    uint32_t lastUpdateTime;
    
    // Speed calculation
    float leftSpeed, rightSpeed;
    int32_t lastLeftCount, lastRightCount;
    
    bool initialized;

public:
    EncoderReader();
    
    // Initialization
    bool begin();
    
    // Update readings (call regularly)
    void update();
    
    // Get counts
    int32_t getLeftCount() const { return leftCount; }
    int32_t getRightCount() const { return rightCount; }
    
    // Get speeds (counts per second)
    float getLeftSpeed() const { return leftSpeed; }
    float getRightSpeed() const { return rightSpeed; }
    
    // Reset counts
    void resetCounts();
    
    // Status
    bool isInitialized() const { return initialized; }
    
    // Static interrupt handlers
    static void IRAM_ATTR leftEncoderISR();
    static void IRAM_ATTR rightEncoderISR();
};

// Global instance for ISR access
extern EncoderReader* encoderInstance;

#endif // ENCODER_READER_H