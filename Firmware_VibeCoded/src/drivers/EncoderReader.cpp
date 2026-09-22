/**
 * @file EncoderReader.cpp
 * @brief Quadrature Encoder Reader Implementation
 * @author Peter
 */

#include "EncoderReader.h"

// Global instance for ISR access
EncoderReader* encoderInstance = nullptr;

EncoderReader::EncoderReader()
    : leftPinA(ENCODER_LEFT_A_PIN), leftPinB(ENCODER_LEFT_B_PIN),
      rightPinA(ENCODER_RIGHT_A_PIN), rightPinB(ENCODER_RIGHT_B_PIN),
      leftCount(0), rightCount(0),
      lastLeftA(0), lastLeftB(0), lastRightA(0), lastRightB(0),
      lastUpdateTime(0), leftSpeed(0), rightSpeed(0),
      lastLeftCount(0), lastRightCount(0), initialized(false) {
}

bool EncoderReader::begin() {
    DEBUG_PRINTLN("Initializing Encoders...");
    
    // Set global instance for ISR access
    encoderInstance = this;
    
    // Configure pins as inputs with pullup
    pinMode(leftPinA, INPUT_PULLUP);
    pinMode(leftPinB, INPUT_PULLUP);
    pinMode(rightPinA, INPUT_PULLUP);
    pinMode(rightPinB, INPUT_PULLUP);
    
    // Read initial states
    lastLeftA = digitalRead(leftPinA);
    lastLeftB = digitalRead(leftPinB);
    lastRightA = digitalRead(rightPinA);
    lastRightB = digitalRead(rightPinB);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(leftPinA), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightPinA), rightEncoderISR, CHANGE);
    
    lastUpdateTime = millis();
    initialized = true;
    
    DEBUG_PRINTLN("Encoders initialized");
    return true;
}

void EncoderReader::update() {
    if (!initialized) return;
    
    uint32_t currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
    
    if (deltaTime >= 0.1f) { // Update speed every 100ms
        // Calculate speeds (counts per second)
        leftSpeed = (leftCount - lastLeftCount) / deltaTime;
        rightSpeed = (rightCount - lastRightCount) / deltaTime;
        
        lastLeftCount = leftCount;
        lastRightCount = rightCount;
        lastUpdateTime = currentTime;
    }
}

void EncoderReader::resetCounts() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    lastLeftCount = 0;
    lastRightCount = 0;
    interrupts();
}

// Left encoder ISR
void IRAM_ATTR EncoderReader::leftEncoderISR() {
    if (!encoderInstance) return;
    
    int currentA = digitalRead(encoderInstance->leftPinA);
    int currentB = digitalRead(encoderInstance->leftPinB);
    
    // Quadrature decoding
    if (currentA != encoderInstance->lastLeftA) {
        if (currentA == currentB) {
            encoderInstance->leftCount++;
        } else {
            encoderInstance->leftCount--;
        }
    }
    
    encoderInstance->lastLeftA = currentA;
    encoderInstance->lastLeftB = currentB;
}

// Right encoder ISR
void IRAM_ATTR EncoderReader::rightEncoderISR() {
    if (!encoderInstance) return;
    
    int currentA = digitalRead(encoderInstance->rightPinA);
    int currentB = digitalRead(encoderInstance->rightPinB);
    
    // Quadrature decoding
    if (currentA != encoderInstance->lastRightA) {
        if (currentA == currentB) {
            encoderInstance->rightCount++;
        } else {
            encoderInstance->rightCount--;
        }
    }
    
    encoderInstance->lastRightA = currentA;
    encoderInstance->lastRightB = currentB;
}