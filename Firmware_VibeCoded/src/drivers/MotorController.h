/**
 * @file MotorController.h
 * @brief DRV8833 Motor Driver Controller for R6 Recon Drone
 * @author Peter
 * 
 * Controls two motors using DRV8833 dual H-bridge driver
 * Features smooth acceleration, deceleration, and safety limits
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class MotorController {
private:
    // Motor states
    float currentLeftSpeed;
    float currentRightSpeed;
    float targetLeftSpeed;  
    float targetRightSpeed;
    
    // Safety and control
    bool motorsEnabled;
    bool emergencyStop;
    uint32_t lastCommandTime;
    
    // Smooth control
    bool smoothControlEnabled;
    float accelerationRate;
    float decelerationRate;
    
    // PWM channels (ESP32 LEDC)
    uint8_t pwmChannels[4];
    bool initialized;
    
    // Internal methods
    void setPWM(uint8_t channel, int16_t speed);
    void updateMotorPWM();
    float applySmoothing(float current, float target, float rate);
    
public:
    MotorController();
    
    // Initialization
    bool begin();
    void reset();
    
    // Basic motor control
    void setMotorSpeeds(float leftSpeed, float rightSpeed);
    void setLeftMotorSpeed(float speed);
    void setRightMotorSpeed(float speed);
    void stop();
    void emergencyStopMotors();
    
    // Advanced control
    void setSmoothControl(bool enabled, float accelRate = ACCELERATION_RATE, 
                         float decelRate = DECELERATION_RATE);
    void updateSmoothControl();
    
    // Movement primitives
    void moveForward(float speed = SPEED_FORWARD);
    void moveBackward(float speed = SPEED_BACKWARD);
    void turnLeft(float speed = SPEED_TURN_LEFT);
    void turnRight(float speed = SPEED_TURN_RIGHT);
    void pivotLeft(float speed);
    void pivotRight(float speed);
    
    // Status and monitoring
    float getLeftSpeed() const { return currentLeftSpeed; }
    float getRightSpeed() const { return currentRightSpeed; }
    float getTargetLeftSpeed() const { return targetLeftSpeed; }
    float getTargetRightSpeed() const { return targetRightSpeed; }
    bool isMoving() const;
    bool isEnabled() const { return motorsEnabled; }
    bool isInitialized() const { return initialized; }
    
    // Safety features
    void enable();
    void disable();
    void setEmergencyStop(bool emergency);
    bool checkTimeout(uint32_t timeoutMs = CONTROL_TIMEOUT_MS);
    
    // Configuration
    void setSpeedLimits(float maxSpeed);
    void setDeadband(float deadband);
    void calibrateMotors();
    
    // Diagnostics
    void printStatus();
    float getPowerConsumption(); // Estimated power consumption
};

#endif // MOTOR_CONTROLLER_H