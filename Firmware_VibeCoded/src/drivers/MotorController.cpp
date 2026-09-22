/**
 * @file MotorController.cpp
 * @brief DRV8833 Motor Driver Implementation
 * @author Peter
 */

#include "MotorController.h"

MotorController::MotorController()
    : currentLeftSpeed(0), currentRightSpeed(0),
      targetLeftSpeed(0), targetRightSpeed(0),
      motorsEnabled(false), emergencyStop(false),
      lastCommandTime(0), smoothControlEnabled(true),
      accelerationRate(ACCELERATION_RATE), decelerationRate(DECELERATION_RATE),
      initialized(false) {
}

bool MotorController::begin() {
    DEBUG_PRINTLN("Initializing DRV8833 Motor Controller...");
    
    // Initialize PWM channels for motor control
    pwmChannels[0] = MOTOR_PWM_CHANNEL_A1; // Motor A Forward
    pwmChannels[1] = MOTOR_PWM_CHANNEL_A2; // Motor A Reverse  
    pwmChannels[2] = MOTOR_PWM_CHANNEL_B1; // Motor B Forward
    pwmChannels[3] = MOTOR_PWM_CHANNEL_B2; // Motor B Reverse
    
    // Configure PWM channels
    for (int i = 0; i < 4; i++) {
        if (!ledcSetup(pwmChannels[i], MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION)) {
            DEBUG_PRINTF("Failed to setup PWM channel %d\n", pwmChannels[i]);
            return false;
        }
    }
    
    // Attach pins to PWM channels
    ledcAttachPin(MOTOR_A_PIN1, MOTOR_PWM_CHANNEL_A1);
    ledcAttachPin(MOTOR_A_PIN2, MOTOR_PWM_CHANNEL_A2);
    ledcAttachPin(MOTOR_B_PIN1, MOTOR_PWM_CHANNEL_B1);
    ledcAttachPin(MOTOR_B_PIN2, MOTOR_PWM_CHANNEL_B2);
    
    // Initialize all motors to stopped state
    stop();
    
    initialized = true;
    motorsEnabled = true;
    lastCommandTime = millis();
    
    DEBUG_PRINTLN("DRV8833 Motor Controller initialized");
    return true;
}

void MotorController::reset() {
    stop();
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    emergencyStop = false;
    lastCommandTime = millis();
}

void MotorController::setMotorSpeeds(float leftSpeed, float rightSpeed) {
    if (!initialized || !motorsEnabled || emergencyStop) {
        return;
    }
    
    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    
    // Apply deadband
    if (abs(leftSpeed) < MOTOR_DEADBAND) leftSpeed = 0;
    if (abs(rightSpeed) < MOTOR_DEADBAND) rightSpeed = 0;
    
    targetLeftSpeed = leftSpeed;
    targetRightSpeed = rightSpeed;
    lastCommandTime = millis();
    
    if (!smoothControlEnabled) {
        currentLeftSpeed = targetLeftSpeed;
        currentRightSpeed = targetRightSpeed;
        updateMotorPWM();
    }
}

void MotorController::setLeftMotorSpeed(float speed) {
    setMotorSpeeds(speed, targetRightSpeed);
}

void MotorController::setRightMotorSpeed(float speed) {
    setMotorSpeeds(targetLeftSpeed, speed);
}

void MotorController::stop() {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    
    if (!smoothControlEnabled || emergencyStop) {
        currentLeftSpeed = 0;
        currentRightSpeed = 0;
        updateMotorPWM();
    }
}

void MotorController::emergencyStopMotors() {
    emergencyStop = true;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    updateMotorPWM();
    DEBUG_PRINTLN("EMERGENCY STOP: Motors halted!");
}

void MotorController::setSmoothControl(bool enabled, float accelRate, float decelRate) {
    smoothControlEnabled = enabled;
    accelerationRate = accelRate;
    decelerationRate = decelRate;
}

void MotorController::updateSmoothControl() {
    if (!smoothControlEnabled || !initialized) {
        return;
    }
    
    // Apply smooth acceleration/deceleration
    currentLeftSpeed = applySmoothing(currentLeftSpeed, targetLeftSpeed, 
                                     (abs(targetLeftSpeed) > abs(currentLeftSpeed)) ? 
                                     accelerationRate : decelerationRate);
                                     
    currentRightSpeed = applySmoothing(currentRightSpeed, targetRightSpeed,
                                      (abs(targetRightSpeed) > abs(currentRightSpeed)) ? 
                                      accelerationRate : decelerationRate);
    
    updateMotorPWM();
}

float MotorController::applySmoothing(float current, float target, float rate) {
    if (abs(target - current) <= rate) {
        return target;
    }
    
    if (target > current) {
        return current + rate;
    } else {
        return current - rate;
    }
}

void MotorController::updateMotorPWM() {
    if (!initialized) return;
    
    // Left motor (Motor A)
    if (currentLeftSpeed >= 0) {
        // Forward direction
        ledcWrite(MOTOR_PWM_CHANNEL_A1, (uint8_t)abs(currentLeftSpeed));
        ledcWrite(MOTOR_PWM_CHANNEL_A2, 0);
    } else {
        // Reverse direction  
        ledcWrite(MOTOR_PWM_CHANNEL_A1, 0);
        ledcWrite(MOTOR_PWM_CHANNEL_A2, (uint8_t)abs(currentLeftSpeed));
    }
    
    // Right motor (Motor B)
    if (currentRightSpeed >= 0) {
        // Forward direction
        ledcWrite(MOTOR_PWM_CHANNEL_B1, (uint8_t)abs(currentRightSpeed));
        ledcWrite(MOTOR_PWM_CHANNEL_B2, 0);
    } else {
        // Reverse direction
        ledcWrite(MOTOR_PWM_CHANNEL_B1, 0);
        ledcWrite(MOTOR_PWM_CHANNEL_B2, (uint8_t)abs(currentRightSpeed));
    }
}

void MotorController::moveForward(float speed) {
    setMotorSpeeds(speed, speed);
}

void MotorController::moveBackward(float speed) {
    setMotorSpeeds(-speed, -speed);
}

void MotorController::turnLeft(float speed) {
    // Differential turning - slow down left motor
    setMotorSpeeds(speed * TURN_SENSITIVITY, speed);
}

void MotorController::turnRight(float speed) {
    // Differential turning - slow down right motor
    setMotorSpeeds(speed, speed * TURN_SENSITIVITY);
}

void MotorController::pivotLeft(float speed) {
    // Pivot turn - opposite motor directions
    setMotorSpeeds(-speed, speed);
}

void MotorController::pivotRight(float speed) {
    // Pivot turn - opposite motor directions
    setMotorSpeeds(speed, -speed);
}

bool MotorController::isMoving() const {
    return (abs(currentLeftSpeed) > MOTOR_DEADBAND || 
            abs(currentRightSpeed) > MOTOR_DEADBAND);
}

void MotorController::enable() {
    if (initialized && !emergencyStop) {
        motorsEnabled = true;
        DEBUG_PRINTLN("Motors enabled");
    }
}

void MotorController::disable() {
    motorsEnabled = false;
    stop();
    DEBUG_PRINTLN("Motors disabled");
}

void MotorController::setEmergencyStop(bool emergency) {
    emergencyStop = emergency;
    if (emergency) {
        emergencyStopMotors();
    }
}

bool MotorController::checkTimeout(uint32_t timeoutMs) {
    uint32_t timeSinceCommand = millis() - lastCommandTime;
    
    if (timeSinceCommand > timeoutMs) {
        DEBUG_PRINTLN("Motor command timeout - stopping motors");
        stop();
        return true;
    }
    
    return false;
}

void MotorController::setSpeedLimits(float maxSpeed) {
    // This would update MOTOR_MAX_SPEED if it was not const
    // Could implement dynamic speed limiting here
}

void MotorController::setDeadband(float deadband) {
    // This would update MOTOR_DEADBAND if it was not const
    // Could implement dynamic deadband adjustment here
}

void MotorController::calibrateMotors() {
    DEBUG_PRINTLN("Calibrating motors...");
    
    // Motor calibration sequence
    // Test forward direction
    DEBUG_PRINTLN("Testing forward...");
    setMotorSpeeds(100, 100);
    delay(1000);
    stop();
    delay(500);
    
    // Test reverse direction
    DEBUG_PRINTLN("Testing reverse...");
    setMotorSpeeds(-100, -100);
    delay(1000);
    stop();
    delay(500);
    
    // Test turning
    DEBUG_PRINTLN("Testing left turn...");
    pivotLeft(80);
    delay(1000);
    stop();
    delay(500);
    
    DEBUG_PRINTLN("Testing right turn...");
    pivotRight(80);
    delay(1000);
    stop();
    
    DEBUG_PRINTLN("Motor calibration complete");
}

void MotorController::printStatus() {
    DEBUG_PRINTLN("=== Motor Controller Status ===");
    DEBUG_PRINTF("Initialized: %s\n", initialized ? "Yes" : "No");
    DEBUG_PRINTF("Enabled: %s\n", motorsEnabled ? "Yes" : "No");
    DEBUG_PRINTF("Emergency Stop: %s\n", emergencyStop ? "ACTIVE" : "No");
    DEBUG_PRINTF("Smooth Control: %s\n", smoothControlEnabled ? "Yes" : "No");
    DEBUG_PRINTF("Left Motor - Current: %.1f, Target: %.1f\n", currentLeftSpeed, targetLeftSpeed);
    DEBUG_PRINTF("Right Motor - Current: %.1f, Target: %.1f\n", currentRightSpeed, targetRightSpeed);
    DEBUG_PRINTF("Moving: %s\n", isMoving() ? "Yes" : "No");
    DEBUG_PRINTF("Last Command: %lu ms ago\n", millis() - lastCommandTime);
    DEBUG_PRINTLN("==============================");
}

float MotorController::getPowerConsumption() {
    // Estimated power consumption based on motor speeds
    float totalSpeed = abs(currentLeftSpeed) + abs(currentRightSpeed);
    return (totalSpeed / (2.0f * MOTOR_MAX_SPEED)) * 100.0f; // Percentage
}