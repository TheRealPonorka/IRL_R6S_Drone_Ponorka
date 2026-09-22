/**
 * @file PIDController.cpp
 * @brief PID Controller Implementation
 * @author Peter
 */

#include "PIDController.h"

PIDController::PIDController(float Kp, float Ki, float Kd)
    : kp(Kp), ki(Ki), kd(Kd),
      previousError(0), integral(0), derivative(0), lastInput(0),
      outputMin(-255), outputMax(255), integralMin(-100), integralMax(100),
      lastTime(0), firstRun(true), sampleTime(100),
      reversed(false), antiWindup(true), derivativeFilter(0.1f),
      filteredDerivative(0), setpoint(0), setpointRate(1000),
      lastSetpoint(0), deadband(0), bangBangMode(false),
      bangBangThreshold(0), feedforwardGain(0) {
}

float PIDController::compute(float newSetpoint, float input) {
    setpoint = newSetpoint;
    return compute(input);
}

float PIDController::compute(float input) {
    uint32_t currentTime = millis();
    
    if (firstRun || (currentTime - lastTime) < sampleTime) {
        lastInput = input;
        firstRun = false;
        return 0;
    }
    
    float deltaTime = (currentTime - lastTime) / 1000.0f;
    
    // Calculate error
    float error = setpoint - input;
    
    // Apply deadband
    if (abs(error) < deadband) {
        error = 0;
    }
    
    // Bang-bang mode check
    if (bangBangMode && abs(error) > bangBangThreshold) {
        lastTime = currentTime;
        lastInput = input;
        previousError = error;
        
        if (error > 0) {
            return reversed ? outputMin : outputMax;
        } else {
            return reversed ? outputMax : outputMin;
        }
    }
    
    // Proportional term
    float proportional = kp * error;
    
    // Integral term
    integral += error * deltaTime;
    
    // Anti-windup
    if (antiWindup) {
        integral = constrain(integral, integralMin, integralMax);
    }
    
    float integralTerm = ki * integral;
    
    // Derivative term (using input derivative to avoid derivative kick)
    float inputDerivative = (input - lastInput) / deltaTime;
    
    // Apply low-pass filter to derivative
    filteredDerivative = derivativeFilter * inputDerivative + 
                        (1.0f - derivativeFilter) * filteredDerivative;
    
    derivative = -kd * filteredDerivative;
    
    // Feedforward term
    float feedforward = feedforwardGain * setpoint;
    
    // Calculate output
    float output = proportional + integralTerm + derivative + feedforward;
    
    // Apply output limits
    output = constrainOutput(output);
    
    // Apply direction
    if (reversed) {
        output = -output;
    }
    
    // Store values for next iteration
    previousError = error;
    lastInput = input;
    lastTime = currentTime;
    
    return output;
}

void PIDController::setTunings(float Kp, float Ki, float Kd) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;
    
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PIDController::setOutputLimits(float min, float max) {
    if (min >= max) return;
    
    outputMin = min;
    outputMax = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    if (min >= max) return;
    
    integralMin = min;
    integralMax = max;
}

void PIDController::setSampleTime(uint32_t newSampleTime) {
    if (newSampleTime > 0) {
        sampleTime = newSampleTime;
    }
}

void PIDController::setDirection(bool reverse) {
    reversed = reverse;
}

void PIDController::setDerivativeFilter(float filterCoeff) {
    derivativeFilter = constrain(filterCoeff, 0.0f, 1.0f);
}

void PIDController::setSetpoint(float newSetpoint) {
    // Apply setpoint ramping if enabled
    if (setpointRate > 0) {
        setpoint = rampSetpoint(newSetpoint, setpoint, 
                               (millis() - lastTime) / 1000.0f);
    } else {
        setpoint = newSetpoint;
    }
    
    lastSetpoint = setpoint;
}

void PIDController::reset() {
    integral = 0;
    derivative = 0;
    previousError = 0;
    lastInput = 0;
    filteredDerivative = 0;
    firstRun = true;
    lastTime = millis();
}

void PIDController::enable() {
    reset();
}

void PIDController::disable() {
    // Controller is always enabled, just reset
    reset();
}

void PIDController::setDeadband(float deadbandValue) {
    deadband = abs(deadbandValue);
}

void PIDController::setBangBang(bool enable, float threshold) {
    bangBangMode = enable;
    bangBangThreshold = abs(threshold);
}

void PIDController::setFeedforward(float feedforward) {
    feedforwardGain = feedforward;
}

void PIDController::printTunings() {
    DEBUG_PRINTLN("=== PID Tunings ===");
    DEBUG_PRINTF("Kp: %.3f\n", kp);
    DEBUG_PRINTF("Ki: %.3f\n", ki);
    DEBUG_PRINTF("Kd: %.3f\n", kd);
    DEBUG_PRINTF("Setpoint: %.3f\n", setpoint);
    DEBUG_PRINTF("Output Limits: %.1f to %.1f\n", outputMin, outputMax);
    DEBUG_PRINTF("Sample Time: %lu ms\n", sampleTime);
    DEBUG_PRINTF("Reversed: %s\n", reversed ? "Yes" : "No");
    DEBUG_PRINTLN("==================");
}

String PIDController::getTuningsJSON() {
    String json = "{";
    json += "\"kp\":" + String(kp) + ",";
    json += "\"ki\":" + String(ki) + ",";
    json += "\"kd\":" + String(kd) + ",";
    json += "\"setpoint\":" + String(setpoint) + ",";
    json += "\"error\":" + String(previousError) + ",";
    json += "\"integral\":" + String(integral) + ",";
    json += "\"derivative\":" + String(derivative) + ",";
    json += "\"reversed\":" + String(reversed ? 1 : 0);
    json += "}";
    return json;
}

float PIDController::constrainOutput(float output) {
    return constrain(output, outputMin, outputMax);
}

float PIDController::rampSetpoint(float newSetpoint, float currentSetpoint, float deltaTime) {
    float maxChange = setpointRate * deltaTime;
    float change = newSetpoint - currentSetpoint;
    
    if (abs(change) <= maxChange) {
        return newSetpoint;
    } else {
        return currentSetpoint + (change > 0 ? maxChange : -maxChange);
    }
}