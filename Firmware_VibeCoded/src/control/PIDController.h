/**
 * @file PIDController.h
 * @brief PID Controller Implementation for R6 Recon Drone
 * @author Peter
 * 
 * High-performance PID controller optimized for self-balancing applications
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class PIDController {
private:
    // PID parameters
    float kp, ki, kd;
    
    // PID state variables
    float previousError;
    float integral;
    float derivative;
    float lastInput;
    
    // Output limits
    float outputMin, outputMax;
    float integralMin, integralMax;
    
    // Timing
    uint32_t lastTime;
    bool firstRun;
    
    // Sample time control
    uint32_t sampleTime; // in milliseconds
    
    // Direction control
    bool reversed;
    
    // Anti-windup and filtering
    bool antiWindup;
    float derivativeFilter; // Low-pass filter coefficient for derivative
    float filteredDerivative;
    
    // Setpoint ramping
    float setpoint;
    float setpointRate; // Maximum rate of setpoint change
    float lastSetpoint;
    
public:
    PIDController(float Kp = 0.0, float Ki = 0.0, float Kd = 0.0);
    
    // Core PID computation
    float compute(float setpoint, float input);
    float compute(float input); // Uses internal setpoint
    
    // Parameter tuning
    void setTunings(float Kp, float Ki, float Kd);
    void setKp(float Kp) { kp = Kp; }
    void setKi(float Ki) { ki = Ki; }
    void setKd(float Kd) { kd = Kd; }
    
    // Output limits
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    
    // Configuration
    void setSampleTime(uint32_t newSampleTime);
    void setDirection(bool reverse);
    void setAntiWindup(bool enable) { antiWindup = enable; }
    void setDerivativeFilter(float filterCoeff);
    void setSetpointRate(float rate) { setpointRate = rate; }
    
    // Setpoint management
    void setSetpoint(float newSetpoint);
    float getSetpoint() const { return setpoint; }
    
    // State management
    void reset();
    void enable();
    void disable();
    
    // Status getters
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    float getError() const { return previousError; }
    float getIntegral() const { return integral; }
    float getDerivative() const { return derivative; }
    bool isReversed() const { return reversed; }
    
    // Advanced features
    void setDeadband(float deadband);
    void setBangBang(bool enable, float threshold = 0.0);
    void setFeedforward(float feedforward);
    
    // Diagnostics
    void printTunings();
    String getTuningsJSON();
    
private:
    // Internal helper functions
    float constrainOutput(float output);
    float rampSetpoint(float newSetpoint, float currentSetpoint, float deltaTime);
    
    // Advanced parameters
    float deadband;
    bool bangBangMode;
    float bangBangThreshold;
    float feedforwardGain;
};

#endif // PID_CONTROLLER_H