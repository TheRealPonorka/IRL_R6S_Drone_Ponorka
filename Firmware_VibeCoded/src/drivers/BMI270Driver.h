/**
 * @file BMI270Driver.h
 * @brief BMI270 6-axis IMU Driver for R6 Recon Drone
 * @author Peter
 * 
 * High-performance BMI270 driver optimized for self-balancing drone applications
 * Features complementary filter for stable angle estimation
 */

#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class BMI270Driver {
private:
    // BMI270 register addresses
    static const uint8_t BMI270_CHIP_ID_REG = 0x00;
    static const uint8_t BMI270_CHIP_ID_VALUE = 0x24;
    static const uint8_t BMI270_ACC_DATA_X_LSB = 0x0C;
    static const uint8_t BMI270_GYR_DATA_X_LSB = 0x12;
    static const uint8_t BMI270_PWR_CTRL = 0x7D;
    static const uint8_t BMI270_ACC_CONF = 0x40;
    static const uint8_t BMI270_GYR_CONF = 0x42;
    static const uint8_t BMI270_ACC_RANGE = 0x41;
    static const uint8_t BMI270_GYR_RANGE = 0x43;
    
    // Calibration and filtering
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float complementaryFilterAlpha;
    
    // Current readings
    float currentPitch, currentRoll, currentYaw;
    float rawAccelX, rawAccelY, rawAccelZ;
    float rawGyroX, rawGyroY, rawGyroZ;
    
    // Timing
    uint32_t lastUpdateTime;
    bool initialized;
    
    // I2C communication
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Data processing
    void calibrateGyroscope();
    void calibrateAccelerometer();
    void updateAngles(float dt);
    float complementaryFilter(float angle, float gyroRate, float accelAngle, float dt);
    
public:
    BMI270Driver();
    
    // Initialization and configuration
    bool begin();
    bool isConnected();
    void reset();
    
    // Data reading
    bool readAccelerometer(float& x, float& y, float& z);
    bool readGyroscope(float& x, float& y, float& z);
    bool readSensorData(float& pitch, float& roll, float& yaw, 
                       float& gyroX, float& gyroY, float& gyroZ);
    
    // Calibration
    void startCalibration(uint16_t samples = 1000);
    bool isCalibrated();
    
    // Configuration
    void setAccelerometerRange(uint8_t range);
    void setGyroscopeRange(uint16_t range);
    void setFilterAlpha(float alpha);
    
    // Status
    float getPitch() const { return currentPitch; }
    float getRoll() const { return currentRoll; }
    float getYaw() const { return currentYaw; }
    bool isInitialized() const { return initialized; }
    
    // Temperature reading (if available)
    float getTemperature();
};

#endif // BMI270_DRIVER_H