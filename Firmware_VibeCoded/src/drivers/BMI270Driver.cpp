/**
 * @file BMI270Driver.cpp  
 * @brief BMI270 6-axis IMU Driver Implementation
 * @author Peter
 * 
 * Self-contained BMI270 driver using standard I2C
 */

#include "BMI270Driver.h"
#include <math.h>

BMI270Driver::BMI270Driver() 
    : gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0),
      accelOffsetX(0), accelOffsetY(0), accelOffsetZ(0),
      complementaryFilterAlpha(IMU_FILTER_ALPHA),
      currentPitch(0), currentRoll(0), currentYaw(0),
      lastUpdateTime(0), initialized(false) {
}

bool BMI270Driver::begin() {
    DEBUG_PRINTLN("Initializing BMI270 IMU...");
    
    // Initialize I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ);
    delay(100);
    
    // Check chip ID
    if (!isConnected()) {
        DEBUG_PRINTLN("BMI270 not found!");
        return false;
    }
    
    // Software reset
    reset();
    delay(50);
    
    // Power up accelerometer and gyroscope
    if (!writeRegister(BMI270_PWR_CTRL, 0x0E)) {
        DEBUG_PRINTLN("Failed to power up BMI270");
        return false;
    }
    delay(10);
    
    // Configure accelerometer
    // ODR: 100Hz, Normal mode, No decimation
    writeRegister(BMI270_ACC_CONF, 0x28); 
    
    // Set accelerometer range to ±16g
    setAccelerometerRange(ACCEL_RANGE);
    
    // Configure gyroscope  
    // ODR: 100Hz, Normal mode, No decimation
    writeRegister(BMI270_GYR_CONF, 0x28);
    
    // Set gyroscope range to ±2000°/s
    setGyroscopeRange(GYRO_RANGE);
    
    delay(100);
    
    // Perform initial calibration
    startCalibration();
    
    lastUpdateTime = millis();
    initialized = true;
    
    DEBUG_PRINTLN("BMI270 initialized successfully");
    return true;
}

bool BMI270Driver::isConnected() {
    uint8_t chipId = readRegister(BMI270_CHIP_ID_REG);
    return (chipId == BMI270_CHIP_ID_VALUE);
}

void BMI270Driver::reset() {
    // Soft reset command
    writeRegister(0x7E, 0xB6);
    delay(5);
}

bool BMI270Driver::readAccelerometer(float& x, float& y, float& z) {
    uint8_t data[6];
    if (!readRegisters(BMI270_ACC_DATA_X_LSB, data, 6)) {
        return false;
    }
    
    // Convert to 16-bit signed values
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to g (16-bit, ±16g range)
    const float scale = 16.0f / 32768.0f;
    x = (rawX * scale) - accelOffsetX;
    y = (rawY * scale) - accelOffsetY; 
    z = (rawZ * scale) - accelOffsetZ;
    
    rawAccelX = x;
    rawAccelY = y;
    rawAccelZ = z;
    
    return true;
}

bool BMI270Driver::readGyroscope(float& x, float& y, float& z) {
    uint8_t data[6];
    if (!readRegisters(BMI270_GYR_DATA_X_LSB, data, 6)) {
        return false;
    }
    
    // Convert to 16-bit signed values
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to degrees per second (16-bit, ±2000°/s range)
    const float scale = 2000.0f / 32768.0f;
    x = (rawX * scale) - gyroOffsetX;
    y = (rawY * scale) - gyroOffsetY;
    z = (rawZ * scale) - gyroOffsetZ;
    
    // Apply deadband
    if (abs(x) < GYRO_DEADBAND) x = 0;
    if (abs(y) < GYRO_DEADBAND) y = 0;
    if (abs(z) < GYRO_DEADBAND) z = 0;
    
    rawGyroX = x;
    rawGyroY = y;
    rawGyroZ = z;
    
    return true;
}

bool BMI270Driver::readSensorData(float& pitch, float& roll, float& yaw,
                                 float& gyroX, float& gyroY, float& gyroZ) {
    
    float accelX, accelY, accelZ;
    
    // Read both sensors
    if (!readAccelerometer(accelX, accelY, accelZ) || 
        !readGyroscope(gyroX, gyroY, gyroZ)) {
        return false;
    }
    
    // Calculate time delta
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    
    // Skip first reading (invalid dt)
    if (dt > 0.1f || dt <= 0) {
        return false;
    }
    
    // Update angles using complementary filter
    updateAngles(dt);
    
    pitch = currentPitch;
    roll = currentRoll;
    yaw = currentYaw;
    
    return true;
}

void BMI270Driver::updateAngles(float dt) {
    // Calculate angles from accelerometer
    float accelPitch = atan2(-rawAccelX, sqrt(rawAccelY * rawAccelY + rawAccelZ * rawAccelZ)) * 180.0f / PI;
    float accelRoll = atan2(rawAccelY, rawAccelZ) * 180.0f / PI;
    
    // Integrate gyroscope for angles
    float gyroPitch = currentPitch + rawGyroY * dt;
    float gyroRoll = currentRoll + rawGyroX * dt;
    currentYaw += rawGyroZ * dt;
    
    // Apply complementary filter
    currentPitch = complementaryFilter(gyroPitch, rawGyroY, accelPitch, dt);
    currentRoll = complementaryFilter(gyroRoll, rawGyroX, accelRoll, dt);
    
    // Normalize yaw to ±180°
    while (currentYaw > 180.0f) currentYaw -= 360.0f;
    while (currentYaw < -180.0f) currentYaw += 360.0f;
}

float BMI270Driver::complementaryFilter(float angle, float gyroRate, float accelAngle, float dt) {
    return complementaryFilterAlpha * angle + (1.0f - complementaryFilterAlpha) * accelAngle;
}

void BMI270Driver::startCalibration(uint16_t samples) {
    DEBUG_PRINTLN("Calibrating BMI270...");
    
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    uint16_t validSamples = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        float gx, gy, gz, ax, ay, az;
        
        if (readGyroscope(gx, gy, gz) && readAccelerometer(ax, ay, az)) {
            sumGyroX += rawGyroX + gyroOffsetX;  // Add back offset for raw reading
            sumGyroY += rawGyroY + gyroOffsetY;
            sumGyroZ += rawGyroZ + gyroOffsetZ;
            
            sumAccelX += rawAccelX + accelOffsetX;
            sumAccelY += rawAccelY + accelOffsetY;  
            sumAccelZ += rawAccelZ + accelOffsetZ;
            
            validSamples++;
        }
        
        delay(2);
        
        if (i % 100 == 0) {
            DEBUG_PRINTF("Calibration progress: %d/%d\n", i, samples);
        }
    }
    
    if (validSamples > 0) {
        gyroOffsetX = sumGyroX / validSamples;
        gyroOffsetY = sumGyroY / validSamples;
        gyroOffsetZ = sumGyroZ / validSamples;
        
        accelOffsetX = sumAccelX / validSamples;
        accelOffsetY = sumAccelY / validSamples;
        accelOffsetZ = (sumAccelZ / validSamples) - 1.0f; // Subtract 1g for Z-axis
        
        DEBUG_PRINTF("Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
        DEBUG_PRINTF("Accel offsets: X=%.3f, Y=%.3f, Z=%.3f\n", accelOffsetX, accelOffsetY, accelOffsetZ);
        DEBUG_PRINTLN("BMI270 calibration complete");
    } else {
        DEBUG_PRINTLN("BMI270 calibration failed!");
    }
}

void BMI270Driver::setAccelerometerRange(uint8_t range) {
    uint8_t regValue = 0;
    
    switch (range) {
        case 2:  regValue = 0x00; break;  // ±2g
        case 4:  regValue = 0x01; break;  // ±4g  
        case 8:  regValue = 0x02; break;  // ±8g
        case 16: regValue = 0x03; break;  // ±16g
        default: regValue = 0x03; break;  // Default ±16g
    }
    
    writeRegister(BMI270_ACC_RANGE, regValue);
}

void BMI270Driver::setGyroscopeRange(uint16_t range) {
    uint8_t regValue = 0;
    
    switch (range) {
        case 125:  regValue = 0x04; break;  // ±125°/s
        case 250:  regValue = 0x03; break;  // ±250°/s
        case 500:  regValue = 0x02; break;  // ±500°/s
        case 1000: regValue = 0x01; break;  // ±1000°/s
        case 2000: regValue = 0x00; break;  // ±2000°/s
        default:   regValue = 0x00; break;  // Default ±2000°/s
    }
    
    writeRegister(BMI270_GYR_RANGE, regValue);
}

void BMI270Driver::setFilterAlpha(float alpha) {
    complementaryFilterAlpha = constrain(alpha, 0.0f, 1.0f);
}

float BMI270Driver::getTemperature() {
    // BMI270 temperature reading (if implemented)
    // Temperature formula: temp = (temp_raw / 512.0) + 23.0
    return 25.0f; // Placeholder
}

bool BMI270Driver::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t BMI270Driver::readRegister(uint8_t reg) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 0;
    
    Wire.requestFrom((uint8_t)IMU_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool BMI270Driver::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom((uint8_t)IMU_ADDRESS, length);
    
    for (uint8_t i = 0; i < length; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        } else {
            return false;
        }
    }
    
    return true;
}

bool BMI270Driver::isCalibrated() {
    return initialized && (abs(gyroOffsetX) > 0.1f || abs(gyroOffsetY) > 0.1f || abs(gyroOffsetZ) > 0.1f);
}