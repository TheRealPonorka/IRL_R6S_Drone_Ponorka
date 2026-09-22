/**
 * @file CameraController.h
 * @brief OV3660 Camera Controller for R6 Recon Drone
 * @author Peter
 */

#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <Arduino.h>
#include "esp_camera.h"
#include "config.h"

class CameraController {
private:
    bool initialized;
    bool streaming;
    camera_config_t cameraConfig;
    
    // Statistics
    uint32_t frameCount;
    uint32_t lastFrameTime;
    float currentFPS;
    
    // Configuration
    void setupCameraConfig();
    
public:
    CameraController();
    
    // Initialization
    bool begin();
    void end();
    
    // Camera operations
    bool startStream();
    bool stopStream();
    camera_fb_t* captureFrame();
    void returnFrame(camera_fb_t* fb);
    
    // Streaming
    bool captureAndStream();
    bool isStreaming() const { return streaming; }
    
    // Configuration
    bool setFrameSize(framesize_t size);
    bool setQuality(int quality);
    bool setBrightness(int brightness);
    bool setContrast(int contrast);
    bool setSaturation(int saturation);
    
    // Status
    bool isInitialized() const { return initialized; }
    uint32_t getFrameCount() const { return frameCount; }
    float getCurrentFPS() const { return currentFPS; }
    
    // Settings
    void printStatus();
};

#endif // CAMERA_CONTROLLER_H