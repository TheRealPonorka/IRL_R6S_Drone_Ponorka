/**
 * @file CameraController.cpp
 * @brief OV3660 Camera Controller Implementation
 * @author Peter
 */

#include "CameraController.h"

CameraController::CameraController()
    : initialized(false), streaming(false), frameCount(0),
      lastFrameTime(0), currentFPS(0.0f) {
}

void CameraController::setupCameraConfig() {
    cameraConfig.pin_pwdn = -1;        // Power down pin (not used)
    cameraConfig.pin_reset = -1;       // Reset pin (not used)
    cameraConfig.pin_xclk = CAM_PIN_XCLK;
    cameraConfig.pin_sccb_sda = CAM_PIN_SIOD;
    cameraConfig.pin_sccb_scl = CAM_PIN_SIOC;
    
    cameraConfig.pin_d7 = CAM_PIN_D7;
    cameraConfig.pin_d6 = CAM_PIN_D6;
    cameraConfig.pin_d5 = CAM_PIN_D5;
    cameraConfig.pin_d4 = CAM_PIN_D4;
    cameraConfig.pin_d3 = CAM_PIN_D3;
    cameraConfig.pin_d2 = CAM_PIN_D2;
    cameraConfig.pin_d1 = CAM_PIN_D1;
    cameraConfig.pin_d0 = CAM_PIN_D0;
    cameraConfig.pin_vsync = CAM_PIN_VSYNC;
    cameraConfig.pin_href = CAM_PIN_HREF;
    cameraConfig.pin_pclk = CAM_PIN_PCLK;
    
    // XCLK 20MHz or 10MHz for OV2640 controllers
    cameraConfig.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
    cameraConfig.ledc_timer = LEDC_TIMER_0;
    cameraConfig.ledc_channel = LEDC_CHANNEL_0;
    
    cameraConfig.pixel_format = CAMERA_PIXEL_FORMAT;
    cameraConfig.frame_size = CAMERA_FRAME_SIZE;
    
    cameraConfig.jpeg_quality = CAMERA_QUALITY;
    cameraConfig.fb_count = CAMERA_FB_COUNT;
    cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
    cameraConfig.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
}

bool CameraController::begin() {
    DEBUG_PRINTLN("Initializing OV3660 Camera...");
    
    setupCameraConfig();
    
    // Initialize the camera
    esp_err_t err = esp_camera_init(&cameraConfig);
    if (err != ESP_OK) {
        DEBUG_PRINTF("Camera init failed with error 0x%x\n", err);
        return false;
    }
    
    // Get camera sensor
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) {
        DEBUG_PRINTLN("Failed to get camera sensor");
        return false;
    }
    
    // Initial sensor settings
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
    initialized = true;
    lastFrameTime = millis();
    
    DEBUG_PRINTLN("OV3660 Camera initialized successfully");
    return true;
}

void CameraController::end() {
    if (initialized) {
        esp_camera_deinit();
        initialized = false;
        streaming = false;
    }
}

bool CameraController::startStream() {
    if (!initialized) return false;
    
    streaming = true;
    DEBUG_PRINTLN("Camera streaming started");
    return true;
}

bool CameraController::stopStream() {
    streaming = false;
    DEBUG_PRINTLN("Camera streaming stopped");
    return true;
}

camera_fb_t* CameraController::captureFrame() {
    if (!initialized) return nullptr;
    
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        frameCount++;
        
        // Calculate FPS
        uint32_t currentTime = millis();
        if (currentTime - lastFrameTime > 1000) {
            currentFPS = frameCount * 1000.0f / (currentTime - lastFrameTime);
            frameCount = 0;
            lastFrameTime = currentTime;
        }
    }
    
    return fb;
}

void CameraController::returnFrame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

bool CameraController::captureAndStream() {
    if (!initialized || !streaming) return false;
    
    // This would be implemented as part of the web server streaming
    // For now, just capture and return frame
    camera_fb_t* fb = captureFrame();
    if (fb) {
        returnFrame(fb);
        return true;
    }
    
    return false;
}

bool CameraController::setFrameSize(framesize_t size) {
    if (!initialized) return false;
    
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        return (s->set_framesize(s, size) == 0);
    }
    return false;
}

bool CameraController::setQuality(int quality) {
    if (!initialized) return false;
    
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        return (s->set_quality(s, quality) == 0);
    }
    return false;
}

bool CameraController::setBrightness(int brightness) {
    if (!initialized) return false;
    
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        return (s->set_brightness(s, brightness) == 0);
    }
    return false;
}

bool CameraController::setContrast(int contrast) {
    if (!initialized) return false;
    
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        return (s->set_contrast(s, contrast) == 0);
    }
    return false;
}

bool CameraController::setSaturation(int saturation) {
    if (!initialized) return false;
    
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        return (s->set_saturation(s, saturation) == 0);
    }
    return false;
}

void CameraController::printStatus() {
    DEBUG_PRINTLN("=== Camera Status ===");
    DEBUG_PRINTF("Initialized: %s\n", initialized ? "Yes" : "No");
    DEBUG_PRINTF("Streaming: %s\n", streaming ? "Yes" : "No");
    DEBUG_PRINTF("Frame Count: %u\n", frameCount);
    DEBUG_PRINTF("Current FPS: %.1f\n", currentFPS);
    DEBUG_PRINTLN("====================");
}