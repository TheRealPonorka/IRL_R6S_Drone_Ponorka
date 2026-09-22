/**
 * @file AudioController.h
 * @brief INMP441 Audio Controller for R6 Recon Drone
 * @author Peter
 */

#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H

#include <Arduino.h>
#include "driver/i2s.h"
#include "config.h"

class AudioController {
private:
    bool initialized;
    bool recording;
    bool enabled;
    
    // I2S configuration
    i2s_config_t i2sConfig;
    i2s_pin_config_t pinConfig;
    
    // Audio buffer
    int16_t* audioBuffer;
    size_t bufferSize;
    
    void setupI2SConfig();

public:
    AudioController();
    ~AudioController();
    
    // Initialization
    bool begin();
    void end();
    
    // Control
    bool startRecording();
    bool stopRecording();
    void pause();
    void resume();
    
    // Audio processing
    bool processAudio();
    size_t readAudioData(int16_t* buffer, size_t samples);
    
    // Status
    bool isInitialized() const { return initialized; }
    bool isRecording() const { return recording; }
    bool isEnabled() const { return enabled; }
    
    // Configuration
    void setGain(float gain);
    void setEnabled(bool enable) { enabled = enable; }
};

#endif // AUDIO_CONTROLLER_H