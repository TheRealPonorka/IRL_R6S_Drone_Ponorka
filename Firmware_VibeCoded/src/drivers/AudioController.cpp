/**
 * @file AudioController.cpp
 * @brief INMP441 Audio Controller Implementation
 * @author Peter
 */

#include "AudioController.h"

AudioController::AudioController()
    : initialized(false), recording(false), enabled(false),
      audioBuffer(nullptr), bufferSize(I2S_BUFFER_SIZE) {
}

AudioController::~AudioController() {
    end();
}

void AudioController::setupI2SConfig() {
    // I2S configuration
    i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_BUFFER_COUNT,
        .dma_buf_len = I2S_BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    // Pin configuration
    pinConfig = {
        .bck_io_num = MIC_SCK_PIN,
        .ws_io_num = MIC_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = MIC_SD_PIN
    };
}

bool AudioController::begin() {
    DEBUG_PRINTLN("Initializing INMP441 Audio Controller...");
    
    setupI2SConfig();
    
    // Install and start I2S driver
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2sConfig, 0, NULL);
    if (err != ESP_OK) {
        DEBUG_PRINTF("I2S driver install failed: %d\n", err);
        return false;
    }
    
    err = i2s_set_pin(I2S_PORT, &pinConfig);
    if (err != ESP_OK) {
        DEBUG_PRINTF("I2S set pin failed: %d\n", err);
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    
    // Allocate audio buffer
    audioBuffer = (int16_t*)malloc(bufferSize * sizeof(int16_t));
    if (!audioBuffer) {
        DEBUG_PRINTLN("Failed to allocate audio buffer");
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    
    initialized = true;
    enabled = true;
    
    DEBUG_PRINTLN("INMP441 Audio Controller initialized");
    return true;
}

void AudioController::end() {
    if (initialized) {
        stopRecording();
        
        if (audioBuffer) {
            free(audioBuffer);
            audioBuffer = nullptr;
        }
        
        i2s_driver_uninstall(I2S_PORT);
        initialized = false;
    }
}

bool AudioController::startRecording() {
    if (!initialized || !enabled) return false;
    
    esp_err_t err = i2s_start(I2S_PORT);
    if (err == ESP_OK) {
        recording = true;
        DEBUG_PRINTLN("Audio recording started");
        return true;
    }
    
    DEBUG_PRINTF("Failed to start I2S: %d\n", err);
    return false;
}

bool AudioController::stopRecording() {
    if (!initialized) return false;
    
    esp_err_t err = i2s_stop(I2S_PORT);
    if (err == ESP_OK) {
        recording = false;
        DEBUG_PRINTLN("Audio recording stopped");
        return true;
    }
    
    return false;
}

void AudioController::pause() {
    enabled = false;
}

void AudioController::resume() {
    enabled = true;
}

bool AudioController::processAudio() {
    if (!initialized || !recording || !enabled) return false;
    
    size_t bytesRead = 0;
    
    // Read audio data
    esp_err_t err = i2s_read(I2S_PORT, audioBuffer, bufferSize * sizeof(int16_t), &bytesRead, portMAX_DELAY);
    
    if (err == ESP_OK && bytesRead > 0) {
        // Process audio data here
        // For now, just basic noise gate
        size_t samplesRead = bytesRead / sizeof(int16_t);
        
        for (size_t i = 0; i < samplesRead; i++) {
            // Apply gain
            int32_t sample = audioBuffer[i] * AUDIO_GAIN;
            
            // Apply noise gate
            if (abs(sample) < NOISE_GATE_THRESHOLD) {
                sample = 0;
            }
            
            // Clamp to 16-bit range
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            
            audioBuffer[i] = (int16_t)sample;
        }
        
        return true;
    }
    
    return false;
}

size_t AudioController::readAudioData(int16_t* buffer, size_t samples) {
    if (!initialized || !recording || !enabled || !buffer) return 0;
    
    size_t bytesToRead = samples * sizeof(int16_t);
    size_t bytesRead = 0;
    
    esp_err_t err = i2s_read(I2S_PORT, buffer, bytesToRead, &bytesRead, 100);
    
    if (err == ESP_OK) {
        return bytesRead / sizeof(int16_t);
    }
    
    return 0;
}

void AudioController::setGain(float gain) {
    // Update gain setting
    // This would modify the AUDIO_GAIN constant dynamically
}