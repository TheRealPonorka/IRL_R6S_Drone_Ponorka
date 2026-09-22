/**
 * @file main_camera_stream.cpp
 * @brief R6 Drone - latest-frame camera + LED/IMU that never wait on WiFi
 *
 * MILESTONE LOCK (2026-09-03): working camera view, IMU readout,
 * LED animation, color picker, spotlight. Do not use this file as a
 * scratch pad; branch or copy before large changes.
 *
 * Camera capture runs in a background task into a double buffer.
 * HTTP only copies the newest JPEG out. The browser is allowed one
 * in-flight image request so frames cannot queue for seconds.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "esp_camera.h"
#include "config.h"
#include "SparkFun_BMI270_Arduino_Library.h"

WebServer server(80);
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);
BMI270 imu;

static const size_t MAX_JPEG = 48 * 1024;

bool imuInitialized = false;
volatile float pitch = 0.0, roll = 0.0, yaw = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;

bool ledEnabled = true;
uint32_t lastLedUpdate = 0;
int ledAnimationStep = 0;
uint8_t ledR = 0, ledG = 64, ledB = 0;

bool spotlightEnabled = false;
int spotlightBrightness = 128;
bool cameraInitialized = false;

uint8_t *jpegBuf[2] = {nullptr, nullptr};
uint8_t *sendBuf = nullptr;
volatile size_t jpegLen[2] = {0, 0};
volatile int jpegReady = 0;
SemaphoreHandle_t jpegMutex;

static void updateLedsAndImu() {
    uint32_t now = millis();

    if (imuInitialized) {
        static uint32_t lastImu = 0;
        if (now - lastImu >= 15) {
            float dt = (now - lastImu) / 1000.0f;
            lastImu = now;
            if (dt > 0.05f) {
                dt = 0.05f;
            }
            if (imu.getSensorData() == BMI2_OK) {
                float ax = imu.data.accelX;
                float ay = imu.data.accelY;
                float az = imu.data.accelZ;
                float gx = imu.data.gyroX - gyroOffsetX;
                float gy = imu.data.gyroY - gyroOffsetY;
                float gz = imu.data.gyroZ - gyroOffsetZ;
                float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
                float accelRoll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
                pitch = 0.98f * (pitch + gx * dt) + 0.02f * accelPitch;
                roll = 0.98f * (roll + gy * dt) + 0.02f * accelRoll;
                yaw += gz * dt;
            }
        }
    }

    if (ledEnabled && (now - lastLedUpdate >= LED_ANIMATION_SPEED)) {
        lastLedUpdate = now;
        strip.clear();
        switch (ledAnimationStep) {
            case 0:
                break;
            case 1:
            case 5:
                strip.setPixelColor(2, strip.Color(ledR, ledG, ledB));
                strip.setPixelColor(3, strip.Color(ledR, ledG, ledB));
                break;
            case 2:
            case 4:
                for (int i = 1; i <= 4; i++) {
                    strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
                }
                break;
            case 3:
                for (int i = 0; i < LED_COUNT; i++) {
                    strip.setPixelColor(i, strip.Color(ledR, ledG, ledB));
                }
                break;
        }
        strip.show();
        ledAnimationStep = (ledAnimationStep + 1) % 6;
    }

    ledcWrite(SPOTLIGHT_PWM_CHANNEL, spotlightEnabled ? spotlightBrightness : 0);
}

void ledImuTask(void *param) {
    (void)param;
    for (;;) {
        updateLedsAndImu();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void cameraTask(void *param) {
    (void)param;
    int writeIdx = 1;
    for (;;) {
        if (!cameraInitialized) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            if (fb->len > 0 && fb->len <= MAX_JPEG && jpegBuf[writeIdx]) {
                if (xSemaphoreTake(jpegMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    memcpy(jpegBuf[writeIdx], fb->buf, fb->len);
                    jpegLen[writeIdx] = fb->len;
                    jpegReady = writeIdx;
                    writeIdx ^= 1;
                    xSemaphoreGive(jpegMutex);
                }
            }
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // ~20 captures/s, always overwrite old
    }
}

bool initializeCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_HVGA;  // 480x320 — small enough to send quickly
    config.jpeg_quality = 22;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init failed");
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 0);
        s->set_aec_value(s, 400);
        s->set_agc_gain(s, 8);
        s->set_lenc(s, 1);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_colorbar(s, 0);
    }
    return true;
}

void handleJpg() {
    if (!jpegBuf[0] || jpegLen[jpegReady] == 0) {
        server.send(503, "text/plain", "no frame");
        return;
    }

    size_t len = 0;
    if (!sendBuf || xSemaphoreTake(jpegMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        server.send(503, "text/plain", "busy");
        return;
    }
    int idx = jpegReady;
    len = jpegLen[idx];
    if (len > MAX_JPEG) {
        len = MAX_JPEG;
    }
    memcpy(sendBuf, jpegBuf[idx], len);
    xSemaphoreGive(jpegMutex);

    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "image/jpeg", (PGM_P)sendBuf, len);
}

bool initializeIMU() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    delay(100);

    if (imu.beginI2C(0x69) != BMI2_OK && imu.beginI2C(0x68) != BMI2_OK) {
        Serial.println("BMI270 init failed");
        return false;
    }

    float sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (int i = 0; i < 50; i++) {
        if (imu.getSensorData() == BMI2_OK) {
            sx += imu.data.gyroX;
            sy += imu.data.gyroY;
            sz += imu.data.gyroZ;
            n++;
        }
        delay(10);
    }
    if (n > 0) {
        gyroOffsetX = sx / n;
        gyroOffsetY = sy / n;
        gyroOffsetZ = sz / n;
    }
    Serial.printf("IMU calibrated: %.2f, %.2f, %.2f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    return true;
}

void initializeSpotlight() {
    ledcSetup(SPOTLIGHT_PWM_CHANNEL, SPOTLIGHT_PWM_FREQ, SPOTLIGHT_PWM_RESOLUTION);
    ledcAttachPin(SPOTLIGHT_PIN, SPOTLIGHT_PWM_CHANNEL);
    ledcWrite(SPOTLIGHT_PWM_CHANNEL, 0);
}

void setupWebServer() {
    server.on("/", []() {
        server.send(200, "text/html",
            "<!DOCTYPE html><html><head><title>R6 Drone</title>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<style>"
            "body{font-family:Arial;margin:16px;background:#111;color:#fff}"
            "h1{color:#ff8c00;text-align:center}"
            ".video{width:100%;max-width:800px;aspect-ratio:16/9;object-fit:cover;"
            "border:2px solid #ff8c00;border-radius:8px;background:#000}"
            ".controls{text-align:center;margin:16px}"
            ".btn{background:#ff8c00;color:#000;border:none;padding:12px 20px;margin:6px;border-radius:6px;font-weight:bold}"
            ".data{color:#4CAF50;font-weight:bold}"
            ".color-row{text-align:center;margin:12px auto;max-width:360px}"
            ".color-row input[type=color]{width:64px;height:36px;border:0;padding:0;background:none;vertical-align:middle}"
            ".rgb{margin-top:8px;color:#ccc;font-size:14px}"
            "</style></head><body>"
            "<h1>R6 Recon Drone</h1>"
            "<div style='text-align:center'><img id='cam' class='video' alt='camera'></div>"
            "<div class='controls'>"
            "<button class='btn' onclick=\"fetch('/led/toggle')\">Toggle LEDs</button>"
            "<button class='btn' onclick=\"fetch('/spot/toggle')\">Toggle Spotlight</button>"
            "</div>"
            "<div class='color-row'>"
            "<label>LED color "
            "<input id='ledcol' type='color' value='#004000' oninput='setLedColor(this.value)'>"
            "</label>"
            "<div class='rgb'>R <span id='cr'>0</span> &nbsp; G <span id='cg'>64</span> &nbsp; B <span id='cb'>0</span></div>"
            "</div>"
            "<div style='text-align:center'>"
            "Pitch <span class='data' id='p'>--</span> | "
            "Roll <span class='data' id='r'>--</span> | "
            "Yaw <span class='data' id='y'>--</span>"
            "</div>"
            "<script>"
            "var cam=document.getElementById('cam'), oldUrl=null, busy=false;"
            "function frame(){"
            "if(busy){return;}"
            "busy=true;"
            "fetch('/jpg',{cache:'no-store'}).then(function(r){"
            "if(!r.ok){throw new Error('jpg');}"
            "return r.blob();"
            "}).then(function(b){"
            "var url=URL.createObjectURL(b);"
            "cam.src=url;"
            "if(oldUrl){URL.revokeObjectURL(oldUrl);}"
            "oldUrl=url;"
            "}).catch(function(){}"
            ").finally(function(){busy=false;setTimeout(frame,50);});"
            "}"
            "function imu(){"
            "fetch('/imu',{cache:'no-store'}).then(function(r){return r.json();}).then(function(d){"
            "document.getElementById('p').textContent=d.pitch.toFixed(1)+'\\u00b0';"
            "document.getElementById('r').textContent=d.roll.toFixed(1)+'\\u00b0';"
            "document.getElementById('y').textContent=d.yaw.toFixed(1)+'\\u00b0';"
            "}).catch(function(){});"
            "}"
            "var colorTimer=null;"
            "function setLedColor(hex){"
            "var r=parseInt(hex.substr(1,2),16);"
            "var g=parseInt(hex.substr(3,2),16);"
            "var b=parseInt(hex.substr(5,2),16);"
            "document.getElementById('cr').textContent=r;"
            "document.getElementById('cg').textContent=g;"
            "document.getElementById('cb').textContent=b;"
            "clearTimeout(colorTimer);"
            "colorTimer=setTimeout(function(){"
            "fetch('/led/color?r='+r+'&g='+g+'&b='+b);"
            "},80);"
            "}"
            "frame();"
            "setInterval(imu,200);"
            "imu();"
            "</script></body></html>");
    });

    server.on("/jpg", handleJpg);

    server.on("/imu", []() {
        char json[96];
        snprintf(json, sizeof(json), "{\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f}",
                 (double)pitch, (double)roll, (double)yaw);
        server.send(200, "application/json", json);
    });

    server.on("/led/toggle", []() {
        ledEnabled = !ledEnabled;
        if (!ledEnabled) {
            strip.clear();
            strip.show();
        }
        Serial.printf("LEDs: %s\n", ledEnabled ? "ON" : "OFF");
        server.send(200, "text/plain", ledEnabled ? "ON" : "OFF");
    });

    server.on("/led/color", []() {
        int r = server.hasArg("r") ? server.arg("r").toInt() : ledR;
        int g = server.hasArg("g") ? server.arg("g").toInt() : ledG;
        int b = server.hasArg("b") ? server.arg("b").toInt() : ledB;
        if (r < 0) r = 0;
        if (r > 255) r = 255;
        if (g < 0) g = 0;
        if (g > 255) g = 255;
        if (b < 0) b = 0;
        if (b > 255) b = 255;
        ledR = (uint8_t)r;
        ledG = (uint8_t)g;
        ledB = (uint8_t)b;
        Serial.printf("LED color: %d %d %d\n", ledR, ledG, ledB);
        server.send(200, "text/plain", "OK");
    });

    server.on("/spot/toggle", []() {
        spotlightEnabled = !spotlightEnabled;
        ledcWrite(SPOTLIGHT_PWM_CHANNEL, spotlightEnabled ? spotlightBrightness : 0);
        Serial.printf("Spotlight: %s\n", spotlightEnabled ? "ON" : "OFF");
        server.send(200, "text/plain", spotlightEnabled ? "ON" : "OFF");
    });

    server.on("/favicon.ico", []() { server.send(204); });
    server.on("/apple-touch-icon.png", []() { server.send(204); });
    server.on("/apple-touch-icon-precomposed.png", []() { server.send(204); });
    server.onNotFound([]() { server.send(204); });
    server.begin();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("=================================");
    Serial.println("R6 Drone - Latest frame view");
    Serial.println("=================================");

    jpegMutex = xSemaphoreCreateMutex();
    jpegBuf[0] = (uint8_t *)ps_malloc(MAX_JPEG);
    jpegBuf[1] = (uint8_t *)ps_malloc(MAX_JPEG);
    sendBuf = (uint8_t *)ps_malloc(MAX_JPEG);
    if (!jpegBuf[0] || !jpegBuf[1] || !sendBuf) {
        Serial.println("JPEG buffer alloc failed");
    }

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();
    Serial.println("LED strip ready");

    initializeSpotlight();
    Serial.println("Spotlight ready");

    if (initializeIMU()) {
        imuInitialized = true;
        Serial.println("IMU ready");
    }

    if (initializeCamera()) {
        cameraInitialized = true;
        Serial.println("Camera ready (HVGA latest-frame)");
    }

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    WiFi.setSleep(false);
    Serial.printf("WiFi AP: %s\n", WiFi.softAPIP().toString().c_str());

    setupWebServer();

    xTaskCreatePinnedToCore(ledImuTask, "ledimu", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(cameraTask, "camera", 6144, NULL, 1, NULL, 0);

    Serial.println("Open http://192.168.4.1");
}

void loop() {
    server.handleClient();

    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        lastStatus = millis();
        Serial.printf("[%ds] Mem:%dKB frame:%uB IMU:%s Cam:%s LED:%s\n",
                      millis() / 1000,
                      ESP.getFreeHeap() / 1024,
                      (unsigned)jpegLen[jpegReady],
                      imuInitialized ? "ON" : "OFF",
                      cameraInitialized ? "ON" : "OFF",
                      ledEnabled ? "ON" : "OFF");
    }
    delay(1);
}
