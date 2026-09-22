/**
 * @file main_camera_stream.cpp
 * @brief R6 Drone - camera + LED/IMU + hold-to-drive motors
 *
 * MILESTONE LOCK (2026-09-20): working camera, IMU, LED, spotlight,
 * hold-to-drive motors, encoder readout. Do not use this file as a
 * scratch pad; branch or copy before large changes.
 *
 * GPIO (TTL USB-C / CH340 only — do not plug OTG USB-C):
 *   IMU SDA=1 SCL=2 | LED DIN=3 | spotlight=38
 *   Motor A 41/42 | Motor B 39/40 | Left enc 19/20 | Right enc 21/47
 *   Camera DVP 4,5,6,7,8,9,10,11,12,13,15,16,17,18
 *
 * Hold-to-drive plus pitch/roll PID on a stand: tilt spins the motors,
 * level stays still. One breadboard motor is known-faulty; both channels
 * still get the same balance PWM.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "esp_camera.h"
#include "driver/gpio.h"
#include "config.h"
#include "SparkFun_BMI270_Arduino_Library.h"

WebServer server(80);
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, LED_TYPE + NEO_KHZ800);
BMI270 imu;

static const size_t MAX_JPEG = 48 * 1024;
static const int8_t QUAD_TABLE[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};

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
bool motorsInitialized = false;
bool encodersInitialized = false;

volatile char driveCmd = 's';
volatile uint32_t lastDriveMs = 0;
volatile int16_t appliedLeft = 0;
volatile int16_t appliedRight = 0;

volatile int32_t encLeft = 0;
volatile int32_t encRight = 0;
volatile uint8_t encLeftPrev = 0;
volatile uint8_t encRightPrev = 0;

volatile bool balanceEnabled = true;
volatile bool balanceReady = false;
volatile bool balanceUseRoll = false;
volatile int8_t balanceDir = 1;
volatile float balanceSetpoint = 0.0f;
volatile float pidError = 0.0f;
volatile int16_t pidOut = 0;
float pidIntegral = 0.0f;
float lastGx = 0.0f;
float lastGy = 0.0f;
uint32_t imuReadyMs = 0;

uint8_t *jpegBuf[2] = {nullptr, nullptr};
uint8_t *sendBuf = nullptr;
volatile size_t jpegLen[2] = {0, 0};
volatile int jpegReady = 0;
SemaphoreHandle_t jpegMutex;

void IRAM_ATTR leftEncoderISR() {
    uint8_t curr = (uint8_t)((digitalRead(ENCODER_LEFT_A_PIN) << 1) | digitalRead(ENCODER_LEFT_B_PIN));
    uint8_t idx = (uint8_t)((encLeftPrev << 2) | curr);
    encLeftPrev = curr;
    encLeft += QUAD_TABLE[idx & 0x0F];
}

void IRAM_ATTR rightEncoderISR() {
    uint8_t curr = (uint8_t)((digitalRead(ENCODER_RIGHT_A_PIN) << 1) | digitalRead(ENCODER_RIGHT_B_PIN));
    uint8_t idx = (uint8_t)((encRightPrev << 2) | curr);
    encRightPrev = curr;
    encRight += QUAD_TABLE[idx & 0x0F];
}

static void motorPinLow(uint8_t pin) {
    ledcDetachPin(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

static void motorPinPwm(uint8_t pin, uint8_t channel, int duty) {
    if (duty < 0) {
        duty = 0;
    }
    if (duty > MOTOR_MAX_SPEED) {
        duty = MOTOR_MAX_SPEED;
    }
    ledcAttachPin(pin, channel);
    ledcWrite(channel, (uint32_t)duty);
}

static void writeOneMotor(uint8_t pinFwd, uint8_t chFwd, uint8_t pinRev, uint8_t chRev, int speed, int *lastSpeed) {
    if (speed > MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
    } else if (speed < -MOTOR_MAX_SPEED) {
        speed = -MOTOR_MAX_SPEED;
    }

    int sign = (speed > MOTOR_DEADBAND) ? 1 : (speed < -MOTOR_DEADBAND) ? -1 : 0;
    int lastSign = (*lastSpeed > MOTOR_DEADBAND) ? 1 : (*lastSpeed < -MOTOR_DEADBAND) ? -1 : 0;

    if (sign != lastSign) {
        if (sign > 0) {
            motorPinLow(pinRev);
            motorPinPwm(pinFwd, chFwd, speed);
        } else if (sign < 0) {
            motorPinLow(pinFwd);
            motorPinPwm(pinRev, chRev, -speed);
        } else {
            motorPinLow(pinFwd);
            motorPinLow(pinRev);
        }
    } else if (sign > 0) {
        ledcWrite(chFwd, (uint32_t)speed);
    } else if (sign < 0) {
        ledcWrite(chRev, (uint32_t)(-speed));
    }

    *lastSpeed = speed;
}

static void computeBalance(float dt) {
    if (!imuInitialized || !balanceEnabled) {
        pidIntegral = 0.0f;
        pidOut = 0;
        pidError = 0.0f;
        return;
    }

    if (!balanceReady) {
        if (imuReadyMs != 0 && millis() - imuReadyMs >= BALANCE_WARMUP_MS) {
            balanceSetpoint = balanceUseRoll ? roll : pitch;
            balanceReady = true;
            pidIntegral = 0.0f;
            Serial.printf("Balance ready  axis=%s  setpoint=%.1f deg\n",
                          balanceUseRoll ? "roll" : "pitch", (double)balanceSetpoint);
        }
        pidOut = 0;
        pidError = 0.0f;
        return;
    }

    float angle = balanceUseRoll ? roll : pitch;
    float rate = balanceUseRoll ? lastGy : lastGx;
    float error = angle - balanceSetpoint;
    pidError = error;

    if (error > BALANCE_ANGLE_LIMIT || error < -BALANCE_ANGLE_LIMIT) {
        pidIntegral = 0.0f;
        pidOut = 0;
        return;
    }

    if (error < BALANCE_DEADBAND_DEG && error > -BALANCE_DEADBAND_DEG) {
        pidIntegral = 0.0f;
        pidOut = 0;
        return;
    }

    pidIntegral += PID_BALANCE_KI * error * dt;
    if (pidIntegral > PID_INTEGRAL_LIMIT) {
        pidIntegral = PID_INTEGRAL_LIMIT;
    } else if (pidIntegral < -PID_INTEGRAL_LIMIT) {
        pidIntegral = -PID_INTEGRAL_LIMIT;
    }

    float out = PID_BALANCE_KP * error + pidIntegral + (-PID_BALANCE_KD * rate);
    out *= (float)balanceDir;
    if (out > PID_OUTPUT_LIMIT) {
        out = PID_OUTPUT_LIMIT;
    } else if (out < -PID_OUTPUT_LIMIT) {
        out = -PID_OUTPUT_LIMIT;
    }
    if (out > -MOTOR_DEADBAND && out < MOTOR_DEADBAND) {
        out = 0;
    }
    pidOut = (int16_t)out;
}

static void applyDrive() {
    if (!motorsInitialized) {
        return;
    }

    char cmd = driveCmd;
    uint32_t age = millis() - lastDriveMs;
    if (cmd != 's' && age > DRIVE_HOLD_TIMEOUT_MS) {
        cmd = 's';
        driveCmd = 's';
    }

    int left = 0;
    int right = 0;
    switch (cmd) {
        case 'f':
            left = SPEED_FORWARD;
            right = SPEED_FORWARD;
            break;
        case 'b':
            left = -SPEED_BACKWARD;
            right = -SPEED_BACKWARD;
            break;
        case 'l':
            left = -SPEED_TURN_LEFT;
            right = SPEED_TURN_LEFT;
            break;
        case 'r':
            left = SPEED_TURN_RIGHT;
            right = -SPEED_TURN_RIGHT;
            break;
        default:
            left = 0;
            right = 0;
            break;
    }

    left += pidOut;
    right += pidOut;

    static int lastLeft = 0;
    static int lastRight = 0;
    int prevLeft = lastLeft;
    int prevRight = lastRight;
    appliedLeft = (int16_t)left;
    appliedRight = (int16_t)right;
    writeOneMotor(MOTOR_A_PIN1, MOTOR_PWM_CHANNEL_A1, MOTOR_A_PIN2, MOTOR_PWM_CHANNEL_A2, left, &lastLeft);
    writeOneMotor(MOTOR_B_PIN1, MOTOR_PWM_CHANNEL_B1, MOTOR_B_PIN2, MOTOR_PWM_CHANNEL_B2, right, &lastRight);
    static int16_t lastPidLog = 0;
    if ((lastPidLog == 0) != (pidOut == 0)) {
        Serial.printf("BAL %s  %s err=%.1f pwm=%d\n",
                      pidOut ? "SPIN" : "STILL",
                      balanceUseRoll ? "roll" : "pitch",
                      (double)pidError, (int)pidOut);
    }
    lastPidLog = pidOut;
    if ((left != prevLeft || right != prevRight) && cmd != 's') {
        Serial.printf("PWM L=%d R=%d pid=%d\n", left, right, (int)pidOut);
    }
}

static void updateLedsAndImu() {
    uint32_t now = millis();

    if (imuInitialized) {
        static uint32_t lastImu = 0;
        if (now - lastImu >= PID_LOOP_PERIOD_MS) {
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
                lastGx = gx;
                lastGy = gy;
                float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
                float accelRoll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
                pitch = 0.98f * (pitch + gx * dt) + 0.02f * accelPitch;
                roll = 0.98f * (roll + gy * dt) + 0.02f * accelRoll;
                yaw += gz * dt;
            }
            computeBalance(dt);
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
    applyDrive();
}

void ledImuTask(void *param) {
    (void)param;
    for (;;) {
        updateLedsAndImu();
        vTaskDelay(pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));
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

bool initializeMotors() {
    const int pins[] = {MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_B_PIN1, MOTOR_B_PIN2};
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin((gpio_num_t)pins[i]);
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }

    if (!ledcSetup(MOTOR_PWM_CHANNEL_A1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION) ||
        !ledcSetup(MOTOR_PWM_CHANNEL_A2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION) ||
        !ledcSetup(MOTOR_PWM_CHANNEL_B1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION) ||
        !ledcSetup(MOTOR_PWM_CHANNEL_B2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION)) {
        Serial.println("Motor PWM setup failed");
        return false;
    }

    motorsInitialized = true;
    Serial.printf("Motors ready A=%d/%d B=%d/%d (LEDC %d,%d,%d,%d @ %dHz, idle LOW)\n",
                  MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_B_PIN1, MOTOR_B_PIN2,
                  MOTOR_PWM_CHANNEL_A1, MOTOR_PWM_CHANNEL_A2,
                  MOTOR_PWM_CHANNEL_B1, MOTOR_PWM_CHANNEL_B2,
                  MOTOR_PWM_FREQ);
    return true;
}

bool initializeEncoders() {
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

    encLeftPrev = (uint8_t)((digitalRead(ENCODER_LEFT_A_PIN) << 1) | digitalRead(ENCODER_LEFT_B_PIN));
    encRightPrev = (uint8_t)((digitalRead(ENCODER_RIGHT_A_PIN) << 1) | digitalRead(ENCODER_RIGHT_B_PIN));
    encLeft = 0;
    encRight = 0;

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B_PIN), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B_PIN), rightEncoderISR, CHANGE);

    encodersInitialized = true;
    Serial.printf("Encoders ready L=%d/%d R=%d/%d (TTL USB only, no OTG)\n",
                  ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN,
                  ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    return true;
}

static void setDrive(char c, const char *label) {
    if (c != 'f' && c != 'b' && c != 'l' && c != 'r' && c != 's') {
        c = 's';
    }
    driveCmd = c;
    lastDriveMs = millis();
    Serial.printf("DRIVE %s\n", label);
    server.send(200, "text/plain", label);
}

static void setDriveFromRequest() {
    char c = 's';
    if (server.hasArg("d") && server.arg("d").length() > 0) {
        c = server.arg("d").charAt(0);
    }
    const char *label = (c == 'f') ? "FWD" : (c == 'b') ? "BACK" : (c == 'l') ? "LEFT" : (c == 'r') ? "RIGHT" : "STOP";
    setDrive(c, label);
}

void setupWebServer() {
    server.on("/", []() {
        server.sendHeader("Cache-Control", "no-store");
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
            ".pad{display:grid;grid-template-columns:72px 72px 72px;gap:8px;justify-content:center;margin:18px auto}"
            ".pad .btn{width:72px;height:56px;padding:0;user-select:none;touch-action:none}"
            ".hint{text-align:center;color:#888;font-size:13px;margin-top:4px}"
            "</style></head><body>"
            "<h1>R6 Recon Drone</h1>"
            "<div style='text-align:center'><img id='cam' class='video' alt='camera'></div>"
            "<div class='pad'>"
            "<span></span>"
            "<button type='button' class='btn' id='bf'>FWD</button>"
            "<span></span>"
            "<button type='button' class='btn' id='bl'>LEFT</button>"
            "<button type='button' class='btn' id='bs'>STOP</button>"
            "<button type='button' class='btn' id='br'>RIGHT</button>"
            "<span></span>"
            "<button type='button' class='btn' id='bb'>BACK</button>"
            "<span></span>"
            "</div>"
            "<div class='hint'>Balance uses the board pose ~1.5s after boot as level. Tilt to spin, hold still to stop. Pad still works.</div>"
            "<div class='controls'>"
            "<button class='btn' onclick=\"fetch('/balance/toggle')\">Toggle Balance</button>"
            "<button class='btn' onclick=\"fetch('/balance/level')\">Set level now</button>"
            "<button class='btn' onclick=\"fetch('/balance/axis')\">Pitch / Roll axis</button>"
            "<button class='btn' onclick=\"fetch('/balance/invert')\">Invert motor dir</button>"
            "</div>"
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
            "Yaw <span class='data' id='y'>--</span><br>"
            "Enc L <span class='data' id='el'>--</span> | "
            "Enc R <span class='data' id='er'>--</span> | "
            "Drive <span class='data' id='drv'>stop</span> | "
            "Bal <span class='data' id='bal'>--</span><br>"
            "Axis <span class='data' id='ax'>--</span> | "
            "Err <span class='data' id='be'>--</span> | "
            "PID pwm <span class='data' id='bp'>--</span>"
            "</div>"
            "<script>"
            "var cam=document.getElementById('cam'), oldUrl=null, busy=false;"
            "var holdTimer=null,holdCmd='s',holding=false,holdStart=0;"
            "function frame(){"
            "if(busy||holding){if(holding){setTimeout(frame,80);}return;}"
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
            "function status(){"
            "fetch('/status',{cache:'no-store'}).then(function(r){return r.json();}).then(function(d){"
            "document.getElementById('p').textContent=d.pitch.toFixed(1)+'\\u00b0';"
            "document.getElementById('r').textContent=d.roll.toFixed(1)+'\\u00b0';"
            "document.getElementById('y').textContent=d.yaw.toFixed(1)+'\\u00b0';"
            "document.getElementById('el').textContent=d.encL;"
            "document.getElementById('er').textContent=d.encR;"
            "document.getElementById('drv').textContent=d.drive;"
            "document.getElementById('bal').textContent=d.bal;"
            "document.getElementById('ax').textContent=d.axis;"
            "document.getElementById('be').textContent=d.err.toFixed(1)+'\\u00b0';"
            "document.getElementById('bp').textContent=d.pid;"
            "}).catch(function(){});"
            "}"
            "function drive(d){fetch('/'+d,{cache:'no-store'});}"
            "function startHold(cmd){"
            "holding=true;holdCmd=cmd;holdStart=Date.now();drive(cmd);"
            "if(holdTimer){clearInterval(holdTimer);}"
            "holdTimer=setInterval(function(){drive(holdCmd);},150);"
            "}"
            "function stopHold(){"
            "var cmd=holdCmd;"
            "var dur=Date.now()-holdStart;"
            "holding=false;"
            "if(holdTimer){clearInterval(holdTimer);holdTimer=null;}"
            "holdCmd='s';"
            "if(cmd!=='s' && dur<300){drive(cmd);}else{drive('stop');}"
            "}"
            "function bindHold(id,cmd){"
            "var el=document.getElementById(id);"
            "el.onmousedown=function(e){e.preventDefault();startHold(cmd);};"
            "el.onmouseup=function(e){e.preventDefault();stopHold();};"
            "el.ontouchstart=function(e){e.preventDefault();startHold(cmd);};"
            "el.ontouchend=function(e){e.preventDefault();stopHold();};"
            "}"
            "bindHold('bf','fwd');bindHold('bb','back');bindHold('bl','left');bindHold('br','right');"
            "document.getElementById('bs').onclick=function(){stopHold();};"
            "window.onmouseup=function(){if(holding)stopHold();};"
            "var keys={w:'fwd',a:'left',s:'back',d:'right',ArrowUp:'fwd',ArrowLeft:'left',ArrowDown:'back',ArrowRight:'right'};"
            "window.addEventListener('keydown',function(e){"
            "if(e.repeat)return;"
            "var c=keys[e.key];"
            "if(c){e.preventDefault();startHold(c);}"
            "});"
            "window.addEventListener('keyup',function(e){"
            "if(keys[e.key]){e.preventDefault();stopHold();}"
            "});"
            "window.addEventListener('blur',function(){stopHold();});"
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
            "setInterval(status,200);"
            "status();"
            "</script></body></html>");
    });

    server.on("/jpg", handleJpg);

    server.on("/imu", []() {
        char json[96];
        snprintf(json, sizeof(json), "{\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f}",
                 (double)pitch, (double)roll, (double)yaw);
        server.send(200, "application/json", json);
    });

    server.on("/status", []() {
        char json[320];
        char d = driveCmd;
        const char *drive = (d == 'f') ? "fwd" : (d == 'b') ? "back" : (d == 'l') ? "left" : (d == 'r') ? "right" : "stop";
        const char *bal = !balanceEnabled ? "off" : (!balanceReady ? "warmup" : "on");
        snprintf(json, sizeof(json),
                 "{\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,\"encL\":%ld,\"encR\":%ld,"
                 "\"left\":%d,\"right\":%d,\"drive\":\"%s\",\"bal\":\"%s\",\"axis\":\"%s\","
                 "\"set\":%.1f,\"err\":%.1f,\"pid\":%d}",
                 (double)pitch, (double)roll, (double)yaw,
                 (long)encLeft, (long)encRight,
                 (int)appliedLeft, (int)appliedRight, drive,
                 bal, balanceUseRoll ? "roll" : "pitch",
                 (double)balanceSetpoint, (double)pidError, (int)pidOut);
        server.send(200, "application/json", json);
    });

    server.on("/drive", HTTP_GET, setDriveFromRequest);
    server.on("/drive", HTTP_POST, setDriveFromRequest);
    server.on("/fwd", []() { setDrive('f', "FWD"); });
    server.on("/back", []() { setDrive('b', "BACK"); });
    server.on("/left", []() { setDrive('l', "LEFT"); });
    server.on("/right", []() { setDrive('r', "RIGHT"); });
    server.on("/stop", []() { setDrive('s', "STOP"); });

    server.on("/balance/toggle", []() {
        balanceEnabled = !balanceEnabled;
        pidIntegral = 0.0f;
        pidOut = 0;
        if (balanceEnabled) {
            balanceReady = false;
            imuReadyMs = millis();
        }
        Serial.printf("Balance: %s\n", balanceEnabled ? "ON" : "OFF");
        server.send(200, "text/plain", balanceEnabled ? "ON" : "OFF");
    });

    server.on("/balance/level", []() {
        balanceSetpoint = balanceUseRoll ? roll : pitch;
        pidIntegral = 0.0f;
        balanceReady = true;
        Serial.printf("Balance setpoint %.1f deg\n", (double)balanceSetpoint);
        server.send(200, "text/plain", "OK");
    });

    server.on("/balance/axis", []() {
        balanceUseRoll = !balanceUseRoll;
        balanceSetpoint = balanceUseRoll ? roll : pitch;
        pidIntegral = 0.0f;
        balanceReady = true;
        Serial.printf("Balance axis: %s  setpoint=%.1f\n",
                      balanceUseRoll ? "roll" : "pitch", (double)balanceSetpoint);
        server.send(200, "text/plain", balanceUseRoll ? "roll" : "pitch");
    });

    server.on("/balance/invert", []() {
        balanceDir = (int8_t)(-balanceDir);
        Serial.printf("Balance dir: %d\n", (int)balanceDir);
        server.send(200, "text/plain", balanceDir > 0 ? "+1" : "-1");
    });

    server.on("/enc/reset", []() {
        encLeft = 0;
        encRight = 0;
        server.send(200, "text/plain", "OK");
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
    Serial.println("R6 Drone - Breadboard PID stand test");
    Serial.println("PID balance ON after warmup  |  hold-to-drive ON");
    Serial.println("Use TTL USB-C only (encoders on GPIO 19/20)");
    Serial.println("=================================");
    Serial.printf("LED=%d  Spot=%d  IMU SDA=%d SCL=%d\n",
                  LED_DATA_PIN, SPOTLIGHT_PIN, IMU_SDA_PIN, IMU_SCL_PIN);
    Serial.printf("Motor A=%d/%d  B=%d/%d\n",
                  MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_B_PIN1, MOTOR_B_PIN2);
    Serial.printf("Enc L=%d/%d  R=%d/%d\n",
                  ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN,
                  ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

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

    initializeEncoders();

    if (initializeIMU()) {
        imuInitialized = true;
        imuReadyMs = millis();
        Serial.println("IMU ready (balance warmup 1.5s, keep the board still)");
    }

    if (initializeCamera()) {
        cameraInitialized = true;
        Serial.println("Camera ready (HVGA latest-frame)");
    }

    initializeMotors();

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
        Serial.printf("[%ds] Mem:%dKB IMU:%s Bal:%s/%s err=%.1f pid=%d Enc L/R:%ld/%ld Drive:%c\n",
                      millis() / 1000,
                      ESP.getFreeHeap() / 1024,
                      imuInitialized ? "ON" : "OFF",
                      balanceEnabled ? (balanceReady ? "ON" : "WARM") : "OFF",
                      balanceUseRoll ? "roll" : "pitch",
                      (double)pidError, (int)pidOut,
                      (long)encLeft, (long)encRight,
                      driveCmd);
    }
    delay(1);
}
