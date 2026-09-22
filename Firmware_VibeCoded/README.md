# R6 Recon Drone - Rainbow Six: Siege Replica

A fully functional self-balancing camera drone replica inspired by the recon drones from Tom Clancy's Rainbow Six: Siege. Built with ESP32-S3 and featuring a responsive web interface, real-time camera streaming, and advanced control systems.

## 🎯 Features

### Hardware
- **MCU**: ESP32-S3 CAM N16R8 (GOOUUU v1.5) - 16MB Flash, 8MB PSRAM
- **IMU**: BMI270 6-axis sensor for precise balance control
- **Motors**: Dual DRV8833 H-bridge motor controller
- **Camera**: OV3660 with HD 1280x720 default resolution
- **Audio**: INMP441 I2S digital microphone
- **Lighting**: WS2812B RGB LED strip (12 LEDs)
- **Spotlight**: High-power LED with PWM control
- **Encoders**: Quadrature encoders for precise movement tracking

### Software Architecture
- **Dual-Core FreeRTOS**: 
  - Core 0: High-speed IMU/PID balance control at 100Hz
  - Core 1: Web server, camera streaming, LED animations
- **Real-time PID Control**: Advanced balancing algorithms
- **Responsive Web Interface**: Multi-touch support, mobile-optimized
- **Live Camera Streaming**: HD video with configurable settings
- **WiFi Connectivity**: Station mode with AP fallback

### Control Features
- **Movement Controls**: Forward, backward, left, right with smooth acceleration
- **Camera Stream**: Real-time HD video with overlay controls
- **Audio Streaming**: Live microphone feed (when enabled)
- **LED Control**: RGB color picker with animations and patterns
- **Spotlight Control**: Variable brightness white LED
- **Emergency Safety**: Automatic tilt detection and motor cutoff

## 🛠️ Hardware Setup

### Pin Configuration

| Component | Pin Assignment |
|-----------|----------------|
| **BMI270 IMU (I2C)** | SCL=1, SDA=2 |
| **DRV8833 Motors** | Motor A: 35,36 / Motor B: 37,38 |
| **Encoders** | Left: 45,46 / Right: 48,21 |
| **INMP441 Microphone** | SCK=41, WS=42, SD=20 |
| **WS2812B LEDs** | Data=3 |
| **Spotlight** | Gate=14 |
| **OV3660 Camera** | SIOD=4, SIOC=5, VSYNC=6, HREF=7, PCLK=13, XCLK=15 |
| **Camera Data** | D0-D7: [11,9,8,10,12,18,17,16] |

### Wiring Diagram
```
ESP32-S3 CAM N16R8
├── I2C Bus (IMU)
│   ├── SCL (GPIO 1) ──→ BMI270 SCL
│   └── SDA (GPIO 2) ──→ BMI270 SDA
├── Motor Control
│   ├── GPIO 35 ──→ DRV8833 Motor A1
│   ├── GPIO 36 ──→ DRV8833 Motor A2  
│   ├── GPIO 37 ──→ DRV8833 Motor B1
│   └── GPIO 38 ──→ DRV8833 Motor B2
├── LED & Lighting
│   ├── GPIO 3 ──→ WS2812B Data In
│   └── GPIO 14 ──→ Spotlight Gate
└── Audio & Sensors
    ├── I2S (GPIO 41,42,20) ──→ INMP441
    └── Encoders (GPIO 45,46,48,21)
```

## 🚀 Quick Start

### 1. PlatformIO Setup
```bash
# Clone the repository
git clone <repository-url>
cd IRL_R6_Drone

# Install PlatformIO (if not already installed)
pip install platformio

# Build the project
pio run

# Upload to ESP32-S3
pio run --target upload

# Monitor serial output
pio device monitor
```

### 2. WiFi Configuration
1. Power on the drone
2. Connect to WiFi network "R6_Drone_Network" (password: "Rainbow6Siege")
3. Or modify `config.h` with your WiFi credentials
4. Access the web interface at the drone's IP address

### 3. Web Interface Access
- **Main Control**: `http://[drone-ip]/`
- **Camera Config**: `http://[drone-ip]/camera-config.html`
- **WebSocket**: `ws://[drone-ip]:81`

## 🎮 Web Interface Guide

### Main Control Interface
- **Camera Stream**: Central HD video feed with overlay controls
- **Movement**: Touch-friendly directional buttons (Forward/Back/Left/Right)
- **Audio**: Microphone toggle in top-right overlay
- **Spotlight**: LED spotlight toggle in bottom-center overlay
- **LED Strip**: Color picker and on/off toggle in bottom-left
- **Fullscreen**: Expand interface to full screen
- **Telemetry**: Real-time sensor data display

### Camera Configuration
- **Resolution**: From 96x96 to 1920x1080 (FHD)
- **Quality**: JPEG compression settings
- **Image Enhancement**: Brightness, contrast, saturation
- **Special Effects**: Filters and color adjustments
- **Presets**: Quick configurations (Recon, Night Vision, High Quality, Performance)

### Control Commands
```javascript
// WebSocket command format
{
  "command": "forward|backward|left|right|microphone|spotlight|ledstrip|ledcolor",
  "value": true|false|"255,140,0",
  "timestamp": 1234567890
}
```

## 🔧 Configuration

### PID Tuning
Edit `config.h` to adjust PID parameters:
```cpp
// Balance PID (pitch control)
#define PID_BALANCE_KP 30.0f
#define PID_BALANCE_KI 0.5f
#define PID_BALANCE_KD 0.8f

// Velocity PID (speed control)  
#define PID_VELOCITY_KP 2.0f
#define PID_VELOCITY_KI 0.1f
#define PID_VELOCITY_KD 0.05f
```

### Camera Settings
```cpp
#define CAMERA_FRAME_SIZE FRAMESIZE_HD    // 1280x720
#define CAMERA_QUALITY 10                 // JPEG quality
#define CAMERA_FB_COUNT 2                 // Frame buffers
```

### Performance Tuning
```cpp
#define PID_LOOP_FREQUENCY_HZ 100         // Control loop speed
#define CAMERA_STREAM_FPS 30              // Video frame rate
#define LED_ANIMATION_UPDATE_MS 50        // LED refresh rate
```

## 📡 System Architecture

### FreeRTOS Task Distribution
| Core | Task | Priority | Stack | Frequency |
|------|------|----------|--------|-----------|
| 0 | IMU + PID + Motors | Critical | 4KB | 100Hz |
| 1 | Web Server | Medium | 6KB | Variable |
| 1 | Camera Stream | High | 8KB | 30FPS |
| 1 | LED Animation | Low | 2KB | 20Hz |
| 1 | Audio Process | Medium | 4KB | 50Hz |

### Data Flow
```
IMU Sensor → PID Controller → Motor PWM
     ↓              ↓
WebSocket ← Web Interface ← Camera Stream
     ↓
LED Controller + Audio Stream
```

## 🔒 Safety Features

- **Tilt Protection**: Emergency stop at ±45° tilt angle
- **Command Timeout**: Auto-stop after 2 seconds without commands
- **Watchdog Timer**: System reset on 5-second freeze
- **WiFi Monitoring**: Connection status indicators
- **Battery Monitoring**: Low voltage detection (if connected)

## 🎨 LED Patterns

| Pattern | Description | Trigger |
|---------|-------------|---------|
| Startup | Rainbow wipe | Boot sequence |
| Solid | User color | Normal operation |
| Breathing | Pulsing | Balancing mode |
| Emergency | Red strobe | Tilt detection |
| R6 Theme | Orange/Blue chase | Rainbow Six colors |

## 🐛 Troubleshooting

### Common Issues

**IMU Calibration Failed**
```
- Check I2C connections (GPIO 1,2)
- Ensure drone is level during startup
- Verify BMI270 power supply
```

**Camera Not Streaming**
```
- Check camera module connection
- Verify DVP pin assignments
- Restart with camera disconnected, then reconnect
```

**Motors Not Responding**
```
- Verify DRV8833 connections
- Check motor power supply
- Ensure emergency stop is not active
```

**Web Interface Not Loading**
```
- Confirm WiFi connection
- Check IP address in serial monitor
- Try AP mode if STA fails
```

### Debug Commands
```cpp
// Enable debug output
#define DEBUG_ENABLED true
#define CURRENT_LOG_LEVEL LOG_LEVEL_DEBUG

// Performance monitoring
#define PERFORMANCE_MONITORING true
#define MEMORY_MONITOR_ENABLED true
```

## 📈 Performance Optimization

### Memory Usage
- **Heap**: ~200KB free (of 320KB)
- **PSRAM**: ~7MB available for camera buffers
- **Flash**: ~14MB available for web assets and OTA

### CPU Utilization
- **Core 0**: 60-80% (PID control loop)
- **Core 1**: 40-60% (web server + camera)

## 🔄 OTA Updates

```bash
# Upload via WiFi (change IP address)
pio run -e esp32s3_r6drone_ota --target upload --upload-port 192.168.1.100
```

## 📝 License

This project is licensed under the MIT License. See `LICENSE` file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 🎖️ Credits

- **Author**: Peter
- **Inspired by**: Tom Clancy's Rainbow Six: Siege
- **ESP32 Framework**: Espressif Systems
- **Libraries**: See `platformio.ini` for full list

## 📞 Support

For issues, questions, or contributions:
1. Check the troubleshooting section
2. Review existing GitHub issues
3. Create a new issue with detailed information
4. Include serial monitor output for debugging

---

**⚠️ Safety Warning**: This drone contains spinning motors and electronic components. Always operate in a safe environment, keep fingers away from moving parts, and ensure proper battery handling. The emergency stop (spacebar) should always be accessible during operation.

**🎯 Have fun recreating the tactical recon experience from Rainbow Six: Siege!**