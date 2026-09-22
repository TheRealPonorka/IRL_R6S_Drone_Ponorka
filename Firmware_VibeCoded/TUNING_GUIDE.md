# 🎯 Drone Tuning and Calibration Guide

This guide will walk you through the essential steps to calibrate your IMU sensor and tune the PID controller for optimal self-balancing performance.

## 📋 Prerequisites

Before starting, ensure you have:
- ✅ All hardware properly connected according to the wiring diagram
- ✅ Code uploaded to the ESP32-S3
- ✅ Serial monitor access (PlatformIO or Arduino IDE)
- ✅ A flat, level surface for testing
- ✅ Safety glasses (motors can spin fast!)

## 🔧 Part 1: IMU Calibration

### What is IMU Calibration?
The BMI270 sensor needs to know its "zero" position when the drone is perfectly level. This removes sensor bias and drift for accurate balance detection.

### Step-by-Step IMU Calibration

#### 1. **Prepare the Drone**
```bash
# Open serial monitor first
pio device monitor
# OR use Arduino IDE Serial Monitor at 115200 baud
```

#### 2. **Position the Drone**
- Place drone on a **completely flat surface** (use a spirit level if available)
- Ensure the drone is **not moving at all**
- Keep the area **vibration-free** (no walking around, no washing machine running)

#### 3. **Power On and Observe**
When you power on, you should see output like:
```
=================================
Rainbow Six: Siege Recon Drone
Firmware Version: 1.0.0
=================================
Initializing BMI270 IMU...
Calibrating BMI270...
Calibration progress: 0/1000
Calibration progress: 100/1000
Calibration progress: 200/1000
...
Calibration progress: 900/1000
Gyro offsets: X=-1.234, Y=0.567, Z=-0.123
Accel offsets: X=0.045, Y=-0.012, Z=0.023
BMI270 calibration complete
```

#### 4. **Calibration Success Indicators**
✅ **Good calibration**: Offset values between -5.0 and +5.0  
❌ **Poor calibration**: Offset values > ±10.0 or calibration fails

#### 5. **If Calibration Fails**
```cpp
// Troubleshooting steps:
1. Check I2C connections (GPIO 1, 2)
2. Verify BMI270 power supply (3.3V)
3. Ensure drone is completely still
4. Try different surface (more level)
5. Restart and try again
```

### Manual Re-calibration
If you need to recalibrate later, there are several options:

#### Option A: Automatic on Startup (Default)
- Simply restart the drone on a level surface
- Calibration runs automatically each boot

#### Option B: Web Interface Calibration (Future Feature)
We can add a calibration button to the web interface if needed.

#### Option C: Serial Commands (Advanced)
```cpp
// Add this to your serial monitor while drone is running
CALIBRATE_IMU
```

## 🎮 Part 2: PID Tuning

### What is PID Control?
PID stands for:
- **P**roportional: How hard to push back when tilted
- **I**ntegral: Corrects for steady-state errors over time  
- **D**erivative: Dampens oscillations and overshooting

### Understanding the Three PID Controllers

Your drone has **three separate PID controllers**:

#### 1. **Balance PID** (Most Important)
```cpp
// In config.h - these control the main balancing
#define PID_BALANCE_KP 30.0f    // Proportional gain
#define PID_BALANCE_KI 0.5f     // Integral gain  
#define PID_BALANCE_KD 0.8f     // Derivative gain
```
**Purpose**: Keeps the drone upright (pitch control)

#### 2. **Velocity PID** 
```cpp
#define PID_VELOCITY_KP 2.0f    // Forward/backward movement
#define PID_VELOCITY_KI 0.1f    
#define PID_VELOCITY_KD 0.05f   
```
**Purpose**: Controls forward/backward movement smoothness

#### 3. **Position PID**
```cpp
#define PID_POSITION_KP 1.5f    // Left/right turning
#define PID_POSITION_KI 0.0f    
#define PID_POSITION_KD 0.1f    
```
**Purpose**: Controls turning and rotation

### Step-by-Step PID Tuning Process

#### Phase 1: Safety Setup
1. **Prepare Testing Area**
   - Clear 3x3 meter space minimum
   - Soft surface (carpet or foam mats)
   - Remove breakable objects
   - Have emergency stop ready (spacebar in web interface)

2. **Initial Test Position**
   - Hold drone in your hands initially
   - Feel the motor response
   - Check that motors spin in correct directions

#### Phase 2: Basic Balance Tuning

**Start with Conservative Values:**
```cpp
// In config.h - start with these SAFE values
#define PID_BALANCE_KP 10.0f    // Start low!
#define PID_BALANCE_KI 0.0f     // Disable integral first
#define PID_BALANCE_KD 0.2f     // Start low!
```

**Tuning Process:**

1. **Test Kp (Proportional) First**
   ```cpp
   // Step 1: Start with Kp=5, Ki=0, Kd=0
   // Gently tilt drone - does it try to correct?
   
   // If NO response: Increase Kp by 5
   // If WEAK response: Increase Kp by 2-3
   // If STRONG response but oscillates: Decrease Kp
   // If VIOLENT oscillation: EMERGENCY STOP, lower Kp significantly
   ```

2. **Add Kd (Derivative) for Stability**
   ```cpp
   // Step 2: Once Kp gives decent correction, add Kd
   // Start with Kd = Kp / 20
   // Example: If Kp=20, try Kd=1.0
   
   // Too much oscillation: Increase Kd
   // Too sluggish: Decrease Kd
   // Good balance but slight bounce: Fine-tune both
   ```

3. **Fine-tune Ki (Integral) Last**
   ```cpp
   // Step 3: Add Ki only if drone drifts when "balanced"
   // Start with Ki = Kp / 100
   // Example: If Kp=20, try Ki=0.2
   
   // Ki too high: Oscillation and instability
   // Ki too low: Slow drift correction
   ```

#### Phase 3: Real-World Testing

**Progressive Testing:**

1. **Hand Testing**
   - Hold drone, tilt gently, observe correction
   - Should feel like it "wants" to stay level
   - No violent jerking or continuous buzzing

2. **Table Edge Testing**  
   - Place on table edge (safely!)
   - Let one wheel hang over edge
   - Should balance itself back onto table

3. **Free Balance Testing**
   - Place on flat surface
   - Should stay upright for 10+ seconds
   - Minor adjustments are normal

4. **Movement Testing**
   - Try gentle forward/backward commands
   - Should move smoothly without face-planting
   - Should stop and balance when released

### 🔍 Troubleshooting Common Issues

#### Problem: Violent Oscillation
```cpp
// Solution: Reduce Kp dramatically
#define PID_BALANCE_KP 5.0f     // Was too high
#define PID_BALANCE_KD 0.5f     // Add more damping
```

#### Problem: No Response to Tilt
```cpp
// Solution: Check motor directions and increase Kp
// Verify motors spin correct direction for correction
#define PID_BALANCE_KP 25.0f    // Increase response
```

#### Problem: Balances but Drifts
```cpp
// Solution: Add integral term
#define PID_BALANCE_KI 0.3f     // Correct steady-state error
```

#### Problem: Unstable at Speed
```cpp
// Solution: Tune velocity PID
#define PID_VELOCITY_KP 1.0f    // Reduce for smoother movement
#define PID_VELOCITY_KD 0.1f    // Add damping
```

### 📊 Tuning Parameter Ranges

| Parameter | Typical Range | Start Value | Notes |
|-----------|---------------|-------------|-------|
| Balance Kp | 5.0 - 50.0 | 15.0 | Most critical parameter |
| Balance Ki | 0.0 - 2.0 | 0.0 | Add last, use sparingly |
| Balance Kd | 0.1 - 5.0 | 0.5 | Prevents oscillation |
| Velocity Kp | 0.5 - 5.0 | 1.5 | Smooth movement |
| Position Kp | 0.5 - 3.0 | 1.0 | Turning response |

## 🚨 Safety Guidelines

### Emergency Procedures
1. **Spacebar** = Emergency stop (web interface)
2. **Power switch** = Ultimate emergency stop
3. **Tilt limit** = 45° automatic stop (already coded)

### Safe Tuning Practices
- ✅ Start with LOW values, increase gradually
- ✅ Test in safe area with soft surfaces  
- ✅ Keep emergency stop accessible
- ✅ Never tune with people nearby
- ✅ Wear safety glasses
- ❌ Don't tune near walls or furniture
- ❌ Don't skip the hand-testing phase
- ❌ Don't increase all parameters at once

## 📈 Advanced Tuning Tips

### Real-Time Tuning (Advanced)
You can modify PID values while running by adding serial commands:

```cpp
// Example serial commands to add to main.cpp:
// "KP:25.0" = Set balance Kp to 25.0
// "KI:0.5"  = Set balance Ki to 0.5  
// "KD:1.2"  = Set balance Kd to 1.2
```

### Data Logging for Analysis
The telemetry shows current values:
- **Pitch angle**: Should stay near 0°
- **Motor speeds**: Should be smooth, not jerky
- **CPU usage**: Should stay under 80%

### Environmental Factors
- **Surface**: Hard floors need different tuning than carpet
- **Weight**: Adding camera/payload affects balance point
- **Battery level**: Lower voltage = less responsive motors
- **Temperature**: Can affect sensor readings slightly

## 🎯 Success Criteria

Your drone is properly tuned when:
- ✅ Stays upright on flat surface for 30+ seconds
- ✅ Recovers from gentle pushes smoothly
- ✅ Moves forward/backward without falling
- ✅ Turns left/right controllably
- ✅ No violent oscillations or buzzing
- ✅ Responds to web interface commands reliably

## 🔧 Quick Reference: Tuning Workflow

```bash
1. IMU Calibration (automatic on flat surface)
2. Set conservative PID values
3. Hand test motor response
4. Gradually increase Kp until responsive
5. Add Kd to reduce oscillation
6. Test free balancing
7. Add Ki if needed for drift
8. Test movement commands
9. Fine-tune for your environment
10. Document final values!
```

## 📝 Final Values Template

Once you find good values, document them:

```cpp
// My Final Tuned Values - [Date]
// Surface: [carpet/hardwood/concrete]
// Weight: [total drone weight]
#define PID_BALANCE_KP XX.Xf
#define PID_BALANCE_KI X.Xf  
#define PID_BALANCE_KD X.Xf
#define PID_VELOCITY_KP X.Xf
#define PID_VELOCITY_KI X.Xf
#define PID_VELOCITY_KD X.Xf
// Notes: [what works well, any quirks]
```

## 🆘 Getting Help

If you run into issues:
1. Check serial monitor output for error messages
2. Verify all connections match the wiring diagram
3. Ensure IMU calibration completed successfully
4. Start with very conservative PID values
5. Test each component individually (IMU, motors, encoders)

Remember: **PID tuning is an iterative process**. Don't expect perfection on the first try. Each adjustment teaches you how your specific drone behaves!

Good luck with your R6 Recon Drone! 🚁