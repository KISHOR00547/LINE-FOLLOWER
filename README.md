# LINE-FOLLOWER
<img width="524" height="489" alt="image" src="https://github.com/user-attachments/assets/5219c467-9a15-4268-a3d0-6181bebba7b2" />

# High-Speed Line Follower Robot

## Overview
This project is a high-speed autonomous line follower robot developed for robotics competition challenges. The robot uses a **16-array QTR-inspired sensor setup**, PID-based control, and aggressive turn-handling logic for fast and stable navigation through complex tracks.

The objective was to complete the track by accurately following the line, detecting checkpoints, and handling sharp turns at high speed.

Although our robot was eliminated in the first round due to a logical checkpoint-handling issue, the project provided valuable hands-on experience in embedded systems, control algorithms, and real-time debugging.

---

## Hardware Components
- ESP32 Dev Module
- 16-array QTR Line Sensors
- N20 600 RPM DC Motors
- TB6612 Motor Driver
- Li-ion Battery
- Buck Converter

---

## Features
- High-speed autonomous line following
- PID-based line tracking
- Sharp acute turn detection
- Line loss recovery logic
- Strong right-priority decision logic
- Dynamic speed adjustment
- Real-time sensor-based steering correction
- High-speed competition optimization

---

## System Working

### 1. Line Detection
The robot continuously reads sensor values from the line sensor array.

The sensors detect:
- Left line presence
- Center line alignment
- Right line deviation
- Extreme edge detection

Weighted sensor positioning is used to calculate the robot’s relative error from the center line.

---

### 2. PID Control
The robot uses PID-inspired control logic:

- **Proportional control** for steering correction
- **Derivative filtering** for smoother response
- Dynamic gain switching based on turn severity

This allows stable tracking even at higher speeds.

---

### 3. Acute Turn Handling
For sharp turns:

- The robot detects strong left/right line shifts
- Confidence-based detection avoids false triggers
- Snap-turn correction is applied using aggressive motor steering

This improves performance in competition-style tracks.

---

### 4. Line Loss Recovery
If the robot completely loses the line:

- It remembers the last detected direction
- Rotates toward the previous line direction
- Recovery speed increases dynamically based on time since line loss

---

### 5. Priority Turn Logic
The navigation logic prioritizes aggressive right turns when specific sensor conditions are detected.

This was optimized for the competition track layout.

---

## Software Details
### Language
- C++

### Platform
- ESP32 Arduino Framework

### Control Techniques
- PID Control
- Sensor weighting
- State-based navigation
- Turn confidence logic
- Dynamic speed adaptation

---

## Performance Configuration
### Speed Settings
- Base Speed: 150 PWM
- Boost Speed: 180 PWM
- Turn Speed: 150 PWM
- Snap Turn Speed: 245 PWM

### Control Parameters
- Adaptive PID gains
- Sensor threshold calibration
- Filtered derivative correction
- Soft turn compensation

---

## Challenges Faced
During the event:

- The robot missed one checkpoint due to a logic issue
- This caused elimination in the first round

However, the development process involved:

- Late-night debugging
- Control algorithm tuning
- Sensor calibration
- Speed optimization
- Real-world competition testing

This experience significantly improved our understanding of autonomous robotics.

---

## Key Learnings
This project helped us learn:

- ESP32 embedded programming
- High-speed motor control
- PID tuning
- Real-time sensor processing
- Competition robotics debugging
- Decision-making logic design
- Recovery algorithm implementation

---

## Future Improvements
- Checkpoint detection fix
- Full PID with integral control
- Better intersection handling
- Dynamic track adaptation
- Speed optimization
- Improved competition reliability

---

## Competition Experience
Even though we were eliminated early, this project was one of our most valuable practical learning experiences in robotics.

Debugging, testing, and optimizing the bot under competition pressure taught us far more than simulation alone.
