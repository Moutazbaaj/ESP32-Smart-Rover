# robot_Project

a rover project

# Smart Rover with ESP32

This is an open-source Smart Rover project built with ESP32 microcontrollers. The system includes a custom-built ESP-NOW remote controller and supports both manual and autonomous driving modes. The entire project is designed to operate without Wi-Fi, using low-latency ESP-NOW communication between the rover and its controller.

## Features

### ✅ ESP-NOW Wireless Communication
- No Wi-Fi or external server needed
- Low-latency, peer-to-peer communication between the controller and the rover
- Stable and efficient real-time command transfer

### ✅ Dual Motor Control with L298N
- Two DC motors: one for movement, one for steering
- Smooth directional control with PWM signals

### ✅ Autonomous Navigation Mode
- Servo-mounted ultrasonic sensor scans the environment
- Automatically avoids obstacles within 30 cm
- Performs 180° scan and selects the safest direction to move forward

### ✅ Manual Control via Custom ESP32 Remote
- Four-directional movement (forward, backward, left, right)
- Speed control via button combo
- Toggle autonomous/self-driving mode
- Instant command transmission over ESP-NOW

### ✅ LED Indication System
- Controlled using SN74HC595 shift register
- LED patterns:
  - White front lights for forward movement
  - Red rear lights for reverse
  - Yellow side lights for turning
  - Blinking yellow lights during scan mode

### ✅ Power System
- Powered by 3x AA 1.2V rechargeable batteries (3.6V total)
- Boost converter steps up to 5V for consistent ESP32 & motor operation

### ✅ Gyroscope & Accelerometer Integration (Upcoming)
- Integration of GY-BNO055 module
- Real-time tracking of orientation, angle, and movement
- Enhances autonomous mode decisions

### ✅ OLED Display on Controller (Upcoming)
- Displays:
  - Direction of movement
  - Angle and position of the rover
  - Status of autonomous decisions

## Current Progress

- [x] ESP-NOW Communication
- [x] Manual Motor Control
- [x] Obstacle Avoidance Algorithm
- [x] Ultrasonic Sensor with Servo
- [x] LED Indicators with Shift Register
- [ ] Gyroscope + Orientation Module//
- [ ] OLED Display Integration
- [ ] Solar system

## How It Works

### Autonomous Mode
1. Ultrasonic sensor checks for obstacles.
2. If blocked within 30 cm, rover halts.
3. Rotates 180° and scans for open paths.
4. Selects the optimal direction and speed.
5. Continues forward until the next scan cycle.

### Manual Mode
- Button inputs on the ESP32-based controller are transmitted via ESP-NOW.
- The rover interprets and acts on these commands in real-time.

## Components Used

| Component              | Description                                   |
|------------------------|-----------------------------------------------|
| ESP32 (x2)             | One for the rover, one for the controller     |
| L298N Motor Driver     | Dual DC motor control                         |
| DC Motors (x2)         | One for movement, one for steering            |
| Ultrasonic Sensor      | Distance detection for obstacle avoidance     |
| Servo Motor            | Mounted ultrasonic for dynamic scanning       |
| SN74HC595              | Shift register for LED control                |
| LEDs (White, Red, Yellow) | Indication system                         |
| GY-BNO055 (Upcoming)   | Gyroscope & Accelerometer                     |
| OLED Display (Upcoming)| Display on controller                        |
| 4x AA 1.5V Batteries   | Power supply    |
| LM2596 DC to DC Buck Converter | to step down the current to 5V  |
//update the motor driver
## Project Goals

- Build a feature-rich, educational rover platform
- Focus on autonomy, user interaction, and low-cost components
- Share full code and schematics as an open-source resource

## Open Source

The entire project will be published on GitHub once finalized, including:
- Full Arduino/ESP32 source code
- Wiring diagrams & schematics
- Controller layout & logic
- Parts list and purchasing links

Stay tuned for the GitHub release!

---

**Feedback & Contributions Welcome!**  
Let me know if you have ideas, suggestions, or want to collaborate on future features.

---

MIT License

Copyright (c) 2025 Moutaz Baaj

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights  
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell    
copies of the Software, and to permit persons to whom the Software is       
furnished to do so, subject to the following conditions:                     

The above copyright notice and this permission notice shall be included in   
all copies or substantial portions of the Software.                          

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         
DEALINGS IN THE SOFTWARE.

### Tags
`#ESP32` `#Arduino` `#SmartRover` `#Robotics` `#IoT` `#AutonomousVehicle` `#OpenSource` `#MakerProject` `#DIY` `#EmbeddedSystems` `#Microcontrollers`