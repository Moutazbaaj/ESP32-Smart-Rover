# ESP32 Smart Rover Project 🚗🔋☀️

This project showcases a fully functional, autonomous ESP32-powered Smart Rover with remote control capabilities. It includes complete source code, wiring instructions, and a detailed component breakdown for both the rover and its wireless controller.

---

## 📦 Table of Contents

1. [Components List](#components-list)
2. [Wiring Diagrams](#wiring-diagrams)
3. [Code Overview](#code-overview)
4. [Features and Functionality](#features-and-functionality)
5. [Operating Modes](#operating-modes)
6. [Power Setup](#power-setup)
7. [Controller Button Mapping](#controller-button-mapping)
8. [Troubleshooting](#troubleshooting)
9. [How to Control the Rover](#how-to-control-the-rover)

---

## 🔧 Components List

### 1. Microcontrollers
- **2× ESP32 Mini D1** – one for the rover, one for the controller  
  [Mini ESP32 Development Boards (Amazon)](https://www.amazon.de/dp/B0CJNMRG37?ref=ppx_pop_mob_ap_share)

### 2. Motors & Drivers
- **2× 3–6V DC Gear Motors** – rear-wheel drive  
  [Mini DC Gear Motors (Amazon)](https://amzn.eu/d/8vJv5VG)
- **1× 9G Micro Servo Motor** – rotates ultrasonic sensor  
  [9G Servo Motor (Amazon)](https://www.amazon.de/dp/B0CMT8CF76?ref=ppx_pop_mob_ap_share)
- **1× TB6612FNG Motor Driver** – dual DC motor driver  
  [TB6612FNG Module (Amazon)](https://www.amazon.de/dp/B09MN8F1SD?ref=ppx_pop_mob_ap_share)

### 3. Sensors
- **HC-SR04 Ultrasonic Sensor** – obstacle detection  
  [HC-SR04 (Amazon)](https://amzn.eu/d/ilI85lk)
- **IR Obstacle Sensor** – short-range detection  
  [IR Obstacle Module (Amazon)](https://amzn.eu/d/j3NJQmh)

### 4. Lighting
- **RGB LED Ring (8 LEDs)** – front lighting & visual feedback  
  [RGB LED Ring (Amazon)](https://www.amazon.de/dp/B0BZDQM8SX?ref=ppx_pop_mob_ap_share)

### 5. Power Supply

#### Logic Power
- **4× 1.5V AA Lithium Batteries** (regulated via LM2596)  
  [Miady Lithium AA Batteries (Amazon)](https://www.amazon.de/dp/B0DHZCVMVM)
- **1× LM2596 Buck Converter**  
  [LM2596 Converter (Amazon)](https://www.amazon.de/dp/B0823P6PW6)

#### Motor Power
- **1× 3000mAh 18650 Li-ion Battery**  
  [18650 Battery (Amazon)](https://www.amazon.de/dp/B0D73P7Q5C)
- **1× XL6009 Boost Converter** (3.7V → 6V)  
  [XL6009 Module (Amazon)](https://www.amazon.de/dp/B0D9VPKHLK)
- **1× BMS Module** – 18650 protection  
  [BMS Board (Amazon)](https://www.amazon.de/dp/B0CSJR4CYJ)
- **1× 6W Solar Panel** – solar charging  
  [6W Solar Panel (Amazon)](https://www.amazon.de/dp/B0B8HPS3SB)

### 6. Display
- **0.96" OLED Display** – mounted on the controller  
  [OLED Display (Amazon)](https://www.amazon.de/dp/B0CXY8SM1H)

---

## ⚙️ Wiring Diagrams

- Follow pin definitions from the source code.
- Ensure **all components share the same GND**.

---

## 💻 Code Overview

The codebase is divided into:

### 🔸 Rover Code
- Motor control
- Sensor reading
- ESP-NOW communication
- Autonomous navigation logic

### 🔹 Controller Code
- Button input handling
- OLED display updates
- Command transmission via ESP-NOW
- Manual and autonomous mode control

---

## 🌟 Features and Functionality

- 🚗 **Manual Driving** via tactile buttons
- 🤖 **Self-Driving Mode** with obstacle avoidance
- 📏 **Ultrasonic Scanning** with a servo motor
- 💡 **RGB LED Feedback**
- 📺 **OLED Status Display** (speed, direction, servo angle, distance)

---

## 🔄 Operating Modes

1. **Manual Mode** – Controlled via directional buttons.
2. **Autonomous Mode** – Activated by holding **Left + Right** for 3 seconds.

---

## 🔋 Power Setup

- **ESP32 + Peripherals**: Powered by 4× AA lithium batteries (regulated to 5V via LM2596).
- **Motors**: Powered by a solar-rechargeable 3000mAh 18650 battery (boosted to 6V via XL6009).
- **Fallback**: If the 18650 is depleted, motors draw power from the main 5V system.

---

## 🎮 Controller Button Mapping

- **Forward / Backward / Left / Right**: Directional movement
- **Center**: Center the ultrasonic servo (90°)
- **Up / Down**: Adjust servo angle
- **Light Button**: Toggle LED ring
- **Speed Buttons**: Adjust PWM motor speed
- **Autonomous Mode**: Hold Left + Right for 3 seconds

---

## 🛠️ Troubleshooting

| Issue                          | Solution                                              |
|-------------------------------|-------------------------------------------------------|
| Rover not moving              | Check motor wiring and power                         |
| No response from controller   | Verify ESP-NOW pairing and power supply               |
| Obstacle avoidance failing    | Confirm sensor positioning and remove obstructions    |

---

## 🕹️ How to Control the Rover

- Power on both the rover and controller.
- Use the OLED display to monitor:
  - Distance (cm)
  - Speed level
  - Servo angle
- Control the rover using the 5-way button:
  - **Forward/Back**: Drive
  - **Left/Right**: Steering
  - **Up/Down**: Servo angle
  - **Long press Left + Right**: Toggle autonomous mode

---

> Made with 💡, solar power ☀️, and a whole lot of ESP32 magic.
