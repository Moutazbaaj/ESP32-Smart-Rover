
# ESP32 Smart Rover Project

This repository contains the complete codebase, wiring details, and setup instructions for building an ESP32-powered Smart Rover system. It includes both the rover and the dedicated controller configurations.

---

**ROVER and Controller**

<table>
  <tr>
    <td><img src="IMG/8.HEIC" width="250"></td>
    <td><img src="IMG/9.HEIC" width="250"></td>
    <td><img src="IMG/12.HEIC" width="250"></td>
  </tr>
</table>

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Components List](#Components-list)
3. [Architecture and Wiring](#architecture-and-wiring)
4. [Code Structure](#code-structure)
5. [Features](#features)
6. [Power System](#power-system)
7. [Operating Modes](#operating-modes)
8. [Controller Button Mapping](#controller-button-mapping)
9. [Troubleshooting](#troubleshooting)
10. [User Customization Notes](#User-customization-notes)
11. [Control Instructions](#control-instructions)

---

## Project Overview

The ESP32 Smart Rover is a DIY autonomous rover platform featuring:

- Dual-microcontroller architecture (ESP-NOW wireless protocol)
- Obstacle avoidance using ultrasonic + IR sensors
- Servo-mounted scanning head for enhanced pathfinding
- Remote control via tactile button-based interface with OLED feedback
- Solar-charged dual battery system
- PWM-based speed control and LED signaling

---

**components**

<table>
  <tr>
    <td><img src="IMG/1.HEIC" width="250"></td>
    <td><img src="IMG/2.HEIC" width="250"></td>
    <td><img src="IMG/3.HEIC" width="250"></td>
    <td><img src="IMG/4.HEIC" width="250"></td>
  </tr>
</table>
---

## Components List

This is the full list of hardware components used in the ESP32 Smart Rover project.

### 1. Microcontrollers

* **2× ESP32 Mini D1** – one for the rover, one for the controller
  [Pack of 3 Mini ESP32 Development Boards (Amazon)](https://www.amazon.de/dp/B0CJNMRG37?ref=ppx_pop_mob_ap_share)

### 2. Motors & Motor Drivers

* **2× 3–6V DC Gear Motors** – rear-wheel drive and front steering
  [DIYDC Pack of 2 Mini DC Gear Motors (Amazon)](https://amzn.eu/d/8vJv5VG)
* **1× 9G Micro Servo Motor** – for ultrasonic sensor rotation
  [10pcs 9G Micro Servo Motor (Amazon)](https://www.amazon.de/dp/B0CMT8CF76?ref=ppx_pop_mob_ap_share)
* **1× TB6612FNG Motor Driver** – dual-channel driver for DC motors
  [2pcs TB6612FNG Dual DC Motor Driver (Amazon)](https://www.amazon.de/dp/B09MN8F1SD?ref=ppx_pop_mob_ap_share)

### 3. Sensors

* **1× Ultrasonic Distance Sensor (HC-SR04)** – for obstacle detection
  [Ultraschallsensor HC-SR04 (Amazon)](https://amzn.eu/d/ilI85lk)
* **1× IR Obstacle Detection Sensor** – short-range detection
  [IR Infrared Obstacle Avoidance Module (Amazon)](https://amzn.eu/d/j3NJQmh)

### 4. Lighting

* **1× RGB LED Ring (8 Bit)** – front-mounted for illumination and signals
  [AZDelivery 3x RGB LED Ring 8 Bit (Amazon)](https://www.amazon.de/dp/B0BZDQM8SX?ref=ppx_pop_mob_ap_share)

### 5. Power System

#### Primary Battery (Logic)

* **4× 1.5V AA Lithium Batteries** – for powering ESP32 and peripherals
  [Miady Rechargeable Lithium AA Batteries (Amazon)](https://www.amazon.de/dp/B0DHZCVMVM?ref=ppx_pop_mob_ap_share)
* **1× LM2596 Buck Converter** – steps down from \~6V to 5V
  [Yizhet LM2596 DC to DC Buck Converter (Amazon)](https://www.amazon.de/dp/B0823P6PW6?ref=ppx_pop_mob_ap_share)

#### Secondary Battery (Motors + Solar)

* **1× 3000mAh 18650 Lithium Battery** – solar-rechargeable power for motors
  [Pack of 6 18650 Battery 3.7V 3000mAh (Amazon)](https://www.amazon.de/dp/B0D73P7Q5C?ref=ppx_pop_mob_ap_share)
* **1× BMS Module** – Battery Management System for protection and regulation
  [GTIWUNG Pack of 10 3.7V 18650 BMS (Amazon)](https://www.amazon.de/dp/B0CSJR4CYJ?ref=ppx_pop_mob_ap_share)
* **1× XL6009 Boost Converter** – steps 3.7V up to 6V for motors
  [XL6009 DC-DC Boost Converter Module (Amazon)](https://www.amazon.de/dp/B0D9VPKHLK?ref=ppx_pop_mob_ap_share)
* **1× 6W Solar Panel** – recharges 18650 battery under sunlight
  [6W Solar Panel for Wireless Applications (Amazon)](https://www.amazon.de/dp/B0B8HPS3SB?ref=ppx_pop_mob_ap_share)

### 6. Display (Controller)

* **1× 0.96" OLED Display** – mounted on controller for feedback
  [VoltMate 3x 0.96 inch OLED Displays (Amazon)](https://www.amazon.de/dp/B0CXY8SM1H?ref=ppx_pop_mob_ap_share)

---

## Architecture and Wiring

- All GNDs are tied together to form a shared common ground.
- Control logic (ESP32 + sensors + OLED) powered from regulated 5V line.
- Motors are isolated via TB6612FNG and powered from stepped-up 6V.

> Refer to the pin assignments within the code and ensure VIN/EN pins are properly handled (ESP32 EN tied to 10k pull-up to reduce noise if needed).

---

**Solar System**

<table>
  <tr>
    <td><img src="IMG/5.HEIC" width="250"></td>
    <td><img src="IMG/6.HEIC" width="250"></td>
    <td><img src="IMG/7.HEIC" width="250"></td>
    <td><img src="IMG/11.HEIC" width="250"></td>
  </tr>
</table>
---

## Code Structure

### 1. `rover.ino`

- Sets up PWM for motor speed :

  Example:
```ccp
// PWM Configuration for motor speed control
#define PWMA 21    // Rear Motor PWM pin
#define PWMB 22    // Front Motor PWM pin
#define PWM_CHANNEL_A 21  // PWM Channel 2 for Rear Motor
#define PWM_CHANNEL_B 22  // PWM Channel 0 for Front Motor
#define PWM_FREQ 5000     // PWM frequency in Hz
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)
```



- Receives ESP-NOW commands, parses instruction struct:

  Example:
  ```cpp
  struct Command {
    int direction;
    int speed;
    bool toggleLight;
    bool autonomous;
  };
  ```
- Manages servo scanning using `SG90` on 9G Micro Servo
- Obstacle avoidance runs in main loop if `autonomousMode == true`
- LED ring controlled via `Adafruit_NeoPixel`

### 2. `controller.ino`

- Uses debounced tactile input
- Implements combo detection:
  - Simultaneous press of LEFT + RIGHT for 3s → toggles autonomous mode
- Sends structured data packet over ESP-NOW
- Displays on OLED via `Adafruit_SSD1306`:
  - Action label
  - Distance (from rover feedback)
  - Servo angle
  - Speed

---

**IOS COntroller**

<table>
  <tr>
    <td><img src="IMG/13.PNG" width="250"></td>
    <td><img src="IMG/14.PNG" width="250"></td>
  </tr>
</table>
---

## Features

- **Manual Control**  
- **Autonomous Driving with Obstacle Avoidance**
- **WIFI AP control**
- **PWM-based Speed Adjustment**  
- **Servo-Based Ultrasonic Scanning**  
- **OLED Display for Real-time Feedback**  
- **RGB LED Ring Status Indicator**  
- **ESP-NOW Wireless Communication**  
- **Solar Backup Charging System**

---

## Power System

| Source         | Powers                  | Regulated by         |
|----------------|--------------------------|-----------------------|
| 4× AA Lithium  | ESP32, sensors, OLED     | LM2596 to 5V          |
| 18650 Battery  | Motors                   | XL6009 to 6V          |
| Solar Panel    | Charges 18650            | BMS Module            |

> Fallback logic: if motor battery drops, the rover switches to main power.

---

## Operating Modes

### Manual Mode

- Direction and speed controlled by buttons
- Servo angle adjusted manually
- LED toggle + speed tuning supported

### Autonomous Mode

- Triggered by combo (LEFT + RIGHT)
- Activates continuous scanning and obstacle logic:
  ```cpp
  if (distance < threshold) {
    scanServo();
    decideNewDirection();
  }
  ```

---

## Controller Button Mapping

| Button | Function                      |
|--------|-------------------------------|
| UP     | Rotate Servo Left             |
| DOWN   | Rotate Servo Right            |
| FORWARD| Move Forward                  |
| BACK   | Move Backward                 |
| LEFT   | Turn Left                     |
| RIGHT  | Turn Right                    |
| CENTER | Center Servo to 90°           |
| LIGHT  | Toggle RGB LED Ring           |
| SPEED+ | Increase Motor Speed (PWM)    |
| SPEED– | Decrease Motor Speed (PWM)    |
| LEFT+RIGHT | Hold 3s to toggle Autonomous Mode |

---

## Troubleshooting

- **No Movement** → Check battery voltage and motor wiring.
- **No Response** → Ensure ESP-NOW is initialized with matching MACs.
- **Sensor Issues** → Recalibrate and check alignment/obstruction.

---

## User Customization Notes

To personalize and adapt the Smart Rover system to your setup, you’ll need to manually edit a few parameters in the code:

### 1. Wi-Fi Access Point (AP) Configuration

The Rover can host its own Wi-Fi Access Point, allowing you to connect directly via a web browser or a custom-built iOS/Android app.  
> 📱 A small iOS controller sketch is included in the repository as a starting point.

Update the following lines in the rover code to customize your AP name and password:

```cpp
// Wi-Fi AP settings
const char* ssid = "SmartRover";   // <-- Customize the AP name
const char* pass = "12345678";     // <-- Set your desired password
```

### 2. ESP-NOW MAC Address Setup

#### On the Rover:
Set the MAC address of the controller ESP32 so the rover can recognize and connect to it:

```cpp
// MAC Address of Controller (must match controller's MAC)
uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90}; // <-- Replace with your controller's MAC
```

#### On the Controller:
Set the MAC address of the rover ESP32:

```cpp
// MAC Address of the Rover (must match rover's MAC)
uint8_t roverMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x1D, 0xB4}; // <-- Replace with your rover's MAC
```

> 🛠️ If you don’t know your ESP32’s MAC address, a helper sketch is included in the repository.  
> Flash it to your ESP32, connect it to Wi-Fi, and read the MAC address via the serial monitor before uploading the main rover or controller code.


___

## Control Instructions

- Power ON both Rover and Controller
- Rover enters standby awaiting ESP-NOW input
- Use 5-way input to move, scan, and toggle features
- OLED will show real-time telemetry:
  - Distance in cm
  - Current mode
  - Speed (PWM level)
  - Servo angle

---

> 📘 diagrams, and wiring visuals will be providedd later in this repo.
> Developed and tested using Arduino IDE with ESP32 board definitions installed.

---

© 2025 – Built by Moutaz Baaj. Powered by open hardware and creative engineering.
