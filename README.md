# ESP32 Smart Rover Project

This repository contains the full code, wiring details, and instructions for building an ESP32-powered Smart Rover. It includes both the rover and controller configurations, as well as an overview of components and how they work together.

## Table of Contents

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

## ESP32 Smart Rover – Components List

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

## Wiring Diagrams

Folow the pin in/output in the code

---

## Code Overview

The code consists of two main sections:

1. **Rover Code**: Controls the movement of the rover, sensor readings, and autonomous navigation. It uses ESP-NOW for communication between the rover and the controller.

2. **Controller Code**: Handles button presses, sends commands to the rover, and displays the rover's status on the OLED display. It includes functionality for both manual and autonomous driving modes.

---

## Features and Functionality

* **Manual Control**: Use the buttons on the controller to move the rover forward, backward, left, and right.
* **Self-driving Mode**: Activate by pressing the left and right buttons simultaneously. The rover will autonomously navigate based on sensor input.
* **Obstacle Avoidance**: The rover uses the ultrasonic sensor to detect obstacles and avoid them during movement.
* **OLED Display**: The controller's OLED display shows the current status of the rover, including its action, distance, servo angle, and motor speed.

---

## Operating Modes

1. **Manual Mode**: The user controls the rover using the directional buttons (forward, backward, left, right). The controller sends the corresponding commands to the rover.
2. **Autonomous Mode**: Activated by pressing both the left and right buttons simultaneously for 3 seconds. The rover will navigate on its own based on the ultrasonic sensor's feedback.

---

## Power Setup

* **Rover Power**: The controller is powered by 4x 1.5V AA lithium batteries (regulated to 5V using the LM2596 Buck Converter).

---

## Controller Button Mapping

* **Forward Button**: Move the rover forward.
* **Backward Button**: Move the rover backward.
* **Left Button**: Turn the rover left.
* **Right Button**: Turn the rover right.
* **Servo Button (Up/Down)**: Adjust the servo angle for the ultrasonic sensor.
* **Self-driving Mode**: Activate by pressing both the left and right buttons simultaneously for 3 seconds.

---

## Troubleshooting

1. **Rover Not Moving**: Check the motor connections and ensure the power supply is sufficient.
2. **Controller Not Responding**: Make sure both ESP32 boards are properly paired using ESP-NOW.
3. **Obstacle Avoidance Not Working**: Ensure the ultrasonic sensor is correctly positioned and not obstructed.

---

## How to Control the Rover

When powered on:

* The rover will wait for commands via ESP-NOW from the controller.
* The controller shows the rover’s state on the OLED screen (distance, angle, speed).
* Use the 5-way button to:

  * Press Up → Rotate servo left (increase angle).
  * Press Down → Rotate servo right (decrease angle).
  * Press Forward → Move rover forward.
  * Press Back → Move rover backward.
  * Press Left/Right → Rotate front wheel left/right.
  * Press both Left and Right together for 3 seconds → Toggle self-driving mode.

The OLED updates the current state and displays the latest distance detected by the ultrasonic sensor.

---

