/*
===== SMART ROVER MAIN =====
Features:
- ESP-NOW wireless communication with controller
- Web server interface for control and monitoring
- Autonomous obstacle avoidance with ultrasonic sensor
- Manual control modes with speed adjustment
- LED lighting control
- Servo-mounted environmental scanning
- Motor control with PWM
- Status reporting to controller

Hardware Components:
- ESP32 microcontroller
- L298N Motor Driver
- HC-SR04 Ultrasonic Sensor
- SG90 Servo Motor
- WS2812B LED Strip
- KY-032 Obstacle Sensor
*/

// Include necessary libraries
#include <esp_now.h>      // For ESP-NOW wireless communication
#include <WiFi.h>         // For WiFi and AP functionality
#include <ESP32Servo.h>   // For servo motor control
#include <EEPROM.h>       // For potential future EEPROM usage
#include <FastLED.h>      // For LED strip control
#include <WebServer.h>    // For web server functionality

// ========== HARDWARE CONFIGURATION ==========
// LED Configuration
#define LED_PIN 12       // GPIO pin for LED strip
#define LED_COUNT 8      // Number of LEDs in strip

// Motor Control Pins (L298N driver connections)
#define AIN1 18    // Rear Motor Forward control
#define AIN2 19    // Rear Motor Backward control
#define BIN1 16    // Front Motor Left control
#define BIN2 17    // Front Motor Right control
#define STBY 14    // Motor Driver Standby pin

// PWM Configuration for motor speed control
#define PWMA 21    // Rear Motor PWM pin
#define PWMB 22    // Front Motor PWM pin
#define PWM_CHANNEL_A 21  // PWM Channel 2 for Rear Motor
#define PWM_CHANNEL_B 22  // PWM Channel 0 for Front Motor
#define PWM_FREQ 5000     // PWM frequency in Hz
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

// Sensor Configuration
#define TRIG_PIN 23       // Ultrasonic Trigger pin
#define ECHO_PIN 25       // Ultrasonic Echo pin
#define SERVO_PIN 26      // Servo Control pin
#define OBSTACLE_PIN 5    // KY-032 Obstacle Sensor pin

// MAC Address of Controller (must match controller's MAC)
uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90};

// ========== DATA STRUCTURES ==========
/**
 * @brief Structure to hold rover status information
 * 
 * This structure is used to send status updates to the controller
 * via ESP-NOW communication.
 */
typedef struct RoverStatus {
    char action[20];      // Current action description (max 19 chars + null)
    float distanceCM;     // Measured distance in cm from ultrasonic sensor
    int servoAngle;       // Current servo position (0-180 degrees)
    int motorSpeed;       // Current PWM speed (0-255)
    bool autoMode;        // Autonomous mode status (true/false)
} RoverStatus;

// ========== NAVIGATION SETTINGS ==========
const int OBSTACLE_DISTANCE_CM = 30;  // Stop threshold distance in cm
const int MIN_CLEARANCE = 35;         // Minimum clearance for path in cm
const unsigned long AUTO_DRIVE_INTERVAL = 500; // Autonomous check interval in ms

// ========== SERVO CONFIGURATION ==========
Servo usServo;            // Ultrasonic servo instance
const int SERVO_MIN = 0;  // Minimum servo angle (degrees)
const int SERVO_MAX = 180;// Maximum servo angle (degrees)
const int SERVO_CENTER = 90;  // Center position (degrees)
const int SERVO_SPEED = 2;    // Movement delay between steps (ms)
const int SCAN_STEP = 5;      // Scanning step size (degrees)
int currentServoAngle = SERVO_CENTER; // Current servo position

// ========== Ultrasonic CONFIGURATION ==========
unsigned long lastDistanceCheck = 0;    // Last time distance was checked
const unsigned long DISTANCE_UPDATE_INTERVAL = 500; // Distance check interval (ms)
float currentDistance = 0.0;            // Last measured distance

// Motor speed configuration
int motorSpeed = 255;  // Default speed (0-255, 255 = max)

// Movement states for state machine
enum State { STOPPED, FORWARD, BACKWARD, TURNINGR, TURNINGL, SCANNING };
State currentState = STOPPED;  // Initial state

// Self-driving mode settings
bool selfDrivingMode = false;           // Autonomous mode flag
unsigned long lastAutoDriveCheck = 0;   // Last autonomous check time

// Light control configuration
CRGB leds[LED_COUNT];      // LED strip array
bool isLightsOn = true;    // Current LED state

// Idle status tracking
unsigned long lastActiveTime = 0;          // Last time motors were active
unsigned long lastIdleReportTime = 0;      // Last idle status report
const unsigned long IDLE_TIMEOUT = 20;     // 2 seconds idle timeout (units?)
const unsigned long IDLE_REPORT_INTERVAL = 50; // Send idle update every 5 seconds (units?)

// wifi AP setting:
const char* ssid = "SmartRover";
const char* pass = "12345678";

// Web server instance on port 80
WebServer server(80);

/**
 * @brief Setup function - runs once at startup
 * 
 * Initializes all hardware components, communication protocols,
 * and web server endpoints.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("===== SMART ROVER INITIALIZED =====");

  // Initialize LED strip
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  
  // Initialize motor control pins and PWM
  ledcAttach(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize motor driver standby pin
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize obstacle sensor pin
  pinMode(OBSTACLE_PIN, INPUT);

  // Initialize servo motor
  ESP32PWM::allocateTimer(0);
  usServo.setPeriodHertz(50);
  usServo.attach(SERVO_PIN, 500, 2400);
  usServo.write(SERVO_CENTER);
  delay(500);

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_AP_STA);  // Support both AP and STA (ESP-NOW)
  WiFi.softAP(ssid, pass); // Create access point
  Serial.println("Access Point Started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP()); 
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed, restarting...");
    ESP.restart();
  }

  // Register callback for received data
  esp_now_register_recv_cb(onDataReceived);
  
  // Configure peer (controller) information
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, controllerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  // Add peer to ESP-NOW
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }

  // Ensure motors are stopped at startup
  stopAllMotors();

  // === HTTP Server Handlers ===

  // Root endpoint
  server.on("/", []() {
    server.send(200, "text/plain", "Smart Rover Ready");
  });

  // Toggle autonomous mode
  server.on("/toggle_auto", []() {
    selfDrivingMode = !selfDrivingMode;
    stopAllMotors();
    sendRoverStatus(selfDrivingMode ? "AUTO ENABLED" : "Idle", currentDistance, SERVO_CENTER, 0);
    server.send(200, "text/plain", selfDrivingMode ? "Self-Driving ON" : "Self-Driving OFF");
  });

  // Movement commands (manual mode only)
  server.on("/forward", []() {
    if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
    moveForward(motorSpeed);
    currentState = FORWARD;
    sendRoverStatus("Manual FORWARD", currentDistance, currentServoAngle, motorSpeed);
    server.send(200, "text/plain", "Moving Forward");
  });

  server.on("/backward", []() {
    if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
    moveBackward(motorSpeed);
    currentState = BACKWARD;
    sendRoverStatus("Manual BACKWARD", currentDistance, currentServoAngle, motorSpeed);
    server.send(200, "text/plain", "Moving Backward");
  });

  server.on("/left", []() {
    if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
    turnLeft(255);
    currentState = TURNINGL;
    sendRoverStatus("Manual TURNING L", currentDistance, currentServoAngle, motorSpeed);
    server.send(200, "text/plain", "Turning Left");
  });

  server.on("/right", []() {
    if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
    turnRight(255);
    currentState = TURNINGR;
    sendRoverStatus("Manual TURNING R", currentDistance, currentServoAngle, motorSpeed);
    server.send(200, "text/plain", "Turning Right");
  });

  // Forward + Right combination
  server.on("/forward_right", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveForward(motorSpeed);
   turnRight(255);
   currentState = FORWARD;
   sendRoverStatus("Manual FORWARD R", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Forward Right");
  });

  // Forward + Left combination
  server.on("/forward_left", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveForward(motorSpeed);
   turnLeft(255);
   currentState = FORWARD;
   sendRoverStatus("Manual FORWARD L", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Forward Left");
  });

  // Backward + Right combination
  server.on("/backward_right", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveBackward(motorSpeed);
   turnRight(255);
   currentState = BACKWARD;
   sendRoverStatus("Manual BACKWARD R", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Backward Right");
  });

  // Backward + Left combination
  server.on("/backward_left", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveBackward(motorSpeed);
   turnLeft(255);
   currentState = BACKWARD;
   sendRoverStatus("Manual BACKWARD L", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Backward Left");
  });

  // Stop all motors
  server.on("/stop", []() {
    stopAllMotors();
    currentState = STOPPED;
    sendRoverStatus("Idle", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Stopped");
  });

  // Increase motor speed
  server.on("/speedup", []() {
    motorSpeed = min(motorSpeed + 5, 255);
    server.send(200, "text/plain", "Speed Increased");
  });

  // Decrease motor speed
  server.on("/speeddown", []() {
    motorSpeed = max(motorSpeed - 5, 200);
    server.send(200, "text/plain", "Speed Decreased");
  });

  // Center servo position
  server.on("/center_servo", []() {
    currentServoAngle = SERVO_CENTER;
    usServo.write(currentServoAngle);
    sendRoverStatus("Center Servo", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Centered");
  });

  // Move servo left
  server.on("/servo_left", []() {
    currentServoAngle = constrain(currentServoAngle + 2, SERVO_MIN, SERVO_MAX);
    usServo.write(currentServoAngle);
    sendRoverStatus("Scanning Left", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Left");
  });

  // Move servo right
  server.on("/servo_right", []() {
    currentServoAngle = constrain(currentServoAngle - 2, SERVO_MIN, SERVO_MAX);
    usServo.write(currentServoAngle);
    sendRoverStatus("Scanning Right", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Right");
  });

  // Toggle LED strip
  server.on("/toggle_led", []() {
    isLightsOn = !isLightsOn;
    digitalWrite(LED_PIN, isLightsOn ? HIGH : LOW);
    server.send(200, "text/plain", isLightsOn ? "LED ON" : "LED OFF");
  });

  // Start the web server
  server.begin();
  Serial.println("HTTP server started");
}

/**
 * @brief Controls the LED strip
 * 
 * @param ledStatus True to turn on LEDs, false to turn off
 */
void ledControl(bool ledStatus) {
  if (ledStatus) {
    fill_solid(leds, LED_COUNT, CRGB::White);  // Set all LEDs to white
    FastLED.show();  // Update LEDs
  } else {
    FastLED.clear();  // Turn all LEDs off
    FastLED.show();   // Update LEDs
  }
}

// ========== MOTOR CONTROL FUNCTIONS ==========

/**
 * @brief Move rover forward at specified speed
 * 
 * @param speed PWM value (0-255)
 */
void moveForward(int speed) {
    lastActiveTime = millis();  // Update last active time
    ledcWrite(PWM_CHANNEL_A, speed);  // Set rear motor speed
    digitalWrite(AIN1, LOW);    // Set direction forward
    digitalWrite(AIN2, HIGH);
    Serial.println("MOVING FORWARD");
}

/**
 * @brief Move rover backward at specified speed
 * 
 * @param speed PWM value (0-255)
 */
void moveBackward(int speed){
    lastActiveTime = millis();
    ledcWrite(PWM_CHANNEL_A, speed);
    digitalWrite(AIN1, HIGH);   // Set direction backward
    digitalWrite(AIN2, LOW);
    Serial.println("MOVING BACKWARD");
}

/**
 * @brief Turn rover right at specified speed
 * 
 * @param speed PWM value (0-255)
 */
void turnRight(int speed) {
    lastActiveTime = millis();
    ledcWrite(PWM_CHANNEL_B, speed);  // Set front motor speed
    digitalWrite(BIN1, LOW);    // Set direction right
    digitalWrite(BIN2, HIGH);
    Serial.println("TURNING RIGHT");
}

/**
 * @brief Turn rover left at specified speed
 * 
 * @param speed PWM value (0-255)
 */
void turnLeft(int speed) {
    lastActiveTime = millis();
    ledcWrite(PWM_CHANNEL_B, speed);
    digitalWrite(BIN1, HIGH);   // Set direction left
    digitalWrite(BIN2, LOW);
    Serial.println("TURNING LEFT");
}

/**
 * @brief Stop all motors
 */
void stopAllMotors() {
    // Set all PWM outputs to 0
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    
    // Set all direction pins low
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    
    Serial.println("STOPPED");
}

// ========== SENSOR FUNCTIONS ==========

/**
 * @brief Measure distance using ultrasonic sensor
 * 
 * @return float Distance in centimeters
 */
float getDistanceCM() {
    // Trigger ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure echo pulse duration
    long duration = pulseIn(ECHO_PIN, HIGH);
    // Convert duration to distance (cm)
    return duration * 0.034 / 2;
}

/**
 * @brief Adjust motor speed based on measured distance
 * 
 * @param distance Measured distance in cm
 * @return int Adjusted motor speed (200-255)
 */
int adjustSpeedBasedOnDistance(float distance) {
    int minSpeed = 200;  // Minimum motor speed (3.3V)
    int maxSpeed = 255;  // Maximum motor speed

    // Clamp distance to a safe range
    if (distance < 50) distance = 50;
    if (distance > 400) distance = 400;

    // Map distance to motor speed
    int motorSpeed = map(distance, 50, 400, minSpeed, maxSpeed);

    Serial.print("Adjusted Speed: ");
    Serial.println(motorSpeed);

    return motorSpeed;
}

/**
 * @brief Smoothly move servo to target angle
 * 
 * @param targetAngle Desired angle (0-180 degrees)
 */
void smoothServoMove(int targetAngle) {
    // Constrain angle to valid range
    targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
    
    // Move servo incrementally to target
    while (currentServoAngle != targetAngle) {
        currentServoAngle += (targetAngle > currentServoAngle) ? 1 : -1;
        usServo.write(currentServoAngle);
        delay(SERVO_SPEED);
    }
}

// ========== AUTONOMOUS NAVIGATION FUNCTIONS ==========

/**
 * @brief Perform 180° environment scan and choose best path
 */
void scanEnvironment() {
    Serial.println("Starting 180° environment scan");

    int bestAngle = SERVO_CENTER;   // Default to center
    float maxDistance = 0;          // Track farthest distance
    
    // Left to right scan
    for (int angle = SERVO_MIN; angle <= SERVO_MAX; angle += SCAN_STEP) {
        smoothServoMove(angle);             // Move servo to angle
        float distance = getDistanceCM();   // Measure distance
        
        // Print scan results
        Serial.print("Angle:");
        Serial.print(angle);
        Serial.print("° Distance:");
        Serial.print(distance);
        Serial.println("cm");
        
        // Send status update to controller
        sendRoverStatus("Scanning...", distance, angle, 0);

        // Track best path (farthest with minimum clearance)
        if (distance > maxDistance && distance > MIN_CLEARANCE) {
            maxDistance = distance;
            bestAngle = angle;
        }
        delay(50);
    }
    
    // Return servo to center
    smoothServoMove(SERVO_CENTER);
    
    // Print scan results
    Serial.print("Best path: ");
    Serial.print(bestAngle);
    Serial.print("° (");
    Serial.print(maxDistance);
    Serial.println("cm clearance)");
    
    // Send decision to controller
    sendRoverStatus("Scanning Done", maxDistance, bestAngle, 0);

    // Decision making based on best angle
    if (bestAngle < 60) {
        Serial.println(">>> Hard left turn");
        sendRoverStatus("Hard left turn", maxDistance, bestAngle, 205);
        moveBackward(205);
        delay(800);
        turnLeft(255);
        delay(1200);
    } 
    else if (bestAngle > 120) {
        Serial.println(">>> Hard right turn");
        sendRoverStatus("Hard right turn", maxDistance, bestAngle, 205);
        moveBackward(205);
        delay(800);
        turnRight(255);
        delay(1200);
    }
    else if (bestAngle < 90) {
        Serial.println(">>> Slight left turn");
        sendRoverStatus("Slight left turn", maxDistance, bestAngle, 205);
        turnLeft(255);
        delay(800);
        moveForward(205);
        delay(1200);
    }
    else if (bestAngle > 90) {
        Serial.println(">>> Slight right turn");
        sendRoverStatus("Slight right turn", maxDistance, bestAngle, 205);
        turnRight(255);
        delay(800);
        moveForward(205);
        delay(1200);
    }
    else {
        Serial.println(">>> Path clear ahead");
        int speed = adjustSpeedBasedOnDistance(maxDistance);
        sendRoverStatus("Path clear ahead", maxDistance, bestAngle, speed);
        moveForward(speed);
    }
}

/**
 * @brief Autonomous driving logic
 */
void autonomousDrive() {
    // Center servo for forward measurement
    usServo.write(SERVO_CENTER);
    Serial.print("centering servo to 90 degree");

    // Get sensor readings
    float distance = getDistanceCM();
    bool irObstacle = digitalRead(OBSTACLE_PIN) == LOW; // LOW means obstacle detected

    Serial.print("Forward distance: ");
    Serial.print(distance);
    Serial.print(" cm | IR Sensor: ");
    Serial.println(irObstacle ? "Obstacle Detected" : "Clear");

    // Obstacle avoidance logic
    if (distance < OBSTACLE_DISTANCE_CM || irObstacle) {
        Serial.println("! OBSTACLE DETECTED !");
        sendRoverStatus("Obstacle Detected", distance, SERVO_CENTER, 0);
        moveBackward(220);  // Back up
        delay(400);
        stopAllMotors();
        currentState = SCANNING;
        scanEnvironment();  // Find new path
        stopAllMotors();
    } else {
        // Adjust speed based on distance and move forward
        int speed = adjustSpeedBasedOnDistance(distance);
        sendRoverStatus("Moving Forward", distance, SERVO_CENTER, speed);
        moveForward(speed);
    }
}

// ========== COMMUNICATION FUNCTIONS ==========

/**
 * @brief Callback for received ESP-NOW data
 * 
 * @param sender MAC address of sender
 * @param data Received data
 * @param len Length of data
 */
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
    // Verify sender MAC and data length
    if (memcmp(sender->src_addr, controllerMac, 6) == 0 && len == 1) {
        Serial.print("Received command: ");
        Serial.println(data[0]);

        // Check for autonomous mode toggle (command 9)
        if (data[0] == 9) {
            selfDrivingMode = !selfDrivingMode;
            sendRoverStatus(selfDrivingMode ? "AUTO ENABLED" : "Idel", currentDistance, SERVO_CENTER, 0);
            Serial.print("Self-Driving Mode: ");
            Serial.println(selfDrivingMode ? "ON" : "OFF");
            stopAllMotors();
            return;
        }

        // Ignore other commands if in self-driving mode
        if (selfDrivingMode) {
            Serial.println("(Ignoring - in self-driving mode)");
            return;
        }

        // Process manual control commands
        switch (data[0]) {
            case 1:  // Forward
                moveForward(motorSpeed);
                currentState = FORWARD;
                sendRoverStatus("Manual FORWARD", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 2:  // Backward
                moveBackward(motorSpeed);
                currentState = BACKWARD;
                sendRoverStatus("Manual BACKWARD", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 3:  // Right turn
                turnRight(255);
                currentState = TURNINGR;
                sendRoverStatus("Manual TURNING R", currentDistance, currentServoAngle, motorSpeed);
                break;  
            case 4:  // Left turn
                turnLeft(255);
                currentState = TURNINGL;
                sendRoverStatus("Manual TURNING L", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 5:  // Forward-right
                moveForward(motorSpeed);
                turnRight(255);
                currentState = FORWARD;
                sendRoverStatus("Manual FORWARD R", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 6:  // Forward-left
                moveForward(motorSpeed);
                turnLeft(255);
                currentState = FORWARD;
                sendRoverStatus("Manual FORWARD L", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 7:  // Backward-right
                moveBackward(motorSpeed);
                turnRight(255);
                currentState = BACKWARD;
                sendRoverStatus("Manual BACKWARD R", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 8:  // Backward-left
                moveBackward(motorSpeed);
                turnLeft(255);
                currentState = BACKWARD;
                sendRoverStatus("Manual BACKWARD L", currentDistance, currentServoAngle, motorSpeed);
                break;
            case 10:  // Speed up
                motorSpeed = min(motorSpeed + 5, 255);
                Serial.print("Speed Increased: ");
                Serial.println(motorSpeed);
                break;
            case 11:  // Speed down
                motorSpeed = max(motorSpeed - 5, 200);
                Serial.print("Speed Decreased: ");
                Serial.println(motorSpeed);
                break;
            case 12:  // Center servo
                currentServoAngle = SERVO_CENTER;
                usServo.write(currentServoAngle);
                sendRoverStatus("Center Servo", currentDistance, currentServoAngle, 0);
                break; 
            case 13:  // Servo right
                currentServoAngle -= 2;
                currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);
                usServo.write(currentServoAngle);
                sendRoverStatus("Scaning Right", currentDistance, currentServoAngle, 0);
                break;
            case 14:  // Servo left
                currentServoAngle += 2;
                currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);
                usServo.write(currentServoAngle);
                sendRoverStatus("Scaning Left", currentDistance, currentServoAngle, 0);
                break;
            case 15:  // Toggle LEDs
                isLightsOn = !isLightsOn;
                Serial.print("LED: ");
                Serial.println(isLightsOn ? "ON" : "OFF");
                break;
            case 0:  // Stop
                stopAllMotors();
                currentState = STOPPED;
                sendRoverStatus("Idle", currentDistance, currentServoAngle, 0);
                break;
        }
    }
}

/**
 * @brief Send rover status to controller via ESP-NOW
 * 
 * @param action Current action description
 * @param distance Measured distance in cm
 * @param angle Current servo angle
 * @param motorSpeed Current motor speed
 */
void sendRoverStatus(const char* action, float distance, int angle, int motorSpeed) {
    RoverStatus status;
    strncpy(status.action, action, sizeof(status.action));
    status.distanceCM = distance;
    status.servoAngle = angle;
    status.motorSpeed = motorSpeed;
    status.autoMode = selfDrivingMode; 
    
    // Send status via ESP-NOW
    esp_err_t result = esp_now_send(controllerMac, (uint8_t*)&status, sizeof(status));
    if (result != ESP_OK) {
        Serial.println("Failed to send status");
    } else {
        Serial.print("Sent status: ");
        Serial.println(action);
    }
}

/**
 * @brief Main loop - runs continuously
 */
void loop() {
    // Handle web server requests
    server.handleClient();

    // Update LED status
    ledControl(isLightsOn);
    
    unsigned long now = millis();

    // Autonomous driving logic
    if (selfDrivingMode && now - lastAutoDriveCheck >= AUTO_DRIVE_INTERVAL) {
        autonomousDrive();
        lastAutoDriveCheck = now;
    }

    // Update distance in manual mode
    if (!selfDrivingMode && now - lastDistanceCheck >= DISTANCE_UPDATE_INTERVAL) {
        lastDistanceCheck = now;
        currentDistance = getDistanceCM();
        Serial.print("Distance (manual mode): ");
        Serial.println(currentDistance);
            
        if (currentState == STOPPED) {
            sendRoverStatus("Idle", currentDistance, currentServoAngle, 0);
        }
    }

    // Small delay to prevent watchdog timer issues
    delay(50);
}