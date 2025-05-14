/*
===== SMART ROVER MAIN CONTROLLER =====
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

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <WebServer.h>


// ========== HARDWARE CONFIGURATION ==========
// LED Configuration
#define LED_PIN 12       // GPIO pin for LED strip
#define LED_COUNT 8      // Number of LEDs in strip

// Motor Control Pins
#define AIN1 18    // Rear Motor Forward
#define AIN2 19    // Rear Motor Backward
#define BIN1 16    // Front Motor Left
#define BIN2 17    // Front Motor Right
#define STBY 14    // Motor Driver Standby

// PWM Configuration
#define PWMA 21    // Rear Motor PWM
#define PWMB 22    // Front Motor PWM
#define PWM_CHANNEL_A 21  // PWM Channel 2 for Rear Motor
#define PWM_CHANNEL_B 22  // PWM Channel 0 for Front Motor
#define PWM_FREQ 5000     // PWM frequency in Hz
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

// Sensor Configuration
#define TRIG_PIN 23       // Ultrasonic Trigger
#define ECHO_PIN 25       // Ultrasonic Echo
#define SERVO_PIN 26      // Servo Control
#define OBSTACLE_PIN 5    // KY-032 Obstacle Sensor

// MAC Address of Controller
uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90};

// ========== DATA STRUCTURES ==========
typedef struct RoverStatus {
    char action[20];      // Current action description
    float distanceCM;     // Measured distance in cm
    int servoAngle;       // Current servo position
    int motorSpeed;       // Current PWM speed (0-255)
    bool autoMode;        // Autonomous mode status
} RoverStatus;

// ========== NAVIGATION SETTINGS ==========
const int OBSTACLE_DISTANCE_CM = 30;  // Stop threshold
const int MIN_CLEARANCE = 35;         // Minimum clearance for path in cm
const unsigned long AUTO_DRIVE_INTERVAL = 500; // Autonomous check interval

// ========== SERVO CONFIGURATION ==========
Servo usServo;            // Ultrasonic servo instance
const int SERVO_MIN = 0;  // Minimum servo angle
const int SERVO_MAX = 180;// Maximum servo angle
const int SERVO_CENTER = 90;  // Center position
const int SERVO_SPEED = 2;    // Movement delay (ms)
const int SCAN_STEP = 5;      // Scanning step size (degrees)
int currentServoAngle = SERVO_CENTER; // Current position

// ========== Ultrasonic CONFIGURATION ==========
unsigned long lastDistanceCheck = 0;
const unsigned long DISTANCE_UPDATE_INTERVAL = 500; // check distance every 0.5s
float currentDistance = 0.0;


// Mototr speed
int motorSpeed = 255;  // Default speed (0-255)

// Movement states
enum State { STOPPED, FORWARD, BACKWARD, TURNINGR, TURNINGL, SCANNING };
State currentState = STOPPED;

// Selfe driving Setting
bool selfDrivingMode = false;
unsigned long lastAutoDriveCheck = 0;

// Light controll 
CRGB leds[LED_COUNT];
bool isLightsOn = true;

// Idel status
unsigned long lastActiveTime = 0;
unsigned long lastIdleReportTime = 0;
const unsigned long IDLE_TIMEOUT = 20;        // 2 seconds idle timeout
const unsigned long IDLE_REPORT_INTERVAL = 50; // Send idle update every 5 seconds


// Init web server 
WebServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.println("===== SMART ROVER INITIALIZED =====");


  
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  
  // Initialize motors
  ledcAttach(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // ---------------------

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Intilize KY-032
  pinMode(OBSTACLE_PIN, INPUT);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  usServo.setPeriodHertz(50);
  usServo.attach(SERVO_PIN, 500, 2400);
  usServo.write(SERVO_CENTER);
  delay(500);


  // Initialize ESP-NOW and the AP
  WiFi.mode(WIFI_AP_STA);  // Support both AP and STA (ESP-NOW)

  WiFi.softAP("SmartRover", "12345678"); // SSID and password
  Serial.println("Access Point Started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP()); 
  
   if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed, restarting...");
    ESP.restart();
    //return;
  }

  esp_now_register_recv_cb(onDataReceived);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, controllerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;


  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }

  stopAllMotors();

  // === HTTP Handlers ===

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

  // Forward + Right
  server.on("/forward_right", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveForward(motorSpeed);
   turnRight(255);
   currentState = FORWARD;
   sendRoverStatus("Manual FORWARD R", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Forward Right");
  });

  // Forward + Left
  server.on("/forward_left", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveForward(motorSpeed);
   turnLeft(255);
   currentState = FORWARD;
   sendRoverStatus("Manual FORWARD L", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Forward Left");
  });

  // Backward + Right
  server.on("/backward_right", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveBackward(motorSpeed);
   turnRight(255);
   currentState = BACKWARD;
   sendRoverStatus("Manual BACKWARD R", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Backward Right");
  });

  // Backward + Left
  server.on("/backward_left", []() {
   if (selfDrivingMode) return server.send(403, "text/plain", "In Auto Mode");
   moveBackward(motorSpeed);
   turnLeft(255);
   currentState = BACKWARD;
   sendRoverStatus("Manual BACKWARD L", currentDistance, currentServoAngle, motorSpeed);
   server.send(200, "text/plain", "Moving Backward Left");
  });


  server.on("/stop", []() {
    stopAllMotors();
    currentState = STOPPED;
    sendRoverStatus("Idle", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Stopped");
  });

  server.on("/speedup", []() {
    motorSpeed = min(motorSpeed + 5, 255);
    server.send(200, "text/plain", "Speed Increased");
  });

  server.on("/speeddown", []() {
    motorSpeed = max(motorSpeed - 5, 200);
    server.send(200, "text/plain", "Speed Decreased");
  });

  server.on("/center_servo", []() {
    currentServoAngle = SERVO_CENTER;
    usServo.write(currentServoAngle);
    sendRoverStatus("Center Servo", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Centered");
  });

  server.on("/servo_left", []() {
    currentServoAngle = constrain(currentServoAngle + 2, SERVO_MIN, SERVO_MAX);
    usServo.write(currentServoAngle);
    sendRoverStatus("Scanning Left", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Left");
  });

  server.on("/servo_right", []() {
    currentServoAngle = constrain(currentServoAngle - 2, SERVO_MIN, SERVO_MAX);
    usServo.write(currentServoAngle);
    sendRoverStatus("Scanning Right", currentDistance, currentServoAngle, 0);
    server.send(200, "text/plain", "Servo Right");
  });

  server.on("/toggle_led", []() {
    isLightsOn = !isLightsOn;
    digitalWrite(LED_PIN, isLightsOn ? HIGH : LOW);
    server.send(200, "text/plain", isLightsOn ? "LED ON" : "LED OFF");
  });

  // Start the server
  server.begin();
  Serial.println("HTTP server started");

}

// LED CONTROLL 
void ledControl(bool ledStatus) {
  if (ledStatus) {
    fill_solid(leds, LED_COUNT, CRGB::White);
    FastLED.show();
  } else {
    FastLED.clear(); 
    FastLED.show();  
  }
}

// Motor control functions
void moveForward(int speed) {
    lastActiveTime = millis();
  ledcWrite(PWM_CHANNEL_A, speed);  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
 // currentState = FORWARD;
 // ledControl();
  Serial.println("MOVING FORWARD");

}

void moveBackward(int speed){
    lastActiveTime = millis();
  ledcWrite(PWM_CHANNEL_A, speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
 // currentState = BACKWARD; 
 // ledControl();
  Serial.println("MOVING BACKWARD");
}

void turnRight(int speed) {
    lastActiveTime = millis();
  ledcWrite(PWM_CHANNEL_B, speed);  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  //currentState = TURNINGL;
  // ledControl();
  Serial.println("TURNING RIGHT");
}

void turnLeft(int speed) {
    lastActiveTime = millis();
  ledcWrite(PWM_CHANNEL_B, speed); 
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  //ledControl();
  Serial.println("TURNING LEFT");
}

void stopAllMotors() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
 // currentState = STOPPED;
 // ledControl();
  Serial.println("STOPPED");
}

// Ultrasonic distance measurement
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

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

// Smooth servo movement
void smoothServoMove(int targetAngle) {
  targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
  
  while (currentServoAngle != targetAngle) {
    currentServoAngle += (targetAngle > currentServoAngle) ? 1 : -1;
    usServo.write(currentServoAngle);
    delay(SERVO_SPEED);
  }
}

// Full environment scan
void scanEnvironment() {
  Serial.println("Starting 180° environment scan");

  int bestAngle = SERVO_CENTER;   
  float maxDistance = 0;
  
  // Left to right scan
  for (int angle = SERVO_MIN; angle <= SERVO_MAX; angle += SCAN_STEP) {
    smoothServoMove(angle);
    float distance = getDistanceCM();
    
    Serial.print("Angle:");
    Serial.print(angle);
    Serial.print("° Distance:");
    Serial.print(distance);
    Serial.println("cm");
    
    sendRoverStatus("Scanning...", distance, angle, 0);

    if (distance > maxDistance && distance > MIN_CLEARANCE) {
      maxDistance = distance;
      bestAngle = angle;
    }
    delay(50);
  }
  
  smoothServoMove(SERVO_CENTER);
  
  Serial.print("Best path: ");
  Serial.print(bestAngle);
  Serial.print("° (");
  Serial.print(maxDistance);
  Serial.println("cm clearance)");
  

  // Send decision update to controller
  sendRoverStatus("Scanning Done", maxDistance, bestAngle, 0);

  // Decision making
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

// Autonomous navigation
void autonomousDrive() {
  usServo.write(SERVO_CENTER);
  Serial.print("centering servo to 90 dgree");

  float distance = getDistanceCM();
  bool irObstacle = digitalRead(OBSTACLE_PIN) == LOW; // LOW means obstacle detected

  Serial.print("Forward distance: ");
  Serial.print(distance);
  Serial.print(" cm | IR Sensor: ");
  Serial.println(irObstacle ? "Obstacle Detected" : "Clear");

  if (distance < OBSTACLE_DISTANCE_CM || irObstacle) {
    Serial.println("! OBSTACLE DETECTED !");
    sendRoverStatus("Obstacle Detected", distance, SERVO_CENTER, 0);
    moveBackward(220);
    delay(400);
    stopAllMotors();
    currentState = SCANNING;
    //ledControl();
    scanEnvironment();
    stopAllMotors();
  } else {
    int speed = adjustSpeedBasedOnDistance(distance);
    sendRoverStatus("Moving Forward", distance, SERVO_CENTER, speed);
    moveForward(speed);
  }
}

// Handle incoming commands
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
  if (memcmp(sender->src_addr, controllerMac, 6) == 0 && len == 1) {
    Serial.print("Received command: ");
    Serial.println(data[0]);


     if (data[0] == 9) {  // Toggle autonomous mode
      selfDrivingMode = !selfDrivingMode;
      sendRoverStatus(selfDrivingMode ? "AUTO ENABLED" : "Idel", currentDistance, SERVO_CENTER, 0);
      Serial.print("Self-Driving Mode: ");
      Serial.println(selfDrivingMode ? "ON" : "OFF");
      stopAllMotors();
      return;
    }

    if (selfDrivingMode) {
      Serial.println("(Ignoring - in self-driving mode)");
      return;
    }

    //stopAllMotors();

        switch (data[0]) {
            case 1: moveForward(motorSpeed); currentState = FORWARD; sendRoverStatus("Manual FORWARD", currentDistance, currentServoAngle, motorSpeed);  break;
            case 2: moveBackward(motorSpeed); currentState = BACKWARD; sendRoverStatus("Manual BACKWARD", currentDistance, currentServoAngle, motorSpeed); break;
            case 3: turnRight(255); currentState = TURNINGR; sendRoverStatus("Manual TURNING R", currentDistance, currentServoAngle, motorSpeed); break;  
            case 4: turnLeft(255); currentState = TURNINGL; sendRoverStatus("Manual TURNING L", currentDistance, currentServoAngle, motorSpeed); break;
            case 5: moveForward(motorSpeed); turnRight(255); currentState = FORWARD; sendRoverStatus("Manual FORWARD R", currentDistance, currentServoAngle, motorSpeed); break;
            case 6: moveForward(motorSpeed); turnLeft(255); currentState = FORWARD; sendRoverStatus("Manual FORWARD L", currentDistance, currentServoAngle, motorSpeed); break;
            case 7: moveBackward(motorSpeed); turnRight(255); currentState = BACKWARD; sendRoverStatus("Manual BACKWARD R", currentDistance, currentServoAngle, motorSpeed); break;
            case 8: moveBackward(motorSpeed); turnLeft(255); currentState = BACKWARD; sendRoverStatus("Manual BACKWARD L", currentDistance, currentServoAngle, motorSpeed); break;
            case 10: motorSpeed = min(motorSpeed + 5, 255); Serial.print("Speed Increased: "); Serial.println(motorSpeed); break;
            case 11: motorSpeed = max(motorSpeed - 5, 200);  Serial.print("Speed Decreased: "); Serial.println(motorSpeed); break;
            case 12: currentServoAngle = SERVO_CENTER; usServo.write(currentServoAngle); sendRoverStatus("Center Servo", currentDistance, currentServoAngle, 0); break; 
            case 13: currentServoAngle -= 2; currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);usServo.write(currentServoAngle);
            sendRoverStatus("Scaning Right", currentDistance, currentServoAngle, 0); break;
            case 14: currentServoAngle += 2; currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);usServo.write(currentServoAngle);
            sendRoverStatus("Scaning Left", currentDistance, currentServoAngle, 0); break;
            case 15: isLightsOn = !isLightsOn; Serial.print("LED: ");Serial.println(isLightsOn ? "ON" : "OFF"); break;
            case 0: stopAllMotors(); currentState = STOPPED; sendRoverStatus("Idle", currentDistance, currentServoAngle, 0); break;
        }

  }
}

// Function to send data back to the controller
void sendRoverStatus(const char* action, float distance, int angle, int motorSpeed) {
    RoverStatus status;
    strncpy(status.action, action, sizeof(status.action));
    status.distanceCM = distance;
    status.servoAngle = angle;
    status.motorSpeed = motorSpeed;
    status.autoMode = selfDrivingMode; 
    
    esp_err_t result = esp_now_send(controllerMac, (uint8_t*)&status, sizeof(status));
    if (result != ESP_OK) {
        Serial.println("Failed to send status");
    } else {
        Serial.print("Sent status: ");
        Serial.println(action);
    }
}

void loop() {

  server.handleClient();

  ledControl(isLightsOn);
  
  unsigned long now = millis();

  // Autonomous driving logic
  if (selfDrivingMode && now - lastAutoDriveCheck >= AUTO_DRIVE_INTERVAL) {
    autonomousDrive();
    lastAutoDriveCheck = now;
  }


  // Always update distance if NOT in self-driving mode
  if (!selfDrivingMode && now - lastDistanceCheck >= DISTANCE_UPDATE_INTERVAL) {
    lastDistanceCheck = now;
    currentDistance = getDistanceCM();  // Store to a global variable
    Serial.print("Distance (manual mode): ");
    Serial.println(currentDistance);
        
      if (currentState == STOPPED) {
      sendRoverStatus("Idle", currentDistance, currentServoAngle, 0);
    }
    
  }



  delay(50);
}