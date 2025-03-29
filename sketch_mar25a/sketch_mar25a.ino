#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Motor pins
#define IN1 18    // Rear Motor Forward
#define IN2 19    // Rear Motor Backward
#define IN3 21    // Front Motor Left
#define IN4 22    // Front Motor Right

// Ultrasonic Sensor pins
#define TRIG_PIN 23
#define ECHO_PIN 25

// Servo pin
#define SERVO_PIN 26

// MAC address
uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90};

// Navigation settings
const int OBSTACLE_DISTANCE_CM = 20;  // Stop if obstacle < 20cm
const int MIN_CLEARANCE = 30;         // Minimum acceptable clearance (cm)
const unsigned long AUTO_DRIVE_INTERVAL = 500; // Check every 500ms

// Servo settings
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;
const int SERVO_SPEED = 15;           // ms between steps (lower = smoother)
const int SCAN_STEP = 5;              // Degrees per step

// Movement states
enum State { STOPPED, FORWARD, BACKWARD, TURNING, SCANNING };
State currentState = STOPPED;

Servo usServo;
bool selfDrivingMode = false;
unsigned long lastAutoDriveCheck = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("===== SMART ROVER INITIALIZED =====");

  // Initialize motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  usServo.setPeriodHertz(50);
  usServo.attach(SERVO_PIN, 500, 2400);
  usServo.write(SERVO_CENTER);
  delay(500);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, controllerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  stopAllMotors();
  randomSeed(analogRead(0));
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  currentState = FORWARD;
  Serial.println("MOVING FORWARD");
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  currentState = BACKWARD;
  Serial.println("MOVING BACKWARD");
}

void turnLeft() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  currentState = TURNING;
  Serial.println("TURNING LEFT");
}

void turnRight() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  currentState = TURNING;
  Serial.println("TURNING RIGHT");
}

void stopAllMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  currentState = STOPPED;
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

// Smooth servo movement
void smoothServoMove(int targetAngle) {
  static int currentPos = SERVO_CENTER;
  targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
  
  while (currentPos != targetAngle) {
    currentPos += (targetAngle > currentPos) ? 1 : -1;
    usServo.write(currentPos);
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
  
  // Decision making
  if (bestAngle < 60) {
    Serial.println(">>> Hard left turn");
    moveBackward();
    delay(800);
    turnLeft();
    delay(1200);
  } 
  else if (bestAngle > 120) {
    Serial.println(">>> Hard right turn");
    moveBackward();
    delay(800);
    turnRight();
    delay(1200);
  }
  else if (bestAngle < 90) {
    Serial.println(">>> Slight left turn");
    turnLeft();
    delay(600);
  }
  else if (bestAngle > 90) {
    Serial.println(">>> Slight right turn");
    turnRight();
    delay(600);
  }
  else {
    Serial.println(">>> Path clear ahead");
  }
}

// Autonomous navigation
void autonomousDrive() {
  float distance = getDistanceCM();
  Serial.print("Forward distance: ");
  Serial.print(distance);
  Serial.println("cm");

  if (distance < OBSTACLE_DISTANCE_CM) {
    Serial.println("! OBSTACLE DETECTED !");
    stopAllMotors();
    scanEnvironment();
  } 
  else {
    moveForward();
  }
}

// Handle incoming commands
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
  if (memcmp(sender->src_addr, controllerMac, 6) == 0 && len == 1) {
    Serial.print("Received command: ");
    Serial.println(data[0]);

    if (data[0] == 9) {  // Toggle autonomous mode
      selfDrivingMode = !selfDrivingMode;
      Serial.print("Self-Driving Mode: ");
      Serial.println(selfDrivingMode ? "ON" : "OFF");
      stopAllMotors();
      return;
    }

    if (selfDrivingMode) {
      Serial.println("(Ignoring - in self-driving mode)");
      return;
    }
    
    stopAllMotors();
        switch (data[0]) {
            case 1: moveForward(); break;
            case 2: moveBackward(); break;
            case 3: turnLeft(); break; 
            case 4: turnRight(); break;  
            case 5: moveForward(); turnLeft(); break;
            case 6: moveForward(); turnRight(); break;
            case 7: moveBackward(); turnLeft(); break;
            case 8: moveBackward(); turnRight(); break;
            case 0: stopAllMotors(); break;
        }

  }
}

void loop() {
  if (selfDrivingMode && millis() - lastAutoDriveCheck >= AUTO_DRIVE_INTERVAL) {
    autonomousDrive();
    lastAutoDriveCheck = millis();
  }
  delay(50);
}