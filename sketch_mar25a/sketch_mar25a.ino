#include <esp_now.h>
#include <WiFi.h>

// Motor pins
#define IN1 18
#define IN2 19
#define IN3 21
#define IN4 22

// Ultrasonic pins
#define TRIG_PIN 23
#define ECHO_PIN 25

uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90};

// Movement config
const int OBSTACLE_DISTANCE_CM = 15;
const int TURN_DISTANCE_CM = 30;
const unsigned long AUTO_DRIVE_INTERVAL = 500;

// States
uint8_t currentMovement = 0;
bool selfDrivingMode = false;
unsigned long lastAutoDriveCheck = 0;

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    currentMovement = 1;
    Serial.println("MOVING FORWARD");
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    currentMovement = 2;
    Serial.println("MOVING BACKWARD");
}

void turnLeft() {
    digitalWrite(IN3, LOW);   // Front left turn
    digitalWrite(IN4, HIGH);
    currentMovement = 3;
    Serial.println("TURNING LEFT");
}

void turnRight() {
    digitalWrite(IN3, HIGH);  // Front right turn
    digitalWrite(IN4, LOW);
    currentMovement = 4;
    Serial.println("TURNING RIGHT");
}


void stopAllMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    currentMovement = 0;
    Serial.println("MOTORS STOPPED");
}

float getDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.034 / 2;
}

void autoDrive() {
    float distance = getDistanceCM();
    Serial.print("[AUTO] Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < OBSTACLE_DISTANCE_CM) {
        Serial.println("[AUTO] Obstacle! Stopping & reversing...");
        stopAllMotors();
        delay(500);
        moveBackward();
        delay(1000);
        stopAllMotors();
        
        if (random(2) == 0) {
            Serial.println("[AUTO] Turning LEFT");
            turnLeft();
        } else {
            Serial.println("[AUTO] Turning RIGHT");
            turnRight();
        }
        delay(800);
        stopAllMotors();
    } 
    else if (distance < TURN_DISTANCE_CM) {
        Serial.println("[AUTO] Near obstacle - turning");
        if (random(2) == 0) turnLeft();
        else turnRight();
        delay(500);
        stopAllMotors();
    } 
    else {
        moveForward();
    }
}

void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
    if (memcmp(sender->src_addr, controllerMac, 6) == 0 && len == 1) {
        Serial.print("Received command: ");
        Serial.println(data[0]);

        if (data[0] == 9) {
            selfDrivingMode = !selfDrivingMode;
            Serial.print("Self-Driving Mode: ");
            Serial.println(selfDrivingMode ? "ON" : "OFF");
            stopAllMotors();
            return;
        }

        if (selfDrivingMode) {
            Serial.println("Ignoring command - in self-driving mode");
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

void setup() {
    Serial.begin(115200);
    Serial.println("===== ROVER INITIALIZED =====");

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

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

void loop() {
    if (selfDrivingMode && millis() - lastAutoDriveCheck >= AUTO_DRIVE_INTERVAL) {
        autoDrive();
        lastAutoDriveCheck = millis();
    }
    delay(10);
}