#include <esp_now.h>
#include <WiFi.h>

// Motor pins
#define IN1 18    // Rear Motor Forward
#define IN2 19    // Rear Motor Backward
#define IN3 21    // Front Motor Left
#define IN4 22    // Front Motor Right

// MAC address of the controller ESP32
//uint8_t controllerMac[6] = {0x3C, 0x71, 0xBF, 0x10, 0xD0, 0x60};
uint8_t controllerMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x62, 0x90};


// Motor control functions
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
}

void turnLeft() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void turnRight() {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopAllMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Setup ESP-NOW
void setup() {
    Serial.begin(115200);
    Serial.println("===== ROVER ESP32 INITIALIZATION =====");

    // Initialize motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callback to handle incoming messages
    esp_now_register_recv_cb(onDataReceived);

    // Add the controller as a peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, controllerMac, 6);
    peerInfo.channel = 0;  // Default channel
    peerInfo.encrypt = false;  // No encryption

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }

    stopAllMotors();  // Ensure motors stop on initialization
}

// Handle incoming control data
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
    // Only process if the MAC address matches the controller
    if (memcmp(sender->src_addr, controllerMac, 6) == 0) {
        Serial.println("Received data from controller");

        if (len == 1) {
            stopAllMotors(); // Stop all motors before executing a new command
            switch (data[0]) {
                case 1:  // Forward
                    moveForward();
                    Serial.println("Moving Forward");
                    break;
                case 2:  // Backward
                    moveBackward();
                    Serial.println("Moving Backward");
                    break;
                 case 3:  // Right
                    turnRight();
                    Serial.println("Turning Right");
                    break;
                case 4:  // Left
                    turnLeft();
                    Serial.println("Turning Left");
                    break;
                 case 5:  // Forward-Right
                    moveForward();
                    turnRight();
                    Serial.println("Moving Forward and Turning Right");
                    break;    
                case 6:  // Forward-Left
                    moveForward();
                    turnLeft();
                    Serial.println("Moving Forward and Turning Left");
                    break;
                case 7:  // Backward-Right
                    moveBackward();
                    turnRight();
                    Serial.println("Moving Backward and Turning Right");
                    break;
                case 8:  // Backward-Left
                    moveBackward();
                    turnLeft();
                    Serial.println("Moving Backward and Turning Left");
                    break;
                case 0:  // Stop
                    stopAllMotors();
                    Serial.println("Stopping All Motors");
                    break;
            }
        }
    }
}

void loop() {
    // Nothing needed here as the motor control happens through ESP-NOW messages
}