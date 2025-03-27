#include <esp_now.h>
#include <WiFi.h>

// MAC address of the main ESP32 (Rover)
uint8_t roverMac[6] = {0x80, 0x7D, 0x3A, 0xED, 0xF3, 0xA0};

// Button pins for control
#define BUTTON_FORWARD_PIN 12
#define BUTTON_BACKWARD_PIN 13
#define BUTTON_LEFT_PIN 14
#define BUTTON_RIGHT_PIN 27

uint8_t lastCommand = 0;  // Stores the last sent command

// Initialize ESP-NOW
void setup() {
    Serial.begin(115200);
    Serial.println("===== CONTROLLER ESP32 INITIALIZATION =====");

    // Initialize button pins
    pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_BACKWARD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Add the Rover as a peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, roverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
}

void loop() {
    uint8_t command = 0;  // Default to stop

    bool forward = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
    bool backward = (digitalRead(BUTTON_BACKWARD_PIN) == LOW);
    bool left = (digitalRead(BUTTON_LEFT_PIN) == LOW);
    bool right = (digitalRead(BUTTON_RIGHT_PIN) == LOW);

    if (forward) {
        if (left) {
            command = 5;  // Forward-Left
        } else if (right) {
            command = 6;  // Forward-Right
        } else {
            command = 1;  // Forward
        }
    } else if (backward) {
        if (left) {
            command = 7;  // Backward-Left
        } else if (right) {
            command = 8;  // Backward-Right
        } else {
            command = 2;  // Backward
        }
    } else if (left) {
        command = 3;  // Left
    } else if (right) {
        command = 4;  // Right
    }

    // Send command only if it has changed
    if (command != lastCommand) {
        sendCommand(command);
        lastCommand = command;
    }

    delay(100);  // Small delay to debounce button presses
}

// Send control command via ESP-NOW
void sendCommand(uint8_t command) {
    esp_now_send(roverMac, &command, sizeof(command));
    Serial.print("Sending command: ");
    Serial.println(command);
}
