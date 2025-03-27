#include <esp_now.h>
#include <WiFi.h>

// Hardware pins
#define VRX_PIN 34  // Joystick X-axis (left/right)
#define VRY_PIN 35  // Joystick Y-axis (forward/backward)
#define DEADZONE 15 // Minimum movement threshold (%)
#define BUTTON_PIN 12 // Pin for the button (replace with your actual pin)

// MAC address of the receiver (ROVER)
uint8_t roverMAC[] = {0x80, 0x7D, 0x3A, 0xED, 0xF3, 0xA0};

typedef struct {
    int x;          // -100 (left) to 100 (right)
    int y;          // -100 (backward) to 100 (forward)
    bool autoMode;  // Not used in this version
    int speed;      // Fixed speed
    uint32_t counter; 
} Command;

Command commandData;
uint32_t commandCounter = 0;
bool peerAdded = false;
unsigned long lastSendTime = 0;
bool lastButtonState = false;

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery failed");
        peerAdded = false;
    }
}

void setupESP_NOW() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        ESP.restart();
    }
    esp_now_register_send_cb(OnDataSent);
}

void addPeer() {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, roverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        peerAdded = true;
        Serial.println("Peer connected");
    }
}

void sendCurrentState() {
    if (!peerAdded) {
        addPeer();
        if (!peerAdded) return;
    }

    commandData.counter = commandCounter++;
    esp_err_t result = esp_now_send(roverMAC, (uint8_t *)&commandData, sizeof(commandData));
    
    if (result == ESP_OK) {
        lastSendTime = millis();
        Serial.printf("Sent: X=%d, Y=%d\n", commandData.x, commandData.y);
    } else {
        Serial.println("Send error");
        peerAdded = false;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("==== PROPER JOYSTICK CONTROL ====");

    pinMode(VRX_PIN, INPUT);
    pinMode(VRY_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Initialize button pin
    setupESP_NOW();
    addPeer();

    // Initialize command data
    commandData.speed = 6;       // Medium speed
    commandData.autoMode = false;

    // Ensure no movement at the start
    commandData.x = 0;
    commandData.y = 0;
}

void loop() {
    // Read raw joystick values (0-4095)
    int xRaw = analogRead(VRX_PIN);
    int yRaw = analogRead(VRY_PIN);
    
    // Convert to -100 to 100 range with proper inversion
    commandData.y = map(yRaw, 0, 4095, 100, -100);  // Forward/Backward
    commandData.x = map(xRaw, 0, 4095, -100, 100);   // Left/Right

    // Apply deadzone
    if (abs(commandData.x) < DEADZONE) commandData.x = 0;
    if (abs(commandData.y) < DEADZONE) commandData.y = 0;

    // Read button state
    bool buttonState = digitalRead(BUTTON_PIN) == LOW; // Active low button

    // Send data only when joystick is moved or button is pressed
    if ((millis() - lastSendTime >= 50) && (commandData.x != 0 || commandData.y != 0 || buttonState != lastButtonState)) {
        sendCurrentState();
        lastButtonState = buttonState; // Update the button state
    }

    delay(10);
}