#include <esp_now.h>
#include <WiFi.h>

uint8_t roverMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x1D, 0xB4};

#define BUTTON_FORWARD_PIN  12
#define BUTTON_BACKWARD_PIN 13
#define BUTTON_LEFT_PIN     14
#define BUTTON_RIGHT_PIN    27

// Combo detection
unsigned long comboStartTime = 0;
bool comboActive = false;
const int COMBO_DURATION = 3000;

uint8_t lastCommand = 0;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY = 50;

void setup() {
  Serial.begin(115200);
  Serial.println("===== CONTROLLER (Direction Fixed) =====");

  pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACKWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, roverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
}

void sendCommand(uint8_t command) {
  esp_now_send(roverMac, &command, sizeof(command));
  Serial.print("Sent Command: ");
  Serial.println(command);
  lastCommand = command;
}

void loop() {
  bool forward  = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
  bool backward = (digitalRead(BUTTON_BACKWARD_PIN) == LOW);
  bool left     = (digitalRead(BUTTON_LEFT_PIN) == LOW);
  bool right    = (digitalRead(BUTTON_RIGHT_PIN) == LOW);

  // Left+Right combo detection
  if (left && right && !comboActive) {
    comboStartTime = millis();
    comboActive = true;
    Serial.println("Combo started (hold 3s for self-driving)");
  } 
  else if (!left || !right) {
    comboActive = false;
  }

  // Send command 9 if combo held for 3 seconds
  if (comboActive && (millis() - comboStartTime >= COMBO_DURATION)) {
    sendCommand(9);
    comboActive = false;
    Serial.println("SELF-DRIVING MODE ACTIVATED!");
    delay(1000);
    return;
  }

  // Normal button commands
  if (!comboActive && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    uint8_t command = 0;
    
    if (forward) {
      if (left)      command = 5; // Forward-Left
      else if (right) command = 6; // Forward-Right
      else           command = 1; // Forward
    } 
    else if (backward) {
      if (left)      command = 7; // Backward-Left
      else if (right) command = 8; // Backward-Right
      else           command = 2; // Backward
    } 
    else if (left)  command = 3; // Left 
    else if (right) command = 4; // Right

    if (command != lastCommand) {
      sendCommand(command);
      lastDebounceTime = millis();
    }
  }
  delay(10);
}