#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


uint8_t roverMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x1D, 0xB4};

#define BUTTON_FORWARD_PIN  12
#define BUTTON_BACKWARD_PIN 13
#define BUTTON_LEFT_PIN     14
#define BUTTON_RIGHT_PIN    27
#define BUTTON_SP_UP        33
#define BUTTON_SP_DW        32
/*
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
*/
// Combo detection
unsigned long comboStartTime = 0;
bool comboActive = false;
const int COMBO_DURATION = 3000;

uint8_t lastCommand = 0;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY = 50;
/*
typedef struct RoverStatus {
    char action[10];  // Action description (e.g., "Turning Left")
    float distanceCM; // Distance detected
    int servoAngle;   // Angle where clearance was found
} RoverStatus;

*/
/*
// Pixel Art
// Car Icon (16x16)
static const unsigned char car_icon[] PROGMEM = {
  0b00000000, 0b00000000,
  0b00001110, 0b00000000,
  0b00011111, 0b10000000,
  0b00111111, 0b11000000,
  0b01111111, 0b11100000,
  0b01110111, 0b10100000,
  0b01111111, 0b11100000,
  0b01111111, 0b11100000,
  0b01110111, 0b10100000,
  0b00111111, 0b11000000,
  0b00011111, 0b10000000,
  0b00001110, 0b00000000,
  0b00000000, 0b00000000
};

// Left Arrow
static const unsigned char left_arrow[] PROGMEM = {
  0b00011000,
  0b00111000,
  0b01111000,
  0b11111000,
  0b01111000,
  0b00111000,
  0b00011000
};

// Right Arrow (Mirrored Left Arrow)
static const unsigned char right_arrow[] PROGMEM = {
  0b00011000,
  0b00011100,
  0b00011110,
  0b00011111,
  0b00011110,
  0b00011100,
  0b00011000
};

// Warning Icon
static const unsigned char warning_icon[] PROGMEM = {
  0b00011000,
  0b00111100,
  0b01111110,
  0b01111110,
  0b01111110,
  0b00111100,
  0b00011000
};
*/
void setup() {
  Serial.begin(115200);
  Serial.println("===== CONTROLLER (Direction Fixed) =====");

  pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACKWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SP_UP, INPUT_PULLUP);
  pinMode(BUTTON_SP_DW, INPUT_PULLUP);
  /*
    // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
     Serial.println("SSD1306 allocation failed");
     for(;;);
  }
   
   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0, 10);
   display.println("Waiting for data...");
   display.display();
*/
  WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW Init Failed, restarting...");
  ESP.restart();
}
/*
  esp_now_register_recv_cb(onDataReceived);
*/


  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, roverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }






}
/*
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
    Serial.print("Data received, length: ");
    Serial.println(len);
    
    if (len == sizeof(RoverStatus)) {
        RoverStatus status;
        memcpy(&status, data, sizeof(status));

        Serial.print("Action: ");
        Serial.println(status.action);
        Serial.print("Distance: ");
        Serial.println(status.distanceCM);
        Serial.print("Angle: ");
        Serial.println(status.servoAngle);

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("ROVER STATUS");
        display.setTextSize(1);

        // Show Action
        display.setCursor(0, 20);
        display.print("Action: ");
        display.println(status.action);

        display.setCursor(0, 35);
        display.print("Distance: ");
        display.print(status.distanceCM);
        display.println(" cm");

        display.setCursor(0, 50);
        display.print("Angle: ");
        display.print(status.servoAngle);
        display.println("°");

        display.display();
    } else {
        Serial.println("Invalid data length, skipping...");
    }
}
*/

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
  bool up       = (digitalRead(BUTTON_SP_UP) == LOW);
  bool down     = (digitalRead(BUTTON_SP_DW) == LOW);

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

    // Movement commands
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
    else if (up)    command = 10; // fast
    else if (down)  command = 11; // slow



    // Ensure command is not blocked and send it
    if (command != lastCommand) {
        sendCommand(command);
        lastDebounceTime = millis();
    }
}
  delay(10);
}