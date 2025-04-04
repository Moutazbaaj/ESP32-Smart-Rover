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

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Combo detection
unsigned long comboStartTime = 0;
bool comboActive = false;
const int COMBO_DURATION = 3000;

uint8_t lastCommand = 0;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY = 50;

typedef struct RoverStatus {
    char action[20];  // Action description (e.g., "Turning Left")
    float distanceCM; // Distance detected
    int servoAngle;   // Angle where clearance was found
} RoverStatus;


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

  WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW Init Failed, restarting...");
  ESP.restart();
 }

  esp_now_register_recv_cb(onDataReceived);



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
        display.setCursor(30, 0);
        display.println("ROVER STATUS");
        display.setTextSize(1);

        // Show Action
        display.setCursor(0, 12);
        display.print("Action:");

        display.setCursor(0, 22);
        display.print(status.action);

        display.setCursor(0, 32);
        display.print("Distance:");

        display.setCursor(0, 42);
        display.print(status.distanceCM);
        display.println(" cm");

        display.setCursor(0, 54);
        display.print("Angle:");
        display.print(status.servoAngle);
        display.println("°");

        display.display();
    } else {
        Serial.println("Invalid data length, skipping...");
    }
}


void sendCommand(uint8_t command) {
  esp_now_send(roverMac, &command, sizeof(command));
  Serial.print("Sent Command: ");
  Serial.println(command);
  lastCommand = command;
}

void loop() {
  static uint8_t lastSentCommand = 0;
  static bool comboWasActive = false;
  
  bool forward  = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
  bool backward = (digitalRead(BUTTON_BACKWARD_PIN) == LOW);
  bool left     = (digitalRead(BUTTON_LEFT_PIN) == LOW);
  bool right    = (digitalRead(BUTTON_RIGHT_PIN) == LOW);
  bool up       = (digitalRead(BUTTON_SP_UP) == LOW);
  bool down     = (digitalRead(BUTTON_SP_DW) == LOW);
  bool anyPressed = forward || backward || left || right || up || down;

  // Self-driving combo detection (Left+Right)
  if (left && right && !comboActive) {
    comboStartTime = millis();
    comboActive = true;
    Serial.println("Combo started (hold 3s for self-driving)");
  } 
  else if (!left || !right) {
    comboActive = false;
  }

  // Handle self-driving activation
  if (comboActive && (millis() - comboStartTime >= COMBO_DURATION)) {
    sendCommand(9);
    comboActive = false;
    comboWasActive = true;
    Serial.println("SELF-DRIVING MODE ACTIVATED!");
    delay(1000);
    return;
  } 


  // Main command logic
  if (!comboActive && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    uint8_t newCommand = 0;
    bool isCombo = false;

    // Determine new command
    if (forward) {
      if (left)      { newCommand = 5; isCombo = true; }
      else if (right) { newCommand = 6; isCombo = true; }
      else           newCommand = 1;
    } 
    else if (backward) {
      if (left)      { newCommand = 7; isCombo = true; }
      else if (right) { newCommand = 8; isCombo = true; }
      else           newCommand = 2;
    } 
    else if (left)  newCommand = 3;
    else if (right) newCommand = 4;
    else if (up)    newCommand = 10;
    else if (down)  newCommand = 11;

    // Handle command transitions
    if (isCombo) {
      // Case 1: Combo command (5-8)
      sendCommand(newCommand);
      lastSentCommand = newCommand;
      comboWasActive = true;
    } 
    else if (comboWasActive && anyPressed) {
      // Case 2: Transition from combo to normal command
      sendCommand(0);
      delay(5);
      sendCommand(newCommand);
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (anyPressed && newCommand != lastSentCommand) {
      // Case 3: Normal command change
      sendCommand(newCommand);
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (!anyPressed && lastSentCommand != 0) {
      // Case 4: All buttons released
      sendCommand(0);
      lastSentCommand = 0;
      comboWasActive = false;
    }

    lastDebounceTime = millis();
  }
  delay(10);
}
