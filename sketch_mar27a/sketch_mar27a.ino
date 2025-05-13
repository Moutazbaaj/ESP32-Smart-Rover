/*
===== SMART ROVER CONTROLLER =====
Features:
- ESP-NOW wireless communication
- OLED status display
- Button input with combo detection
- Autonomous mode activation
- Real-time status updates

Hardware Components:
- ESP32 microcontroller
- SSD1306 OLED display
- Tactile buttons
*/

#include <esp_now.h>                    // Library for ESP-NOW communication
#include <WiFi.h>                       // WiFi library (required for ESP-NOW)
#include <Wire.h>                       // I2C communication library
#include <Adafruit_GFX.h>              // Core graphics library for OLED
#include <Adafruit_SSD1306.h>          // OLED driver library

// MAC address of the rover ESP32 (receiver)
uint8_t roverMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x1D, 0xB4};

// GPIO pin definitions for control buttons
#define BUTTON_FORWARD_PIN  12
#define BUTTON_BACKWARD_PIN 13
#define BUTTON_RIGHT_PIN    27
#define BUTTON_LEFT_PIN     14
#define BUTTON_SP_UP        33
#define BUTTON_SP_DW        32
#define BUTTON_Servo        17

// OLED screen resolution and reset pin
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Create an SSD1306 display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Combo button detection for activating autonomous mode
unsigned long comboStartTime = 0;      // Timestamp when combo started
bool comboActive = false;              // Flag to indicate combo state
const int COMBO_DURATION = 3000;       // Combo hold time in milliseconds

// Command tracking
uint8_t lastCommand = 0;
unsigned long lastDebounceTime = 0;    // For debouncing buttons
const int DEBOUNCE_DELAY = 50;         // Debounce delay in ms

// Structure to receive status updates from the rover
typedef struct RoverStatus {
    char action[20];       // Text description of current action
    float distanceCM;      // Ultrasonic distance in cm
    int servoAngle;        // Servo angle where object detected
    int motorSpeed;        // Motor PWM speed value
    bool autoMode;         // Autonomous mode status
} RoverStatus;

void setup() {
  Serial.begin(115200);    // Start serial communication for debugging
  Serial.println("===== CONTROLLER (Direction Fixed) =====");

  // Set all button pins to input with internal pull-up resistors
  pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACKWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SP_UP, INPUT_PULLUP);
  pinMode(BUTTON_SP_DW, INPUT_PULLUP);
  pinMode(BUTTON_Servo, INPUT_PULLUP);
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
     Serial.println("SSD1306 allocation failed");
     for(;;); // Infinite loop if OLED fails
  }
   
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Waiting for data...");
  display.display();

  // Set WiFi to Station mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed, restarting...");
    ESP.restart();
  }

  // Register receive callback
  esp_now_register_recv_cb(onDataReceived);

  // Configure peer (rover) for ESP-NOW
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, roverMac, 6);   // Set MAC address
  peerInfo.channel = 0;                      // Default WiFi channel
  peerInfo.encrypt = false;                  // No encryption
  peerInfo.ifidx = WIFI_IF_STA;              // Interface

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }
}

// Callback function when data is received from rover
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
  if (comboActive) return;  // Skip display update if in combo detection

  Serial.print("Data received, length: ");
  Serial.println(len);
  
  if (len == sizeof(RoverStatus)) {
    RoverStatus status;
    memcpy(&status, data, sizeof(status)); // Copy received bytes into status struct

    // Debug output
    Serial.print("Action: ");
    Serial.println(status.action);
    Serial.print("Distance: ");
    Serial.println(status.distanceCM);
    Serial.print("Angle: ");
    Serial.println(status.servoAngle);
    Serial.print("Motor PWM: ");
    Serial.println(status.motorSpeed);

    // Update OLED display
    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(32, 0); 
    display.println("ROVER STATUS");

    display.setCursor(0, 12);
    display.print("Act:");
    display.println(status.action);

    display.setCursor(0, 26);
    display.print("Distance:");
    display.print(status.distanceCM);
    display.print(" cm");

    display.setCursor(0, 38);
    display.print("Angle: ");
    display.print(status.servoAngle);
    display.print((char)247); // Degree symbol

    display.setCursor(0, 50);
    display.print("PWM: ");
    display.print(status.motorSpeed);

    display.setCursor(66, 50);
    display.print("AutoM:");
    display.print(status.autoMode ? "ON " : "OFF");

    display.display();
  } else {
    Serial.println("Invalid data length, skipping...");
  }
}

// Sends command to the rover via ESP-NOW
void sendCommand(uint8_t command) {
  esp_now_send(roverMac, &command, sizeof(command));
  Serial.print("Sent Command: ");
  Serial.println(command);
  lastCommand = command;
}

void loop() {
  static uint8_t lastSentCommand = 0;
  static bool comboWasActive = false;

  // Read current state of all buttons
  bool forward  = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
  bool backward = (digitalRead(BUTTON_BACKWARD_PIN) == LOW);
  bool left     = (digitalRead(BUTTON_LEFT_PIN) == LOW);
  bool right    = (digitalRead(BUTTON_RIGHT_PIN) == LOW);
  bool up       = (digitalRead(BUTTON_SP_UP) == LOW);
  bool down     = (digitalRead(BUTTON_SP_DW) == LOW);
  bool servo    = (digitalRead(BUTTON_Servo) == LOW);
  bool anyPressed = forward || backward || left || right || up || down || servo;

  // Detect combo for self-driving mode (LEFT + RIGHT held)
  if (left && right && !comboActive) {
    display.clearDisplay();
    display.setCursor(2, 30);
    display.print("HOLD 3s FOR AUTO MODE");
    display.display();
    comboStartTime = millis();
    comboActive = true;
    Serial.println("Combo started (hold 3s for self-driving)");
  } 
  else if (!left || !right) {
    comboActive = false; // Combo interrupted
  }

  // If combo held long enough, trigger self-driving
  if (comboActive && (millis() - comboStartTime >= COMBO_DURATION)) {
    sendCommand(9); // Command 9 = Auto mode
    comboActive = false;
    comboWasActive = true;
    Serial.println("SELF-DRIVING MODE ACTIVATED!");
    delay(1000);
    return;
  }

  // If not in combo and debounce time passed, check inputs
  if (!comboActive && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    uint8_t newCommand = 0;
    bool isCombo = false;

    // Determine command based on button states
    if (forward) {
      if (left)        { newCommand = 5; isCombo = true; }  // Forward + Left
      else if (right)  { newCommand = 6; isCombo = true; }  // Forward + Right
      else             newCommand = 1;                      // Forward
    } 
    else if (backward) {
      if (left)        { newCommand = 7; isCombo = true; }  // Backward + Left
      else if (right)  { newCommand = 8; isCombo = true; }  // Backward + Right
      else             newCommand = 2;                      // Backward
    }
    else if (servo) {
      if (up)          { newCommand = 10; isCombo = true; } // Servo + Up
      else if (down)   { newCommand = 11; isCombo = true; } // Servo + Down
      else             newCommand = 15;                     // Servo only
    } 
    else if (left)     newCommand = 3;                      // Left
    else if (right)    newCommand = 4;                      // Right
    else if (up) {
      if (down)        { newCommand = 12; isCombo = true; } // Up + Down
      else             newCommand = 13;                     // Speed up
    }
    else if (down)     newCommand = 14;                     // Speed down

    // Handle different transitions between commands
    if (isCombo) {
      sendCommand(newCommand);      // Combo command (e.g., 5-8)
      lastSentCommand = newCommand;
      comboWasActive = true;
    } 
    else if (comboWasActive && anyPressed) {
      sendCommand(0);               // Reset combo
      delay(5);
      sendCommand(newCommand);     // Send regular command
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (anyPressed && (newCommand != lastSentCommand || (newCommand >= 1 && newCommand <= 4 || newCommand == 13 || newCommand == 14))) {
      sendCommand(newCommand);     // Normal command or repeat
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (!anyPressed && lastSentCommand != 0) {
      sendCommand(0);              // Stop all motion when no button pressed
      lastSentCommand = 0;
      comboWasActive = false;
    }

    lastDebounceTime = millis();   // Update debounce timer
  }
  delay(10); // Small delay to reduce loop CPU usage
}