/*
===== SMART ROVER CONTROLLER =====
Features:
- ESP-NOW wireless communication with rover
- OLED status display for real-time feedback
- Button input with combo detection for special commands
- Autonomous mode activation
- Real-time status updates from rover
- Debounced button inputs
- Multiple command combinations

Hardware Components:
- ESP32 microcontroller
- SSD1306 OLED display (128x64)
- Tactile buttons for control
- Internal pull-up resistors for button inputs
*/

// Include necessary libraries
#include <esp_now.h>                    // Library for ESP-NOW communication
#include <WiFi.h>                       // WiFi library (required for ESP-NOW)
#include <Wire.h>                       // I2C communication library
#include <Adafruit_GFX.h>              // Core graphics library for OLED
#include <Adafruit_SSD1306.h>          // OLED driver library

// ========== HARDWARE CONFIGURATION ==========

// MAC address of the rover ESP32 (receiver)
uint8_t roverMac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x1D, 0xB4};

// GPIO pin definitions for control buttons
#define BUTTON_FORWARD_PIN  12    // Forward movement button
#define BUTTON_BACKWARD_PIN 13    // Backward movement button
#define BUTTON_RIGHT_PIN    27    // Right turn button
#define BUTTON_LEFT_PIN     14    // Left turn button
#define BUTTON_SP_UP        33    // Speed increase button
#define BUTTON_SP_DW        32    // Speed decrease button
#define BUTTON_Servo        17    // Servo control button

// OLED display configuration
#define SCREEN_WIDTH 128          // OLED display width in pixels
#define SCREEN_HEIGHT 64          // OLED display height in pixels
#define OLED_RESET    -1          // Reset pin (-1 if sharing Arduino reset pin)

// Create an SSD1306 display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== COMBO BUTTON DETECTION ==========
unsigned long comboStartTime = 0;      // Timestamp when combo started
bool comboActive = false;              // Flag to indicate combo state
const int COMBO_DURATION = 3000;       // Combo hold time in milliseconds (3s)

// ========== COMMAND TRACKING ==========
uint8_t lastCommand = 0;               // Last sent command
unsigned long lastDebounceTime = 0;    // Last time buttons were debounced
const int DEBOUNCE_DELAY = 50;         // Debounce delay in milliseconds

// ========== DATA STRUCTURES ==========

/**
 * @brief Structure to receive status updates from the rover
 * 
 * This structure matches the one used by the rover for ESP-NOW communication
 */
typedef struct RoverStatus {
    char action[20];       // Text description of current action (null-terminated)
    float distanceCM;      // Ultrasonic distance measurement in cm
    int servoAngle;        // Current servo angle (0-180 degrees)
    int motorSpeed;        // Current motor PWM speed value (0-255)
    bool autoMode;         // Autonomous mode status (true/false)
} RoverStatus;

// ========== SETUP FUNCTION ==========

/**
 * @brief Initialization function - runs once at startup
 * 
 * Configures hardware, initializes communication protocols,
 * and sets up the OLED display.
 */
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);    
  Serial.println("===== CONTROLLER (Direction Fixed) =====");

  // Configure all button pins as inputs with internal pull-up resistors
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
   
  // Display startup message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Waiting for data...");
  display.display();

  // Configure WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW communication
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed, restarting...");
    ESP.restart();
  }

  // Register callback for received data
  esp_now_register_recv_cb(onDataReceived);

  // Configure peer (rover) information for ESP-NOW
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, roverMac, 6);   // Set rover MAC address
  peerInfo.channel = 0;                      // Use default WiFi channel
  peerInfo.encrypt = false;                  // No encryption
  peerInfo.ifidx = WIFI_IF_STA;              // Use station interface

  // Add rover as peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }
}

// ========== COMMUNICATION FUNCTIONS ==========

/**
 * @brief Callback function when data is received from rover
 * 
 * @param sender MAC address of the sender
 * @param data Pointer to received data
 * @param len Length of received data
 */
void onDataReceived(const esp_now_recv_info* sender, const uint8_t* data, int len) {
  // Skip display updates during combo detection
  if (comboActive) return;  

  Serial.print("Data received, length: ");
  Serial.println(len);
  
  // Verify data length matches expected status structure
  if (len == sizeof(RoverStatus)) {
    RoverStatus status;
    memcpy(&status, data, sizeof(status)); // Copy received data into structure

    // Print received status to serial monitor
    Serial.print("Action: ");
    Serial.println(status.action);
    Serial.print("Distance: ");
    Serial.println(status.distanceCM);
    Serial.print("Angle: ");
    Serial.println(status.servoAngle);
    Serial.print("Motor PWM: ");
    Serial.println(status.motorSpeed);

    // Update OLED display with new status
    display.clearDisplay();
    display.setTextSize(1);

    // Display header
    display.setCursor(32, 0); 
    display.println("ROVER STATUS");

    // Display current action
    display.setCursor(0, 12);
    display.print("Act:");
    display.println(status.action);

    // Display distance measurement
    display.setCursor(0, 26);
    display.print("Distance:");
    display.print(status.distanceCM);
    display.print(" cm");

    // Display servo angle
    display.setCursor(0, 38);
    display.print("Angle: ");
    display.print(status.servoAngle);
    display.print((char)247); // Degree symbol

    // Display motor speed and autonomous mode
    display.setCursor(0, 50);
    display.print("PWM: ");
    display.print(status.motorSpeed);

    display.setCursor(66, 50);
    display.print("AutoM:");
    display.print(status.autoMode ? "ON " : "OFF");

    // Update physical display
    display.display();
  } else {
    Serial.println("Invalid data length, skipping...");
  }
}

/**
 * @brief Send command to rover via ESP-NOW
 * 
 * @param command The command byte to send (0-15)
 */
void sendCommand(uint8_t command) {
  // Send command to rover's MAC address
  esp_now_send(roverMac, &command, sizeof(command));
  Serial.print("Sent Command: ");
  Serial.println(command);
  lastCommand = command; // Remember last sent command
}

// ========== MAIN LOOP ==========

/**
 * @brief Main program loop - runs continuously
 * 
 * Handles button inputs, combo detection, and command sending
 */
void loop() {
  static uint8_t lastSentCommand = 0;  // Last command actually sent
  static bool comboWasActive = false;  // Flag for combo state management

  // Read current state of all buttons (LOW = pressed due to pull-ups)
  bool forward  = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
  bool backward = (digitalRead(BUTTON_BACKWARD_PIN) == LOW);
  bool left     = (digitalRead(BUTTON_LEFT_PIN) == LOW);
  bool right    = (digitalRead(BUTTON_RIGHT_PIN) == LOW);
  bool up       = (digitalRead(BUTTON_SP_UP) == LOW);
  bool down     = (digitalRead(BUTTON_SP_DW) == LOW);
  bool servo    = (digitalRead(BUTTON_Servo) == LOW);
  bool anyPressed = forward || backward || left || right || up || down || servo;

  // === COMBO DETECTION (LEFT + RIGHT for autonomous mode) ===
  if (left && right && !comboActive) {
    // Show combo prompt on display
    display.clearDisplay();
    display.setCursor(2, 30);
    display.print("HOLD 3s FOR AUTO MODE");
    display.display();
    
    // Start combo timer
    comboStartTime = millis();
    comboActive = true;
    Serial.println("Combo started (hold 3s for self-driving)");
  } 
  // Reset combo if either button is released
  else if (!left || !right) {
    comboActive = false;
  }

  // If combo held for required duration, activate autonomous mode
  if (comboActive && (millis() - comboStartTime >= COMBO_DURATION)) {
    sendCommand(9); // Command 9 = Toggle autonomous mode
    comboActive = false;
    comboWasActive = true;
    Serial.println("SELF-DRIVING MODE ACTIVATED!");
    delay(1000); // Brief delay to prevent multiple triggers
    return;
  }

  // === NORMAL BUTTON PROCESSING ===
  // Only process if not in combo and debounce time has passed
  if (!comboActive && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    uint8_t newCommand = 0;    // New command to send
    bool isCombo = false;       // Flag for combo commands

    // Determine command based on button combinations
    if (forward) {
      if (left)        { newCommand = 5; isCombo = true; }  // Forward + Left
      else if (right)  { newCommand = 6; isCombo = true; }  // Forward + Right
      else             newCommand = 1;                      // Forward only
    } 
    else if (backward) {
      if (left)        { newCommand = 7; isCombo = true; }  // Backward + Left
      else if (right)  { newCommand = 8; isCombo = true; }  // Backward + Right
      else             newCommand = 2;                      // Backward only
    }
    else if (servo) {
      if (up)          { newCommand = 10; isCombo = true; } // Servo + Up
      else if (down)   { newCommand = 11; isCombo = true; } // Servo + Down
      else             newCommand = 15;                     // Servo only
    } 
    else if (left)     newCommand = 3;                      // Left only
    else if (right)    newCommand = 4;                      // Right only
    else if (up) {
      if (down)        { newCommand = 12; isCombo = true; } // Up + Down
      else             newCommand = 13;                     // Speed up
    }
    else if (down)     newCommand = 14;                     // Speed down

    // === COMMAND SENDING LOGIC ===
    // Handle different command transition scenarios
    if (isCombo) {
      // Send combo command immediately
      sendCommand(newCommand);
      lastSentCommand = newCommand;
      comboWasActive = true;
    } 
    else if (comboWasActive && anyPressed) {
      // Reset after combo before sending new command
      sendCommand(0);               // Stop command
      delay(5);                     // Brief delay
      sendCommand(newCommand);      // Send new command
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (anyPressed && (newCommand != lastSentCommand || 
             (newCommand >= 1 && newCommand <= 4 || newCommand == 13 || newCommand == 14))) {
      // Send normal command or repeat certain commands
      sendCommand(newCommand);
      lastSentCommand = newCommand;
      comboWasActive = false;
    }
    else if (!anyPressed && lastSentCommand != 0) {
      // Send stop command when no buttons are pressed
      sendCommand(0);
      lastSentCommand = 0;
      comboWasActive = false;
    }

    // Update debounce timer
    lastDebounceTime = millis();
  }
  
  // Small delay to reduce CPU usage
  delay(10);
}