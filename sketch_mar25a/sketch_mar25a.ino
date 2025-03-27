#define IN1 18    // Rear Motor Forward
#define IN2 19    // Rear Motor Backward
#define IN3 21    // Front Motor Left
#define IN4 22    // Front Motor Right

#define BUTTON_FORWARD_PIN 12  // Pin for the forward button
#define BUTTON_BACKWARD_PIN 13 // Pin for the backward button
#define BUTTON_LEFT_PIN 14     // Pin for the left button
#define BUTTON_RIGHT_PIN 27    // Pin for the right button
#define BUTTON_STOP_PIN 26     // Pin for the stop button

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
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopDriving() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

void stopSteering() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void stopAllMotors() {
    stopDriving();
    stopSteering();
}

// Initialize the motor control pins
void setup() {
    Serial.begin(115200);
    Serial.println("===== ROVER INITIALIZATION =====");

    // Initialize motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Initialize button pins
    pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_BACKWARD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
    
    stopAllMotors();  // Ensure motors stop on initialization
}

// Read button states
bool readButton(int pin) {
    bool state = digitalRead(pin) == LOW;  // Active LOW button
    if (state) {
        Serial.print("Button pressed on pin ");
        Serial.println(pin);
    }
    return state;
}

void loop() {
    // Check button states
    if (readButton(BUTTON_FORWARD_PIN)) {
        moveForward();
        Serial.println("Moving Forward");
    } 
    else if (readButton(BUTTON_BACKWARD_PIN)) {
        moveBackward();
        Serial.println("Moving Backward");
    } 
    else if (readButton(BUTTON_LEFT_PIN)) {
        turnLeft();
        Serial.println("Turning Left");
    } 
    else if (readButton(BUTTON_RIGHT_PIN)) {
        turnRight();
        Serial.println("Turning Right");
    } 
    else if (readButton(BUTTON_STOP_PIN)) {
        stopAllMotors();
        Serial.println("Stopping All Motors");
    } 
    else {
        stopAllMotors();  // Stop if no button is pressed
    }

    delay(100);  // Small delay to debounce button presses
}