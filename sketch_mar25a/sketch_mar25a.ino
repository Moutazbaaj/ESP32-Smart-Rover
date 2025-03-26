#define IN1 18  // Rear Motor Forward
#define IN2 19  // Rear Motor Backward
#define IN3 21  // Front Motor Left
#define IN4 22  // Front Motor Right

void setup() {
    Serial.begin(115200);  // Start Serial Monitor
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    Serial.println("Moving Forward...");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(1000);

    Serial.println("Moving Backward...");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    delay(1000);

    Serial.println("Turning Left...");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(500);

    Serial.println("Turning Right...");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(500);

    Serial.println("Stopping...");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(1000);
}