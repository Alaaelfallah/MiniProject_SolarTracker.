#include <Servo.h>

Servo myServo;
const int SERVO_PIN = 4;  // Servo signal on D4

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  Serial.println("Servo Tester Ready!");
  Serial.println("Enter an angle (0-180) in Serial Monitor");
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();  // Read the angle you type
    
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);  // Move servo
      Serial.print("Moving to: ");
      Serial.print(angle);
      Serial.println("°");
    } else {
      Serial.println("Invalid! Enter 0-180");
    }
    
    // Clear any leftover characters (like newline)
    while (Serial.available()) Serial.read();
  }
}