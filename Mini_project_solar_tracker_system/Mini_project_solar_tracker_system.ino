#include <Wire.h>
#include <U8x8lib.h>
#include <Servo.h>
#include <IRremote.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(/* reset=*/ U8X8_PIN_NONE);

Servo solarServo;

// === Pin Definitions ===
#define SERVO_PIN     4
#define LDR1_PIN      A0
#define LDR2_PIN      A2
#define BUTTON_PIN    2
#define GREEN_LED     6
#define YELLOW_LED    7
#define IR_PIN        3
#define BUZZER_PIN    5

#define IR_LEFT   0xF40B0707
#define IR_RIGHT  0xF8070707

const int PHYS_MIN = 25;
const int PHYS_MAX = 165;
const int PHYS_MID = 95;
const int DISP_MIN = -90;
const int DISP_MAX = 90;

//const float Kp = 1.2;
//const float Ki = 0.08;
//const float Kd = 0.5;

const float Kp = 10;
const float Ki = 0.08;
const float Kd = 0.5;


int currentPulse = PHYS_MID;
bool autoMode = true;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;
float error = 0, lastError = 0, integral = 0, derivative = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.clearDisplay();
  oled.drawString(0, 0, " Solar Tracker");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  solarServo.attach(SERVO_PIN);
  solarServo.write(PHYS_MID);

  playStartupBeep();
  delay(1000);
}

void loop() {
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 20;

  handleModeToggle();
  digitalWrite(GREEN_LED, autoMode);
  digitalWrite(YELLOW_LED, !autoMode);

  int ldr1 = readLDRPercent(LDR1_PIN);
  int ldr2 = readLDRPercent(LDR2_PIN);
  float pidOutput = 0;
  float currentAngle = pulseToAngle(currentPulse);

  if (autoMode) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    error = ldr1 - ldr2;
    integral += error * dt;
    integral = constrain(integral, -100, 100);
    derivative = (error - lastError) / dt;
    pidOutput = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    if (abs(error) > 3) {
      currentPulse += constrain(pidOutput, -2, 2);
      currentPulse = constrain(currentPulse, PHYS_MIN, PHYS_MAX);
      solarServo.write(currentPulse);
    }
  } else {
    if (IrReceiver.decode()) {
      uint32_t code = IrReceiver.decodedIRData.decodedRawData;
      if (code == IR_LEFT) currentPulse -= 5;
      else if (code == IR_RIGHT) currentPulse += 5;
      currentPulse = constrain(currentPulse, PHYS_MIN, PHYS_MAX);
      solarServo.write(currentPulse);
      IrReceiver.resume();
    }
  }

  updateOLED(ldr1, ldr2, currentAngle);

  if (millis() - lastSendTime >= sendInterval) {
    Serial.print(millis()); Serial.print(",");
    Serial.print(ldr1); Serial.print(",");
    Serial.print(ldr2); Serial.print(",");
    Serial.print(currentAngle, 1); Serial.print(",");
    Serial.print(pidOutput, 2); Serial.print(",");
    Serial.println(autoMode ? '1' : '0');
    lastSendTime = millis();
  }
}

void handleModeToggle() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && lastButtonState == HIGH &&
      (millis() - lastDebounceTime > debounceDelay)) {
    autoMode = !autoMode;
    lastDebounceTime = millis();
    playToggleBeep();
  }
  lastButtonState = reading;
}

void updateOLED(int ldr1, int ldr2, float angle) {
  oled.clearLine(1); oled.setCursor(0, 1);
  oled.print(" L:"); oled.print(ldr1); oled.print("  R:"); oled.print(ldr2);

  oled.clearLine(2); oled.setCursor(0, 2);
  oled.print(" Mode: "); oled.print(autoMode ? "Auto" : "Manual");

  oled.clearLine(3); oled.setCursor(0, 3);
  oled.print(" Angle: ");
  if (angle >= 0) oled.print("+");
  oled.print(angle, 0);
  oled.print((char)176); // degree symbol
}

int readLDRPercent(int pin) {
  int raw = analogRead(pin);
  return constrain(map(raw, 50, 950, 0, 100), 0, 100);
}

float pulseToAngle(int pulse) {
  return map(pulse, PHYS_MIN, PHYS_MAX, DISP_MAX, DISP_MIN);
}

void playStartupBeep() {
  digitalWrite(BUZZER_PIN, HIGH); delay(150);
  digitalWrite(BUZZER_PIN, LOW);  delay(100);
  digitalWrite(BUZZER_PIN, HIGH); delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void playToggleBeep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}