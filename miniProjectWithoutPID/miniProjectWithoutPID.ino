#include <Wire.h>
#include <U8x8lib.h>
#include <Servo.h>
#include <IRremote.h>

// === OLED Display ===
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

// === Components ===
Servo solarServo;

// === Pin Definitions ===
#define SERVO_PIN     4
#define LDR1_PIN      A0
#define LDR2_PIN      A2
#define BUTTON_PIN    2
#define GREEN_LED     6
#define YELLOW_LED    7
#define BUZZER_PIN    5
#define IR_PIN        3

#define IR_LEFT   0xF40B0707
#define IR_RIGHT  0xF8070707

// === Servo Calibration ===
const int PHYS_MIN = 25;
const int PHYS_MAX = 165;
const int PHYS_MID = 95;

int currentPulse = PHYS_MID;
bool autoMode = true;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// === Function to Convert Pulse Width to Display Angle (°) ===
float pulseToAngle(int pulse) {
  return map(pulse, PHYS_MIN, PHYS_MAX, 90, -90);
}

// === Read LDR as Percent (0–100%) ===
int readLDRPercent(int pin) {
  return constrain(map(analogRead(pin), 50, 950, 0, 100), 0, 100);
}

// === Beep Feedback ===
void playStartupBeep() {
  digitalWrite(BUZZER_PIN, HIGH); delay(150);
  digitalWrite(BUZZER_PIN, LOW);  delay(100);
  digitalWrite(BUZZER_PIN, HIGH); delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void playToggleBeep() {
  digitalWrite(BUZZER_PIN, HIGH); delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.clear();
  oled.drawString(0, 0, "Solar Tracker");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  solarServo.attach(SERVO_PIN);
  solarServo.write(PHYS_MID);

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);

  delay(1000);
  playStartupBeep();
}

void loop() {
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 20;

  handleModeToggle();
  digitalWrite(GREEN_LED, autoMode);
  digitalWrite(YELLOW_LED, !autoMode);

  int ldr2 = readLDRPercent(LDR1_PIN);  // Right sensor
  int ldr1 = readLDRPercent(LDR2_PIN);  // Left sensor
  float currentAngle = pulseToAngle(currentPulse);

  if (autoMode) {
    if (ldr1 > ldr2) currentPulse -= 5;
    else if (ldr2 > ldr1) currentPulse += 5;

    currentPulse = constrain(currentPulse, PHYS_MIN, PHYS_MAX);
    solarServo.write(currentPulse);
  } else {
    if (IrReceiver.decode()) {
      uint32_t code = IrReceiver.decodedIRData.decodedRawData;
      if (code == IR_LEFT)  currentPulse -= 5;
      if (code == IR_RIGHT) currentPulse += 5;

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
    Serial.print(0); Serial.print(",");  // No PID
    Serial.println(autoMode ? '1' : '0');
    lastSendTime = millis();
  }
}

void handleModeToggle() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && lastButtonState == HIGH &&
      millis() - lastDebounceTime > debounceDelay) {
    autoMode = !autoMode;
    lastDebounceTime = millis();
    playToggleBeep();
  }
  lastButtonState = reading;
}

void updateOLED(int ldr1, int ldr2, float angle) {
  oled.clearLine(1); oled.setCursor(0, 1);
  oled.print("L:"); oled.print(ldr1); oled.print("% R:"); oled.print(ldr2); oled.print("%");

  oled.clearLine(2); oled.setCursor(0, 2);
  oled.print("Mode: "); oled.print(autoMode ? "Auto" : "Manual");

  oled.clearLine(3); oled.setCursor(0, 3);
  oled.print("Angle: ");
  if (angle >= 0) oled.print("+");
  oled.print(angle, 1); oled.print((char)176); oled.print("   ");
}