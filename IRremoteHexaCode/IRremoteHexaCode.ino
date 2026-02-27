#include <IRremote.h>

#define IR_RECEIVE_PIN 8  // D3 for signal pin

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Start receiver with feedback LED
  Serial.println("IR Receiver Ready");
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.print("HEX: 0x");
    Serial.println(IrReceiver.decodedIRData.command, HEX);
    Serial.print("RAW: 0x");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.resume();  // Receive the next signal
  }
}