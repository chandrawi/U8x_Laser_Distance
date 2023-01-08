#include <U8xLaser.h>
#define PWREN_PIN PA15

HardwareSerial SerialU8x(PB7, PB6);
U8xLaser U8x(SerialU8x, PWREN_PIN);

void setup() {
  // Begin U8x sensor and Serial
  U8x.begin(115200);
  Serial.begin(115200);

  // Send read serial number command
  U8xFrame_t frame;
  frame.addr = 0x80;
  frame.regH = U8X_REG_SER_NUM >> 8;
  frame.regL = U8X_REG_SER_NUM;
  frame.size = 0;
  U8x.sendFrame(&frame);
  // Receive response containing serial number
  if (U8x.receiveFrame(&frame)) {
    uint32_t serNum = (frame.payload[0] << 24) | (frame.payload[1] << 16) | (frame.payload[2] << 8) | frame.payload[3];
    // Print serial number
    Serial.print("Serial Number: 0x");
    Serial.println(serNum, HEX);
  }
}

void loop() {
  U8xFrame_t frame;

  // Send read start measure command with slow measurement option
  frame.addr = 0x00;
  frame.regH = U8X_REG_MEA_START >> 8;
  frame.regL = U8X_REG_MEA_START;
  frame.size = 1;
  frame.payload[0] = 0x00;
  frame.payload[1] = 0x01;
  U8x.sendFrame(&frame);

  // Receive response containing measurement result and print it to serial
  if (U8x.receiveFrame(&frame, U8X_TIMEOUT_MEA_SLOW)) {
    Serial.print(U8X_HEAD, HEX); Serial.print(" ");
    Serial.print(frame.addr, HEX); Serial.print(" ");
    Serial.print(frame.regH, HEX); Serial.print(" ");
    Serial.print(frame.regL, HEX); Serial.print(" ");
    Serial.print(0x00, HEX); Serial.print(" ");
    Serial.print(frame.size, HEX); Serial.print(" ");
    for (uint8_t i=0; i<(frame.size*2); i++) {
      Serial.print(frame.payload[i], HEX); Serial.print(" ");
    }
    // Calculate received response checksum
    uint8_t checksum = U8x.checksum(&frame);
    Serial.print(checksum, HEX);
    Serial.println();
  }

  delay(2000);
}
