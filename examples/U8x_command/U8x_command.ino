#include <U8xLaser.h>

HardwareSerial SerialU8x(PB_7, PB_6);
U8xLaser U81(SerialU8x, PA_15);

void setup() {
  // Begin U81 sensor and Serial
  U81.begin(115200);
  SerialUSB.begin();

  // Send read serial number command
  U8xFrame_t frame;
  frame.addr = 0x80;
  frame.regH = U8X_REG_SER_NUM >> 8;
  frame.regL = U8X_REG_SER_NUM;
  frame.size = 0;
  U81.sendFrame(&frame);
  // Receive response containing serial number
  if (U81.receiveFrame(&frame)) {
    uint32_t serNum = (frame.payload[0] << 24) | (frame.payload[1] << 16) | (frame.payload[2] << 8) | frame.payload[3];
    // Print serial number
    SerialUSB.print("Serial Number: 0x");
    SerialUSB.println(serNum, HEX);
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
  U81.sendFrame(&frame);

  // Receive response containing measurement result and print it to serial
  if (U81.receiveFrame(&frame, U8X_TIMEOUT_MEA_SLOW)) {
    SerialUSB.print(U8X_HEAD, HEX); SerialUSB.print(" ");
    SerialUSB.print(frame.addr, HEX); SerialUSB.print(" ");
    SerialUSB.print(frame.regH, HEX); SerialUSB.print(" ");
    SerialUSB.print(frame.regL, HEX); SerialUSB.print(" ");
    SerialUSB.print(0x00, HEX); SerialUSB.print(" ");
    SerialUSB.print(frame.size, HEX); SerialUSB.print(" ");
    for (uint8_t i=0; i<(frame.size*2); i++) {
      SerialUSB.print(frame.payload[i], HEX); SerialUSB.print(" ");
    }
    // Calculate received response checksum
    uint8_t checksum = U81.checksum(&frame);
    SerialUSB.print(checksum, HEX);
    SerialUSB.println();
  }

  delay(2000);
}
