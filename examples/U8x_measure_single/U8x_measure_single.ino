#include <U8xLaser.h>
#include <SoftwareSerial.h>
#define PWREN_PIN 6

// Create software serial instance
SoftwareSerial SerialSoft(2, 3);

// Create U8x instance on a multiple serial port microcontroller like STM32
// HardwareSerial SerialU8x(PB7, PB6);
// U8xLaser U81(SerialU8x, PWREN_PIN);
// Create U8x instance with default HardwareSerial instance
U8xLaser U81(PWREN_PIN);

void setup() {
  // Begin U81 sensor and Serial
  U81.begin(19200);
  SerialSoft.begin(9600);

  // Show hardware and software version and serial number
  SerialSoft.print("Hardware Version: ");
  SerialSoft.println(U81.hardwareVersion(), HEX);
  SerialSoft.print("Software Version: ");
  SerialSoft.println(U81.softwareVersion(), HEX);
  SerialSoft.print("Serial Number: 0x");
  SerialSoft.println(U81.hardwareVersion(), HEX);
  SerialSoft.println();
}

void loop() {

  // Single shot auto (fast or slow depend on ambient condition) measurement
  int32_t distance = U81.measureSingle();
  // Show measure result
  SerialSoft.print(distance);
  SerialSoft.println(" mm");

  delay(2000);
}
