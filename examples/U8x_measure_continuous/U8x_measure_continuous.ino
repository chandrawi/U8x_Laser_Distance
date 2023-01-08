#include <U8xLaser.h>
#include <SoftwareSerial.h>
#define PWREN_PIN 6

// Create software serial instance
SoftwareSerial SerialSoft(2, 3);

// Create U8x instance on a multiple serial port microcontroller like STM32
// HardwareSerial SerialU8x(PB7, PB6);
// U8xLaser U8x(SerialU8x, PWREN_PIN);
// Create U8x instance with default HardwareSerial instance
U8xLaser U8x(PWREN_PIN);

void setup() {
  // Begin U8x sensor and Serial
  U8x.begin(19200);
  SerialSoft.begin(9600);

  // Show hardware and software version and serial number
  SerialSoft.print("Hardware Version: ");
  SerialSoft.println(U8x.hardwareVersion(), HEX);
  SerialSoft.print("Software Version: ");
  SerialSoft.println(U8x.softwareVersion(), HEX);
  SerialSoft.print("Serial Number: 0x");
  SerialSoft.println(U8x.serialNumber(), HEX);
  SerialSoft.println();

  // Start continuous slow (accurate) measurement
  U8x.startMeasureSlow();
}

void loop() {

  // Get measurement result
  int32_t distance = U8x.measureResult();
  // Show measure result
  SerialSoft.print(distance);
  SerialSoft.println(" mm");
}
