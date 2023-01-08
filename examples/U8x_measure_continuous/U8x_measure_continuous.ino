#include <U8xLaser.h>
#include <SoftwareSerial.h>
#define PWREN_PIN 6

// Create software serial instance
SoftwareSerial SerialSoft(2, 3);

// Create U8x instance on a multiple serial port microcontroller like STM32
// HardwareSerial SerialU8x(PB_7, PB_6);
// U8xLaser U81(SerialU8x, PA_15);
// Create U8x instance with default HardwareSerial instance
U8xLaser U81(PWREN_PIN);

void setup() {
  // Begin U81 sensor and Serial
  U81.begin(115200);
  SerialSoft.begin(9600);
  delay(10000);

  // Show hardware and software version and serial number
  SerialSoft.print("Hardware Version: ");
  SerialSoft.println(U81.hardwareVersion(), HEX);
  SerialSoft.print("Software Version: ");
  SerialSoft.println(U81.softwareVersion(), HEX);
  SerialSoft.print("Serial Number: 0x");
  SerialSoft.println(U81.hardwareVersion(), HEX);
  SerialSoft.println();

  // Start continuous slow (accurate) measurement
  U81.startMeasureSlow();
}

void loop() {

  // Get measurement result
  int32_t distance = U81.measureResult();
  // Show measure result
  SerialSoft.print(distance);
  SerialSoft.println(" mm");
}
