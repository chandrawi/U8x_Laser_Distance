#include <U8xLaser.h>
#include <SoftwareSerial.h>
#define PWREN_PIN PA15

// Create software serial instance
// SoftwareSerial Serial(2, 3);

// Create U8x instance on a multiple serial port microcontroller like STM32
HardwareSerial SerialU8x(PB7, PB6);
U8xLaser U8x(SerialU8x, PWREN_PIN);
// Create U8x instance with default HardwareSerial instance
// U8xLaser U8x(PWREN_PIN);

void setup() {
  // Begin U8x sensor and Serial
  U8x.begin(115200);
  Serial.begin(115200);

  // Show hardware and software version and serial number
  Serial.print("Hardware Version: ");
  Serial.println(U8x.hardwareVersion(), HEX);
  Serial.print("Software Version: ");
  Serial.println(U8x.softwareVersion(), HEX);
  Serial.print("Serial Number: 0x");
  Serial.println(U8x.serialNumber(), HEX);
  Serial.println();
}

void loop() {

  // Put U8x to sleep to preserve power
  U8x.sleep();
  delay(5000);
  // Wake U8x from sleep
  U8x.wake();

  // Start continuous fast measurement
  U8x.startMeasureFast();
  // Get measurement result then stop continuous measurement
  int32_t distance = U8x.measureResult();
  U8x.stopMeasure();
  // Show measure result
  Serial.print(distance);
  Serial.println(" mm");
}
