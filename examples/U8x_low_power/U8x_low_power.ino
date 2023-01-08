#include <U8xLaser.h>
#define PWREN_PIN PA15

// Create software serial instance
// SoftwareSerial Serial(2, 3);

// Create U8x instance on a multiple serial port microcontroller like STM32
HardwareSerial SerialU8x(PB7, PB6);
U8xLaser U81(SerialU8x, PWREN_PIN);
// Create U8x instance with default HardwareSerial instance
// U8xLaser U81(PWREN_PIN);

void setup() {
  // Begin U81 sensor and Serial
  U81.begin(115200);
  Serial.begin(115200);

  // Show hardware and software version and serial number
  Serial.print("Hardware Version: ");
  Serial.println(U81.hardwareVersion(), HEX);
  Serial.print("Software Version: ");
  Serial.println(U81.softwareVersion(), HEX);
  Serial.print("Serial Number: 0x");
  Serial.println(U81.hardwareVersion(), HEX);
  Serial.println();
}

void loop() {

  // Put U8x to sleep to preserve power
  U81.sleep();
  delay(5000);
  // Wake U8x from sleep
  U81.wake();

  // Start continuous fast measurement
  U81.startMeasureFast();
  // Get measurement result then stop continuous measurement
  int32_t distance = U81.measureResult();
  U81.stopMeasure();
  // Show measure result
  Serial.print(distance);
  Serial.println(" mm");
}
