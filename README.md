<!-- PROJECT SHIELDS -->
[![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/chandrawi/U8x_Laser_Distance)](https://github.com/chandrawi/U8x_Laser_Distance/releases)
[![GitHub license](https://img.shields.io/github/license/chandrawi/U8x_Laser_Distance)](https://github.com/chandrawi/U8x_Laser_Distance/blob/main/LICENSE)


# U8 Series Laser Distance Sensor

Arduino library for U8 series laser distance sensor. U8 series is an accurate LIDAR (ligth detection and ranging) sensor module with +/- 1 mm accuracy from JRT Meter Technology. This library support for U81x and other laser sensor module from JRT using U(S)ART interface.

## Installation

### Using the Arduino IDE Library Manager

1. Choose `Sketch` -> `Include Library` -> `Manage Libraries...`
2. Type `U8xLaser` into the search box.
3. Search a row with title `U8xLaserDistance`.
4. Click the `Install` button to install the library.

### Using Git

Open terminal, Git bash, or command prompt in Arduino library folder then run following command. Default library folder for windows is `C:\Users\{username}\Documents\Arduino` and for linux is `~/Documents/Arduino/libraries/`. 
```sh
git clone https://github.com/chandrawi/U8x_Laser_Distance.git
```

## Wiring Connection

To work with U8x LIDAR sensor, you must connect Arduino UART pins and PWREN pin to the sensor like picture below.

![alt text](https://bsg-i.nbxc.com/product/cd/0d/21/d31a4d606a028fedbcbd5c1547.png)

## Initialization

To work with the library, first you must include `U8xLaser.h`. Then initialize class by creating an object with a `HardwareSerial` instance and a PWREN pin and optionally a RESET pin.

```c++
#define PWREN_PIN PA15
HardwareSerial SerialU8x(PB7, PB6);
U8xLaser U8x(SerialU8x, PWREN_PIN);
```

Alternatively a default `Serial` object will be used if a `HardwareSerial` instance not supplied.

```c++
#define PWREN_PIN PA15
U8xLaser U8x(PWREN_PIN);
```

## Read Status

Read current laser module status is done by calling `status` method.

```c++
uint16_t status = U8x.status();
```

The return value of the method has meaning like described in the table below.

| Status Code | Description |
|:-----------:|:-----------:|
| U8X_STAT_NO_ERR | No error |
| U8X_STAT_VLTG_LOW | Power input too low, power voltage shoul >= 2.2 V |
| U8X_STAT_INT_ERR | Internal error, don't care |
| U8X_STAT_TEMP_LOW | Module temperature is too low (< -20 °C) |
| U8X_STAT_TEMP_HIGH | Module temperature is too high (> +40 °C) |
| U8X_STAT_OUT_RANGE | Target out range |
| U8X_STAT_INV_RESULT | Invalid measure result |
| U8X_STAT_BG_LIGHT | Background light too strong |
| U8X_STAT_LASER_WEAK | Laser signal too weak |
| U8X_STAT_LASER_STRONG | Laser signal too strong |
| U8X_STAT_HW_FAULT_1 | Hardware fault 1 |
| U8X_STAT_HW_FAULT_2 | Hardware fault 2 |
| U8X_STAT_HW_FAULT_3 | Hardware fault 3 |
| U8X_STAT_HW_FAULT_4 | Hardware fault 4 |
| U8X_STAT_HW_FAULT_5 | Hardware fault 5 |
| U8X_STAT_LASER_UNSTABLE | Laser signal not stable |
| U8X_STAT_HW_FAULT_6 | Hardware fault 6 |
| U8X_STAT_HW_FAULT_7 | Hardware fault 7 |
| U8X_STAT_INV_FRAME | Invalid frame |

## Get Hardware and Software Version and S/N

Get information about laser module hardware version, software version and serial number is done by calling corresponding method.

```c++
uint16_t hwVer = U8x.hardwareVersion();
uint16_t swVer = U8x.softwareVersion();
uint32_t serNum = U8x.serialNumber();
```

## Laser Control

Laser can be turn on and off using following code.

```c++
U8x.laserOn();
U8x.laserOff();
```

## Single Shot Measurement

Single shot measurement is done by calling `measureSingle`, `measureSingleSlow`, or `measureSingleFast` method for auto, slow (accurate), or fast measuring mode respectively. Those methods will block the program and return measurement result.

```c++
void loop() {
  int32_t distance;
  // Single shot auto measurement
  distance = U8x.measureSingle();
  // Single shot slow (accurate) measurement
  distance = U8x.measureSingleSlow();
  // Single shot fast measurement
  distance = U8x.measureSingleFast();
}
```

## Continuous Measurement

Continuous measurement is started by calling `startMeasure`, `startMeasureSlow`, or `startMeasureFast` method for auto, slow (accurate), or fast measuring mode respectively. `measureResult` method must be called to get measurement result after continuous measurement started. For example:

```c++
void setup() {
  // Start continuous slow (accurate) measurement
  U8x.startMeasureSlow();
}

void loop() {
  // Get measurement result
  int32_t distance = U8x.measureResult();
}
```

When no longer needed, continuous mode can be terminated using `stopMeasure` method. For example, to get average of some measurement see code below.

```c++
void loop() {
  // Start continuous slow (accurate) measurement
  U8x.startMeasureFast();
  // Get average of 4 measurement results
  int32_t sum = 0;
  for (int8_t i=0; i<4; i++) {
    sum += U8x.measureResult();
  }
  int32_t distance = (sum + 2) / 4;
  // Stop continuous measurement
  U8x.stopMeasure();
  delay(5000);
}
```

## Signal Quality

After any measurement is done, the laser signal quality of the last measurement can retrieved by calling `getSignalQuality` method.

```c++
uint16_t sigQual = U8x.getSignalQuality();
```

## Low Power Application

This library provide `sleep` method to put laser module in power down mode and `wake` method to wake the module. During power down mode, power consumed is very low. You can see low power implementation in this [example](https://github.com/chandrawi/U8x_Laser_Distance/blob/main/examples/U8x_low_power/U8x_low_power.ino).

## Examples

See examples [here](https://github.com/chandrawi/U8x_Laser_Distance/tree/main/examples)

