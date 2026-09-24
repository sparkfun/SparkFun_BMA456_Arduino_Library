/*
  Example 01 - Basic Readings (I2C)

  Reads acceleration and die temperature from a SparkFun Qwiic BMA456 breakout and prints them
  ten times a second.

  Two things to know about the BMA456:

    1. begin() uploads a 6 kB feature-engine configuration before it returns - about 0.6 s at
       400 kHz, 1.1 s at the default 100 kHz. That is why this sketch raises the I2C clock.
       The accelerometer itself works without the configuration; the features (tap, steps,
       motion) do not.

    2. begin() leaves the accelerometer running at its power-on configuration: 100 Hz,
       +/-4 g, continuous filtering. The output is always 16 bits, so a smaller range is
       free resolution: 8192 LSB/g at +/-4 g, 16384 LSB/g at +/-2 g.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC

  The board answers at 0x18 as shipped. Close the ADR jumper for 0x19.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 1 - Basic Readings (I2C)");

    Wire.begin();
    Wire.setClock(400000); // Roughly halves the configuration upload inside begin()

    // Use SFE_BMA456_ALT_ADDRESS here if the ADR jumper is closed.
    while (myAccel.begin() == false)
    {
        Serial.println("BMA456 not found - check your wiring and the ADR jumper!");
        delay(1000);
    }

    uint16_t configId = 0;
    myAccel.getConfigId(configId);

    Serial.print("Connected to a BMA456 at 0x");
    Serial.print(myAccel.getAddress(), HEX);
    Serial.print(", feature engine config ID 0x");
    Serial.println(configId, HEX);

    Serial.println();
    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tTemp(C)");
}

void loop()
{
    sfe_bma456_sensor_data_t data;

    // One burst read: the three axes and the timestamp describe the same instant.
    if (myAccel.getSensorData(data) != ksfTkErrOk)
    {
        Serial.println("Read failed!");
        delay(1000);
        return;
    }

    Serial.print(data.accel.x, 3);
    Serial.print("\t");
    Serial.print(data.accel.y, 3);
    Serial.print("\t");
    Serial.print(data.accel.z, 3);
    Serial.print("\t");

    // The temperature register updates every 1.28 s and has 1 degree resolution.
    float celsius = 0;
    if (myAccel.getTemperature(celsius) == ksfTkErrOk)
        Serial.println(celsius, 0);
    else
        Serial.println("--");

    delay(100);
}
