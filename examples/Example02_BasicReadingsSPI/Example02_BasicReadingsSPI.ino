/*
  Example 02 - Basic Readings (SPI)

  The same sketch as Example 01, over SPI. Compare the two: only the object type and the
  begin() arguments change. Everything above the bus is shared.

  Two SPI details the library handles for you:

    1. The BMA456 powers up in I2C mode and switches to SPI on the first rising edge of chip
       select. The first read after power-up or reset is a throwaway, done inside begin().

    2. Every SPI read returns one dummy byte ahead of the data. The Bosch SensorAPI strips it.

  SPI does not shorten begin() by much. The 6 kB configuration upload is dominated by the
  450 us gap the part needs after every register write and the 150 ms feature-engine boot,
  not by the bus: about 0.45 s on SPI against 0.6 s on 400 kHz I2C.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  COPI (D11)   --> SDI
  CIPO (D12)   --> SDO
  SCK  (D13)   --> SCK
  D10          --> CS
  3.3V         --> 3.3V
  GND          --> GND

  The BMA456 is a 3.3 V part. On a 5 V board, use a logic level converter.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdSPI myAccel;

const uint8_t kChipSelectPin = 10;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 2 - Basic Readings (SPI)");

    SPI.begin();

    // Default clock is 4 MHz. Pass a second argument to begin() to change it.
    while (myAccel.begin(kChipSelectPin) == false)
    {
        Serial.println("BMA456 not found - check your wiring and the CS pin!");
        delay(1000);
    }

    Serial.print("Connected to a BMA456 on SPI, CS pin ");
    Serial.println(myAccel.getCSPin());

    Serial.println();
    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tTemp(C)");
}

void loop()
{
    sfe_bma456_sensor_data_t data;

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

    float celsius = 0;
    if (myAccel.getTemperature(celsius) == ksfTkErrOk)
        Serial.println(celsius, 0);
    else
        Serial.println("--");

    delay(100);
}
