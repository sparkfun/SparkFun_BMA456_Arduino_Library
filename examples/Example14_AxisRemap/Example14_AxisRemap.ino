/*
  Example 14 - Axis Remapping

  Boards rarely end up mounted the way the datasheet draws them. Rather than scattering minus
  signs and swapped variables through a sketch, tell the driver how the board is mounted once
  and have every reading come out in the product's own frame.

  A remap names, for each REPORTED axis, which PHYSICAL axis feeds it and with which sign. Each
  physical axis must be used exactly once.

  WHERE IT IS APPLIED. The BMA456 does not remap its data registers. With a feature set loaded,
  the map is written into the feature engine - so tap, orientation, high-g and the rest see the
  product's frame - and this library applies the same map to getSensorData(),
  getAccelData() and FIFO frames on the host. With sfeBMA456FeaturesNone it is host-side only,
  which for data is exactly the same result.

  This sketch alternates every five seconds between no remap and "mounted upside down, rotated
  90 degrees": reported X from physical Y, reported Y from physical X, reported Z from -Z. With
  the board flat, Z reads +1 g, then -1 g.

  A remap does not survive softReset() or performSelfTest().

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

bool remapped = false;
uint32_t lastSwitch = 0;

void applyMapping(bool upsideDown)
{
    sfe_bma456_axis_remap_t remap; // Defaults: X, Y, Z

    if (upsideDown)
    {
        remap.x = BMA456_AXIS_Y;
        remap.y = BMA456_AXIS_X;
        remap.z = BMA456_AXIS_NEG_Z;
    }

    if (myAccel.setAxisRemap(remap) != ksfTkErrOk)
        Serial.println("Remap rejected!");

    Serial.println();
    Serial.println(upsideDown ? "--- Remap: X<-Y, Y<-X, Z<- -Z ---" : "--- No remap ---");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 14 - Axis Remapping");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin() == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    applyMapping(false);
    lastSwitch = millis();
}

void loop()
{
    if (millis() - lastSwitch >= 5000)
    {
        lastSwitch = millis();
        remapped = !remapped;
        applyMapping(remapped);
    }

    sfe_bma456_data_t a;
    if (myAccel.getAccelData(a) == ksfTkErrOk)
    {
        Serial.print("X ");
        Serial.print(a.x, 3);
        Serial.print(" g\tY ");
        Serial.print(a.y, 3);
        Serial.print(" g\tZ ");
        Serial.print(a.z, 3);
        Serial.println(" g");
    }

    delay(250);
}
