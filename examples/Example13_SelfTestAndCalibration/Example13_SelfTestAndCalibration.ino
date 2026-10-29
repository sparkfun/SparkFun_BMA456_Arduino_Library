/*
  Example 13 - Self-Test and Offset Calibration

  Two production-line / power-on checks, in the order you would run them:

  1. SELF-TEST checks the sensing element. The proof mass is deflected electrostatically - first
     one way, then the other - and the difference on each axis is compared against the
     datasheet minimum of 1800 mg. A damaged or stuck element fails; a healthy one passes in any
     orientation. It says nothing about offset or noise.

     It is disruptive: Bosch's routine ends with a soft reset, so performSelfTest() reloads the
     feature configuration and restores the accelerometer settings afterwards. Everything else -
     enabled features, interrupt routing, FIFO setup, axis remap and OFFSETS - is lost. That is
     why it runs first here.

  2. FAST OFFSET COMPENSATION (FOC) corrects the offset. Every accelerometer reads slightly wrong
     at rest - tens of milligee is normal, and soldering stress adds more. With the board still
     and one axis pointing straight up, a correct part reads exactly 1 g on that axis and 0 g on
     the other two. performAccelFOC() averages 64 samples, computes the offsets that make that
     true, writes them to the OFFSET registers (3.9 mg per LSB) and turns compensation on. The
     correction is applied inside the device, so every reading and every feature sees it.

  THE OFFSETS ARE VOLATILE. They are lost at power-down, on softReset() and on performSelfTest().
  Calibrate once, store the three numbers (EEPROM, flash, a config file), and restore them at
  every boot with setAccelOffset() - this sketch prints them for exactly that.

  Set the board FLAT on a level surface, component side up, and leave it alone.

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

// Average a few readings so noise does not hide the offset.
void printAverage(const char *label)
{
    const uint8_t kReadings = 50;
    float sx = 0, sy = 0, sz = 0;

    for (uint8_t i = 0; i < kReadings; i++)
    {
        sfe_bma456_data_t a;
        myAccel.getAccelData(a);
        sx += a.x;
        sy += a.y;
        sz += a.z;
        delay(10);
    }

    Serial.print(label);
    Serial.print("  X ");
    Serial.print(sx / kReadings * 1000.0f, 1);
    Serial.print(" mg   Y ");
    Serial.print(sy / kReadings * 1000.0f, 1);
    Serial.print(" mg   Z ");
    Serial.print(sz / kReadings * 1000.0f, 1);
    Serial.println(" mg");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 13 - Self-Test and Offset Calibration");

    Wire.begin();
    Wire.setClock(400000);

    // Neither check needs the feature engine, so skip the 6 kB configuration upload.
    while (myAccel.begin(sfeBMA456FeaturesNone) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    myAccel.setAccelConfig(BMA456_ODR_100_HZ, BMA456_RANGE_2G);

    Serial.println();
    Serial.println("Lay the board flat, component side up, and leave it alone.");
    for (uint8_t i = 5; i > 0; i--)
    {
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }

    // ---- 1. Self-test ----------------------------------------------------------------------
    Serial.println();
    Serial.print("Self-test: ");

    bool passed = false;
    if (myAccel.performSelfTest(passed) != ksfTkErrOk)
    {
        Serial.print("could not run (Bosch code ");
        Serial.print(myAccel.getLastBoschResult());
        Serial.println(")");
        return;
    }

    Serial.println(passed ? "PASS" : "FAIL - the sensing element is out of specification");

    // ---- 2. Offset calibration -------------------------------------------------------------
    Serial.println();
    printAverage("Before FOC:");

    if (myAccel.performAccelFOC(BMA456_FOC_Z_POS) != ksfTkErrOk)
    {
        Serial.print("FOC failed, Bosch code ");
        Serial.println(myAccel.getLastBoschResult());
        return;
    }

    printAverage("After FOC: ");

    int8_t x = 0, y = 0, z = 0;
    myAccel.getAccelOffset(x, y, z);

    Serial.println();
    Serial.println("Save these and restore them at boot:");
    Serial.print("  myAccel.setAccelOffset(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(");");
    Serial.print("  (");
    Serial.print(x * 3.9f, 1);
    Serial.print(" mg, ");
    Serial.print(y * 3.9f, 1);
    Serial.print(" mg, ");
    Serial.print(z * 3.9f, 1);
    Serial.println(" mg)");
}

void loop()
{
}
