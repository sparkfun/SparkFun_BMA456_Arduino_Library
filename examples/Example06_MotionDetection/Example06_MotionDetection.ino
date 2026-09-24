/*
  Example 06 - Motion Detection (Any-Motion and No-Motion)

  The first of the feature-engine examples. Any-motion and no-motion are mirror images: one
  fires when the board starts moving, the other when it has been still for a while. Together
  they are the backbone of a battery-powered design - sleep the host on no-motion, wake it on
  any-motion, and let the accelerometer do the watching.

  Both feature sets have these two detectors. This sketch uses the default, BMA456H; change the
  begin() call to sfeBMA456FeaturesMM and it runs unchanged.

  Any-motion is routed to INT1 and no-motion to INT2, so the pin alone tells them apart.

  THE UNITS ARE THE HARDWARE'S, NOT SI:

    threshold   1/2048 g per LSB, about 0.488 mg. A threshold of 205 is about 100 mg.
    duration    20 ms per LSB - the feature engine runs at 50 Hz. A duration of 5 is 100 ms.

  These detectors watch the SLOPE of acceleration (the change between samples), not its
  magnitude. A board lying still at any angle reads 1 g of gravity but zero slope, which is why
  no-motion works in any orientation.

  ENABLING IS PER AXIS. Pass BMA456_FEAT_ANY_MOTION_Z alone, say, to watch one direction only.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1  (any-motion)
  D3    --> INT2  (no-motion)

  Open the Serial Monitor at 115200 baud, then pick the board up and put it down again.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kAnyMotionPin = 2;
const uint8_t kNoMotionPin = 3;

volatile bool int1Fired = false;
volatile bool int2Fired = false;

void int1ISR()
{
    int1Fired = true;
}

void int2ISR()
{
    int2Fired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 6 - Motion Detection");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesH) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    // Start from the device's current settings and change only what matters.
    sfe_bma456_motion_config_t anyMotion;
    myAccel.getAnyMotionConfig(anyMotion);
    anyMotion.threshold = 205; // 205 x 0.488 mg = 100 mg of slope
    anyMotion.duration = 5;    // 5 x 20 ms = 100 ms
    myAccel.setAnyMotionConfig(anyMotion);

    // No-motion: under about 50 mg of slope for two full seconds. The long duration is
    // deliberate - a short one fires in the pause between footsteps.
    sfe_bma456_motion_config_t noMotion;
    myAccel.getNoMotionConfig(noMotion);
    noMotion.threshold = 102; // 102 x 0.488 mg = 50 mg
    noMotion.duration = 100;  // 100 x 20 ms = 2 s
    myAccel.setNoMotionConfig(noMotion);

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptPinConfig(BMA456_INT2, pinConfig);
    myAccel.setInterruptLatch(true);

    myAccel.mapInterrupt(BMA456_INT_SRC_ANY_MOTION, BMA456_INT1);
    myAccel.mapInterrupt(BMA456_INT_SRC_NO_MOTION, BMA456_INT2);

    if (myAccel.enableFeature(BMA456_FEAT_ANY_MOTION | BMA456_FEAT_NO_MOTION) != ksfTkErrOk)
    {
        Serial.println("Could not enable the motion detectors.");
        while (true)
            delay(1000);
    }

    pinMode(kAnyMotionPin, INPUT);
    pinMode(kNoMotionPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kAnyMotionPin), int1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(kNoMotionPin), int2ISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("Ready. Pick the board up, then set it down and leave it alone.");
    Serial.println();
}

void loop()
{
    if (!int1Fired && !int2Fired)
        return;

    int1Fired = false;
    int2Fired = false;

    // One status register serves both pins and clears on read, so read it once and test
    // every bit. Reading it also releases both latched pins.
    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if (status & BMA456_INT_STATUS_ANY_MOTION)
        Serial.println("[INT1] Motion detected");

    if (status & BMA456_INT_STATUS_NO_MOTION)
        Serial.println("[INT2] Still - this is where a battery design would sleep");
}
