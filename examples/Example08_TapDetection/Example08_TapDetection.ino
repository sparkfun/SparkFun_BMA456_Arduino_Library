/*
  Example 08 - Tap Detection

  Single, double and triple taps, detected by the feature engine. One interrupt covers all
  three; getTapEvent() says which gesture it was.

  Both feature sets have the tap detector. This sketch uses the default, BMA456H; change the
  begin() call to sfeBMA456FeaturesMM and it runs unchanged.

  TUNING. The detector is tuned for a firm fingertip tap on a small device. How a tap arrives
  at the sensor depends enormously on the enclosure - a tap through a plastic lid is a very
  different signal from a tap on a bare board - so the tap parameters are printed at start-up
  as a starting point for tuning with setTapConfig(). sensitivityThreshold is the one that
  matters most.

  GESTURE TIMING. With waitForTimeout set, the detector waits for the whole gesture window
  (maxGestureDuration) to close before it reports, so a double tap arrives once, as a double.
  Without it, the detector can report as soon as it is sure, which is quicker for single taps.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Open the Serial Monitor at 115200 baud, then tap the board.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kInterruptPin = 2;

volatile bool interruptFired = false;

void tapISR()
{
    interruptFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 8 - Tap Detection");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesH) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    sfe_bma456_tap_config_t tap;
    myAccel.getTapConfig(tap);

    Serial.println();
    Serial.println("Tap parameters (tune with setTapConfig()):");
    Serial.print("  sensitivityThreshold  ");
    Serial.println(tap.sensitivityThreshold);
    Serial.print("  maxGestureDuration    ");
    Serial.println(tap.maxGestureDuration);
    Serial.print("  shockDuration         ");
    Serial.println(tap.shockDuration);
    Serial.print("  quietTimeAfterGesture ");
    Serial.println(tap.quietTimeAfterGesture);
    Serial.print("  waitForTimeout        ");
    Serial.println(tap.waitForTimeout);
    Serial.print("  axisSelect            ");
    Serial.println(tap.axisSelect);

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_TAP, BMA456_INT1);

    myAccel.enableFeature(BMA456_FEAT_SINGLE_TAP | BMA456_FEAT_DOUBLE_TAP | BMA456_FEAT_TRIPLE_TAP);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), tapISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("Ready. Tap the board once, twice or three times.");
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if ((status & BMA456_INT_STATUS_TAP) == 0)
        return;

    sfe_bma456_tap_event_t event;
    if (myAccel.getTapEvent(event) != ksfTkErrOk)
        return;

    if (event.tripleTap)
        Serial.println("Triple tap");
    else if (event.doubleTap)
        Serial.println("Double tap");
    else if (event.singleTap)
        Serial.println("Single tap");
}
