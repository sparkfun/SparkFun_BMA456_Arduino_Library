/*
  Example 07 - Orientation Change

  The BMA456MM feature set classifies the board as portrait or landscape, and optionally face
  up or face down, and interrupts when that classification changes - the logic behind a screen
  that rotates when the device is turned.

  ORIENTATION NEEDS THE MM FEATURE SET. begin() is told so explicitly here. With the default
  (BMA456H) set, the orientation calls return ksfBMA456ErrFeatureUnavailable.

  Blocking suppresses changes while the board is moving, so a shake does not flip the result
  back and forth. Hysteresis stops it chattering near a 45 degree boundary.

  Stand the board on its edge and rotate it a quarter turn at a time to walk through the four
  states; lay it flat and turn it over to see face up and face down.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kInterruptPin = 2;

volatile bool interruptFired = false;

void orientationISR()
{
    interruptFired = true;
}

const char *orientationName(sfe_bma456_orientation_t orientation)
{
    switch (orientation)
    {
    case BMA456_ORIENT_PORTRAIT_UPRIGHT:
        return "Portrait, upright";
    case BMA456_ORIENT_LANDSCAPE_LEFT:
        return "Landscape, left";
    case BMA456_ORIENT_PORTRAIT_UPSIDE_DOWN:
        return "Portrait, upside down";
    case BMA456_ORIENT_LANDSCAPE_RIGHT:
        return "Landscape, right";
    default:
        return "Unknown";
    }
}

void printOrientation()
{
    sfe_bma456_orientation_t orientation;
    bool faceDown = false;

    if (myAccel.getOrientation(orientation, faceDown) != ksfTkErrOk)
        return;

    Serial.print(orientationName(orientation));
    Serial.println(faceDown ? ", face down" : ", face up");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 7 - Orientation Change");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesMM) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    sfe_bma456_orientation_config_t config;
    myAccel.getOrientationConfig(config);
    config.upsideDown = true; // Report face up / face down as well
    config.mode = 0;          // Symmetrical: the four sectors are 90 degrees each
    config.blocking = 3;      // Most aggressive: ignore changes during motion
    myAccel.setOrientationConfig(config);

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_ORIENTATION, BMA456_INT1);

    myAccel.enableFeature(BMA456_FEAT_ORIENTATION);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), orientationISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.print("Starting orientation: ");
    printOrientation();
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if (status & BMA456_INT_STATUS_ORIENTATION)
    {
        Serial.print("Orientation changed: ");
        printOrientation();
    }
}
