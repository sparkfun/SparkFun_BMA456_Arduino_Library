/*
  Example 09 - Step Counter and Activity

  The BMA456H feature set counts steps on its own, so a wearable can leave the host asleep all
  day and read one number at the end of it. The same step engine also classifies what the wearer
  is doing - still, walking or running - and interrupts when that changes.

  This sketch uses both, on one pin:

    Step counter   Interrupts every N x 20 steps (the watermark), here every 20. Set the
                   watermark to 0 and enable BMA456_FEAT_STEP_DETECTOR for an interrupt on every
                   step instead.
    Activity       Interrupts on a change of classification; getActivity() says which.

  The status word says which one fired, so one pin is enough. The running total is also polled
  once a second - which is all a pedometer really has to do.

  NEEDS THE H FEATURE SET, which is the default for begin().

  Step counters are built to reject isolated jolts, so the count may hold off for the first few
  steps of a walk and then jump to catch up, and the activity takes a few steps to settle. Judge
  accuracy over a hundred steps, not ten.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Open the Serial Monitor at 115200 baud, pocket the board (or hold it at your waist) and walk,
  then jog.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kInterruptPin = 2;

volatile bool interruptFired = false;

void stepISR()
{
    interruptFired = true;
}

uint32_t lastPrint = 0;

const char *activityName(sfe_bma456_activity_t activity)
{
    switch (activity)
    {
    case BMA456_ACTIVITY_STILL:
        return "Still";
    case BMA456_ACTIVITY_WALKING:
        return "Walking";
    case BMA456_ACTIVITY_RUNNING:
        return "Running";
    default:
        return "Unknown";
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 9 - Step Counter and Activity");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin() == false) // Default feature set: BMA456H
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    // Interrupt every 1 x 20 = 20 steps.
    myAccel.setStepCounterWatermark(1);
    myAccel.resetStepCount();

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_STEP_COUNTER, BMA456_INT1);
    myAccel.mapInterrupt(BMA456_INT_SRC_ACTIVITY, BMA456_INT1);

    myAccel.enableFeature(BMA456_FEAT_STEP_COUNTER | BMA456_FEAT_STEP_ACTIVITY);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), stepISR, RISING);

    // Clear anything latched during setup, so the first edge is a real event.
    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("Ready. Start walking.");
}

void loop()
{
    if (interruptFired)
    {
        interruptFired = false;

        // Clear-on-read and shared by both sources: read once, test both bits.
        uint16_t status = 0;
        myAccel.getInterruptStatus(status);

        if (status & BMA456_INT_STATUS_STEP_COUNTER)
            Serial.println("[INT1] Another 20 steps");

        if (status & BMA456_INT_STATUS_ACTIVITY)
        {
            sfe_bma456_activity_t activity;
            if (myAccel.getActivity(activity) == ksfTkErrOk)
            {
                Serial.print("[INT1] Activity: ");
                Serial.println(activityName(activity));
            }
        }
    }

    if (millis() - lastPrint >= 1000)
    {
        lastPrint = millis();

        uint32_t steps = 0;
        if (myAccel.getStepCount(steps) == ksfTkErrOk)
        {
            Serial.print("Steps: ");
            Serial.println(steps);
        }
    }
}
