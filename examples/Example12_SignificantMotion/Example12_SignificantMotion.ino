/*
  Example 12 - Significant Motion

  Any-motion fires on anything - a door slamming, a desk being leaned on. Significant motion is
  pickier. It waits for motion, then deliberately ignores the sensor for a skip time, then
  requires motion AGAIN within a proof time. A single knock passes the first test and fails the
  second. Picking the device up and carrying it away passes both.

  That makes it the right trigger for "the asset has been moved" - a tracker that should stay
  asleep through vibration but wake when someone walks off with it.

    threshold   1/2048 g per LSB (0.488 mg)
    skipTime    20 ms per LSB - the pause after the first detection
    proofTime   20 ms per LSB - the window for the confirming detection

  NEEDS THE MM FEATURE SET.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Open the Serial Monitor at 115200 baud. Knock on the table (nothing should happen), then pick
  the board up and walk around with it for a few seconds.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kInterruptPin = 2;

volatile bool interruptFired = false;

void sigMotionISR()
{
    interruptFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 12 - Significant Motion");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesMM) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    sfe_bma456_sig_motion_config_t config;
    myAccel.getSigMotionConfig(config);

    Serial.print("Device defaults: threshold ");
    Serial.print(config.threshold);
    Serial.print(", skip ");
    Serial.print(config.skipTime);
    Serial.print(", proof ");
    Serial.println(config.proofTime);

    config.skipTime = 150; // 150 x 20 ms = 3 s pause after the first motion
    config.proofTime = 50; // 50 x 20 ms = 1 s window to see motion again
    myAccel.setSigMotionConfig(config);

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_SIG_MOTION, BMA456_INT1);

    myAccel.enableFeature(BMA456_FEAT_SIG_MOTION);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), sigMotionISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("Ready.");
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if (status & BMA456_INT_STATUS_SIG_MOTION)
        Serial.println("[INT1] Significant motion - the board is being carried");
}
