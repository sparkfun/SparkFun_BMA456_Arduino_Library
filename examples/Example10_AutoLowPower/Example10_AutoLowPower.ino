/*
  Example 10 - Auto Low Power

  The BMA400 library's auto-power example, the BMA456 way. With auto low power enabled, the
  feature engine drops the accelerometer into a slow averaging mode by itself when nothing is
  happening, and restores the host's configuration the moment motion returns. The host does
  nothing at all - no polling, no mode changes.

  Two entry conditions are available, and either or both can be used:

    onTimeout   - any-motion has been silent for timeoutDuration (20 ms per LSB)
    onNoMotion  - the no-motion detector fired

  Exit is always on any-motion, so the any-motion detector is enabled here too.

  This sketch prints the power state whenever it changes. Leave the board alone for two
  seconds to see it drop into low power, then nudge it.

  Both feature sets support auto low power; this sketch uses the default, BMA456H.

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

bool lastState = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 10 - Auto Low Power");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin() == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    // The host's "awake" configuration: 100 Hz with 4-sample averaging.
    myAccel.setAccelConfig(BMA456_ODR_100_HZ, BMA456_RANGE_2G, BMA456_BW_NORMAL_AVG4, BMA456_PERF_MODE_AVERAGING);

    // Wake-up sensitivity: about 50 mg of slope for 40 ms.
    sfe_bma456_motion_config_t anyMotion;
    myAccel.getAnyMotionConfig(anyMotion);
    anyMotion.threshold = 102;
    anyMotion.duration = 2;
    myAccel.setAnyMotionConfig(anyMotion);
    myAccel.enableFeature(BMA456_FEAT_ANY_MOTION);

    sfe_bma456_auto_low_power_config_t alp;
    alp.onTimeout = true;
    alp.timeoutDuration = 100; // 100 x 20 ms = 2 s without motion
    alp.onNoMotion = false;
    alp.lowPowerODR = BMA456_LP_ODR_1_56_HZ;
    alp.featureOptimizedConfig = true; // Let the engine choose the low-power ACC_CONF
    myAccel.setAutoLowPowerConfig(alp);

    myAccel.enableAutoLowPower(true);

    Serial.println();
    Serial.println("Ready. Leave the board still for two seconds, then move it.");
}

void loop()
{
    bool inLowPower = false;

    if (myAccel.getAutoLowPowerState(inLowPower) == ksfTkErrOk && inLowPower != lastState)
    {
        lastState = inLowPower;
        Serial.println(inLowPower ? "Entered low power (1.56 Hz)" : "Awake (host configuration restored)");
    }

    delay(50);
}
