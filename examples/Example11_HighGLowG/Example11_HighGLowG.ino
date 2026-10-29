/*
  Example 11 - High-g and Low-g (Shock and Free Fall)

  Two detectors from the BMA456MM feature set, and each other's opposite:

    HIGH-G fires when the acceleration on a watched axis goes ABOVE a threshold - a knock, a
    drop hitting the floor, a crash. It is evaluated at 200 Hz, so its duration unit is 5 ms.

    LOW-G fires when the acceleration MAGNITUDE falls BELOW a threshold. An object in free fall
    reads close to 0 g on every axis - gravity is not "felt" while falling - so low-g is how a
    device knows it has been dropped, before it lands.

  High-g goes to INT1 and low-g to INT2. Drop the board (onto something soft) from a few tens of
  centimetres to see low-g then high-g in that order.

  NEEDS THE MM FEATURE SET.

  UNITS: thresholds and hysteresis are 1/2048 g per LSB (0.488 mg). Use +/-16 g range for
  high-g work - a real impact can saturate the smaller ranges.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMA456 Breakout
  QWIIC --> QWIIC
  D2    --> INT1  (high-g)
  D3    --> INT2  (low-g)

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMA456.h>

SfeBMA456ArdI2C myAccel;

const uint8_t kHighGPin = 2;
const uint8_t kLowGPin = 3;

volatile bool eventFired = false;

void eventISR()
{
    eventFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 11 - High-g and Low-g");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesMM) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    myAccel.setAccelConfig(BMA456_ODR_200_HZ, BMA456_RANGE_16G);

    // High-g: above 4 g for 10 ms on any axis.
    sfe_bma456_high_g_config_t highG;
    highG.threshold = 8192; // 8192 / 2048 = 4 g
    highG.hysteresis = 1024; // 0.5 g
    highG.duration = 2;     // 2 x 5 ms = 10 ms
    highG.xEnabled = highG.yEnabled = highG.zEnabled = true;
    myAccel.setHighGConfig(highG);

    // Low-g: magnitude below 0.25 g for 60 ms.
    sfe_bma456_low_g_config_t lowG;
    lowG.threshold = 512;  // 512 / 2048 = 0.25 g
    lowG.hysteresis = 256; // 0.125 g
    lowG.duration = 3;     // 3 x 20 ms = 60 ms
    myAccel.setLowGConfig(lowG);

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptPinConfig(BMA456_INT2, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_HIGH_G, BMA456_INT1);
    myAccel.mapInterrupt(BMA456_INT_SRC_LOW_G, BMA456_INT2);

    myAccel.enableFeature(BMA456_FEAT_HIGH_G | BMA456_FEAT_LOW_G);

    pinMode(kHighGPin, INPUT);
    pinMode(kLowGPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kHighGPin), eventISR, RISING);
    attachInterrupt(digitalPinToInterrupt(kLowGPin), eventISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("Ready. Tap the board hard, or drop it onto something soft.");
}

void loop()
{
    if (!eventFired)
        return;

    eventFired = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if (status & BMA456_INT_STATUS_LOW_G)
        Serial.println("[INT2] Low-g - free fall");

    if (status & BMA456_INT_STATUS_HIGH_G)
    {
        sfe_bma456_high_g_event_t event;
        myAccel.getHighGEvent(event);

        Serial.print("[INT1] High-g on");
        if (event.x)
            Serial.print(" X");
        if (event.y)
            Serial.print(" Y");
        if (event.z)
            Serial.print(" Z");
        Serial.println(event.negative ? " (negative direction)" : " (positive direction)");
    }
}
