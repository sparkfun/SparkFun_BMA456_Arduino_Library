/*
  Example 03 - Data-Ready Interrupt

  Instead of polling on a timer, let the BMA456 say when a sample is ready. Every read then
  returns a fresh sample - no duplicates from reading too fast, no gaps from reading too slow -
  and the host is free between samples.

  This sketch sets a 50 Hz output data rate, routes the data-ready interrupt to INT1, and reads
  one sample per interrupt. The timestamp column is the BMA456's own sensor time, so the
  interval it shows is the device's sample period, not the Arduino's loop timing.

  LATCHED MODE. The interrupt is latched: INT1 stays asserted until getInterruptStatus() reads
  the status register, which also clears it. A short pulse is easy for an edge-triggered ISR to
  miss while interrupts are briefly disabled; a latched level is not.

  The status register is CLEAR-ON-READ and shared by both pins, so read it once per event and
  test every bit you care about in the returned word.

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

volatile bool dataReady = false;

void dataReadyISR()
{
    dataReady = true;
}

uint32_t lastSensorTime = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 3 - Data-Ready Interrupt");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin() == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    // 50 Hz, +/-2 g, continuous filtering.
    myAccel.setAccelConfig(BMA456_ODR_50_HZ, BMA456_RANGE_2G);

    // Push-pull, active high - the defaults - and latched until the status is read.
    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_DATA_READY, BMA456_INT1);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), dataReadyISR, RISING);

    // Clear anything already pending, so the first edge is a real one.
    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println();
    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tInterval(us)");
}

void loop()
{
    if (!dataReady)
        return;

    dataReady = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status); // Also releases the latched INT1

    if ((status & BMA456_INT_STATUS_DATA_READY) == 0)
        return;

    sfe_bma456_sensor_data_t data;
    if (myAccel.getSensorData(data) != ksfTkErrOk)
        return;

    // The 24-bit sensor time wraps every 655 s; unsigned subtraction masked to 24 bits handles it.
    uint32_t ticks = (data.sensorTime - lastSensorTime) & 0x00FFFFFF;
    lastSensorTime = data.sensorTime;

    Serial.print(data.accel.x, 3);
    Serial.print("\t");
    Serial.print(data.accel.y, 3);
    Serial.print("\t");
    Serial.print(data.accel.z, 3);
    Serial.print("\t");
    Serial.println(sfDevBMA456::sensorTimeToMicros(ticks)); // About 20000 at 50 Hz
}
