/*
  Example 05 - FIFO Buffer

  The BMA456 has a 1 kB FIFO. Let it collect samples on its own and read them out in a burst:
  the host can sleep, or do other work, between bursts instead of servicing every sample.

  This sketch runs the accelerometer at 100 Hz, sets the watermark at 25 frames, and routes the
  watermark interrupt to INT1 - so INT1 fires four times a second, and each time there are 25
  samples waiting.

  THE FIFO IS HEADERLESS. Every frame is exactly six bytes - X, Y, Z as signed 16-bit - with no
  header in front. That is what lets readFifoData() read whole frames in fixed chunks and never
  tear one in half. The price is that frames carry no timestamp: at a fixed ODR, the time of
  frame N is simply N / ODR before the most recent one.

  THE WATERMARK IS IN BYTES. 25 frames is 25 x 6 = 150 bytes.

  RAM. 25 frames of sfe_bma456_raw_data_t is 150 bytes, which is why this sketch keeps raw
  counts and converts one sample at a time rather than holding an array of floats.

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
const uint16_t kFramesPerBurst = 25;

volatile bool fifoReady = false;

void fifoISR()
{
    fifoReady = true;
}

sfe_bma456_raw_data_t frames[kFramesPerBurst];

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 5 - FIFO Buffer");

    Wire.begin();
    Wire.setClock(400000);

    while (myAccel.begin(sfeBMA456FeaturesNone) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    myAccel.setAccelConfig(BMA456_ODR_100_HZ, BMA456_RANGE_2G);

    // Accelerometer data into the FIFO; overwrite the oldest data if it ever fills.
    myAccel.setFifoConfig(true, false);
    myAccel.setFifoWatermark(kFramesPerBurst * sfDevBMA456::kFifoFrameBytes);
    myAccel.flushFifo();

    sfe_bma456_int_pin_config_t pinConfig;
    myAccel.setInterruptPinConfig(BMA456_INT1, pinConfig);
    myAccel.setInterruptLatch(true);
    myAccel.mapInterrupt(BMA456_INT_SRC_FIFO_WATERMARK, BMA456_INT1);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), fifoISR, RISING);

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    Serial.println("Collecting. One line per burst: frames read, then the average of the burst.");
    Serial.println();
}

void loop()
{
    if (!fifoReady)
        return;

    fifoReady = false;

    uint16_t status = 0;
    myAccel.getInterruptStatus(status);

    if ((status & BMA456_INT_STATUS_FIFO_WATERMARK) == 0)
        return;

    uint16_t count = 0;
    if (myAccel.readFifoData(frames, kFramesPerBurst, count) != ksfTkErrOk)
    {
        Serial.println("FIFO read failed!");
        return;
    }

    // Average the burst in raw counts, then convert once.
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < count; i++)
    {
        sumX += frames[i].x;
        sumY += frames[i].y;
        sumZ += frames[i].z;
    }

    if (count == 0)
        return;

    Serial.print(count);
    Serial.print(" frames   avg X ");
    Serial.print(myAccel.accelRawToG((int16_t)(sumX / count)), 3);
    Serial.print(" g   Y ");
    Serial.print(myAccel.accelRawToG((int16_t)(sumY / count)), 3);
    Serial.print(" g   Z ");
    Serial.print(myAccel.accelRawToG((int16_t)(sumZ / count)), 3);
    Serial.println(" g");
}
