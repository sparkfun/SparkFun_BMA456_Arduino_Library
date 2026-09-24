/*
  Example 04 - Filtering

  The BMA456 has two filter modes, and the same register field means different things in each:

    CONTINUOUS (performance) mode - the sensor runs continuously and a digital low-pass filter
    shapes the output. The bandwidth field picks the oversampling ratio: OSR4 is the quietest,
    NORMAL the widest. This is the power-on mode.

    AVERAGING (low-power) mode - the sensor wakes, averages N samples, and sleeps again. The
    bandwidth field picks N, from 1 to 128. More samples, less noise, more current. At N = 1 it
    is the lowest-current way to run the part.

  This sketch holds the board still and measures the RMS noise on Z at a handful of settings,
  so the trade can be seen rather than taken on trust. Averaging N samples should cut white
  noise by roughly sqrt(N): going from AVG1 to AVG16 should look about four times quieter.

  Leave the board flat and untouched while it runs.

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

struct Setting
{
    const char *name;
    sfe_bma456_perf_mode_t mode;
    sfe_bma456_bandwidth_t bandwidth;
};

// Every entry is legal at 50 Hz. In averaging mode the ceiling on N falls as the ODR rises -
// AVG16 at 50 Hz, AVG8 at 100 Hz, AVG4 at 200 Hz - and the Bosch API rejects anything above it.
const Setting kSettings[] = {
    {"Continuous, NORMAL", BMA456_PERF_MODE_CONTINUOUS, BMA456_BW_NORMAL_AVG4},
    {"Continuous, OSR2  ", BMA456_PERF_MODE_CONTINUOUS, BMA456_BW_OSR2_AVG2},
    {"Continuous, OSR4  ", BMA456_PERF_MODE_CONTINUOUS, BMA456_BW_OSR4_AVG1},
    {"Averaging,  AVG1  ", BMA456_PERF_MODE_AVERAGING, BMA456_BW_OSR4_AVG1},
    {"Averaging,  AVG4  ", BMA456_PERF_MODE_AVERAGING, BMA456_BW_NORMAL_AVG4},
    {"Averaging,  AVG16 ", BMA456_PERF_MODE_AVERAGING, BMA456_BW_RES_AVG16},
};

const uint16_t kSamples = 100; // 2 s per setting at 50 Hz

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMA456 Example 4 - Filtering");

    Wire.begin();
    Wire.setClock(400000);

    // No feature engine needed here, so skip the 6 kB upload.
    while (myAccel.begin(sfeBMA456FeaturesNone) == false)
    {
        Serial.println("BMA456 not found - check your wiring!");
        delay(1000);
    }

    Serial.println();
    Serial.println("Keep the board still. RMS noise on Z, 100 samples at 50 Hz, +/-2 g:");
    Serial.println();
}

void loop()
{
    for (uint8_t i = 0; i < sizeof(kSettings) / sizeof(kSettings[0]); i++)
    {
        const Setting &s = kSettings[i];

        if (myAccel.setAccelConfig(BMA456_ODR_50_HZ, BMA456_RANGE_2G, s.bandwidth, s.mode) != ksfTkErrOk)
        {
            Serial.print(s.name);
            Serial.println("  rejected");
            continue;
        }

        // An ODR / averaging pair the device cannot run is accepted by the write and reported here.
        sfe_bma456_error_status_t errors;
        myAccel.getErrorStatus(errors);
        if (errors.accelConfError)
        {
            Serial.print(s.name);
            Serial.println("  ACC_CONF error");
            continue;
        }

        delay(100); // Let the filter settle on the new setting

        // Welford's running mean and variance - no sample buffer needed.
        float mean = 0.0f, m2 = 0.0f;
        uint16_t n = 0;

        while (n < kSamples)
        {
            bool ready = false;
            myAccel.getDataReady(ready);
            if (!ready)
                continue;

            sfe_bma456_data_t a;
            if (myAccel.getAccelData(a) != ksfTkErrOk)
                continue;

            n++;
            float delta = a.z - mean;
            mean += delta / n;
            m2 += delta * (a.z - mean);
        }

        float rmsMilliG = sqrt(m2 / (n - 1)) * 1000.0f;

        Serial.print(s.name);
        Serial.print("  mean Z = ");
        Serial.print(mean, 4);
        Serial.print(" g,  noise = ");
        Serial.print(rmsMilliG, 2);
        Serial.println(" mg RMS");
    }

    Serial.println();
    Serial.println("Done. Reset to run again.");
    while (true)
        delay(1000);
}
