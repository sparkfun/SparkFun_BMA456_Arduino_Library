/**
 * @file SparkFun_BMA456.h
 * @brief Arduino-specific implementation for the SparkFun Qwiic BMA456 Triple-Axis Accelerometer.
 *
 * @details
 * This file provides the Arduino wrappers for the Bosch BMA456. SfeBMA456ArdI2C and SfeBMA456ArdSPI
 * inherit from the platform-independent sfDevBMA456 driver and supply the bus - one over Arduino's
 * Wire library, the other over SPI - through the SparkFun Toolkit. One of these two classes is all
 * a sketch needs.
 *
 * I2C is the default, and the Qwiic connector makes it the path of least resistance:
 *
 *     SfeBMA456ArdI2C accel;
 *     accel.begin();                          // Qwiic, address 0x18, BMA456H feature set
 *     accel.begin(sfeBMA456FeaturesMM);       // same, BMA456MM feature set
 *     accel.begin(SFE_BMA456_ALT_ADDRESS);    // ADR jumper closed, 0x19
 *
 * The SPI pins are broken out as well, and nothing above the bus layer changes:
 *
 *     SfeBMA456ArdSPI accel;
 *     accel.begin(10);                        // chip select on D10
 *
 * CHOOSING A FEATURE SET. The BMA456 feature engine runs one Bosch configuration at a time. Pick
 * ::sfeBMA456FeaturesH for step counting and activity, ::sfeBMA456FeaturesMM for orientation,
 * high-g, low-g (free fall) and significant motion. Both have tap, any/no-motion and auto low power.
 * ::sfeBMA456FeaturesNone skips the upload and its 6 kB of flash for a plain accelerometer.
 *
 * @author SparkFun Electronics
 * @date September 2026
 * @copyright Copyright (c) 2026, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_BMA456_Arduino_Library
 */

#pragma once

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevBMA456.h"
#include <Arduino.h>
// clang-format on

/**
 * @brief Default SPI clock for SfeBMA456ArdSPI, in hertz.
 *
 * @details 4 MHz moves a sample in a few microseconds, sits comfortably inside the part's SPI
 * limit, and is forgiving of jumper wires. Pass a different clock to begin() if needed.
 */
#define SFE_BMA456_SPI_CLOCK_DEFAULT 4000000

/**
 * @brief Arduino microsecond delay shared by both wrappers.
 *
 * @details delayMicroseconds() is only accurate to about 16 ms on AVR, so long delays - the
 * 150 ms feature-engine boot, for one - go through delay() for the whole milliseconds.
 *
 * @param us Delay in microseconds.
 */
inline void sfeBMA456ArdDelayUs(uint32_t us)
{
    if (us >= 1000)
    {
        delay(us / 1000);
        us %= 1000;
    }

    if (us > 0)
        delayMicroseconds((unsigned int)us);
}

///////////////////////////////////////////////////////////////////////////////
// SfeBMA456ArdI2C
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBMA456ArdI2C
 * @brief Arduino I2C implementation for the SparkFun Qwiic BMA456 breakout.
 *
 * @details Owns an sfTkArdI2C bus object wrapping Arduino's Wire library and hands it to
 * sfDevBMA456. Every measurement, configuration, interrupt, FIFO and feature method is inherited
 * unchanged - this class adds only begin() and the address helpers.
 *
 * @see sfDevBMA456
 * @see SfeBMA456ArdSPI
 */
class SfeBMA456ArdI2C : public sfDevBMA456
{
  public:
    SfeBMA456ArdI2C()
    {
    }

    /**
     * @brief Initialize the board over I2C.
     *
     * @details Confirms something is answering at @p address, checks the chip ID, soft resets,
     * uploads the feature configuration and enables the accelerometer (100 Hz, +/-4 g).
     *
     * @note The upload is 6144 bytes: about 0.6 s at 400 kHz and 1.1 s at 100 kHz, most of it the
     * 450 us gap required after each register write. Call Wire.setClock(400000) after Wire.begin()
     * if boot time matters.
     *
     * @param address 7-bit I2C address. ::SFE_BMA456_DEFAULT_ADDRESS (0x18) or
     * ::SFE_BMA456_ALT_ADDRESS (0x19).
     * @param wirePort TwoWire instance to use (default: Wire).
     * @param features Feature set to load (default: ::sfeBMA456FeaturesH).
     * @return true if the device was found and initialized.
     */
    bool begin(uint8_t address = SFE_BMA456_DEFAULT_ADDRESS, TwoWire &wirePort = Wire,
               const sfe_bma456_feature_set_t &features = sfeBMA456FeaturesH)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        // Confirm something is at this address before reading registers, so a missing board reports
        // a bus failure rather than a chip ID mismatch.
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        return sfDevBMA456::begin(&_theI2CBus, features) == ksfTkErrOk;
    }

    /**
     * @brief Initialize the board over I2C with a given feature set.
     *
     * @param features Feature set to load.
     * @param address 7-bit I2C address (default: ::SFE_BMA456_DEFAULT_ADDRESS).
     * @param wirePort TwoWire instance to use (default: Wire).
     * @return true if the device was found and initialized.
     */
    bool begin(const sfe_bma456_feature_set_t &features, uint8_t address = SFE_BMA456_DEFAULT_ADDRESS,
               TwoWire &wirePort = Wire)
    {
        return begin(address, wirePort, features);
    }

    /**
     * @brief Initialize the board over I2C on an explicit Wire port at the default address.
     *
     * @param wirePort TwoWire instance to use.
     * @param features Feature set to load (default: ::sfeBMA456FeaturesH).
     * @return true if the device was found and initialized.
     */
    bool begin(TwoWire &wirePort, const sfe_bma456_feature_set_t &features = sfeBMA456FeaturesH)
    {
        return begin(SFE_BMA456_DEFAULT_ADDRESS, wirePort, features);
    }

    /**
     * @brief Get the 7-bit I2C address this object is using.
     * @return The configured I2C address.
     */
    uint8_t getAddress(void)
    {
        return _theI2CBus.address();
    }

    /**
     * @brief Check whether a BMA456 answers at either address on a given bus.
     *
     * @details Pings each address and reads the chip ID, so another part that merely answers at
     * 0x18 is not mistaken for a BMA456.
     *
     * @param address Receives the address where a BMA456 was found.
     * @param wirePort TwoWire instance to search (default: Wire).
     * @return true if a BMA456 was found.
     */
    static bool findDevice(uint8_t &address, TwoWire &wirePort = Wire)
    {
        static const uint8_t kAddresses[2] = {SFE_BMA456_DEFAULT_ADDRESS, SFE_BMA456_ALT_ADDRESS};

        for (uint8_t i = 0; i < 2; i++)
        {
            SfeBMA456ArdI2C candidate;

            if (candidate._theI2CBus.init(wirePort, kAddresses[i]) != ksfTkErrOk)
                continue;

            if (candidate._theI2CBus.ping() != ksfTkErrOk)
                continue;

            candidate.setCommunicationBus(&candidate._theI2CBus);

            if (!candidate.isConnected())
                continue;

            address = kAddresses[i];
            return true;
        }

        return false;
    }

  protected:
    /** @brief Arduino microsecond delay for the Bosch SensorAPI. */
    void delayUs(uint32_t us) override
    {
        sfeBMA456ArdDelayUs(us);
    }

  private:
    /** @brief Arduino I2C bus interface used for all communication with the board. */
    sfTkArdI2C _theI2CBus;
};

///////////////////////////////////////////////////////////////////////////////
// SfeBMA456ArdSPI
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBMA456ArdSPI
 * @brief Arduino SPI implementation for the SparkFun Qwiic BMA456 breakout.
 *
 * @details Owns an sfTkArdSPI bus object and hands it to sfDevBMA456. Everything above the bus is
 * identical to the I2C class.
 *
 * @note THE BMA456 POWERS UP IN I2C MODE and switches to SPI on the first rising edge of CSB, and
 * every SPI read returns one dummy byte ahead of the data. Bosch's code handles both - a throwaway
 * read in bma4_init() and after every reset, and a dummy-byte offset on every read.
 *
 * @see sfDevBMA456
 * @see SfeBMA456ArdI2C
 */
class SfeBMA456ArdSPI : public sfDevBMA456
{
  public:
    SfeBMA456ArdSPI()
    {
    }

    /**
     * @brief Initialize the board over SPI.
     *
     * @details Drives chip select high before making it an output, so the pin never glitches low
     * at a device that has not yet decided which interface it speaks.
     *
     * @param csPin Chip select pin.
     * @param clockFrequency SPI clock in hertz (default: ::SFE_BMA456_SPI_CLOCK_DEFAULT).
     * @param spiPort SPIClass instance to use (default: SPI).
     * @param features Feature set to load (default: ::sfeBMA456FeaturesH).
     * @return true if the device was found and initialized.
     */
    bool begin(uint8_t csPin, uint32_t clockFrequency = SFE_BMA456_SPI_CLOCK_DEFAULT, SPIClass &spiPort = SPI,
               const sfe_bma456_feature_set_t &features = sfeBMA456FeaturesH)
    {
        // High first, then output. Reversing these glitches CS low for a moment on most cores.
        digitalWrite(csPin, HIGH);
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);

        // Mode 0. The BMA456 accepts mode 0 and mode 3.
        SPISettings settings(clockFrequency, MSBFIRST, SPI_MODE0);

        if (_theSPIBus.init(spiPort, settings, csPin, true) != ksfTkErrOk)
            return false;

        return sfDevBMA456::begin(&_theSPIBus, features) == ksfTkErrOk;
    }

    /**
     * @brief Initialize the board over SPI with a given feature set.
     *
     * @param csPin Chip select pin.
     * @param features Feature set to load.
     * @param clockFrequency SPI clock in hertz (default: ::SFE_BMA456_SPI_CLOCK_DEFAULT).
     * @param spiPort SPIClass instance to use (default: SPI).
     * @return true if the device was found and initialized.
     */
    bool begin(uint8_t csPin, const sfe_bma456_feature_set_t &features,
               uint32_t clockFrequency = SFE_BMA456_SPI_CLOCK_DEFAULT, SPIClass &spiPort = SPI)
    {
        return begin(csPin, clockFrequency, spiPort, features);
    }

    /**
     * @brief Get the chip select pin this object is using.
     * @return The configured chip select pin.
     */
    uint8_t getCSPin(void)
    {
        return _theSPIBus.cs();
    }

  protected:
    /** @brief Arduino microsecond delay for the Bosch SensorAPI. */
    void delayUs(uint32_t us) override
    {
        sfeBMA456ArdDelayUs(us);
    }

  private:
    /** @brief Arduino SPI bus interface used for all communication with the board. */
    sfTkArdSPI _theSPIBus;
};
