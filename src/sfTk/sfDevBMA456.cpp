/**
 * @file sfDevBMA456.cpp
 * @brief Implementation file for the SparkFun Bosch BMA456 Triple-Axis Accelerometer Driver.
 *
 * @details
 * Implements sfDevBMA456: the Bosch SensorAPI bus callbacks on top of sfTkIBus, the PROGMEM-safe
 * feature-engine configuration upload, the translation between this library's feature and
 * interrupt bits and the BMA456H / BMA456MM register layouts, and the native data, temperature and
 * FIFO paths that stand in for Bosch calls with known problems.
 *
 * @author SparkFun Electronics
 * @date September 2026
 * @copyright Copyright (c) 2026, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_BMA456_Arduino_Library
 */

#include "sfDevBMA456.h"

#include <string.h>

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

// The Bosch SensorAPI is C. bma456h.h and bma456mm.h carry extern "C" guards but bma4.h does not,
// so wrap all three.
extern "C"
{
#include "../bma456_api/bma4.h"
#include "../bma456_api/bma456h.h"
#include "../bma456_api/bma456mm.h"
}

///////////////////////////////////////////////////////////////////////////////
// Feature Set Descriptors
///////////////////////////////////////////////////////////////////////////////
//
// Each descriptor is the ONLY reference to its configuration blob (the Bosch
// bma456x_write_config_file() functions also reference them, but this driver never calls those).
// With -fdata-sections / --gc-sections - which every mainstream Arduino core uses - a descriptor
// the sketch never names is discarded, and its 6144-byte blob goes with it.

extern "C"
{
    extern const uint8_t bma456h_config_file[];
    extern const uint16_t bma456h_config_file_size;
    extern const uint8_t bma456mm_config_file[];
    extern const uint16_t bma456mm_config_file_size;
}

const sfe_bma456_feature_set_t sfeBMA456FeaturesH = {BMA456_FEATURE_SET_H, bma456h_config_file,
                                                     &bma456h_config_file_size};

const sfe_bma456_feature_set_t sfeBMA456FeaturesMM = {BMA456_FEATURE_SET_MM, bma456mm_config_file,
                                                      &bma456mm_config_file_size};

const sfe_bma456_feature_set_t sfeBMA456FeaturesNone = {BMA456_FEATURE_SET_NONE, nullptr, nullptr};

///////////////////////////////////////////////////////////////////////////////
// Local Constants
///////////////////////////////////////////////////////////////////////////////

namespace
{
// Registers this file touches directly. The rest are reached through the Bosch API.
const uint8_t kRegStatus = 0x03;        // STATUS - drdy_acc in bit 7.
const uint8_t kRegTemperature = 0x22;   // TEMPERATURE - signed, 1 K/LSB, 0 = 23 degC.
const uint8_t kRegOffset0 = 0x71;       // OFFSET_0..2 - user offsets, 3.9 mg/LSB.
const uint8_t kRegAsicLsb = 0x5B;       // Feature-engine window address, low nibble.
const uint8_t kRegAsicMsb = 0x5C;       // Feature-engine window address, high byte.
const uint8_t kRegFeatureConfig = 0x5E; // Feature-engine data window. Does not auto-increment.
const uint8_t kRegFifoData = 0x26;      // FIFO_DATA. Does not auto-increment.
const uint8_t kRegInitCtrl = 0x59;      // INIT_CTRL - 0 to stream the config, 1 to start the engine.

const uint8_t kStatusDrdyAcc = 0x80;          // STATUS data-ready bit.
const uint8_t kTempInvalid = 0x80;            // TEMPERATURE "no valid data" value.
const int8_t kTempOffsetC = 23;               // TEMPERATURE zero point.
const uint8_t kInternalStatusMessage = 0x0F;  // INTERNAL_STATUS message field.
const uint8_t kInternalStatusInitOk = 0x01;   // INTERNAL_STATUS "initialized".
const uint8_t kSoftResetCmd = 0xB6;           // CMD soft reset.

// Timing. The datasheet asks for 450 us between writes while advanced power save is on, and for
// the feature engine to be given 140-150 ms to boot after INIT_CTRL is set.
const uint32_t kSoftResetDelayUs = 2000;
const uint32_t kWriteDelayUs = 450;
const uint32_t kConfigBootDelayUs = 150000;
const uint32_t kConfigPollDelayUs = 10000;
const uint8_t kConfigPollRetries = 10;

// Largest single I2C read. The Toolkit's Arduino I2C layer splits anything longer into several
// requestFrom() calls WITHOUT re-sending the register address, which is only correct if the device
// keeps auto-incrementing across a STOP. Staying at or under its 32-byte chunk sidesteps the
// question entirely.
const uint8_t kMaxI2CRead = 32;

// Fast offset compensation. 64 samples at 100 Hz is 0.64 s and cuts the 120 ug/sqrt(Hz) noise
// floor well below one 3.9 mg offset step. At +/-4 g there are 8192 counts per g and 32 counts
// per offset step (1/256 g).
const uint8_t kFocSamples = 64;
const uint16_t kFocMaxAttempts = 2000;
const uint32_t kFocSettleUs = 50000;
const uint32_t kFocPollUs = 1000;
const int16_t kFocLsbPerG = 8192;
const int16_t kFocCountsPerStep = 32;

// BMA456MM auto low power time-out ceiling - see setAutoLowPowerConfig().
const uint16_t kMMTimeoutMax = 1023;
const uint16_t kHTimeoutMax = 2047; // BMA456H time-out field is 11 bits

// Headerless FIFO frame that means "nothing here" (an over-read).
const int16_t kFifoEmptyWord = (int16_t)0x8000;

// Library feature bits grouped by where they live.
const uint16_t kFeatMotionMask = BMA456_FEAT_NO_MOTION | BMA456_FEAT_ANY_MOTION;
const uint16_t kFeatTapMask = BMA456_FEAT_SINGLE_TAP | BMA456_FEAT_DOUBLE_TAP | BMA456_FEAT_TRIPLE_TAP;
const uint16_t kFeatHOnlyMask = BMA456_FEAT_STEP_DETECTOR | BMA456_FEAT_STEP_COUNTER | BMA456_FEAT_STEP_ACTIVITY;
const uint16_t kFeatMMOnlyMask =
    BMA456_FEAT_ORIENTATION | BMA456_FEAT_LOW_G | BMA456_FEAT_HIGH_G | BMA456_FEAT_SIG_MOTION;
} // namespace

// ============================= Construction ================================

sfDevBMA456::sfDevBMA456()
    : _theBus{nullptr}, _featureSet{nullptr}, _featureSetId{BMA456_FEATURE_SET_NONE}, _range{BMA456_RANGE_4G},
      _autoLowPowerEnabled{false}, _lastBoschResult{BMA4_OK}
{
    memset(&_dev, 0, sizeof(_dev));
}

// ======================= Bosch SensorAPI Callbacks =========================

int8_t sfDevBMA456::boschRead(uint8_t reg, uint8_t *data, uint32_t len, void *intfPtr)
{
    sfDevBMA456 *self = static_cast<sfDevBMA456 *>(intfPtr);

    if (self == nullptr)
        return -1;

    return self->busRead(reg, data, len) == ksfTkErrOk ? BMA4_INTF_RET_SUCCESS : -1;
}

int8_t sfDevBMA456::boschWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *intfPtr)
{
    sfDevBMA456 *self = static_cast<sfDevBMA456 *>(intfPtr);

    if (self == nullptr)
        return -1;

    return self->busWrite(reg, data, len) == ksfTkErrOk ? BMA4_INTF_RET_SUCCESS : -1;
}

void sfDevBMA456::boschDelayUs(uint32_t period, void *intfPtr)
{
    sfDevBMA456 *self = static_cast<sfDevBMA456 *>(intfPtr);

    if (self != nullptr)
        self->delayUs(period);
}

void sfDevBMA456::delayUs(uint32_t us)
{
    // The Toolkit has no microsecond delay. Round up: too long is safe, too short is not.
    sftk_delay_ms((us + 999) / 1000);
}

// ============================ Raw Bus Access ===============================

sfTkError_t sfDevBMA456::busRead(uint8_t reg, uint8_t *data, uint32_t len)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    if (data == nullptr)
        return ksfTkErrBusNullBuffer;

    // SPI has no buffer limit to respect, and Bosch has already accounted for the dummy byte and
    // set the read bit (the Toolkit sets it again, harmlessly). One transaction.
    if (_theBus->type() == ksfTkBusTypeSPI)
    {
        size_t readBytes = 0;
        sfTkError_t rc = _theBus->readRegister(reg, data, (size_t)len, readBytes);
        if (rc != ksfTkErrOk)
            return rc;
        return readBytes == len ? ksfTkErrOk : ksfTkErrBusUnderRead;
    }

    // I2C: split into pieces that each carry their own register address. For the two windowed
    // registers that do not auto-increment, re-addressing would restart the window - but Bosch
    // never reads more than read_write_len from 0x5E and this driver never reads more than
    // kMaxTransferBytes from the FIFO, so refuse anything that would need it rather than corrupt.
    bool fixedAddress = (reg == kRegFifoData) || (reg == kRegFeatureConfig);

    if (fixedAddress && len > kMaxI2CRead)
        return ksfTkErrBusDataTooLong;

    uint32_t offset = 0;
    while (offset < len)
    {
        uint32_t chunk = len - offset;
        if (chunk > kMaxI2CRead)
            chunk = kMaxI2CRead;

        uint8_t chunkReg = fixedAddress ? reg : (uint8_t)(reg + offset);
        size_t readBytes = 0;

        sfTkError_t rc = _theBus->readRegister(chunkReg, data + offset, (size_t)chunk, readBytes);
        if (rc != ksfTkErrOk)
            return rc;
        if (readBytes != chunk)
            return ksfTkErrBusUnderRead;

        offset += chunk;
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::busWrite(uint8_t reg, const uint8_t *data, uint32_t len)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    if (data == nullptr)
        return ksfTkErrBusNullBuffer;

    if (_theBus->type() == ksfTkBusTypeSPI)
        return _theBus->writeRegister(reg, data, (size_t)len);

    // I2C: the Toolkit writes register + payload in one Wire transmission with no chunking, and
    // an AVR Wire buffer silently truncates past 32 bytes. Split, re-addressing each piece.
    bool fixedAddress = (reg == kRegFifoData) || (reg == kRegFeatureConfig);

    if (fixedAddress && len > kMaxTransferBytes)
        return ksfTkErrBusDataTooLong;

    uint32_t offset = 0;
    while (offset < len)
    {
        uint32_t chunk = len - offset;
        if (chunk > kMaxTransferBytes)
            chunk = kMaxTransferBytes;

        uint8_t chunkReg = fixedAddress ? reg : (uint8_t)(reg + offset);

        sfTkError_t rc = _theBus->writeRegister(chunkReg, data + offset, (size_t)chunk);
        if (rc != ksfTkErrOk)
            return rc;

        offset += chunk;
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::bosch(int8_t result)
{
    _lastBoschResult = result;

    // Bosch picks the gap after every register write - 2 us or 450 us - from perf_mode_status,
    // which it copies from the ACC_CONF performance bit whenever it reads or writes ACC_CONF. That
    // bit does not say whether the part is actually awake: with the accelerometer off, or in
    // averaging mode, or after auto low power has quietly rewritten ACC_CONF, the part needs
    // 450 us and would get 2. Writes here are configuration, never the data path, so always take
    // the safe gap between Bosch calls. (Within a single Bosch call its own bookkeeping applies.)
    _dev.perf_mode_status = BMA4_DISABLE;

    return result == BMA4_OK ? ksfTkErrOk : ksfBMA456ErrBosch;
}

// ========================== Setup and Identity =============================

sfTkError_t sfDevBMA456::begin(sfTkIBus *theBus, const sfe_bma456_feature_set_t &features)
{
    if (theBus != nullptr)
        setCommunicationBus(theBus);

    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    _featureSet = &features;
    _featureSetId = features.id;

    return initialize();
}

void sfDevBMA456::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;

    _dev.intf = (_theBus != nullptr && _theBus->type() == ksfTkBusTypeSPI) ? BMA4_SPI_INTF : BMA4_I2C_INTF;
    _dev.bus_read = boschRead;
    _dev.bus_write = boschWrite;
    _dev.delay_us = boschDelayUs;
    _dev.intf_ptr = this;
    _dev.variant = BMA45X_VARIANT;

    // Bosch splits feature-page accesses and its own config stream into pieces of this size.
    _dev.read_write_len = kMaxTransferBytes;
}

sfTkError_t sfDevBMA456::initialize(void)
{
    // bma4_init() performs the SPI-select dummy read when on SPI and reads CHIP_ID.
    if (bosch(bma4_init(&_dev)) != ksfTkErrOk)
        return ksfTkErrBusNoResponse;

    if (_dev.chip_id != SFE_BMA456_CHIP_ID)
        return ksfTkErrBusNoResponse;

    // Soft reset. Bosch's bma4_soft_reset() re-selects SPI immediately after the command, before
    // the part has come back; wait out the reset first and then do the dummy read.
    uint8_t command = kSoftResetCmd;
    if (bosch(bma4_write_regs(BMA4_CMD_ADDR, &command, 1, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    delayUs(kSoftResetDelayUs);

    if (_dev.intf == BMA4_SPI_INTF)
    {
        uint8_t discard = 0;
        bma4_read_regs(BMA4_CHIP_ID_ADDR, &discard, 1, &_dev);
    }

    // After reset the accelerometer is off and advanced power save is on - suspend - so every
    // write needs the long post-write delay. See bosch().
    _dev.perf_mode_status = BMA4_DISABLE;
    _autoLowPowerEnabled = false;

    // Variant setup: resolution, feature page length, default remap.
    int8_t result = BMA4_OK;
    switch (_featureSetId)
    {
    case BMA456_FEATURE_SET_H:
        result = bma456h_init(&_dev);
        break;

    case BMA456_FEATURE_SET_MM:
        result = bma456mm_init(&_dev);
        break;

    default:
        result = bma4_init(&_dev);
        _dev.resolution = BMA4_16_BIT_RESOLUTION;
        _dev.feature_len = 0;
        _dev.remap.x_axis = BMA4_MAP_X_AXIS;
        _dev.remap.y_axis = BMA4_MAP_Y_AXIS;
        _dev.remap.z_axis = BMA4_MAP_Z_AXIS;
        _dev.remap.x_axis_sign = BMA4_MAP_POSITIVE;
        _dev.remap.y_axis_sign = BMA4_MAP_POSITIVE;
        _dev.remap.z_axis_sign = BMA4_MAP_POSITIVE;
        break;
    }

    if (bosch(result) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    sfTkError_t rc = loadFeatureConfig();
    if (rc != ksfTkErrOk)
    {
        // The engine is not running, so no H/MM call may reach Bosch as if it were.
        _featureSetId = BMA456_FEATURE_SET_NONE;
        return rc;
    }

    // Match the BMA400 library: leave the part producing data.
    rc = enableAccel(true);
    if (rc != ksfTkErrOk)
        return rc;

    // Cache the range from the device rather than assuming the reset value.
    sfe_bma456_odr_t odr;
    sfe_bma456_range_t range;
    sfe_bma456_bandwidth_t bandwidth;
    sfe_bma456_perf_mode_t perfMode;

    return getAccelConfig(odr, range, bandwidth, perfMode);
}

sfTkError_t sfDevBMA456::loadFeatureConfig(void)
{
    if (_featureSet == nullptr || _featureSet->config == nullptr || _featureSet->configSize == nullptr)
        return ksfTkErrOk; // Nothing to load - the NONE set.

    const uint8_t *blob = _featureSet->config;
    const uint16_t size = *_featureSet->configSize;

    // This mirrors bma4_write_config_file() step for step, except that the blob is copied out of
    // (possibly program) memory a chunk at a time instead of being handed to the bus as a pointer.
    if (bosch(bma4_set_advance_power_save(BMA4_DISABLE, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    uint8_t initCtrl = 0x00;
    if (bosch(bma4_write_regs(kRegInitCtrl, &initCtrl, 1, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    uint8_t chunk[kMaxTransferBytes];

    for (uint16_t index = 0; index < size; index += kMaxTransferBytes)
    {
        uint16_t count = size - index;
        if (count > kMaxTransferBytes)
            count = kMaxTransferBytes;

#if defined(__AVR__)
        memcpy_P(chunk, blob + index, count);
#else
        memcpy(chunk, blob + index, count);
#endif

        // The window address is in 16-bit words: low nibble in 0x5B, the rest in 0x5C.
        uint16_t word = index / 2;
        uint8_t asicLsb = (uint8_t)(word & 0x0F);
        uint8_t asicMsb = (uint8_t)(word >> 4);

        if (bosch(bma4_write_regs(kRegAsicLsb, &asicLsb, 1, &_dev)) != ksfTkErrOk)
            return ksfBMA456ErrBosch;

        if (bosch(bma4_write_regs(kRegAsicMsb, &asicMsb, 1, &_dev)) != ksfTkErrOk)
            return ksfBMA456ErrBosch;

        // Straight to the bus: bma4_write_regs() would treat 0x5E as a feature-page access and
        // apply its length checks against the (much shorter) feature page.
        sfTkError_t rc = busWrite(kRegFeatureConfig, chunk, count);
        if (rc != ksfTkErrOk)
            return rc;

        delayUs(kWriteDelayUs);
    }

    initCtrl = 0x01;
    if (bosch(bma4_write_regs(kRegInitCtrl, &initCtrl, 1, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    delayUs(kConfigBootDelayUs);

    // Bosch checks once at 150 ms. Polling a little longer costs nothing on success and rescues
    // a slow boot.
    bool initialized = false;
    for (uint8_t i = 0; i < kConfigPollRetries && !initialized; i++)
    {
        uint8_t status = 0;
        if (bma4_read_regs(BMA4_INTERNAL_STAT, &status, 1, &_dev) == BMA4_OK &&
            (status & kInternalStatusMessage) == kInternalStatusInitOk)
        {
            initialized = true;
            break;
        }
        delayUs(kConfigPollDelayUs);
    }

    if (!initialized)
    {
        _lastBoschResult = BMA4_E_CONFIG_STREAM_ERROR;
        return ksfBMA456ErrConfigLoad;
    }

    if (bosch(bma4_set_advance_power_save(BMA4_ENABLE, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    // Bosch's loader finishes by recording where the feature page starts; its chunked feature
    // accesses depend on it. That helper is static in bma4.c, so do the same here.
    uint8_t asicLsb = 0, asicMsb = 0;
    if (bosch(bma4_read_regs(kRegAsicLsb, &asicLsb, 1, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;
    if (bosch(bma4_read_regs(kRegAsicMsb, &asicMsb, 1, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    _dev.asic_data.asic_lsb = asicLsb & 0x0F;
    _dev.asic_data.asic_msb = asicMsb;

    return ksfTkErrOk;
}

bool sfDevBMA456::isConnected(void)
{
    uint8_t chipId = 0;

    if (getChipID(chipId) != ksfTkErrOk)
        return false;

    return chipId == SFE_BMA456_CHIP_ID;
}

sfTkError_t sfDevBMA456::getChipID(uint8_t &chipId)
{
    chipId = 0;

    // On SPI the Bosch dummy-byte handling is needed, and bma4_read_regs() provides it. It only
    // needs the interface fields, which setCommunicationBus() has filled in.
    if (_dev.intf == BMA4_SPI_INTF)
        _dev.dummy_byte = 1;

    return bosch(bma4_read_regs(BMA4_CHIP_ID_ADDR, &chipId, 1, &_dev));
}

sfTkError_t sfDevBMA456::softReset(void)
{
    if (_theBus == nullptr || _featureSet == nullptr)
        return ksfTkErrBusNotInit;

    return initialize();
}

sfTkError_t sfDevBMA456::getConfigId(uint16_t &configId)
{
    configId = 0;

    switch (_featureSetId)
    {
    case BMA456_FEATURE_SET_H:
        return bosch(bma456h_get_config_id(&configId, &_dev));
    case BMA456_FEATURE_SET_MM:
        return bosch(bma456mm_get_config_id(&configId, &_dev));
    default:
        return ksfBMA456ErrFeatureUnavailable;
    }
}

sfTkError_t sfDevBMA456::getInternalStatus(uint8_t &status)
{
    status = 0;
    return bosch(bma4_read_regs(BMA4_INTERNAL_STAT, &status, 1, &_dev));
}

sfTkError_t sfDevBMA456::getErrorStatus(sfe_bma456_error_status_t &status)
{
    struct bma4_err_reg err;
    memset(&err, 0, sizeof(err));

    sfTkError_t rc = bosch(bma4_get_error_status(&err, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    status.fatalError = err.fatal_err != 0;
    status.commandError = err.cmd_err != 0;
    status.errorCode = err.err_code;
    status.accelConfError = err.err_code == 0x01;
    status.fifoError = err.fifo_err != 0;

    return ksfTkErrOk;
}

// ======================= Accelerometer Configuration =======================

sfTkError_t sfDevBMA456::setAccelConfig(sfe_bma456_odr_t odr, sfe_bma456_range_t range,
                                        sfe_bma456_bandwidth_t bandwidth, sfe_bma456_perf_mode_t perfMode)
{
    struct bma4_accel_config config;
    config.odr = (uint8_t)odr;
    config.range = (uint8_t)range;
    config.bandwidth = (uint8_t)bandwidth;
    config.perf_mode = (uint8_t)perfMode;

    sfTkError_t rc = bosch(bma4_set_accel_config(&config, &_dev));
    if (rc == ksfTkErrOk)
        _range = range;

    return rc;
}

sfTkError_t sfDevBMA456::getAccelConfig(sfe_bma456_odr_t &odr, sfe_bma456_range_t &range,
                                        sfe_bma456_bandwidth_t &bandwidth, sfe_bma456_perf_mode_t &perfMode)
{
    struct bma4_accel_config config;
    memset(&config, 0, sizeof(config));

    sfTkError_t rc = bosch(bma4_get_accel_config(&config, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    odr = (sfe_bma456_odr_t)config.odr;
    range = (sfe_bma456_range_t)(config.range & 0x03);
    bandwidth = (sfe_bma456_bandwidth_t)(config.bandwidth & 0x07);
    perfMode = (sfe_bma456_perf_mode_t)(config.perf_mode & 0x01);

    _range = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::enableAccel(bool enable)
{
    return bosch(bma4_set_accel_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &_dev));
}

sfTkError_t sfDevBMA456::setAccelODR(sfe_bma456_odr_t odr)
{
    sfe_bma456_odr_t oldOdr;
    sfe_bma456_range_t range;
    sfe_bma456_bandwidth_t bandwidth;
    sfe_bma456_perf_mode_t perfMode;

    sfTkError_t rc = getAccelConfig(oldOdr, range, bandwidth, perfMode);
    if (rc != ksfTkErrOk)
        return rc;

    return setAccelConfig(odr, range, bandwidth, perfMode);
}

sfTkError_t sfDevBMA456::setAccelRange(sfe_bma456_range_t range)
{
    sfe_bma456_odr_t odr;
    sfe_bma456_range_t oldRange;
    sfe_bma456_bandwidth_t bandwidth;
    sfe_bma456_perf_mode_t perfMode;

    sfTkError_t rc = getAccelConfig(odr, oldRange, bandwidth, perfMode);
    if (rc != ksfTkErrOk)
        return rc;

    return setAccelConfig(odr, range, bandwidth, perfMode);
}

sfTkError_t sfDevBMA456::setAccelFilter(sfe_bma456_bandwidth_t bandwidth, sfe_bma456_perf_mode_t perfMode)
{
    sfe_bma456_odr_t odr;
    sfe_bma456_range_t range;
    sfe_bma456_bandwidth_t oldBandwidth;
    sfe_bma456_perf_mode_t oldPerfMode;

    sfTkError_t rc = getAccelConfig(odr, range, oldBandwidth, oldPerfMode);
    if (rc != ksfTkErrOk)
        return rc;

    return setAccelConfig(odr, range, bandwidth, perfMode);
}

sfTkError_t sfDevBMA456::setAdvancedPowerSave(bool enable)
{
    return bosch(bma4_set_advance_power_save(enable ? BMA4_ENABLE : BMA4_DISABLE, &_dev));
}

sfTkError_t sfDevBMA456::getAdvancedPowerSave(bool &enabled)
{
    uint8_t value = 0;
    sfTkError_t rc = bosch(bma4_get_advance_power_save(&value, &_dev));
    enabled = value != 0;
    return rc;
}

// ============================== Measurement ================================

void sfDevBMA456::applyRemap(sfe_bma456_raw_data_t &data)
{
    // Same arithmetic as Bosch's get_remapped_data(): reported axis N = sign * physical[map[N]].
    const int16_t physical[3] = {data.x, data.y, data.z};

    const struct bma4_axes_remap &map = _dev.remap;

    // Guard the index - a zeroed map before begin() is still X/Y/Z.
    int16_t x = physical[map.x_axis < 3 ? map.x_axis : 0];
    int16_t y = physical[map.y_axis < 3 ? map.y_axis : 1];
    int16_t z = physical[map.z_axis < 3 ? map.z_axis : 2];

    data.x = (map.x_axis_sign == BMA4_MAP_NEGATIVE) ? (int16_t)-x : x;
    data.y = (map.y_axis_sign == BMA4_MAP_NEGATIVE) ? (int16_t)-y : y;
    data.z = (map.z_axis_sign == BMA4_MAP_NEGATIVE) ? (int16_t)-z : z;
}

sfTkError_t sfDevBMA456::getSensorData(sfe_bma456_sensor_data_t &data)
{
    // DATA_8 (0x12) through SENSORTIME_2 (0x1A): six bytes of acceleration then three of time.
    uint8_t buffer[9] = {0};

    sfTkError_t rc = bosch(bma4_read_regs(BMA4_DATA_8_ADDR, buffer, sizeof(buffer), &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    data.accelRaw.x = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
    data.accelRaw.y = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
    data.accelRaw.z = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
    applyRemap(data.accelRaw);

    data.accel.x = accelRawToG(data.accelRaw.x);
    data.accel.y = accelRawToG(data.accelRaw.y);
    data.accel.z = accelRawToG(data.accelRaw.z);

    data.sensorTime = (uint32_t)buffer[6] | ((uint32_t)buffer[7] << 8) | ((uint32_t)buffer[8] << 16);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::getAccelDataRaw(sfe_bma456_raw_data_t &data)
{
    uint8_t buffer[6] = {0};

    sfTkError_t rc = bosch(bma4_read_regs(BMA4_DATA_8_ADDR, buffer, sizeof(buffer), &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    data.x = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
    data.y = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
    data.z = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
    applyRemap(data);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::getAccelData(sfe_bma456_data_t &data)
{
    sfe_bma456_raw_data_t raw;

    sfTkError_t rc = getAccelDataRaw(raw);
    if (rc != ksfTkErrOk)
        return rc;

    data.x = accelRawToG(raw.x);
    data.y = accelRawToG(raw.y);
    data.z = accelRawToG(raw.z);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::getTemperature(float &celsius)
{
    uint8_t raw = 0;

    sfTkError_t rc = bosch(bma4_read_regs(kRegTemperature, &raw, 1, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    if (raw == kTempInvalid)
        return ksfBMA456ErrNoData;

    // Signed. This cast is the whole fix for the Bosch below-23-degC bug.
    celsius = (float)((int8_t)raw) + (float)kTempOffsetC;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::getSensorTime(uint32_t &sensorTime)
{
    sensorTime = 0;
    return bosch(bma4_get_sensor_time(&sensorTime, &_dev));
}

sfTkError_t sfDevBMA456::getDataReady(bool &ready)
{
    uint8_t status = 0;

    sfTkError_t rc = bosch(bma4_read_regs(kRegStatus, &status, 1, &_dev));
    ready = (status & kStatusDrdyAcc) != 0;

    return rc;
}

// ============================ Unit Conversion ==============================

float sfDevBMA456::accelRawToG(int16_t raw)
{
    return (float)raw / accelSensitivity(_range);
}

float sfDevBMA456::accelSensitivity(sfe_bma456_range_t range)
{
    // 16-bit output across 2, 4, 8 or 16 g: 32768 / 2 g = 16384 LSB/g, halving per step.
    return (float)(16384u >> ((uint8_t)range & 0x03));
}

uint32_t sfDevBMA456::sensorTimeToMicros(uint32_t ticks)
{
    // 39.0625 us = 39 + 1/16 us. Kept in 32-bit integers so an AVR does not pull in 64-bit math:
    // a full 24-bit count times 39 is still below 2^30.
    return ticks * 39u + ticks / 16u;
}

// =============================== Interrupts ================================

sfTkError_t sfDevBMA456::setInterruptPinConfig(sfe_bma456_int_pin_t pin, const sfe_bma456_int_pin_config_t &config)
{
    struct bma4_int_pin_config pinConfig;
    pinConfig.edge_ctrl = BMA4_LEVEL_TRIGGER; // Input-mode setting; unused as an output.
    pinConfig.lvl = config.activeHigh ? BMA4_ACTIVE_HIGH : BMA4_ACTIVE_LOW;
    pinConfig.od = config.openDrain ? BMA4_OPEN_DRAIN : BMA4_PUSH_PULL;
    pinConfig.output_en = config.enabled ? BMA4_OUTPUT_ENABLE : BMA4_OUTPUT_DISABLE;
    pinConfig.input_en = BMA4_INPUT_DISABLE;

    return bosch(bma4_set_int_pin_config(&pinConfig, (uint8_t)pin, &_dev));
}

sfTkError_t sfDevBMA456::setInterruptLatch(bool latched)
{
    return bosch(bma4_set_interrupt_mode(latched ? BMA4_LATCH_MODE : BMA4_NON_LATCH_MODE, &_dev));
}

sfTkError_t sfDevBMA456::sourceToBoschMap(sfe_bma456_int_source_t source, uint16_t &map)
{
    map = 0;

    // Data interrupts live in INT_MAP_DATA and are the same for every configuration.
    switch (source)
    {
    case BMA456_INT_SRC_DATA_READY:
        // BMA4_DATA_RDY_INT, not BMA4_ACCEL_DATA_RDY_INT. The latter (0x8000) is the drdy_acc bit
        // in INT_STATUS_1; bma4_map_interrupt() shifts the map word into INT_MAP_DATA, where
        // 0x0400 lands on drdy_int1 (bit 2) or drdy_int2 (bit 6) and 0x8000 lands on nothing.
        map = BMA4_DATA_RDY_INT;
        return ksfTkErrOk;
    case BMA456_INT_SRC_FIFO_WATERMARK:
        map = BMA4_FIFO_WM_INT;
        return ksfTkErrOk;
    case BMA456_INT_SRC_FIFO_FULL:
        map = BMA4_FIFO_FULL_INT;
        return ksfTkErrOk;
    default:
        break;
    }

    // Feature interrupts live in INT1_MAP / INT2_MAP, and their bit positions depend on the blob.
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        switch (source)
        {
        case BMA456_INT_SRC_ANY_MOTION:
            map = BMA456H_ANY_MOT_INT;
            break;
        case BMA456_INT_SRC_NO_MOTION:
            map = BMA456H_NO_MOT_INT;
            break;
        case BMA456_INT_SRC_TAP:
            map = BMA456H_TAP_OUT_INT;
            break;
        case BMA456_INT_SRC_ERROR:
            map = BMA456H_ERROR_INT;
            break;
        case BMA456_INT_SRC_STEP_COUNTER:
            map = BMA456H_STEP_CNTR_INT;
            break;
        case BMA456_INT_SRC_ACTIVITY:
            map = BMA456H_ACTIVITY_INT;
            break;
        default:
            return ksfBMA456ErrFeatureUnavailable;
        }
        return ksfTkErrOk;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        switch (source)
        {
        case BMA456_INT_SRC_ANY_MOTION:
            map = BMA456MM_ANY_MOT_INT;
            break;
        case BMA456_INT_SRC_NO_MOTION:
            map = BMA456MM_NO_MOT_INT;
            break;
        case BMA456_INT_SRC_TAP:
            map = BMA456MM_TAP_OUT_INT;
            break;
        case BMA456_INT_SRC_ERROR:
            map = BMA456MM_ERROR_INT;
            break;
        case BMA456_INT_SRC_ORIENTATION:
            map = BMA456MM_ORIENT_INT;
            break;
        case BMA456_INT_SRC_LOW_G:
            map = BMA456MM_LOW_G_INT;
            break;
        case BMA456_INT_SRC_HIGH_G:
            map = BMA456MM_HIGH_G_INT;
            break;
        case BMA456_INT_SRC_SIG_MOTION:
            map = BMA456MM_SIG_MOT_INT;
            break;
        default:
            return ksfBMA456ErrFeatureUnavailable;
        }
        return ksfTkErrOk;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

uint16_t sfDevBMA456::boschStatusToLibrary(uint16_t boschStatus)
{
    uint16_t status = 0;

    // INT_STATUS_1 (high byte) - data interrupts, configuration independent.
    if (boschStatus & BMA4_FIFO_FULL_INT)
        status |= BMA456_INT_STATUS_FIFO_FULL;
    if (boschStatus & BMA4_FIFO_WM_INT)
        status |= BMA456_INT_STATUS_FIFO_WATERMARK;
    if (boschStatus & BMA4_ACCEL_DATA_RDY_INT)
        status |= BMA456_INT_STATUS_DATA_READY;

    // INT_STATUS_0 (low byte) - feature interrupts, laid out per configuration.
    uint8_t features = (uint8_t)(boschStatus & 0x00FF);

    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        if (features & BMA456H_ANY_MOT_INT)
            status |= BMA456_INT_STATUS_ANY_MOTION;
        if (features & BMA456H_NO_MOT_INT)
            status |= BMA456_INT_STATUS_NO_MOTION;
        if (features & BMA456H_TAP_OUT_INT)
            status |= BMA456_INT_STATUS_TAP;
        if (features & BMA456H_STEP_CNTR_INT)
            status |= BMA456_INT_STATUS_STEP_COUNTER;
        if (features & BMA456H_ACTIVITY_INT)
            status |= BMA456_INT_STATUS_ACTIVITY;
        if (features & BMA456H_ERROR_INT)
            status |= BMA456_INT_STATUS_ERROR;
    }
    else if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        if (features & BMA456MM_ANY_MOT_INT)
            status |= BMA456_INT_STATUS_ANY_MOTION;
        if (features & BMA456MM_NO_MOT_INT)
            status |= BMA456_INT_STATUS_NO_MOTION;
        if (features & BMA456MM_TAP_OUT_INT)
            status |= BMA456_INT_STATUS_TAP;
        if (features & BMA456MM_ORIENT_INT)
            status |= BMA456_INT_STATUS_ORIENTATION;
        if (features & BMA456MM_LOW_G_INT)
            status |= BMA456_INT_STATUS_LOW_G;
        if (features & BMA456MM_HIGH_G_INT)
            status |= BMA456_INT_STATUS_HIGH_G;
        if (features & BMA456MM_SIG_MOT_INT)
            status |= BMA456_INT_STATUS_SIG_MOTION;
        if (features & BMA456MM_ERROR_INT)
            status |= BMA456_INT_STATUS_ERROR;
    }

    return status;
}

sfTkError_t sfDevBMA456::mapInterrupt(sfe_bma456_int_source_t source, sfe_bma456_int_pin_t pin, bool enable)
{
    uint16_t map = 0;

    sfTkError_t rc = sourceToBoschMap(source, map);
    if (rc != ksfTkErrOk)
        return rc;

    return bosch(bma4_map_interrupt((uint8_t)pin, map, enable ? BMA4_ENABLE : BMA4_DISABLE, &_dev));
}

sfTkError_t sfDevBMA456::getInterruptStatus(uint16_t &status)
{
    uint16_t raw = 0;
    status = 0;

    sfTkError_t rc = bosch(bma4_read_int_status(&raw, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    status = boschStatusToLibrary(raw);
    return ksfTkErrOk;
}

// ================================== FIFO ===================================

sfTkError_t sfDevBMA456::setFifoConfig(bool enable, bool stopOnFull)
{
    // Headerless, accelerometer only: no header, no sensor-time frame, no auxiliary data.
    uint8_t off = BMA4_FIFO_HEADER | BMA4_FIFO_TIME | BMA4_FIFO_MAG;
    if (!stopOnFull)
        off |= BMA4_FIFO_STOP_ON_FULL;
    if (!enable)
        off |= BMA4_FIFO_ACCEL;

    sfTkError_t rc = bosch(bma4_set_fifo_config(off, BMA4_DISABLE, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    uint8_t on = 0;
    if (enable)
        on |= BMA4_FIFO_ACCEL;
    if (stopOnFull)
        on |= BMA4_FIFO_STOP_ON_FULL;

    if (on == 0)
        return ksfTkErrOk;

    return bosch(bma4_set_fifo_config(on, BMA4_ENABLE, &_dev));
}

sfTkError_t sfDevBMA456::setFifoDownsampling(uint8_t downsampling, bool filtered)
{
    if (downsampling > 7)
        return ksfTkErrInvalidParam;

    sfTkError_t rc = bosch(bma4_set_fifo_down_accel(downsampling, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    return bosch(bma4_set_accel_fifo_filter_data(filtered ? 1 : 0, &_dev));
}

sfTkError_t sfDevBMA456::setFifoWatermark(uint16_t bytes)
{
    if (bytes > kFifoSizeBytes)
        return ksfTkErrInvalidParam;

    return bosch(bma4_set_fifo_wm(bytes, &_dev));
}

sfTkError_t sfDevBMA456::getFifoWatermark(uint16_t &bytes)
{
    bytes = 0;
    return bosch(bma4_get_fifo_wm(&bytes, &_dev));
}

sfTkError_t sfDevBMA456::getFifoLength(uint16_t &bytes)
{
    bytes = 0;
    return bosch(bma4_get_fifo_length(&bytes, &_dev));
}

sfTkError_t sfDevBMA456::flushFifo(void)
{
    return bosch(bma4_set_command_register(BMA4_FIFO_FLUSH, &_dev));
}

sfTkError_t sfDevBMA456::readFifoData(sfe_bma456_raw_data_t *frames, uint16_t maxFrames, uint16_t &numFrames)
{
    numFrames = 0;

    if (frames == nullptr)
        return ksfTkErrInvalidParam;

    uint16_t length = 0;
    sfTkError_t rc = getFifoLength(length);
    if (rc != ksfTkErrOk)
        return rc;

    uint16_t available = length / kFifoFrameBytes;
    uint16_t wanted = available < maxFrames ? available : maxFrames;

    // Whole frames per transaction, so a burst boundary never falls inside a frame.
    const uint8_t framesPerChunk = kMaxTransferBytes / kFifoFrameBytes;
    uint8_t buffer[framesPerChunk * kFifoFrameBytes];

    while (numFrames < wanted)
    {
        uint16_t count = wanted - numFrames;
        if (count > framesPerChunk)
            count = framesPerChunk;

        rc = bosch(bma4_read_regs(BMA4_FIFO_DATA_ADDR, buffer, count * kFifoFrameBytes, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        for (uint16_t i = 0; i < count; i++)
        {
            const uint8_t *p = &buffer[i * kFifoFrameBytes];

            sfe_bma456_raw_data_t sample;
            sample.x = (int16_t)((uint16_t)p[1] << 8 | p[0]);
            sample.y = (int16_t)((uint16_t)p[3] << 8 | p[2]);
            sample.z = (int16_t)((uint16_t)p[5] << 8 | p[4]);

            // An over-read returns 0x8000 in every word. The length check above makes that
            // unlikely, but a frame of it is not a sample and should not be reported as one.
            if (sample.x == kFifoEmptyWord && sample.y == kFifoEmptyWord && sample.z == kFifoEmptyWord)
                return ksfTkErrOk;

            applyRemap(sample);
            frames[numFrames++] = sample;
        }
    }

    return ksfTkErrOk;
}

// ======================= Feature Engine - Common ===========================

sfTkError_t sfDevBMA456::enableFeature(uint16_t features, bool enable)
{
    uint8_t onOff = enable ? TRUE : FALSE;

    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        if (features & kFeatMMOnlyMask)
            return ksfBMA456ErrFeatureUnavailable;

        // Motion axis bits are laid out identically in BMA456H's feature-enable word.
        uint16_t bits = features & kFeatMotionMask;

        if (features & BMA456_FEAT_SINGLE_TAP)
            bits |= BMA456H_SINGLE_TAP_EN;
        if (features & BMA456_FEAT_DOUBLE_TAP)
            bits |= BMA456H_DOUBLE_TAP_EN;
        if (features & BMA456_FEAT_TRIPLE_TAP)
            bits |= BMA456H_TRIPLE_TAP_EN;
        if (features & BMA456_FEAT_STEP_DETECTOR)
            bits |= BMA456H_STEP_DETECTOR_EN;
        if (features & BMA456_FEAT_STEP_COUNTER)
            bits |= BMA456H_STEP_COUNTER_EN;
        if (features & BMA456_FEAT_STEP_ACTIVITY)
            bits |= BMA456H_STEP_ACTIVITY_EN;

        if (bits == 0)
            return ksfTkErrOk;

        return bosch(bma456h_feature_enable(bits, onOff, &_dev));
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        if (features & kFeatHOnlyMask)
            return ksfBMA456ErrFeatureUnavailable;

        uint8_t bits = 0;
        if (features & BMA456_FEAT_SINGLE_TAP)
            bits |= BMA456MM_SINGLE_TAP;
        if (features & BMA456_FEAT_DOUBLE_TAP)
            bits |= BMA456MM_DOUBLE_TAP;
        if (features & BMA456_FEAT_TRIPLE_TAP)
            bits |= BMA456MM_TRIPLE_TAP;
        if (features & BMA456_FEAT_ORIENTATION)
            bits |= BMA456MM_ORIENT;
        if (features & BMA456_FEAT_LOW_G)
            bits |= BMA456MM_LOW_G;
        if (features & BMA456_FEAT_HIGH_G)
            bits |= BMA456MM_HIGH_G;
        if (features & BMA456_FEAT_SIG_MOTION)
            bits |= BMA456MM_SIG_MOTION;

        if (bits != 0)
        {
            sfTkError_t rc = bosch(bma456mm_feature_enable(bits, onOff, &_dev));
            if (rc != ksfTkErrOk)
                return rc;
        }

        // Under BMA456MM any-motion and no-motion have no enable bit: they run whenever their
        // configuration has at least one axis enabled. Edit the axis field in place.
        uint8_t anyAxes = (uint8_t)((features & BMA456_FEAT_ANY_MOTION) >> 3); // x=1, y=2, z=4
        uint8_t noAxes = (uint8_t)(features & BMA456_FEAT_NO_MOTION);

        if (anyAxes != 0)
        {
            struct bma456mm_any_no_mot_config config;
            memset(&config, 0, sizeof(config));

            sfTkError_t rc = bosch(bma456mm_get_any_mot_config(&config, &_dev));
            if (rc != ksfTkErrOk)
                return rc;

            config.axes_en = enable ? (uint8_t)(config.axes_en | anyAxes) : (uint8_t)(config.axes_en & ~anyAxes);

            rc = bosch(bma456mm_set_any_mot_config(&config, &_dev));
            if (rc != ksfTkErrOk)
                return rc;
        }

        if (noAxes != 0)
        {
            struct bma456mm_any_no_mot_config config;
            memset(&config, 0, sizeof(config));

            sfTkError_t rc = bosch(bma456mm_get_no_mot_config(&config, &_dev));
            if (rc != ksfTkErrOk)
                return rc;

            config.axes_en = enable ? (uint8_t)(config.axes_en | noAxes) : (uint8_t)(config.axes_en & ~noAxes);

            rc = bosch(bma456mm_set_no_mot_config(&config, &_dev));
            if (rc != ksfTkErrOk)
                return rc;
        }

        return ksfTkErrOk;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::setAnyMotionConfig(const sfe_bma456_motion_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456h_get_any_mot_config(&c, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        c.threshold = config.threshold;
        c.duration = config.duration;
        c.intr_bhvr = config.intrBehavior;
        c.slope = config.slope;
        return bosch(bma456h_set_any_mot_config(&c, &_dev));
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        // Read first: the MM structure also carries the axis enables, which must survive.
        struct bma456mm_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456mm_get_any_mot_config(&c, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        c.threshold = config.threshold;
        c.duration = config.duration;
        c.intr_bhvr = config.intrBehavior;
        c.slope = config.slope;
        return bosch(bma456mm_set_any_mot_config(&c, &_dev));
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::getAnyMotionConfig(sfe_bma456_motion_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456h_get_any_mot_config(&c, &_dev));
        config.threshold = c.threshold;
        config.duration = c.duration;
        config.intrBehavior = c.intr_bhvr;
        config.slope = c.slope;
        return rc;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456mm_get_any_mot_config(&c, &_dev));
        config.threshold = c.threshold;
        config.duration = c.duration;
        config.intrBehavior = c.intr_bhvr;
        config.slope = c.slope;
        return rc;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::setNoMotionConfig(const sfe_bma456_motion_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456h_get_no_mot_config(&c, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        c.threshold = config.threshold;
        c.duration = config.duration;
        c.intr_bhvr = config.intrBehavior;
        c.slope = config.slope;
        return bosch(bma456h_set_no_mot_config(&c, &_dev));
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456mm_get_no_mot_config(&c, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        c.threshold = config.threshold;
        c.duration = config.duration;
        c.intr_bhvr = config.intrBehavior;
        c.slope = config.slope;
        return bosch(bma456mm_set_no_mot_config(&c, &_dev));
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::getNoMotionConfig(sfe_bma456_motion_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456h_get_no_mot_config(&c, &_dev));
        config.threshold = c.threshold;
        config.duration = c.duration;
        config.intrBehavior = c.intr_bhvr;
        config.slope = c.slope;
        return rc;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_any_no_mot_config c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456mm_get_no_mot_config(&c, &_dev));
        config.threshold = c.threshold;
        config.duration = c.duration;
        config.intrBehavior = c.intr_bhvr;
        config.slope = c.slope;
        return rc;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::setTapConfig(const sfe_bma456_tap_config_t &config)
{
    // Both configurations use the same twelve-word structure. Read it whole so the reserved words
    // go back exactly as they came out.
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_multitap_settings s;
        memset(&s, 0, sizeof(s));
        sfTkError_t rc = bosch(bma456h_tap_get_parameter(&s, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        s.tap_sens_thres = config.sensitivityThreshold;
        s.max_gest_dur = config.maxGestureDuration;
        s.tap_shock_dur = config.shockDuration;
        s.quite_time_after_gest = config.quietTimeAfterGesture;
        s.wait_for_timeout = config.waitForTimeout;
        s.axis_sel = config.axisSelect;
        return bosch(bma456h_tap_set_parameter(&s, &_dev));
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_multitap_settings s;
        memset(&s, 0, sizeof(s));
        sfTkError_t rc = bosch(bma456mm_tap_get_parameter(&s, &_dev));
        if (rc != ksfTkErrOk)
            return rc;

        s.tap_sens_thres = config.sensitivityThreshold;
        s.max_gest_dur = config.maxGestureDuration;
        s.tap_shock_dur = config.shockDuration;
        s.quite_time_after_gest = config.quietTimeAfterGesture;
        s.wait_for_timeout = config.waitForTimeout;
        s.axis_sel = config.axisSelect;
        return bosch(bma456mm_tap_set_parameter(&s, &_dev));
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::getTapConfig(sfe_bma456_tap_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_multitap_settings s;
        memset(&s, 0, sizeof(s));
        sfTkError_t rc = bosch(bma456h_tap_get_parameter(&s, &_dev));
        config.sensitivityThreshold = s.tap_sens_thres;
        config.maxGestureDuration = s.max_gest_dur;
        config.shockDuration = s.tap_shock_dur;
        config.quietTimeAfterGesture = s.quite_time_after_gest;
        config.waitForTimeout = s.wait_for_timeout;
        config.axisSelect = s.axis_sel;
        return rc;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_multitap_settings s;
        memset(&s, 0, sizeof(s));
        sfTkError_t rc = bosch(bma456mm_tap_get_parameter(&s, &_dev));
        config.sensitivityThreshold = s.tap_sens_thres;
        config.maxGestureDuration = s.max_gest_dur;
        config.shockDuration = s.tap_shock_dur;
        config.quietTimeAfterGesture = s.quite_time_after_gest;
        config.waitForTimeout = s.wait_for_timeout;
        config.axisSelect = s.axis_sel;
        return rc;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::getTapEvent(sfe_bma456_tap_event_t &event)
{
    event.singleTap = event.doubleTap = event.tripleTap = false;

    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_out_state out;
        memset(&out, 0, sizeof(out));
        sfTkError_t rc = bosch(bma456h_output_state(&out, &_dev));
        event.singleTap = out.single_tap != 0;
        event.doubleTap = out.double_tap != 0;
        event.tripleTap = out.triple_tap != 0;
        return rc;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_out_state out;
        memset(&out, 0, sizeof(out));
        sfTkError_t rc = bosch(bma456mm_output_state(&out, &_dev));
        event.singleTap = out.s_tap != 0;
        event.doubleTap = out.d_tap != 0;
        event.tripleTap = out.t_tap != 0;
        return rc;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::setAutoLowPowerConfig(const sfe_bma456_auto_low_power_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        // The BMA456H time-out field is 11 bits; larger values would be silently truncated.
        if (config.timeoutDuration > kHTimeoutMax)
            return ksfTkErrInvalidParam;

        struct bma456h_auto_low_power c;
        c.no_motion = config.onNoMotion ? 1 : 0;
        c.time_out = config.onTimeout ? 1 : 0;
        c.time_out_dur = config.timeoutDuration;
        c.lp_odr = (uint8_t)config.lowPowerODR;
        c.pwr_mgt = config.featureOptimizedConfig ? 1 : 0;
        return bosch(bma456h_set_auto_low_power_config(&c, &_dev));
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        // Under BMA456MM the time-out is 10 bits and the auto-low-power ENABLE bit sits directly
        // above it (bit 12 of the word at feature offset 0x2C). Bosch's setter masks the duration
        // with 0x1FFC, which reaches into that bit: any write clears the enable (or sets it, for a
        // duration of 1024 or more). Reject the durations that would set it, and put the enable
        // back afterwards.
        if (config.timeoutDuration > kMMTimeoutMax)
            return ksfTkErrInvalidParam;

        struct bma456mm_auto_low_power c;
        c.no_motion = config.onNoMotion ? 1 : 0;
        c.time_out = config.onTimeout ? 1 : 0;
        c.time_out_dur = config.timeoutDuration;
        c.lp_odr = (uint8_t)config.lowPowerODR;
        c.pwr_mgt = config.featureOptimizedConfig ? 1 : 0;

        sfTkError_t rc = bosch(bma456mm_set_auto_low_power_config(&c, &_dev));
        if (rc != ksfTkErrOk || !_autoLowPowerEnabled)
            return rc;

        return bosch(bma456mm_feature_enable(BMA456MM_AUTO_LOW_POWER, TRUE, &_dev));
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::getAutoLowPowerConfig(sfe_bma456_auto_low_power_config_t &config)
{
    if (_featureSetId == BMA456_FEATURE_SET_H)
    {
        struct bma456h_auto_low_power c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456h_get_auto_low_power_config(&c, &_dev));
        config.onNoMotion = c.no_motion != 0;
        config.onTimeout = c.time_out != 0;
        config.timeoutDuration = c.time_out_dur;
        config.lowPowerODR = (sfe_bma456_lp_odr_t)(c.lp_odr & 0x03);
        config.featureOptimizedConfig = c.pwr_mgt != 0;
        return rc;
    }

    if (_featureSetId == BMA456_FEATURE_SET_MM)
    {
        struct bma456mm_auto_low_power c;
        memset(&c, 0, sizeof(c));
        sfTkError_t rc = bosch(bma456mm_get_auto_low_power_config(&c, &_dev));
        config.onNoMotion = c.no_motion != 0;
        config.onTimeout = c.time_out != 0;
        config.timeoutDuration = c.time_out_dur;
        config.lowPowerODR = (sfe_bma456_lp_odr_t)(c.lp_odr & 0x03);
        config.featureOptimizedConfig = c.pwr_mgt != 0;
        return rc;
    }

    return ksfBMA456ErrFeatureUnavailable;
}

sfTkError_t sfDevBMA456::enableAutoLowPower(bool enable)
{
    uint8_t onOff = enable ? TRUE : FALSE;
    sfTkError_t rc;

    if (_featureSetId == BMA456_FEATURE_SET_H)
        rc = bosch(bma456h_feature_enable(BMA456H_AUTO_LOW_POWER_EN, onOff, &_dev));
    else if (_featureSetId == BMA456_FEATURE_SET_MM)
        rc = bosch(bma456mm_feature_enable(BMA456MM_AUTO_LOW_POWER, onOff, &_dev));
    else
        return ksfBMA456ErrFeatureUnavailable;

    if (rc == ksfTkErrOk)
        _autoLowPowerEnabled = enable; // setAutoLowPowerConfig() needs this under MM

    return rc;
}

sfTkError_t sfDevBMA456::getAutoLowPowerState(bool &inLowPower)
{
    uint8_t state = 0;
    inLowPower = false;

    sfTkError_t rc;
    if (_featureSetId == BMA456_FEATURE_SET_H)
        rc = bosch(bma456h_get_auto_low_power_state(&state, &_dev));
    else if (_featureSetId == BMA456_FEATURE_SET_MM)
        rc = bosch(bma456mm_get_auto_low_power_state(&state, &_dev));
    else
        return ksfBMA456ErrFeatureUnavailable;

    inLowPower = state != 0;
    return rc;
}

sfTkError_t sfDevBMA456::setAxisRemap(const sfe_bma456_axis_remap_t &remap)
{
    // Each physical axis exactly once. Bosch checks the same thing, but the NONE path does not
    // reach Bosch, so check here for both.
    uint8_t used = (uint8_t)((remap.x | remap.y | remap.z) & BMA4_AXIS_MASK);
    uint8_t sum = (uint8_t)((remap.x & BMA4_AXIS_MASK) + (remap.y & BMA4_AXIS_MASK) + (remap.z & BMA4_AXIS_MASK));

    if (used != BMA4_AXIS_MASK || sum != BMA4_AXIS_MASK)
        return ksfTkErrInvalidParam;

    struct bma4_remap boschRemap;
    boschRemap.x = (uint8_t)remap.x;
    boschRemap.y = (uint8_t)remap.y;
    boschRemap.z = (uint8_t)remap.z;

    // Bosch stores the map in _dev.remap (which applyRemap() uses) and writes it into the
    // feature engine.
    if (_featureSetId == BMA456_FEATURE_SET_H)
        return bosch(bma456h_set_remap_axes(&boschRemap, &_dev));

    if (_featureSetId == BMA456_FEATURE_SET_MM)
        return bosch(bma456mm_set_remap_axes(&boschRemap, &_dev));

    // No feature engine: host-side only. Same encoding as Bosch's assign_remap_axis().
    const uint8_t sources[3] = {boschRemap.x, boschRemap.y, boschRemap.z};
    uint8_t *axes[3] = {&_dev.remap.x_axis, &_dev.remap.y_axis, &_dev.remap.z_axis};
    uint8_t *signs[3] = {&_dev.remap.x_axis_sign, &_dev.remap.y_axis_sign, &_dev.remap.z_axis_sign};

    for (uint8_t i = 0; i < 3; i++)
    {
        switch (sources[i] & BMA4_AXIS_MASK)
        {
        case BMA4_X:
            *axes[i] = BMA4_MAP_X_AXIS;
            break;
        case BMA4_Y:
            *axes[i] = BMA4_MAP_Y_AXIS;
            break;
        default:
            *axes[i] = BMA4_MAP_Z_AXIS;
            break;
        }
        *signs[i] = (sources[i] & BMA4_AXIS_SIGN) ? BMA4_MAP_NEGATIVE : BMA4_MAP_POSITIVE;
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMA456::getAxisRemap(sfe_bma456_axis_remap_t &remap)
{
    // The host copy is authoritative for the data path and matches what was written to the engine.
    const uint8_t axisCodes[3] = {BMA4_X, BMA4_Y, BMA4_Z};

    const struct bma4_axes_remap &map = _dev.remap;

    uint8_t x = axisCodes[map.x_axis < 3 ? map.x_axis : 0] | (map.x_axis_sign ? BMA4_AXIS_SIGN : 0);
    uint8_t y = axisCodes[map.y_axis < 3 ? map.y_axis : 1] | (map.y_axis_sign ? BMA4_AXIS_SIGN : 0);
    uint8_t z = axisCodes[map.z_axis < 3 ? map.z_axis : 2] | (map.z_axis_sign ? BMA4_AXIS_SIGN : 0);

    remap.x = (sfe_bma456_axis_t)x;
    remap.y = (sfe_bma456_axis_t)y;
    remap.z = (sfe_bma456_axis_t)z;

    return ksfTkErrOk;
}

// ======================= Feature Engine - BMA456H ==========================

sfTkError_t sfDevBMA456::getStepCount(uint32_t &steps)
{
    steps = 0;

    if (!haveFeatureSet(BMA456_FEATURE_SET_H))
        return ksfBMA456ErrFeatureUnavailable;

    return bosch(bma456h_step_counter_output(&steps, &_dev));
}

sfTkError_t sfDevBMA456::resetStepCount(void)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_H))
        return ksfBMA456ErrFeatureUnavailable;

    return bosch(bma456h_reset_step_counter(&_dev));
}

sfTkError_t sfDevBMA456::setStepCounterWatermark(uint16_t watermark)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_H))
        return ksfBMA456ErrFeatureUnavailable;

    if (watermark > BMA456H_STEP_CNTR_WM_MSK)
        return ksfTkErrInvalidParam;

    return bosch(bma456h_step_counter_set_watermark(watermark, &_dev));
}

sfTkError_t sfDevBMA456::getStepCounterWatermark(uint16_t &watermark)
{
    watermark = 0;

    if (!haveFeatureSet(BMA456_FEATURE_SET_H))
        return ksfBMA456ErrFeatureUnavailable;

    return bosch(bma456h_step_counter_get_watermark(&watermark, &_dev));
}

sfTkError_t sfDevBMA456::getActivity(sfe_bma456_activity_t &activity)
{
    activity = BMA456_ACTIVITY_UNKNOWN;

    if (!haveFeatureSet(BMA456_FEATURE_SET_H))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456h_out_state out;
    memset(&out, 0, sizeof(out));

    sfTkError_t rc = bosch(bma456h_output_state(&out, &_dev));
    activity = (sfe_bma456_activity_t)(out.activity_type & 0x03);

    return rc;
}

// ======================= Feature Engine - BMA456MM =========================

sfTkError_t sfDevBMA456::setOrientationConfig(const sfe_bma456_orientation_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_orientation_config c;
    c.upside_down = config.upsideDown ? 1 : 0;
    c.mode = config.mode;
    c.blocking = config.blocking;
    c.theta = config.theta;
    c.hysteresis = config.hysteresis;

    return bosch(bma456mm_set_orientation_config(&c, &_dev));
}

sfTkError_t sfDevBMA456::getOrientationConfig(sfe_bma456_orientation_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_orientation_config c;
    memset(&c, 0, sizeof(c));

    sfTkError_t rc = bosch(bma456mm_get_orientation_config(&c, &_dev));
    config.upsideDown = c.upside_down != 0;
    config.mode = c.mode;
    config.blocking = c.blocking;
    config.theta = c.theta;
    config.hysteresis = c.hysteresis;

    return rc;
}

sfTkError_t sfDevBMA456::getOrientation(sfe_bma456_orientation_t &orientation, bool &faceDown)
{
    orientation = BMA456_ORIENT_PORTRAIT_UPRIGHT;
    faceDown = false;

    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_out_state out;
    memset(&out, 0, sizeof(out));

    sfTkError_t rc = bosch(bma456mm_output_state(&out, &_dev));
    orientation = (sfe_bma456_orientation_t)(out.orientation_out & 0x03);
    faceDown = out.orientation_faceup_down == BMA456MM_FACE_DOWN;

    return rc;
}

sfTkError_t sfDevBMA456::setLowGConfig(const sfe_bma456_low_g_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_low_g_config c;
    c.threshold = config.threshold;
    c.hysteresis = config.hysteresis;
    c.duration = config.duration;

    return bosch(bma456mm_set_low_g_config(&c, &_dev));
}

sfTkError_t sfDevBMA456::getLowGConfig(sfe_bma456_low_g_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_low_g_config c;
    memset(&c, 0, sizeof(c));

    sfTkError_t rc = bosch(bma456mm_get_low_g_config(&c, &_dev));
    config.threshold = c.threshold;
    config.hysteresis = c.hysteresis;
    config.duration = c.duration;

    return rc;
}

sfTkError_t sfDevBMA456::setHighGConfig(const sfe_bma456_high_g_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_high_g_config c;
    c.threshold = config.threshold;
    c.hysteresis = config.hysteresis;
    c.duration = config.duration;
    c.axes_en = (uint8_t)((config.xEnabled ? BMA456MM_HIGH_G_X_EN : 0) | (config.yEnabled ? BMA456MM_HIGH_G_Y_EN : 0) |
                          (config.zEnabled ? BMA456MM_HIGH_G_Z_EN : 0));

    return bosch(bma456mm_set_high_g_config(&c, &_dev));
}

sfTkError_t sfDevBMA456::getHighGConfig(sfe_bma456_high_g_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_high_g_config c;
    memset(&c, 0, sizeof(c));

    sfTkError_t rc = bosch(bma456mm_get_high_g_config(&c, &_dev));
    config.threshold = c.threshold;
    config.hysteresis = c.hysteresis;
    config.duration = c.duration;
    config.xEnabled = (c.axes_en & BMA456MM_HIGH_G_X_EN) != 0;
    config.yEnabled = (c.axes_en & BMA456MM_HIGH_G_Y_EN) != 0;
    config.zEnabled = (c.axes_en & BMA456MM_HIGH_G_Z_EN) != 0;

    return rc;
}

sfTkError_t sfDevBMA456::getHighGEvent(sfe_bma456_high_g_event_t &event)
{
    event.x = event.y = event.z = event.negative = false;

    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_out_state out;
    memset(&out, 0, sizeof(out));

    sfTkError_t rc = bosch(bma456mm_output_state(&out, &_dev));
    event.x = out.high_g_detect_x != 0;
    event.y = out.high_g_detect_y != 0;
    event.z = out.high_g_detect_z != 0;
    event.negative = out.high_g_detect_sign != 0;

    return rc;
}

sfTkError_t sfDevBMA456::setSigMotionConfig(const sfe_bma456_sig_motion_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_sig_motion_config c;
    c.threshold = config.threshold;
    c.skiptime = config.skipTime;
    c.prooftime = config.proofTime;

    return bosch(bma456mm_set_sig_motion_config(&c, &_dev));
}

sfTkError_t sfDevBMA456::getSigMotionConfig(sfe_bma456_sig_motion_config_t &config)
{
    if (!haveFeatureSet(BMA456_FEATURE_SET_MM))
        return ksfBMA456ErrFeatureUnavailable;

    struct bma456mm_sig_motion_config c;
    memset(&c, 0, sizeof(c));

    sfTkError_t rc = bosch(bma456mm_get_sig_motion_config(&c, &_dev));
    config.threshold = c.threshold;
    config.skipTime = c.skiptime;
    config.proofTime = c.prooftime;

    return rc;
}

// ======================== Self-Test and Calibration ========================

sfTkError_t sfDevBMA456::performSelfTest(bool &passed)
{
    passed = false;

    // Save what the reset at the end of Bosch's routine is about to throw away.
    sfe_bma456_odr_t odr;
    sfe_bma456_range_t range;
    sfe_bma456_bandwidth_t bandwidth;
    sfe_bma456_perf_mode_t perfMode;

    sfTkError_t rc = getAccelConfig(odr, range, bandwidth, perfMode);
    if (rc != ksfTkErrOk)
        return rc;

    uint8_t accelEnabled = 0;
    rc = bosch(bma4_get_accel_enable(&accelEnabled, &_dev));
    if (rc != ksfTkErrOk)
        return rc;

    // Bosch returns the verdict both through the pointer and as the return value: BMA4_OK on a
    // pass, BMA4_SELFTEST_FAIL (+1) on a fail, negative only if the test could not run.
    int8_t verdict = BMA4_SELFTEST_FAIL;
    int8_t result = bma4_perform_accel_selftest(&verdict, &_dev);
    _lastBoschResult = result;

    if (result < 0)
        return ksfBMA456ErrBosch;

    passed = (result == BMA4_OK) && (verdict == BMA4_SELFTEST_PASS);

    // A pass ends in a soft reset; a fail leaves the self-test configuration in place. Either way,
    // a full re-initialization puts the part back into a known state with the feature engine up.
    rc = initialize();
    if (rc != ksfTkErrOk)
        return rc;

    rc = setAccelConfig(odr, range, bandwidth, perfMode);
    if (rc != ksfTkErrOk)
        return rc;

    return enableAccel(accelEnabled != 0);
}

sfTkError_t sfDevBMA456::performAccelFOC(sfe_bma456_foc_axis_t gravityAxis)
{
    // Done here rather than with bma4_perform_accel_foc(), which has three problems for this
    // library: it keeps 128 samples on the stack (768 bytes - most of an Uno's free RAM), it
    // ignores the sign of the gravity axis (the -1 g cases come out right only because a 2 g
    // error happens to wrap to zero in the 8-bit offset register), and it averages REMAPPED data
    // but writes the offsets in PHYSICAL axis order.
    //
    // Offsets are in 3.90625 mg (1/256 g) steps and are added to the output. At +/-4 g one step is
    // 8192 / 256 = 32 raw counts, so offset = (expected - measured) / 32.

    uint8_t axisIndex;
    bool negative;

    switch (gravityAxis)
    {
    case BMA456_FOC_X_POS:
    case BMA456_FOC_X_NEG:
        axisIndex = 0;
        break;
    case BMA456_FOC_Y_POS:
    case BMA456_FOC_Y_NEG:
        axisIndex = 1;
        break;
    case BMA456_FOC_Z_POS:
    case BMA456_FOC_Z_NEG:
        axisIndex = 2;
        break;
    default:
        return ksfTkErrInvalidParam;
    }

    negative = (gravityAxis == BMA456_FOC_X_NEG || gravityAxis == BMA456_FOC_Y_NEG || gravityAxis == BMA456_FOC_Z_NEG);

    // gravityAxis is given in the REPORTED frame. Translate it through the remap to the physical
    // axis the offset registers act on.
    const uint8_t sources[3] = {_dev.remap.x_axis, _dev.remap.y_axis, _dev.remap.z_axis};
    const uint8_t signs[3] = {_dev.remap.x_axis_sign, _dev.remap.y_axis_sign, _dev.remap.z_axis_sign};

    uint8_t physicalAxis = sources[axisIndex] < 3 ? sources[axisIndex] : axisIndex;
    if (signs[axisIndex] == BMA4_MAP_NEGATIVE)
        negative = !negative;

    // Save the configuration the measurement is about to replace.
    sfe_bma456_odr_t odr;
    sfe_bma456_range_t range;
    sfe_bma456_bandwidth_t bandwidth;
    sfe_bma456_perf_mode_t perfMode;

    sfTkError_t rc = getAccelConfig(odr, range, bandwidth, perfMode);
    if (rc != ksfTkErrOk)
        return rc;

    uint8_t accelEnabled = 0;
    uint8_t aps = 0;
    uint8_t compEnabled = 0;
    if (bosch(bma4_get_accel_enable(&accelEnabled, &_dev)) != ksfTkErrOk ||
        bosch(bma4_get_advance_power_save(&aps, &_dev)) != ksfTkErrOk ||
        bosch(bma4_get_offset_comp(&compEnabled, &_dev)) != ksfTkErrOk)
        return ksfBMA456ErrBosch;

    // Measure without the old correction: 100 Hz, +/-4 g, continuous filtering, awake.
    rc = enableOffsetCompensation(false);
    if (rc == ksfTkErrOk)
        rc = setAdvancedPowerSave(false);
    if (rc == ksfTkErrOk)
        rc = setAccelConfig(BMA456_ODR_100_HZ, BMA456_RANGE_4G, BMA456_BW_NORMAL_AVG4, BMA456_PERF_MODE_CONTINUOUS);
    if (rc == ksfTkErrOk)
        rc = enableAccel(true);

    int32_t sum[3] = {0, 0, 0};

    if (rc == ksfTkErrOk)
    {
        delayUs(kFocSettleUs);

        uint8_t taken = 0;
        uint16_t attempts = 0;

        while (taken < kFocSamples && attempts < kFocMaxAttempts)
        {
            attempts++;

            bool ready = false;
            rc = getDataReady(ready);
            if (rc != ksfTkErrOk)
                break;

            if (!ready)
            {
                delayUs(kFocPollUs);
                continue;
            }

            // Physical axes: read the data registers directly, no remap.
            uint8_t buffer[6];
            rc = bosch(bma4_read_regs(BMA4_DATA_8_ADDR, buffer, sizeof(buffer), &_dev));
            if (rc != ksfTkErrOk)
                break;

            sum[0] += (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
            sum[1] += (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
            sum[2] += (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
            taken++;
        }

        if (rc == ksfTkErrOk && taken < kFocSamples)
            rc = ksfTkErrFail; // The data-ready flag never kept up - the part is not sampling.
    }

    if (rc == ksfTkErrOk)
    {
        int8_t offsets[3];

        for (uint8_t i = 0; i < 3; i++)
        {
            int32_t expected = 0;
            if (i == physicalAxis)
                expected = negative ? -(int32_t)kFocLsbPerG : (int32_t)kFocLsbPerG;

            // Mean error in counts, then to offset steps, rounded to nearest (not toward zero).
            int32_t correction = expected * (int32_t)kFocSamples - sum[i];
            int32_t divisor = (int32_t)kFocSamples * (int32_t)kFocCountsPerStep;
            int32_t steps = (correction >= 0) ? (correction + divisor / 2) / divisor : (correction - divisor / 2) / divisor;

            if (steps > 127)
                steps = 127;
            if (steps < -128)
                steps = -128;

            offsets[i] = (int8_t)steps;
        }

        rc = setAccelOffset(offsets[0], offsets[1], offsets[2]); // Also enables compensation
    }

    // Restore the caller's configuration whether or not the measurement worked.
    sfTkError_t restore = setAccelConfig(odr, range, bandwidth, perfMode);
    // On success setAccelOffset() has already turned compensation on. On failure the old offsets
    // are still in the registers, so put the old enable state back with them.
    if (restore == ksfTkErrOk && rc != ksfTkErrOk)
        restore = enableOffsetCompensation(compEnabled != 0);
    if (restore == ksfTkErrOk)
        restore = enableAccel(accelEnabled != 0);
    if (restore == ksfTkErrOk)
        restore = setAdvancedPowerSave(aps != 0);

    return rc != ksfTkErrOk ? rc : restore;
}

sfTkError_t sfDevBMA456::setAccelOffset(int8_t x, int8_t y, int8_t z)
{
    const uint8_t offsets[3] = {(uint8_t)x, (uint8_t)y, (uint8_t)z};

    // One register at a time: with advanced power save on, burst writes are not allowed.
    for (uint8_t i = 0; i < 3; i++)
    {
        sfTkError_t rc = bosch(bma4_write_regs((uint8_t)(kRegOffset0 + i), &offsets[i], 1, &_dev));
        if (rc != ksfTkErrOk)
            return rc;
    }

    return enableOffsetCompensation(true);
}

sfTkError_t sfDevBMA456::getAccelOffset(int8_t &x, int8_t &y, int8_t &z)
{
    uint8_t offsets[3] = {0, 0, 0};

    sfTkError_t rc = bosch(bma4_read_regs(kRegOffset0, offsets, sizeof(offsets), &_dev));

    x = (int8_t)offsets[0];
    y = (int8_t)offsets[1];
    z = (int8_t)offsets[2];

    return rc;
}

sfTkError_t sfDevBMA456::enableOffsetCompensation(bool enable)
{
    return bosch(bma4_set_offset_comp(enable ? BMA4_ENABLE : BMA4_DISABLE, &_dev));
}
