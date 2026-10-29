/**
 * @file sfDevBMA456.h
 * @brief Header file for the SparkFun Bosch BMA456 Triple-Axis Accelerometer Driver.
 *
 * @details
 * sfDevBMA456 is a comms-agnostic driver for the Bosch BMA456 16-bit triaxial accelerometer, built
 * on the SparkFun Toolkit. It wraps Bosch's BMA456 SensorAPI (v2.29.0, BSD-3-Clause, vendored in
 * src/bma456_api) the same way the SparkFun BMA400 library wraps Bosch's BMA400 API, and routes
 * every bus transaction the Bosch code makes through an sfTkIBus - so I2C and SPI share one code
 * path and nothing below this layer knows about Arduino.
 *
 * @note A CONFIGURATION FILE IS REQUIRED FOR ANY FEATURE. The BMA456 reports acceleration straight
 * out of reset, but every "smart" feature - any/no-motion, tap, step counting, orientation, high-g
 * and the rest - runs on an on-chip feature engine that boots from a Bosch firmware blob the host
 * has to upload. Bosch publishes several blobs for the same silicon, each with a different feature
 * set, and only one can be loaded at a time. This library supports two of them, chosen at begin():
 *
 *   - ::sfeBMA456FeaturesH  (BMA456H)  - any/no-motion, step detector, step counter, step activity,
 *                                        single/double/triple tap, auto low power. The default.
 *   - ::sfeBMA456FeaturesMM (BMA456MM) - any/no-motion, orientation, high-g, low-g, significant
 *                                        motion, single/double/triple tap, auto low power.
 *   - ::sfeBMA456FeaturesNone          - no upload; plain accelerometer, FIFO and data interrupts.
 *
 * Each blob is 6144 bytes. Only the one a sketch actually names is linked into the image - the
 * others are discarded by the linker - so a sketch that uses the MM set does not pay flash for H.
 *
 * @note THE UPLOAD IS DONE HERE, NOT BY BOSCH. Bosch's bma4_write_config_file() streams the blob
 * straight out of a RAM pointer. On an AVR that would put 6 kB of const data into 2 kB of SRAM, so
 * the vendored blobs are stored with PROGMEM on AVR and loadFeatureConfig() copies them out in
 * small chunks. The chunk size (sfDevBMA456::kMaxTransferBytes) also keeps every I2C write inside a 32-byte
 * Wire buffer, which Bosch's code does not know exists.
 *
 * @note AXIS REMAPPING IS APPLIED HOST-SIDE AS WELL. Bosch's remap writes the map into the feature
 * engine (so the features see remapped axes) but the data registers are NOT remapped by the device
 * - bma4_read_accel_xyz() applies the map in software. This driver reads the data block itself, so
 * it applies the same map to getSensorData() and to FIFO frames.
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

#include <stddef.h>
#include <stdint.h>

// SparkFun Toolkit core headers
#include <sfTk/sfTkIBus.h>
#include <sfTk/sfTkII2C.h>
#include <sfTk/sfTkISPI.h>
#include <sfTk/sfToolkit.h>

// Bosch BMA456 SensorAPI - only the type definitions are needed here; the API itself is used from
// sfDevBMA456.cpp.
#include "../bma456_api/bma4_defs.h"

///////////////////////////////////////////////////////////////////////////////
// Device Identity
///////////////////////////////////////////////////////////////////////////////

/** @brief Default 7-bit I2C address (SDO low). SparkFun ships the Qwiic board this way. */
#define SFE_BMA456_DEFAULT_ADDRESS 0x18

/** @brief Alternate 7-bit I2C address, selected by the board's ADR jumper (SDO high). */
#define SFE_BMA456_ALT_ADDRESS 0x19

/** @brief Value of CHIP_ID (0x00) for a BMA456. */
#define SFE_BMA456_CHIP_ID 0x16

///////////////////////////////////////////////////////////////////////////////
// Error Codes
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief The requested feature is not part of the loaded feature set.
 *
 * @details Returned, for instance, by getStepCount() when the board was started with
 * ::sfeBMA456FeaturesMM, or by setOrientationConfig() with ::sfeBMA456FeaturesH. Nothing was
 * written to the device.
 */
const sfTkError_t ksfBMA456ErrFeatureUnavailable = ksfTkErrFail * 0x2001;

/**
 * @brief The feature engine did not report "initialized" after the configuration upload.
 *
 * @details Usually a corrupted transfer - long wires at 400 kHz, or a marginal pull-up. The part
 * still reports acceleration, but no feature will respond until begin() succeeds.
 */
const sfTkError_t ksfBMA456ErrConfigLoad = ksfTkErrFail * 0x2002;

/**
 * @brief A call into the Bosch SensorAPI returned an error. getLastBoschResult() has the code.
 */
const sfTkError_t ksfBMA456ErrBosch = ksfTkErrFail * 0x2003;

/** @brief The temperature register holds its "no valid data" value (0x80). */
const sfTkError_t ksfBMA456ErrNoData = ksfTkErrFail * 0x2004;

///////////////////////////////////////////////////////////////////////////////
// Feature Sets
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Which Bosch feature-engine configuration is loaded.
 */
typedef enum sfe_bma456_feature_set_id_t : uint8_t
{
    BMA456_FEATURE_SET_NONE = 0, ///< No configuration uploaded. Accelerometer, FIFO and data interrupts only.
    BMA456_FEATURE_SET_H = 1,    ///< Bosch BMA456H configuration: steps, activity, tap, any/no-motion.
    BMA456_FEATURE_SET_MM = 2    ///< Bosch BMA456MM configuration: orientation, high/low-g, sig-motion, tap, any/no-motion.
} sfe_bma456_feature_set_id_t;

/**
 * @brief Descriptor for one feature-engine configuration.
 *
 * @details Pass one of ::sfeBMA456FeaturesH, ::sfeBMA456FeaturesMM or ::sfeBMA456FeaturesNone to
 * begin(). These are objects rather than an enum on purpose: a descriptor is what references its
 * 6 kB blob, so a sketch that never names a descriptor never links that blob.
 */
struct sfe_bma456_feature_set_t
{
    sfe_bma456_feature_set_id_t id; ///< Which configuration this is.
    const uint8_t *config;          ///< The blob. In program memory on AVR. nullptr for NONE.
    const uint16_t *configSize;     ///< Size of the blob in bytes. nullptr for NONE.
};

/** @brief BMA456H feature set - any/no-motion, step detector/counter/activity, tap, auto low power. */
extern const sfe_bma456_feature_set_t sfeBMA456FeaturesH;

/** @brief BMA456MM feature set - any/no-motion, orientation, high-g, low-g, sig-motion, tap, auto low power. */
extern const sfe_bma456_feature_set_t sfeBMA456FeaturesMM;

/** @brief No feature engine. Skips the 6 kB upload - and its flash - entirely. */
extern const sfe_bma456_feature_set_t sfeBMA456FeaturesNone;

///////////////////////////////////////////////////////////////////////////////
// Accelerometer Configuration
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Output data rate, the ODR field of ACC_CONF (0x40).
 *
 * @details Continuous mode runs from 12.5 Hz to 1600 Hz; averaging (low-power) mode from 0.78 Hz
 * to 400 Hz. In averaging mode the number of samples averaged is capped by the rate - AVG128 up to
 * 6.25 Hz, AVG64 at 12.5 Hz, AVG32 at 25 Hz, AVG16 at 50 Hz, AVG8 at 100 Hz, AVG4 at 200 Hz, AVG2
 * at 400 Hz - and setAccelConfig() rejects a combination above the cap. Most feature-engine
 * features run at 50 Hz internally and need ODR >= 50 Hz.
 */
typedef enum sfe_bma456_odr_t : uint8_t
{
    BMA456_ODR_0_78_HZ = 0x01, ///< 0.78125 Hz. Averaging mode only.
    BMA456_ODR_1_56_HZ = 0x02, ///< 1.5625 Hz. Averaging mode only.
    BMA456_ODR_3_12_HZ = 0x03, ///< 3.125 Hz. Averaging mode only.
    BMA456_ODR_6_25_HZ = 0x04, ///< 6.25 Hz. Averaging mode only.
    BMA456_ODR_12_5_HZ = 0x05, ///< 12.5 Hz.
    BMA456_ODR_25_HZ = 0x06,   ///< 25 Hz.
    BMA456_ODR_50_HZ = 0x07,   ///< 50 Hz. The feature engine's internal rate.
    BMA456_ODR_100_HZ = 0x08,  ///< 100 Hz. Power-on default.
    BMA456_ODR_200_HZ = 0x09,  ///< 200 Hz.
    BMA456_ODR_400_HZ = 0x0A,  ///< 400 Hz.
    BMA456_ODR_800_HZ = 0x0B,  ///< 800 Hz. Continuous mode only.
    BMA456_ODR_1600_HZ = 0x0C  ///< 1600 Hz. Continuous mode only.
} sfe_bma456_odr_t;

/**
 * @brief Full-scale range, ACC_RANGE (0x41).
 *
 * @details The output is 16-bit signed across the selected range, so sensitivity halves each time
 * the range doubles: 16384 LSB/g at +/-2 g down to 2048 LSB/g at +/-16 g.
 */
typedef enum sfe_bma456_range_t : uint8_t
{
    BMA456_RANGE_2G = 0x00,  ///< +/-2 g, 16384 LSB/g.
    BMA456_RANGE_4G = 0x01,  ///< +/-4 g, 8192 LSB/g. Power-on default.
    BMA456_RANGE_8G = 0x02,  ///< +/-8 g, 4096 LSB/g.
    BMA456_RANGE_16G = 0x03  ///< +/-16 g, 2048 LSB/g.
} sfe_bma456_range_t;

/**
 * @brief Filter mode, the acc_perf_mode bit of ACC_CONF (0x40).
 */
typedef enum sfe_bma456_perf_mode_t : uint8_t
{
    BMA456_PERF_MODE_AVERAGING = 0x00, ///< Duty-cycled, averages 1 to 128 samples. Lowest current.
    BMA456_PERF_MODE_CONTINUOUS = 0x01 ///< Continuous filtering. Lowest noise. Power-on default.
} sfe_bma456_perf_mode_t;

/**
 * @brief Filter setting, the acc_bwp field of ACC_CONF (0x40).
 *
 * @details One field, two meanings, which is why each name carries both. In
 * ::BMA456_PERF_MODE_CONTINUOUS only the first three are legal and select the oversampling ratio of
 * the digital low-pass filter (OSR4 is the quietest, NORMAL the widest). In
 * ::BMA456_PERF_MODE_AVERAGING all eight are legal and select how many samples are averaged per
 * output - noise drops by sqrt(N), current rises with N.
 */
typedef enum sfe_bma456_bandwidth_t : uint8_t
{
    BMA456_BW_OSR4_AVG1 = 0x00,   ///< Continuous: OSR4. Averaging: 1 sample.
    BMA456_BW_OSR2_AVG2 = 0x01,   ///< Continuous: OSR2. Averaging: 2 samples.
    BMA456_BW_NORMAL_AVG4 = 0x02, ///< Continuous: normal. Averaging: 4 samples. Power-on default.
    BMA456_BW_CIC_AVG8 = 0x03,    ///< Averaging only: 8 samples.
    BMA456_BW_RES_AVG16 = 0x04,   ///< Averaging only: 16 samples.
    BMA456_BW_RES_AVG32 = 0x05,   ///< Averaging only: 32 samples.
    BMA456_BW_RES_AVG64 = 0x06,   ///< Averaging only: 64 samples.
    BMA456_BW_RES_AVG128 = 0x07   ///< Averaging only: 128 samples.
} sfe_bma456_bandwidth_t;

///////////////////////////////////////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Interrupt pin.
 *
 * @details Values match Bosch's BMA4_INTR1_MAP / BMA4_INTR2_MAP.
 */
typedef enum sfe_bma456_int_pin_t : uint8_t
{
    BMA456_INT1 = 0x00, ///< The INT1 pin.
    BMA456_INT2 = 0x01  ///< The INT2 pin.
} sfe_bma456_int_pin_t;

/**
 * @brief Interrupt sources that can be routed to a pin with mapInterrupt().
 *
 * @details The feature-engine sources sit at DIFFERENT bit positions in INT_MAP / INT_STATUS
 * depending on which configuration is loaded - any-motion is bit 4 under BMA456H and bit 5 under
 * BMA456MM, for example. This enum is the same for both; the driver translates.
 */
typedef enum sfe_bma456_int_source_t : uint8_t
{
    BMA456_INT_SRC_DATA_READY = 0, ///< New acceleration sample. Always available.
    BMA456_INT_SRC_FIFO_WATERMARK, ///< FIFO reached its watermark. Always available.
    BMA456_INT_SRC_FIFO_FULL,      ///< FIFO full. Always available.
    BMA456_INT_SRC_ANY_MOTION,     ///< Any-motion. H and MM.
    BMA456_INT_SRC_NO_MOTION,      ///< No-motion. H and MM.
    BMA456_INT_SRC_TAP,            ///< Single, double or triple tap. H and MM.
    BMA456_INT_SRC_ERROR,          ///< Feature engine error. H and MM.
    BMA456_INT_SRC_STEP_COUNTER,   ///< Step detector / step counter watermark. H only.
    BMA456_INT_SRC_ACTIVITY,       ///< Step activity changed. H only.
    BMA456_INT_SRC_ORIENTATION,    ///< Orientation changed. MM only.
    BMA456_INT_SRC_LOW_G,          ///< Low-g (free fall). MM only.
    BMA456_INT_SRC_HIGH_G,         ///< High-g (shock). MM only.
    BMA456_INT_SRC_SIG_MOTION      ///< Significant motion. MM only.
} sfe_bma456_int_source_t;

/**
 * @brief Bits in the word returned by getInterruptStatus().
 *
 * @details These are library bits, not register bits - the driver translates INT_STATUS_0/1
 * (0x1C/0x1D) according to the loaded feature set, so the same test works under H and MM.
 *
 * @note The BMA456 has ONE status register pair shared by both pins, and it is CLEAR-ON-READ.
 * Read it once per event and test every bit you care about in the returned word.
 */
typedef enum sfe_bma456_int_status_t : uint16_t
{
    BMA456_INT_STATUS_ANY_MOTION = 0x0001,     ///< Any-motion detected.
    BMA456_INT_STATUS_NO_MOTION = 0x0002,      ///< No-motion detected.
    BMA456_INT_STATUS_TAP = 0x0004,            ///< Tap detected - see getTapEvent() for which kind.
    BMA456_INT_STATUS_STEP_COUNTER = 0x0008,   ///< Step detected / step watermark reached (H).
    BMA456_INT_STATUS_ACTIVITY = 0x0010,       ///< Step activity changed (H).
    BMA456_INT_STATUS_ORIENTATION = 0x0020,    ///< Orientation changed (MM).
    BMA456_INT_STATUS_LOW_G = 0x0040,          ///< Low-g detected (MM).
    BMA456_INT_STATUS_HIGH_G = 0x0080,         ///< High-g detected (MM).
    BMA456_INT_STATUS_SIG_MOTION = 0x0100,     ///< Significant motion detected (MM).
    BMA456_INT_STATUS_ERROR = 0x0200,          ///< Feature engine error.
    BMA456_INT_STATUS_FIFO_FULL = 0x1000,      ///< FIFO full.
    BMA456_INT_STATUS_FIFO_WATERMARK = 0x2000, ///< FIFO watermark reached.
    BMA456_INT_STATUS_DATA_READY = 0x8000      ///< New acceleration sample.
} sfe_bma456_int_status_t;

/**
 * @brief Electrical configuration of an interrupt pin, INT1_IO_CTRL / INT2_IO_CTRL (0x53 / 0x54).
 *
 * @details Defaults are push-pull, active high. Use open-drain with @p activeHigh false when
 * several devices share one interrupt line into the host; the line then needs a pull-up.
 */
struct sfe_bma456_int_pin_config_t
{
    bool enabled = true;    ///< Drive the pin. False leaves it high-impedance.
    bool activeHigh = true; ///< true: idle low, assert high. false: idle high, assert low.
    bool openDrain = false; ///< true: open-drain output (needs a pull-up). false: push-pull.
};

///////////////////////////////////////////////////////////////////////////////
// Sensor Data
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief One triaxial reading in raw 16-bit signed counts.
 *
 * @note A value pinned at -32768 or 32767 means the reading saturated the selected range.
 */
struct sfe_bma456_raw_data_t
{
    int16_t x = 0; ///< X axis, raw counts.
    int16_t y = 0; ///< Y axis, raw counts.
    int16_t z = 0; ///< Z axis, raw counts.
};

/**
 * @brief One triaxial reading in g.
 */
struct sfe_bma456_data_t
{
    float x = 0.0f; ///< X axis, g.
    float y = 0.0f; ///< Y axis, g.
    float z = 0.0f; ///< Z axis, g.
};

/**
 * @brief Everything one burst read of the data block produces.
 *
 * @details getSensorData() fills this from a single 9-byte read of DATA_8 (0x12) through
 * SENSORTIME_2 (0x1A), so the acceleration and its timestamp describe the same instant.
 */
struct sfe_bma456_sensor_data_t
{
    sfe_bma456_data_t accel;        ///< Acceleration in g, axis remap applied.
    sfe_bma456_raw_data_t accelRaw; ///< Acceleration in raw counts, axis remap applied.
    uint32_t sensorTime = 0;        ///< 24-bit sensor timestamp, 39.0625 us per LSB.
};

/**
 * @brief Decoded contents of ERR_REG (0x02).
 */
struct sfe_bma456_error_status_t
{
    bool fatalError = false;     ///< Chip not operational. Cleared only by reset or power cycle.
    bool commandError = false;   ///< A command written to CMD (0x7E) failed.
    bool accelConfError = false; ///< ACC_CONF holds an illegal ODR / filter combination.
    bool fifoError = false;      ///< FIFO configuration or overflow error.
    uint8_t errorCode = 0;       ///< Raw error_code field (bits 4:2). 0 = none, 1 = ACC_CONF error.
};

///////////////////////////////////////////////////////////////////////////////
// Feature Engine
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Feature enable bits for enableFeature(). Combine with bitwise OR.
 *
 * @details Library bits, translated per feature set. Any-motion and no-motion are enabled PER
 * AXIS - the detector runs when at least one of its axis bits is set - and both feature sets
 * support them. The rest are single bits; asking for one the loaded set does not have returns
 * ::ksfBMA456ErrFeatureUnavailable and changes nothing.
 */
typedef enum sfe_bma456_feature_t : uint16_t
{
    BMA456_FEAT_NO_MOTION_X = 0x0001,   ///< No-motion on X. H and MM.
    BMA456_FEAT_NO_MOTION_Y = 0x0002,   ///< No-motion on Y. H and MM.
    BMA456_FEAT_NO_MOTION_Z = 0x0004,   ///< No-motion on Z. H and MM.
    BMA456_FEAT_NO_MOTION = 0x0007,     ///< No-motion on all axes. H and MM.
    BMA456_FEAT_ANY_MOTION_X = 0x0008,  ///< Any-motion on X. H and MM.
    BMA456_FEAT_ANY_MOTION_Y = 0x0010,  ///< Any-motion on Y. H and MM.
    BMA456_FEAT_ANY_MOTION_Z = 0x0020,  ///< Any-motion on Z. H and MM.
    BMA456_FEAT_ANY_MOTION = 0x0038,    ///< Any-motion on all axes. H and MM.
    BMA456_FEAT_SINGLE_TAP = 0x0040,    ///< Single tap. H and MM.
    BMA456_FEAT_DOUBLE_TAP = 0x0080,    ///< Double tap. H and MM.
    BMA456_FEAT_TRIPLE_TAP = 0x0100,    ///< Triple tap. H and MM.
    BMA456_FEAT_STEP_DETECTOR = 0x0200, ///< One interrupt per step. H only.
    BMA456_FEAT_STEP_COUNTER = 0x0400,  ///< Step counter. H only.
    BMA456_FEAT_STEP_ACTIVITY = 0x0800, ///< Still / walking / running classification. H only.
    BMA456_FEAT_ORIENTATION = 0x1000,   ///< Portrait / landscape / face up-down. MM only.
    BMA456_FEAT_LOW_G = 0x2000,         ///< Low-g (free fall). MM only.
    BMA456_FEAT_HIGH_G = 0x4000,        ///< High-g (shock). MM only.
    BMA456_FEAT_SIG_MOTION = 0x8000     ///< Significant motion. MM only.
} sfe_bma456_feature_t;

/**
 * @brief Any-motion / no-motion detector settings.
 *
 * @details Any-motion fires when the slope between successive samples (at the engine's 50 Hz rate)
 * stays ABOVE @p threshold for @p duration samples on an enabled axis; no-motion fires when it
 * stays BELOW. Slope, not magnitude - a board lying still at any angle has zero slope.
 *
 * @note UNITS ARE THE HARDWARE'S. Threshold is Bosch's "5.11 format": 1/2048 g, about 0.488 mg per
 * LSB, 11 bits (0 to ~1 g). Duration is 20 ms per LSB, 13 bits. The defaults are this library's
 * suggestion, not the device's power-on values - read those with getAnyMotionConfig() first.
 */
struct sfe_bma456_motion_config_t
{
    uint16_t threshold = 170;  ///< Slope threshold, 0.488 mg/LSB. 170 is about 83 mg. Range 0 to 2047.
    uint16_t duration = 5;     ///< Samples the condition must hold, 20 ms/LSB. Range 0 to 8191.
    uint8_t intrBehavior = 0;  ///< Bosch intr_bhvr: 0 = interrupt on every detection, 1 = single shot.
    uint8_t slope = 0;         ///< Bosch slope: 0 = non-consecutive, 1 = consecutive samples (any-motion only).
};

/**
 * @brief Tap detector settings (both feature sets use the same layout).
 *
 * @details Field names follow Bosch's bma456h_multitap_settings / bma456mm_multitap_settings. The
 * reserved words in that structure are preserved by a read-modify-write inside setTapConfig().
 *
 * @note Tap tuning is enclosure-specific. Sweep @p sensitivityThreshold against the real product -
 * a tap through a plastic lid is a very different signal from a tap on a bare board.
 */
struct sfe_bma456_tap_config_t
{
    uint16_t sensitivityThreshold = 0; ///< tap_sens_thres - scaling factor for the detection threshold.
    uint16_t maxGestureDuration = 0;   ///< max_gest_dur - window after the first tap for the rest of the gesture.
    uint16_t shockDuration = 0;        ///< tap_shock_dur - settling time allowed for one tap's impact.
    uint16_t quietTimeAfterGesture = 0;///< quite_time_after_gest - minimum quiet time between gestures.
    uint16_t waitForTimeout = 0;       ///< wait_for_timeout - wait out max_gest_dur before reporting.
    uint16_t axisSelect = 0;           ///< axis_sel - which axis (or combination) the detector watches.
};

/**
 * @brief Which tap gesture the device just reported. More than one can be set.
 */
struct sfe_bma456_tap_event_t
{
    bool singleTap = false; ///< Single tap.
    bool doubleTap = false; ///< Double tap.
    bool tripleTap = false; ///< Triple tap.
};

/**
 * @brief Step activity classification (BMA456H).
 */
typedef enum sfe_bma456_activity_t : uint8_t
{
    BMA456_ACTIVITY_STILL = 0x00,   ///< User stationary.
    BMA456_ACTIVITY_WALKING = 0x01, ///< User walking.
    BMA456_ACTIVITY_RUNNING = 0x02, ///< User running.
    BMA456_ACTIVITY_UNKNOWN = 0x03  ///< Unclassified.
} sfe_bma456_activity_t;

/**
 * @brief Device orientation (BMA456MM).
 */
typedef enum sfe_bma456_orientation_t : uint8_t
{
    BMA456_ORIENT_PORTRAIT_UPRIGHT = 0x00,     ///< Portrait, upright.
    BMA456_ORIENT_LANDSCAPE_LEFT = 0x01,       ///< Landscape, left.
    BMA456_ORIENT_PORTRAIT_UPSIDE_DOWN = 0x02, ///< Portrait, upside down.
    BMA456_ORIENT_LANDSCAPE_RIGHT = 0x03       ///< Landscape, right.
} sfe_bma456_orientation_t;

/**
 * @brief Orientation detector settings (BMA456MM).
 */
struct sfe_bma456_orientation_config_t
{
    bool upsideDown = true;  ///< Also report face up / face down.
    uint8_t mode = 0;        ///< 0 or 3 symmetrical, 1 high asymmetrical, 2 low asymmetrical.
    uint8_t blocking = 3;    ///< Suppress changes during motion. 0 none to 3 most aggressive.
    uint8_t theta = 40;      ///< Coded blocking angle to horizontal. Range 0 to 63.
    uint16_t hysteresis = 128; ///< Acceleration hysteresis, 0.488 mg/LSB. Range 0 to 2047.
};

/**
 * @brief Low-g (free-fall) detector settings (BMA456MM).
 *
 * @details Low-g fires when the acceleration MAGNITUDE stays below @p threshold for @p duration.
 * A dropped board reads close to 0 g on every axis, which is what makes free fall detectable.
 */
struct sfe_bma456_low_g_config_t
{
    uint16_t threshold = 512;  ///< Magnitude threshold, 0.488 mg/LSB. 512 is 250 mg. Range 0 to 32767.
    uint16_t hysteresis = 256; ///< Hysteresis, 0.488 mg/LSB. Range 0 to 4095.
    uint16_t duration = 0;     ///< Samples the condition must hold, 20 ms/LSB. Range 0 to 4095.
};

/**
 * @brief High-g (shock) detector settings (BMA456MM).
 *
 * @details High-g fires when the acceleration on an enabled axis exceeds @p threshold for
 * @p duration. It is the only feature here evaluated at 200 Hz, hence the 5 ms duration unit.
 */
struct sfe_bma456_high_g_config_t
{
    uint16_t threshold = 4096; ///< Threshold, 0.488 mg/LSB. 4096 is 2 g. Range 0 to 32767.
    uint16_t hysteresis = 1024;///< Hysteresis, 0.488 mg/LSB. Range 0 to 4095.
    uint16_t duration = 4;     ///< Samples the condition must hold, 5 ms/LSB. Range 0 to 4095.
    bool xEnabled = true;      ///< Watch X.
    bool yEnabled = true;      ///< Watch Y.
    bool zEnabled = true;      ///< Watch Z.
};

/**
 * @brief Which axis tripped the high-g detector, and in which direction (BMA456MM).
 */
struct sfe_bma456_high_g_event_t
{
    bool x = false;        ///< High-g on X.
    bool y = false;        ///< High-g on Y.
    bool z = false;        ///< High-g on Z.
    bool negative = false; ///< true if the event was in the negative direction.
};

/**
 * @brief Significant-motion detector settings (BMA456MM).
 *
 * @details Significant motion waits for motion above @p threshold, then ignores the sensor for
 * @p skipTime, then requires motion again within @p proofTime. A knock on the desk passes the first
 * test and fails the second; picking the device up and walking away passes both.
 */
struct sfe_bma456_sig_motion_config_t
{
    uint16_t threshold = 0xFA; ///< Threshold, 0.488 mg/LSB. Range 0 to 32767.
    uint16_t skipTime = 150;   ///< Wait after the first detection, 20 ms/LSB. Range 0 to 511.
    uint8_t proofTime = 50;    ///< Window for the confirming detection, 20 ms/LSB. Range 0 to 127.
};

/**
 * @brief Output data rate the part drops to when auto low power engages.
 */
typedef enum sfe_bma456_lp_odr_t : uint8_t
{
    BMA456_LP_ODR_1_56_HZ = 0x00, ///< 25/16 Hz.
    BMA456_LP_ODR_3_12_HZ = 0x01, ///< 25/8 Hz.
    BMA456_LP_ODR_6_25_HZ = 0x02, ///< 25/4 Hz.
    BMA456_LP_ODR_12_5_HZ = 0x03  ///< 25/2 Hz.
} sfe_bma456_lp_odr_t;

/**
 * @brief Auto low power settings (both feature sets).
 *
 * @details With auto low power enabled, the feature engine drops the accelerometer into a low-rate
 * averaging mode on its own - after no-motion, or after @p timeoutDuration without any-motion, or
 * both - and returns it to the host's configuration on any-motion. The host does nothing.
 */
struct sfe_bma456_auto_low_power_config_t
{
    bool onNoMotion = false;       ///< Enter low power when the no-motion detector fires.
    bool onTimeout = true;         ///< Enter low power when any-motion has been silent for @p timeoutDuration.
    uint16_t timeoutDuration = 100;///< Time-out, 20 ms/LSB. 100 is 2 s. Range 0 to 2047 (H), 0 to 1023 (MM).
    sfe_bma456_lp_odr_t lowPowerODR = BMA456_LP_ODR_1_56_HZ; ///< Rate while in low power.
    bool featureOptimizedConfig = true; ///< Bosch pwr_mgt: let the engine pick the low-power ACC_CONF.
};

///////////////////////////////////////////////////////////////////////////////
// Axis Remapping
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief A physical axis, optionally negated. Values match Bosch's BMA4_X ... BMA4_NEG_Z.
 */
typedef enum sfe_bma456_axis_t : uint8_t
{
    BMA456_AXIS_X = 0x01,     ///< Physical +X.
    BMA456_AXIS_Y = 0x02,     ///< Physical +Y.
    BMA456_AXIS_Z = 0x04,     ///< Physical +Z.
    BMA456_AXIS_NEG_X = 0x09, ///< Physical -X.
    BMA456_AXIS_NEG_Y = 0x0A, ///< Physical -Y.
    BMA456_AXIS_NEG_Z = 0x0C  ///< Physical -Z.
} sfe_bma456_axis_t;

/**
 * @brief Which physical axis feeds each reported axis.
 *
 * @details Read as "reported X comes from @p x". Each physical axis must be used exactly once.
 * A board mounted upside down under a chassis is {BMA456_AXIS_X, BMA456_AXIS_NEG_Y,
 * BMA456_AXIS_NEG_Z}.
 */
struct sfe_bma456_axis_remap_t
{
    sfe_bma456_axis_t x = BMA456_AXIS_X; ///< Source of reported X.
    sfe_bma456_axis_t y = BMA456_AXIS_Y; ///< Source of reported Y.
    sfe_bma456_axis_t z = BMA456_AXIS_Z; ///< Source of reported Z.
};

///////////////////////////////////////////////////////////////////////////////
// Calibration
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Which way gravity points during a fast offset compensation.
 *
 * @details FOC assumes the board is still and that exactly one axis sees +1 g or -1 g. Flat on a
 * bench, component side up, is ::BMA456_FOC_Z_POS.
 */
typedef enum sfe_bma456_foc_axis_t : uint8_t
{
    BMA456_FOC_X_POS = 0, ///< +X points up (reads +1 g).
    BMA456_FOC_X_NEG,     ///< -X points up (reads -1 g).
    BMA456_FOC_Y_POS,     ///< +Y points up.
    BMA456_FOC_Y_NEG,     ///< -Y points up.
    BMA456_FOC_Z_POS,     ///< +Z points up. Board flat, component side up.
    BMA456_FOC_Z_NEG      ///< -Z points up. Board flat, component side down.
} sfe_bma456_foc_axis_t;

///////////////////////////////////////////////////////////////////////////////
// sfDevBMA456
///////////////////////////////////////////////////////////////////////////////

/**
 * @class sfDevBMA456
 * @brief Platform-independent driver for the Bosch BMA456 accelerometer.
 *
 * @details
 * Owns the Bosch bma4_dev structure, supplies its bus and delay callbacks, and exposes the
 * SensorAPI through SparkFun types. It knows nothing about Arduino, Wire or SPI - the wrappers in
 * SparkFun_BMA456.h provide those. A sketch normally uses SfeBMA456ArdI2C or SfeBMA456ArdSPI.
 *
 * Methods that touch the bus return sfTkError_t (`ksfTkErrOk` on success); methods that only read
 * cached state return the value directly. When a Bosch call fails the method returns
 * ::ksfBMA456ErrBosch and getLastBoschResult() holds Bosch's own code.
 *
 * @see SfeBMA456ArdI2C
 * @see SfeBMA456ArdSPI
 */
class sfDevBMA456
{
  public:
    /**
     * @brief Default constructor. The device is not touched until begin().
     */
    sfDevBMA456();

    virtual ~sfDevBMA456()
    {
    }

    ///////////////////////////////////////////////////////////////////////////
    // Setup and Identity
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Adopt a bus, verify the device, reset it, and load a feature-engine configuration.
     *
     * @details Checks CHIP_ID, soft resets, uploads the configuration for @p features (about 0.6 s over
     * 400 kHz I2C), then ENABLES the accelerometer at its power-on configuration -
     * 100 Hz, +/-4 g, continuous filtering - so data is available immediately, as with the BMA400
     * library. Change it with setAccelConfig().
     *
     * @param theBus Bus to communicate over. Pass nullptr to keep a bus set earlier.
     * @param features ::sfeBMA456FeaturesH, ::sfeBMA456FeaturesMM or ::sfeBMA456FeaturesNone.
     * @return `ksfTkErrOk` on success, `ksfTkErrBusNotInit` with no bus, `ksfTkErrBusNoResponse` if
     * the chip ID does not match, ::ksfBMA456ErrConfigLoad if the feature engine did not start.
     */
    sfTkError_t begin(sfTkIBus *theBus, const sfe_bma456_feature_set_t &features);

    /**
     * @brief Set the communication bus without initializing the device.
     * @param theBus Bus to use for communication.
     */
    void setCommunicationBus(sfTkIBus *theBus);

    /**
     * @brief Check that a BMA456 is responding.
     * @return true if CHIP_ID reads ::SFE_BMA456_CHIP_ID.
     */
    bool isConnected(void);

    /**
     * @brief Read CHIP_ID (0x00).
     * @param chipId Receives the chip ID - 0x16 for a BMA456.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getChipID(uint8_t &chipId);

    /**
     * @brief Soft reset the device and reload the feature-engine configuration.
     *
     * @details A reset clears the feature engine along with every register, so this re-uploads the
     * configuration chosen at begin() and re-enables the accelerometer at its power-on settings.
     * Any feature configuration, interrupt mapping, FIFO setup or axis remap is lost.
     *
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t softReset(void);

    /**
     * @brief Which feature set is loaded.
     * @return The feature set ID chosen at begin().
     */
    sfe_bma456_feature_set_id_t getFeatureSet(void)
    {
        return _featureSetId;
    }

    /**
     * @brief Read the configuration ID the feature engine reports.
     *
     * @details A quick check that the blob the device is running is the one you think it is.
     *
     * @param configId Receives the ID.
     * @return `ksfTkErrOk` on success, ::ksfBMA456ErrFeatureUnavailable with no feature set.
     */
    sfTkError_t getConfigId(uint16_t &configId);

    /**
     * @brief Read INTERNAL_STATUS (0x2A).
     *
     * @details The low nibble is the feature-engine message: 0x00 not initialized, 0x01 initialized,
     * 0x02 initialization error. Bit 5 flags an invalid axis remap; bit 6 an ODR below 50 Hz while a
     * feature that needs 50 Hz is enabled.
     *
     * @param status Receives the register.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getInternalStatus(uint8_t &status);

    /**
     * @brief Read and decode ERR_REG (0x02).
     * @param status Receives the decoded error flags.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getErrorStatus(sfe_bma456_error_status_t &status);

    /**
     * @brief The result code of the most recent Bosch SensorAPI call.
     * @return BMA4_OK (0) or a negative BMA4_E_... code from bma4_defs.h.
     */
    int8_t getLastBoschResult(void)
    {
        return _lastBoschResult;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Accelerometer Configuration
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the whole accelerometer configuration.
     *
     * @details Validated by Bosch before it is written: a bandwidth above ::BMA456_BW_NORMAL_AVG4
     * in continuous mode is rejected. The range is cached for the raw-to-g conversion.
     *
     * @param odr Output data rate.
     * @param range Full-scale range.
     * @param bandwidth Filter setting. Meaning depends on @p perfMode.
     * @param perfMode Continuous filtering or averaging.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAccelConfig(sfe_bma456_odr_t odr, sfe_bma456_range_t range,
                               sfe_bma456_bandwidth_t bandwidth = BMA456_BW_NORMAL_AVG4,
                               sfe_bma456_perf_mode_t perfMode = BMA456_PERF_MODE_CONTINUOUS);

    /**
     * @brief Read the accelerometer configuration back from ACC_CONF / ACC_RANGE.
     *
     * @details Also refreshes the cached range.
     *
     * @param odr Receives the output data rate.
     * @param range Receives the full-scale range.
     * @param bandwidth Receives the filter setting.
     * @param perfMode Receives the filter mode.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAccelConfig(sfe_bma456_odr_t &odr, sfe_bma456_range_t &range, sfe_bma456_bandwidth_t &bandwidth,
                               sfe_bma456_perf_mode_t &perfMode);

    /**
     * @brief Turn the accelerometer on or off (PWR_CTRL acc_en).
     *
     * @note The feature engine runs on accelerometer data, so every feature stops while the
     * accelerometer is off.
     *
     * @param enable true to enable.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t enableAccel(bool enable = true);

    /**
     * @brief Set the output data rate, leaving the rest of the configuration alone.
     * @param odr Output data rate.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAccelODR(sfe_bma456_odr_t odr);

    /**
     * @brief Set the full-scale range, leaving the rest of the configuration alone.
     * @param range Full-scale range.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAccelRange(sfe_bma456_range_t range);

    /**
     * @brief Set the filter setting and mode, leaving ODR and range alone.
     * @param bandwidth Filter setting.
     * @param perfMode Continuous filtering or averaging.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAccelFilter(sfe_bma456_bandwidth_t bandwidth, sfe_bma456_perf_mode_t perfMode);

    /**
     * @brief The cached full-scale range used by the conversions. Does not read the device.
     * @return The cached range.
     */
    sfe_bma456_range_t getAccelRange(void)
    {
        return _range;
    }

    /**
     * @brief Enable or disable advanced power save (PWR_CONF adv_power_save).
     *
     * @details With APS on, the part sleeps between samples in averaging mode and draws a few
     * microamps. Register writes then need 450 us between them, which the driver already honors.
     * begin() leaves APS on, as Bosch's configuration loader does.
     *
     * @param enable true to enable.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAdvancedPowerSave(bool enable);

    /**
     * @brief Read the advanced power save setting.
     * @param enabled Receives the setting.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAdvancedPowerSave(bool &enabled);

    ///////////////////////////////////////////////////////////////////////////
    // Measurement
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read acceleration and timestamp in one transaction.
     * @param data Receives the reading.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getSensorData(sfe_bma456_sensor_data_t &data);

    /**
     * @brief Read acceleration in g.
     * @param data Receives the acceleration.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAccelData(sfe_bma456_data_t &data);

    /**
     * @brief Read acceleration in raw counts.
     * @param data Receives the raw acceleration.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAccelDataRaw(sfe_bma456_raw_data_t &data);

    /**
     * @brief Read the die temperature in degrees Celsius.
     *
     * @details TEMPERATURE (0x22) is a SIGNED byte, 1 K per LSB, 0 = 23 degC, updated every 1.28 s.
     *
     * @note Bosch's bma4_get_temperature() casts the register as unsigned, so anything below 23 degC
     * comes back 256 degrees too hot. This method reads the register itself.
     *
     * @param celsius Receives the temperature.
     * @return `ksfTkErrOk`, or ::ksfBMA456ErrNoData before the first conversion.
     */
    sfTkError_t getTemperature(float &celsius);

    /**
     * @brief Read the 24-bit sensor timestamp, 39.0625 us per LSB. It wraps every 655.36 s.
     * @param sensorTime Receives the timestamp.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getSensorTime(uint32_t &sensorTime);

    /**
     * @brief Check the data-ready flag in STATUS (0x03).
     * @param ready Receives true if a new sample is waiting.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getDataReady(bool &ready);

    ///////////////////////////////////////////////////////////////////////////
    // Unit Conversion
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Convert a raw count to g using the cached range.
     * @param raw Raw 16-bit signed count.
     * @return Acceleration in g.
     */
    float accelRawToG(int16_t raw);

    /**
     * @brief Sensitivity of a range, in LSB per g.
     * @param range Full-scale range.
     * @return 16384, 8192, 4096 or 2048.
     */
    static float accelSensitivity(sfe_bma456_range_t range);

    /**
     * @brief Convert sensor-time ticks to microseconds.
     * @param ticks Sensor time difference.
     * @return Microseconds.
     */
    static uint32_t sensorTimeToMicros(uint32_t ticks);

    ///////////////////////////////////////////////////////////////////////////
    // Interrupts
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Configure the electrical behavior of one interrupt pin.
     * @param pin Which pin.
     * @param config Output enable, polarity and drive.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setInterruptPinConfig(sfe_bma456_int_pin_t pin, const sfe_bma456_int_pin_config_t &config);

    /**
     * @brief Choose between latched and non-latched interrupts (INT_LATCH, 0x55).
     *
     * @details Non-latched (the default) pulses the pin per event. Latched holds it asserted until
     * getInterruptStatus() is called - what an edge-triggered host ISR that might miss a short pulse
     * wants. The setting is global to both pins.
     *
     * @param latched true to latch.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setInterruptLatch(bool latched);

    /**
     * @brief Route an interrupt source to a pin, or remove it.
     *
     * @param source The source.
     * @param pin The pin.
     * @param enable true to route, false to remove.
     * @return `ksfTkErrOk`, or ::ksfBMA456ErrFeatureUnavailable for a source the loaded set lacks.
     */
    sfTkError_t mapInterrupt(sfe_bma456_int_source_t source, sfe_bma456_int_pin_t pin, bool enable = true);

    /**
     * @brief Read and clear the interrupt status.
     *
     * @note CLEAR-ON-READ, and shared by both pins. Test the returned word against the
     * ::sfe_bma456_int_status_t masks rather than calling again per bit.
     *
     * @param status Receives the library status word.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getInterruptStatus(uint16_t &status);

    ///////////////////////////////////////////////////////////////////////////
    // FIFO
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Enable or disable accelerometer data in the 1 kB FIFO.
     *
     * @details The FIFO is run HEADERLESS: every frame is exactly six bytes of X/Y/Z, which is what
     * lets readFifoData() read whole frames in fixed-size chunks and never tear one in half.
     *
     * @param enable true to record accelerometer data.
     * @param stopOnFull true to stop writing when full (keep the oldest), false to overwrite the
     * oldest data and keep the newest.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setFifoConfig(bool enable, bool stopOnFull = false);

    /**
     * @brief Choose downsampling and filtering for FIFO data (FIFO_DOWNS, 0x45).
     *
     * @param downsampling Keep one sample in 2^@p downsampling. Range 0 to 7.
     * @param filtered true for filtered data (at ODR), false for unfiltered data (at 1600 Hz before
     * downsampling).
     * @return `ksfTkErrOk` on success, `ksfTkErrInvalidParam` if @p downsampling > 7.
     */
    sfTkError_t setFifoDownsampling(uint8_t downsampling, bool filtered = true);

    /**
     * @brief Set the FIFO watermark, in BYTES.
     *
     * @details One frame is sfDevBMA456::kFifoFrameBytes (6) bytes, so 60 is ten samples.
     *
     * @param bytes Watermark level. Range 0 to 1024.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setFifoWatermark(uint16_t bytes);

    /**
     * @brief Read the FIFO watermark, in bytes.
     * @param bytes Receives the watermark level.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getFifoWatermark(uint16_t &bytes);

    /**
     * @brief Read how many bytes the FIFO holds.
     * @param bytes Receives the fill level.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getFifoLength(uint16_t &bytes);

    /**
     * @brief Discard everything in the FIFO.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t flushFifo(void);

    /**
     * @brief Read whole frames out of the FIFO.
     *
     * @details Reads at most @p maxFrames, only whole frames, with the axis remap applied. Bosch's
     * bma4_read_fifo_data() is deliberately not used: it reads the ENTIRE fill level into the
     * caller's buffer whatever size that buffer is, which on a 2 kB AVR is an overflow waiting to
     * happen.
     *
     * @param frames Destination array.
     * @param maxFrames Capacity of @p frames.
     * @param numFrames Receives the number of frames read.
     * @return `ksfTkErrOk` on success, including when the FIFO was empty.
     */
    sfTkError_t readFifoData(sfe_bma456_raw_data_t *frames, uint16_t maxFrames, uint16_t &numFrames);

    ///////////////////////////////////////////////////////////////////////////
    // Feature Engine - common to H and MM
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Enable or disable feature-engine features.
     *
     * @param features Bitwise OR of ::sfe_bma456_feature_t values.
     * @param enable true to enable.
     * @return `ksfTkErrOk`, or ::ksfBMA456ErrFeatureUnavailable if any bit is not in the loaded
     * set (in which case nothing is changed).
     */
    sfTkError_t enableFeature(uint16_t features, bool enable = true);

    /**
     * @brief Write the any-motion configuration.
     * @param config Threshold, duration and behavior.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAnyMotionConfig(const sfe_bma456_motion_config_t &config);

    /**
     * @brief Read the any-motion configuration.
     * @param config Receives the configuration.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAnyMotionConfig(sfe_bma456_motion_config_t &config);

    /**
     * @brief Write the no-motion configuration.
     * @param config Threshold, duration and behavior.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setNoMotionConfig(const sfe_bma456_motion_config_t &config);

    /**
     * @brief Read the no-motion configuration.
     * @param config Receives the configuration.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getNoMotionConfig(sfe_bma456_motion_config_t &config);

    /**
     * @brief Write the tap detector configuration (read-modify-write; reserved words preserved).
     * @param config Tap parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setTapConfig(const sfe_bma456_tap_config_t &config);

    /**
     * @brief Read the tap detector configuration.
     * @param config Receives the parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getTapConfig(sfe_bma456_tap_config_t &config);

    /**
     * @brief Read which tap gesture was detected.
     * @param event Receives the gesture flags.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getTapEvent(sfe_bma456_tap_event_t &event);

    /**
     * @brief Write the auto low power configuration.
     *
     * @note Under BMA456MM the time-out field is 10 bits (1023 x 20 ms = 20.5 s) and Bosch's setter
     * overwrites the auto-low-power enable bit that sits above it. This method rejects longer
     * time-outs and restores the enable bit after the write.
     *
     * @param config Entry conditions, time-out and low-power rate.
     * @return `ksfTkErrOk`, or `ksfTkErrInvalidParam` for an MM time-out above 1023.
     */
    sfTkError_t setAutoLowPowerConfig(const sfe_bma456_auto_low_power_config_t &config);

    /**
     * @brief Read the auto low power configuration.
     * @param config Receives the configuration.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAutoLowPowerConfig(sfe_bma456_auto_low_power_config_t &config);

    /**
     * @brief Turn auto low power on or off.
     * @param enable true to enable.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t enableAutoLowPower(bool enable = true);

    /**
     * @brief Check whether auto low power has currently put the part into low power.
     * @param inLowPower Receives true while the engine holds the part in low power.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAutoLowPowerState(bool &inLowPower);

    /**
     * @brief Remap the axes, in the feature engine and in this driver's data path.
     *
     * @details With ::sfeBMA456FeaturesNone there is no feature engine to tell, so the map is
     * applied host-side only - which for data and FIFO is exactly the same result.
     *
     * @param remap Source of each reported axis. Each physical axis must appear once.
     * @return `ksfTkErrOk`, or `ksfTkErrInvalidParam` if an axis is repeated.
     */
    sfTkError_t setAxisRemap(const sfe_bma456_axis_remap_t &remap);

    /**
     * @brief Read back the axis remap.
     * @param remap Receives the map.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAxisRemap(sfe_bma456_axis_remap_t &remap);

    ///////////////////////////////////////////////////////////////////////////
    // Feature Engine - BMA456H only
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read the step counter.
     * @param steps Receives the count.
     * @return `ksfTkErrOk`, or ::ksfBMA456ErrFeatureUnavailable without the H set.
     */
    sfTkError_t getStepCount(uint32_t &steps);

    /**
     * @brief Zero the step counter.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t resetStepCount(void);

    /**
     * @brief Set the step counter interrupt watermark.
     *
     * @details The step-counter interrupt fires every @p watermark x 20 steps. Zero turns the
     * interrupt into a per-step detector - which is how Bosch's step-detector example uses it.
     *
     * @param watermark Watermark in units of 20 steps. Range 0 to 1023.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setStepCounterWatermark(uint16_t watermark);

    /**
     * @brief Read the step counter watermark.
     * @param watermark Receives the watermark, 20 steps per LSB.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getStepCounterWatermark(uint16_t &watermark);

    /**
     * @brief Read the step activity classification.
     * @param activity Receives still / walking / running / unknown.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getActivity(sfe_bma456_activity_t &activity);

    ///////////////////////////////////////////////////////////////////////////
    // Feature Engine - BMA456MM only
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the orientation configuration.
     * @param config Orientation parameters.
     * @return `ksfTkErrOk`, or ::ksfBMA456ErrFeatureUnavailable without the MM set.
     */
    sfTkError_t setOrientationConfig(const sfe_bma456_orientation_config_t &config);

    /**
     * @brief Read the orientation configuration.
     * @param config Receives the parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getOrientationConfig(sfe_bma456_orientation_config_t &config);

    /**
     * @brief Read the current orientation.
     * @param orientation Receives portrait / landscape.
     * @param faceDown Receives true if face down (valid only with upsideDown enabled).
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getOrientation(sfe_bma456_orientation_t &orientation, bool &faceDown);

    /**
     * @brief Write the low-g configuration.
     * @param config Low-g parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setLowGConfig(const sfe_bma456_low_g_config_t &config);

    /**
     * @brief Read the low-g configuration.
     * @param config Receives the parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getLowGConfig(sfe_bma456_low_g_config_t &config);

    /**
     * @brief Write the high-g configuration.
     * @param config High-g parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setHighGConfig(const sfe_bma456_high_g_config_t &config);

    /**
     * @brief Read the high-g configuration.
     * @param config Receives the parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getHighGConfig(sfe_bma456_high_g_config_t &config);

    /**
     * @brief Read which axis tripped the high-g detector.
     * @param event Receives the axis and direction.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getHighGEvent(sfe_bma456_high_g_event_t &event);

    /**
     * @brief Write the significant-motion configuration.
     * @param config Significant-motion parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setSigMotionConfig(const sfe_bma456_sig_motion_config_t &config);

    /**
     * @brief Read the significant-motion configuration.
     * @param config Receives the parameters.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getSigMotionConfig(sfe_bma456_sig_motion_config_t &config);

    ///////////////////////////////////////////////////////////////////////////
    // Self-Test and Calibration
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Run the accelerometer self-test.
     *
     * @details Bosch's routine deflects the proof mass electrostatically in both directions and
     * checks the difference on each axis against the datasheet minimum (1800 mg for the BMA45x).
     * It ends in a soft reset, so this method reloads the feature configuration and restores the
     * accelerometer configuration afterwards. Anything else - features, interrupts, FIFO, remap -
     * must be set up again.
     *
     * @note Keep the board still. Takes roughly 400 ms plus a configuration upload.
     *
     * @param passed Receives the verdict.
     * @return `ksfTkErrOk` if the test ran (check @p passed), an error if it could not.
     */
    sfTkError_t performSelfTest(bool &passed);

    /**
     * @brief Run a fast offset compensation.
     *
     * @details Averages 64 samples at 100 Hz / +/-4 g with the board STILL and @p gravityAxis
     * pointing up, computes the offsets that make that axis read exactly +1 g (or -1 g) and the
     * other two read 0 g, writes them to OFFSET_0..2 (0x71..0x73) and enables compensation. The
     * accelerometer configuration, enable state and advanced power save are restored afterwards.
     *
     * @p gravityAxis is in the REPORTED frame, so with an axis remap active it names the axis as
     * the application sees it; the driver translates it to the physical axis the offset registers
     * act on.
     *
     * @note Implemented here rather than by bma4_perform_accel_foc(), which needs 768 bytes of stack,
     * ignores the sign of the gravity axis, and mixes remapped and physical axes.
     *
     * @note The offsets are volatile - lost on reset or power-down. Save them with
     * getAccelOffset() and restore them with setAccelOffset() at boot.
     *
     * @param gravityAxis Which axis points up during the measurement.
     * @return `ksfTkErrOk` on success, `ksfTkErrFail` if the part stopped producing samples.
     */
    sfTkError_t performAccelFOC(sfe_bma456_foc_axis_t gravityAxis);

    /**
     * @brief Write the user offsets and enable compensation.
     * @param x X offset, 3.9 mg per LSB, signed. Range +/-500 mg.
     * @param y Y offset.
     * @param z Z offset.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t setAccelOffset(int8_t x, int8_t y, int8_t z);

    /**
     * @brief Read the user offsets.
     * @param x Receives the X offset, 3.9 mg per LSB.
     * @param y Receives the Y offset.
     * @param z Receives the Z offset.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t getAccelOffset(int8_t &x, int8_t &y, int8_t &z);

    /**
     * @brief Enable or disable offset compensation (NV_CONF acc_off_en).
     * @param enable true to apply the offsets to the data.
     * @return `ksfTkErrOk` on success.
     */
    sfTkError_t enableOffsetCompensation(bool enable = true);

    ///////////////////////////////////////////////////////////////////////////
    // Constants
    ///////////////////////////////////////////////////////////////////////////

    /** @brief Bytes in one headerless FIFO frame (X, Y, Z as int16). */
    static const uint8_t kFifoFrameBytes = 6;

    /** @brief FIFO capacity, in bytes. */
    static const uint16_t kFifoSizeBytes = 1024;

    /**
     * @brief Largest payload moved in one bus transaction.
     *
     * @details Thirty, and even, for two reasons. A write is the register byte plus the payload, and
     * the AVR Wire buffer is 32 bytes, so 31 is the ceiling; the feature-engine window takes whole
     * 16-bit words, so the payload must be even. This is also Bosch's read_write_len, so the
     * SensorAPI splits feature-page accesses into 30-byte pieces by itself.
     */
    static const uint8_t kMaxTransferBytes = 30;

  protected:
    /**
     * @brief Microsecond delay used by the Bosch SensorAPI.
     *
     * @details The Toolkit only offers millisecond delays, so the default rounds up to whole
     * milliseconds. The Arduino wrappers override this with delayMicroseconds(), which matters: the
     * configuration upload issues ~600 of the 450 us post-write delays.
     *
     * @param us Delay in microseconds.
     */
    virtual void delayUs(uint32_t us);

    /** @brief Bus supplied by begin() or setCommunicationBus(). */
    sfTkIBus *_theBus;

  private:
    // Bosch SensorAPI callbacks. intf_ptr is the sfDevBMA456 instance.
    static int8_t boschRead(uint8_t reg, uint8_t *data, uint32_t len, void *intfPtr);
    static int8_t boschWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *intfPtr);
    static void boschDelayUs(uint32_t period, void *intfPtr);

    // Raw register access through the Toolkit, split into I2C-sized pieces where that is safe.
    sfTkError_t busRead(uint8_t reg, uint8_t *data, uint32_t len);
    sfTkError_t busWrite(uint8_t reg, const uint8_t *data, uint32_t len);

    // Map a Bosch result onto a Toolkit result, remembering the Bosch code.
    sfTkError_t bosch(int8_t result);

    // Chip ID check, reset, variant init, configuration upload, accelerometer on.
    sfTkError_t initialize(void);

    // Stream the feature-engine configuration blob into the device.
    sfTkError_t loadFeatureConfig(void);

    // Apply the host-side axis remap to one sample.
    void applyRemap(sfe_bma456_raw_data_t &data);

    // Translate between library feature/interrupt bits and the loaded set's register bits.
    sfTkError_t sourceToBoschMap(sfe_bma456_int_source_t source, uint16_t &map);
    uint16_t boschStatusToLibrary(uint16_t boschStatus);

    // True if the loaded feature set is the one required.
    bool haveFeatureSet(sfe_bma456_feature_set_id_t required)
    {
        return _featureSetId == required;
    }

    /** @brief Bosch SensorAPI device structure. */
    struct bma4_dev _dev;

    /** @brief The feature set chosen at begin(). */
    const sfe_bma456_feature_set_t *_featureSet;

    /** @brief Cached ID of the feature set, for the variant dispatch. */
    sfe_bma456_feature_set_id_t _featureSetId;

    /** @brief Cached full-scale range for raw-to-g conversion. */
    sfe_bma456_range_t _range;

    /** @brief Whether auto low power is enabled - needed to work around a Bosch BMA456MM setter. */
    bool _autoLowPowerEnabled;

    /** @brief Result of the most recent Bosch call. */
    int8_t _lastBoschResult;
};
