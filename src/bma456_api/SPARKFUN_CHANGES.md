# Bosch BMA456 SensorAPI - SparkFun Changes

This folder holds part of Bosch Sensortec's [BMA456 SensorAPI](https://github.com/boschsensortec/BMA456_SensorAPI),
**v2.29.0** (commit `3266db2`, February 2024), released under the BSD-3-Clause license in [LICENSE](LICENSE).
Only the files this library needs are included: the common `bma4` core and the `bma456h` and
`bma456mm` variants.

The files are unmodified except for the changes below. Each one is marked with a `SparkFun` comment
in the source, so `grep -n SparkFun *.c *.h` lists them all.

| File | Change | Why |
| ---- | ------ | --- |
| `bma456h.c`, `bma456mm.c` | Configuration array tagged `BMA4_CONFIG_FILE_STORAGE`, defined as `PROGMEM` on AVR and empty everywhere else. | An AVR copies initialized `const` data into SRAM at start-up. 6144 bytes does not fit in the 2048 bytes of an ATmega328P. |
| `bma456h.c`, `bma456mm.c` | Added `bma456h_config_file_size` / `bma456mm_config_file_size`. | The driver needs the blob size from outside the file that defines the array. |
| `bma4_defs.h` | `BMA4_MS_TO_US(X)` changed from `(X * 1000)` to `((uint32_t)(X) * 1000)`. | With a 16-bit `int`, `BMA4_MS_TO_US(50)` overflows to a negative number, which becomes a delay of about 71 minutes inside `bma4_perform_accel_selftest()`. |
| `bma4.c` | `convert_lsb_g()` multiplies by 1000 before dividing by LSB/g instead of after. | Dividing first truncates to whole g, so the 1800 mg self-test limit really required 2000 mg and a healthy part at 1.8-1.99 g failed. The `int32_t` operands (max about 65536 x 1000) do not overflow. |
| `bma4.c` | `bma4_get_sensor_time()` and the FIFO sensor-time parser cast each byte to `uint32_t` before shifting it left by 16. | With a 16-bit `int` the top byte of the timestamp is shifted out, so the time wraps every 2.56 s instead of every 655 s. |

## Consequences

- **Do not call `bma456h_write_config_file()` or `bma456mm_write_config_file()` on AVR.** They hand
  the address of the configuration array to the bus callback as if it were in RAM. The SparkFun
  driver uploads the configuration itself (`sfDevBMA456::loadFeatureConfig()`) and never calls them.
- On every non-AVR target the configuration arrays are byte-for-byte identical to Bosch's.

## Known issues worked around outside this folder

These are left as Bosch wrote them; the SparkFun driver avoids the affected calls instead.

- `bma4_get_temperature()` reads the signed temperature register into an unsigned byte, so readings
  below 23 &deg;C come back 256 &deg;C too high. `sfDevBMA456::getTemperature()` reads the register itself.
- `bma4_read_fifo_data()` reads the whole FIFO fill level into the caller's buffer regardless of the
  buffer's size. `sfDevBMA456::readFifoData()` reads at most the requested number of frames.
- `bma4_soft_reset()` performs the SPI re-select read immediately after the reset command, before
  the device has come back. `sfDevBMA456` waits out the reset first.
- `bma4_perform_accel_foc()` keeps 128 samples on the stack (768 bytes), ignores the sign of the
  gravity axis, and averages remapped data while writing offsets in physical axis order.
  `sfDevBMA456::performAccelFOC()` is a native implementation.
- `bma456mm_set_auto_low_power_config()` masks the time-out with `0x1FFC`, which overlaps the
  auto-low-power enable bit (bit 12 of the word at feature offset `0x2C`) - any write clears the
  enable, and a time-out of 1024 or more sets it. `sfDevBMA456::setAutoLowPowerConfig()` limits the
  MM time-out to 1023 and restores the enable bit.
- Bosch picks the gap after each register write (2 us or 450 us) from the ACC_CONF performance bit,
  which does not reflect suspend, averaging mode or changes made by auto low power.
  `sfDevBMA456` resets `perf_mode_status` after every Bosch call so the 450 us gap always applies.
- `BMA4_ACCEL_DATA_RDY_INT` is the data-ready bit in the *status* register; routing data-ready to a
  pin with `bma4_map_interrupt()` needs `BMA4_DATA_RDY_INT`. The driver uses each in its place.

## Updating

To move to a newer SensorAPI release, copy the same seven files from Bosch's repository, then
re-apply the changes in the table above.
