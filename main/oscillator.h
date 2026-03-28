/*
 * Copyright 2025 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/ESP32_WSPR *
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @file oscillator.h
 * @brief Runtime-detected RF oscillator driver (Si5351A or AD9850).
 *
 * This module provides a unified, hardware-agnostic interface to the RF
 * signal source used by the WSPR transmitter.  During initialisation it
 * probes for a Si5351A over I2C; if that fails it falls back to an AD9850
 * (which is write-only and therefore always assumed present if configured);
 * if both fail it enters a silent dummy mode so the rest of the firmware
 * continues to run without crashing.
 *
 * @par Supported hardware
 * | Chip    | Interface | Detection           | Notes                        |
 * |---------|-----------|---------------------|------------------------------|
 * | Si5351A | I2C       | I2C ACK probe       | Preferred; fractional-N PLL  |
 * | AD9850  | GPIO SPI  | Always assumed      | DDS; write-only bus          |
 * | None    | —         | Both probes failed  | Dummy mode; all calls no-op  |
 *
 * @par Frequency representation
 * The oscillator API uses a split integer/milli-Hz representation to avoid
 * 64-bit software multiplications on the Xtensa LX6 core:
 *  - @p base_hz  : carrier frequency in whole Hz (uint32_t, max ~28.1 MHz).
 *  - @p offset_mhz : additional offset in **milli-Hz** (int32_t).
 *
 * For WSPR the maximum symbol offset is 3 × 1464.84375 mHz ≈ 4395 mHz,
 * well within the int32_t range.
 *
 * @par Crystal calibration
 * All drivers support a parts-per-billion (ppb) correction applied via
 * @ref oscillator_set_cal().  A positive ppb value indicates a fast crystal
 * and lowers the effective VCO / DDS frequency to compensate.
 *
 * @par Thread safety
 * All functions are safe to call from any single task.  The AD9850 bit-bang
 * sequence is protected by a FreeRTOS critical section (portMUX) so it cannot
 * be interrupted by the second Xtensa core or by a task switch mid-transfer.
 * The Si5351 uses the I2C driver which is internally mutex-protected.
 */

/**
 * @defgroup oscillator_api Oscillator API
 * @{
 */

/**
 * @brief Probe for and initialise the RF oscillator hardware.
 *
 * Execution sequence:
 *  1. Attempt to initialise the Si5351A via I2C (probe, PLL lock, CLK0 setup).
 *     On success, set internal mode to @c OSC_SI5351.
 *  2. If the Si5351 is absent, attempt to initialise the AD9850 via GPIO
 *     bit-bang (reset pulse, serial-mode entry).  Set mode to @c OSC_AD9850.
 *     Because the AD9850 bus is write-only, presence is *assumed*.
 *  3. If neither hardware responds, set mode to @c OSC_NONE (dummy).
 *
 * The function **always returns @c ESP_OK** regardless of which branch is
 * taken, so that @c ESP_ERROR_CHECK() in @c app_main() does not abort the
 * boot sequence when no oscillator hardware is present.  Use
 * @ref oscillator_hw_ok() after this call to determine whether real hardware
 * was found.
 *
 * @note Call this function exactly once at startup, before any other
 *       @c oscillator_* function.
 *
 * @return @c ESP_OK always.
 */
esp_err_t oscillator_init(void);

/**
 * @brief Set the oscillator output to an exact integer frequency.
 *
 * Convenience wrapper around @ref oscillator_set_freq_mhz() with a zero
 * fractional offset.  Typically used to pre-program the carrier before a
 * WSPR transmission begins (before the first symbol offset is applied).
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] freq_hz  Desired output frequency in Hz.  Must be in the range
 *                     supported by the oscillator hardware (typically
 *                     1 kHz – 30 MHz).  For WSPR the practical range is
 *                     137 600 Hz (2200 m) to 28 127 100 Hz (10 m) plus the
 *                     1 500 Hz audio centre offset.
 *
 * @return @c ESP_OK on success, or an error code if the Si5351 divider is
 *         out of range or an I2C transaction fails.
 */
esp_err_t oscillator_set_freq(uint32_t freq_hz);

/**
 * @brief Set the oscillator output frequency with a sub-Hz fractional offset.
 *
 * Used during WSPR symbol transmission to apply the per-symbol tone offset
 * without floating-point arithmetic.  The effective output frequency is:
 *
 * @code
 *   f_out = base_hz + offset_mhz / 1000  (Hz)
 * @endcode
 *
 * @par Si5351 implementation
 * The fractional part is fed into the Si5351 MS0 output divider numerator
 * (p2 / p3) which provides sub-Hz resolution across the full WSPR tone range.
 *
 * @par AD9850 implementation
 * The milli-Hz offset is added to the 32-bit DDS tuning word using the
 * pre-computed @c AD9850_SCALE_KHZ constant.  Maximum accumulated error over
 * 162 symbols is less than 1 mHz.
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] base_hz     Carrier base frequency in Hz (same value as passed
 *                        to @ref oscillator_set_freq()).
 * @param[in] offset_mhz  Symbol tone offset in **milli-Hz** (NOT MHz despite
 *                        the parameter name, which is a legacy artefact).
 *                        Range for WSPR: 0 to 3 × 1465 = 4395 mHz.
 *
 * @return @c ESP_OK on success, or an error code on driver failure.
 */
esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_mhz);

/**
 * @brief Enable or disable the RF output of the oscillator.
 *
 * @par Si5351 behaviour
 * Writes register 3 (Output Enable Control) to unmute (@c 0xFE, CLK0 on)
 * or mute (@c 0xFF, all outputs off) the CLK0 output.  The PLL and VCO
 * remain locked in both states.
 *
 * @par AD9850 behaviour
 * - Enable  : re-writes the last programmed frequency word with a power-up
 *             control byte (@c 0x00).
 * - Disable : writes frequency word 0 with the power-down bit set in the
 *             control byte (@c 0x04), halting the DDS output.
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] en  @c true to enable RF output; @c false to disable it.
 *
 * @return @c ESP_OK on success, or an I2C error code for the Si5351 case.
 */
esp_err_t oscillator_enable(bool en);

/**
 * @brief Apply a crystal frequency calibration correction.
 *
 * Stores the @p ppb offset and applies it in subsequent calls to
 * @ref oscillator_set_freq() and @ref oscillator_set_freq_mhz().
 *
 * @par Si5351 calibration
 * The ppb offset is applied to the internal VCO frequency before computing
 * the MS0 output divider: @c vco_cal = vco_nominal × (1 + ppb/1e9).  A
 * positive @p ppb value reduces the effective VCO (crystal runs fast,
 * output is lowered to compensate).
 *
 * @par AD9850 calibration
 * The ppb offset scales the target frequency before the 32-bit DDS tuning
 * word is computed: @c freq_cal = freq_nominal × (1 − ppb/1e9).
 *
 * @note Must be called **after** @ref oscillator_init() for the correction to
 *       take effect.  In @c app_main() it is called immediately after
 *       @c oscillator_init() using the @c xtal_cal_ppb value loaded from NVS.
 *
 * @param[in] ppb  Crystal calibration offset in parts-per-billion.
 *                 Positive = crystal fast (output lowered).
 *                 Negative = crystal slow (output raised).
 *                 Practical range: ±100 000 ppb (= ±100 ppm).
 *
 * @return @c ESP_OK always (value is stored unconditionally).
 */
esp_err_t oscillator_set_cal(int32_t ppb);

/**
 * @defgroup oscillator_detection Hardware detection API
 * @{
 */

/**
 * @brief Query whether oscillator hardware was found during initialisation.
 *
 * Returns @c true when either the Si5351A or the AD9850 was successfully
 * initialised.  Returns @c false when @ref oscillator_init() fell through to
 * dummy mode because neither device responded.
 *
 * In dummy mode all @c oscillator_* functions succeed silently so the rest of
 * the firmware keeps running; this flag allows the web UI to display a warning.
 *
 * @return @c true  — real oscillator hardware is active.
 * @return @c false — no hardware found; running in dummy (no-op) mode.
 */
bool oscillator_hw_ok(void);

/**
 * @brief Return a human-readable string identifying the active oscillator.
 *
 * The returned string is a pointer to a string literal in flash; do not free
 * or modify it.
 *
 * | Mode          | Returned string                |
 * |---------------|--------------------------------|
 * | Si5351A found | @c "Si5351A"                   |
 * | AD9850 found  | @c "AD9850 (assumed)"          |
 * | Dummy mode    | @c "None (DUMMY)"              |
 *
 * @return Pointer to a NUL-terminated constant string in read-only flash.
 */
const char *oscillator_hw_name(void);

/** @} */
/** @} */
