/**
 * @file oscillator.h
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @brief Runtime-detected RF oscillator driver — Si5351A or AD9850.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module provides a unified, hardware-agnostic interface to the RF signal
 * source used by the WSPR transmitter.  At initialisation time the module probes
 * the I2C bus for a Si5351A; if that probe succeeds the Si5351A is used.  If the
 * probe fails, the module unconditionally assumes an AD9850 is present (the AD9850
 * serial bus is write-only and cannot be read back to confirm physical presence).
 * If the Kconfig option @c CONFIG_OSCILLATOR_ASSUME_AD9850 is disabled, the module
 * enters a silent dummy mode when neither device is confirmed.
 *
 * @par Supported hardware
 * | Chip    | Interface         | Detection method              | Notes                              |
 * |---------|-------------------|-------------------------------|------------------------------------|
 * | Si5351A | I2C (400 kHz)     | I2C ACK probe on address 0x60 | Preferred; fractional-N PLL, high resolution |
 * | AD9850  | GPIO bit-bang SPI | Always assumed (write-only)   | DDS; maximum ~30 MHz output        |
 * | None    | —                 | Both probes failed            | Dummy mode; all calls are no-ops   |
 *
 * @par Si5351A details
 * The Si5351A is initialised with its PLL A locked to a VCO frequency that is a
 * multiple of the crystal frequency (25 MHz or 27 MHz by default, configurable in
 * menuconfig).  The MS0 output divider is set in integer mode for the base carrier
 * frequency; per-symbol tone offsets are applied by updating only the PLL A
 * fractional numerator (p2), avoiding the need to reset the PLL between symbols.
 * A band cache (@c si5351_band_cache_t) pre-computes the divider chain coefficients
 * once per carrier frequency change; within a WSPR window only the six PLL numerator
 * registers are rewritten (≈ 1 I2C transaction per symbol).
 *
 * @par AD9850 details
 * The AD9850 is driven via GPIO bit-bang in serial mode.  Frequency words are
 * computed from pre-scaled integer constants to avoid 64-bit division at runtime.
 * Phase continuity within a transmission window is maintained by not calling
 * @ref oscillator_enable() between symbols — the DDS accumulator continues running.
 * Note that @ref oscillator_enable(false) powers down the DAC output on the AD9850
 * but does @em not reset the phase accumulator; phase continuity is only guaranteed
 * within a single uninterrupted TX window.
 *
 * @par Frequency representation
 * The oscillator API uses a split integer / milli-Hz representation to avoid
 * 64-bit software multiplications on the Xtensa LX6 core:
 *  - @p base_hz         : carrier frequency in whole Hz (@c uint32_t, max ~28.1 MHz).
 *  - @p offset_millihz  : additional per-symbol tone offset in milli-Hz (@c int32_t).
 *
 * For WSPR the maximum symbol offset is 3 × 1 464.84375 mHz ≈ 4 395 mHz, which
 * fits comfortably within the @c int32_t range.  The WSPR tone spacing is exactly
 * 375 000 / 256 mHz ≈ 1 464.844 mHz.
 *
 * @par Crystal calibration
 * All drivers support a parts-per-billion (ppb) correction applied via
 * @ref oscillator_set_cal().  A positive ppb value indicates a fast crystal and
 * lowers the effective output frequency to compensate; a negative value raises it.
 * For the Si5351A the correction is applied to the PLL VCO target frequency.
 * For the AD9850 it scales the pre-computed frequency-to-tuning-word constants.
 *
 * @par TX window bracketing
 * @ref oscillator_tx_begin() and @ref oscillator_tx_end() bracket each WSPR
 * transmission window.  While the window is open, any call to
 * @ref oscillator_set_cal() is deferred rather than applied immediately, preventing
 * mid-symbol calibration changes from corrupting the symbol timing loop.  The
 * deferred calibration is applied atomically by @ref oscillator_tx_end().
 *
 * @par Thread safety
 * All functions are safe to call from a single task context.  The AD9850
 * bit-bang sequence is protected by a FreeRTOS critical section (@c portMUX_TYPE)
 * so it cannot be preempted by the second Xtensa core or by a task switch
 * mid-transfer.  The Si5351A uses the ESP-IDF I2C master driver which is
 * internally mutex-protected.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @defgroup oscillator_api Oscillator API
 * @{
 */

/**
 * @brief Probe for and initialise the RF oscillator hardware.
 *
 * Execution sequence:
 *  1. Attempt to initialise the Si5351A:
 *     a. Create the I2C master bus on the port and GPIO pins set in menuconfig.
 *     b. Add a device at address 0x60 and probe with a register-read.
 *     c. If the probe succeeds: configure CLK0 drive current, lock PLL A,
 *        and set the internal mode to @c OSC_SI5351.
 *  2. If the Si5351A probe fails (no ACK), fall through to the AD9850 path:
 *     a. If @c CONFIG_OSCILLATOR_ASSUME_AD9850 is enabled (default): configure
 *        the four AD9850 GPIO pins as outputs, issue a reset pulse, enter serial
 *        mode, and set the internal mode to @c OSC_AD9850.
 *     b. If @c CONFIG_OSCILLATOR_ASSUME_AD9850 is disabled: set the internal
 *        mode to @c OSC_NONE (dummy).
 *  3. In all cases return @c ESP_OK so that @c ESP_ERROR_CHECK() in
 *     @c app_main() does not abort the boot sequence when no oscillator
 *     hardware is fitted.
 *
 * @note Call this function exactly once at startup, before any other
 *       @c oscillator_* function.  Use @ref oscillator_hw_ok() after this
 *       call to determine which hardware, if any, was found.
 *
 * @return @c ESP_OK always.
 */
esp_err_t oscillator_init(void);

/**
 * @brief Set the oscillator output to an exact integer carrier frequency.
 *
 * Convenience wrapper around @ref oscillator_set_freq_mhz() with a zero
 * fractional offset.  Used to pre-program the carrier frequency before the
 * first WSPR symbol offset is applied.
 *
 * For the Si5351A, this call triggers a full band-cache recomputation
 * (@c si_cache_band()) which reprograms the MS0 output divider and PLL A.
 * If the same frequency is set twice in a row and the cache is still valid,
 * only the tone register is updated (tone = 0 mHz).
 *
 * For the AD9850, the frequency word is computed and written immediately.
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] freq_hz  Desired output frequency in Hz.  For the WSPR bands
 *                     this is the dial frequency + 1 500 Hz audio centre
 *                     offset, e.g. 14 098 600 Hz for 20 m.
 *                     Practical range: 137 600 Hz (2200 m dial + 1500 Hz)
 *                     to 28 127 600 Hz (10 m dial + 1500 Hz).
 *
 * @return @c ESP_OK on success.
 * @return @c ESP_ERR_INVALID_ARG if the Si5351A output divider or PLL
 *         multiplier falls outside the supported range for the given frequency.
 * @return An I2C error code if a register write fails on the Si5351A.
 */
esp_err_t oscillator_set_freq(uint32_t freq_hz);

/**
 * @brief Set the oscillator output frequency with a sub-Hz fractional tone offset.
 *
 * Called once per WSPR symbol (162 times per transmission) to apply the
 * per-symbol 4-FSK tone offset without floating-point arithmetic.  The
 * effective output frequency is:
 * @code
 *   f_out [Hz] = base_hz + offset_millihz / 1000
 * @endcode
 *
 * @par Si5351A implementation
 * The fractional offset is translated into an increment of the PLL A fractional
 * numerator (p2).  Only the six PLL numerator registers are rewritten; the MS0
 * divider and PLL denominator remain unchanged between symbols.  This provides
 * sub-Hz frequency resolution across the full WSPR tone range with a single
 * I2C transaction per symbol.
 *
 * @par AD9850 implementation
 * The milli-Hz offset is converted to an additional tuning-word increment using
 * the pre-scaled @c AD9850_FTW_FRAC_PER_MHZ_NUM / @c AD9850_FTW_FRAC_PER_MHZ_DEN
 * constants.  All arithmetic is performed in 32-bit integers; no 64-bit division
 * occurs at runtime.  The maximum accumulated rounding error over 162 symbols is
 * below 1 mHz.
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] base_hz          Carrier base frequency in Hz.  Must match the value
 *                             most recently passed to @ref oscillator_set_freq()
 *                             so that the Si5351A band cache is valid.
 * @param[in] offset_millihz   Per-symbol tone offset in milli-Hz.
 *                             Range for WSPR 4-FSK: 0, 1 465, 2 930, 4 395 mHz
 *                             (corresponding to tone symbols 0, 1, 2, 3).
 *                             Negative values are clamped to 0 on the Si5351A path.
 *
 * @return @c ESP_OK on success.
 * @return @c ESP_ERR_INVALID_STATE if the Si5351A band cache is not valid
 *         (i.e. @ref oscillator_set_freq() was not called beforehand).
 * @return An I2C error code if a register write fails on the Si5351A.
 * @return @c ESP_ERR_INVALID_ARG if the AD9850 extra-Hz calculation produces
 *         a negative intermediate value (should not occur with valid WSPR offsets).
 */
esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_millihz);

/**
 * @brief Enable or disable the RF output of the oscillator.
 *
 * @par Si5351A behaviour
 * Writes the Output Enable Control register (register 3):
 *  - Enable  (@p en = @c true)  : write @c 0xFE — CLK0 enabled, all others off.
 *  - Disable (@p en = @c false) : write @c 0xFF — all outputs disabled.
 * The PLL A and VCO remain locked in both states, so re-enabling the output
 * does not require a PLL reset and produces an immediate, phase-coherent output.
 *
 * @par AD9850 behaviour
 * The AD9850 control byte carries a power-down bit (bit 2 of byte W4):
 *  - Enable  (@p en = @c true)  : re-sends the last programmed frequency word
 *                                 with control byte @c 0x00 (power up).
 *  - Disable (@p en = @c false) : sends frequency word 0 with control byte
 *                                 @c 0x04 (power-down bit set), halting the DDS
 *                                 output but @em not resetting the phase accumulator.
 *
 * In dummy mode the call is a no-op and @c ESP_OK is returned.
 *
 * @param[in] en  @c true to enable RF output; @c false to disable it.
 *
 * @return @c ESP_OK on success.
 * @return An I2C error code if the Si5351A register write fails.
 */
esp_err_t oscillator_enable(bool en);

/**
 * @brief Apply a crystal frequency calibration correction in parts-per-billion.
 *
 * Stores the @p ppb offset and applies it on the next call to
 * @ref oscillator_set_freq() or @ref oscillator_set_freq_mhz().  If a WSPR
 * transmission is currently in progress (i.e. @ref oscillator_tx_begin() has
 * been called but @ref oscillator_tx_end() has not), the calibration update is
 * queued internally and applied by @ref oscillator_tx_end() after the
 * transmission completes.
 *
 * @par Si5351A calibration
 * The ppb correction is applied to the PLL A VCO target frequency before the
 * output divider chain is computed:
 * @code
 *   vco_cal = vco_nominal × (1 + ppb / 1e9)    [positive ppb: crystal fast]
 * @endcode
 * After applying the calibration, the Si5351A band cache is invalidated so
 * that the next call to @ref oscillator_set_freq() recomputes the divider chain
 * with the corrected VCO.
 *
 * @par AD9850 calibration
 * The ppb correction scales the pre-computed frequency-to-tuning-word constants:
 * @code
 *   ftw_per_mhz_cal = AD9850_FTW_PER_MHZ × (1 − ppb / 1e9)
 * @endcode
 * All frequency words computed after this call use the corrected constants.
 *
 * @note Must be called @em after @ref oscillator_init() for the correction to
 *       take effect.  In @c app_main() it is called immediately after
 *       @c oscillator_init() using the @c xtal_cal_ppb value loaded from NVS.
 *
 * @param[in] ppb  Crystal calibration offset in parts-per-billion.
 *                 Positive value: crystal runs fast — output frequency is lowered.
 *                 Negative value: crystal runs slow — output frequency is raised.
 *                 Practical range: ±100 000 ppb (±100 ppm).
 *                 Zero: no correction.
 *
 * @return @c ESP_OK always.  The value is stored unconditionally even in
 *         dummy mode or while a TX window is in progress (deferred).
 */
esp_err_t oscillator_set_cal(int32_t ppb);

/**
 * @brief Mark the start of a WSPR transmission window.
 *
 * Sets an internal @c _osc_tx_active flag to @c true.  While this flag is set,
 * calls to @ref oscillator_set_cal() do not apply the calibration immediately;
 * instead, the new ppb value is queued and applied by @ref oscillator_tx_end().
 *
 * Call this function immediately before @ref oscillator_enable(true) at the
 * beginning of each WSPR symbol loop.
 *
 * @note For the AD9850: calling @ref oscillator_enable(false) after
 * @ref oscillator_tx_begin() powers down the DAC output but does @em not reset
 * the phase accumulator.  Phase continuity within a single TX window is
 * maintained by not calling @ref oscillator_enable() between symbols.
 */
void oscillator_tx_begin(void);

/**
 * @brief Mark the end of a WSPR transmission window.
 *
 * Clears the internal @c _osc_tx_active flag to @c false.  If a calibration
 * update was deferred during the symbol loop (i.e. @ref oscillator_set_cal()
 * was called while @c _osc_tx_active was @c true), @ref oscillator_tx_end()
 * applies the deferred ppb value atomically before returning.
 *
 * For the Si5351A, applying the deferred calibration invalidates the band
 * cache (@c si5351_band_cache_t::valid = @c false) so that the next call to
 * @ref oscillator_set_freq() recomputes the full divider chain with the
 * corrected VCO frequency.
 *
 * Call this function immediately after @ref oscillator_enable(false) at the
 * end of each WSPR symbol loop.
 */
void oscillator_tx_end(void);

/**
 * @defgroup oscillator_detection Hardware detection API
 * @{
 */

/**
 * @brief Query whether real oscillator hardware was found during initialisation.
 *
 * Returns @c true when either the Si5351A or the AD9850 was successfully
 * initialised.  Returns @c false when @ref oscillator_init() entered dummy
 * mode because neither device was confirmed (i.e. @c CONFIG_OSCILLATOR_ASSUME_AD9850
 * was disabled and the Si5351A I2C probe failed).
 *
 * In dummy mode all @c oscillator_* functions succeed silently so the rest of
 * the firmware keeps running for development purposes.  This flag allows the
 * web UI to display a hardware warning banner when dummy mode is active.
 *
 * @return @c true  — real oscillator hardware is active (Si5351A or AD9850).
 * @return @c false — no hardware confirmed; running in dummy (no-op) mode.
 */
bool oscillator_hw_ok(void);

/**
 * @brief Return a human-readable string identifying the active oscillator.
 *
 * The returned pointer references a string literal stored in read-only flash;
 * it must not be @c free()'d or modified by the caller.
 *
 * | Active mode   | Returned string         |
 * |---------------|-------------------------|
 * | Si5351A found | @c "Si5351A"            |
 * | AD9850 found  | @c "AD9850 (assumed)"   |
 * | Dummy mode    | @c "None (DUMMY)"       |
 *
 * The string is passed to @c web_server_set_hw_status() in @c app_main() so
 * that the web UI status panel can display the active RF hardware type.
 *
 * @return Pointer to a NUL-terminated constant string in read-only flash.
 */
const char *oscillator_hw_name(void);

/** @} */
/** @} */
