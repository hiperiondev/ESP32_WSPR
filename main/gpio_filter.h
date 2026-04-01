/**
 * @file gpio_filter.h
 * @brief 3-bit GPIO low-pass filter bank driver.
 * @copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module drives three ESP32 output-only GPIO pins that together form a
 * 3-bit binary address bus selecting one of up to eight low-pass filter (LPF)
 * sections in an external relay or solid-state switch bank.
 *
 * The typical hardware arrangement is a BCD-to-decimal decoder IC
 * (e.g. a 74HC138 demultiplexer or a CD4051 analog switch) connected between
 * the three GPIOs and a bank of relays, each relay switching in a different
 * LC low-pass filter optimised for a particular amateur band group.  Only the
 * selected filter is placed in the RF path; all others are bypassed.
 *
 * @par GPIO assignment
 * The three GPIO numbers are configured at build time via menuconfig
 * (@em "WSPR Transmitter" > @em "LPF select GPIO"):
 *  - @c CONFIG_FILTER_GPIO_A — address bit 0 (LSB).
 *  - @c CONFIG_FILTER_GPIO_B — address bit 1.
 *  - @c CONFIG_FILTER_GPIO_C — address bit 2 (MSB).
 *
 * A compile-time @c static_assert in @c gpio_filter.c verifies that all three
 * GPIOs are assigned to distinct pins.  Assigning the same GPIO to two or
 * more signals would create a bus conflict and must be corrected in menuconfig
 * before the project will compile.
 *
 * @par Address encoding
 * The 3-bit address word maps directly to GPIO levels:
 * @code
 *  filter_id  GPIO_C  GPIO_B  GPIO_A   Intended band group
 *      0         0       0       0     2200 m / 630 m
 *      1         0       0       1     160 m
 *      2         0       1       0     80 m / 60 m
 *      3         0       1       1     40 m
 *      4         1       0       0     30 m
 *      5         1       0       1     20 m / 17 m
 *      6         1       1       0     15 m / 12 m
 *      7         1       1       1     10 m
 * @endcode
 *
 * The mapping from WSPR band index to filter address is defined by the
 * @c BAND_FILTER[] table in @c config.c and should be adjusted to match
 * the relay wiring of the specific hardware LPF board in use.
 *
 * @par Relay settle time
 * After calling @ref gpio_filter_select(), the caller must wait for relay
 * contacts to settle mechanically before enabling the oscillator RF output.
 * In @c wspr_transmit() (main.c) this is achieved with a configurable delay
 * of @c CONFIG_WSPR_LPF_SETTLE_MS milliseconds (default 10 ms) inserted
 * between the GPIO write and @c oscillator_enable(true).  Solid-state
 * relays may need as little as 1–2 ms; mechanical relays typically need
 * 5–20 ms.  The settle time is set in menuconfig under
 * @em "WSPR Transmitter" > @em "LPF relay settle time (ms)".
 *
 * @par Thread safety
 * Both functions are safe to call from any FreeRTOS task or from an ISR
 * context.  @ref gpio_filter_select() calls only @c gpio_set_level(), which
 * is documented as ISR-safe by the ESP-IDF GPIO driver.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

/**
 * @defgroup gpio_filter_api GPIO filter bank API
 * @{
 */

/**
 * @brief Initialise the three filter-select GPIO pins as push-pull outputs.
 *
 * Configures @c CONFIG_FILTER_GPIO_A, @c CONFIG_FILTER_GPIO_B, and
 * @c CONFIG_FILTER_GPIO_C as output-only GPIO pins with pull resistors
 * disabled and edge interrupts disabled.
 *
 * After configuring the GPIO direction, the function immediately calls
 * @ref gpio_filter_select() with @p filter_id = 0 so that all address lines
 * are driven low and the relay board starts in a defined, reproducible state
 * before the oscillator is enabled.
 *
 * @note Must be called exactly once during @c app_main() initialisation, before
 *       any call to @ref gpio_filter_select() or to the oscillator driver.
 *       In @c app_main() it is invoked via @c ESP_ERROR_CHECK(gpio_filter_init()).
 *
 * @return @c ESP_OK on success.
 * @return A GPIO driver error code (e.g. @c ESP_ERR_INVALID_ARG) if any of
 *         the configured GPIO numbers are invalid for the ESP32 variant in use.
 */
esp_err_t gpio_filter_init(void);

/**
 * @brief Select a low-pass filter section by 3-bit address.
 *
 * Decomposes @p filter_id into its three constituent bits and drives
 * @c CONFIG_FILTER_GPIO_A, @c CONFIG_FILTER_GPIO_B, and @c CONFIG_FILTER_GPIO_C
 * to the corresponding levels:
 * @code
 *   GPIO_A = (filter_id >> 0) & 1   // bit 0 — LSB
 *   GPIO_B = (filter_id >> 1) & 1   // bit 1
 *   GPIO_C = (filter_id >> 2) & 1   // bit 2 — MSB
 * @endcode
 *
 * Any bits of @p filter_id above bit 2 are silently masked off with
 * @c (filter_id & 0x07), so the function always produces a valid 3-bit
 * address word regardless of the input value passed by the caller.
 *
 * @par Typical call site
 * In the WSPR scheduler (@c wspr_transmit() in @c main.c), this function is
 * called immediately before enabling the oscillator RF output:
 * @code
 *   gpio_filter_select(BAND_FILTER[g_band_idx]);
 *   vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
 *   oscillator_enable(true);
 * @endcode
 *
 * @param[in] filter_id  3-bit filter bank address (0–7).  Bits 3–7 are ignored.
 *                       Use the @c BAND_FILTER[] table from @c config.h to
 *                       convert a @ref wspr_band_t value to the correct address.
 *
 * @return @c ESP_OK always.  The function never returns an error because
 *         @c gpio_set_level() for a correctly configured output pin is always
 *         successful after @ref gpio_filter_init() has been called.
 */
esp_err_t gpio_filter_select(uint8_t filter_id);

/** @} */
