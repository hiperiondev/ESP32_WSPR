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

#include <stdint.h>

#include "esp_err.h"

/**
 * @file gpio_filter.h
 * @brief 3-bit GPIO low-pass filter bank driver.
 *
 * This module drives three ESP32 GPIO pins that together form a 3-bit binary
 * address bus selecting one of up to eight low-pass filter sections.  The
 * typical hardware arrangement is a BCD-decoder IC (e.g. a 74HC138 or
 * CD4051) connected to a relay or switch bank, with each relay selecting a
 * different LC low-pass filter optimised for a particular amateur band group.
 *
 * @par GPIO assignment
 * The three GPIO numbers are set at build time via menuconfig:
 *  - @c CONFIG_FILTER_GPIO_A — bit 0 (LSB) of the address bus.
 *  - @c CONFIG_FILTER_GPIO_B — bit 1 of the address bus.
 *  - @c CONFIG_FILTER_GPIO_C — bit 2 (MSB) of the address bus.
 *
 * A compile-time @c static_assert in @c gpio_filter.c guarantees that the
 * three GPIOs are not assigned to the same pin, which would create a
 * short circuit on the address bus.
 *
 * @par Address encoding
 * @code
 *  filter_id  GPIO_C  GPIO_B  GPIO_A   Selected filter
 *      0         0       0       0     filter 0 (e.g. 2200 m / 630 m)
 *      1         0       0       1     filter 1 (e.g. 160 m)
 *      2         0       1       0     filter 2 (e.g. 80 m / 60 m)
 *      3         0       1       1     filter 3 (e.g. 40 m)
 *      4         1       0       0     filter 4 (e.g. 30 m)
 *      5         1       0       1     filter 5 (e.g. 20 m / 17 m)
 *      6         1       1       0     filter 6 (e.g. 15 m / 12 m)
 *      7         1       1       1     filter 7 (e.g. 10 m)
 * @endcode
 *
 * The mapping from WSPR band to filter index is defined by the
 * @c BAND_FILTER[] table in @c config.c and can be adjusted to suit any
 * hardware LPF board layout.
 *
 * @par Thread safety
 * Both functions are safe to call from any task or ISR context.
 * @c gpio_filter_select() only calls @c gpio_set_level() which is documented
 * as ISR-safe in the ESP-IDF GPIO driver.
 */

/**
 * @defgroup gpio_filter_api GPIO filter bank API
 * @{
 */

/**
 * @brief Initialise the three filter-select GPIO pins as push-pull outputs.
 *
 * Configures @c CONFIG_FILTER_GPIO_A, @c CONFIG_FILTER_GPIO_B, and
 * @c CONFIG_FILTER_GPIO_C as output-only GPIO pins with no internal pull
 * resistors and interrupts disabled.  After configuration the function calls
 * @ref gpio_filter_select() with @c filter_id = 0 to drive all address lines
 * low and ensure the relay board starts in a defined state.
 *
 * @note Must be called once during @c app_main() initialisation, before any
 *       call to @ref gpio_filter_select().
 *
 * @return @c ESP_OK on success, or a GPIO driver error code on failure.
 */
esp_err_t gpio_filter_init(void);

/**
 * @brief Select a low-pass filter section by 3-bit address.
 *
 * Decomposes @p filter_id into its three constituent bits and drives
 * @c GPIO_A, @c GPIO_B, and @c GPIO_C accordingly:
 *
 * @code
 *   GPIO_A = (filter_id >> 0) & 1   // bit 0, LSB
 *   GPIO_B = (filter_id >> 1) & 1   // bit 1
 *   GPIO_C = (filter_id >> 2) & 1   // bit 2, MSB
 * @endcode
 *
 * The upper bits of @p filter_id above bit 2 are silently masked off, so the
 * function always produces a valid 3-bit address regardless of the input value.
 *
 * @note In @c wspr_transmit() (main.c) a 10 ms @c vTaskDelay() is inserted
 *       after this call to allow relay contacts to settle mechanically before
 *       the oscillator output is enabled.
 *
 * @param[in] filter_id  3-bit filter bank address (0–7).  Bits above bit 2
 *                       are ignored.
 *
 * @return Always returns @c ESP_OK.
 */
esp_err_t gpio_filter_select(uint8_t filter_id);

/** @} */
