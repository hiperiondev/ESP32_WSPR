/*
 * Copyright 2026 Emiliano Augusto Gonzalez (egonzalez . hiperion @ gmail . com))
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

#include <time.h>

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file time_sync.h
 * @brief UTC time synchronisation subsystem (NTP or GPS, selected via Kconfig).
 *
 * This module abstracts two time sources behind a single API:
 *
 * @par NTP mode (@c CONFIG_WSPR_TIME_NTP)
 * Uses the ESP-IDF SNTP client (@c esp_sntp) to periodically query an NTP
 * server over the Wi-Fi connection.  The system wall clock (@c settimeofday /
 * @c gettimeofday) is updated automatically by the SNTP callback.  Typical
 * accuracy: 1–50 ms.
 *
 * @par GPS mode (@c CONFIG_WSPR_TIME_GPS)
 * Reads NMEA-0183 @c $GPRMC / @c $GNRMC sentences from a UART-connected GPS
 * receiver.  The @c TZ environment variable is forced to @c "UTC0" before the
 * first @c mktime() call so that the NMEA UTC timestamps are interpreted
 * correctly regardless of the system locale.  Typical accuracy: ±1 s (limited
 * by 1 Hz NMEA sentence rate and UART latency).
 *
 * @par Selecting the time source
 * Set @c CONFIG_WSPR_TIME_NTP or @c CONFIG_WSPR_TIME_GPS in menuconfig.
 * Exactly one must be selected; the compilation fails with @c #error otherwise.
 *
 * @par Internal synchronisation flag
 * A module-level @c volatile @c bool @c _synced is set to @c true when the
 * first valid time update is received (NTP callback or valid RMC sentence).
 * All public query functions return @c false / 0 until this flag is set.
 *
 * @par Thread safety
 * @ref time_sync_init() must be called from a single task during startup.
 * All other functions are safe to call from any task concurrently; they
 * read the volatile @c _synced flag and delegate to @c gettimeofday() which
 * is reentrant on ESP-IDF.
 */

/**
 * @defgroup time_sync_api Time synchronisation API
 * @{
 */

/**
 * @brief Initialise the time synchronisation subsystem.
 *
 * In NTP mode, configures and starts the SNTP client targeting @p ntp_server.
 * In GPS mode, configures the UART peripheral, forces the @c TZ environment
 * variable to @c "UTC0", and launches a background task that parses incoming
 * NMEA sentences.
 *
 * @note Call this function exactly once at startup, after Wi-Fi is connected
 *       (for NTP mode) or after the UART pins are free (for GPS mode).
 *
 * @param[in] ntp_server  Hostname or IP address of the NTP server to query.
 *                        Used only in @c CONFIG_WSPR_TIME_NTP mode; ignored
 *                        (and may be @c NULL) in GPS mode.
 *                        If @c NULL in NTP mode, @c "pool.ntp.org" is used.
 *
 * @return @c ESP_OK on success, or a driver error code on UART / SNTP
 *         initialisation failure.
 */
esp_err_t time_sync_init(const char *ntp_server);

/**
 * @brief Check whether the system clock is currently synchronised.
 *
 * Returns @c true as soon as the first successful NTP update or valid GPS RMC
 * sentence has been processed.  The flag is never cleared after it is set, so
 * this function returns @c true for the lifetime of the session once sync
 * occurs.
 *
 * @return @c true  — at least one successful time synchronisation has occurred.
 * @return @c false — no synchronisation yet; the system clock should not be
 *                   trusted for WSPR scheduling.
 */
bool time_sync_is_ready(void);

/**
 * @brief Block the calling task until the clock is synchronised.
 *
 * Polls @ref time_sync_is_ready() every 500 ms.  If @p timeout_ms is zero the
 * function waits indefinitely.  Used in @c scheduler_task() at startup to
 * prevent the transmitter from firing before a valid UTC timestamp is
 * available.
 *
 * @param[in] timeout_ms  Maximum time to wait in milliseconds.
 *                        Pass @c 0 to wait forever.
 *
 * @return @c true  — synchronised within the timeout window.
 * @return @c false — timeout elapsed before synchronisation occurred.
 *                   (Never returned when @p timeout_ms is 0.)
 */
bool time_sync_wait(uint32_t timeout_ms);

/**
 * @brief Retrieve the current UTC time.
 *
 * A thin wrapper around @c gettimeofday() that also checks whether the clock
 * has been synchronised.  If not yet synchronised, the output struct is not
 * modified and @c false is returned.
 *
 * @param[out] tv  Pointer to a @c struct @c timeval to fill with the current
 *                 time.  The @c tv_usec field provides microsecond resolution
 *                 (accuracy is source-dependent).  Must not be @c NULL.
 *
 * @return @c true  — @p tv contains a valid synchronised UTC timestamp.
 * @return @c false — clock not yet synchronised; @p tv is unchanged.
 */
bool time_sync_get(struct timeval *tv);

/**
 * @brief Compute the number of seconds until the next WSPR transmit slot.
 *
 * WSPR transmissions begin at even UTC minute boundaries plus one second
 * (i.e. HH:MM:01 where MM is even).  The one-second offset gives the
 * oscillator and low-pass filter relay time to settle before the first symbol
 * is emitted.
 *
 * The formula is:
 * @code
 *   next = (now - (now % 120)) + 120 + 1
 *   return (int32_t)(next - now)
 * @endcode
 *
 * This function reads the wall clock unconditionally and does not check
 * @ref time_sync_is_ready().  The caller (@c scheduler_task in main.c)
 * gates on @ref time_sync_wait() before calling this function, so the clock
 * is always valid at the call site.
 *
 * @return Positive integer — seconds remaining until the next slot opens.
 *         The value is typically in the range 1–120.  A value of 1 means the
 *         slot boundary is imminent.
 */
int32_t time_sync_secs_to_next_tx(void);

/** @} */
