/**
 * @file time_sync.h
 * @brief UTC time synchronisation subsystem — NTP or GPS, selected at build time.
 * @copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module abstracts two hardware-independent time sources behind a single
 * compile-time-selected API.  Exactly one source must be chosen in menuconfig;
 * the compilation fails with a @c #error directive if neither or both are selected.
 *
 * @par NTP mode (@c CONFIG_WSPR_TIME_NTP)
 * Uses the ESP-IDF SNTP client (@c esp_sntp) to periodically query the
 * configured NTP server over the active Wi-Fi connection.  On the first
 * successful server response the system wall clock is updated via
 * @c settimeofday() inside the SNTP callback and the internal @c _synced flag
 * is set to @c true.  The SNTP client continues polling in the background and
 * silently updates the system clock for the lifetime of the session.
 * Typical accuracy: 1–50 ms over a good Wi-Fi link.
 *
 * @par GPS mode (@c CONFIG_WSPR_TIME_GPS)
 * Reads NMEA-0183 sentences from a UART-connected GPS receiver and parses
 * @c $GPRMC / @c $GNRMC (RMC) and @c $GPZDA / @c $GNZDA (ZDA) sentences.
 * When a valid sentence is received, the system clock is updated via
 * @c settimeofday() and @c _synced is set to @c true.  The @c TZ environment
 * variable is forced to @c "UTC0" before the first @c mktime() call so that
 * NMEA UTC timestamps are interpreted correctly regardless of the system locale.
 * Typical accuracy: ±1 s (limited by the 1 Hz NMEA sentence rate and UART
 * receive latency).
 *
 * @par Optional PPS support (GPS mode only)
 * When @c CONFIG_GPS_PPS_GPIO is set to a valid GPIO number (≥ 0), a
 * rising-edge ISR fires on each PPS pulse and zeroes the sub-second component
 * of the wall clock.  This reduces the timing uncertainty from ~10 ms
 * (UART latency) to a few microseconds.  The NMEA sentence still provides the
 * correct UTC second value; PPS only improves sub-second accuracy.
 *
 * @par Selecting the time source
 * Open menuconfig, navigate to @em "WSPR Transmitter" > @em "Time synchronization
 * source", and select either @em "NTP (SNTP over WiFi)" or
 * @em "GPS (NMEA RMC/ZDA sentences via UART)".  Exactly one must be enabled.
 *
 * @par Internal synchronisation flag
 * A module-level @c volatile @c bool @c _synced is initialised to @c false and
 * set to @c true on the first successful time update.  The flag is never cleared
 * once set; subsequent poll cycles only update the wall clock.  All public query
 * functions return @c false or 0 until @c _synced is @c true.
 *
 * @par NTP server reconfiguration at runtime
 * In NTP mode the @ref time_sync_restart_ntp() function allows the NTP server
 * hostname to be changed at runtime (e.g. after the user updates it through the
 * web UI) without a device reboot.  This function is declared only when
 * @c CONFIG_WSPR_TIME_NTP is defined.  GPS mode has no equivalent because the
 * UART-based time source has no configurable server parameter.
 *
 * @par Thread safety
 * @ref time_sync_init() must be called from a single task during startup.
 * All other functions are safe to call from any task concurrently:
 * they read the @c volatile @c _synced flag and delegate to @c gettimeofday()
 * which is reentrant on ESP-IDF.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

/**
 * @defgroup time_sync_api Time synchronisation API
 * @{
 */

/**
 * @brief Initialise the time synchronisation subsystem.
 *
 * In @c CONFIG_WSPR_TIME_NTP mode:
 *  1. Sets the SNTP operating mode to @c SNTP_OPMODE_POLL.
 *  2. Sets the NTP server to @p ntp_server (or @c "pool.ntp.org" if @c NULL).
 *  3. Registers the internal synchronisation callback that sets @c _synced.
 *  4. Starts the SNTP client.
 *
 * In @c CONFIG_WSPR_TIME_GPS mode (the @p ntp_server parameter is ignored):
 *  1. Forces @c TZ = @c "UTC0" and calls @c tzset() to prevent timezone
 *     conversion of NMEA UTC timestamps.
 *  2. Installs and configures the UART driver on the port specified by
 *     @c CONFIG_GPS_UART_PORT at the baud rate @c CONFIG_GPS_BAUD_RATE,
 *     connecting to @c CONFIG_GPS_RX_GPIO and @c CONFIG_GPS_TX_GPIO.
 *  3. Creates a background FreeRTOS task (@c gps_task) that reads NMEA bytes
 *     from the UART, assembles complete sentences, validates their checksums,
 *     and sets the system wall clock from RMC or ZDA sentences.
 *  4. If @c CONFIG_GPS_PPS_GPIO >= 0, configures that GPIO as an input with a
 *     rising-edge interrupt and installs a fast ISR (@c pps_isr) that zeroes the
 *     sub-second wall-clock component on each pulse.
 *
 * @note Must be called exactly once during startup, after Wi-Fi is connected
 *       (for NTP mode) or after the UART pins are free (for GPS mode).
 *       In @c app_main() it is invoked via @c ESP_ERROR_CHECK(time_sync_init(...)).
 *
 * @param[in] ntp_server  NTP server hostname or IPv4/IPv6 address string.
 *                        Used only in NTP mode; silently ignored in GPS mode.
 *                        If @c NULL in NTP mode, @c "pool.ntp.org" is used.
 *                        Must remain valid until the SNTP client is stopped
 *                        (i.e. must point to a string with application lifetime,
 *                        such as the @c ntp_server field of @ref wspr_config_t).
 *
 * @return @c ESP_OK on success.
 * @return A UART driver error code if UART initialisation fails (GPS mode).
 * @return A FreeRTOS or SNTP error code on task or SNTP client startup failure.
 */
esp_err_t time_sync_init(const char *ntp_server);

#ifdef CONFIG_WSPR_TIME_NTP
/**
 * @brief Restart the SNTP client with a new NTP server hostname.
 *
 * Stops the current SNTP client, reconfigures the server name, re-registers
 * the synchronisation callback (which may be cleared by some ESP-IDF versions
 * on @c esp_sntp_stop()), and restarts the client.  The @c _synced flag is
 * @em not cleared: if the clock was already synchronised, it remains valid.
 *
 * This function is declared only when @c CONFIG_WSPR_TIME_NTP is defined.
 * GPS mode has no equivalent because the UART-based time source is always
 * active and does not use a configurable server hostname.
 *
 * Called from the web server @c h_post_config() handler when the user saves
 * a new NTP server address through the web UI, allowing the change to take
 * effect immediately without a device reboot.
 *
 * @param[in] ntp_server  New NTP server hostname or IP address.
 *                        If @c NULL, @c "pool.ntp.org" is used.
 *                        The string is consumed by @c esp_sntp_setservername()
 *                        and must remain valid for the lifetime of the SNTP
 *                        session (pointer stored internally by esp_sntp).
 */
void time_sync_restart_ntp(const char *ntp_server);
#endif

/**
 * @brief Check whether the system clock is currently synchronised.
 *
 * Returns @c true as soon as the first successful NTP update or the first
 * valid GPS RMC/ZDA sentence has been processed and the system wall clock has
 * been set.  The @c _synced flag is never cleared after it is set, so this
 * function returns @c true for the entire remaining session once synchronisation
 * occurs.
 *
 * The WSPR scheduler polls this function every second while waiting for the
 * first sync.  It is also used by the status task to populate the "Time
 * synchronization" field in the web UI.
 *
 * @return @c true  — at least one successful time synchronisation has occurred;
 *                   the wall clock can be trusted for WSPR scheduling.
 * @return @c false — no synchronisation yet; the wall clock value is unreliable.
 */
bool time_sync_is_ready(void);

/**
 * @brief Block the calling task until the clock is synchronised.
 *
 * Polls @ref time_sync_is_ready() every 500 ms.  If @p timeout_ms is zero
 * the function waits indefinitely.  Non-zero values cause the function to
 * return @c false if synchronisation does not occur within the timeout window.
 *
 * Used in @c scheduler_task() (main.c) at startup to prevent the transmitter
 * from firing before a valid UTC timestamp is available.  The scheduler calls
 * this function with a zero timeout to block indefinitely.
 *
 * @param[in] timeout_ms  Maximum waiting time in milliseconds.
 *                        Pass @c 0 to wait indefinitely (never times out).
 *
 * @return @c true  — clock synchronised within the timeout window.
 * @return @c false — @p timeout_ms > 0 and synchronisation did not occur
 *                   before the timeout elapsed.  Never returned when
 *                   @p timeout_ms is @c 0.
 */
bool time_sync_wait(uint32_t timeout_ms);

/**
 * @brief Retrieve the current UTC wall-clock time.
 *
 * A thin convenience wrapper around @c gettimeofday() that also checks
 * whether the clock has been synchronised at least once.  If @c _synced is
 * @c false, @p tv is left unmodified and @c false is returned, allowing the
 * caller to distinguish an unsynchronised clock from a successfully read value
 * of zero.
 *
 * @param[out] tv  Pointer to a @c struct @c timeval to fill.
 *                 On success, @c tv->tv_sec contains the UTC Unix timestamp
 *                 and @c tv->tv_usec contains the microsecond offset.
 *                 Accuracy is source-dependent (see module description).
 *                 Must not be @c NULL.
 *
 * @return @c true  — @p tv was filled with a valid synchronised UTC time.
 * @return @c false — clock not yet synchronised; @p tv is unchanged.
 */
bool time_sync_get(struct timeval *tv);

/**
 * @brief Compute the number of seconds until the next WSPR transmission slot.
 *
 * WSPR transmissions begin at even UTC-minute boundaries.  The first symbol
 * is emitted at exactly HH:MM:01 (second = 1) to allow the oscillator and
 * LPF relay one second to settle after being programmed.
 *
 * The function reads the wall clock with @c gettimeofday() and computes:
 * @code
 *   phase  = now_sec % 120        // position within the 2-minute cycle
 *   if (phase <= 3) return 0;     // already inside the TX window
 *   return 120 - phase;           // seconds until the next window
 * @endcode
 *
 * Returning 0 when @c phase <= 3 gives the scheduler a generous window to
 * detect the boundary and begin transmission without missing the slot.
 *
 * This function reads the wall clock unconditionally and does @em not check
 * @ref time_sync_is_ready().  The caller (@c scheduler_task in @c main.c)
 * gates on @ref time_sync_wait() before calling this function, ensuring the
 * clock is valid at the call site.
 *
 * @return @c 0 if the current time is within the first 3 seconds of an
 *         even-minute boundary (the TX window is open).
 * @return Positive integer (1–120) — seconds remaining until the next even
 *         UTC minute.
 */
int32_t time_sync_secs_to_next_tx(void);

/** @} */
