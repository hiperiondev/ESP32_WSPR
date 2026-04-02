/**
 * @file time_sync.h
 * @brief UTC time synchronisation subsystem — auto-detected GPS or NTP fallback.
 * @copyright 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module provides GPS/NTP automatic detection: at init time it probes the
 * configured GPS UART for valid NMEA sentences during a short window
 * (CONFIG_GPS_DETECT_TIMEOUT_MS milliseconds).  If a valid sentence is received
 * the module enters GPS mode; otherwise it falls back to NTP over Wi-Fi.
 *
 * Both GPS and NTP drivers are compiled into the firmware unconditionally.
 * No compile-time source selection is required.  The active source can be
 * queried at runtime with time_sync_source().
 *
 * @par GPS mode
 * Reads NMEA-0183 $GPRMC/$GNRMC and $GPZDA/$GNZDA sentences from a UART-connected
 * GPS receiver.  When a valid sentence is received the system clock is updated via
 * settimeofday() and _synced is set to true.  The TZ environment variable is forced
 * to "UTC0" before the first mktime() call.
 * Typical accuracy: +-1 s (limited by NMEA 1 Hz rate and UART latency).
 *
 * @par Optional PPS support (GPS mode only)
 * When CONFIG_GPS_PPS_GPIO is set to a valid GPIO number (>= 0), a rising-edge ISR
 * fires on each PPS pulse and zeroes the sub-second wall-clock component.
 *
 * @par NTP mode (fallback)
 * Uses the ESP-IDF SNTP client to query the configured NTP server.  On the first
 * successful response the wall clock is updated and _synced is set to true.
 * Typical accuracy: 1-50 ms over a good Wi-Fi link.
 *
 * @par Detection timeout
 * The GPS probe window is CONFIG_GPS_DETECT_TIMEOUT_MS (default 3000 ms).
 * During this window the UART is read with a short timeout per iteration.
 * The window is kept short enough to not delay NTP-only boot significantly.
 *
 * @par Thread safety
 * time_sync_init() must be called from a single task during startup.
 * All other functions are safe to call from any task concurrently.
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

// Enumeration of the active time synchronisation source
typedef enum {
    TIME_SYNC_NONE = 0, // not yet determined (before init completes)
    TIME_SYNC_GPS = 1,  // GPS NMEA sentences (auto-detected on UART)
    TIME_SYNC_NTP = 2,  // NTP / SNTP over Wi-Fi (fallback)
} time_sync_source_t;

/**
 * @brief Initialise the time synchronisation subsystem with auto-detection.
 *
 * Probes the GPS UART (configured by CONFIG_GPS_* Kconfig symbols) for valid
 * NMEA sentences during CONFIG_GPS_DETECT_TIMEOUT_MS milliseconds:
 *  - If a valid sentence is found  -> GPS mode is activated; NTP is not started.
 *  - If no valid sentence is found -> NTP mode is started with ntp_server.
 *
 * In GPS mode the function also installs the UART driver, creates the gps_task,
 * and optionally installs the PPS ISR (when CONFIG_GPS_PPS_GPIO >= 0).
 *
 * In NTP mode the function starts the ESP-IDF SNTP client.
 *
 * The TZ environment variable is set to "UTC0" in both modes.
 *
 * @param[in] ntp_server  NTP server hostname or IPv4 address string.
 *                        Used only when the GPS probe fails (NTP fallback).
 *                        If NULL, "pool.ntp.org" is used.
 *                        Must remain valid for the lifetime of the SNTP session.
 *
 * @return ESP_OK on success.
 * @return A UART driver error code if UART initialisation fails.
 * @return A FreeRTOS or SNTP error code on task or client startup failure.
 */
esp_err_t time_sync_init(const char *ntp_server);

/**
 * @brief Restart the SNTP client with a new NTP server hostname.
 *
 * Only meaningful when the active source is TIME_SYNC_NTP.
 * Has no effect when GPS mode is active.
 * Called from the web server h_post_config() handler when the user saves a new
 * NTP server address through the web UI.
 *
 * @param[in] ntp_server  New NTP server hostname or IP address.
 *                        If NULL, "pool.ntp.org" is used.
 */
void time_sync_restart_ntp(const char *ntp_server);

/**
 * @brief Return the time synchronisation source selected by auto-detection.
 *
 * Returns TIME_SYNC_NONE before time_sync_init() completes, TIME_SYNC_GPS
 * when a GPS module was detected, or TIME_SYNC_NTP when the GPS probe failed
 * and the SNTP client was started instead.
 *
 * @return Active time_sync_source_t value.
 */
time_sync_source_t time_sync_source(void);

/**
 * @brief Check whether the system clock is currently synchronised.
 *
 * Returns true as soon as the first successful NTP update or the first valid GPS
 * RMC/ZDA sentence has been processed.  Never cleared once set.
 *
 * @return true  - clock is synchronised.
 * @return false - no synchronisation yet.
 */
bool time_sync_is_ready(void);

/**
 * @brief Block the calling task until the clock is synchronised.
 *
 * Polls time_sync_is_ready() every 500 ms.  If timeout_ms is zero the function
 * waits indefinitely.
 *
 * @param[in] timeout_ms  Maximum waiting time in ms. 0 = wait indefinitely.
 * @return true  - clock synchronised within the timeout window.
 * @return false - timeout_ms > 0 and timeout elapsed before sync.
 */
bool time_sync_wait(uint32_t timeout_ms);

/**
 * @brief Retrieve the current UTC wall-clock time.
 *
 * Thin wrapper around gettimeofday() that also checks _synced.
 *
 * @param[out] tv  Pointer to struct timeval to fill. Must not be NULL.
 * @return true  - tv filled with valid synchronised UTC time.
 * @return false - clock not yet synchronised; tv is unchanged.
 */
bool time_sync_get(struct timeval *tv);

/**
 * @brief Compute seconds until the next WSPR transmission slot.
 *
 * WSPR transmissions begin at second 1 of each even UTC minute.
 * Returns 0 when the current phase is within the TX window (seconds 0..4),
 * otherwise returns the seconds remaining until the next even minute.
 *
 * @return 0 if within the TX window.
 * @return Positive integer (1-120) seconds to next window.
 */
int32_t time_sync_secs_to_next_tx(void);

/** @} */
