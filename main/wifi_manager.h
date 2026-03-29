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

#include <stdbool.h>

#include "esp_err.h"

/**
 * @file wifi_manager.h
 * @brief Wi-Fi connection manager with STA / soft-AP fallback.
 *
 * This module manages the ESP32 Wi-Fi subsystem for the WSPR transmitter.
 * It implements a two-phase connection strategy:
 *
 * @par Phase 1 — Station (STA) mode
 * If @p sta_ssid is non-empty, the module attempts to connect to the named
 * access point.  Up to @c MAX_RETRY (5) connection attempts are made.  On
 * success the function returns @c ESP_OK with the assigned IP address
 * available via @ref wifi_manager_ip().
 *
 * @par Phase 2 — Soft-AP fallback
 * If the STA credentials are empty, or if all STA connection attempts fail
 * (e.g. wrong password, AP out of range), the module switches to soft-AP
 * mode broadcasting the SSID configured in @c CONFIG_WSPR_AP_SSID with
 * password @c CONFIG_WSPR_AP_PASS.  The fixed IP address is @c 192.168.4.1.
 *
 * @par Background STA reconnection
 * When operating in AP fallback mode with saved STA credentials, a periodic
 * @c esp_timer fires every 5 minutes and reattempts the STA connection.  If
 * the home AP comes back online, the device transitions to APSTA mode
 * (both AP and STA active) and eventually full STA mode.
 *
 * @par Wi-Fi scan
 * @ref wifi_manager_scan() performs a blocking scan and returns a
 * heap-allocated JSON array string describing nearby APs.  The web UI
 * calls the @c /api/wifi_scan endpoint to populate the SSID picker.
 *
 * @par Thread safety
 * @ref wifi_manager_start() must be called once from a single task during
 * startup.  @ref wifi_manager_sta_connected() and @ref wifi_manager_ip() read
 * module-level variables; they are safe to call from any task after startup
 * completes.  @ref wifi_manager_scan() is safe to call from any task but
 * performs a blocking scan of approximately 2 seconds.
 */

/**
 * @defgroup wifi_manager_api Wi-Fi manager API
 * @{
 */

/**
 * @brief Initialise the Wi-Fi subsystem and establish a network connection.
 *
 * This function is blocking: it waits up to 15 seconds for the STA
 * association to complete before falling back to AP mode.
 *
 * Execution sequence:
 *  1. Initialise the TCP/IP stack (@c esp_netif_init) and default event loop.
 *  2. Create default STA and AP netif objects.
 *  3. Register Wi-Fi and IP event handlers.
 *  4. If @p sta_ssid is non-empty:
 *     a. Configure and start the Wi-Fi driver in STA mode.
 *     b. Wait up to 15 s for @c IP_EVENT_STA_GOT_IP or @c STA_FAIL_BIT.
 *     c. On success, return @c ESP_OK (AP is not started).
 *     d. On failure, stop the Wi-Fi driver and fall through to AP mode.
 *  5. Start the soft-AP with the configured SSID / password.
 *  6. If STA credentials were provided, start the 5-minute reconnect timer.
 *
 * @param[in] sta_ssid  Station SSID to connect to.  Pass an empty string or
 *                      @c NULL to skip STA mode and start the AP immediately.
 *                      Maximum 32 characters (IEEE 802.11 limit).
 * @param[in] sta_pass  Station password.  Pass @c NULL or an empty string for
 *                      open networks.  Maximum 64 characters (WPA2 limit).
 *
 * @return @c ESP_OK on success (either STA connected or AP started).
 * @return @c ESP_FAIL if the default netif objects cannot be created.
 * @return An @c esp_err_t code if the event loop or Wi-Fi driver fails to
 *         initialise.
 */
esp_err_t wifi_manager_start(const char *sta_ssid, const char *sta_pass);

/**
 * @brief Query whether the device is currently connected to an AP in STA mode.
 *
 * Reflects the @c _sta_ok flag which is set to @c true on
 * @c IP_EVENT_STA_GOT_IP and cleared to @c false on
 * @c WIFI_EVENT_STA_DISCONNECTED.
 *
 * @return @c true  — STA is associated and has a valid IP address.
 * @return @c false — STA is disconnected; the device may be in AP mode.
 */
bool wifi_manager_sta_connected(void);

/**
 * @brief Return the current IP address of the device as a string.
 *
 * In STA mode, returns the DHCP-assigned address obtained from the AP.
 * In AP mode (fallback), returns the fixed gateway address @c "192.168.4.1".
 * Before any address is assigned the string is @c "0.0.0.0".
 *
 * The returned pointer points to a module-level buffer; it remains valid for
 * the lifetime of the application.  Do not free or modify it.
 *
 * @return NUL-terminated IPv4 address string (e.g. @c "192.168.1.42").
 */
const char *wifi_manager_ip(void);

/**
 * @brief Scan for nearby Wi-Fi access points and return a JSON array.
 *
 * Performs a blocking, passive scan across all channels (up to ~2 seconds).
 * Results are capped at 20 APs to keep the HTTP response payload small.
 * SSIDs containing double-quote or backslash characters are JSON-escaped.
 *
 * If the current Wi-Fi mode is AP-only, the mode is temporarily elevated to
 * APSTA to activate the STA interface required for scanning, then restored
 * to AP after the scan completes.
 *
 * @par JSON format
 * The returned string is a JSON array where each element is an object:
 * @code
 * [
 *   { "ssid": "MyNetwork", "rssi": -65, "auth": 1 },
 *   { "ssid": "OpenNet",   "rssi": -80, "auth": 0 }
 * ]
 * @endcode
 *  - @c ssid : AP network name (up to 32 characters, JSON-escaped).
 *  - @c rssi : Received signal strength in dBm (typically −30 to −100).
 *  - @c auth : @c 0 = open (no authentication); @c 1 = any security mode
 *               (WEP, WPA, WPA2, WPA3).
 *
 * @note The caller **must** @c free() the returned pointer when done.
 *
 * @return Heap-allocated NUL-terminated JSON array string on success.
 *         Returns a heap-allocated @c "[]" string (empty array) if no APs
 *         are found or the scan cannot be started.
 *         Returns @c NULL only on heap allocation failure.
 */
char *wifi_manager_scan(void);

/** @} */
