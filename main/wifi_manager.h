/**
 * @file wifi_manager.h
 * @brief Wi-Fi connection manager with STA mode and soft-AP fallback.
 *
 * @details
 * This module manages the ESP32 Wi-Fi subsystem for the WSPR transmitter.
 * It implements a two-phase connection strategy designed to guarantee
 * network availability for both web-based configuration access and NTP
 * time synchronisation.
 *
 * @par Phase 1 — Station (STA) mode
 * If @p sta_ssid is non-empty, the module configures the Wi-Fi driver in
 * @c WIFI_MODE_STA, associates with the named access point, and waits up to
 * 15 seconds for a DHCP IP address.  Up to @c MAX_RETRY (5) consecutive
 * association attempts are made before the phase is declared failed.  On
 * success, @ref wifi_manager_ip() returns the assigned IPv4 address and the
 * module returns @c ESP_OK without starting the soft-AP.
 *
 * @par Phase 2 — Soft-AP fallback
 * If the STA credentials are empty, or if all STA association attempts fail
 * (wrong password, AP out of range, DHCP timeout), the module switches to
 * @c WIFI_MODE_AP and broadcasts the SSID configured in @c CONFIG_WSPR_AP_SSID
 * with password @c CONFIG_WSPR_AP_PASS.  The static IP address of the soft-AP
 * interface is @c 192.168.4.1 (the ESP-IDF default for the AP netif).
 * When no password is configured or the password is shorter than 8 characters,
 * the soft-AP starts as an open network and a warning is logged.
 *
 * @par Background STA reconnection
 * When operating in soft-AP fallback mode with valid STA credentials saved,
 * a periodic @c esp_timer fires every 5 minutes and reattempts the STA
 * connection.  If the home access point returns online, the Wi-Fi mode is
 * elevated to @c WIFI_MODE_APSTA (both AP and STA simultaneously) and — once
 * an IP address is obtained — the @c _sta_ok flag is set and the reconnect
 * timer is stopped.  The soft-AP remains active throughout so that any
 * connected configuration clients are not disconnected.
 *
 * @par Concurrent Wi-Fi scan
 * @ref wifi_manager_scan() performs a blocking, passive scan across all 2.4 GHz
 * channels.  Because an AP-mode-only interface cannot scan without temporarily
 * activating the STA radio, the function elevates the mode to @c WIFI_MODE_APSTA
 * when necessary and restores @c WIFI_MODE_AP afterwards.  A FreeRTOS mutex
 * (@c _scan_mutex) prevents concurrent scan calls.  Results are capped at 20 APs
 * and hidden networks (empty SSID) are filtered out.
 *
 * @par Thread safety
 * @ref wifi_manager_start() must be called from a single task during startup.
 * @ref wifi_manager_sta_connected() and @ref wifi_manager_ip() read
 * module-level variables that are only written from the Wi-Fi event handler,
 * which runs in the ESP-IDF event-loop task; they are safe to call from any
 * task after startup completes.  @ref wifi_manager_scan() is safe to call from
 * any task but performs a blocking scan of approximately 2 seconds.
 *
 * @copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)
 * @par License
 * GNU General Public License v3 or later — see COPYING.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

/**
 * @defgroup wifi_manager_api Wi-Fi manager API
 * @{
 */

/**
 * @brief Initialise the Wi-Fi subsystem and establish a network connection.
 *
 * This function is partially blocking: it waits up to 15 seconds for a
 * STA association before falling back to soft-AP mode.
 *
 * Detailed execution sequence:
 *  1. Create the FreeRTOS event group and scan mutex.
 *  2. Call @c esp_netif_init() (TCP/IP stack initialisation, idempotent).
 *  3. Call @c esp_event_loop_create_default() (idempotent on subsequent calls).
 *  4. Create the default STA and AP netif objects via
 *     @c esp_netif_create_default_wifi_sta() and @c esp_netif_create_default_wifi_ap().
 *  5. Initialise the Wi-Fi driver with @c esp_wifi_init().
 *  6. If @p sta_ssid is non-empty, save it to the module-level reconnect buffers.
 *  7. Register handlers for @c WIFI_EVENT (any ID) and @c IP_EVENT_STA_GOT_IP.
 *  8. If @p sta_ssid is non-empty:
 *     a. Set @c WIFI_MODE_STA, configure credentials, call @c esp_wifi_start().
 *     b. Wait up to 15 s for @c STA_CONNECTED_BIT or @c STA_FAIL_BIT on the
 *        internal event group.
 *     c. If @c STA_CONNECTED_BIT: log the IP address and return @c ESP_OK.
 *     d. If @c STA_FAIL_BIT or timeout: call @c esp_wifi_stop(), fall through.
 *  9. Start the soft-AP (see @c start_ap() internal function).
 * 10. If STA credentials are available but STA failed, start the 5-minute
 *     background reconnect timer.
 *
 * @param[in] sta_ssid  SSID of the Wi-Fi access point to connect to.
 *                      Pass an empty string or @c NULL to skip STA mode and
 *                      start the soft-AP immediately.
 *                      Maximum 32 characters (IEEE 802.11 limit); longer values
 *                      are silently truncated.
 * @param[in] sta_pass  WPA2 pre-shared key for @p sta_ssid.
 *                      Pass @c NULL or an empty string for open networks.
 *                      Maximum 64 characters (WPA2 limit).
 *
 * @return @c ESP_OK on success — either STA connected and has an IP address, or
 *         the soft-AP was started successfully.
 * @return @c ESP_FAIL if the default STA or AP netif objects cannot be created
 *         (typically indicates a double-initialisation programming error).
 * @return A Wi-Fi or event-loop @c esp_err_t code if the underlying ESP-IDF
 *         driver fails to initialise.
 */
esp_err_t wifi_manager_start(const char *sta_ssid, const char *sta_pass);

/**
 * @brief Query whether the device is currently connected to an AP in STA mode.
 *
 * Reflects the internal @c _sta_ok flag:
 *  - Set to @c true in the @c IP_EVENT_STA_GOT_IP handler when a valid DHCP
 *    lease is obtained.
 *  - Cleared to @c false in the @c WIFI_EVENT_STA_DISCONNECTED handler.
 *
 * A return value of @c false does not necessarily mean the device is offline;
 * it may be operating as a soft-AP and reachable at @c 192.168.4.1.
 *
 * @return @c true  — the STA interface is associated and has a valid IP address.
 * @return @c false — the STA interface is disconnected; the device may be in
 *                   soft-AP fallback mode or transitioning between modes.
 */
bool wifi_manager_sta_connected(void);

/**
 * @brief Return the current IPv4 address of the device as a NUL-terminated string.
 *
 * The string is updated in the following circumstances:
 *  - On @c IP_EVENT_STA_GOT_IP: set to the DHCP-assigned address (e.g. @c "192.168.1.42").
 *  - When the soft-AP starts: set to @c "192.168.4.1".
 *  - At module initialisation: @c "0.0.0.0".
 *
 * The returned pointer references a module-level character array that remains
 * valid for the lifetime of the application.  The caller must not @c free() it
 * or modify it.
 *
 * @return NUL-terminated IPv4 address string in dotted-decimal notation.
 *         Examples: @c "192.168.1.42" (STA mode), @c "192.168.4.1" (AP mode),
 *         @c "0.0.0.0" (not yet initialised).
 */
const char *wifi_manager_ip(void);

/**
 * @brief Scan for nearby Wi-Fi access points and return a heap-allocated JSON array.
 *
 * Performs a blocking passive channel scan (approximately 1–2 seconds) and
 * returns the results as a heap-allocated JSON array string.  Results are:
 *  - Capped at 20 access points to keep the HTTP response payload small.
 *  - Filtered to exclude hidden networks (empty SSID byte at index 0).
 *  - SSID strings are JSON-escaped: any @c " or @c \\ character is prefixed with @c \\.
 *
 * If the current Wi-Fi mode is @c WIFI_MODE_AP, the mode is temporarily elevated
 * to @c WIFI_MODE_APSTA before starting the scan (the STA radio is required for
 * scanning) and restored to @c WIFI_MODE_AP after the scan completes.  A 300 ms
 * delay is inserted after the mode switch to allow the STA interface to stabilise.
 *
 * A FreeRTOS mutex (@c _scan_mutex) serialises concurrent scan requests.  If a
 * scan is already in progress, subsequent callers block for up to 3 seconds;
 * after that the new call returns an empty JSON array.
 *
 * @par JSON format
 * Each element of the returned array is a JSON object with three fields:
 * @code
 * [
 *   { "ssid": "HomeNetwork", "rssi": -58, "auth": 1 },
 *   { "ssid": "OpenCafe",    "rssi": -75, "auth": 0 }
 * ]
 * @endcode
 *  - @c "ssid" : AP network name (up to 32 bytes, JSON-escaped).
 *  - @c "rssi" : Received signal strength in dBm (typical range: −30 to −100).
 *  - @c "auth" : @c 0 = open network (no authentication); @c 1 = secured
 *                (any of WEP, WPA, WPA2, WPA3, or mixed modes).
 *
 * @note The caller is responsible for calling @c free() on the returned pointer
 *       when it is no longer needed.  The pointer is returned to the HTTP
 *       response handler in @c web_server.c which frees it immediately after
 *       sending the response.
 *
 * @return Heap-allocated NUL-terminated JSON array string on success.
 *         An empty JSON array @c "[]" is returned (as a heap allocation) when:
 *          - No access points are found after a successful scan.
 *          - @c esp_wifi_scan_start() fails (e.g. STA interface not ready).
 *          - Mode elevation to APSTA fails.
 *          - A concurrent scan call times out on the mutex.
 * @return @c NULL only if a @c malloc() call fails due to heap exhaustion.
 */
char *wifi_manager_scan(void);

/** @} */
