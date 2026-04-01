/**
 * @file web_server.h
 * @brief Embedded HTTP configuration and status server.
 * @copyright 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module runs a lightweight single-task HTTP server built on the ESP-IDF
 * @c httpd component.  It serves a single-page application (SPA) that exposes
 * the complete WSPR transmitter configuration and provides real-time status
 * updates.  All HTML, CSS, and JavaScript are embedded as compile-time string
 * literals in the firmware image; no filesystem or SPIFFS partition is required.
 *
 * @par REST endpoint summary
 * | Method | URI               | Description                                                 |
 * |--------|-------------------|-------------------------------------------------------------|
 * | GET    | @c /              | Serve the full SPA HTML page                                |
 * | GET    | @c /api/config    | Return the current configuration as a JSON object           |
 * | POST   | @c /api/config    | Parse a JSON body, validate, update, and persist config     |
 * | POST   | @c /api/tx_toggle | Toggle @c tx_enabled and return the new state as JSON       |
 * | GET    | @c /api/status    | Return a live status snapshot as a JSON object              |
 * | GET    | @c /api/wifi_scan | Trigger a blocking Wi-Fi scan; return AP list as JSON array |
 * | POST   | @c /api/reset     | Schedule an @c esp_restart() after a short delay            |
 *
 * @par HTTP Basic Authentication
 * When @c CONFIG_WSPR_HTTP_AUTH_ENABLE is defined, all endpoints require a valid
 * @c Authorization: Basic <base64> header.  Credentials are checked against
 * @c CONFIG_WSPR_HTTP_AUTH_USER and @c CONFIG_WSPR_HTTP_AUTH_PASS configured in
 * menuconfig.  Unauthenticated requests receive a 401 response with a
 * @c WWW-Authenticate header that triggers the browser's native credential dialog.
 *
 * @par Shared configuration access and concurrency model
 * The live @ref wspr_config_t pointed to by @ref web_server_start() is shared
 * by three concurrent tasks:
 *  - The @c httpd task (reads and writes configuration via the REST API).
 *  - @c scheduler_task (reads configuration to drive TX scheduling).
 *  - @c status_task (reads configuration to compute the status snapshot).
 *
 * All accesses are serialised through a FreeRTOS mutex exposed via
 * @ref web_server_cfg_lock() and @ref web_server_cfg_unlock().  Callers must
 * always hold this mutex while reading or writing any field of the shared
 * @ref wspr_config_t.  The mutex must not be held for longer than necessary
 * (in particular, not across blocking calls such as NVS writes or delay loops).
 *
 * @par Status snapshot and status mutex
 * A separate internal @c wspr_status_t structure caches the last-known runtime
 * state (time string, band name, frequency string, next TX countdown, TX active
 * flag, symbol index, hardware status, and boot information).  This snapshot is
 * written by @ref web_server_update_status() and @ref web_server_set_hw_status()
 * under a dedicated status mutex so that HTTP @c GET /api/status can read a
 * consistent view without blocking the scheduler.
 *
 * @par Initialisation order requirement
 * @ref web_server_start() creates both mutexes before returning.
 * @c scheduler_task and @c status_task must be created @em after
 * @ref web_server_start() returns so that their calls to
 * @ref web_server_cfg_lock() and @ref web_server_update_status() are safe.
 *
 * @par Language support
 * The HTML page is rendered using the string constants from @c webui_strings.h,
 * which selects the appropriate language header (@c webui_en.h or @c webui_es.h)
 * at compile time via the @c CONFIG_WEBUI_LANG_* Kconfig choice.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "config.h"

/**
 * @defgroup web_server_api Web server API
 * @{
 */

/**
 * @brief Start the HTTP configuration server.
 *
 * Creates the configuration mutex and status mutex, then calls
 * @c httpd_start() and registers all URI handlers listed in the endpoint
 * table above.  The server runs in its own task created internally by
 * @c httpd_start().
 *
 * @note @c scheduler_task and @c status_task must be created @em after this
 *       function returns, because they call @ref web_server_cfg_lock() and
 *       @ref web_server_update_status() which require the mutexes to exist.
 *
 * @param[in] cfg  Pointer to the live @ref wspr_config_t instance managed by
 *                 @c app_main().  The web server stores this pointer internally
 *                 and accesses the structure for the lifetime of the server.
 *                 The caller remains the owner; the web server never frees it.
 *                 Must not be @c NULL.
 *
 * @return @c ESP_OK on success.
 * @return @c ESP_ERR_NO_MEM if FreeRTOS mutex or @c httpd task allocation fails.
 * @return A @c httpd error code if @c httpd_start() fails.
 */
esp_err_t web_server_start(wspr_config_t *cfg);

/**
 * @brief Stop the HTTP server and release its resources.
 *
 * Calls @c httpd_stop() on the server handle.  The configuration and status
 * mutexes are @em not deleted; in practice this function is only called during
 * a controlled shutdown that is immediately followed by @c esp_restart().
 *
 * @return @c ESP_OK on success.
 * @return A @c httpd error code if @c httpd_stop() fails.
 */
esp_err_t web_server_stop(void);

/**
 * @brief Update the real-time status snapshot seen by GET /api/status.
 *
 * Called once per second from @c status_task() in @c main.c.  Acquires the
 * internal status mutex, copies all arguments into the status cache, and
 * releases the mutex.  The HTTP handler @c h_status() reads the cache under
 * the same mutex and serialises it to JSON, so the two operations never race.
 *
 * @param[in] time_ok      @c true if the system clock is synchronised;
 *                         @c false if time sync is still pending.
 * @param[in] time_str     NUL-terminated UTC time string, e.g. @c "14:23:05 UTC".
 *                         Displayed in the "Time synchronization" status row.
 *                         May be @c NULL; if so the field is cleared.
 * @param[in] band         NUL-terminated band name string, e.g. @c "20m".
 *                         Displayed in the "Current band" status row.
 *                         May be @c NULL; if so the field is cleared.
 * @param[in] freq_str     NUL-terminated frequency string, e.g. @c "14.0986 MHz".
 *                         Displayed in the "Frequency" status row.
 *                         May be @c NULL; if so the field is cleared.
 * @param[in] next_tx_sec  Seconds until the next WSPR TX window, or @c -1 if
 *                         the clock is not yet synchronised.
 * @param[in] tx_active    @c true if a WSPR symbol loop is currently executing.
 * @param[in] tx_enabled   @c true if the master TX enable flag is set.
 * @param[in] symbol_idx   Index of the WSPR symbol being transmitted (0–161),
 *                         or 0 when idle.  Displayed in the "Symbol" status row.
 */
void web_server_update_status(bool time_ok, const char *time_str, const char *band, const char *freq_str, int32_t next_tx_sec, bool tx_active, bool tx_enabled,
                              int symbol_idx);

/**
 * @brief Store the RF hardware status for inclusion in GET /api/status responses.
 *
 * Called once from @c app_main() after @ref oscillator_init() completes.
 * The @p hw_name string and @p hw_ok flag are copied into the status cache
 * under the status mutex and included in every subsequent @c GET /api/status
 * response.  They are displayed in the "RF Hardware" row of the web UI status
 * panel.
 *
 * @param[in] hw_ok    @c true if real oscillator hardware was detected
 *                     (@ref oscillator_hw_ok() returned @c true).
 *                     @c false if the firmware is running in dummy mode.
 * @param[in] hw_name  NUL-terminated hardware name string returned by
 *                     @ref oscillator_hw_name() (e.g. @c "Si5351A",
 *                     @c "AD9850 (assumed)", or @c "None (DUMMY)").
 *                     May be @c NULL; if so the field is cleared.
 */
void web_server_set_hw_status(bool hw_ok, const char *hw_name);

/**
 * @brief Store the last reboot wall-clock time and reset reason in the status cache.
 *
 * Both strings are included in every @c GET /api/status JSON response and
 * displayed as a subtitle line below the page title in the web UI once set.
 *
 * This function may be called multiple times.  A @c NULL argument leaves the
 * corresponding previously stored value unchanged, allowing the caller to update
 * the two fields independently:
 *  - At startup (@c app_main()), @p boot_time_str is @c NULL and @p reason_str
 *    is set to the human-readable @c esp_reset_reason() string.
 *  - After the first NTP/GPS synchronisation (@c status_task()), @p boot_time_str
 *    is computed from @c (now - uptime_seconds) and @p reason_str is @c NULL.
 *
 * Must be called after @ref web_server_start() (the status mutex must exist
 * before this function is invoked).  Safe to call from any task.
 *
 * @param[in] boot_time_str  Formatted UTC date/time string of the last boot,
 *                           e.g. @c "2026-03-01 12:34 UTC".
 *                           Pass @c NULL to leave the stored value unchanged.
 * @param[in] reason_str     Human-readable reset cause string,
 *                           e.g. @c "Power-on", @c "Software", @c "Watchdog".
 *                           Pass @c NULL to leave the stored value unchanged.
 */
void web_server_set_reboot_info(const char *boot_time_str, const char *reason_str);

/**
 * @brief Acquire the configuration mutex.
 *
 * Blocks the calling task until the configuration mutex is available, then
 * takes it.  Must be paired with a matching call to @ref web_server_cfg_unlock()
 * in every code path that exits the critical section (including error paths).
 *
 * @par Usage pattern
 * @code
 *   web_server_cfg_lock();
 *   // read or write fields of the shared wspr_config_t
 *   bool snap = g_cfg.tx_enabled;
 *   web_server_cfg_unlock();
 * @endcode
 *
 * @warning Do not hold this mutex across blocking operations (NVS writes,
 *          @c vTaskDelay(), I2C transactions, etc.) to avoid starving the
 *          @c httpd task and causing HTTP request timeouts.
 */
void web_server_cfg_lock(void);

/**
 * @brief Release the configuration mutex.
 *
 * Releases the mutex previously acquired by @ref web_server_cfg_lock().
 * Must be called exactly once for each successful call to
 * @ref web_server_cfg_lock() and must be called from the same task that
 * acquired the mutex.
 */
void web_server_cfg_unlock(void);

/** @} */
