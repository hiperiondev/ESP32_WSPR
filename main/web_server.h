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

#include "config.h"
#include "esp_err.h"

/**
 * @file web_server.h
 * @brief Embedded HTTP configuration and status server.
 *
 * This module runs a lightweight HTTP server (ESP-IDF @c httpd) that exposes:
 *
 * @par REST endpoints
 * | Method | URI               | Description                              |
 * |--------|-------------------|------------------------------------------|
 * | GET    | @c /              | Single-page application HTML (no files)  |
 * | GET    | @c /api/config    | Return current config as JSON            |
 * | POST   | @c /api/config    | Update and persist config from JSON body |
 * | POST   | @c /api/tx_toggle | Toggle the TX enable flag                |
 * | GET    | @c /api/status    | Live status snapshot as JSON             |
 * | GET    | @c /api/wifi_scan | Trigger a Wi-Fi scan; return AP list     |
 * | POST   | @c /api/reset     | Schedule an @c esp_restart()             |
 *
 * @par Concurrency model
 * The live @ref wspr_config_t pointed to by @ref web_server_start() is shared
 * between the HTTP task, @c scheduler_task, and @c status_task.  Access is
 * serialised through a FreeRTOS mutex exposed via @ref web_server_cfg_lock()
 * and @ref web_server_cfg_unlock().
 *
 * A separate status mutex protects the string fields inside the internal
 * @c wspr_status_t snapshot that is updated every second by
 * @ref web_server_update_status() and read by @c h_status() on HTTP requests.
 *
 * @par Initialisation order
 * @ref web_server_start() creates both mutexes before returning.
 * @c scheduler_task and @c status_task must therefore be created **after**
 * @ref web_server_start() returns, so that calls to @ref web_server_cfg_lock()
 * and @ref web_server_update_status() from those tasks are safe.
 */

/**
 * @defgroup web_server_api Web server API
 * @{
 */

/**
 * @brief Start the HTTP configuration and status server.
 *
 * Performs the following steps:
 *  1. Creates the configuration mutex (@c _cfg_mutex) if not already created.
 *  2. Creates the status mutex (@c _status_mutex) if not already created.
 *  3. Starts the @c httpd server with a 10-handler table and an 8 KiB stack.
 *  4. Registers all REST URI handlers.
 *
 * The @p cfg pointer is stored internally and used by all subsequent HTTP
 * handler calls.  The caller retains ownership and must ensure the pointed-to
 * struct remains valid for the lifetime of the server.
 *
 * @note Must be called **before** @c scheduler_task and @c status_task are
 *       created (see module description).
 *
 * @param[in] cfg  Pointer to the live configuration struct.  The server will
 *                 read and write this struct under the config mutex on every
 *                 @c /api/config POST and TX toggle request.  Must not be
 *                 @c NULL.
 *
 * @return @c ESP_OK on success.
 * @return @c ESP_ERR_NO_MEM if a mutex could not be allocated.
 * @return @c ESP_FAIL if @c httpd_start() fails.
 */
esp_err_t web_server_start(wspr_config_t *cfg);

/**
 * @brief Stop the HTTP server and release its resources.
 *
 * Calls @c httpd_stop() on the server handle and clears the internal handle.
 * The configuration and status mutexes are not deleted; the server can be
 * restarted with @ref web_server_start() after this call.
 *
 * @return @c ESP_OK always (errors from @c httpd_stop() are ignored).
 */
esp_err_t web_server_stop(void);

/**
 * @brief Push a live status snapshot to the internal status cache.
 *
 * Called once per second from @c status_task (main.c).  All scalar fields are
 * written atomically with respect to the status mutex; all string fields are
 * copied under the mutex and explicitly NUL-terminated.
 *
 * The cached values are serialised to JSON and returned by @c h_status() on
 * each @c GET /api/status request.
 *
 * @param[in] time_ok      @c true when the system clock is synchronised via
 *                         NTP or GPS.
 * @param[in] time_str     Human-readable UTC time string (e.g.
 *                         @c "12:34:56 UTC").  Copied into a 24-byte internal
 *                         buffer; truncated silently if longer.  May be @c NULL
 *                         (field not updated).
 * @param[in] band         Current WSPR band name (e.g. @c "40m").  Copied into
 *                         an 8-byte internal buffer; truncated if longer.  May
 *                         be @c NULL.
 * @param[in] freq_str     Formatted RF frequency string (e.g.
 *                         @c "7.0416 MHz").  Copied into a 20-byte internal
 *                         buffer; truncated if longer.  May be @c NULL.
 * @param[in] next_tx_sec  Seconds until the next WSPR TX slot starts.
 *                         Negative or zero means a transmission is in progress
 *                         or the scheduler is disabled.
 * @param[in] tx_active    @c true while a WSPR transmission is in progress.
 * @param[in] tx_enabled   @c true when the TX scheduler is armed (master
 *                         enable flag from @ref wspr_config_t::tx_enabled).
 * @param[in] symbol_idx   Index of the WSPR symbol currently being transmitted
 *                         (0–161).  Meaningful only when @p tx_active is
 *                         @c true; set to 0 otherwise.
 */
void web_server_update_status(bool time_ok, const char *time_str, const char *band, const char *freq_str, int32_t next_tx_sec, bool tx_active, bool tx_enabled,
                              int symbol_idx);

/**
 * @brief Set the hardware detection result in the status cache.
 *
 * Called once from @c app_main() after @ref oscillator_init() completes,
 * before any tasks are started.  The values are included in every
 * @c GET /api/status response and displayed in the web UI hardware status row.
 *
 * @param[in] hw_ok    @c true when oscillator hardware was successfully
 *                     detected and initialised; @c false in dummy mode.
 * @param[in] hw_name  Human-readable chip name from @c oscillator_hw_name()
 *                     (e.g. @c "Si5351A" or @c "None (DUMMY)").  Copied into a
 *                     32-byte internal buffer.  May be @c NULL (field cleared).
 */
void web_server_set_hw_status(bool hw_ok, const char *hw_name);

/**
 * @brief Acquire the configuration mutex.
 *
 * Blocks the calling task until the internal @c _cfg_mutex is available, then
 * takes it.  Must be paired with exactly one call to @ref web_server_cfg_unlock()
 * in the same task.
 *
 * Used by @c scheduler_task and @c status_task in main.c to safely read or
 * write fields of the live @ref wspr_config_t without racing against HTTP
 * handler tasks.
 *
 * @note Do not call from an ISR context; @c xSemaphoreTake() is not
 *       ISR-safe.  If the mutex has not been created yet (before
 *       @ref web_server_start() returns), this function is a no-op.
 */
void web_server_cfg_lock(void);

/**
 * @brief Release the configuration mutex.
 *
 * Releases the @c _cfg_mutex previously acquired by @ref web_server_cfg_lock().
 * Calling this function without a prior successful @ref web_server_cfg_lock()
 * is undefined behaviour (FreeRTOS mutex invariant).
 *
 * @note If the mutex has not been created yet (before @ref web_server_start()
 *       returns), this function is a no-op.
 */
void web_server_cfg_unlock(void);

/** @} */
