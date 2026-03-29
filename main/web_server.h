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
#include <stdint.h>

#include "esp_err.h"

#include "config.h"

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

esp_err_t web_server_start(wspr_config_t *cfg);
esp_err_t web_server_stop(void);

void web_server_update_status(bool time_ok, const char *time_str, const char *band, const char *freq_str, int32_t next_tx_sec, bool tx_active, bool tx_enabled,
                              int symbol_idx);

void web_server_set_hw_status(bool hw_ok, const char *hw_name);

// store last reboot wall-clock time and reset reason in status cache.
// boot_time_str: formatted UTC date/time string (e.g. "2026-03-01 12:34 UTC"),
//               or NULL to leave the previously stored value unchanged.
// reason_str:   human-readable reset cause (e.g. "Power-on", "Watchdog"),
//               or NULL to leave the previously stored value unchanged.
// Both fields are included in every GET /api/status response and displayed
// below the page title in the web UI once populated.
// Call after web_server_start() (mutex must exist). Safe from any task.
void web_server_set_reboot_info(const char *boot_time_str, const char *reason_str);

void web_server_cfg_lock(void);
void web_server_cfg_unlock(void);

/** @} */
