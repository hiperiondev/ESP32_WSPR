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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// ---------------------------------------------------------------------------
// Schema version
// ---------------------------------------------------------------------------

#define CONFIG_SCHEMA_VERSION 4u

// ---------------------------------------------------------------------------
// String field size constants
// ---------------------------------------------------------------------------

#define CALLSIGN_LEN 12
#define LOCATOR_LEN  7

// ---------------------------------------------------------------------------
// IARU Region enumeration  -- ADDED
// ---------------------------------------------------------------------------

// ITU/IARU administrative regions that govern the 60 m WSPR dial frequency.
// All other amateur bands use identical dial frequencies across all regions.
// Reference: IARU Band Plans and WSPRnet frequency coordination.
typedef enum {
    IARU_REGION_1 = 1, // Europe, Africa, Middle East -- 60 m: 5 288 600 Hz
    IARU_REGION_2 = 2, // Americas (North/South/Caribbean) -- 60 m: 5 346 500 Hz
    IARU_REGION_3 = 3, // Asia, Pacific, Oceania -- 60 m: 5 367 000 Hz
} iaru_region_t;

// ---------------------------------------------------------------------------
// Band enumeration
// ---------------------------------------------------------------------------

typedef enum {
    BAND_2200M = 0,
    BAND_630M,
    BAND_160M,
    BAND_80M,
    BAND_60M,
    BAND_40M,
    BAND_30M,
    BAND_20M,
    BAND_17M,
    BAND_15M,
    BAND_12M,
    BAND_10M,
    BAND_COUNT
} wspr_band_t;

static_assert(BAND_COUNT == 12, "BAND_COUNT changed; update all band tables");
extern const uint32_t BAND_FREQ_HZ[3][BAND_COUNT];

static inline uint32_t config_band_freq_hz(iaru_region_t region, wspr_band_t band) {
    int idx = (int)region - 1;
    if (idx < 0 || idx > 2)
        idx = 0; // default to Region 1
    return BAND_FREQ_HZ[idx][(int)band];
}

extern const char *BAND_NAME[BAND_COUNT];
extern const uint8_t BAND_FILTER[BAND_COUNT];

// ---------------------------------------------------------------------------
// Persistent configuration structure
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t version;
    char callsign[CALLSIGN_LEN];
    char locator[LOCATOR_LEN];
    uint8_t power_dbm;
    char wifi_ssid[33];
    char wifi_pass[65];
    bool band_enabled[BAND_COUNT];
    bool hop_enabled;
    uint32_t hop_interval_sec;
    char ntp_server[64];
    bool tx_enabled;
    uint8_t tx_duty_pct;
    int32_t xtal_cal_ppb;
    uint8_t iaru_region;
} wspr_config_t;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t config_init(void);
esp_err_t config_load(wspr_config_t *cfg);
esp_err_t config_save(const wspr_config_t *cfg);
void config_defaults(wspr_config_t *cfg);
// MODIFIED 3.16: returns true if config was reset to defaults this boot
// (schema version mismatch or blob size mismatch); false on normal load.
// Call after config_load(). Safe from any task after startup.
bool config_was_reset(void);
