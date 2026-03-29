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

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "config.h"

static const char *TAG = "config";
#define NVS_NS "wspr"

// set to true in config_load() when stored config is discarded
// due to blob size mismatch or schema version mismatch; queried by web_server
// to display a one-time "settings reset to defaults" warning in the UI.
static bool _was_reset = false;

// 60 m dial frequencies by region (source: IARU band plans + WSPRnet):
//   Region 1 (EU/Africa)  : 5 288 600 Hz  (IARU R1 60 m WSPR allocation)
//   Region 2 (Americas)   : 5 346 500 Hz  (FCC channels centre, ARRL coord.)
//   Region 3 (Asia/Pacific): 5 367 000 Hz  (WIA/JARL coordination)
//
// Row 0 = IARU Region 1, Row 1 = Region 2, Row 2 = Region 3.
const uint32_t BAND_FREQ_HZ[3][BAND_COUNT] = {
    // -- Region 1: Europe, Africa, Middle East --
    {
        137600UL,   // 2200m
        475700UL,   // 630m
        1838100UL,  // 160m
        3570100UL,  // 80m
        5288600UL,  // 60m  <-- Region 1 specific
        7040100UL,  // 40m
        10140200UL, // 30m
        14097100UL, // 20m
        18106100UL, // 17m
        21096100UL, // 15m
        24926100UL, // 12m
        28126100UL, // 10m
    },
    // -- Region 2: Americas (North, Central, South, Caribbean) --
    {
        137600UL,   // 2200m
        475700UL,   // 630m
        1838100UL,  // 160m
        3570100UL,  // 80m
        5346500UL,  // 60m  <-- Region 2 specific (FCC / ARRL coordination)
        7040100UL,  // 40m
        10140200UL, // 30m
        14097100UL, // 20m
        18106100UL, // 17m
        21096100UL, // 15m
        24926100UL, // 12m
        28126100UL, // 10m
    },
    // -- Region 3: Asia, Pacific, Oceania --
    {
        137600UL,   // 2200m
        475700UL,   // 630m
        1838100UL,  // 160m
        3570100UL,  // 80m
        5367000UL,  // 60m  <-- Region 3 specific (WIA/JARL coordination)
        7040100UL,  // 40m
        10140200UL, // 30m
        14097100UL, // 20m
        18106100UL, // 17m
        21096100UL, // 15m
        24926100UL, // 12m
        28126100UL, // 10m
    },
};

const char *BAND_NAME[BAND_COUNT] = {
    "2200m", "630m", "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m",
};

const uint8_t BAND_FILTER[BAND_COUNT] = {
    0, // 2200m -> filter 0
    0, // 630m  -> filter 0
    1, // 160m  -> filter 1
    2, // 80m   -> filter 2
    2, // 60m   -> filter 2
    3, // 40m   -> filter 3
    4, // 30m   -> filter 4
    5, // 20m   -> filter 5
    5, // 17m   -> filter 5
    6, // 15m   -> filter 6
    6, // 12m   -> filter 6
    7, // 10m   -> filter 7
};

static_assert(sizeof(CONFIG_WSPR_DEFAULT_CALLSIGN) <= CALLSIGN_LEN, "Default callsign too long for CALLSIGN_LEN");
static_assert(sizeof(CONFIG_WSPR_DEFAULT_LOCATOR) == 5, "Default locator must be exactly 4 characters (DDLL format)");

void config_defaults(wspr_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = CONFIG_SCHEMA_VERSION;
    strncpy(cfg->callsign, CONFIG_WSPR_DEFAULT_CALLSIGN, CALLSIGN_LEN - 1);
    strncpy(cfg->locator, CONFIG_WSPR_DEFAULT_LOCATOR, LOCATOR_LEN - 1);
    cfg->power_dbm = CONFIG_WSPR_DEFAULT_POWER_DBM;
    strncpy(cfg->ntp_server, "pool.ntp.org", sizeof(cfg->ntp_server) - 1);
    cfg->hop_enabled = false;
    cfg->hop_interval_sec = 120;
    cfg->tx_enabled = false;
    cfg->band_enabled[BAND_40M] = true;
    cfg->band_enabled[BAND_20M] = true;
    cfg->tx_duty_pct = 20;
    cfg->xtal_cal_ppb = 0;
    cfg->iaru_region = (uint8_t)IARU_REGION_1;
}

esp_err_t config_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t config_load(wspr_config_t *cfg) {
    config_defaults(cfg);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved config, using defaults");
        return ESP_OK;
    }
    if (err != ESP_OK)
        return err;

    size_t sz = sizeof(*cfg);
    err = nvs_get_blob(h, "cfg", cfg, &sz);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config_defaults(cfg);
        return ESP_OK;
    }

    if (err != ESP_OK) {
        config_defaults(cfg);
        return err;
    }

    if (sz != sizeof(*cfg)) {
        ESP_LOGW(TAG, "Config blob size mismatch (stored=%u expected=%u), using defaults", (unsigned)sz, (unsigned)sizeof(*cfg));
        config_defaults(cfg);
        _was_reset = true; // flag reset so UI can warn user
        return ESP_OK;
    }

    if (cfg->version != CONFIG_SCHEMA_VERSION) {
        ESP_LOGW(TAG, "Config schema mismatch (stored=%u expected=%u), using defaults", cfg->version, (unsigned)CONFIG_SCHEMA_VERSION);
        config_defaults(cfg);
        _was_reset = true; // flag reset so UI can warn user
        return ESP_OK;
    }

    cfg->callsign[CALLSIGN_LEN - 1] = '\0';
    cfg->locator[LOCATOR_LEN - 1] = '\0';
    cfg->wifi_ssid[sizeof(cfg->wifi_ssid) - 1] = '\0';
    cfg->wifi_pass[sizeof(cfg->wifi_pass) - 1] = '\0';
    cfg->ntp_server[sizeof(cfg->ntp_server) - 1] = '\0';

    // clamp iaru_region to valid range 1-3; fix any corrupt NVS value.
    if (cfg->iaru_region < 1 || cfg->iaru_region > 3)
        cfg->iaru_region = (uint8_t)IARU_REGION_1;

    // clamp hop_interval_sec to the minimum of one WSPR TX slot.
    // A corrupt or legacy NVS value below 120 s would rotate bands on every
    // scheduler iteration instead of once per slot, breaking normal operation.
    if (cfg->hop_interval_sec < 120u)
        cfg->hop_interval_sec = 120u;

    ESP_LOGI(TAG, "Config loaded: cs=%s loc=%s pwr=%d dBm cal=%ld ppb region=%d", cfg->callsign, cfg->locator, cfg->power_dbm, (long)cfg->xtal_cal_ppb,
             (int)cfg->iaru_region);
    return err;
}

esp_err_t config_save(const wspr_config_t *cfg) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(h, "cfg", cfg, sizeof(*cfg));
    if (err == ESP_OK)
        err = nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "Config saved: cs=%s loc=%s pwr=%d dBm region=%d", cfg->callsign, cfg->locator, cfg->power_dbm, (int)cfg->iaru_region);
    return err;
}

bool config_was_reset(void) {
    return _was_reset;
}
