/**
 * @file config.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @brief Persistent configuration subsystem for the ESP32 WSPR transmitter.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "config.h"

static const char *TAG = "config";

// NVS namespace used to group all WSPR configuration keys
#define NVS_NS "wspr"

// Set to true when config_load() falls back to defaults due to schema mismatch
static bool _was_reset = false;

// WRC-15 new 5 MHz allocation (5 351.5 - 5 366.5 kHz): in countries that have
// adopted WRC-15 the preferred WSPR dial has shifted to 5 364 700 Hz
// (RF center 5 366 200 Hz). This is not yet globally adopted and is not
// represented as a region preset here; users should configure it manually
// until a fourth region entry is warranted.

// WSPR RF center frequencies (= SSB dial + 1500 Hz) by region and band.
// All bands except 60 m are identical across regions.
//
// 60 m WSPR dial frequencies per IARU/ITU region:
//   Region 1 (Europe/Africa/ME):  5 287 200 Hz dial -> RF center 5 288 700 Hz
//   Region 2 (Americas):          5 346 500 Hz dial -> RF center 5 348 000 Hz
//   Region 3 (Asia/Pacific):      5 367 000 Hz dial -> RF center 5 368 500 Hz
//
const uint32_t BAND_FREQ_HZ[3][BAND_COUNT] = {
    // -- Region 1: Europe, Africa, Middle East --
    {
        137600UL,   // 2200m
        475700UL,   // 630m
        1838100UL,  // 160m
        3570100UL,  // 80m
        5288700UL,  // 60m  (dial 5 287 200 + 1500 Hz RF center) -- Region 1 correct
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
        137600UL,  // 2200m
        475700UL,  // 630m
        1838100UL, // 160m
        3570100UL, // 80m
        // Region 2 60 m WSPR dial = 5 346 500 Hz; RF center = dial + 1500 = 5 348 000 Hz.
        5348000UL,  // 60m  (dial 5 346 500 + 1500 Hz RF center) -- Region 2 corrected
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
        137600UL,  // 2200m
        475700UL,  // 630m
        1838100UL, // 160m
        3570100UL, // 80m
        // Region 3 60 m WSPR dial = 5 367 000 Hz; RF center = dial + 1500 = 5 368 500 Hz.
        5368500UL,  // 60m  (dial 5 367 000 + 1500 Hz RF center) -- Region 3 corrected
        7040100UL,  // 40m
        10140200UL, // 30m
        14097100UL, // 20m
        18106100UL, // 17m
        21096100UL, // 15m
        24926100UL, // 12m
        28126100UL, // 10m
    },
};

// Human-readable band name strings, indexed by wspr_band_t
const char *BAND_NAME[BAND_COUNT] = {
    "2200m", "630m", "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m",
};

// Maps each WSPR band to a 3-bit hardware filter bank address (0-7).
// The three address bits drive GPIO_A/B/C to select one of 8 LPF relay sections.
// Groups of nearby bands share the same filter when their frequencies are close
// enough to be served by a single LC low-pass filter design.
// Adjust this table if the physical relay board wiring differs from the default.
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

// Compile-time checks: default callsign and locator lengths must fit their fields
static_assert(sizeof(CONFIG_WSPR_DEFAULT_CALLSIGN) <= CALLSIGN_LEN, "Default callsign too long for CALLSIGN_LEN");
static_assert(sizeof(CONFIG_WSPR_DEFAULT_LOCATOR) == 5 || sizeof(CONFIG_WSPR_DEFAULT_LOCATOR) == 7,
              "Default locator must be 4 characters (DDLL) or 6 characters (DDLLSS)");

void config_defaults(wspr_config_t *cfg) {
    // Zero the entire structure first to avoid stale padding bytes in the NVS blob
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = CONFIG_SCHEMA_VERSION;
    strncpy(cfg->callsign, CONFIG_WSPR_DEFAULT_CALLSIGN, CALLSIGN_LEN - 1);
    strncpy(cfg->locator, CONFIG_WSPR_DEFAULT_LOCATOR, LOCATOR_LEN - 1);
    cfg->power_dbm = CONFIG_WSPR_DEFAULT_POWER_DBM;
    strncpy(cfg->ntp_server, "pool.ntp.org", sizeof(cfg->ntp_server) - 1);
    cfg->hop_enabled = false;
    cfg->hop_interval_sec = 120; // minimum is one full WSPR TX slot (110.6 s rounded up)
    cfg->tx_enabled = false;
    // Enable 40 m and 20 m by default — the two most popular WSPR bands worldwide
    cfg->band_enabled[BAND_40M] = true;
    cfg->band_enabled[BAND_20M] = true;
    // 20 % duty cycle: transmit 1 in 5 slots, the WSPR recommended default
    cfg->tx_duty_pct = 20;
    cfg->xtal_cal_ppb = 0;
    cfg->iaru_region = (uint8_t)IARU_REGION_1;
    // Runtime-only flags: not stored in NVS, always reset to clean state
    cfg->bands_changed = false;
    cfg->tx_slot_parity = 0;
    cfg->tone_active = false;
    cfg->tone_freq_khz = 0.0f;
}

esp_err_t config_init(void) {
    // Initialize NVS flash; erase and re-init if the partition is full or version-changed
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t config_load(wspr_config_t *cfg) {
    // Always start with a fully valid default structure
    config_defaults(cfg);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot or erased flash: use defaults silently
        ESP_LOGI(TAG, "No saved config, using defaults");
        return ESP_OK;
    }
    if (err != ESP_OK)
        return err;

    size_t sz = sizeof(*cfg);
    err = nvs_get_blob(h, "cfg", cfg, &sz);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Namespace exists but the blob key is absent: use defaults
        config_defaults(cfg);
        return ESP_OK;
    }

    if (err != ESP_OK) {
        config_defaults(cfg);
        return err;
    }

    // Schema size check: a size mismatch means the struct layout changed
    if (sz != sizeof(*cfg)) {
        ESP_LOGW(TAG, "Config blob size mismatch (stored=%u expected=%u), using defaults", (unsigned)sz, (unsigned)sizeof(*cfg));
        config_defaults(cfg);
        _was_reset = true;
        return ESP_OK;
    }

    // Schema version check: reject blobs from older firmware builds
    if (cfg->version != CONFIG_SCHEMA_VERSION) {
        ESP_LOGW(TAG, "Config schema mismatch (stored=%u expected=%u), using defaults", cfg->version, (unsigned)CONFIG_SCHEMA_VERSION);
        config_defaults(cfg);
        _was_reset = true;
        return ESP_OK;
    }

    // Force NUL-termination on all string fields regardless of stored content
    cfg->callsign[CALLSIGN_LEN - 1] = '\0';
    cfg->locator[LOCATOR_LEN - 1] = '\0';
    cfg->wifi_ssid[sizeof(cfg->wifi_ssid) - 1] = '\0';
    cfg->wifi_pass[sizeof(cfg->wifi_pass) - 1] = '\0';
    cfg->ntp_server[sizeof(cfg->ntp_server) - 1] = '\0';

    // Clamp IARU region: values outside 1-3 are invalid (memset-zero would give 0)
    if (cfg->iaru_region < 1 || cfg->iaru_region > 3)
        cfg->iaru_region = (uint8_t)IARU_REGION_1;

    // Enforce minimum hop interval: one full WSPR TX slot = 110.6 s, rounded to 120 s
    if (cfg->hop_interval_sec < 120u)
        cfg->hop_interval_sec = 120u;

    // Snap hop_interval_sec to the nearest multiple of 120 s
    // at load time so the hopping timer always aligns with even-minute UTC boundaries.
    // A hop interval of e.g. 300 s (not a multiple of 120) would cause hops at
    // arbitrary offsets within the 2-minute cycle, wasting partial TX slots.
    cfg->hop_interval_sec = ((cfg->hop_interval_sec + 60u) / 120u) * 120u;
    if (cfg->hop_interval_sec < 120u)
        cfg->hop_interval_sec = 120u;

    // Unconditionally reset ALL runtime-only fields
    // AFTER loading the blob. config_save() stores the entire struct including
    // runtime fields (tone_active, tone_freq_khz, tx_slot_parity, bands_changed).
    // If config_save() was called while tone_active=true, the next boot would
    // restore tone_active=true and potentially output a carrier immediately.
    // tone_freq_khz was also not previously reset here, which could cause the
    // scheduler to output a stale tone frequency from the previous session.
    cfg->bands_changed = false;
    cfg->tx_slot_parity = 0;
    cfg->tone_active = false;
    cfg->tone_freq_khz = 0.0f; // prevents stale tone frequency

    ESP_LOGI(TAG,
             "Config loaded:\n  Call Sign=%s\n  Locator=%s\n  TX Power=%d dBm\n  XTAL Calibration=%ld ppb\n  IARU Region=%d\n  Frequency hopping(%s):%usec",
             cfg->callsign, cfg->locator, cfg->power_dbm, (long)cfg->xtal_cal_ppb, (int)cfg->iaru_region, cfg->hop_enabled ? "enabled" : "disabled",
             cfg->hop_interval_sec);
    return err;
}

esp_err_t config_save(const wspr_config_t *cfg) {
    // Make a sanitized copy with all runtime-only fields zeroed before
    // persisting. Without this, a call to config_save() while tone_active=true (e.g.
    // the user saves config during a tone test) would write tone_active=true and a
    // non-zero tone_freq_khz to NVS. Although config_load() already resets these on
    // boot, the blob on flash would contain garbage runtime state that confuses future
    // schema migrations and produces misleading NVS dumps.
    // tx_slot_parity is also zeroed so the Type-2/Type-3 alternation always starts
    // cleanly from the Type-2 frame after a reboot.
    wspr_config_t save_cfg = *cfg;
    save_cfg.bands_changed = false; // runtime: band-list rebuild flag
    save_cfg.tx_slot_parity = 0;    // runtime: Type-2/Type-3 alternation state
    save_cfg.tone_active = false;   // runtime: tone test in progress
    save_cfg.tone_freq_khz = 0.0f;  // runtime: tone test frequency

    // Write the sanitized copy as a single NVS blob for atomic persistence
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(h, "cfg", &save_cfg, sizeof(save_cfg));
    if (err == ESP_OK)
        err = nvs_commit(h); // flush to flash before closing
    nvs_close(h);

    ESP_LOGI(TAG, "Config saved: cs=%s loc=%s pwr=%d dBm region=%d", save_cfg.callsign, save_cfg.locator, save_cfg.power_dbm, (int)save_cfg.iaru_region);
    return err;
}

bool config_was_reset(void) {
    return _was_reset;
}
