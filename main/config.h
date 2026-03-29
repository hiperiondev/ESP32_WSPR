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

/**
 * @file config.h
 * @brief Persistent configuration subsystem for the WSPR transmitter.
 *
 * This module owns the single source of truth for all user-configurable
 * parameters. It provides:
 *  - A versioned flat struct (@ref wspr_config_t) that is stored as a binary
 *    blob in the ESP32 Non-Volatile Storage (NVS).
 *  - Band frequency and filter tables for all supported WSPR allocations.
 *  - Load / save / defaults helpers that wrap the NVS flash API.
 *
 * @par Schema versioning
 * @ref CONFIG_SCHEMA_VERSION must be incremented whenever the layout of
 * @ref wspr_config_t changes (fields added, removed, or reordered).  The
 * loader detects a mismatch and silently reverts to built-in defaults,
 * preventing corrupted or misaligned fields after a firmware update.
 *
 * @par Thread safety
 * The functions in this module are **not** thread-safe on their own.  Callers
 * that access the live @ref wspr_config_t from multiple tasks must protect it
 * with an external mutex (see @c web_server_cfg_lock / @c web_server_cfg_unlock
 * in @c web_server.h).
 */

// ---------------------------------------------------------------------------
// Schema version
// ---------------------------------------------------------------------------

/**
 * @defgroup cfg_schema Schema version
 * @{
 */

/**
 * @brief NVS blob schema version.
 *
 * Increment this constant whenever the binary layout of @ref wspr_config_t
 * changes.  The loader compares this value against the @c version field stored
 * in NVS; a mismatch causes the blob to be discarded and defaults to be used.
 *
 * History:
 *  - 1: initial layout
 *  - 2: added @c tx_duty_pct
 *  - 3: added @c xtal_cal_ppb
 */
#define CONFIG_SCHEMA_VERSION 3u

/** @} */

// ---------------------------------------------------------------------------
// String field size constants
// ---------------------------------------------------------------------------

/**
 * @defgroup cfg_sizes String field sizes
 * @{
 */

/**
 * @brief Maximum callsign storage length **including** the NUL terminator.
 *
 * A WSPR callsign is at most 6 printable characters; the extra bytes allow
 * for a leading space that the encoder prepends when the second character is
 * a letter (e.g. "G4JNT" becomes " G4JNT").
 */
#define CALLSIGN_LEN 12

/**
 * @brief Maximum Maidenhead locator storage length **including** the NUL
 *        terminator.
 *
 * WSPR Type-1 messages encode only a 4-character grid square (e.g. "GF05").
 * The extra bytes allow future expansion to 6-character sub-squares, but the
 * encoder will reject any locator longer than 4 significant characters.
 */
#define LOCATOR_LEN 7

/** @} */

// ---------------------------------------------------------------------------
// Band enumeration
// ---------------------------------------------------------------------------

/**
 * @defgroup cfg_bands Band definitions
 * @{
 */

/**
 * @brief Enumeration of all WSPR-capable amateur bands.
 *
 * The numeric value of each enumerator is used as a direct index into
 * @ref BAND_FREQ_HZ, @ref BAND_NAME, and @ref BAND_FILTER.  The sentinel
 * @ref BAND_COUNT gives the total number of entries and must equal 12.
 */
typedef enum {
    BAND_2200M = 0, /**< 2200 m — 0.137600 MHz  (137 600 Hz) */
    BAND_630M,      /**< 630 m  — 0.475700 MHz  (475 700 Hz) */
    BAND_160M,      /**< 160 m  — 1.838100 MHz               */
    BAND_80M,       /**< 80 m   — 3.570100 MHz               */
    BAND_60M,       /**< 60 m   — 5.288600 MHz               */
    BAND_40M,       /**< 40 m   — 7.040100 MHz               */
    BAND_30M,       /**< 30 m   — 10.140200 MHz              */
    BAND_20M,       /**< 20 m   — 14.097100 MHz              */
    BAND_17M,       /**< 17 m   — 18.106100 MHz              */
    BAND_15M,       /**< 15 m   — 21.096100 MHz              */
    BAND_12M,       /**< 12 m   — 24.926100 MHz              */
    BAND_10M,       /**< 10 m   — 28.126100 MHz              */
    BAND_COUNT      /**< Sentinel — total number of bands (must equal 12). */
} wspr_band_t;

/**
 * @brief Compile-time guard: catches future enum additions that forget to
 *        update the band frequency / name / filter tables.
 */
static_assert(BAND_COUNT == 12, "BAND_COUNT changed; update all band tables");

/**
 * @brief WSPR dial frequencies in Hz, indexed by @ref wspr_band_t.
 *
 * Each entry is the standard WSPR dial frequency for the corresponding band.
 * The actual transmitted carrier is @c BAND_FREQ_HZ[band] + 1500 Hz (the
 * WSPR audio passband centre), with individual symbol offsets of up to
 * ~4.4 Hz on top of that.
 */
extern const uint32_t BAND_FREQ_HZ[BAND_COUNT];

/**
 * @brief Human-readable band name strings, indexed by @ref wspr_band_t.
 *
 * Examples: @c "40m", @c "20m".  The strings are in read-only flash; do not
 * attempt to modify them.
 */
extern const char *BAND_NAME[BAND_COUNT];

/**
 * @brief 3-bit low-pass filter bank address for each band, indexed by
 *        @ref wspr_band_t.
 *
 * Values are in the range 0–7 and correspond to the binary address driven on
 * GPIO_A / GPIO_B / GPIO_C by @c gpio_filter_select().  Adjust the table in
 * @c config.c to match the relay / switch topology of the hardware LPF board.
 */
extern const uint8_t BAND_FILTER[BAND_COUNT];

/** @} */

// ---------------------------------------------------------------------------
// Persistent configuration structure
// ---------------------------------------------------------------------------

/**
 * @defgroup cfg_struct Persistent configuration
 * @{
 */

/**
 * @brief All user-configurable parameters stored in NVS.
 *
 * The struct is written to and read from NVS as a flat binary blob under the
 * namespace @c "wspr" with the key @c "cfg".  The @c version field at offset 0
 * is checked on every load; if it does not equal @ref CONFIG_SCHEMA_VERSION the
 * blob is considered stale and defaults are applied instead.
 *
 * @warning Fields must **never** be reordered or removed without incrementing
 *          @ref CONFIG_SCHEMA_VERSION.  New fields should be appended at the
 *          end of the struct to minimise invalidation of existing NVS blobs.
 */
typedef struct {
    /**
     * @brief Schema version tag.
     *
     * Written by @ref config_save() as @ref CONFIG_SCHEMA_VERSION.  Checked by
     * @ref config_load(); a mismatch resets the config to defaults.
     */
    uint8_t version;

    /**
     * @brief Amateur radio callsign, NUL-terminated.
     *
     * Must be a valid WSPR callsign (3–6 alphanumeric characters, with the
     * third character a digit after optional leading space normalisation).
     * Maximum useful length is 6 characters; the buffer is @ref CALLSIGN_LEN
     * bytes including the terminator.
     */
    char callsign[CALLSIGN_LEN];

    /**
     * @brief 4-character Maidenhead grid square locator, NUL-terminated.
     *
     * Accepted format: two letters (A–R), two digits (0–9), e.g. @c "GF05".
     * Six-character sub-squares are stored in the oversized buffer but will be
     * rejected by the WSPR encoder at encode time.
     * Buffer is @ref LOCATOR_LEN bytes including the terminator.
     */
    char locator[LOCATOR_LEN];

    /**
     * @brief Transmit power in dBm.
     *
     * Must be one of the valid WSPR power levels:
     * 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60.
     * Values not in the list are silently rounded to the nearest valid level
     * by the encoder.
     */
    uint8_t power_dbm;

    /**
     * @brief Wi-Fi station SSID, NUL-terminated.
     *
     * Maximum 32 characters (IEEE 802.11 limit) plus terminator.
     * Leave empty to skip STA mode and start the fallback soft-AP immediately.
     */
    char wifi_ssid[33];

    /**
     * @brief Wi-Fi station password, NUL-terminated.
     *
     * Maximum 64 characters (WPA2 maximum) plus terminator.
     * Leave empty for open networks.
     */
    char wifi_pass[65];

    /**
     * @brief Per-band transmit enable flags.
     *
     * @c band_enabled[i] is @c true when band @c i (a @ref wspr_band_t value)
     * is included in the active rotation.  At least one band must be enabled;
     * the scheduler falls back to 40 m if the array is entirely @c false.
     */
    bool band_enabled[BAND_COUNT];

    /**
     * @brief Enable automatic frequency hopping across active bands.
     *
     * When @c true the scheduler advances to the next active band after every
     * @ref hop_interval_sec seconds.  When @c false the device stays on the
     * first enabled band indefinitely.
     */
    bool hop_enabled;

    /**
     * @brief Seconds between automatic band hops.
     *
     * Only effective when @ref hop_enabled is @c true.  Values below 120 s
     * are clamped to 120 s at runtime (one WSPR slot minimum) to avoid
     * changing the band mid-transmission.  Maximum allowed by the web UI is
     * 86 400 s (24 h).
     */
    uint32_t hop_interval_sec;

    /**
     * @brief NTP server hostname or IP address string, NUL-terminated.
     *
     * Used when the firmware is compiled with @c CONFIG_WSPR_TIME_NTP.
     * Ignored in GPS time mode.  Defaults to @c "pool.ntp.org".
     */
    char ntp_server[64];

    /**
     * @brief Master transmit enable flag.
     *
     * When @c false the scheduler loop runs but never fires a transmission.
     * Toggled by the web UI TX button and persisted to NVS so the setting
     * survives a reboot.
     */
    bool tx_enabled;

    /**
     * @brief TX duty cycle as a percentage of available 2-minute WSPR slots.
     *
     * Range 0–100.
     *  - @c 0   : never transmit (TX off).
     *  - @c 20  : transmit 1 in every 5 slots (WSPR convention default).
     *  - @c 100 : transmit every available slot.
     *
     * The scheduler uses a Bresenham-style accumulator so the long-term
     * average converges to exactly the configured percentage for any integer
     * value without division rounding errors.
     */
    uint8_t tx_duty_pct;

    /**
     * @brief Crystal frequency calibration offset in parts-per-billion (ppb).
     *
     * Applied to the oscillator driver to correct for crystal manufacturing
     * tolerance.
     *  - @c 0       : no correction (default).
     *  - Positive   : crystal runs faster than nominal; VCO is lowered to
     *                 compensate.
     *  - Negative   : crystal runs slower; VCO is raised.
     *
     * Typical TCXO / XTAL tolerance is ±20 ppm (±20 000 ppb); the allowed
     * range of ±100 000 ppb covers worst-case ageing and temperature drift of
     * any common oscillator module.
     */
    int32_t xtal_cal_ppb;
} wspr_config_t;

/** @} */

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @defgroup cfg_api Configuration API
 * @{
 */

/**
 * @brief Initialise the NVS flash subsystem.
 *
 * Must be called once at startup before any other @c config_* function.
 * Erases and re-initialises the NVS partition if it is full or a version
 * mismatch is detected by the NVS library itself.
 *
 * @return @c ESP_OK on success, or an NVS error code on failure.
 */
esp_err_t config_init(void);

/**
 * @brief Load the configuration from NVS into @p cfg.
 *
 * The function always starts by applying built-in defaults via
 * @ref config_defaults() so that every field has a well-defined value even if
 * the NVS blob is absent, truncated, or from a different schema version.
 *
 * Loading sequence:
 *  1. Apply defaults.
 *  2. Open the NVS namespace.
 *  3. Read the blob.
 *  4. Verify @c cfg->version == @ref CONFIG_SCHEMA_VERSION; revert to defaults
 *     on mismatch.
 *  5. Force-NUL-terminate all string fields.
 *
 * @param[out] cfg  Pointer to the configuration struct to populate.  Must not
 *                  be @c NULL.
 *
 * @return @c ESP_OK on success (including the "no saved config" case where
 *         defaults are used), or an NVS error code on a storage-level failure.
 */
esp_err_t config_load(wspr_config_t *cfg);

/**
 * @brief Persist @p cfg to NVS.
 *
 * Writes the entire @ref wspr_config_t struct as a binary blob and commits
 * the NVS transaction.  The @c version field is **not** automatically updated
 * by this function; the caller is responsible for setting it to
 * @ref CONFIG_SCHEMA_VERSION before saving (done by @ref config_defaults()).
 *
 * @param[in] cfg  Pointer to the configuration to save.  Must not be @c NULL.
 *                 The struct is read under the caller's lock; pass a snapshot
 *                 copy if the live config may be modified concurrently.
 *
 * @return @c ESP_OK on success, or an NVS error code on failure.
 */
esp_err_t config_save(const wspr_config_t *cfg);

/**
 * @brief Fill @p cfg with compiled-in default values.
 *
 * Resets every field to a safe, usable state derived from Kconfig symbols and
 * compile-time constants.  Sets @c cfg->version to @ref CONFIG_SCHEMA_VERSION.
 * Called internally by @ref config_load() before attempting to read NVS.
 *
 * Default band selection: 40 m and 20 m enabled, all others disabled.
 * Default TX duty cycle: 20 % (1 in 5 slots, matching the WSPR convention).
 *
 * @param[out] cfg  Pointer to the configuration struct to reset.  Must not be
 *                  @c NULL.
 */
void config_defaults(wspr_config_t *cfg);

/** @} */
