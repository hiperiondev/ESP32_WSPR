/**
 * @file config.h
 * @brief Persistent configuration subsystem for the ESP32 WSPR transmitter.
 * @copyright 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module owns the single authoritative @ref wspr_config_t structure that
 * holds every run-time-tunable parameter of the WSPR transmitter.  Persistent
 * storage is provided by the ESP-IDF Non-Volatile Storage (NVS) library, which
 * writes the structure as a single blob under the namespace @c "wspr" with key
 * @c "cfg".
 *
 * @par Schema versioning
 * The @ref CONFIG_SCHEMA_VERSION constant is embedded in the blob.  When the
 * firmware reads a blob whose version does not match @ref CONFIG_SCHEMA_VERSION,
 * or whose byte size differs from @c sizeof(wspr_config_t), the stored data is
 * silently discarded and compile-time defaults are used instead.  The
 * @ref config_was_reset() predicate allows callers to detect and report this
 * event through the web UI.
 *
 * @par Compile-time defaults
 * Default values for callsign, locator, and TX power are supplied by the
 * Kconfig symbols @c CONFIG_WSPR_DEFAULT_CALLSIGN, @c CONFIG_WSPR_DEFAULT_LOCATOR,
 * and @c CONFIG_WSPR_DEFAULT_POWER_DBM respectively.  These are set in
 * @c menuconfig under the @em "WSPR Transmitter" menu.
 *
 * @par IARU region and 60 m frequency
 * WSPR uses a different 60 m dial frequency in each ITU/IARU region.  The
 * three-dimensional table @ref BAND_FREQ_HZ [region-1][band] stores the
 * correct frequency for every combination.  The helper function
 * @ref config_band_freq_hz() maps a @ref iaru_region_t and a @ref wspr_band_t
 * to the corresponding entry with bounds-checked array access.
 *
 * @par Band filter mapping
 * The @ref BAND_FILTER array maps each @ref wspr_band_t to a 3-bit hardware
 * filter bank address (0–7) that is written to the GPIO pins defined in
 * menuconfig.  Adjust this table when the physical LPF board layout differs
 * from the default assignment.
 *
 * @par Thread safety
 * @ref config_load() and @ref config_save() access the NVS flash driver which
 * is internally mutex-protected by ESP-IDF.  They must, however, not be called
 * concurrently on the same @ref wspr_config_t pointer because neither function
 * takes its own mutex.  In @c main.c all access to the live @c g_cfg instance
 * is serialised through the web-server config mutex (@ref web_server_cfg_lock()
 * / @ref web_server_cfg_unlock()).
 */

#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @defgroup config_schema Configuration schema
 * @{
 */

/**
 * @brief NVS blob schema version.
 *
 * This constant is stored as the first byte of the NVS blob.  @ref config_load()
 * rejects any blob whose stored version differs from this value and falls back
 * to compile-time defaults, setting the @ref config_was_reset() flag to @c true.
 *
 * Increment this constant whenever the layout or semantics of @ref wspr_config_t
 * change in a way that would make an old blob incompatible with the new firmware.
 * The current value is @c 6.
 */
#define CONFIG_SCHEMA_VERSION 6u

/** @} */

/**
 * @defgroup config_string_sizes String field size constants
 * @{
 */

/**
 * @brief Maximum number of bytes (including NUL terminator) for a callsign string.
 *
 * WSPR Type-1 callsigns are at most 6 characters.  Compound callsigns
 * (e.g. @c "PJ4/K1ABC") can be up to 11 characters.  This constant reserves
 * 12 bytes to accommodate the longest possible compound callsign plus the
 * mandatory NUL terminator.
 *
 * A @c static_assert in @c config.c verifies that the default callsign defined
 * in Kconfig fits within this limit.
 */
#define CALLSIGN_LEN 12

/**
 * @brief Maximum number of bytes (including NUL terminator) for a Maidenhead locator.
 *
 * WSPR supports two locator precisions:
 *  - 4-character grid square (e.g. @c "GF05"): encodes to Type-1 message.
 *  - 6-character sub-square (e.g. @c "GF05ab"): triggers alternating
 *    Type-1 + Type-3 transmission for sub-square precision.
 *
 * This constant reserves 7 bytes: 6 characters plus the NUL terminator.
 * A @c static_assert in @c config.c verifies that the default locator
 * in Kconfig is either 4 or 6 characters (5 or 7 bytes including NUL).
 */
#define LOCATOR_LEN 7

/** @} */

/**
 * @defgroup config_iaru IARU region enumeration
 * @{
 */

/**
 * @brief ITU/IARU administrative region identifier.
 *
 * The International Amateur Radio Union divides the world into three regions
 * for the purpose of band-plan coordination.  All WSPR bands use identical
 * dial frequencies worldwide **except 60 m**, where each region operates on a
 * different channel to avoid interference with local secondary services.
 *
 * | Value            | Region                               | 60 m dial frequency |
 * |------------------|--------------------------------------|---------------------|
 * | @c IARU_REGION_1 | Europe, Africa, Middle East          | 5 288 600 Hz        |
 * | @c IARU_REGION_2 | Americas (North/South/Caribbean)     | 5 346 500 Hz        |
 * | @c IARU_REGION_3 | Asia, Pacific, Oceania               | 5 367 000 Hz        |
 *
 * The selected region is persisted in @ref wspr_config_t::iaru_region and
 * used by @ref config_band_freq_hz() to index the correct row of
 * @ref BAND_FREQ_HZ.
 *
 * @note The numeric values intentionally start at 1 (not 0) so that a zeroed
 * structure produced by @c memset() is not silently accepted as a valid region.
 * @ref config_load() resets any stored value outside the range 1–3 to
 * @c IARU_REGION_1.
 */
typedef enum {
    IARU_REGION_1 = 1, /**< Europe, Africa, Middle East — 60 m: 5 288 600 Hz */
    IARU_REGION_2 = 2, /**< Americas (North/South/Caribbean) — 60 m: 5 346 500 Hz */
    IARU_REGION_3 = 3, /**< Asia, Pacific, Oceania — 60 m: 5 367 000 Hz */
} iaru_region_t;

/** @} */

/**
 * @defgroup config_bands Band enumeration and frequency tables
 * @{
 */

/**
 * @brief Index of each supported WSPR amateur band.
 *
 * The enumeration is used as an array index into @ref BAND_FREQ_HZ,
 * @ref BAND_NAME, and @ref BAND_FILTER.  @c BAND_COUNT is the total number of
 * supported bands and is verified by a @c static_assert in @c config.h to equal 12.
 *
 * The bands are ordered from lowest to highest frequency (longest to shortest
 * wavelength), which also corresponds to the assignment of relay addresses in
 * the default @ref BAND_FILTER table.
 */
typedef enum {
    BAND_2200M = 0, /**< 2200 m MF band — dial frequency 137 600 Hz */
    BAND_630M,      /**< 630 m LF band — dial frequency 475 700 Hz */
    BAND_160M,      /**< 160 m HF band — dial frequency 1 838 100 Hz */
    BAND_80M,       /**< 80 m HF band — dial frequency 3 570 100 Hz */
    BAND_60M,       /**< 60 m HF band — dial frequency region-dependent (see @ref iaru_region_t) */
    BAND_40M,       /**< 40 m HF band — dial frequency 7 040 100 Hz */
    BAND_30M,       /**< 30 m HF band — dial frequency 10 140 200 Hz */
    BAND_20M,       /**< 20 m HF band — dial frequency 14 097 100 Hz */
    BAND_17M,       /**< 17 m HF band — dial frequency 18 106 100 Hz */
    BAND_15M,       /**< 15 m HF band — dial frequency 21 096 100 Hz */
    BAND_12M,       /**< 12 m HF band — dial frequency 24 926 100 Hz */
    BAND_10M,       /**< 10 m HF band — dial frequency 28 126 100 Hz */
    BAND_COUNT      /**< Sentinel — total number of bands; do not use as a band index */
} wspr_band_t;

static_assert(BAND_COUNT == 12, "BAND_COUNT changed; update all band tables");

/**
 * @brief WSPR dial frequencies in Hz, indexed by [region-1][band].
 *
 * The first dimension is the IARU region index (0 = Region 1, 1 = Region 2,
 * 2 = Region 3).  The second dimension is the @ref wspr_band_t value.
 *
 * All bands except 60 m have identical entries in all three rows.
 * The 60 m entry (index @c BAND_60M) differs per region:
 *  - Row 0 (Region 1): 5 288 600 Hz
 *  - Row 1 (Region 2): 5 346 500 Hz
 *  - Row 2 (Region 3): 5 367 000 Hz
 *
 * The oscillator is programmed to @c BAND_FREQ_HZ[region-1][band] + 1500 Hz
 * (the audio centre offset) at the start of each transmission window.
 *
 * Do not access this array directly; use @ref config_band_freq_hz() which
 * bounds-checks the region index and falls back to Region 1 on invalid input.
 */
extern const uint32_t BAND_FREQ_HZ[3][BAND_COUNT];

/**
 * @brief Retrieve the WSPR dial frequency for a given IARU region and band.
 *
 * This inline helper converts the 1-based @p region value to a 0-based array
 * index and returns the corresponding entry from @ref BAND_FREQ_HZ.  If
 * @p region is outside the valid range 1–3, Region 1 frequencies are used
 * and no error is raised.
 *
 * @param[in] region  IARU region (1–3).  Values outside this range silently
 *                    fall back to Region 1.
 * @param[in] band    WSPR band identifier.  Must be a valid @ref wspr_band_t
 *                    value less than @c BAND_COUNT.
 *
 * @return Dial frequency in Hz for the requested region and band.
 */
static inline uint32_t config_band_freq_hz(iaru_region_t region, wspr_band_t band) {
    int idx = (int)region - 1;
    if (idx < 0 || idx > 2)
        idx = 0;
    // Clamp band index to prevent out-of-bounds access on
    // BAND_FREQ_HZ if an invalid wspr_band_t value is passed (e.g. race condition
    // or uninitialized memory). Default to BAND_40M on any invalid band value.
    if ((int)band < 0 || (int)band >= (int)BAND_COUNT)
        band = BAND_40M;
    return BAND_FREQ_HZ[idx][(int)band];
}

/**
 * @brief Human-readable name strings for each band, indexed by @ref wspr_band_t.
 *
 * Each entry is a pointer to a string literal in read-only flash.  The strings
 * are used in log messages and in the web UI status display.  The mapping is:
 * @c {"2200m", "630m", "160m", "80m", "60m", "40m", "30m", "20m", "17m",
 *    "15m", "12m", "10m"}.
 */
extern const char *BAND_NAME[BAND_COUNT];

/**
 * @brief 3-bit hardware filter bank address for each band, indexed by @ref wspr_band_t.
 *
 * Each entry is a value in the range 0–7 that is written to the three GPIO
 * filter-select pins (A, B, C) by @c gpio_filter_select().  Multiple adjacent
 * bands may share the same filter if the LPF pass-band covers both.
 *
 * Default mapping:
 * | Band index | Band     | Filter ID | GPIO C | GPIO B | GPIO A |
 * |------------|----------|-----------|--------|--------|--------|
 * | 0          | 2200m    |     0     |   0    |   0    |   0    |
 * | 1          | 630m     |     0     |   0    |   0    |   0    |
 * | 2          | 160m     |     1     |   0    |   0    |   1    |
 * | 3          | 80m      |     2     |   0    |   1    |   0    |
 * | 4          | 60m      |     2     |   0    |   1    |   0    |
 * | 5          | 40m      |     3     |   0    |   1    |   1    |
 * | 6          | 30m      |     4     |   1    |   0    |   0    |
 * | 7          | 20m      |     5     |   1    |   0    |   1    |
 * | 8          | 17m      |     5     |   1    |   0    |   1    |
 * | 9          | 15m      |     6     |   1    |   1    |   0    |
 * | 10         | 12m      |     6     |   1    |   1    |   0    |
 * | 11         | 10m      |     7     |   1    |   1    |   1    |
 *
 * Modify the array in @c config.c to match your hardware LPF board layout.
 */
extern const uint8_t BAND_FILTER[BAND_COUNT];

/** @} */

/**
 * @defgroup config_struct Persistent configuration structure
 * @{
 */

/**
 * @brief Complete set of persistent WSPR transmitter configuration parameters.
 *
 * This structure is stored as a single NVS blob of fixed size.  Every field
 * is initialised by @ref config_defaults() and optionally overridden by
 * @ref config_load() from NVS.
 *
 * @warning Never add, remove, or reorder fields without also incrementing
 * @ref CONFIG_SCHEMA_VERSION.  A schema mismatch causes @ref config_load() to
 * discard the stored blob and use compile-time defaults instead.
 *
 * @note All string fields are NUL-terminated; @ref config_load() enforces this
 * by writing a NUL to the last byte of each character array after loading.
 */
typedef struct {
    /**
     * @brief Schema version stored at save time.
     *
     * Compared against @ref CONFIG_SCHEMA_VERSION on load.  A mismatch
     * causes the blob to be discarded and defaults to be used.
     */
    uint8_t version;

    /**
     * @brief Amateur radio callsign, NUL-terminated.
     *
     * Used as the WSPR source identifier.  Accepted formats:
     *  - Simple Type-1: 1–6 alphanumeric characters with a digit at position 2
     *    (e.g. @c "LU3VEA", @c "W1AW", @c "G4JNT").
     *  - Compound Type-2: PREFIX/CALL or CALL/SUFFIX
     *    (e.g. @c "PJ4/K1ABC", @c "K1ABC/P").
     *
     * Maximum length: @ref CALLSIGN_LEN − 1 = 11 characters.
     * Validated client-side in the web UI by the JavaScript validator.
     */
    char callsign[CALLSIGN_LEN];

    /**
     * @brief Maidenhead grid locator, NUL-terminated.
     *
     * Accepted lengths:
     *  - 4 characters (e.g. @c "GF05"): transmitted as a WSPR Type-1 message.
     *  - 6 characters (e.g. @c "GF05ab"): triggers alternating Type-1 +
     *    Type-3 transmissions to convey sub-square precision.
     *
     * Maximum storage: @ref LOCATOR_LEN − 1 = 6 characters.
     * Validated client-side in the web UI.
     */
    char locator[LOCATOR_LEN];

    /**
     * @brief Transmit power in dBm.
     *
     * WSPR specifies a discrete set of legal values:
     * 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 dBm.
     * The WSPR encoder rounds any other value to the nearest legal level.
     * The Kconfig default is 23 dBm (≈ 200 mW), suitable for most Si5351 + PA stages.
     */
    uint8_t power_dbm;

    /**
     * @brief Wi-Fi station SSID, NUL-terminated.
     *
     * The SSID of the access point the device should connect to.  Maximum
     * 32 characters (IEEE 802.11 limit).  If empty, the device starts in
     * soft-AP mode immediately without attempting a STA connection.
     */
    char wifi_ssid[33];

    /**
     * @brief Wi-Fi station password, NUL-terminated.
     *
     * The WPA2 pre-shared key for @ref wifi_ssid.  Maximum 64 characters.
     * Leave empty for open networks.  The web UI displays this field as
     * @c "(saved password)" once a password has been stored.
     */
    char wifi_pass[65];

    /**
     * @brief Per-band enable flags, indexed by @ref wspr_band_t.
     *
     * @c true means the band is included in the active band list used by the
     * scheduler.  If all bands are @c false, the scheduler falls back to 40 m
     * to ensure at least one band is always active.
     *
     * The web UI renders a checkbox grid for all 12 bands.  When the user saves
     * a new band selection, the web server sets @ref bands_changed to @c true
     * so that the scheduler task rebuilds its active-band list at the next
     * transmission boundary.
     */
    bool band_enabled[BAND_COUNT];

    /**
     * @brief Enable automatic frequency hopping.
     *
     * When @c true, the scheduler advances to the next enabled band each time
     * the hop interval (@ref hop_interval_sec) expires.  When @c false, the
     * device stays on the first enabled band indefinitely.
     */
    bool hop_enabled;

    /**
     * @brief Frequency hopping interval in seconds.
     *
     * The scheduler moves to the next band when
     * @c (current_time - last_hop_time) >= hop_interval_sec.
     * Minimum enforced value is 120 s (one WSPR transmission window).
     * Values below 120 are silently raised to 120 by @ref config_load().
     */
    uint32_t hop_interval_sec;

    /**
     * @brief NTP server hostname or IP address, NUL-terminated.
     *
     * Used only when @c CONFIG_WSPR_TIME_NTP is selected.  Passed directly
     * to @c esp_sntp_setservername().  The default is @c "pool.ntp.org".
     * Maximum 63 characters.
     */
    char ntp_server[64];

    /**
     * @brief Master TX enable flag.
     *
     * When @c false, the scheduler does not call @c wspr_transmit() regardless
     * of timing.  The web UI TX toggle button and @c /api/tx_toggle endpoint
     * flip this flag.  The flag is not saved to NVS by the toggle endpoint,
     * so it resets to @c false on every cold boot.
     */
    bool tx_enabled;

    /**
     * @brief TX duty cycle as a percentage (0–100).
     *
     * Controls how many available WSPR transmission slots are actually used.
     * The scheduler implements this with an accumulator: a slot is transmitted
     * only when the accumulator reaches or exceeds 100, then 100 is subtracted.
     *
     * Practical values:
     *  - @c 0  : never transmit (same effect as @ref tx_enabled = @c false).
     *  - @c 20 : transmit 1 in 5 slots — the WSPR standard recommendation.
     *  - @c 100: transmit every available slot.
     *
     * Values between 1 and 100 produce approximate duty cycles; the actual
     * slot selection is deterministic, not random.
     */
    uint8_t tx_duty_pct;

    /**
     * @brief Crystal oscillator frequency correction in parts-per-billion.
     *
     * Applied to the oscillator driver by @c oscillator_set_cal() at boot.
     * A positive value compensates for a fast crystal (lowers the output
     * frequency); a negative value compensates for a slow crystal.
     * Practical range: approximately ±100 000 ppb (±100 ppm).
     * The web UI provides an input field for this parameter.
     */
    int32_t xtal_cal_ppb;

    /**
     * @brief IARU region selection (1, 2, or 3).
     *
     * Stored as @c uint8_t for compactness; cast to @ref iaru_region_t before
     * passing to @ref config_band_freq_hz().  Values outside 1–3 are
     * normalised to 1 by @ref config_load().
     */
    uint8_t iaru_region;

    /**
     * @brief Flag set by the web server when band selection or hop state changes.
     *
     * The @c h_post_config() handler in @c web_server.c sets this to @c true
     * after saving a new configuration that changed @ref band_enabled or
     * @ref hop_enabled.  The scheduler's @c rebuild_active_bands() function
     * clears it to @c false after rebuilding the active-band list.
     *
     * This flag is @em not stored in NVS.  It is reset to @c false by both
     * @ref config_defaults() and @ref config_load() so the scheduler always
     * starts with a clean rebuild state.
     */
    bool bands_changed;

    /**
     * @brief Even/odd TX slot parity for Type-2 / Type-3 alternation.
     *
     * When @ref wspr_encode_type() returns @ref WSPR_MSG_TYPE_2 or
     * @ref WSPR_MSG_TYPE_3, @c wspr_transmit() alternates between encoding the
     * primary message (parity = 0) and the Type-3 companion message (parity = 1)
     * on successive even-minute windows.  The parity is toggled after each
     * transmission.
     *
     * This field is a runtime-only counter: it is reset to 0 on every cold boot
     * by @ref config_defaults() and @ref config_load(), and is @em not written
     * to NVS.  Persistent alternation state across power cycles is not required
     * because receiving stations decode both message types independently.
     */
    uint8_t tx_slot_parity;

    // Tone test mode: runtime-only, never persisted across reboots.
    // tone_active: when true the scheduler outputs a CW carrier at tone_freq_khz
    // instead of executing the normal WSPR schedule.
    bool tone_active;
    // tone_freq_khz: carrier frequency for tone test in kHz (0.1 to 30000.0).
    float tone_freq_khz;
} wspr_config_t;

/** @} */

/**
 * @defgroup config_api Configuration API
 * @{
 */

/**
 * @brief Initialise the NVS flash storage subsystem.
 *
 * Calls @c nvs_flash_init().  If the NVS partition has no free pages or
 * contains a format from a different SDK version (@c ESP_ERR_NVS_NO_FREE_PAGES
 * or @c ESP_ERR_NVS_NEW_VERSION_FOUND), the partition is erased and
 * re-initialised automatically.
 *
 * @note Must be called exactly once at startup, before @ref config_load() or
 *       @ref config_save().  In @c app_main() it is invoked via
 *       @c ESP_ERROR_CHECK(config_init()).
 *
 * @return @c ESP_OK on success.
 * @return @c ESP_ERR_NVS_NO_FREE_PAGES or @c ESP_ERR_NVS_NEW_VERSION_FOUND
 *         if the partition needs erasing (handled internally; these codes
 *         are never returned to the caller).
 * @return Any other @c esp_err_t code from @c nvs_flash_init() on hard failure.
 */
esp_err_t config_init(void);

/**
 * @brief Load the persistent configuration from NVS into @p cfg.
 *
 * The function always initialises @p cfg with @ref config_defaults() first,
 * ensuring a fully valid structure is returned even when NVS contains no data
 * or incompatible data.
 *
 * Load sequence:
 *  1. Call @ref config_defaults() to populate @p cfg with safe compile-time values.
 *  2. Open the @c "wspr" NVS namespace in read-only mode.
 *  3. If the namespace does not exist, return @c ESP_OK (defaults apply).
 *  4. Read the @c "cfg" blob.
 *  5. If the blob size differs from @c sizeof(wspr_config_t), discard it,
 *     set the @ref config_was_reset() flag, and return @c ESP_OK.
 *  6. If @c cfg->version != @ref CONFIG_SCHEMA_VERSION, discard it,
 *     set the @ref config_was_reset() flag, and return @c ESP_OK.
 *  7. NUL-terminate all string fields.
 *  8. Clamp out-of-range values (@ref iaru_region, @ref hop_interval_sec).
 *  9. Clear the runtime-only fields (@ref bands_changed, @ref tx_slot_parity).
 * 10. Log the loaded parameters and return @c ESP_OK.
 *
 * @param[out] cfg  Pointer to the @ref wspr_config_t to populate.
 *                  Must not be @c NULL.  On return the structure is fully
 *                  initialised regardless of the return code.
 *
 * @return @c ESP_OK on success (including cases where defaults were used).
 * @return An @c esp_err_t code from the NVS driver on unexpected I/O failure.
 */
esp_err_t config_load(wspr_config_t *cfg);

/**
 * @brief Persist the current configuration to NVS.
 *
 * Opens the @c "wspr" namespace in read-write mode, writes @p cfg as a blob
 * under key @c "cfg", and commits the transaction.  The caller is responsible
 * for ensuring @p cfg is a valid, fully initialised @ref wspr_config_t.
 *
 * @note In @c web_server.c this function is called from the HTTP POST handler
 *       @c h_post_config() while the config mutex is held, so concurrent
 *       modification by the scheduler task is prevented.
 *
 * @param[in] cfg  Pointer to the configuration to save.  Must not be @c NULL.
 *
 * @return @c ESP_OK on success.
 * @return An @c esp_err_t code from @c nvs_open(), @c nvs_set_blob(), or
 *         @c nvs_commit() on failure.
 */
esp_err_t config_save(const wspr_config_t *cfg);

/**
 * @brief Fill @p cfg with compile-time default values.
 *
 * Zeroes the structure with @c memset() then sets every field to a sensible
 * default:
 *  - @ref wspr_config_t::version : @ref CONFIG_SCHEMA_VERSION
 *  - @ref wspr_config_t::callsign : @c CONFIG_WSPR_DEFAULT_CALLSIGN
 *  - @ref wspr_config_t::locator  : @c CONFIG_WSPR_DEFAULT_LOCATOR
 *  - @ref wspr_config_t::power_dbm: @c CONFIG_WSPR_DEFAULT_POWER_DBM
 *  - @ref wspr_config_t::ntp_server: @c "pool.ntp.org"
 *  - @ref wspr_config_t::hop_enabled: @c false
 *  - @ref wspr_config_t::hop_interval_sec: @c 120
 *  - @ref wspr_config_t::tx_enabled: @c false
 *  - @ref wspr_config_t::band_enabled[BAND_40M] and [BAND_20M]: @c true (all others @c false)
 *  - @ref wspr_config_t::tx_duty_pct: @c 20 (WSPR standard recommendation)
 *  - @ref wspr_config_t::xtal_cal_ppb: @c 0
 *  - @ref wspr_config_t::iaru_region: @c IARU_REGION_1
 *  - @ref wspr_config_t::bands_changed: @c false
 *  - @ref wspr_config_t::tx_slot_parity: @c 0
 *
 * This function is called internally by @ref config_load() and may also be
 * called directly to reset the in-memory configuration to factory state
 * without touching NVS.
 *
 * @param[out] cfg  Pointer to the structure to initialise.  Must not be @c NULL.
 */
void config_defaults(wspr_config_t *cfg);

/**
 * @brief Report whether the last @ref config_load() fell back to defaults.
 *
 * Returns @c true if @ref config_load() encountered a schema version mismatch
 * or blob size mismatch and discarded the NVS data.  The flag is a static
 * module variable; it is set to @c false at program start and to @c true on
 * the first mismatch detection.  It is never cleared once set.
 *
 * The web server calls this function after startup to display a notice in the
 * UI warning the user that settings were reset to defaults.
 *
 * @return @c true  — stored config was incompatible; defaults are in use.
 * @return @c false — stored config was loaded successfully, or no config existed.
 */
bool config_was_reset(void);

/** @} */
