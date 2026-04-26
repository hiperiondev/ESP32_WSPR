/**
 * @file main.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_WSPR_TASK_WDT_ENABLE
#include "esp_task_wdt.h"
#endif

#include "config.h"
#include "gpio_filter.h"
#include "oscillator.h"
#include "time_sync.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "wspr_encode.h"

static const char *TAG = "wspr_main";

// WSPR tone spacing: 12000/8192 Hz = 1.4648 Hz per tone step.
// Represented as numerator/denominator to avoid floating-point at runtime.
// The oscillator driver receives offsets in milli-Hz, so:
//   tone_offset_mHz = symbol_value * 375000 / 256
// which equals symbol_value * 1464.844 mHz, matching the WSPR specification.
#define WSPR_TONE_NUM 375000UL // tone numerator: spacing_mHz * 256
#define WSPR_TONE_DEN 256UL    // tone denominator

// Exact WSPR symbol period: 8192/12000 s = 682666.666... us
// Expressed as: each symbol adds 682666 us plus 2/3 us fractional part.
// After 3 symbols the accumulated fraction equals exactly 2 us (3 * 2/3 = 2),
// so the long-term average is exact with zero drift over 162 symbols.
#define WSPR_SYM_PERIOD_US_INT   682666LL // integer part of symbol period in us
#define WSPR_SYM_PERIOD_FRAC_NUM 2LL      // fractional numerator: 2/3 us per symbol
#define WSPR_SYM_PERIOD_FRAC_DEN 3LL      // fractional denominator

// Map an arbitrary frequency in Hz to the best hardware LPF filter ID.
// Compares against all Region 1 WSPR band dial frequencies and returns the
// BAND_FILTER address for the closest band. Used by tone test mode to select
// the appropriate low-pass filter for the user-specified test frequency.
static uint8_t freq_to_filter(uint32_t freq_hz) {
    uint32_t min_diff = UINT32_MAX;
    int best_band = BAND_40M;
    for (int i = 0; i < BAND_COUNT; i++) {
        uint32_t bf = BAND_FREQ_HZ[0][i];
        uint32_t diff = (freq_hz >= bf) ? (freq_hz - bf) : (bf - freq_hz);
        if (diff < min_diff) {
            min_diff = diff;
            best_band = i;
        }
    }
    return BAND_FILTER[best_band];
}

// Global live config — shared between scheduler, status and web-server tasks
static wspr_config_t g_cfg;
// Uptime in seconds at boot, used to compute the boot wall-clock time after NTP sync
static uint32_t g_boot_uptime_sec = 0;
// Currently selected WSPR band index (-1 = not yet selected)
static volatile int g_band_idx = -1;
// Active band list and round-robin pointer for frequency hopping
static int g_hop_active_bands[BAND_COUNT];
static int g_hop_active_count = 0;
static int g_hop_ptr = 0;
// Transmission state flags read by the status task
static volatile bool g_tx_active = false;
static volatile int g_symbol_idx = 0;
// Pre-arm state: oscillator and LPF are set up one second before TX starts
static bool g_pre_armed = false;
static uint32_t g_pre_arm_base_hz = 0;

static const char *reset_reason_to_str(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:
            return "Power-on";
        case ESP_RST_EXT:
            return "External pin";
        case ESP_RST_SW:
            return "Software";
        case ESP_RST_PANIC:
            return "Panic/exception";
        case ESP_RST_INT_WDT:
            return "Interrupt watchdog";
        case ESP_RST_TASK_WDT:
            return "Task watchdog";
        case ESP_RST_WDT:
            return "Other watchdog";
        case ESP_RST_DEEPSLEEP:
            return "Deep-sleep wake";
        case ESP_RST_BROWNOUT:
            return "Brownout";
        case ESP_RST_SDIO:
            return "SDIO reset";
        default:
            return "Unknown";
    }
}

static void rebuild_active_bands(void) {
    // Save the band currently pointed to before rebuilding the list.
    int prev_band = (g_hop_ptr < g_hop_active_count) ? g_hop_active_bands[g_hop_ptr] : -1;

    g_hop_active_count = 0;
    for (int i = 0; i < BAND_COUNT; i++) {
        if (g_cfg.band_enabled[i])
            g_hop_active_bands[g_hop_active_count++] = i;
    }

    // Safety fallback: always have at least one active band
    if (g_hop_active_count == 0) {
        g_hop_active_bands[0] = BAND_40M;
        g_hop_active_count = 1;
    }

    g_hop_ptr = 0; // default: fall back to first band if prev_band not found
    if (prev_band >= 0) {
        for (int i = 0; i < g_hop_active_count; i++) {
            if (g_hop_active_bands[i] == prev_band) {
                g_hop_ptr = i;
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Active bands: %d", g_hop_active_count);
    // Clear the flag so the scheduler does not call rebuild again next slot
    g_cfg.bands_changed = false;
}

static void select_next_band(bool force_next, bool force_rebuild) {
    if (force_rebuild || g_hop_active_count == 0)
        rebuild_active_bands();

    // Advance pointer only when hopping is enabled and a new band is requested
    if (g_cfg.hop_enabled && force_next) {
        g_hop_ptr = (g_hop_ptr + 1) % g_hop_active_count;
    }
    g_band_idx = g_hop_active_bands[g_hop_ptr];
    ESP_LOGI(TAG, "Selected band: %s (%lu Hz)", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx));
}

// Per the WSPR 2.0 specification (K1JT), Type-3 companion messages are defined
// ONLY as companions to Type-2 messages (compound callsigns containing '/').
// There is NO standard defined for a simple-callsign Type-1 + Type-3
// alternation. WSJT-X itself never transmits this combination.
//
// When a receiver decodes a Type-3 message that was NOT preceded by a Type-2
// from the same station, the 28-bit field (containing pack_loc6_type3 output)
// is interpreted as a Type-1 callsign, yielding a phantom callsign (e.g.
// "4G8ILP") and the 15-bit hash of the real callsign appears as a phantom
// locator (e.g. "LL59"). These phantom spots are uploaded to WSPRnet,
// polluting the propagation database with non-existent stations.
// The WSPR_MSG_TYPE_3 case (simple callsign + 6-char locator) is separated
// from the WSPR_MSG_TYPE_2 case (compound callsign). For TYPE_3, both parity
// slots now encode a standard Type-1 message using only the first 4 characters
// of the locator. No Type-3 companion is ever transmitted. Parity is not
// toggled (no alternation needed). This matches WSJT-X WSPR beacon behavior
// and produces clean, unambiguous spots on WSPRnet.
//
// The TYPE_2 path (compound callsign, parity alternation between Type-2
// primary and Type-3 companion) is completely unchanged.
static void wspr_transmit(void) {
    uint8_t symbols[WSPR_SYMBOLS];
    int enc_result;
    char snap_callsign[CALLSIGN_LEN];
    char snap_locator[LOCATOR_LEN];
    uint8_t snap_power;
    uint8_t snap_parity;
    uint8_t snap_region;
    web_server_cfg_lock();
    memcpy(snap_callsign, g_cfg.callsign, CALLSIGN_LEN);
    memcpy(snap_locator, g_cfg.locator, LOCATOR_LEN);
    snap_power = g_cfg.power_dbm;
    snap_parity = g_cfg.tx_slot_parity;
    snap_region = g_cfg.iaru_region;
    web_server_cfg_unlock();

    if (snap_callsign[0] == '\0' || snap_locator[0] == '\0') {
        ESP_LOGE(TAG, "TX skipped: callsign or locator is empty");
        g_pre_armed = false;
        return;
    }

    wspr_msg_type_t msg_type = wspr_encode_type(snap_callsign, snap_locator);

    if (msg_type == WSPR_MSG_TYPE_1) {
        // Standard Type-1: simple callsign + 4-char locator. No alternation.
        enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);

        // WSPR_MSG_TYPE_3 (simple callsign + 6-char locator)
        // is handled separately from WSPR_MSG_TYPE_2 (compound callsign).
        // A Type-3 companion MUST NOT be sent for simple callsigns: doing so
        // generates phantom spots (e.g. "4G8ILP LL59") on WSPRnet because receiving
        // stations decode the Type-3 28-bit locator field as a Type-1 callsign.
        // Per the WSPR 2.0 spec and WSJT-X beacon practice, simple callsigns always
        // use Type-1 only, truncating the locator to the first 4 characters.
        // Parity is not toggled and no companion slot is needed.
    } else if (msg_type == WSPR_MSG_TYPE_3) {
        // Simple callsign + 6-char locator: transmit Type-1 only.
        // Truncate the locator to 4 characters for Type-1 encoding.
        // The sub-square precision (chars 5-6) is intentionally NOT transmitted
        // because the Type-3 companion would create phantom spots.
        char loc4[5];
        loc4[0] = snap_locator[0];
        loc4[1] = snap_locator[1];
        loc4[2] = snap_locator[2];
        loc4[3] = snap_locator[3];
        loc4[4] = '\0';
        enc_result = wspr_encode(snap_callsign, loc4, snap_power, symbols);
        // No parity toggle: no alternation, no companion message.
        ESP_LOGD(TAG, "TYPE_3 path: sending Type-1 only (loc truncated to 4 chars '%s')", loc4);

    } else {
        // WSPR_MSG_TYPE_2: compound callsign (contains '/').
        // Alternates between Type-2 primary (parity==0) and Type-3 companion
        // (parity==1). The Type-3 companion is valid here because the receiver
        // links it to the Type-2 via the 15-bit callsign hash.
        if (snap_parity == 0) {
            enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
        } else {
            // For compound callsigns (Type-2), skip the Type-3
            // companion when the stored locator is shorter than 6 characters.
            // Transmitting a fabricated "aa" sub-square would report a wrong location
            // to worldwide receivers. Reset parity to 0 so next slot is the Type-2.
            size_t loc_len = strlen(snap_locator);
            if (loc_len < 6) {
                ESP_LOGW(TAG, "Type-3 skipped: locator '%s' is only %zu chars (need 6 for compound callsign companion)", snap_locator, loc_len);
                web_server_cfg_lock();
                g_cfg.tx_slot_parity = 0; // reset parity — next slot will be Type-2 again
                web_server_cfg_unlock();
                g_pre_armed = false;
                return;
            }
            char loc6[7];
            memcpy(loc6, snap_locator, 6);
            loc6[6] = '\0';
            enc_result = wspr_encode_type3(snap_callsign, loc6, snap_power, symbols);
        }

        // Toggle parity ONLY on successful encode, not before.
        if (enc_result >= 0) {
            web_server_cfg_lock();
            g_cfg.tx_slot_parity ^= 1u;
            web_server_cfg_unlock();
        }
    }

    if (enc_result < 0) {
        ESP_LOGE(TAG,
                 "WSPR encode failed: cs='%s' loc='%s' -- "
                 "simple callsign: 1-6 chars with digit at pos 2 (e.g. LU3VEA, W1AW, G4JNT); "
                 "compound callsign: PREFIX/CALL or CALL/SUFFIX (e.g. PJ4/K1ABC, K1ABC/P); "
                 "locator: 4 chars (AA00, e.g. GF05) or 6 chars (AA00AA, e.g. GF05aa)",
                 snap_callsign, snap_locator);
        g_pre_armed = false;
        return;
    }

    uint32_t base_hz;
    if (g_pre_armed) {
        base_hz = g_pre_arm_base_hz;
    } else {
        // BAND_FREQ_HZ already contains the RF center frequency (SSB dial + 1500 Hz); no additional offset
        base_hz = config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx);
        oscillator_set_freq(base_hz);
        gpio_filter_select(BAND_FILTER[g_band_idx]);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
    }

    g_pre_armed = false;

    // oscillator_tx_begin() sets _osc_tx_active=true which defers any
    // calibration change inside oscillator_set_cal(). If a web-UI calibration
    // update arrives between si_apply_tone() (inside set_freq_mhz) and the
    // previous oscillator_tx_begin() call, oscillator_set_cal() would invalidate
    // the Si5351 band cache (_si_cache.valid=false) BEFORE the TX window is
    // protected. The next si_apply_tone() call in the symbol loop would then
    // return ESP_ERR_INVALID_STATE, causing the Si5351 to hold the last written
    // PLL value for the remainder of the WSPR frame -- all remaining symbols at
    // wrong frequency, frame undecodable.
    oscillator_tx_begin(); // protect cache from cal races BEFORE first si_apply_tone

    // Pre-compute all 162 tone offsets before enabling RF output.
    // This separates the pure-integer precomputation from the timing-critical loop,
    // and enables the restructured loop to set each tone AFTER the deadline for the
    // previous symbol is met (at the boundary) rather than ~400 us early.
    int32_t tone_offsets[WSPR_SYMBOLS];
    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        // round tone offset: (sym*375000 + 128) / 256 — the +128 is half-step rounding
        tone_offsets[i] = (int32_t)((symbols[i] * 375000UL + 128UL) / 256UL);
    }

    // apply symbol 0 tone BEFORE enabling RF output so the oscillator
    // is at the correct frequency the instant it is keyed.
    oscillator_set_freq_mhz(base_hz, tone_offsets[0]);

    oscillator_enable(true);
    g_tx_active = true;

    // Use 64-bit esp_timer_get_time() directly for the TX
    // start reference. This avoids the 32-bit truncation in timer_us32() and
    // eliminates any theoretical wrap-around ambiguity over long uptimes.
    int64_t tx_start_us = esp_timer_get_time();

    // Fractional accumulator for the 2/3 us per-symbol
    // fractional part of the WSPR symbol period (682666.666... us).
    // frac_accum counts accumulated thirds; every 3 symbols we get +2 us extra.
    // This gives bit-exact timing across all 162 symbols with zero long-term drift.
    int64_t frac_accum = 0;

    // ── TX timing diagnostic ──
    struct timeval tv_tx;
    gettimeofday(&tv_tx, NULL);
    struct tm tm_tx;
    gmtime_r(&tv_tx.tv_sec, &tm_tx);
    uint32_t tx_sec = (uint32_t)(tv_tx.tv_sec % 120u); // phase within 2-min cycle
    uint32_t tx_usec = (uint32_t)tv_tx.tv_usec;

    // Classify alignment quality for easy log grepping
    const char *align_verdict;
    if (tx_sec == 1u && tx_usec < 50000u)
        align_verdict = "OK";
    else if (tx_sec == 1u && tx_usec < 200000u)
        align_verdict = "LATE_USEC";
    else if (tx_sec == 0u)
        align_verdict = "EARLY";
    else if (tx_sec >= 2u && tx_sec <= 4u)
        align_verdict = "VERY_LATE";
    else
        align_verdict = "BAD_PHASE";

    ESP_LOGI(TAG,
             "TX start: %02d:%02d:%02d.%06lu UTC  "
             "phase=%lu s  usec=%lu  verdict=%s  "
             "band=%s  freq=%lu Hz  type=%d  parity=%d  "
             "pre_armed=%d  stack_HWM=%u B",
             tm_tx.tm_hour, tm_tx.tm_min, tm_tx.tm_sec, (unsigned long)tv_tx.tv_usec, (unsigned long)tx_sec, (unsigned long)tx_usec, align_verdict,
             BAND_NAME[g_band_idx], (unsigned long)base_hz, (int)msg_type, (int)snap_parity, (int)(g_pre_arm_base_hz != 0),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    if (tx_sec != 1u) {
        ESP_LOGW(TAG,
                 "TX TIMING WARNING: symbol 0 started at phase=%lu s (expected 1). "
                 "Check NTP/GPS sync and fine-alignment loop.",
                 (unsigned long)tx_sec);
    }
    // ── end TX timing diagnostic ──

    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

        // check tone_active on every symbol (abort TX if tone test activated)
        {
            web_server_cfg_lock();
            bool tone_abort = g_cfg.tone_active;
            web_server_cfg_unlock();
            if (tone_abort) {
                ESP_LOGW(TAG, "WSPR TX aborted at symbol %d: tone test activated", i);
                break;
            }
        }

#if CONFIG_WSPR_TASK_WDT_ENABLE
        esp_task_wdt_reset();
#endif

        // Compute the exact deadline for the END of symbol i
        // using 64-bit arithmetic and the fraction accumulator.
        // Symbol period = 682666 + 2/3 us. Each symbol adds WSPR_SYM_PERIOD_FRAC_NUM
        // to frac_accum; when frac_accum reaches WSPR_SYM_PERIOD_FRAC_DEN we add 1 us.
        frac_accum += WSPR_SYM_PERIOD_FRAC_NUM;
        int64_t extra_us = frac_accum / WSPR_SYM_PERIOD_FRAC_DEN;
        frac_accum %= WSPR_SYM_PERIOD_FRAC_DEN;
        int64_t target_us = tx_start_us + (int64_t)(i + 1) * WSPR_SYM_PERIOD_US_INT + extra_us;

        // Timing wait with graduated sleep strategy.
        // Far from deadline: sleep in chunks to yield to other tasks.
        // Within 10 ms: short 1-tick sleep for finer resolution.
        // Within 2 ms: busy-spin for sub-100 us accuracy (Wi-Fi on PRO_CPU
        // when scheduler is pinned to APP_CPU per Bug 8 fix in app_main).
        for (;;) {
            int64_t now_us = esp_timer_get_time();
            int64_t rem_us = target_us - now_us;
            if (rem_us <= 0)
                break;
            if (rem_us > 10000LL) {
                // Sleep most of the remaining time minus 5 ms safety
                // margin. vTaskDelay(1) is at least 1 tick; with CONFIG_HZ=1000 that
                // is 1 ms, with CONFIG_HZ=100 it is 10 ms.
                int32_t sleep_ms = (int32_t)((rem_us - 5000LL) / 1000LL);
                if (sleep_ms > 0)
                    vTaskDelay(pdMS_TO_TICKS((uint32_t)sleep_ms));
            } else if (rem_us > 2000LL) {
                vTaskDelay(1); // one tick sleep for the 2-10 ms window
            }
            // else: busy-spin for the final 2 ms
            // On APP_CPU with Wi-Fi on PRO_CPU this gives < 100 us accuracy
        }

#if CONFIG_WSPR_SYMBOL_OVERRUN_LOG

        int64_t actual_us = esp_timer_get_time() - tx_start_us;
        int64_t deadline_us = (int64_t)(i + 1) * WSPR_SYM_PERIOD_US_INT + extra_us;
        if (actual_us > deadline_us + 10000LL) {
            ESP_LOGW(TAG, "Symbol %d overrun: deadline=%lld actual=%lld overrun=%lld us", i, (long long)deadline_us, (long long)actual_us,
                     (long long)(actual_us - deadline_us));
        }

#endif

        // Set the NEXT symbol's tone frequency AFTER the
        // current symbol's deadline has been met, not before the wait.
        // This ensures every symbol boundary transition happens at the exact
        // symbol boundary rather than ~400 us early (I2C transaction time).
        if (i + 1 < WSPR_SYMBOLS) {
            // Check return value and abort on I2C error.
            // A failed set_freq_mhz during the symbol loop means remaining symbols
            // would be transmitted at the wrong frequency (undecodable frame).
            esp_err_t osc_err = oscillator_set_freq_mhz(base_hz, tone_offsets[i + 1]);
            if (osc_err != ESP_OK) {
                ESP_LOGE(TAG, "oscillator_set_freq_mhz failed at symbol %d->%d: %s -- aborting TX", i, i + 1, esp_err_to_name(osc_err));
                break;
            }
        }
    }

    oscillator_enable(false);
    g_tx_active = false;
    oscillator_tx_end();
    g_symbol_idx = 0;

    ESP_LOGI(TAG, "TX complete");
}

static void status_task(void *arg) {
    char time_str[24] = "---";
    char freq_str[24] = "---";
    bool reboot_info_time_set = false;

    while (1) {
        struct timeval tv;
        bool time_ok = time_sync_get(&tv);

        if (time_ok) {
            struct tm t;
            gmtime_r(&tv.tv_sec, &t);
            strftime(time_str, sizeof(time_str), "%H:%M:%S UTC", &t);

            if (!reboot_info_time_set) {
                time_t boot_ts = tv.tv_sec - (time_t)g_boot_uptime_sec;
                struct tm bt;
                gmtime_r(&boot_ts, &bt);
                char boot_str[32];
                strftime(boot_str, sizeof(boot_str), "%Y-%m-%d %H:%M UTC", &bt);
                web_server_set_reboot_info(boot_str, NULL);
                reboot_info_time_set = true;
            }
        }

        const char *band_name = (g_band_idx >= 0) ? BAND_NAME[g_band_idx] : "---";
        web_server_cfg_lock();
        uint8_t region_snap = g_cfg.iaru_region;
        bool tx_en = g_cfg.tx_enabled;
        web_server_cfg_unlock();

        if (g_band_idx >= 0) {
            // BAND_FREQ_HZ already stores the RF center frequency (SSB dial + 1500 Hz); no additional offset needed
            uint32_t freq_hz = config_band_freq_hz((iaru_region_t)region_snap, (wspr_band_t)g_band_idx);
            uint32_t mhz_int = freq_hz / 1000000u;
            uint32_t khz_frac = (freq_hz % 1000000u) / 100u;
            snprintf(freq_str, sizeof(freq_str), "%u.%04u MHz", (unsigned)mhz_int, (unsigned)khz_frac);
        }

        int32_t next_tx = time_ok ? time_sync_secs_to_next_tx() : -1;

        web_server_update_status(time_ok, time_str, band_name, freq_str, next_tx, g_tx_active, tx_en, g_symbol_idx);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Evaluate hop selection and duty-cycle accumulator FIRST, before the
// coarse sleep and fine alignment loop. Only enter the pre-arm loop when
// do_tx is true. Duty-cycle skips now happen before any hardware interaction.
static void scheduler_task(void *arg) {
    ESP_LOGI(TAG, "Scheduler started -- waiting for time sync");

#if CONFIG_WSPR_TASK_WDT_ENABLE

    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK)
        ESP_LOGW(TAG, "WDT add failed (%s) -- watchdog not active for scheduler task", esp_err_to_name(wdt_err));

#endif

    // Initial band selection: build the active list and choose the first band
    select_next_band(false, true);

    // The last frequency hop wall-clock second.
    uint32_t s_last_hop_time_sec = 0;
    bool s_hop_anchored = false;
    bool s_duty_primed = false;
    uint16_t s_duty_accum = 0u;
    // Track previous duty value to detect web-UI changes and reset accumulator
    uint8_t s_prev_duty = 0u;

    bool time_synced_logged = false;
    // Tone test mode: tracks oscillator state across loop iterations
    bool s_tone_was_active = false;
    uint32_t s_tone_last_hz = 0u;

    while (1) {
        // Tone test mode: takes priority over all WSPR scheduling.
        web_server_cfg_lock();
        bool tone_snap = g_cfg.tone_active;
        float tone_freq_snap = g_cfg.tone_freq_khz;
        web_server_cfg_unlock();
        if (tone_snap) {
            uint32_t tone_hz = (uint32_t)(tone_freq_snap * 1000.0f);
            bool _cache_invalid = s_tone_was_active && !oscillator_cache_valid();
            if (!s_tone_was_active || tone_hz != s_tone_last_hz || _cache_invalid) {
                // Disable output before reprogramming to avoid spurious
                // tone during Si5351 PLL reset transient.
                if (s_tone_was_active) {
                    oscillator_enable(false);
                    g_tx_active = false;
                }
                s_tone_last_hz = tone_hz;
                uint8_t tone_filter = freq_to_filter(tone_hz);
                gpio_filter_select(tone_filter);
                vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
                oscillator_set_freq(tone_hz);
                oscillator_enable(true);
                g_tx_active = true;
                if (!s_tone_was_active) {
                    ESP_LOGI(TAG, "Tone test activated: %.3f kHz (%lu Hz) filter=%u", (double)tone_freq_snap, (unsigned long)tone_hz, (unsigned)tone_filter);
                } else {
                    ESP_LOGI(TAG, "Tone frequency updated: %.3f kHz", (double)tone_freq_snap);
                }
                s_tone_was_active = true;
                if (_cache_invalid) {
                    ESP_LOGI(TAG, "Tone cal update applied: %.3f kHz reprogrammed with new calibration", (double)tone_freq_snap);
                }
            }
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        // Tone mode was active but is now deactivated: restore idle state
        if (s_tone_was_active) {
            oscillator_enable(false);
            g_tx_active = false;
            s_tone_was_active = false;
            s_tone_last_hz = 0u;
            g_pre_armed = false;
            g_pre_arm_base_hz = 0;
            ESP_LOGI(TAG, "Tone test deactivated, resuming normal WSPR scheduling");
        }

        // Gate on time sync: WSPR transmissions require UTC accuracy within +-1 s
        if (!time_sync_is_ready()) {
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!time_synced_logged) {
            ESP_LOGI(TAG, "Time synced -- entering TX schedule loop");
            time_synced_logged = true;
        }

        // Check if transmission is globally enabled
        web_server_cfg_lock();
        bool tx_enabled_snap = g_cfg.tx_enabled;
        web_server_cfg_unlock();

        if (!tx_enabled_snap) {
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        uint32_t now = (uint32_t)(tv_now.tv_sec & 0xFFFFFFFFUL);

        web_server_cfg_lock();
        bool hop_en = g_cfg.hop_enabled;
        uint32_t hop_intv = g_cfg.hop_interval_sec;
        bool need_rebuild = g_cfg.bands_changed;
        g_cfg.bands_changed = false;
        uint8_t duty = g_cfg.tx_duty_pct;
        web_server_cfg_unlock();

        if (hop_intv == 0u)
            hop_intv = 120u;

        // Snap hop_intv to the nearest multiple of 120 s
        // so frequency hops always align with even-minute TX boundaries.
        // A non-multiple hop interval (e.g. 600 s) would otherwise cause hops
        // at odd offsets within the 2-minute cycle, wasting partial TX slots.
        hop_intv = ((hop_intv + 60u) / 120u) * 120u;
        if (hop_intv < 120u)
            hop_intv = 120u;

        // Duty accumulator reset on duty-percentage change.
        if (duty != s_prev_duty) {
            s_duty_primed = false;
            s_duty_accum = 0u;
            s_prev_duty = duty;
            ESP_LOGI(TAG, "Duty cycle changed to %u%% -- accumulator reset", (unsigned)duty);
        }

        // Hop timer: compare elapsed seconds against hop_interval_sec.
        bool do_hop;
        if (!s_hop_anchored) {
            do_hop = false;
            // Anchor hop timer to the nearest even-minute
            // UTC boundary rather than the raw current second. This ensures the
            // first and subsequent hops align with WSPR even-minute slots.
            s_last_hop_time_sec = (now / 120u) * 120u;
            s_hop_anchored = true;
        } else {
            do_hop = hop_en && ((now - s_last_hop_time_sec) >= hop_intv);
        }

        if (do_hop || g_band_idx < 0 || need_rebuild) {
            select_next_band(do_hop, need_rebuild || g_band_idx < 0);
            if (do_hop) {
                // Snap the last hop time to the even-minute
                // boundary to prevent drift accumulation in the hop timer.
                s_last_hop_time_sec = (now / 120u) * 120u;
            }
            if (g_pre_armed) {
                web_server_cfg_lock();
                uint8_t new_region = g_cfg.iaru_region;
                web_server_cfg_unlock();
                uint32_t new_base_hz = config_band_freq_hz((iaru_region_t)new_region, (wspr_band_t)g_band_idx);
                if (new_base_hz != g_pre_arm_base_hz) {
                    g_pre_armed = false;
                    g_pre_arm_base_hz = 0;
                }
            }
        }

        // Prime the duty accumulator on the very first slot so that
        // the first TX fires immediately.
        if (!s_duty_primed) {
            if (duty > 0u && duty < 100u) {
                s_duty_accum = (uint16_t)(100u - duty);
            } else if (duty >= 100u) {
                s_duty_accum = 0u;
            }
            s_duty_primed = true;
        }

        bool do_tx = false;
        if (duty > 0u) {
            if (duty >= 100u) {
                do_tx = true;
            } else {
                s_duty_accum += (uint16_t)duty;
                if (s_duty_accum >= 100u) {
                    s_duty_accum -= 100u;
                    do_tx = true;
                }
            }
        }

        if (!do_tx) {
            ESP_LOGI(TAG, "Duty cycle skip (pct=%u accum=%u)", (unsigned)duty, (unsigned)s_duty_accum);
            g_pre_armed = false;
            g_pre_arm_base_hz = 0;

            if (s_duty_accum > 200u)
                s_duty_accum = 200u;

#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            {
                uint32_t duty_guard_ms = 2000u;
                while (duty_guard_ms > 0u) {
                    uint32_t chunk = (duty_guard_ms > 100u) ? 100u : duty_guard_ms;
                    vTaskDelay(pdMS_TO_TICKS(chunk));
                    duty_guard_ms -= chunk;
#if CONFIG_WSPR_TASK_WDT_ENABLE
                    esp_task_wdt_reset();
#endif
                    web_server_cfg_lock();
                    bool tone_duty_chk = g_cfg.tone_active;
                    web_server_cfg_unlock();
                    if (tone_duty_chk)
                        goto duty_skip_done;
                }
            }
            for (int _guard = 0; _guard < 10; _guard++) {
                vTaskDelay(pdMS_TO_TICKS(100));
#if CONFIG_WSPR_TASK_WDT_ENABLE
                esp_task_wdt_reset();
#endif
                web_server_cfg_lock();
                bool tone_duty_g = g_cfg.tone_active;
                web_server_cfg_unlock();
                if (tone_duty_g)
                    goto duty_skip_done;
                if (time_sync_secs_to_next_tx() > 0)
                    break;
            }
        duty_skip_done:
            continue;
        }

        // Duty cycle approved: proceed to coarse sleep + fine alignment + TX.
        bool do_skip = false;

        int32_t wait_sec = time_sync_secs_to_next_tx();
        ESP_LOGI(TAG, "Next TX in %d s (band=%s)", wait_sec, BAND_NAME[g_band_idx]);

        if (wait_sec > 3) {
            uint32_t sleep_ms = (uint32_t)(wait_sec - 3) * 1000u;
            while (sleep_ms > 0u) {
                uint32_t chunk = (sleep_ms > 200u) ? 200u : sleep_ms;
                vTaskDelay(pdMS_TO_TICKS(chunk));
#if CONFIG_WSPR_TASK_WDT_ENABLE
                esp_task_wdt_reset();
#endif
                sleep_ms -= chunk;
                web_server_cfg_lock();
                bool tone_coarse = g_cfg.tone_active;
                web_server_cfg_unlock();
                if (tone_coarse) {
                    sleep_ms = 0u; // break out of sleep; tone loop at top will handle it
                }
            }
        }

        // Fine alignment loop: spin until phase == 0 (pre-arm) then phase == 1 (TX)
        web_server_cfg_lock();
        uint8_t pre_snap_region = g_cfg.iaru_region;
        web_server_cfg_unlock();
        bool pre_armed_local = false;

        for (;;) {
            struct timeval tv_align;
            gettimeofday(&tv_align, NULL);
            uint32_t phase = (uint32_t)(tv_align.tv_sec % 120u);

            if (phase == 0u) {
                if (!pre_armed_local) {
                    g_pre_arm_base_hz = config_band_freq_hz((iaru_region_t)pre_snap_region, (wspr_band_t)g_band_idx);
                    oscillator_set_freq(g_pre_arm_base_hz);
                    gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                    g_pre_armed = true;
                    pre_armed_local = true;
                    ESP_LOGD(TAG, "Pre-armed: band=%s base_hz=%lu (phase=0)", BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                }
                // Sleep proportionally to remaining time in the phase==0 window instead of a fixed 1 ms.
                // With CONFIG_HZ=100, pdMS_TO_TICKS(1)=0, causing a 100% CPU busy
                // loop for up to 1 second, starving the web server and status task.
                // Compute how much of the current second remains and sleep most of
                // it, waking ~100 ms before the second rolls to phase==1.
                {
                    struct timeval tv_phase0;
                    gettimeofday(&tv_phase0, NULL);
                    uint32_t usec_into_second = (uint32_t)tv_phase0.tv_usec;
                    uint32_t remaining_ms = (1000000u - usec_into_second) / 1000u;
                    if (remaining_ms > 200u) {
                        vTaskDelay(pdMS_TO_TICKS(remaining_ms - 100u));
                    } else if (remaining_ms > 20u) {
                        vTaskDelay(pdMS_TO_TICKS(10u));
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1u));
                    }
                }
            } else if (phase == 1u) {
                if (!pre_armed_local) {
                    g_pre_arm_base_hz = config_band_freq_hz((iaru_region_t)pre_snap_region, (wspr_band_t)g_band_idx);
                    oscillator_set_freq(g_pre_arm_base_hz);
                    gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                    vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
                    g_pre_armed = true;
                    pre_armed_local = true;
                    ESP_LOGW(TAG, "Late pre-arm at phase=1: band=%s base_hz=%lu", BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                }

                // Sub-second alignment spin: exit when usec < 50000.
                // This prevents the spin from overshooting the 50 ms
                // window due to tick quantisation (10 ms per tick at CONFIG_HZ=100).
                for (;;) {
                    struct timeval tv_sub;
                    gettimeofday(&tv_sub, NULL);
                    uint32_t cur_phase = (uint32_t)(tv_sub.tv_sec % 120u);
                    // Exit immediately if phase has changed (rolled past :01)
                    if (cur_phase != 1u)
                        break;
                    // Exit when usec is within the 50ms alignment window
                    if ((uint32_t)tv_sub.tv_usec < 50000u)
                        break;
                    // Compute remaining time before the window
                    // and use a 1 ms sleep when more than 10 ms away, otherwise
                    // busy-spin for sub-millisecond accuracy.
                    uint32_t rem_us = 1000000u - (uint32_t)tv_sub.tv_usec;
                    if (rem_us > 10000u) {
                        vTaskDelay(pdMS_TO_TICKS(1u)); // sleep 1 ms when far
                    }
                    // else: busy-spin for the last 10 ms
                }

                // Post-spin verify: check BOTH phase AND usec.
                // Original code only checked phase (second), missing the case where
                // the spin exits with phase==1 but usec >= 50000 (late start).
                // Also verify the sub-second alignment is within the window.
                struct timeval tv_verify;
                gettimeofday(&tv_verify, NULL);
                uint32_t verify_phase = (uint32_t)(tv_verify.tv_sec % 120u);
                if (verify_phase != 1u || (uint32_t)tv_verify.tv_usec >= 50000u) {
                    // Phase has rolled past :01 or usec window was missed
                    ESP_LOGW(TAG,
                             "Sub-second spin exited outside window: phase=%lu usec=%lu "
                             "(expected phase=1 usec<50000) -- slot skipped",
                             (unsigned long)verify_phase, (unsigned long)tv_verify.tv_usec);
                    g_pre_armed = false;
                    g_pre_arm_base_hz = 0;
                    do_skip = true;
                }
                // phase==1 and usec<50000: alignment is good, proceed with TX

                break; // exit the outer fine-alignment for(;;) loop
            } else if (phase >= 2u && phase <= 4u) {
                ESP_LOGW(TAG, "Missed TX window (phase=%lu), skipping slot", (unsigned long)phase);
                g_pre_armed = false;
                g_pre_arm_base_hz = 0;
                do_skip = true;
                break;
            } else {
                // phase >= 5 -- sleep proportionally to distance from :00
                uint32_t dist = (phase < 120u) ? (120u - phase) : 1u;
                uint32_t chunk_ms = (dist > 1u) ? ((dist - 1u) * 1000u) : 1u;
                if (chunk_ms > 200u)
                    chunk_ms = 200u;
                vTaskDelay(pdMS_TO_TICKS(chunk_ms));
            }

#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            web_server_cfg_lock();
            bool still_en = g_cfg.tx_enabled;
            bool tone_req = g_cfg.tone_active;
            web_server_cfg_unlock();

            if (!still_en || tone_req) {
                g_pre_armed = false;
                g_pre_arm_base_hz = 0;
                do_skip = true;
                break;
            }
        }

        if (do_skip) {
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif

            uint32_t guard_ms = 2000u;
            while (guard_ms > 0u) {
                uint32_t chunk = (guard_ms > 100u) ? 100u : guard_ms;
                vTaskDelay(pdMS_TO_TICKS(chunk));
                guard_ms -= chunk;
#if CONFIG_WSPR_TASK_WDT_ENABLE
                esp_task_wdt_reset();
#endif
                web_server_cfg_lock();
                bool tone_skip_check = g_cfg.tone_active;
                web_server_cfg_unlock();
                if (tone_skip_check)
                    goto skip_post_guard;
            }

            for (int _guard = 0; _guard < 10; _guard++) {
                vTaskDelay(pdMS_TO_TICKS(100));
#if CONFIG_WSPR_TASK_WDT_ENABLE
                esp_task_wdt_reset();
#endif
                web_server_cfg_lock();
                bool tone_guard_check = g_cfg.tone_active;
                web_server_cfg_unlock();
                if (tone_guard_check)
                    goto skip_post_guard;
                if (time_sync_secs_to_next_tx() > 0)
                    break;
            }
        skip_post_guard:
            continue;
        }

        // All checks passed: encode and transmit the WSPR message for this slot
        wspr_transmit();
    }
}

void app_main(void) {
    g_boot_uptime_sec = (uint32_t)(esp_log_timestamp() / 1000u);

    esp_reset_reason_t reset_rsn = esp_reset_reason();

    ESP_LOGI(TAG, "=== WSPR Transmitter for ESP32 ===");
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason_to_str(reset_rsn));

    ESP_ERROR_CHECK(config_init());
    ESP_ERROR_CHECK(config_load(&g_cfg));

    ESP_LOGI(TAG, "Callsign: %s  Locator: %s  Power: %d dBm", g_cfg.callsign, g_cfg.locator, g_cfg.power_dbm);

    ESP_ERROR_CHECK(gpio_filter_init());

    ESP_ERROR_CHECK(oscillator_init());
    oscillator_enable(false);

    oscillator_set_cal(g_cfg.xtal_cal_ppb);

    ESP_ERROR_CHECK(wifi_manager_start(g_cfg.wifi_ssid, g_cfg.wifi_pass));

    ESP_ERROR_CHECK(time_sync_init(g_cfg.ntp_server));
    ESP_LOGI(TAG, "Time source: %s", time_sync_source() == TIME_SYNC_GPS ? "GPS (auto-detected)" : "NTP (fallback)");

    ESP_ERROR_CHECK(web_server_start(&g_cfg));

    ESP_LOGI(TAG, "Web config at http://%s", wifi_manager_ip());

    web_server_set_hw_status(oscillator_hw_ok(), oscillator_hw_name());

    web_server_set_reboot_info(NULL, reset_reason_to_str(reset_rsn));

    xTaskCreate(status_task, "wspr_status", 6144, NULL, 3, NULL);

    // Pin the scheduler task to APP_CPU (core 1).
    // The ESP32 Wi-Fi stack runs primarily on PRO_CPU (core 0) and causes
    // interrupt-driven preemptions of up to 7000 us on that core.
    // Pinning to APP_CPU isolates the timing-critical WSPR symbol loop from
    // Wi-Fi jitter, reducing per-symbol timing error from ~7 ms to < 100 us.
    xTaskCreatePinnedToCore(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL, 1); // APP_CPU = core 1

    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
