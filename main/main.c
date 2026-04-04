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

// Inline helper: read esp_timer and truncate to 32 bits.
static inline uint32_t timer_us32(void) {
    return (uint32_t)esp_timer_get_time();
}

// Symbol period: 8192/12000 s = 682666.67 µs per symbol.
// Stored as 3x the period (2 048 000 µs) to allow comparison using integer
// arithmetic without accumulated floating-point error across 162 symbols.
#define WSPR_PERIOD_3X_US 2048000UL // 3 x exact symbol period in microseconds

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

// wspr_transmit: parity toggle moved AFTER successful encode to avoid
// parity desync when wspr_encode or wspr_encode_type3 returns an error.
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
        enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
    } else {
        if (snap_parity == 0) {
            enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
        } else {
            char loc6[7];
            size_t loc_len = strlen(snap_locator);
            if (loc_len >= 6) {
                memcpy(loc6, snap_locator, 6);
                loc6[6] = '\0';
            } else {
                snprintf(loc6, sizeof(loc6), "%.4saa", snap_locator);
            }
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
    // return ESP_ERR_INVALID_STATE (return value ignored by the loop), causing
    // the Si5351 to hold the last written PLL value for the remainder of the
    // WSPR frame -- all remaining symbols wrong frequency, frame undecodable.
    oscillator_tx_begin(); // protect cache from cal races BEFORE first si_apply_tone

    // apply symbol 0 tone BEFORE enabling RF output so the oscillator
    // is at the correct frequency the instant it is keyed.
    // round tone offset with +128 before divide (Bug #3).
    int32_t tone0_millihz = (int32_t)((symbols[0] * 375000UL + 128UL) / 256UL);
    oscillator_set_freq_mhz(base_hz, tone0_millihz);

    oscillator_enable(true);
    g_tx_active = true;
    uint32_t tx_start_us = timer_us32();

    // ── TX timing diagnostic ──
    {
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
    }
    // ── fin diagnostico ──

    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

        // check tone_active on every symbol (was every 10 = ~6.8 s gap).
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

        // skip redundant set_freq_mhz for i==0.
        // round tone offset: (sym*375000 + 128) / 256.
        if (i > 0) {
            int32_t tone_offset_millihz = (int32_t)((symbols[i] * 375000UL + 128UL) / 256UL);
            oscillator_set_freq_mhz(base_hz, tone_offset_millihz);
        }

        uint32_t target_x3 = (uint32_t)(i + 1) * WSPR_PERIOD_3X_US;
        for (;;) {
            uint32_t elapsed = timer_us32() - tx_start_us;

            if (elapsed * 3UL >= target_x3)
                break;
            uint32_t elapsed_x3 = elapsed * 3UL;
            uint32_t rem_x3 = (elapsed_x3 < target_x3) ? (target_x3 - elapsed_x3) : 0u;
            uint32_t rem_us = rem_x3 / 3UL;
            if (rem_us > 10000u) {
                uint32_t sleep_ms = rem_us / 1000u;
                if (sleep_ms > 5u)
                    sleep_ms -= 5u;
                else
                    sleep_ms = 1u;
                vTaskDelay(pdMS_TO_TICKS(sleep_ms));
            } else if (rem_us > 1000u) {
                vTaskDelay(1);
            } else {
                taskYIELD();
            }
        }

#if CONFIG_WSPR_SYMBOL_OVERRUN_LOG
        {
            uint32_t actual_us = timer_us32() - tx_start_us;
            uint32_t deadline_us = target_x3 / 3UL;
            if (actual_us > deadline_us + 10000u) {
                ESP_LOGW(TAG, "Symbol %d overrun: deadline=%lu actual=%lu overrun=%lu us", i, (unsigned long)deadline_us, (unsigned long)actual_us,
                         (unsigned long)(actual_us - deadline_us));
            }
        }
#endif
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
    {
        esp_err_t wdt_err = esp_task_wdt_add(NULL);
        if (wdt_err != ESP_OK)
            ESP_LOGW(TAG, "WDT add failed (%s) -- watchdog not active for scheduler task", esp_err_to_name(wdt_err));
    }
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
        {
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
                        ESP_LOGI(TAG, "Tone test activated: %.3f kHz (%lu Hz) filter=%u", (double)tone_freq_snap, (unsigned long)tone_hz,
                                 (unsigned)tone_filter);
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

        {
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
                s_last_hop_time_sec = now;
                s_hop_anchored = true;
            } else {
                do_hop = hop_en && ((now - s_last_hop_time_sec) >= hop_intv);
            }

            if (do_hop || g_band_idx < 0 || need_rebuild) {
                select_next_band(do_hop, need_rebuild || g_band_idx < 0);
                if (do_hop)
                    s_last_hop_time_sec = now;
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
        {
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
                    // Prevent 100% CPU spin during the ~1-second window
                    // between phase==0 (pre-arm) and phase==1 (TX start).
                    // Without this delay the loop iterates at maximum speed after
                    // pre_armed_local becomes true, starving other tasks of CPU time
                    // and increasing the risk of OS jitter when the critical phase==1
                    // sub-second spin begins.
                    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms sleep prevents CPU starvation
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

                    // Sub-second spin to align TX start to within
                    // 50ms of the :01 second boundary.
                    // WSPR decoders accept at most +-1s DT; starting at :01.000-:01.050
                    // gives DT=0 which is ideal. Starting at :01.050-:01.999 still
                    // gives DT<1 and is decodable but with reduced margin.
                    // Previous bug: if the spin exhausted its 950-iteration safety limit
                    // at a point where the second had already rolled to :02, the outer
                    // loop would call wspr_transmit() at phase 2 without setting
                    // do_skip=true, causing a consistently-undecodable late TX.
                    // after the spin exits, re-check the phase; if we have drifted
                    // past :01, set do_skip=true so this slot is abandoned and the
                    // scheduler waits for the next even-minute window.

                    uint32_t spins = 0;
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
                        // Safety exit: should not be reached under normal conditions
                        if (++spins > 1500u)
                            break;
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }

                    // Re-verify phase after the spin loop exits.
                    // If the second rolled past :01 (phase != 1 or usec >= 50000
                    // at entry-to-next-second), the slot is undecodable.
                    // Set do_skip=true so wspr_transmit() is NOT called.

                    struct timeval tv_verify;
                    gettimeofday(&tv_verify, NULL);
                    uint32_t verify_phase = (uint32_t)(tv_verify.tv_sec % 120u);
                    if (verify_phase != 1u) {
                        // Phase has rolled past :01 — TX would be too late
                        ESP_LOGW(TAG,
                                 "Sub-second spin exited at phase=%lu (expected 1): "
                                 "slot skipped to protect decode quality",
                                 (unsigned long)verify_phase);
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
        }

        if (do_skip) {
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            {
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

    xTaskCreate(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL);

    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
