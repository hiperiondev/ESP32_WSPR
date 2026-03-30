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

#include <stdio.h>
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

// WSPR timing constants
// Each symbol lasts 8192/12000 s ~= 682.667 ms
// Tone spacing = 12000/8192 Hz ~= 1.4648 Hz
// Total TX time = 162 x 682.667 ms ~= 110.6 s
#define WSPR_SYMBOL_MS 683 // ms per symbol (rounded)

// Exact WSPR tone spacing: 12000/8192 Hz = 375/256 Hz = 1464.84375 Hz.
// Expressed as integer numerator/denominator in milli-Hz * 256 to avoid float.
// symbol * 375000 / 256 gives milli-Hz offset; max = 3*375000/256 = 4394 mHz.
// Only 32-bit multiply and divide needed; no 64-bit ops, no float.
#define WSPR_TONE_NUM 375000UL // tone numerator: spacing_mHz * 256
#define WSPR_TONE_DEN 256UL    // tone denominator

// Global live config
static wspr_config_t g_cfg;
static uint32_t g_boot_uptime_sec = 0;

// Hop state
// Declared volatile: written by scheduler_task and read by status_task
// which may run simultaneously on the second ESP32 core; a non-volatile
// int may be cached in a CPU register, causing status_task to observe a
// stale band index.
static volatile int g_band_idx = -1; // index into BAND_COUNT
static int g_hop_active_bands[BAND_COUNT];
static int g_hop_active_count = 0;
static int g_hop_ptr = 0;

// TX state (updated for web status)
static volatile bool g_tx_active = false;
static volatile int g_symbol_idx = 0;

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

// Helper: build list of enabled bands
static void rebuild_active_bands(void) {
    g_hop_active_count = 0;
    for (int i = 0; i < BAND_COUNT; i++) {
        if (g_cfg.band_enabled[i])
            g_hop_active_bands[g_hop_active_count++] = i;
    }
    if (g_hop_active_count == 0) {
        // Fallback: 40m
        g_hop_active_bands[0] = BAND_40M;
        g_hop_active_count = 1;
    }
    // Clamp pointer to new count; do NOT reset to 0 so hop rotation is preserved
    if (g_hop_ptr >= g_hop_active_count)
        g_hop_ptr = 0;
    ESP_LOGI(TAG, "Active bands: %d", g_hop_active_count);
    g_cfg.bands_changed = false; // clear the flag after rebuild to prevent unnecessary calls in scheduler
}

static void select_next_band(bool force_next) {
    rebuild_active_bands();

    if (g_cfg.hop_enabled && force_next) {
        g_hop_ptr = (g_hop_ptr + 1) % g_hop_active_count;
    }
    g_band_idx = g_hop_active_bands[g_hop_ptr];
    ESP_LOGI(TAG, "Selected band: %s (%lu Hz)", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx));
}

static void wspr_transmit(void) {
    uint8_t symbols[WSPR_SYMBOLS];
    int enc_result;

    // Snapshot all required config fields under a single lock to eliminate
    // the TOCTOU race: an HTTP POST /api/config could partially overwrite g_cfg between
    // the first cfg_unlock and the wspr_encode() call in the original two-lock design.
    char snap_callsign[CALLSIGN_LEN];
    char snap_locator[LOCATOR_LEN];
    uint8_t snap_power;
    uint8_t snap_parity;
    web_server_cfg_lock();
    memcpy(snap_callsign, g_cfg.callsign, CALLSIGN_LEN);
    memcpy(snap_locator, g_cfg.locator, LOCATOR_LEN);
    snap_power = g_cfg.power_dbm;
    snap_parity = g_cfg.tx_slot_parity;
    web_server_cfg_unlock();

    // Determine WSPR message type and handle Type-2/Type-3 alternation.
    // Type-1: simple callsign + 4-char locator. No alternation needed.
    // Type-2: compound callsign (PREFIX/CALL or CALL/SUFFIX). Must alternate with
    //         a companion Type-3 message on consecutive even-minute TX slots so that
    //         receiving stations can decode both the compound callsign and the locator.
    // Type-3: simple callsign + 6-char locator. Must alternate Type-1 (primary) with
    //         Type-3 (companion) to transmit the sub-square precision locator.
    // tx_slot_parity: 0 = primary slot, 1 = companion Type-3 slot.
    wspr_msg_type_t msg_type = wspr_encode_type(snap_callsign, snap_locator);

    if (msg_type == WSPR_MSG_TYPE_1) {
        // Standard Type-1: encode normally; no alternation.
        enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
    } else {
        // Type-2 or Type-1+Type-3 path: alternate primary / companion slots.
        if (snap_parity == 0) {
            // Even slot: transmit primary message.
            // For compound callsign: wspr_encode() produces Type-2 automatically.
            // For 6-char locator: wspr_encode() produces Type-1 using the first 4 chars.
            enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
        } else {
            // Odd slot: transmit companion Type-3 message.
            // wspr_encode_type3() requires exactly 6-char locator.
            // If the stored locator is only 4 chars, append "aa" (centre of square).
            char loc6[7];
            size_t loc_len = strlen(snap_locator);
            if (loc_len >= 6) {
                memcpy(loc6, snap_locator, 6);
                loc6[6] = '\0';
            } else {
                // 4-char locator: pad with "aa" subsquare (centre of grid square).
                snprintf(loc6, sizeof(loc6), "%.4saa", snap_locator);
            }
            enc_result = wspr_encode_type3(snap_callsign, loc6, snap_power, symbols);
        }
        // Advance parity after every attempted alternating TX.
        // This ensures the companion slot fires on the very next scheduler cycle
        // regardless of whether the current encode succeeded or the TX was skipped.
        web_server_cfg_lock();
        g_cfg.tx_slot_parity ^= 1u;
        web_server_cfg_unlock();
    }

    if (enc_result < 0) {
        // Updated error message: locator now accepts 4 or 6 chars;
        // callsign now accepts compound form (PREFIX/CALL or CALL/SUFFIX).
        ESP_LOGE(TAG,
                 "WSPR encode failed: cs='%s' loc='%s' -- "
                 "simple callsign: 1-6 chars with digit at pos 2 (e.g. LU3VEA, W1AW, G4JNT); "
                 "compound callsign: PREFIX/CALL or CALL/SUFFIX (e.g. PJ4/K1ABC, K1ABC/P); "
                 "locator: 4 chars (AA00, e.g. GF05) or 6 chars (AA00AA, e.g. GF05aa)",
                 snap_callsign, snap_locator);
        return;
    }

    // base_hz computed before the filter switch; oscillator_set_freq()
    // pre-programs the correct band frequency with RF output still off so
    // the relay never routes a stale previous-band signal through the new
    // filter passband.  vTaskDelay after gpio_filter_select() lets
    // relay contacts settle before the oscillator output is enabled.
    uint32_t base_hz = config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx) + 1500u;
    oscillator_set_freq(base_hz);
    gpio_filter_select(BAND_FILTER[g_band_idx]);
    // WSPR_LPF_SETTLE_MS: settle time driven by Kconfig
    //  instead of the previous hard-coded 10 ms constant.
    //  Mechanical relays typically need 5-20 ms; solid-state relays 1-5 ms.
    vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));

    oscillator_enable(true);
    g_tx_active = true;

    uint32_t tx_start_us = (uint32_t)esp_timer_get_time();

    // log stack high-watermark at TX start to help diagnose
    // stack overflows during the ~110 s transmission window.  The scheduler_task
    // stack is 8 KiB; this value confirms how much headroom remains at TX entry.
    // Also log message type and slot parity for Type-2/3 diagnosis.
    ESP_LOGI(TAG, "TX start: band=%s freq=%lu+1500 Hz type=%d parity=%d stack HWM=%u bytes", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx), (int)msg_type, (int)snap_parity,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

        // WSPR_TASK_WDT_ENABLE: feed the task watchdog at every symbol
        // boundary (~683 ms interval) to prove the scheduler task is alive.
        // Without this the WDT fires during the ~110 s TX window if the timeout
        // is shorter than the full transmission duration.
#if CONFIG_WSPR_TASK_WDT_ENABLE
        esp_task_wdt_reset();
#endif

        // Exact tone offset: symbol * 375000 / 256 milli-Hz.
        int32_t tone_offset_millihz = (int32_t)((uint32_t)symbols[i] * WSPR_TONE_NUM / WSPR_TONE_DEN);
        oscillator_set_freq_mhz(base_hz, tone_offset_millihz);

        uint32_t target_us = (uint32_t)(i + 1) * 682667UL;
        for (;;) {
            uint32_t elapsed = (uint32_t)esp_timer_get_time() - tx_start_us;
            if (elapsed >= target_us)
                break;
            uint32_t rem = target_us - elapsed;
            if (rem > 10000u) {
                uint32_t sleep_ms = rem / 1000u;
                if (sleep_ms > 5u)
                    sleep_ms -= 5u;
                else
                    sleep_ms = 1u;
                vTaskDelay(pdMS_TO_TICKS(sleep_ms));
            } else if (rem > 1000u) {
                vTaskDelay(1);
            } else {
                taskYIELD();
            }
        }

        // WSPR_SYMBOL_OVERRUN_LOG: warn when the actual elapsed time
        // at symbol boundary exceeds the ideal deadline by more than 10 ms.
        // Overruns indicate scheduler jitter or ISR load that may cause audible
        // frequency discontinuities; useful during initial hardware calibration.
#if CONFIG_WSPR_SYMBOL_OVERRUN_LOG
        {
            uint32_t actual_us = (uint32_t)esp_timer_get_time() - tx_start_us;
            if (actual_us > target_us + 10000u) {
                ESP_LOGW(TAG, "Symbol %d overrun: deadline=%lu actual=%lu overrun=%lu us", i, (unsigned long)target_us, (unsigned long)actual_us,
                         (unsigned long)(actual_us - target_us));
            }
        }
#endif
    }

    oscillator_enable(false);
    g_tx_active = false;
    g_symbol_idx = 0;
    ESP_LOGI(TAG, "TX complete");
}

static void status_task(void *arg) {
    char time_str[24] = "---";
    char freq_str[24] = "---";
    // flag ensures boot time is computed and pushed exactly once
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
                // Pass only the time string; reason was already stored in app_main
                web_server_set_reboot_info(boot_str, NULL);
                reboot_info_time_set = true;
            }
        }

        const char *band_name = (g_band_idx >= 0) ? BAND_NAME[g_band_idx] : "---";

        if (g_band_idx >= 0) {
            uint32_t freq_hz = config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx) + 1500u;
            uint32_t mhz_int = freq_hz / 1000000u;
            uint32_t khz_frac = (freq_hz % 1000000u) / 100u;
            snprintf(freq_str, sizeof(freq_str), "%u.%04u MHz", (unsigned)mhz_int, (unsigned)khz_frac);
        }

        int32_t next_tx = time_ok ? time_sync_secs_to_next_tx() : -1;

        // Take cfg lock only for the tx_enabled scalar read; all other status
        // fields are volatile or read-only and do not require the lock.
        web_server_cfg_lock();
        bool tx_en = g_cfg.tx_enabled;
        web_server_cfg_unlock();

        web_server_update_status(time_ok, time_str, band_name, freq_str, next_tx, g_tx_active, tx_en, g_symbol_idx);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void scheduler_task(void *arg) {
    ESP_LOGI(TAG, "Scheduler started — waiting for time sync");

    // WSPR_TASK_WDT_ENABLE: subscribe this task to the ESP-IDF
    // task watchdog so a hard-hang is detected and triggers a reboot.
    // esp_task_wdt_reset() is called every symbol (~683 ms) during TX;
    // during idle the scheduler loops on vTaskDelay(1000) which keeps
    // the inter-reset interval well under any sane WDT timeout.
#if CONFIG_WSPR_TASK_WDT_ENABLE
    esp_task_wdt_add(NULL);
#endif

    select_next_band(false);

    uint32_t last_hop_ts = 0;
    // initialize to 100 so the very first eligible TX slot fires
    // immediately. With 0 the Bresenham accumulator requires one full duty cycle
    // of slots to expire before the first TX, which delays the first transmission
    // and produces unexpected behavior after a duty-cycle config change.
    uint16_t s_duty_accum = 100u;
    // Track whether we have already logged the "time synced" banner so it
    // only appears once rather than on every loop iteration after sync.
    bool time_synced_logged = false;

    while (1) {
        // Non-blocking time sync check: yield for 1 s and retry instead of
        // blocking forever with time_sync_wait(0).  This keeps the scheduler
        // task alive so the web UI, status task and config changes remain
        // responsive even when NTP is unavailable (AP-only mode, no internet).
        if (!time_sync_is_ready()) {
            // Feed the task watchdog on every iteration while
            // waiting for NTP/GPS to sync.  Without this, a long startup
            // period in AP-only mode (no internet) fires the WDT.
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!time_synced_logged) {
            ESP_LOGI(TAG, "Time synced — entering TX schedule loop");
            time_synced_logged = true;
        }

        web_server_cfg_lock();
        bool tx_enabled_snap = g_cfg.tx_enabled;
        web_server_cfg_unlock();

        if (!tx_enabled_snap) {
            // feed the task watchdog while TX is disabled so an
            // arbitrarily long TX-off period does not trigger a WDT timeout.
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool do_skip = false;

        // Wait for next even-minute + 1 s mark
        int32_t wait_sec = time_sync_secs_to_next_tx();
        if (wait_sec > 0) {
            ESP_LOGI(TAG, "Next TX in %d s (band=%s)", wait_sec, BAND_NAME[g_band_idx]);
            // chunked loop that feeds the WDT every 1 s instead.
            if (wait_sec > 3) {
                uint32_t sleep_ms = (uint32_t)(wait_sec - 2) * 1000u;
                while (sleep_ms > 0u) {
                    uint32_t chunk = (sleep_ms > 1000u) ? 1000u : sleep_ms;
                    vTaskDelay(pdMS_TO_TICKS(chunk));
                    // reset once per 1 s chunk during coarse pre-TX sleep
#if CONFIG_WSPR_TASK_WDT_ENABLE
                    esp_task_wdt_reset();
#endif
                    sleep_ms -= chunk;
                }
            }

            for (;;) {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                uint32_t phase = (uint32_t)(tv.tv_sec % 120u);
                // TX starts at phase=3 at the latest: 3 + 110.6 s = 113.6 s,
                // leaving 6.4 s before the 120 s slot boundary. This prevents
                // the last symbols from overflowing into the next decode window.
                // Matches the window in time_sync_secs_to_next_tx().
                if (phase >= 1u && phase <= 3u)
                    break;

                vTaskDelay(pdMS_TO_TICKS(10));
                // feed WDT inside fine-grained phase-check loop;
                // this loop runs for up to ~3 s in 10 ms steps.
#if CONFIG_WSPR_TASK_WDT_ENABLE
                esp_task_wdt_reset();
#endif
                web_server_cfg_lock();
                bool still_en = g_cfg.tx_enabled;
                web_server_cfg_unlock();

                if (!still_en) {
                    do_skip = true;
                    break;
                }
            }
        }

        if (!do_skip) {

            // Frequency hopping decision
            struct timeval tv;
            gettimeofday(&tv, NULL);
            // explicit mask makes the 64-bit time_t -> uint32_t
            // truncation visible in source and suppresses compiler warnings on
            // platforms where time_t is wider than 32 bits.
            uint32_t now = (uint32_t)(tv.tv_sec & 0xFFFFFFFFUL);

            web_server_cfg_lock();
            bool hop_en = g_cfg.hop_enabled;
            uint32_t hop_intv = g_cfg.hop_interval_sec;
            bool need_rebuild = g_cfg.bands_changed;
            g_cfg.bands_changed = false; // clear flag under lock so rebuild happens only on actual config change
            web_server_cfg_unlock();

            if (hop_intv == 0u)
                hop_intv = 120u;

            bool do_hop = hop_en && (now - last_hop_ts) >= hop_intv;

            if (do_hop || g_band_idx < 0 || need_rebuild) {
                select_next_band(do_hop);
                last_hop_ts = now;
            }

            // Duty cycle control: Bresenham-style accumulator avoids the integer
            // division rounding bug where 100/duty==1 for duty=51..99.
            web_server_cfg_lock();
            uint8_t duty = g_cfg.tx_duty_pct;
            web_server_cfg_unlock();

            bool do_tx = false;
            if (duty > 0u) {
                s_duty_accum += (uint16_t)duty;
                if (s_duty_accum >= 100u) {
                    s_duty_accum -= 100u;
                    do_tx = true;
                }
            }
            if (!do_tx) {
                ESP_LOGI(TAG, "Duty cycle skip (pct=%u accum=%u)", (unsigned)duty, (unsigned)s_duty_accum);
                do_skip = true;
            }

        } // close if (!do_skip)

        if (do_skip) {
            // feed WDT before skip delay so that duty-cycle skips
            // and TX-disabled paths do not accumulate toward a WDT timeout.
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Transmit
        wspr_transmit();
    }
}

void app_main(void) {
    // esp_log_timestamp() returns uint32_t ms since boot (no __divdi3 call needed).
    // Dividing uint32_t ms by 1000u uses 32-bit integer division only.
    g_boot_uptime_sec = (uint32_t)(esp_log_timestamp() / 1000u);

    // read reset reason before NVS or WiFi can trigger a secondary reset.
    esp_reset_reason_t reset_rsn = esp_reset_reason();

    ESP_LOGI(TAG, "=== WSPR Transmitter for ESP32 ===");
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason_to_str(reset_rsn));

    // 1. NVS + Config
    ESP_ERROR_CHECK(config_init());
    ESP_ERROR_CHECK(config_load(&g_cfg));

    ESP_LOGI(TAG, "Callsign: %s  Locator: %s  Power: %d dBm", g_cfg.callsign, g_cfg.locator, g_cfg.power_dbm);

    // 2. GPIO filter bank
    ESP_ERROR_CHECK(gpio_filter_init());

    // 3. Oscillator — now always returns ESP_OK; sets dummy mode internally if absent
    ESP_ERROR_CHECK(oscillator_init());
    oscillator_enable(false);

    // Apply crystal calibration offset from NVS before any TX.
    oscillator_set_cal(g_cfg.xtal_cal_ppb);

    // 4. WiFi (STA or AP fallback)
    ESP_ERROR_CHECK(wifi_manager_start(g_cfg.wifi_ssid, g_cfg.wifi_pass));

    // 5. Time synchronization
    ESP_ERROR_CHECK(time_sync_init(g_cfg.ntp_server));

    // 6. Web server
    // web_server_start() creates the _cfg_mutex internally before returning.
    // scheduler_task and status_task must only be started after this call so
    // that web_server_cfg_lock/unlock are safe to call from those tasks.
    ESP_ERROR_CHECK(web_server_start(&g_cfg));

    ESP_LOGI(TAG, "Web config at http://%s", wifi_manager_ip());

    // Push oscillator hardware detection result to the web status API.
    web_server_set_hw_status(oscillator_hw_ok(), oscillator_hw_name());

    // push reset reason into the status cache now that the mutex exists.
    // The boot time string is left empty here; status_task fills it in once
    // NTP/GPS provides a valid wall-clock reference.
    web_server_set_reboot_info(NULL, reset_reason_to_str(reset_rsn));

    // 7. Status task (low priority, 1s tick)
    xTaskCreate(status_task, "wspr_status", 6144, NULL, 3, NULL);

    // 8. TX Scheduler task (higher priority)
    xTaskCreate(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL);

    // Main task idles
    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
