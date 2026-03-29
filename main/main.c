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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "gpio_filter.h"
#include "oscillator.h"
#include "time_sync.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "wspr_encode.h"

static const char *TAG = "wspr_main";

// ── WSPR timing constants ───────────────────────────────────────────────────
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

// ── Global live config ──────────────────────────────────────────────────────
static wspr_config_t g_cfg;

// ── Hop state ───────────────────────────────────────────────────────────────
// Declared volatile: written by scheduler_task and read by status_task
// which may run simultaneously on the second ESP32 core; a non-volatile
// int may be cached in a CPU register, causing status_task to observe a
// stale band index.
static volatile int g_band_idx = -1; // index into BAND_COUNT
static int g_hop_active_bands[BAND_COUNT];
static int g_hop_active_count = 0;
static int g_hop_ptr = 0;

// ── TX state (updated for web status) ──────────────────────────────────────
static volatile bool g_tx_active = false;
static volatile int g_symbol_idx = 0;

// ── Helper: build list of enabled bands ─────────────────────────────────────
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

    if (wspr_encode(g_cfg.callsign, g_cfg.locator, g_cfg.power_dbm, symbols) < 0) {
        ESP_LOGE(TAG, "WSPR encode failed — check callsign/locator");
        return;
    }

    // base_hz computed before the filter switch; oscillator_set_freq()
    // pre-programs the correct band frequency with RF output still off so
    // the relay never routes a stale previous-band signal through the new
    // filter passband.  vTaskDelay(10 ms) after gpio_filter_select() lets
    // relay contacts settle before the oscillator output is enabled.
    // MODIFIED: use config_band_freq_hz() with iaru_region from g_cfg
    uint32_t base_hz = config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx) + 1500u;
    oscillator_set_freq(base_hz);
    gpio_filter_select(BAND_FILTER[g_band_idx]);
    vTaskDelay(pdMS_TO_TICKS(10));

    oscillator_enable(true);
    g_tx_active = true;

    // Symbol timing uses uint32_t to avoid 64-bit software multiply on LX6.
    // Rationale (doc §3.9 / §3.10):
    //   - esp_timer_get_time() returns int64_t; truncating to uint32_t keeps
    //     only the low 32 bits, which wrap every ~4294 s (~71 min).
    //   - The full WSPR transmission is 162 * 682667 us = 110,592,054 us (~110 s),
    //     well under 71 min, so no wrap occurs during a single TX burst.
    //   - Unsigned 32-bit subtraction (elapsed = now - start) correctly handles
    //     any timer roll-over at the 32-bit boundary.
    //   - (i+1) * 682667UL: max = 162 * 682667 = 110,592,054, fits uint32_t.
    //   - Replaces: const int64_t SYMBOL_US = 682667LL and int64_t arithmetic.
    uint32_t tx_start_us = (uint32_t)esp_timer_get_time();

    // MODIFIED: use config_band_freq_hz() instead of BAND_FREQ_HZ[g_band_idx]
    ESP_LOGI(TAG, "TX start: band=%s freq=%lu+1500 Hz", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)g_cfg.iaru_region, (wspr_band_t)g_band_idx));

    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

        // Exact tone offset: symbol * 375000 / 256 milli-Hz.
        // Avoids the 0.47 mHz rounding error of the old 1465 constant.
        // 32-bit multiply: max 3*375000=1125000, fits uint32_t.
        // 32-bit divide by 256 is a single right-shift on LX6.
        int32_t tone_offset_mhz = (int32_t)((uint32_t)symbols[i] * WSPR_TONE_NUM / WSPR_TONE_DEN);
        oscillator_set_freq_mhz(base_hz, tone_offset_mhz);

        uint32_t target_us = (uint32_t)(i + 1) * 682667UL;
        for (;;) {
            uint32_t elapsed = (uint32_t)esp_timer_get_time() - tx_start_us;
            if (elapsed >= target_us)
                break;
            uint32_t rem = target_us - elapsed;
            if (rem > 10000u) {
                // Coarse sleep: wake up ~5 ms before the boundary.
                // rem/1000u >= 10, so (rem/1000u - 5u) >= 5; no underflow.
                vTaskDelay(pdMS_TO_TICKS(rem / 1000u - 5u));
            } else if (rem > 1000u) {
                // Middle tier: 1-tick sleep feeds the idle task every ~1 ms.
                // Worst overrun from a late wakeup is one tick (~1 ms),
                // which is negligible against the 682 ms symbol period.
                vTaskDelay(1);
            } else {
                // Fine busy-poll: <= 1 ms remaining; a task switch would
                // overshoot. Total busy time per symbol <= 1 ms.
                taskYIELD();
            }
        }
    }

    oscillator_enable(false);
    g_tx_active = false;
    g_symbol_idx = 0;
    ESP_LOGI(TAG, "TX complete");
}

static void status_task(void *arg) {
    char time_str[24] = "---";
    char freq_str[24] = "---";

    while (1) {
        struct timeval tv;
        bool time_ok = time_sync_get(&tv);

        if (time_ok) {
            struct tm t;
            gmtime_r(&tv.tv_sec, &t);
            strftime(time_str, sizeof(time_str), "%H:%M:%S UTC", &t);
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

    select_next_band(false);

    uint32_t last_hop_ts = 0;
    uint16_t s_duty_accum = 0u;
    // Track whether we have already logged the "time synced" banner so it
    // only appears once rather than on every loop iteration after sync.
    bool time_synced_logged = false;

    while (1) {
        // Non-blocking time sync check: yield for 1 s and retry instead of
        // blocking forever with time_sync_wait(0).  This keeps the scheduler
        // task alive so the web UI, status task and config changes remain
        // responsive even when NTP is unavailable (AP-only mode, no internet).
        if (!time_sync_is_ready()) {
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
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool do_skip = false;

        // Wait for next even-minute + 1 s mark
        int32_t wait_sec = time_sync_secs_to_next_tx();
        if (wait_sec > 0) {
            ESP_LOGI(TAG, "Next TX in %d s (band=%s)", wait_sec, BAND_NAME[g_band_idx]);
            // Coarse sleep until ~2 s before TX
            if (wait_sec > 3)
                vTaskDelay(pdMS_TO_TICKS((uint32_t)(wait_sec - 2) * 1000u));

            // Fine busy-wait: tolerant window fires when tv_sec%120 is in [1..5].
            // An NTP clock step can jump from sec N to N+2, skipping the :01
            // boundary entirely.  The 5-second window catches late arrivals while
            // still leaving >115 s of the 120 s slot for the 110 s TX burst.
            // If phase > 5 the window was missed; log and skip this slot.
            for (;;) {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                uint32_t phase = (uint32_t)(tv.tv_sec % 120u);
                if (phase >= 1u && phase <= 5u)
                    break;
                if (phase > 5u) {
                    ESP_LOGW(TAG, "TX slot missed (phase=%u), waiting for next", (unsigned)phase);
                    do_skip = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                web_server_cfg_lock();
                bool still_en = g_cfg.tx_enabled;
                web_server_cfg_unlock();

                if (!still_en) {
                    do_skip = true;
                    break;
                }
            }
        }

        // Hopping and duty-cycle blocks wrapped under !do_skip guard so they
        // are bypassed when a slot miss or TX-disable was flagged above.
        if (!do_skip) {

            // Frequency hopping decision
            struct timeval tv;
            gettimeofday(&tv, NULL);
            uint32_t now = (uint32_t)tv.tv_sec;

            web_server_cfg_lock();
            bool hop_en = g_cfg.hop_enabled;
            uint32_t hop_intv = g_cfg.hop_interval_sec;
            web_server_cfg_unlock();

            // Guard against hop_intv==0 (user error or uninitialised field):
            // a zero interval makes do_hop always true, causing band changes on
            // every scheduler loop iteration.  Clamp to the minimum WSPR slot
            // length (120 s) to match the WSPR 2-minute transmission grid.
            if (hop_intv == 0u)
                hop_intv = 120u;

            bool do_hop = hop_en && (now - last_hop_ts) >= hop_intv;

            if (do_hop || g_band_idx < 0) {
                select_next_band(do_hop);
                last_hop_ts = now;
            }

            // Duty cycle control: Bresenham-style accumulator avoids the integer
            // division rounding bug where 100/duty==1 for duty=51..99, which made
            // every slot fire regardless of the configured percentage.
            // Algorithm: accumulate duty percentage each slot; when the sum reaches
            // or exceeds 100, fire TX and subtract 100.  This gives exact long-term
            // duty for any integer 0-100 without division.
            // Examples:  duty=20 -> fires slots 0,5,10,... (every 5th = 20%)
            //            duty=51 -> fires ~51 of every 100 slots (not 100%)
            //            duty=100 -> fires every slot
            //            duty=0   -> never fires
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

        } // close if (!do_skip) — wraps hopping and duty-cycle blocks above

        // Unified exit point replacing the skip_tx label; all skip conditions
        // (slot miss, TX disabled, duty cycle) converge here.
        if (do_skip) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Transmit
        wspr_transmit();
    }
}

// ── app_main ─────────────────────────────────────────────────────────────────
void app_main(void) {
    ESP_LOGI(TAG, "=== WSPR Transmitter for ESP32 ===");

    // 1. NVS + Config
    ESP_ERROR_CHECK(config_init());
    ESP_ERROR_CHECK(config_load(&g_cfg));

    ESP_LOGI(TAG, "Callsign: %s  Locator: %s  Power: %d dBm", g_cfg.callsign, g_cfg.locator, g_cfg.power_dbm);

    // 2. GPIO filter bank
    ESP_ERROR_CHECK(gpio_filter_init());

    // 3. Oscillator — now always returns ESP_OK; sets dummy mode internally if absent
    ESP_ERROR_CHECK(oscillator_init());
    oscillator_enable(false);

    // Apply crystal calibration offset from NVS before any TX. Must be called
    // after oscillator_init() (which sets up _vco_hz / GPIO) so that the ppb
    // correction is visible to the first si_set_freq_hz_mhz() / ad9850_freq_word()
    // call inside wspr_transmit(). Without this call, xtal_cal_ppb saved to NVS
    // via the web UI is loaded into g_cfg but never forwarded to the driver.
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

    // push oscillator hardware detection result to the web status API.
    // Called after web_server_start() (mutexes exist) and before tasks start
    // (single-threaded here, no race).  The result persists for the session.
    web_server_set_hw_status(oscillator_hw_ok(), oscillator_hw_name());

    // 7. Status task (low priority, 1s tick)
    xTaskCreate(status_task, "wspr_status", 6144, NULL, 3, NULL);

    // 8. TX Scheduler task (higher priority)
    xTaskCreate(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL);

    // Main task idles
    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
