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
#include <stdlib.h> // MODIFIED (Bug 6): setenv() / tzset() for GPS UTC timezone init
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

// Exact WSPR tone spacing: 12000/8192 Hz = 375/256 Hz = 1464.84375 Hz.
// Expressed as integer numerator/denominator in milli-Hz * 256 to avoid float.
// symbol * 375000 / 256 gives mHz offset; max = 3*375000/256 = 4394 mHz.
// Only 32-bit multiply and divide needed; no 64-bit ops, no float.
#define WSPR_TONE_NUM 375000UL // tone numerator: spacing_mHz * 256
#define WSPR_TONE_DEN 256UL    // tone denominator

// [FIXED] Exact WSPR symbol period timing using 3x scaling trick.
// The exact symbol period is 8192/12000 s = 682666.666... us (repeating).
// Rounding to 682667 introduced +0.333 us per symbol = +54 us accumulated drift
// over 162 symbols, causing late symbol boundaries (monotonic positive bias).
// Fix: multiply both target and elapsed by 3:
//   3 x (8192/12000 x 1e6) = 2048000.000 us exactly (no rounding error).
// Overflow proof: 162 x 2048000 = 331,776,000 < 2^32 (safe in uint32_t).
//                 max_elapsed x 3 = 110,600,000 x 3 = 331,800,000 < 2^32 (safe).
#define WSPR_PERIOD_3X_US 2048000UL // 3 x exact symbol period in microseconds

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

// [MODIFIED - BUG 4 FIX] Pre-arm state shared between scheduler_task and wspr_transmit().
// Set to true by the scheduler when phase==0 pre-arming is completed successfully.
// Read by wspr_transmit() to skip oscillator_set_freq(), gpio_filter_select(), and
// vTaskDelay() -- all of which were executed during phase==0 already, before phase==1.
// Written and read only from scheduler_task (which calls wspr_transmit() directly),
// so no mutex is needed.
static bool g_pre_armed = false;

// [MODIFIED - BUG 4 FIX] Pre-armed base frequency, set by the scheduler during phase==0.
// wspr_transmit() uses this value directly when g_pre_armed is true, avoiding a
// redundant oscillator_set_freq() call that would take ~2 ms and reset the PLL.
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

// force_rebuild parameter to select_next_band().
// O(BAND_COUNT=12) loop that clears and repopulates g_hop_active_bands[]. It only needs to run when:
//   (a) the band list has actually changed (need_rebuild=true from bands_changed), or
//   (b) the active-band array has never been populated (g_hop_active_count==0).
// Passing force_rebuild=true from the scheduler when need_rebuild is set
// ensures correctness while avoiding the unnecessary loop on every hop cycle.
static void select_next_band(bool force_next, bool force_rebuild) {
    // Conditionally rebuild: only when explicitly requested or on first call.
    if (force_rebuild || g_hop_active_count == 0)
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
    // snap_region added to the locked snapshot to eliminate the C11 data race
    // on g_cfg.iaru_region. Previously it was read unlocked on the base_hz line,
    // which is undefined behaviour when the HTTP task writes it concurrently.
    // status_task already read it under the lock; wspr_transmit() now does the same.
    uint8_t snap_region;
    web_server_cfg_lock();
    memcpy(snap_callsign, g_cfg.callsign, CALLSIGN_LEN);
    memcpy(snap_locator, g_cfg.locator, LOCATOR_LEN);
    snap_power = g_cfg.power_dbm;
    snap_parity = g_cfg.tx_slot_parity;
    snap_region = g_cfg.iaru_region; // snapshot iaru_region under the mutex
    web_server_cfg_unlock();

    // Guard against empty callsign or locator before encoding.
    // wspr_encode_type() returns WSPR_MSG_TYPE_1 for NULL inputs which misleads
    // the caller into calling wspr_encode() -- that call would then fail with
    // enc_result=-1 and produce a confusing "encode failed" error message.
    // Catching empty strings here gives a clear log and exits cleanly without
    // wasting the encode cycle or the TX time slot.
    if (snap_callsign[0] == '\0' || snap_locator[0] == '\0') {
        ESP_LOGE(TAG, "TX skipped: callsign or locator is empty");
        g_pre_armed = false; // [MODIFIED - BUG 4 FIX] clear pre-arm on early exit
        return;
    }

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
        g_pre_armed = false; // [MODIFIED - BUG 4 FIX] clear pre-arm on encode failure
        return;
    }

    // [MODIFIED - BUG 4 FIX] Compute base_hz from the pre-arm snapshot if available,
    // otherwise compute it fresh. The base_hz must be consistent between the
    // pre-arm oscillator_set_freq() call and the per-symbol oscillator_set_freq_mhz() calls.
    // When pre-armed: g_pre_arm_base_hz was set by the scheduler during phase==0
    // using the same snap_region / g_band_idx values visible here, so we reuse it.
    // When not pre-armed: compute from snap_region as before.
    uint32_t base_hz;
    if (g_pre_armed) {
        // Reuse the frequency programmed into the oscillator during phase==0.
        base_hz = g_pre_arm_base_hz;
    } else {
        // Not pre-armed: compute and program the oscillator now (original path).
        // Use snap_region (captured under mutex above) instead of the
        // unlocked g_cfg.iaru_region read that was here previously.
        // base_hz computed before the filter switch; oscillator_set_freq()
        // pre-programs the correct band frequency with RF output still off so
        // the relay never routes a stale previous-band signal through the new
        // filter passband.  vTaskDelay after gpio_filter_select() lets
        // relay contacts settle before the oscillator output is enabled.
        base_hz = config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx) + 1500u;
        oscillator_set_freq(base_hz);
        gpio_filter_select(BAND_FILTER[g_band_idx]);
        // WSPR_LPF_SETTLE_MS: settle time driven by Kconfig
        //  instead of the previous hard-coded 10 ms constant.
        //  Mechanical relays typically need 5-20 ms; solid-state relays 1-5 ms.
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
    }
    // [MODIFIED - BUG 4 FIX] Clear pre-arm flag now that it has been consumed.
    // Must be cleared before the TX loop so a subsequent call (after TX completes)
    // does not inadvertently reuse a stale pre-arm state from the previous cycle.
    g_pre_armed = false;

    oscillator_enable(true);
    g_tx_active = true;

    // [Use explicit 64-bit mask when narrowing esp_timer_get_time() to uint32_t.
    // This documents the intentional truncation and mirrors the pattern already used in
    // the scheduler for the hop timestamp. uint32_t subtraction (current - start) is
    // correct by C11 §6.2.5p9 (unsigned wrap is defined) for any TX window < 4294 s.
    // The WSPR TX window is 110 s, well within that bound; no timing corruption possible.
    uint32_t tx_start_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);

    // log stack high-watermark at TX start to help diagnose
    // stack overflows during the ~110 s transmission window.  The scheduler_task
    // stack is 8 KiB; this value confirms how much headroom remains at TX entry.
    // Also log message type and slot parity for Type-2/3 diagnosis.
    // Use snap_region for the log line as well (was g_cfg.iaru_region).
    // [MODIFIED - BUG 4 FIX] Also log whether pre-arm was used for diagnostics.
    ESP_LOGI(TAG, "TX start: band=%s freq=%lu+1500 Hz type=%d parity=%d pre_armed=%d stack HWM=%u bytes", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx), (int)msg_type, (int)snap_parity,
             (int)(g_pre_arm_base_hz != 0), // log whether pre-arm path was taken
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

        // Round tone offset to nearest mHz instead of truncating.
        // Exact WSPR tone offsets (mHz): s=0->0, s=1->1464.844, s=2->2929.688, s=3->4394.531.
        // Truncation (old): 0, 1464, 2929, 4394 (max error -0.844 mHz).
        // Rounding (new):   0, 1465, 2930, 4395 -- correct nearest-integer values.
        // Adding WSPR_TONE_DEN/2 before dividing implements round-half-up.
        int32_t tone_offset_millihz = (int32_t)(((uint32_t)symbols[i] * WSPR_TONE_NUM + (WSPR_TONE_DEN / 2UL)) / WSPR_TONE_DEN);

        oscillator_set_freq_mhz(base_hz, tone_offset_millihz);

        // [FIXED] Use exact 3x-scaled rational timing to eliminate 54 us accumulated drift.
        // Old code: target_us = (i+1) * 682667 -- +0.333 us/symbol error, +54 us total.
        // New code: target_x3 = (i+1) * 2048000 -- 3 x 682666.666... = 2048000 exactly.
        // Both target and elapsed are multiplied by 3 so the comparison is dimensionally
        // consistent. The factor of 3 cancels in the inequality; no loss of precision.
        uint32_t target_x3 = (uint32_t)(i + 1) * WSPR_PERIOD_3X_US;
        for (;;) {
            // Explicit mask matches tx_start_us; see comment there.
            uint32_t elapsed = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL) - tx_start_us;
            // Multiply elapsed by 3 to match the 3x-scaled target.
            // max elapsed*3 = 110,600,000*3 = 331,800,000 < 2^32: no overflow.
            if (elapsed * 3UL >= target_x3)
                break;
            uint32_t rem_x3 = target_x3 - elapsed * 3UL;
            // Convert 3x remainder back to microseconds (round down = conservative,
            // prevents sleeping past the deadline).
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

        // WSPR_SYMBOL_OVERRUN_LOG: warn when the actual elapsed time
        // at symbol boundary exceeds the ideal deadline by more than 10 ms.
        // Overruns indicate scheduler jitter or ISR load that may cause audible
        // frequency discontinuities; useful during initial hardware calibration.
        // [FIXED] Compute the equivalent plain-us deadline for the overrun log only:
        //   ideal_us = target_x3 / 3 = (i+1) * 682666 (truncated, conservative).
        // The overrun check uses this reconstructed value; it is only for logging
        // and does not affect the timing loop above.
#if CONFIG_WSPR_SYMBOL_OVERRUN_LOG
        {
            // Explicit mask for consistency with tx_start_us narrowing.
            uint32_t actual_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL) - tx_start_us;
            // Reconstruct plain-us deadline for log message (truncated / 3).
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
    g_symbol_idx = 0;
    ESP_LOGI(TAG, "TX complete");
}

// status_task: iaru_region read under cfg mutex to avoid data race.
// g_cfg.iaru_region is a uint8_t shared with the HTTP task writing via
// h_post_config.  Although a single-byte read is likely atomic on Xtensa LX6,
// reading it outside the lock is technically undefined behaviour in C11.
// Snapshot both region and tx_enabled under a single lock acquisition to keep
// the critical section short.
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

        // Snapshot iaru_region and tx_enabled under the cfg mutex.
        // Reading g_cfg.iaru_region without the lock is a C11 data race because
        // h_post_config writes it from the HTTP task concurrently.
        web_server_cfg_lock();
        uint8_t region_snap = g_cfg.iaru_region;
        bool tx_en = g_cfg.tx_enabled;
        web_server_cfg_unlock();

        if (g_band_idx >= 0) {
            uint32_t freq_hz = config_band_freq_hz((iaru_region_t)region_snap, (wspr_band_t)g_band_idx) + 1500u;
            uint32_t mhz_int = freq_hz / 1000000u;
            uint32_t khz_frac = (freq_hz % 1000000u) / 100u;
            snprintf(freq_str, sizeof(freq_str), "%u.%04u MHz", (unsigned)mhz_int, (unsigned)khz_frac);
        }

        int32_t next_tx = time_ok ? time_sync_secs_to_next_tx() : -1;

        web_server_update_status(time_ok, time_str, band_name, freq_str, next_tx, g_tx_active, tx_en, g_symbol_idx);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void scheduler_task(void *arg) {
    ESP_LOGI(TAG, "Scheduler started -- waiting for time sync");

    // WSPR_TASK_WDT_ENABLE: subscribe this task to the ESP-IDF
    // task watchdog so a hard-hang is detected and triggers a reboot.
    // esp_task_wdt_reset() is called every symbol (~683 ms) during TX;
    // during idle the scheduler loops on vTaskDelay(1000) which keeps
    // the inter-reset interval well under any sane WDT timeout.
// Check the return value of esp_task_wdt_add().
// Previously the return value was silently discarded. If CONFIG_ESP_TASK_WDT
// is not enabled in sdkconfig, esp_task_wdt_add() returns ESP_ERR_INVALID_STATE
// and the watchdog is NOT active -- the old code gave no indication of this.
// Now a LOGW is emitted so the developer knows WDT registration failed.
#if CONFIG_WSPR_TASK_WDT_ENABLE
    {
        esp_err_t wdt_err = esp_task_wdt_add(NULL);
        if (wdt_err != ESP_OK)
            ESP_LOGW(TAG, "WDT add failed (%s) -- watchdog not active for scheduler task", esp_err_to_name(wdt_err));
    }
#endif

    // Pass force_rebuild=true on the initial call: band list is empty at startup.
    select_next_band(false, true);

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
            ESP_LOGI(TAG, "Time synced -- entering TX schedule loop");
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

            // [MODIFIED - BUG 4 FIX] Fine-grained phase-check loop with phase==0 pre-arm.
            // Original: waited for phase in [1,3], then wspr_transmit() spent ~12 ms
            // on oscillator_set_freq() + gpio_filter_select() + vTaskDelay(settle) before
            // enabling RF output. At worst case (phase=3 when loop exits), the first symbol
            // fired at phase ~3.013, missing the protocol deadline by over 2 seconds.
            //
            // Fix: at phase==0 (the second immediately before the TX window), pre-arm the
            // oscillator and filter relay with RF output muted. The relay settle time
            // (CONFIG_WSPR_LPF_SETTLE_MS, default 10 ms) elapses entirely within the
            // ~980 ms remaining in phase==0. At phase==1 we enter wspr_transmit() which
            // skips setup and calls oscillator_enable(true) immediately, reducing the
            // first-symbol latency to a single I2C write (~0.25 ms) instead of ~12 ms.
            //
            // Pre-arm state is communicated via module-level g_pre_armed and
            // g_pre_arm_base_hz so wspr_transmit()'s signature is unchanged.
            {
                // [MODIFIED - BUG 4 FIX] Capture snap_region once for pre-arm freq computation.
                // This mirrors the pattern in wspr_transmit(); both must agree on the region.
                web_server_cfg_lock();
                uint8_t pre_snap_region = g_cfg.iaru_region;
                web_server_cfg_unlock();

                // [MODIFIED - BUG 4 FIX] pre_armed tracks whether phase==0 arm occurred.
                bool pre_armed_local = false;

                for (;;) {
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    uint32_t phase = (uint32_t)(tv.tv_sec % 120u);

                    // [MODIFIED - BUG 4 FIX] At phase==0: program oscillator and select
                    // filter relay with RF output still muted. This moves the ~2 ms
                    // oscillator setup and ~10 ms relay settle entirely out of the TX
                    // window into the preceding second, so phase==1 entry is instant.
                    // g_band_idx is read without a lock (volatile int); it is written only
                    // by select_next_band() which runs in this same task, so the read is safe.
                    if (phase == 0u && !pre_armed_local) {
                        // Compute the base frequency for the upcoming transmission.
                        // +1500 Hz is the WSPR audio centre offset (same as in wspr_transmit).
                        g_pre_arm_base_hz = config_band_freq_hz(
                            (iaru_region_t)pre_snap_region,
                            (wspr_band_t)g_band_idx) + 1500u;
                        // Program the oscillator (RF output remains muted: oscillator_enable
                        // has not been called with true). For Si5351 this runs si_cache_band()
                        // (~8 I2C writes, ~2 ms). For AD9850 this writes the frequency word.
                        oscillator_set_freq(g_pre_arm_base_hz);
                        // Select the correct LPF relay. Relay contacts settle within
                        // CONFIG_WSPR_LPF_SETTLE_MS ms, which must complete before phase==1.
                        // At 10 ms settle (default) the arm happens any time during the
                        // ~980 ms remaining in phase==0, leaving >970 ms for settling.
                        gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                        // No vTaskDelay here: relay settles during the remainder of phase==0.
                        // wspr_transmit() will skip oscillator_set_freq(), gpio_filter_select(),
                        // and vTaskDelay() because g_pre_armed will be true.
                        g_pre_armed = true;
                        pre_armed_local = true;
                        ESP_LOGD(TAG, "Pre-armed: band=%s base_hz=%lu (phase=0)",
                                 BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                    }

                    // TX starts at phase==1 (original protocol deadline).
                    // The original code allowed phase up to 3; we tighten to exactly 1
                    // because the pre-arm means setup is already done. Accepting phase==2
                    // or 3 as fallback (e.g. if phase==1 was missed due to task latency)
                    // is retained for robustness: if the scheduler was delayed past phase==1
                    // we still transmit rather than silently drop the slot.
                    // Matches the original window in time_sync_secs_to_next_tx().
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
                        // [MODIFIED - BUG 4 FIX] Clear pre-arm state if TX is disabled
                        // mid-wait. The oscillator was already programmed; leaving
                        // g_pre_armed true would cause wspr_transmit() to skip setup on
                        // a future slot with potentially a different band/frequency.
                        g_pre_armed = false;
                        g_pre_arm_base_hz = 0;
                        do_skip = true;
                        break;
                    }
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
                // Pass need_rebuild as force_rebuild so the O(BAND_COUNT)
                // list rebuild only runs when the band configuration actually changed
                // or the band index is uninitialised (g_band_idx < 0). On hop-only
                // cycles the cached list is reused and only the pointer is advanced.
                select_next_band(do_hop, need_rebuild || g_band_idx < 0);
                last_hop_ts = now;
                // [MODIFIED - BUG 4 FIX] If the band changed after the pre-arm (unlikely
                // but possible if hop fires exactly at phase==0..1 boundary), invalidate
                // the pre-arm so wspr_transmit() uses the freshly selected band instead
                // of the one programmed during phase==0.
                if (g_pre_armed) {
                    web_server_cfg_lock();
                    uint8_t new_region = g_cfg.iaru_region;
                    web_server_cfg_unlock();
                    uint32_t new_base_hz = config_band_freq_hz(
                        (iaru_region_t)new_region,
                        (wspr_band_t)g_band_idx) + 1500u;
                    if (new_base_hz != g_pre_arm_base_hz) {
                        // Band changed after pre-arm: re-arm immediately for new band.
                        g_pre_arm_base_hz = new_base_hz;
                        oscillator_set_freq(g_pre_arm_base_hz);
                        gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                        // Brief settle: still in pre-TX window, so this is safe.
                        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
                        ESP_LOGD(TAG, "Re-armed after hop: band=%s base_hz=%lu",
                                 BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                    }
                }
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
                // [MODIFIED - BUG 4 FIX] Clear pre-arm state when duty-cycle skip is taken.
                // The oscillator was pre-programmed during phase==0 but TX will not happen
                // this cycle; g_pre_armed must be false before the next cycle so the next
                // slot's pre-arm is not incorrectly skipped.
                g_pre_armed = false;
                g_pre_arm_base_hz = 0;
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
    // MODIFIED (Bug 6): Force UTC timezone at the earliest possible point in app_main(),
    // before any subsystem that might call mktime() is initialised.
    // GPS NMEA sentences always carry UTC time; mktime() must interpret them as UTC.
    // Placing this here (rather than inside time_sync_init()) guarantees correctness
    // even if a future library, driver, or FreeRTOS timer callback calls mktime()
    // before time_sync_init() runs -- which would silently corrupt GPS timestamps by
    // the local UTC offset. The matching setenv/tzset inside time_sync_init() is kept
    // as a redundant safety guard (belt-and-suspenders).
#if defined(CONFIG_WSPR_TIME_GPS)
    setenv("TZ", "UTC0", 1);
    tzset();
#endif

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

    // 3. Oscillator -- now always returns ESP_OK; sets dummy mode internally if absent
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
