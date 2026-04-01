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
#define WSPR_TONE_NUM     375000UL  // tone numerator: spacing_mHz * 256
#define WSPR_TONE_DEN     256UL     // tone denominator

// Symbol period: 8192/12000 s = 682666.67 µs per symbol.
// Stored as 3x the period (2 048 000 µs) to allow comparison using integer
// arithmetic without accumulated floating-point error across 162 symbols.
#define WSPR_PERIOD_3X_US 2048000UL // 3 x exact symbol period in microseconds

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

/**
 * @brief Convert an esp_reset_reason_t to a human-readable string.
 * @param r Reset reason code from esp_reset_reason().
 * @return Constant string describing the reset cause.
 */
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

/**
 * @brief Rebuild the list of enabled bands from the current configuration.
 *
 * Scans band_enabled[] and populates g_hop_active_bands[].
 * Falls back to 40 m if no band is enabled to guarantee at least one TX target.
 * Resets g_hop_ptr if it would be out of range after the rebuild.
 */
static void rebuild_active_bands(void) {
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
    // Clamp pointer to valid range after rebuild
    if (g_hop_ptr >= g_hop_active_count)
        g_hop_ptr = 0;
    ESP_LOGI(TAG, "Active bands: %d", g_hop_active_count);
    // Clear the flag so the scheduler does not call rebuild again next slot
    g_cfg.bands_changed = false; // clear the flag after rebuild to prevent unnecessary calls in scheduler
}

/**
 * @brief Select the next band according to the hop policy.
 *
 * @param force_next    Advance the round-robin pointer (hop to next band).
 * @param force_rebuild Re-scan the enabled-band list before selecting.
 */
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

/**
 * @brief Encode and transmit one complete WSPR message (162 symbols).
 *
 * WSPR (Weak Signal Propagation Reporter) uses 4-FSK modulation with
 * 1.4648 Hz tone spacing. Each of the 162 symbols represents one of four
 * frequencies separated by multiples of that spacing. A full transmission
 * lasts 162 * 8192/12000 s ≈ 110.6 seconds.
 *
 * Timing is derived from the high-resolution ESP timer. The target deadline
 * for symbol N is N * 682 666.67 µs after the first symbol. The inner loop
 * uses coarse vTaskDelay() for long waits and busy-waits (taskYIELD) for
 * the final microseconds, keeping scheduling jitter below one RTOS tick.
 *
 * For compound callsigns (Type-2) or 6-char locators (Type-3), this function
 * alternates between the primary message and the companion Type-3 message on
 * successive even-minute slots using tx_slot_parity.
 */
static void wspr_transmit(void) {
    uint8_t symbols[WSPR_SYMBOLS];
    int enc_result;
    // Snapshot all config fields needed for this TX under the mutex
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

    // Refuse to transmit if essential parameters are missing
    if (snap_callsign[0] == '\0' || snap_locator[0] == '\0') {
        ESP_LOGE(TAG, "TX skipped: callsign or locator is empty");
        g_pre_armed = false; // [MODIFIED - BUG 4 FIX] clear pre-arm on early exit
        return;
    }

    // Determine which WSPR message type is required for this callsign/locator:
    // Type-1: simple callsign + 4-char locator (most common case).
    // Type-2: compound callsign with '/' (e.g. "PJ4/K1ABC"), no locator in this frame.
    // Type-3: companion to Type-2 or Type-1 with 6-char locator; carries hashed
    //         callsign + full 6-char sub-square in alternating even-minute slots.
    wspr_msg_type_t msg_type = wspr_encode_type(snap_callsign, snap_locator);

    if (msg_type == WSPR_MSG_TYPE_1) {
        // Standard single-slot transmission: callsign + 4-char grid + power
        enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
    } else {
        // Two-slot alternation: parity=0 -> primary frame, parity=1 -> Type-3 companion
        if (snap_parity == 0) {
            enc_result = wspr_encode(snap_callsign, snap_locator, snap_power, symbols);
        } else {
            // Build a 6-char locator for the Type-3 companion frame.
            // If the stored locator is shorter than 6 chars, append "aa" (sub-square AA)
            // so decoding software can still associate the hash with the primary frame.
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

        // Toggle parity for the next slot so the frames alternate correctly
        web_server_cfg_lock();
        g_cfg.tx_slot_parity ^= 1u;
        web_server_cfg_unlock();
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

    // Determine the carrier base frequency.
    // WSPR transmitters place the audio-center tone at 1500 Hz above the dial frequency,
    // so the base_hz here is dial_freq + 1500 Hz. The four WSPR tones then sit at
    // base_hz + 0, +1.46, +2.93, +4.39 Hz (tone 0 to 3 * 1.4648 Hz spacing).
    uint32_t base_hz;
    if (g_pre_armed) {
        // Oscillator and LPF were already programmed during the pre-arm phase (phase==0)
        base_hz = g_pre_arm_base_hz;
    } else {
        base_hz = config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx) + 1500u;
        oscillator_set_freq(base_hz);
        gpio_filter_select(BAND_FILTER[g_band_idx]);
        // Wait for LPF relay contacts to settle before enabling RF output
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
    }

    g_pre_armed = false;

    // Begin TX: mark oscillator busy (defers any pending calibration changes)
    oscillator_tx_begin();
    oscillator_enable(true);
    g_tx_active = true;
    uint32_t tx_start_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);

    ESP_LOGI(TAG, "TX start: band=%s freq=%lu+1500 Hz type=%d parity=%d pre_armed=%d stack HWM=%u bytes", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx), (int)msg_type, (int)snap_parity,
             (int)(g_pre_arm_base_hz != 0), // log whether pre-arm path was taken
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    // Symbol transmission loop: send all 162 4-FSK symbols with precise timing.
    // Each symbol period is exactly 8192/12000 s = 682 666.67 µs.
    // The inner timing loop compares elapsed*3 against the 3x target to avoid
    // floating-point division while maintaining microsecond accuracy.
    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

#if CONFIG_WSPR_TASK_WDT_ENABLE
        esp_task_wdt_reset();
#endif

        // Compute tone offset in milli-Hz for this symbol value (0-3).
        // Formula: offset_mHz = symbol * 375000 / 256 = symbol * 1464.844 mHz
        // This matches WSPR spec: tone spacing = 12000/8192 Hz = 1.4648 Hz
        int32_t tone_offset_millihz = (int32_t)(((uint32_t)symbols[i] * WSPR_TONE_NUM + (WSPR_TONE_DEN / 2UL)) / WSPR_TONE_DEN);
        oscillator_set_freq_mhz(base_hz, tone_offset_millihz);

        // Wait until the deadline for the end of this symbol period.
        // Use coarse sleep for most of the wait, then busy-wait the last millisecond.
        uint32_t target_x3 = (uint32_t)(i + 1) * WSPR_PERIOD_3X_US;
        for (;;) {
            uint32_t elapsed = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL) - tx_start_us;

            if (elapsed * 3UL >= target_x3)
                break;
            uint32_t rem_x3 = target_x3 - elapsed * 3UL;
            uint32_t rem_us = rem_x3 / 3UL;
            if (rem_us > 10000u) {
                // More than 10 ms remaining: sleep most of it, keep 5 ms margin
                uint32_t sleep_ms = rem_us / 1000u;
                if (sleep_ms > 5u)
                    sleep_ms -= 5u;
                else
                    sleep_ms = 1u;
                vTaskDelay(pdMS_TO_TICKS(sleep_ms));
            } else if (rem_us > 1000u) {
                // 1-10 ms remaining: yield one RTOS tick
                vTaskDelay(1);
            } else {
                // Final microseconds: cooperative yield without sleeping
                taskYIELD();
            }
        }

#if CONFIG_WSPR_SYMBOL_OVERRUN_LOG
        // Warn if the actual symbol boundary was missed by more than 10 ms
        {
            uint32_t actual_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL) - tx_start_us;
            uint32_t deadline_us = target_x3 / 3UL;
            if (actual_us > deadline_us + 10000u) {
                ESP_LOGW(TAG, "Symbol %d overrun: deadline=%lu actual=%lu overrun=%lu us", i, (unsigned long)deadline_us, (unsigned long)actual_us,
                         (unsigned long)(actual_us - deadline_us));
            }
        }
#endif
    }

    // Disable RF output and release the oscillator for calibration updates
    oscillator_enable(false);
    g_tx_active = false;
    oscillator_tx_end();
    g_symbol_idx = 0;

    ESP_LOGI(TAG, "TX complete");
}

/**
 * @brief FreeRTOS task: update the web status panel once per second.
 *
 * Reads the wall clock, current band, frequency, and TX state, then pushes
 * a status snapshot to the web server. Also computes and stores the
 * boot wall-clock time once the first NTP/GPS sync has occurred.
 *
 * @param arg Unused task argument.
 */
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

            // Compute and store the boot wall-clock time on the first sync
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

        // Build the frequency string: dial_freq + 1500 Hz (WSPR center offset)
        if (g_band_idx >= 0) {
            uint32_t freq_hz = config_band_freq_hz((iaru_region_t)region_snap, (wspr_band_t)g_band_idx) + 1500u;
            uint32_t mhz_int = freq_hz / 1000000u;
            uint32_t khz_frac = (freq_hz % 1000000u) / 100u;
            snprintf(freq_str, sizeof(freq_str), "%u.%04u MHz", (unsigned)mhz_int, (unsigned)khz_frac);
        }

        // Countdown to next even-minute TX window; -1 if clock not yet synced
        int32_t next_tx = time_ok ? time_sync_secs_to_next_tx() : -1;

        web_server_update_status(time_ok, time_str, band_name, freq_str, next_tx, g_tx_active, tx_en, g_symbol_idx);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief FreeRTOS task: WSPR transmission scheduler.
 *
 * Implements the core WSPR timing protocol:
 * - Waits for time synchronization (NTP or GPS).
 * - Blocks until tx_enabled is set via the web UI.
 * - Sleeps until the next even-minute UTC boundary (HH:MM:00).
 * - Pre-arms the oscillator and LPF relay at phase=0 (second 0 of the slot).
 * - Aligns to the second boundary with sub-millisecond precision.
 * - Calls wspr_transmit() to send 162 symbols over ~110.6 s.
 * - Applies duty-cycle throttling and frequency hopping between slots.
 *
 * Per the WSPR specification, all transmissions must start within ±1 s of
 * the even-minute boundary. This scheduler targets phase=1 (one second past
 * the boundary) for compatibility with the majority of WSPR decoders.
 *
 * @param arg Unused task argument.
 */
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

    uint32_t last_hop_ts = 0;
    // Duty-cycle accumulator: TX fires when accum >= 100; 20% default = fire every 5 slots
    uint16_t s_duty_accum = 100u;
    bool time_synced_logged = false;

    while (1) {
        // Gate on time sync: WSPR transmissions require UTC accuracy within ±1 s
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

        bool do_skip = false;

        // Compute seconds until the next even-minute WSPR TX window
        int32_t wait_sec = time_sync_secs_to_next_tx();
        if (wait_sec > 0) {
            ESP_LOGI(TAG, "Next TX in %d s (band=%s)", wait_sec, BAND_NAME[g_band_idx]);
            // Sleep most of the wait period in 1-second chunks so the WDT stays fed
            if (wait_sec > 3) {
                uint32_t sleep_ms = (uint32_t)(wait_sec - 2) * 1000u;
                while (sleep_ms > 0u) {
                    uint32_t chunk = (sleep_ms > 1000u) ? 1000u : sleep_ms;
                    vTaskDelay(pdMS_TO_TICKS(chunk));
#if CONFIG_WSPR_TASK_WDT_ENABLE
                    esp_task_wdt_reset();
#endif
                    sleep_ms -= chunk;
                }
            }

            // Fine-grained phase alignment: poll every 10 ms for the slot boundary.
            // Pre-arm the oscillator and LPF relay at phase=0 (second 0 of the slot)
            // so they are settled before RF is enabled at phase=1.
            {
                web_server_cfg_lock();
                uint8_t pre_snap_region = g_cfg.iaru_region;
                web_server_cfg_unlock();
                bool pre_armed_local = false;

                for (;;) {
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    // phase = seconds elapsed since the start of the current 2-minute window
                    uint32_t phase = (uint32_t)(tv.tv_sec % 120u);

                    // At phase=0: configure oscillator frequency and LPF relay one second early
                    if (phase == 0u && !pre_armed_local) {
                        g_pre_arm_base_hz = config_band_freq_hz((iaru_region_t)pre_snap_region, (wspr_band_t)g_band_idx) + 1500u;
                        oscillator_set_freq(g_pre_arm_base_hz);
                        gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                        g_pre_armed = true;
                        pre_armed_local = true;
                        ESP_LOGD(TAG, "Pre-armed: band=%s base_hz=%lu (phase=0)", BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                    }

                    // At phase=1-3: the TX window is open; enter only at phase=1
                    if (phase >= 1u && phase <= 3u) {
                        if (phase > 1u) {
                            // Missed the first second of the window: skip this slot
                            ESP_LOGW(TAG, "Missed TX window (phase=%lu), skipping slot", (unsigned long)phase);
                            g_pre_armed = false;
                            g_pre_arm_base_hz = 0;
                            do_skip = true;
                        }
                        break;
                    }

                    vTaskDelay(pdMS_TO_TICKS(10));

#if CONFIG_WSPR_TASK_WDT_ENABLE
                    esp_task_wdt_reset();
#endif
                    // Abort pre-arm loop if TX was disabled while waiting
                    web_server_cfg_lock();
                    bool still_en = g_cfg.tx_enabled;
                    web_server_cfg_unlock();

                    if (!still_en) {
                        g_pre_armed = false;
                        g_pre_arm_base_hz = 0;
                        do_skip = true;
                        break;
                    }
                }

                // Sub-second alignment: spin until the wall clock is within 5 ms of
                // a full second boundary to minimize the phase error at TX start.
                // WSPR decoders accept up to ±2 s of start-time offset, but tighter
                // alignment improves decoder SNR on marginal signals.
                if (!do_skip) {
                    struct timeval tv_align;
                    gettimeofday(&tv_align, NULL);
                    uint32_t usec_now = (uint32_t)tv_align.tv_usec;

                    if (usec_now >= 5000u && usec_now < 900000u) {
                        uint32_t coarse_us = 1000000u - usec_now;
                        if (coarse_us > 3000u) {
                            uint32_t sleep_ms = (coarse_us - 3000u) / 1000u;
                            if (sleep_ms > 0u)
                                vTaskDelay(pdMS_TO_TICKS(sleep_ms));
                        }

                        // Busy-wait for the final few milliseconds
                        do {
                            gettimeofday(&tv_align, NULL);
                            usec_now = (uint32_t)tv_align.tv_usec;
                            taskYIELD();
#if CONFIG_WSPR_TASK_WDT_ENABLE

                            esp_task_wdt_reset();
#endif
                        } while (usec_now >= 5000u && usec_now < 950000u);
                    }
                }
            }
        }

        if (!do_skip) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            uint32_t now = (uint32_t)(tv.tv_sec & 0xFFFFFFFFUL);

            // Read hop parameters and check if the band list needs rebuilding
            web_server_cfg_lock();
            bool hop_en = g_cfg.hop_enabled;
            uint32_t hop_intv = g_cfg.hop_interval_sec;
            bool need_rebuild = g_cfg.bands_changed;
            g_cfg.bands_changed = false;
            web_server_cfg_unlock();

            if (hop_intv == 0u)
                hop_intv = 120u;

            // Frequency hopping: advance to the next band when the interval expires
            bool do_hop = hop_en && (now - last_hop_ts) >= hop_intv;

            if (do_hop || g_band_idx < 0 || need_rebuild) {
                select_next_band(do_hop, need_rebuild || g_band_idx < 0);
                last_hop_ts = now;

                // If already pre-armed on the old band, re-arm on the new band
                if (g_pre_armed) {
                    web_server_cfg_lock();
                    uint8_t new_region = g_cfg.iaru_region;
                    web_server_cfg_unlock();
                    uint32_t new_base_hz = config_band_freq_hz((iaru_region_t)new_region, (wspr_band_t)g_band_idx) + 1500u;
                    if (new_base_hz != g_pre_arm_base_hz) {
                        g_pre_arm_base_hz = new_base_hz;
                        oscillator_set_freq(g_pre_arm_base_hz);
                        gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
                        ESP_LOGD(TAG, "Re-armed after hop: band=%s base_hz=%lu", BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                    }
                }
            }

            // Duty-cycle check: accumulate the percentage; fire when >= 100
            // This gives a deterministic slot selection (not random), e.g.
            // duty=20 fires slots 0, 5, 10, ... (every fifth 2-minute window).
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
                g_pre_armed = false;
                g_pre_arm_base_hz = 0;
                do_skip = true;
            }
        }

        if (do_skip) {
#if CONFIG_WSPR_TASK_WDT_ENABLE
            esp_task_wdt_reset();
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // All checks passed: encode and transmit the WSPR message for this slot
        wspr_transmit();
    }
}

void app_main(void) {

    // GPS mode forces UTC timezone so NMEA timestamps are not affected by locale
#if defined(CONFIG_WSPR_TIME_GPS)
    setenv("TZ", "UTC0", 1);
    tzset();
#endif

    // Record uptime at boot to compute the boot wall-clock time after first NTP sync
    g_boot_uptime_sec = (uint32_t)(esp_log_timestamp() / 1000u);

    esp_reset_reason_t reset_rsn = esp_reset_reason();

    ESP_LOGI(TAG, "=== WSPR Transmitter for ESP32 ===");
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason_to_str(reset_rsn));

    // Initialize NVS and load persistent configuration
    ESP_ERROR_CHECK(config_init());
    ESP_ERROR_CHECK(config_load(&g_cfg));

    ESP_LOGI(TAG, "Callsign: %s  Locator: %s  Power: %d dBm", g_cfg.callsign, g_cfg.locator, g_cfg.power_dbm);

    // Configure the three LPF-select GPIO pins and drive them to filter 0
    ESP_ERROR_CHECK(gpio_filter_init());

    // Auto-detect oscillator hardware (Si5351 via I2C, then AD9850 via GPIO)
    ESP_ERROR_CHECK(oscillator_init());
    oscillator_enable(false);  // ensure RF output is off at startup

    // Apply any stored crystal calibration offset
    oscillator_set_cal(g_cfg.xtal_cal_ppb);

    // Connect to WiFi in STA mode; fall back to soft-AP if credentials are absent
    ESP_ERROR_CHECK(wifi_manager_start(g_cfg.wifi_ssid, g_cfg.wifi_pass));

    // Start NTP or GPS time sync (selected at build time via Kconfig)
    ESP_ERROR_CHECK(time_sync_init(g_cfg.ntp_server));

    // Start the HTTP configuration server; creates config and status mutexes
    ESP_ERROR_CHECK(web_server_start(&g_cfg));

    ESP_LOGI(TAG, "Web config at http://%s", wifi_manager_ip());

    // Store hardware identity in the status cache for the web UI RF Hardware row
    web_server_set_hw_status(oscillator_hw_ok(), oscillator_hw_name());

    // Store reset reason; boot time string is filled later after first NTP sync
    web_server_set_reboot_info(NULL, reset_reason_to_str(reset_rsn));

    // Create the status update task (lower priority than scheduler)
    xTaskCreate(status_task, "wspr_status", 6144, NULL, 3, NULL);

    // Create the WSPR scheduler task (higher priority: drives precise TX timing)
    xTaskCreate(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL);

    // app_main may return to the idle task; keep it alive with a long sleep loop
    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
