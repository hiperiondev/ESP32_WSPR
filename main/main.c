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

    uint32_t base_hz;
    if (g_pre_armed) {
        base_hz = g_pre_arm_base_hz;
    } else {
        base_hz = config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx);
        oscillator_set_freq(base_hz);
        gpio_filter_select(BAND_FILTER[g_band_idx]);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
    }

    g_pre_armed = false;

    oscillator_tx_begin();
    oscillator_enable(true);
    g_tx_active = true;
    uint32_t tx_start_us = timer_us32();

    ESP_LOGI(TAG, "TX start: band=%s freq=%lu+1500 Hz type=%d parity=%d pre_armed=%d stack HWM=%u bytes", BAND_NAME[g_band_idx],
             (unsigned long)config_band_freq_hz((iaru_region_t)snap_region, (wspr_band_t)g_band_idx), (int)msg_type, (int)snap_parity,
             (int)(g_pre_arm_base_hz != 0), (unsigned)uxTaskGetStackHighWaterMark(NULL));

    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        g_symbol_idx = i;

#if CONFIG_WSPR_TASK_WDT_ENABLE
        esp_task_wdt_reset();
#endif

        int32_t tone_offset_millihz = (int32_t)(((uint32_t)symbols[i] * WSPR_TONE_NUM + (WSPR_TONE_DEN / 2UL)) / WSPR_TONE_DEN);
        oscillator_set_freq_mhz(base_hz, tone_offset_millihz);

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

#if CONFIG_WSPR_TASK_WDT_ENABLE
    {
        esp_err_t wdt_err = esp_task_wdt_add(NULL);
        if (wdt_err != ESP_OK)
            ESP_LOGW(TAG, "WDT add failed (%s) -- watchdog not active for scheduler task", esp_err_to_name(wdt_err));
    }
#endif

    // Initial band selection: build the active list and choose the first band
    select_next_band(false, true);

    uint32_t s_last_hop_slot = 0;
    bool s_hop_anchored = false;
    bool s_duty_primed = false; // true after first-slot pre-load
    uint16_t s_duty_accum = 0u;

    bool time_synced_logged = false;

    while (1) {
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

        bool do_skip = false;

        int32_t wait_sec = time_sync_secs_to_next_tx();
        if (wait_sec >= 0) {
            ESP_LOGI(TAG, "Next TX in %d s (band=%s)", wait_sec, BAND_NAME[g_band_idx]);
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
                    } else if (phase == 1u) {
                        break;
                    } else if (phase >= 2u && phase <= 4u) {
                        ESP_LOGW(TAG, "Missed TX window (phase=%lu), skipping slot", (unsigned long)phase);
                        g_pre_armed = false;
                        g_pre_arm_base_hz = 0;
                        do_skip = true;
                        break;
                    }

                    vTaskDelay(pdMS_TO_TICKS(1));

#if CONFIG_WSPR_TASK_WDT_ENABLE
                    esp_task_wdt_reset();
#endif
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

            web_server_cfg_lock();
            bool hop_en = g_cfg.hop_enabled;
            uint32_t hop_intv = g_cfg.hop_interval_sec;
            bool need_rebuild = g_cfg.bands_changed;
            g_cfg.bands_changed = false;
            web_server_cfg_unlock();

            if (hop_intv == 0u)
                hop_intv = 120u;

            uint32_t current_slot = now / 120u;
            uint32_t hop_slots = (hop_intv >= 120u) ? (hop_intv / 120u) : 1u;

            bool do_hop;
            if (!s_hop_anchored) {
                do_hop = false;
                s_last_hop_slot = current_slot;
                s_hop_anchored = true;
            } else {
                do_hop = hop_en && ((current_slot - s_last_hop_slot) >= hop_slots);
            }

            if (do_hop || g_band_idx < 0 || need_rebuild) {
                select_next_band(do_hop, need_rebuild || g_band_idx < 0);
                if (do_hop)
                    s_last_hop_slot = current_slot;

                if (g_pre_armed) {
                    web_server_cfg_lock();
                    uint8_t new_region = g_cfg.iaru_region;
                    web_server_cfg_unlock();
                    uint32_t new_base_hz = config_band_freq_hz((iaru_region_t)new_region, (wspr_band_t)g_band_idx);
                    if (new_base_hz != g_pre_arm_base_hz) {
                        g_pre_arm_base_hz = new_base_hz;
                        oscillator_enable(false);
                        oscillator_set_freq(g_pre_arm_base_hz);
                        gpio_filter_select(BAND_FILTER[(int)g_band_idx]);
                        vTaskDelay(pdMS_TO_TICKS(CONFIG_WSPR_LPF_SETTLE_MS));
                        ESP_LOGD(TAG, "Re-armed after hop: band=%s base_hz=%lu", BAND_NAME[(int)g_band_idx], (unsigned long)g_pre_arm_base_hz);
                    }
                }
            }

            // Prime the duty accumulator on the very first slot so that
            // the first TX fires immediately.
            web_server_cfg_lock();
            uint8_t duty = g_cfg.tx_duty_pct;
            web_server_cfg_unlock();

            if (!s_duty_primed) {
                // Load enough credit so that the first add brings accum to >= 100.
                // Clamp to 99 so that a duty of 100 (always TX) still takes one add
                // to fire and the normal accumulator logic remains uniform.
                if (duty > 0u && duty < 100u) {
                    s_duty_accum = (uint16_t)(100u - duty);
                } else if (duty >= 100u) {
                    s_duty_accum = 0u; // will fire immediately (100 added below)
                }
                // duty==0: leave at 0; the duty>0 guard below prevents TX anyway.
                s_duty_primed = true;
            }

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
        } else {
            web_server_cfg_lock();
            bool still_tx_en = g_cfg.tx_enabled;
            uint8_t duty_snap = g_cfg.tx_duty_pct;
            web_server_cfg_unlock();

            if (still_tx_en && duty_snap > 0u) {
                if (!s_duty_primed) {
                    // Same first-slot priming as the normal path above
                    if (duty_snap < 100u)
                        s_duty_accum = (uint16_t)(100u - duty_snap);
                    s_duty_primed = true;
                }
                s_duty_accum += (uint16_t)duty_snap;
                // Clamp: don't allow unbounded growth if many windows are missed in a row
                if (s_duty_accum > 200u)
                    s_duty_accum = 200u;
                ESP_LOGD(TAG, "Duty accum (missed window): accum=%u", (unsigned)s_duty_accum);
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

#if defined(CONFIG_WSPR_TIME_GPS)
    setenv("TZ", "UTC0", 1);
    tzset();
#endif

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

    ESP_ERROR_CHECK(web_server_start(&g_cfg));

    ESP_LOGI(TAG, "Web config at http://%s", wifi_manager_ip());

    web_server_set_hw_status(oscillator_hw_ok(), oscillator_hw_name());

    web_server_set_reboot_info(NULL, reset_reason_to_str(reset_rsn));

    xTaskCreate(status_task, "wspr_status", 6144, NULL, 3, NULL);

    xTaskCreate(scheduler_task, "wspr_sched", 8192, NULL, 5, NULL);

    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
