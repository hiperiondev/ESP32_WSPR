/**
 * @file time_sync.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @brief UTC time synchronisation subsystem — NTP or GPS, selected at build time.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "time_sync.h"

static const char *TAG = "time_sync";

// Set to true on the first successful time update; never cleared
static volatile bool _synced = false;

// =============================================================================
//  NTP (SNTP) implementation
// =============================================================================
//
// WSPR requires the transmitter clock to be accurate within ±1 second of UTC.
// The SNTP client polls the configured NTP server periodically and calls
// settimeofday() when a valid response is received, updating the system wall
// clock.  After the first sync the _synced flag is set and the WSPR scheduler
// is allowed to begin transmission timing.
//
// Typical NTP accuracy over Wi-Fi: 1-50 ms — well within WSPR's ±1 s budget.
#ifdef CONFIG_WSPR_TIME_NTP

#include "esp_sntp.h"

/**
 * @brief SNTP synchronization callback — called by the SNTP client task.
 *
 * Invoked by esp_sntp on each successful time synchronization. Sets the
 * _synced flag and logs the new UTC time.
 *
 * @param tv Pointer to the timeval set by the SNTP client.
 */
static void sntp_cb(struct timeval *tv) {
    _synced = true;
    char buf[32];
    struct tm t;
    gmtime_r(&tv->tv_sec, &t);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    ESP_LOGI(TAG, "NTP synchronized: %s UTC", buf);
}

esp_err_t time_sync_init(const char *ntp_server) {
    // Configure and start the ESP-IDF SNTP client in polling mode
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_server ? ntp_server : "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started with server: %s", ntp_server ? ntp_server : "pool.ntp.org");
    return ESP_OK;
}

void time_sync_restart_ntp(const char *ntp_server) {
    // Stop the current SNTP session, reconfigure the server, and restart.
    // The _synced flag is intentionally NOT cleared: a previously synced clock
    // remains valid; the restart only changes the server for future polls.
    esp_sntp_stop();
    esp_sntp_setservername(0, ntp_server ? ntp_server : "pool.ntp.org");
    // Re-register callback -- esp_sntp_init() may clear it on some IDF versions.
    sntp_set_time_sync_notification_cb(sntp_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "NTP restarted: server=%s", ntp_server ? ntp_server : "pool.ntp.org");
}

// =============================================================================
//  GPS (NMEA UART) implementation
// =============================================================================
//
// WSPR requires UTC time accuracy within ±1 second.  A UART-connected GPS
// receiver provides UTC time via NMEA-0183 sentences at 1 Hz.  This driver
// parses $GPRMC/$GNRMC (Recommended Minimum Specific GNSS Data) and
// $GPZDA/$GNZDA (Time and Date) sentences.
//
// Optional PPS (Pulse Per Second) support improves sub-second accuracy from
// ~10 ms (limited by UART latency) to a few microseconds by zeroing the
// sub-second component of the system clock on each rising PPS edge.
//
// NMEA sentence checksum: XOR of all bytes between '$' and '*' (exclusive),
// compared with the two-character hex value following '*'.
#elif defined(CONFIG_WSPR_TIME_GPS)

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

// UART port and buffer size for GPS NMEA data reception
#define GPS_UART_PORT_NUM ((uart_port_t)CONFIG_GPS_UART_PORT)
#define GPS_BUF_SIZE      512

static TaskHandle_t _gps_task = NULL;

// PPS support is compiled in only when a valid GPIO is configured
#if defined(CONFIG_GPS_PPS_GPIO) && (CONFIG_GPS_PPS_GPIO >= 0)
#define GPS_PPS_ENABLED 1

// Timestamp of the last PPS rising edge in microseconds (esp_timer_get_time())
static volatile int64_t _pps_us = 0;
// flag set by ISR when a PPS pulse fires; cleared by gps_task
static volatile bool _pps_fired = false;
#else
#define GPS_PPS_ENABLED 0
#endif

#if GPS_PPS_ENABLED
/**
 * @brief PPS rising-edge ISR — runs from IRAM for minimum latency.
 *
 * Captures the current esp_timer timestamp and zeroes the sub-second field of
 * the system wall clock.  The NMEA parser sets the integer second correctly;
 * this ISR only improves the sub-second accuracy from ~10 ms to a few µs.
 *
 * @param arg Unused ISR argument.
 */
static void IRAM_ATTR pps_isr(void *arg) {
    // only record timestamp in the ISR.
    // gettimeofday() and settimeofday() are NOT ISR-safe on ESP-IDF — they
    // acquire FreeRTOS mutexes internally and can deadlock or assert.
    // The sub-second correction is applied from gps_task() in task context.
    _pps_us = esp_timer_get_time(); // esp_timer_get_time is ISR-safe
    _pps_fired = true;              // signal gps_task to zero tv_usec
}
#endif

/**
 * @brief Validate the XOR checksum of an NMEA-0183 sentence.
 *
 * NMEA checksum is defined as the XOR of all characters between '$' and '*'
 * (the delimiters themselves are excluded).  The checksum is expressed as a
 * two-character uppercase hex value after the '*'.
 *
 * @param sentence  NUL-terminated NMEA sentence string starting with '$'.
 * @return true if the computed checksum matches the transmitted checksum.
 */
static bool validate_nmea_checksum(const char *sentence) {
    const char *p = sentence;
    if (*p != '$')
        return false;
    p++;
    uint8_t calc = 0;
    while (*p && *p != '*') {
        calc ^= (uint8_t)*p;
        p++;
    }
    if (*p != '*')
        return false;
    p++;

    if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1]))
        return false;
    uint8_t hi = (uint8_t)(isdigit((unsigned char)p[0]) ? p[0] - '0' : toupper((unsigned char)p[0]) - 'A' + 10);
    uint8_t lo = (uint8_t)(isdigit((unsigned char)p[1]) ? p[1] - '0' : toupper((unsigned char)p[1]) - 'A' + 10);
    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return calc == expected;
}

/**
 * @brief Parse an NMEA $GPRMC or $GNRMC sentence and extract the UTC time.
 *
 * RMC (Recommended Minimum Specific GNSS Data) sentence format:
 *   $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,...*hh
 * Field 1: UTC time HHMMSS[.ss]
 * Field 2: Status A=valid, V=void (no fix)
 * Field 9: Date DDMMYY
 *
 * Sanity check rejects years outside [2020, 2035] to guard against GPS
 * rollover or unprogrammed receiver dates.
 *
 * @param sentence  NUL-terminated NMEA RMC sentence.
 * @param tv        Output timeval set from the parsed UTC date and time.
 * @return true if the sentence is valid, has an active fix, and parsed cleanly.
 */
static bool parse_rmc(const char *sentence, struct timeval *tv) {
    if (!validate_nmea_checksum(sentence))
        return false;

    char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save;
    char *tok = strtok_r(buf, ",", &save);
    if (!tok)
        return false;
    if (strcmp(tok, "$GPRMC") != 0 && strcmp(tok, "$GNRMC") != 0)
        return false;

    // Field 1: HHMMSS[.ss]
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 6)
        return false;
    int hh = (tok[0] - '0') * 10 + (tok[1] - '0');
    int mm = (tok[2] - '0') * 10 + (tok[3] - '0');
    int ss = (tok[4] - '0') * 10 + (tok[5] - '0');

    // Field 2: status A=valid V=void
    tok = strtok_r(NULL, ",", &save);
    if (!tok || tok[0] != 'A')
        return false; // no valid fix; reject the sentence

    // Fields 3-8: lat, N/S, lon, E/W, speed, course -- skip with NULL guard
    for (int i = 0; i < 6; i++) {
        tok = strtok_r(NULL, ",", &save);
        if (!tok)
            return false;
    }

    // Field 9: DDMMYY
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 6)
        return false;
    int dd = (tok[0] - '0') * 10 + (tok[1] - '0');
    int mo = (tok[2] - '0') * 10 + (tok[3] - '0');
    int yy = (tok[4] - '0') * 10 + (tok[5] - '0') + 2000;

    // Reject clearly invalid or rollover years
    if (yy < 2020 || yy > 2099)
        return false;

    struct tm t = { 0 };
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;
    t.tm_mday = dd;
    t.tm_mon = mo - 1; // tm_mon is 0-based
    t.tm_year = yy - 1900;

    time_t ts = mktime(&t);

    if (ts == (time_t)-1)
        return false;

    tv->tv_sec = ts;
    tv->tv_usec = 0;
    return true;
}

/**
 * @brief Parse an NMEA $GPZDA or $GNZDA sentence and extract the UTC time.
 *
 * ZDA (Time and Date) sentence format:
 *   $GPZDA,HHMMSS.ss,DD,MM,YYYY,xx,yy*hh
 * Field 1: UTC time HHMMSS[.ss]
 * Field 2: Day (01-31)
 * Field 3: Month (01-12)
 * Field 4: Year (4 digits)
 *
 * ZDA provides a 4-digit year, avoiding the 2-digit year ambiguity of RMC.
 * Sanity bounds check prevents accepting obviously bad timestamps.
 *
 * @param sentence  NUL-terminated NMEA ZDA sentence.
 * @param tv        Output timeval set from the parsed UTC date and time.
 * @return true if the sentence parsed cleanly and values are in valid ranges.
 */
static bool parse_zda(const char *sentence, struct timeval *tv) {
    if (!validate_nmea_checksum(sentence))
        return false;

    char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save;
    char *tok = strtok_r(buf, ",", &save);
    if (!tok)
        return false;
    // Accept both $GPZDA (single-constellation) and $GNZDA (multi-constellation).
    if (strcmp(tok, "$GPZDA") != 0 && strcmp(tok, "$GNZDA") != 0)
        return false;

    // Field 1: HHMMSS[.ss] -- time in UTC
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 6)
        return false;
    int hh = (tok[0] - '0') * 10 + (tok[1] - '0');
    int mm = (tok[2] - '0') * 10 + (tok[3] - '0');
    int ss = (tok[4] - '0') * 10 + (tok[5] - '0');

    // Field 2: DD (day of month)
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 2)
        return false;
    int dd = (tok[0] - '0') * 10 + (tok[1] - '0');

    // Field 3: MM (month)
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 2)
        return false;
    int mo = (tok[0] - '0') * 10 + (tok[1] - '0');

    // Field 4: YYYY (4-digit year)
    tok = strtok_r(NULL, ",", &save);
    if (!tok || strlen(tok) < 4)
        return false;
    int yy = (tok[0] - '0') * 1000 + (tok[1] - '0') * 100 + (tok[2] - '0') * 10 + (tok[3] - '0');

    // Sanity-check parsed values before passing to mktime().
    // Reject obviously invalid dates to avoid silent garbage timestamps.
    if (hh > 23 || mm > 59 || ss > 60 || dd < 1 || dd > 31 || mo < 1 || mo > 12 || yy < 2000 || yy > 2099)
        return false;

    struct tm t = { 0 };
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;
    t.tm_mday = dd;
    t.tm_mon = mo - 1;
    t.tm_year = yy - 1900;

    time_t ts = mktime(&t);
    if (ts == (time_t)-1)
        return false;

    tv->tv_sec = ts;
    tv->tv_usec = 0;
    return true;
}

/**
 * @brief Dispatch an NMEA sentence to the appropriate parser.
 *
 * Checks the sentence identifier prefix and routes to parse_zda() (preferred,
 * has 4-digit year) or parse_rmc().  All other sentence types are ignored.
 *
 * @param sentence  NUL-terminated NMEA sentence.
 * @param tv        Output timeval filled on successful parse.
 * @return true if the sentence was recognized and parsed successfully.
 */
static bool parse_rmc_or_zda(const char *sentence, struct timeval *tv) {
    if (strncmp(sentence, "$GPZDA", 6) == 0 || strncmp(sentence, "$GNZDA", 6) == 0)
        return parse_zda(sentence, tv);
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
        return parse_rmc(sentence, tv);

    return false;
}

/**
 * @brief FreeRTOS task: read NMEA sentences from UART and update the system clock.
 *
 * Continuously reads bytes from the GPS UART, assembles complete NMEA sentences
 * terminated by LF, and calls parse_rmc_or_zda().  When a valid sentence is
 * received the system wall clock is updated via settimeofday() and _synced is set.
 *
 * At 9600 baud a full RMC sentence (~70 bytes) takes ~73 ms, so a 1-second
 * uart_read_bytes timeout is sufficient to detect UART failure without busy-polling.
 *
 * @param arg Unused task argument.
 */
static void gps_task(void *arg) {
    uint8_t buf[GPS_BUF_SIZE];
    char line[GPS_BUF_SIZE];
    int line_pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(1000));

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                // End of sentence: attempt to parse it
                line[line_pos] = '\0';
                struct timeval tv;

                if (parse_rmc_or_zda(line, &tv)) {
                    settimeofday(&tv, NULL);
                    _synced = true;
                    ESP_LOGI(TAG, "GPS time set from NMEA sentence");
#if GPS_PPS_ENABLED
                    // apply PPS sub-second correction here in
                    // task context where gettimeofday/settimeofday are safe.
                    // The ISR only sets _pps_fired; the actual clock adjustment
                    // happens here to avoid calling mutex-acquiring functions from ISR.
                    if (_pps_fired) {
                        _pps_fired = false;
                        struct timeval tv_pps;
                        gettimeofday(&tv_pps, NULL);
                        tv_pps.tv_usec = 0;
                        settimeofday(&tv_pps, NULL);
                    }
#endif
                }
                line_pos = 0;
            } else if (c != '\r' && line_pos < (int)(sizeof(line) - 1)) {
                line[line_pos++] = c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t time_sync_init(const char *ntp_server) {
    (void)ntp_server; // ignored in GPS mode

    // Force UTC timezone so NMEA UTC timestamps are not affected by locale or DST
    setenv("TZ", "UTC0", 1);
    tzset();

    // Configure and install the UART driver for the GPS receiver
    uart_config_t uart_cfg = {
        .baud_rate = CONFIG_GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(GPS_UART_PORT_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT_NUM, &uart_cfg);
    uart_set_pin(GPS_UART_PORT_NUM, CONFIG_GPS_TX_GPIO, CONFIG_GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Create the NMEA parsing task (6 KB stack; NMEA lines are < 100 bytes)
    xTaskCreate(gps_task, "gps", 6144, NULL, 5, &_gps_task);
    ESP_LOGI(TAG, "GPS UART started on UART%d RX=%d (TZ forced to UTC)", CONFIG_GPS_UART_PORT, CONFIG_GPS_RX_GPIO);

#if GPS_PPS_ENABLED
    // Configure the PPS GPIO as an input with a rising-edge interrupt
    gpio_config_t pps_cfg = {
        .pin_bit_mask = BIT64(CONFIG_GPS_PPS_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t pps_err = gpio_config(&pps_cfg);
    if (pps_err == ESP_OK) {
        // Install the GPIO ISR service (shared for all GPIO interrupts)
        esp_err_t svc_err = gpio_install_isr_service(0);
        if (svc_err != ESP_OK && svc_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "GPS PPS: gpio_install_isr_service failed (%s) -- PPS disabled", esp_err_to_name(svc_err));
        } else {
            pps_err = gpio_isr_handler_add((gpio_num_t)CONFIG_GPS_PPS_GPIO, pps_isr, NULL);
            if (pps_err != ESP_OK) {
                ESP_LOGW(TAG, "GPS PPS: gpio_isr_handler_add failed (%s) -- PPS disabled", esp_err_to_name(pps_err));
            } else {
                ESP_LOGI(TAG, "GPS PPS ISR installed on GPIO %d", CONFIG_GPS_PPS_GPIO);
            }
        }
    } else {
        ESP_LOGW(TAG, "GPS PPS: gpio_config failed (%s) -- PPS disabled", esp_err_to_name(pps_err));
    }
#endif

    return ESP_OK;
}

#else
#error "Select WSPR_TIME_NTP or WSPR_TIME_GPS in menuconfig"
#endif

// =============================================================================
//  Common implementation
// =============================================================================

bool time_sync_is_ready(void) {
    return _synced;
}

// clamp timeout_ms to avoid uint32_t wrap when elapsed += 500 is repeated many times.
// Maximum meaningful value: 0xFFFFFE00 ms (~49.7 days). Values above this would silently
// wrap the elapsed accumulator before the timeout expires, causing premature false returns.
bool time_sync_wait(uint32_t timeout_ms) {
    // Clamp to avoid uint32_t wrap in the elapsed accumulator (step = 500 ms)
    if (timeout_ms > 0xFFFFFE00u)
        timeout_ms = 0xFFFFFE00u;
    // Poll _synced every 500 ms; block indefinitely when timeout_ms is 0
    uint32_t elapsed = 0;
    while (!_synced) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (timeout_ms > 0) {
            elapsed += 500;
            if (elapsed >= timeout_ms)
                return false;
        }
    }
    return true;
}

bool time_sync_get(struct timeval *tv) {
    if (!_synced)
        return false;
    gettimeofday(tv, NULL);
    return true;
}

// WSPR transmissions must start at second 1 of each even UTC minute.
// Per the G4JNT/K1JT WSPR 2.0 specification:
//   "Transmissions nominally start one second into an even UTC minute
//    (e.g. at hh:00:01, hh:02:01, ...)"
// References:
//   - G4JNT 2009: http://www.g4jnt.com/Coding/WSPR_Coding_Process.pdf
//   - K1JT WSPR 2.0 User Guide (Princeton University)
//   - m0xpd blog 2013: "a WSPR message must start 1 second into an even minute"
//     http://m0xpd.blogspot.com/2013/01/
//   - https://swharden.com/software/FSKview/wspr/ ("begins 1 sec after even minutes")
//   - https://en.wikipedia.org/wiki/WSPR_(amateur_radio_software)
//
// The 2-minute cycle comes from the WSPR frame duration:
//   162 symbols * 8192/12000 s/symbol = 110.6 s, rounded up to 120 s (2 min).
//
// Phase mapping for scheduler_task (main.c):
//   p == 0  : pre-arm second (:00) — return 0 so the scheduler enters the fine loop
//             immediately and can set oscillator + LPF one second before TX starts.
//   p == 1  : TX start second (:01) — return 0 (window is open now).
//   p == 2..4: just missed the TX window — return 0 so the scheduler enters the fine
//             loop, detects phase >= 2 && <= 4, and skips this slot without sleeping
//             through the entire remainder of the 2-minute cycle.
//   p >= 5  : wait until next phase == 1, which is (120 - p + 1) = (121 - p) seconds away.
int32_t time_sync_secs_to_next_tx(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t p = (uint32_t)(tv.tv_sec % 120u);
    // Phase 0: pre-arm second — return 0 so the scheduler enters the fine loop
    // Phase 1: TX start second — return 0 (window open)
    if (p <= 1u)
        return 0;
    // Phase 2..4: just missed the window — caller will skip this slot via phase check
    if (p <= 4u)
        return 0;
    // p >= 5: wait until the next phase == 1, which is (121 - p) seconds away
    return (int32_t)(121u - p);
}
