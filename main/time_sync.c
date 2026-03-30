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
static volatile bool _synced = false;

// =============================================================================
//  NTP (SNTP) implementation
// =============================================================================
#ifdef CONFIG_WSPR_TIME_NTP

#include "esp_sntp.h"

static void sntp_cb(struct timeval *tv) {
    _synced = true;
    char buf[32];
    struct tm t;
    gmtime_r(&tv->tv_sec, &t);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    ESP_LOGI(TAG, "NTP synchronized: %s UTC", buf);
}

esp_err_t time_sync_init(const char *ntp_server) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_server ? ntp_server : "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started with server: %s", ntp_server ? ntp_server : "pool.ntp.org");
    return ESP_OK;
}

// =============================================================================
//  GPS (NMEA UART) implementation
// =============================================================================
#elif defined(CONFIG_WSPR_TIME_GPS)

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define GPS_UART_PORT_NUM ((uart_port_t)CONFIG_GPS_UART_PORT)
#define GPS_BUF_SIZE      512

static TaskHandle_t _gps_task = NULL;

// PPS GPIO ISR support. When CONFIG_GPS_PPS_GPIO is set to a
// valid GPIO number (>= 0), a rising-edge ISR snaps the system clock sub-second
// part to zero on each PPS pulse. This gives microsecond-level accuracy compared
// to the ~10 ms jitter of pure NMEA sentence parsing via UART polling.
// Set CONFIG_GPS_PPS_GPIO = -1 (Kconfig default) to disable PPS entirely.
#if defined(CONFIG_GPS_PPS_GPIO) && (CONFIG_GPS_PPS_GPIO >= 0)
#define GPS_PPS_ENABLED 1
// Timestamp of the last PPS rising edge in microseconds (esp_timer_get_time()).
// Written from the ISR (IRAM), read from gps_task for logging only.
static volatile int64_t _pps_us = 0;
#else
#define GPS_PPS_ENABLED 0
#endif

// PPS ISR: called on each rising edge of the PPS signal.
// Snaps tv_usec to 0 to align the system wall clock to the exact second
// boundary. The GPS NMEA sentence (parsed by gps_task) has already set the
// correct second value; the PPS pulse corrects the sub-second offset.
// Must be in IRAM because it is called from an ISR context.
#if GPS_PPS_ENABLED
static void IRAM_ATTR pps_isr(void *arg) {
    // Record ISR arrival time for optional diagnostic logging.
    _pps_us = esp_timer_get_time();
    // Snap the sub-second component of the wall clock to zero.
    // settimeofday() is documented as ISR-safe on ESP-IDF (it calls
    // adjtime / clock_settime which use a spinlock internally).
    struct timeval tv;
    gettimeofday(&tv, NULL);
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
}
#endif

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
    // Accept upper and lower case hex digits for robustness
    if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1]))
        return false;
    uint8_t hi = (uint8_t)(isdigit((unsigned char)p[0]) ? p[0] - '0' : toupper((unsigned char)p[0]) - 'A' + 10);
    uint8_t lo = (uint8_t)(isdigit((unsigned char)p[1]) ? p[1] - '0' : toupper((unsigned char)p[1]) - 'A' + 10);
    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return calc == expected;
}

static bool parse_rmc(const char *sentence, struct timeval *tv) {
    // Reject sentences with a bad or missing checksum before parsing any field
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
        return false;

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

    struct tm t = { 0 };
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;
    t.tm_mday = dd;
    t.tm_mon = mo - 1;
    t.tm_year = yy - 1900;

    time_t ts = mktime(&t);
    // mktime() returns (time_t)-1 for invalid or out-of-range dates; reject them
    // to avoid setting the system clock to a garbage epoch value.
    if (ts == (time_t)-1)
        return false;

    tv->tv_sec = ts;
    tv->tv_usec = 0;
    return true;
}

// parse_zda: parse a $GPZDA or $GNZDA sentence.
// Format: $GPZDA,HHMMSS.ss,DD,MM,YYYY,ZH,ZM*CS
// Fields: 1=time, 2=day, 3=month, 4=year, 5=local zone hours (ignored), 6=zone minutes (ignored).
// GPZDA always carries UTC time regardless of local zone fields.
// Returns true and fills *tv on success; returns false on any parse or checksum error.
static bool parse_zda(const char *sentence, struct timeval *tv) {
    // Validate checksum before touching any field.
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

// parse_rmc_or_zda: dispatcher that tries the correct parser
// based on the sentence talker+sentence-id prefix. This is more efficient than
// calling both parsers sequentially and avoids re-copying the buffer.
// Accepts $GPRMC, $GNRMC (RMC path) and $GPZDA, $GNZDA (ZDA path).
// All other sentence types return false immediately without checksum check.
static bool parse_rmc_or_zda(const char *sentence, struct timeval *tv) {
    // Determine sentence type from first 6 characters to dispatch without copying.
    // strncmp is safe even for short strings because the buffer is NUL-terminated.
    if (strncmp(sentence, "$GPZDA", 6) == 0 || strncmp(sentence, "$GNZDA", 6) == 0)
        return parse_zda(sentence, tv);
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
        return parse_rmc(sentence, tv);
    // All other sentence types (GGA, GLL, GSA, etc.) are silently ignored.
    return false;
}

static void gps_task(void *arg) {
    uint8_t buf[GPS_BUF_SIZE];
    char line[GPS_BUF_SIZE];
    int line_pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(1000));

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                line[line_pos] = '\0';
                struct timeval tv;
                // Use parse_rmc_or_zda() dispatcher instead of
                // parse_rmc() alone so that $GPZDA/$GNZDA sentences are also used
                // for time synchronisation. This improves compatibility with
                // dual-constellation GPS modules that may emit GNZDA but not GNRMC.
                if (parse_rmc_or_zda(line, &tv)) {
                    settimeofday(&tv, NULL);
                    _synced = true;
                    ESP_LOGI(TAG, "GPS time set from NMEA sentence");
                }
                line_pos = 0;
            } else if (c != '\r' && line_pos < (int)(sizeof(line) - 1)) {
                line[line_pos++] = c;
            }
        }
        // Explicit short yield after every buffer processing
        // (even when len==0) to guarantee idle task runs and prevent
        // any edge-case watchdog triggers on long GPS-silent periods.
        // uart_read_bytes timeout already yields, but this makes
        // behaviour deterministic and fully explicit.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t time_sync_init(const char *ntp_server) {
    (void)ntp_server;

    // Force UTC timezone before any mktime() call so that NMEA sentences,
    // which always carry UTC time, are interpreted correctly.
    // Without this, mktime() applies the local timezone offset and produces
    // a timestamp that is wrong by the local UTC offset.
    setenv("TZ", "UTC0", 1);
    tzset();

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

    xTaskCreate(gps_task, "gps", 6144, NULL, 5, &_gps_task);
    ESP_LOGI(TAG, "GPS UART started on UART%d RX=%d (TZ forced to UTC)", CONFIG_GPS_UART_PORT, CONFIG_GPS_RX_GPIO);

    // Optionally install PPS GPIO ISR for sub-millisecond
    // time accuracy. The ISR snaps tv_usec to 0 on every rising edge,
    // aligning the system wall clock to the exact GPS second boundary.
    // This requires CONFIG_GPS_PPS_GPIO to be set to a valid GPIO number
    // in menuconfig (range 0-39). Set to -1 (default) to disable PPS.
#if GPS_PPS_ENABLED
    gpio_config_t pps_cfg = {
        .pin_bit_mask = BIT64(CONFIG_GPS_PPS_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t pps_err = gpio_config(&pps_cfg);
    if (pps_err == ESP_OK) {
        // gpio_install_isr_service returns ESP_ERR_INVALID_STATE if already
        // installed (e.g. by the GPIO filter driver); treat that as success.
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

bool time_sync_wait(uint32_t timeout_ms) {
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

int32_t time_sync_secs_to_next_tx(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;

    // Returning 0 only in [1,3] ensures the scheduler never starts a TX later
    // than phase=3, giving a 6.4 s margin before the 120 s slot boundary.
    uint32_t p = (uint32_t)(now % 120);
    if (p < 1) {
        return 1;
    } else if (p <= 3) {
        return 0;
    } else {
        return (int32_t)(121 - p);
    }
}
