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
 
// MODIFIED 3.14: added ctype.h for isxdigit(), isdigit(), toupper() used in GPS mode
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

#include "driver/uart.h"

#define GPS_UART_PORT_NUM ((uart_port_t)CONFIG_GPS_UART_PORT)
#define GPS_BUF_SIZE      512

static TaskHandle_t _gps_task = NULL;

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

    // Fields 3-8: lat, N/S, lon, E/W, speed, course — skip with NULL guard
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
                if (parse_rmc(line, &tv)) {
                    settimeofday(&tv, NULL);
                    _synced = true;
                    ESP_LOGI(TAG, "GPS time set from RMC sentence");
                }
                line_pos = 0;
            } else if (c != '\r' && line_pos < (int)(sizeof(line) - 1)) {
                line[line_pos++] = c;
            }
        }
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

    xTaskCreate(gps_task, "gps", 4096, NULL, 5, &_gps_task);
    ESP_LOGI(TAG, "GPS UART started on UART%d RX=%d (TZ forced to UTC)", CONFIG_GPS_UART_PORT, CONFIG_GPS_RX_GPIO);
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
    // Next TX slot starts 1 second past the next even minute
    time_t next = (now - (now % 120)) + 120 + 1;
    return (int32_t)(next - now);
}
