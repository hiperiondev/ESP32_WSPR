/**
 * @file time_sync.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
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

void time_sync_restart_ntp(const char *ntp_server) {
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
#elif defined(CONFIG_WSPR_TIME_GPS)

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define GPS_UART_PORT_NUM ((uart_port_t)CONFIG_GPS_UART_PORT)
#define GPS_BUF_SIZE      512

static TaskHandle_t _gps_task = NULL;

#if defined(CONFIG_GPS_PPS_GPIO) && (CONFIG_GPS_PPS_GPIO >= 0)
#define GPS_PPS_ENABLED 1

static volatile int64_t _pps_us = 0;
#else
#define GPS_PPS_ENABLED 0
#endif

#if GPS_PPS_ENABLED
static void IRAM_ATTR pps_isr(void *arg) {
    _pps_us = esp_timer_get_time();

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

    if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1]))
        return false;
    uint8_t hi = (uint8_t)(isdigit((unsigned char)p[0]) ? p[0] - '0' : toupper((unsigned char)p[0]) - 'A' + 10);
    uint8_t lo = (uint8_t)(isdigit((unsigned char)p[1]) ? p[1] - '0' : toupper((unsigned char)p[1]) - 'A' + 10);
    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return calc == expected;
}

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

    if (yy < 2020 || yy > 2035)
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

static bool parse_rmc_or_zda(const char *sentence, struct timeval *tv) {
    if (strncmp(sentence, "$GPZDA", 6) == 0 || strncmp(sentence, "$GNZDA", 6) == 0)
        return parse_zda(sentence, tv);
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
        return parse_rmc(sentence, tv);

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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t time_sync_init(const char *ntp_server) {
    (void)ntp_server;

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

    uint32_t p = (uint32_t)(now % 120u);
    if (p <= 3u) {
        return 0;
    } else {
        return (int32_t)(120u - p);
    }
}
