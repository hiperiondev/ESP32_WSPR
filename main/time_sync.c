/**
 * @file time_sync.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @brief UTC time synchronisation subsystem — auto-detected GPS or NTP fallback.
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

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_sntp.h"
#include "esp_timer.h"

#include "time_sync.h"

static const char *TAG = "time_sync";

// Set to true on the first successful time update; never cleared
static volatile bool _synced = false;

// runtime-selected time source (set during time_sync_init)
static volatile time_sync_source_t _source = TIME_SYNC_NONE;

// Last valid GPS position extracted from $GPRMC or $GPGGA sentences.
// Written only by gps_task; read by time_sync_get_position() from any task.
// double reads are not atomic on 32-bit MCUs, but worst case is a stale value
// which is acceptable — the UI will retry next poll cycle.
static volatile double _gps_lat = 0.0;
static volatile double _gps_lon = 0.0;
static volatile bool _gps_pos_valid = false;

// =============================================================================
//  GPS (NMEA UART) implementation
// =============================================================================

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
// PPS rising-edge ISR — runs from IRAM for minimum latency.
// Only records timestamp; actual clock correction done in task context.
static void IRAM_ATTR pps_isr(void *arg) {
    _pps_us = esp_timer_get_time(); // esp_timer_get_time is ISR-safe
    _pps_fired = true;              // signal gps_task to zero tv_usec
}
#endif

// Validate the XOR checksum of an NMEA-0183 sentence.
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

// helper to convert NMEA DDMM.MMMM format to decimal degrees.
// deg_digits is 2 for latitude (DDMM.MM) or 3 for longitude (DDDMM.MM).
static double nmea_to_decimal(const char *field, int deg_digits) {
    if (!field || field[0] == '\0')
        return 0.0;
    // Integer degrees: first deg_digits characters
    double deg = 0.0;
    for (int i = 0; i < deg_digits; i++) {
        if (field[i] < '0' || field[i] > '9')
            return 0.0;
        deg = deg * 10.0 + (field[i] - '0');
    }
    // Remaining characters are minutes (MM.MMMM...)
    double min = 0.0;
    // strtod-free approach: parse integer minutes then fraction
    const char *mp = field + deg_digits;
    double min_int = 0.0;
    while (*mp && *mp != '.') {
        if (*mp < '0' || *mp > '9')
            break;
        min_int = min_int * 10.0 + (*mp - '0');
        mp++;
    }
    double min_frac = 0.0;
    double divisor = 10.0;
    if (*mp == '.') {
        mp++;
        while (*mp && *mp >= '0' && *mp <= '9') {
            min_frac += (*mp - '0') / divisor;
            divisor *= 10.0;
            mp++;
        }
    }
    min = min_int + min_frac;
    return deg + min / 60.0;
}

// Parse an NMEA $GPRMC or $GNRMC sentence and extract the UTC time.
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

    // Field 3: latitude (DDMM.MMMM)
    tok = strtok_r(NULL, ",", &save);
    char lat_field[16] = { 0 };
    if (tok)
        strncpy(lat_field, tok, sizeof(lat_field) - 1);

    // Field 4: N/S hemisphere
    tok = strtok_r(NULL, ",", &save);
    char lat_ns = tok ? tok[0] : 'N';

    // Field 5: longitude (DDDMM.MMMM)
    tok = strtok_r(NULL, ",", &save);
    char lon_field[16] = { 0 };
    if (tok)
        strncpy(lon_field, tok, sizeof(lon_field) - 1);

    // Field 6: E/W hemisphere
    tok = strtok_r(NULL, ",", &save);
    char lon_ew = tok ? tok[0] : 'E';

    // Skip speed and course fields
    tok = strtok_r(NULL, ",", &save); // speed
    tok = strtok_r(NULL, ",", &save); // course

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

    // update GPS position from RMC lat/lon fields when fix is valid
    if (lat_field[0] != '\0' && lon_field[0] != '\0') {
        double lat = nmea_to_decimal(lat_field, 2);
        double lon = nmea_to_decimal(lon_field, 3);
        if (lat_ns == 'S' || lat_ns == 's')
            lat = -lat;
        if (lon_ew == 'W' || lon_ew == 'w')
            lon = -lon;
        _gps_lat = lat;
        _gps_lon = lon;
        _gps_pos_valid = true;
    }

    return true;
}

// Parse an NMEA $GPZDA or $GNZDA sentence and extract the UTC time.
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

// parse $GPGGA or $GNGGA sentence to extract position (lat/lon).
// GGA provides more precise fix quality info. Position update only; returns false
// so the caller does not try to use the tv for time (ZDA/RMC handle time).
static bool parse_gga_position(const char *sentence) {
    if (!validate_nmea_checksum(sentence))
        return false;

    char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save;
    char *tok = strtok_r(buf, ",", &save);
    if (!tok)
        return false;
    if (strcmp(tok, "$GPGGA") != 0 && strcmp(tok, "$GNGGA") != 0)
        return false;

    // Field 1: UTC time — skip
    tok = strtok_r(NULL, ",", &save);
    if (!tok)
        return false;

    // Field 2: latitude DDMM.MMMM
    tok = strtok_r(NULL, ",", &save);
    char lat_field[16] = { 0 };
    if (tok)
        strncpy(lat_field, tok, sizeof(lat_field) - 1);

    // Field 3: N/S
    tok = strtok_r(NULL, ",", &save);
    char lat_ns = tok ? tok[0] : 'N';

    // Field 4: longitude DDDMM.MMMM
    tok = strtok_r(NULL, ",", &save);
    char lon_field[16] = { 0 };
    if (tok)
        strncpy(lon_field, tok, sizeof(lon_field) - 1);

    // Field 5: E/W
    tok = strtok_r(NULL, ",", &save);
    char lon_ew = tok ? tok[0] : 'E';

    // Field 6: fix quality (0 = invalid)
    tok = strtok_r(NULL, ",", &save);
    if (!tok || tok[0] == '0')
        return false; // no fix

    if (lat_field[0] == '\0' || lon_field[0] == '\0')
        return false;

    // store position from GGA sentence
    double lat = nmea_to_decimal(lat_field, 2);
    double lon = nmea_to_decimal(lon_field, 3);
    if (lat_ns == 'S' || lat_ns == 's')
        lat = -lat;
    if (lon_ew == 'W' || lon_ew == 'w')
        lon = -lon;
    _gps_lat = lat;
    _gps_lon = lon;
    _gps_pos_valid = true;
    return true;
}

// Dispatch an NMEA sentence to the appropriate parser.
static bool parse_rmc_or_zda(const char *sentence, struct timeval *tv) {
    if (strncmp(sentence, "$GPZDA", 6) == 0 || strncmp(sentence, "$GNZDA", 6) == 0)
        return parse_zda(sentence, tv);
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
        return parse_rmc(sentence, tv);
    return false;
}

// gps_task — runs permanently once GPS mode is confirmed.
// Reads NMEA from UART, sets system clock, handles PPS correction.
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

                // also handle GGA for position extraction
                if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
                    parse_gga_position(line);
                } else if (parse_rmc_or_zda(line, &tv)) {
                    settimeofday(&tv, NULL);
                    _synced = true;
                    ESP_LOGI(TAG, "GPS time set from NMEA sentence");
#if GPS_PPS_ENABLED
                    // Apply PPS sub-second correction in task context (not ISR)
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

// gps_uart_init — configure UART driver for GPS reception.
// Shared by both the probe and the permanent GPS task.
static esp_err_t gps_uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = CONFIG_GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t err = uart_driver_install(GPS_UART_PORT_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means driver already installed (probe called first)
        return err;
    }
    err = uart_param_config(GPS_UART_PORT_NUM, &uart_cfg);
    if (err != ESP_OK)
        return err;
    err = uart_set_pin(GPS_UART_PORT_NUM, CONFIG_GPS_TX_GPIO, CONFIG_GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return err;
}

// gps_probe_detect — attempt to receive a valid NMEA sentence within
// timeout_ms milliseconds. Returns true if GPS is detected (valid NMEA received).
// The UART driver is installed and left open when returning true so gps_task can
// reuse it without reinstalling.  When returning false the UART driver is uninstalled.
static bool gps_probe_detect(uint32_t timeout_ms) {
    // Install UART driver for the probe attempt
    esp_err_t err = gps_uart_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPS probe: UART init failed (%s) -- assuming no GPS", esp_err_to_name(err));
        return false;
    }

    uint8_t buf[GPS_BUF_SIZE];
    char line[GPS_BUF_SIZE];
    int line_pos = 0;
    uint32_t elapsed_ms = 0;
    // Read in CONFIG_GPS_DETECT_TIMEOUT_MS / 200 ms chunks
    const uint32_t CHUNK_MS = 200;

    ESP_LOGI(TAG, "GPS auto-detect: probing UART%d RX=%d for %u ms ...", CONFIG_GPS_UART_PORT, CONFIG_GPS_RX_GPIO, (unsigned)timeout_ms);

    // Flush stale bytes ONCE before the loop, not on every iteration.
    // Flushing inside the loop discards partial NMEA sentences accumulated in the
    // previous 200 ms window: any sentence spanning two consecutive read windows
    // was silently dropped, making GPS detection unreliable at 9600 baud where
    // an 80-char sentence takes ~83 ms and can easily span two 200 ms windows.
    uart_flush(GPS_UART_PORT_NUM);

    while (elapsed_ms < timeout_ms) {
        int len = uart_read_bytes(GPS_UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(CHUNK_MS));
        elapsed_ms += CHUNK_MS;

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                line[line_pos] = '\0';
                // Check for '$' prefix and '*' checksum delimiter — that is a valid NMEA frame
                if (line_pos > 6 && line[0] == '$' && strchr(line, '*') != NULL) {
                    // Try to validate checksum as a stronger confirmation
                    if (validate_nmea_checksum(line)) {
                        ESP_LOGI(TAG, "GPS auto-detect: valid NMEA sentence received after %u ms", (unsigned)elapsed_ms);
                        // Leave UART driver installed for gps_task
                        return true;
                    }
                }
                line_pos = 0;
            } else if (c != '\r' && line_pos < (int)(sizeof(line) - 1)) {
                line[line_pos++] = c;
            }
        }
    }

    ESP_LOGI(TAG, "GPS auto-detect: no valid NMEA received in %u ms -- using NTP", (unsigned)timeout_ms);
    // Uninstall UART driver so it doesn't consume resources in NTP mode
    uart_driver_delete(GPS_UART_PORT_NUM);
    return false;
}

// install_gps_pps — configure PPS ISR if a valid GPIO is configured.
static void install_gps_pps(void) {
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
}

// =============================================================================
//  NTP (SNTP) implementation
// =============================================================================

// SNTP synchronization callback — called by the SNTP client task.
static void sntp_cb(struct timeval *tv) {
    _synced = true;
    char buf[32];
    struct tm t;
    gmtime_r(&tv->tv_sec, &t);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    ESP_LOGI(TAG, "NTP synchronized: %s UTC", buf);
}

// start_ntp_client — configure and start the SNTP client.
static void start_ntp_client(const char *ntp_server) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_server ? ntp_server : "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started with server: %s", ntp_server ? ntp_server : "pool.ntp.org");
}

// =============================================================================
//  Public API
// =============================================================================

// time_sync_init now performs GPS auto-detection then falls back to NTP.
// The CONFIG_WSPR_TIME_NTP / CONFIG_WSPR_TIME_GPS compile-time choice is no longer used.
// Both GPS UART code and SNTP code are compiled in unconditionally.
esp_err_t time_sync_init(const char *ntp_server) {
    // Force UTC timezone in all modes to prevent locale interference
    setenv("TZ", "UTC0", 1);
    tzset();

    // probe GPS UART; activate GPS mode if a valid sentence is found,
    // otherwise start NTP.  The detection timeout is taken from Kconfig.
    bool gps_found = gps_probe_detect((uint32_t)CONFIG_GPS_DETECT_TIMEOUT_MS);

    if (gps_found) {
        // GPS module confirmed — start the permanent NMEA parsing task
        _source = TIME_SYNC_GPS;
        xTaskCreate(gps_task, "gps", 6144, NULL, 5, &_gps_task);
        install_gps_pps();
        ESP_LOGI(TAG, "Time source: GPS (UART%d RX=%d)", CONFIG_GPS_UART_PORT, CONFIG_GPS_RX_GPIO);
    } else {
        // No GPS — fall back to NTP
        _source = TIME_SYNC_NTP;
        start_ntp_client(ntp_server);
        ESP_LOGI(TAG, "Time source: NTP (%s)", ntp_server ? ntp_server : "pool.ntp.org");
    }

    return ESP_OK;
}

// time_sync_restart_ntp — only acts when NTP is the active source.
void time_sync_restart_ntp(const char *ntp_server) {
    if (_source != TIME_SYNC_NTP) {
        ESP_LOGW(TAG, "time_sync_restart_ntp: active source is GPS, ignoring");
        return;
    }
    // Stop, reconfigure, and restart the SNTP client
    esp_sntp_stop();
    esp_sntp_setservername(0, ntp_server ? ntp_server : "pool.ntp.org");
    // Re-register callback -- esp_sntp_init() may clear it on some IDF versions
    sntp_set_time_sync_notification_cb(sntp_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "NTP restarted: server=%s", ntp_server ? ntp_server : "pool.ntp.org");
}

// Return the auto-detected time source
time_sync_source_t time_sync_source(void) {
    return _source;
}

// =============================================================================
//  Common implementation (shared by both GPS and NTP modes)
// =============================================================================

bool time_sync_is_ready(void) {
    return _synced;
}

// Clamp timeout_ms to avoid uint32_t wrap in the elapsed accumulator (step = 500 ms)
bool time_sync_wait(uint32_t timeout_ms) {
    if (timeout_ms > 0xFFFFFE00u)
        timeout_ms = 0xFFFFFE00u;
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

// WSPR transmissions start at second 1 of each even UTC minute.
// Phase mapping:
//   p == 0  : pre-arm second (:00) -- return 0
//   p == 1  : TX start second (:01) -- return 0
//   p >= 2  : return seconds until next phase 1 (= 121 - p)
// When p==2..4 the post-skip guard loop (main.c) checked secs_to_next_tx()>0 to exit;
int32_t time_sync_secs_to_next_tx(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t p = (uint32_t)(tv.tv_sec % 120u);
    if (p <= 1u)
        return 0;

    return (int32_t)(121u - p);
}

// time_sync_get_position — returns last GPS fix coordinates.
// Returns true only when GPS mode is active and a valid fix has been received.
// The gps_position_t struct is filled with lat/lon in decimal degrees.
bool time_sync_get_position(gps_position_t *pos) {
    if (!pos)
        return false;
    // Only meaningful in GPS mode
    if (_source != TIME_SYNC_GPS) {
        pos->valid = false;
        pos->latitude_deg = 0.0;
        pos->longitude_deg = 0.0;
        return false;
    }
    pos->valid = _gps_pos_valid;
    pos->latitude_deg = _gps_lat;
    pos->longitude_deg = _gps_lon;
    return _gps_pos_valid;
}
