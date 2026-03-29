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
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "wifi_manager.h"

static const char *TAG = "wifi";

#define STA_CONNECTED_BIT BIT0
#define STA_FAIL_BIT      BIT1
#define MAX_RETRY         5

static EventGroupHandle_t _eg;
static bool _sta_ok = false;
static char _ip_str[20] = "0.0.0.0";
static int _retry = 0;
static char _sta_ssid_saved[33] = { 0 };
static char _sta_pass_saved[65] = { 0 };
static esp_timer_handle_t _sta_retry_timer = NULL;

static void try_sta_reconnect(void *arg) {
    if (_sta_ok)
        return;
    if (_sta_ssid_saved[0] == '\0')
        return;
    ESP_LOGI(TAG, "Periodic STA reconnect attempt (SSID=%s)", _sta_ssid_saved);
    _retry = 0;
    wifi_config_t sta_cfg = { 0 };
    strncpy((char *)sta_cfg.sta.ssid, _sta_ssid_saved, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, _sta_pass_saved, sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK)
        return;
    if (esp_wifi_set_config(WIFI_IF_STA, &sta_cfg) != ESP_OK)
        return;
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            _sta_ok = false;
            if (_retry++ < MAX_RETRY) {
                ESP_LOGI(TAG, "Retry STA connection (%d/%d)", _retry, MAX_RETRY);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(_eg, STA_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(_ip_str, sizeof(_ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "STA got IP: %s", _ip_str);
        _retry = 0;
        _sta_ok = true;
        xEventGroupSetBits(_eg, STA_CONNECTED_BIT);

        if (_sta_retry_timer != NULL) {
            esp_timer_stop(_sta_retry_timer);
        }
    }
}

static void start_ap(void) {
    ESP_LOGI(TAG, "Starting SoftAP: %s", CONFIG_WSPR_AP_SSID);
    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, CONFIG_WSPR_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(CONFIG_WSPR_AP_SSID);

    if (strlen(CONFIG_WSPR_AP_PASS) >= 8) {
        strncpy((char *)ap_cfg.ap.password, CONFIG_WSPR_AP_PASS, sizeof(ap_cfg.ap.password));
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        if (strlen(CONFIG_WSPR_AP_PASS) > 0) {
            ESP_LOGW(TAG,
                     "AP password '%s' is shorter than 8 chars; "
                     "starting open network instead",
                     CONFIG_WSPR_AP_PASS);
        }
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.channel = 1;

    esp_err_t err;
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start() AP mode failed: %s", esp_err_to_name(err));
        return;
    }

    snprintf(_ip_str, sizeof(_ip_str), "192.168.4.1");
    ESP_LOGI(TAG, "AP started — connect to http://192.168.4.1");

    if (_sta_retry_timer == NULL && _sta_ssid_saved[0] != '\0') {
        const esp_timer_create_args_t ta = {
            .callback = try_sta_reconnect,
            .arg = NULL,
            .name = "sta_retry",
        };
        if (esp_timer_create(&ta, &_sta_retry_timer) == ESP_OK) {
            esp_timer_start_periodic(_sta_retry_timer, 5ULL * 60ULL * 1000000ULL);
            ESP_LOGI(TAG, "STA reconnect timer started (5 min interval)");
        } else {
            ESP_LOGW(TAG, "Failed to create STA reconnect timer");
        }
    }
}

esp_err_t wifi_manager_start(const char *sta_ssid, const char *sta_pass) {
    _eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t loop_err = esp_event_loop_create_default();
    if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(loop_err));
        return loop_err;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA netif");
        return ESP_FAIL;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (sta_ssid && strlen(sta_ssid) > 0) {
        strncpy(_sta_ssid_saved, sta_ssid, sizeof(_sta_ssid_saved) - 1);
        _sta_ssid_saved[sizeof(_sta_ssid_saved) - 1] = '\0';
        strncpy(_sta_pass_saved, sta_pass ? sta_pass : "", sizeof(_sta_pass_saved) - 1);
        _sta_pass_saved[sizeof(_sta_pass_saved) - 1] = '\0';
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    if (sta_ssid && strlen(sta_ssid) > 0) {
        wifi_config_t sta_cfg = { 0 };
        strncpy((char *)sta_cfg.sta.ssid, sta_ssid, sizeof(sta_cfg.sta.ssid));
        strncpy((char *)sta_cfg.sta.password, sta_pass ? sta_pass : "", sizeof(sta_cfg.sta.password));
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        esp_wifi_start();

        ESP_LOGI(TAG, "Connecting to STA: %s", sta_ssid);
        EventBits_t bits = xEventGroupWaitBits(_eg, STA_CONNECTED_BIT | STA_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

        if (bits & STA_CONNECTED_BIT) {
            ESP_LOGI(TAG, "STA connected");
            return ESP_OK;
        }
        ESP_LOGW(TAG, "STA failed, starting AP");
        esp_wifi_stop();
    }

    start_ap();
    return ESP_OK;
}

bool wifi_manager_sta_connected(void) {
    return _sta_ok;
}

const char *wifi_manager_ip(void) {
    return _ip_str;
}

char *wifi_manager_scan(void) {
    // Determine current mode so we can restore it after the scan.
    wifi_mode_t mode_before = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode_before);

    // The scan STA interface needs to be up.  In AP-only mode, elevate to APSTA.
    bool elevated = false;
    if (mode_before == WIFI_MODE_AP) {
        if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK) {
            ESP_LOGW(TAG, "scan: could not switch to APSTA — returning empty list");
            // Return an empty JSON array so the UI does not break.
            char *empty = malloc(3);
            if (empty) {
                empty[0] = '[';
                empty[1] = ']';
                empty[2] = '\0';
            }
            return empty;
        }
        // delay after mode switch to APSTA so the STA
        // interface PHY has time to initialise before the scan starts.
        // Without this delay, esp_wifi_scan_start() runs before the STA
        // radio is ready and consistently returns 0 results.
        vTaskDelay(pdMS_TO_TICKS(150));
        elevated = true;
    }

    // Blocking scan: scan_config all-zero uses defaults (all channels, active).
    wifi_scan_config_t scan_cfg = { 0 };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true); // true = blocking
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan: esp_wifi_scan_start failed: %s", esp_err_to_name(err));
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }
        char *empty = malloc(3);
        if (empty) {
            empty[0] = '[';
            empty[1] = ']';
            empty[2] = '\0';
        }
        return empty;
    }

    // Retrieve the number of discovered APs.
    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }
        char *empty = malloc(3);
        if (empty) {
            empty[0] = '[';
            empty[1] = ']';
            empty[2] = '\0';
        }
        return empty;
    }

    // Cap at 20 results to keep the JSON payload small enough for the HTTP stack.
    if (count > 20) {
        count = 20;
    }

    wifi_ap_record_t *records = malloc(sizeof(wifi_ap_record_t) * count);
    if (!records) {
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }
        return NULL;
    }

    esp_wifi_scan_get_ap_records(&count, records);

    // Restore the original WiFi mode before building the response.
    if (elevated) {
        esp_wifi_set_mode(WIFI_MODE_AP);
    }

    // Each entry worst case: {"ssid":"<32 chars>","rssi":-100,"auth":1}
    // = ~60 bytes + 2 for brackets + count-1 commas + NUL.
    size_t buf_sz = 4 + (size_t)count * 80;
    char *buf = malloc(buf_sz);
    if (!buf) {
        free(records);
        return NULL;
    }

    size_t pos = 0;
    buf[pos++] = '[';

    for (uint16_t i = 0; i < count; i++) {
        // Escape any double-quote or backslash in the SSID to produce valid JSON.
        char ssid_safe[96] = { 0 };
        size_t sp = 0;
        for (int k = 0; k < 32 && records[i].ssid[k] != '\0' && sp < sizeof(ssid_safe) - 3; k++) {
            unsigned char c = (unsigned char)records[i].ssid[k];
            if (c == '"' || c == '\\') {
                ssid_safe[sp++] = '\\';
            }
            ssid_safe[sp++] = (char)c;
        }
        ssid_safe[sp] = '\0';

        // auth: 0 = open (WIFI_AUTH_OPEN), 1 = any form of security.
        int auth = (records[i].authmode == WIFI_AUTH_OPEN) ? 0 : 1;

        int written =
            snprintf(buf + pos, buf_sz - pos, "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}", (i > 0) ? "," : "", ssid_safe, (int)records[i].rssi, auth);
        if (written < 0 || (size_t)written >= buf_sz - pos) {
            // Buffer too small: stop here; the array is still valid JSON.
            break;
        }
        pos += (size_t)written;
    }

    if (pos < buf_sz) {
        buf[pos++] = ']';
    }
    if (pos < buf_sz) {
        buf[pos] = '\0';
    } else {
        buf[buf_sz - 1] = '\0';
    }

    free(records);
    ESP_LOGI(TAG, "WiFi scan complete: %u APs found", (unsigned)count);
    return buf;
}
