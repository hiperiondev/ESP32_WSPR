/**
 * @file wifi_manager.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @brief Wi-Fi connection manager with STA mode and soft-AP fallback.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
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

// Event group bits used to signal STA connection outcome to wifi_manager_start()
#define STA_CONNECTED_BIT BIT0 // set by IP_EVENT_STA_GOT_IP handler
#define STA_FAIL_BIT      BIT1 // set after MAX_RETRY consecutive association failures

// Maximum number of consecutive STA connection attempts before falling back to AP mode
#define MAX_RETRY 5

// FreeRTOS event group used by wifi_manager_start() to wait for STA outcome
static EventGroupHandle_t _eg;

// True when the STA interface has an active DHCP lease
static bool _sta_ok = false;

// Current device IP address as a printable string (STA or soft-AP)
static char _ip_str[20] = "0.0.0.0";

// Consecutive STA association attempt counter; reset on success
static int _retry = 0;

// Saved STA credentials used by the background reconnect timer
static char _sta_ssid_saved[33] = { 0 };
static char _sta_pass_saved[65] = { 0 };

// Periodic timer that retries STA connection every 5 minutes while in AP-only mode
static esp_timer_handle_t _sta_retry_timer = NULL;

// Mutex that prevents concurrent calls to wifi_manager_scan()
static SemaphoreHandle_t _scan_mutex = NULL;

/**
 * @brief Periodic timer callback — retry STA connection while in soft-AP fallback mode.
 *
 * Fires every 5 minutes when the device is in soft-AP mode with saved STA
 * credentials.  Elevates the Wi-Fi mode to APSTA and calls esp_wifi_connect()
 * so the device can reconnect if the home access point comes back online.
 * The soft-AP remains active throughout so connected clients are not dropped.
 * Once a DHCP lease is obtained (_sta_ok becomes true) the event handler stops
 * this timer.
 *
 * @param[in] arg  Unused (required by esp_timer callback signature).
 */
static void try_sta_reconnect(void *arg) {
    // Do nothing if already connected or if no credentials are saved
    if (_sta_ok)
        return;
    if (_sta_ssid_saved[0] == '\0')
        return;
    ESP_LOGI(TAG, "Periodic STA reconnect attempt (SSID=%s)", _sta_ssid_saved);
    // Reset the retry counter so the event handler will make a full set of attempts
    _retry = 0;
    wifi_config_t sta_cfg = { 0 };
    strncpy((char *)sta_cfg.sta.ssid, _sta_ssid_saved, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, _sta_pass_saved, sizeof(sta_cfg.sta.password) - 1);
    // Accept open and secured networks
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    // Elevate to APSTA so the STA radio is active alongside the soft-AP
    if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK)
        return;
    if (esp_wifi_set_config(WIFI_IF_STA, &sta_cfg) != ESP_OK)
        return;
    esp_wifi_connect();
}

/**
 * @brief ESP-IDF Wi-Fi and IP event handler.
 *
 * Handles three events:
 *  - WIFI_EVENT_STA_START: immediately attempt the first association.
 *  - WIFI_EVENT_STA_DISCONNECTED: retry up to MAX_RETRY times, then set STA_FAIL_BIT.
 *  - IP_EVENT_STA_GOT_IP: record the assigned IP, set _sta_ok, signal STA_CONNECTED_BIT.
 *
 * This handler runs in the ESP-IDF system event-loop task (not in any application
 * task), so it must not block or call non-ISR-safe APIs.
 *
 * @param[in] arg   Unused user context pointer.
 * @param[in] base  Event base identifier (WIFI_EVENT or IP_EVENT).
 * @param[in] id    Event ID within the base.
 * @param[in] data  Event-specific data pointer (e.g. ip_event_got_ip_t* for IP events).
 */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            // Trigger the first association attempt as soon as the STA interface is ready
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            _sta_ok = false;
            if (_retry++ < MAX_RETRY) {
                ESP_LOGI(TAG, "Retry STA connection (%d/%d)", _retry, MAX_RETRY);
                esp_wifi_connect();
            } else {
                // All retries exhausted — signal the caller to fall back to soft-AP
                xEventGroupSetBits(_eg, STA_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // DHCP lease obtained — record the IP address and mark the STA as connected
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(_ip_str, sizeof(_ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "STA got IP: %s", _ip_str);
        _retry = 0;
        _sta_ok = true;
        // Unblock wifi_manager_start() which is waiting on this bit
        xEventGroupSetBits(_eg, STA_CONNECTED_BIT);

        // Stop the background reconnect timer — connection is now established
        if (_sta_retry_timer != NULL) {
            esp_timer_stop(_sta_retry_timer);
        }
    }
}

/**
 * @brief Configure and start the ESP32 soft-AP (access point) interface.
 *
 * Reads the AP SSID and password from menuconfig
 * (CONFIG_WSPR_AP_SSID / CONFIG_WSPR_AP_PASS).  If the password is shorter
 * than 8 characters (WPA2 minimum), the AP is started as an open network and
 * a warning is logged.  The static IP assigned to the AP interface by the
 * ESP-IDF netif layer is always 192.168.4.1.
 *
 * If saved STA credentials are available and the background reconnect timer has
 * not been created yet, a periodic esp_timer is created here with a 5-minute
 * interval so that the device automatically reconnects when the home AP returns.
 */
static void start_ap(void) {
    ESP_LOGI(TAG, "Starting SoftAP: %s", CONFIG_WSPR_AP_SSID);
    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, CONFIG_WSPR_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(CONFIG_WSPR_AP_SSID);

    // Require at least 8 characters for WPA2-PSK; shorter passwords open an unsecured AP
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

    // Allow up to 4 simultaneous client stations on the soft-AP
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

    // The ESP-IDF default AP netif always gets this static address
    snprintf(_ip_str, sizeof(_ip_str), "192.168.4.1");
    ESP_LOGI(TAG, "AP started — connect to http://192.168.4.1");

    // Start the background reconnect timer if saved STA credentials are available
    if (_sta_retry_timer == NULL && _sta_ssid_saved[0] != '\0') {
        const esp_timer_create_args_t ta = {
            .callback = try_sta_reconnect,
            .arg = NULL,
            .name = "sta_retry",
        };
        if (esp_timer_create(&ta, &_sta_retry_timer) == ESP_OK) {
            // 5-minute interval: 5 * 60 * 1000000 microseconds
            esp_timer_start_periodic(_sta_retry_timer, 5ULL * 60ULL * 1000000ULL);
            ESP_LOGI(TAG, "STA reconnect timer started (5 min interval)");
        } else {
            ESP_LOGW(TAG, "Failed to create STA reconnect timer");
        }
    }
}

esp_err_t wifi_manager_start(const char *sta_ssid, const char *sta_pass) {
    // Create the event group used to synchronise STA connection result
    _eg = xEventGroupCreate();

    // Initialise the scan mutex once; subsequent calls to wifi_manager_start() are safe
    if (_scan_mutex == NULL) {
        _scan_mutex = xSemaphoreCreateMutex();
    }

    // Initialise the LwIP TCP/IP stack (idempotent — safe to call multiple times)
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop; ignore ESP_ERR_INVALID_STATE if already created
    esp_err_t loop_err = esp_event_loop_create_default();
    if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(loop_err));
        return loop_err;
    }

    // Create the default STA and AP netif objects required by the ESP-IDF Wi-Fi driver
    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA netif");
        return ESP_FAIL;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
        return ESP_FAIL;
    }

    // Initialise the Wi-Fi driver with the ROM-default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Save STA credentials for use by the background reconnect timer
    if (sta_ssid && strlen(sta_ssid) > 0) {
        strncpy(_sta_ssid_saved, sta_ssid, sizeof(_sta_ssid_saved) - 1);
        _sta_ssid_saved[sizeof(_sta_ssid_saved) - 1] = '\0';
        strncpy(_sta_pass_saved, sta_pass ? sta_pass : "", sizeof(_sta_pass_saved) - 1);
        _sta_pass_saved[sizeof(_sta_pass_saved) - 1] = '\0';
    }

    // Register the combined event handler for WIFI_EVENT (all IDs) and IP_EVENT_STA_GOT_IP
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    // Attempt STA connection if credentials are provided
    if (sta_ssid && strlen(sta_ssid) > 0) {
        wifi_config_t sta_cfg = { 0 };
        strncpy((char *)sta_cfg.sta.ssid, sta_ssid, sizeof(sta_cfg.sta.ssid));
        strncpy((char *)sta_cfg.sta.password, sta_pass ? sta_pass : "", sizeof(sta_cfg.sta.password));
        // WIFI_AUTH_OPEN as threshold means the driver will also connect to secured networks
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        // Start the STA interface; WIFI_EVENT_STA_START fires and triggers esp_wifi_connect()
        esp_wifi_start();

        ESP_LOGI(TAG, "Connecting to STA: %s", sta_ssid);
        // Block for up to 15 s waiting for either a DHCP lease or all retries to fail
        EventBits_t bits = xEventGroupWaitBits(_eg, STA_CONNECTED_BIT | STA_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

        if (bits & STA_CONNECTED_BIT) {
            // STA connected and has a valid IP — NTP can now be started by the caller
            ESP_LOGI(TAG, "STA connected");
            return ESP_OK;
        }
        // STA failed or timed out — stop the STA interface before starting the AP
        ESP_LOGW(TAG, "STA failed, starting AP");
        esp_wifi_stop();
    }

    // Fallback: start the soft-AP (and optionally the background reconnect timer)
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
    // Acquire the scan mutex; reject concurrent callers after a 3 s wait
    if (_scan_mutex && xSemaphoreTake(_scan_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGW(TAG, "scan: concurrent call rejected (mutex timeout)");
        char *empty = malloc(3);
        if (empty) {
            empty[0] = '[';
            empty[1] = ']';
            empty[2] = '\0';
        }
        return empty;
    }

    // Remember the current Wi-Fi mode so it can be restored after the scan
    wifi_mode_t mode_before = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode_before);

    // The STA radio must be active to scan; elevate AP-only mode to APSTA temporarily
    bool elevated = false;
    if (mode_before == WIFI_MODE_AP) {
        if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK) {
            ESP_LOGW(TAG, "scan: could not switch to APSTA — returning empty list");
            char *empty = malloc(3);
            if (empty) {
                empty[0] = '[';
                empty[1] = ']';
                empty[2] = '\0';
            }

            if (_scan_mutex)
                xSemaphoreGive(_scan_mutex);
            return empty;
        }

        // Allow 300 ms for the STA interface to stabilise after mode switch
        vTaskDelay(pdMS_TO_TICKS(300));
        elevated = true;
    }

    // Perform a blocking passive channel scan; scan_cfg = {0} uses default settings
    wifi_scan_config_t scan_cfg = { 0 };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true); // true = blocking scan
    if (err != ESP_OK) {

        ESP_LOGW(TAG, "scan: start failed (%s) — likely STA interface not ready", esp_err_to_name(err));
        // Restore original mode if it was elevated
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }

        char *empty = malloc(3);
        if (empty) {
            empty[0] = '[';
            empty[1] = ']';
            empty[2] = '\0';
        }

        if (_scan_mutex)
            xSemaphoreGive(_scan_mutex);
        return empty;
    }

    // Retrieve the number of APs found by the scan
    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        ESP_LOGI(TAG, "scan: completed but no APs found");
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }

        char *empty = malloc(3);
        if (empty) {
            empty[0] = '[';
            empty[1] = ']';
            empty[2] = '\0';
        }

        if (_scan_mutex)
            xSemaphoreGive(_scan_mutex);

        return empty;
    }

    // Cap raw scan results to 30 to limit heap usage; visible list is further trimmed to 20
    if (count > 30) {
        count = 30;
    }

    wifi_ap_record_t *records = malloc(sizeof(wifi_ap_record_t) * count);
    if (!records) {
        if (elevated) {
            esp_wifi_set_mode(WIFI_MODE_AP);
        }

        if (_scan_mutex)
            xSemaphoreGive(_scan_mutex);
        return NULL;
    }

    // Retrieve the AP records from the driver's internal buffer
    esp_wifi_scan_get_ap_records(&count, records);

    // Filter out hidden networks (empty SSID byte at index 0) by compacting in-place
    uint16_t visible = 0;
    for (uint16_t fi = 0; fi < count; fi++) {
        if (records[fi].ssid[0] != '\0') {
            if (fi != visible)
                records[visible] = records[fi];
            visible++;
        }
    }
    // Limit the final list to 20 entries to keep the HTTP response payload small
    count = (visible > 20u) ? 20u : visible;

    // Restore original Wi-Fi mode before building the response (may take time)
    if (elevated) {
        esp_wifi_set_mode(WIFI_MODE_AP);
    }

    // Allocate the JSON output buffer: brackets + up to 20 objects of ~80 bytes each
    size_t buf_sz = 4 + (size_t)count * 80;
    char *buf = malloc(buf_sz);
    if (!buf) {
        free(records);
        if (_scan_mutex)
            xSemaphoreGive(_scan_mutex);

        return NULL;
    }

    // Build a JSON array: [{"ssid":"...","rssi":-NN,"auth":0/1}, ...]
    size_t pos = 0;
    buf[pos++] = '[';

    for (uint16_t i = 0; i < count; i++) {
        // JSON-escape the SSID: prefix '"' and '\' with a backslash
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

        // auth: 0 = open network, 1 = any secured mode (WEP/WPA/WPA2/WPA3)
        int auth = (records[i].authmode == WIFI_AUTH_OPEN) ? 0 : 1;

        int written =
            snprintf(buf + pos, buf_sz - pos, "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}", (i > 0) ? "," : "", ssid_safe, (int)records[i].rssi, auth);
        if (written < 0 || (size_t)written >= buf_sz - pos) {
            // Buffer overflow protection: stop appending and close the array
            break;
        }
        pos += (size_t)written;
    }

    // Close the JSON array and NUL-terminate
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

    // Release the scan mutex so subsequent calls can proceed
    if (_scan_mutex)
        xSemaphoreGive(_scan_mutex);

    return buf;
}
