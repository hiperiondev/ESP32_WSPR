/**
 * @file gpio_filter.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <assert.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "gpio_filter.h"

static_assert(CONFIG_FILTER_GPIO_A != CONFIG_FILTER_GPIO_B, "FILTER_GPIO_A and FILTER_GPIO_B are the same pin — fix in menuconfig");
static_assert(CONFIG_FILTER_GPIO_A != CONFIG_FILTER_GPIO_C, "FILTER_GPIO_A and FILTER_GPIO_C are the same pin — fix in menuconfig");
static_assert(CONFIG_FILTER_GPIO_B != CONFIG_FILTER_GPIO_C, "FILTER_GPIO_B and FILTER_GPIO_C are the same pin — fix in menuconfig");

static const char *TAG = "gpio_filter";

#define GPIO_A CONFIG_FILTER_GPIO_A
#define GPIO_B CONFIG_FILTER_GPIO_B
#define GPIO_C CONFIG_FILTER_GPIO_C

esp_err_t gpio_filter_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = BIT64(GPIO_A) | BIT64(GPIO_B) | BIT64(GPIO_C),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err == ESP_OK)
        gpio_filter_select(0);
    ESP_LOGI(TAG, "Filter GPIOs: A=%d B=%d C=%d", GPIO_A, GPIO_B, GPIO_C);
    return err;
}

esp_err_t gpio_filter_select(uint8_t filter_id) {
    filter_id &= 0x07;
    gpio_set_level(GPIO_A, (filter_id >> 0) & 1);
    gpio_set_level(GPIO_B, (filter_id >> 1) & 1);
    gpio_set_level(GPIO_C, (filter_id >> 2) & 1);
    ESP_LOGD(TAG, "Filter selected: %d (A=%d B=%d C=%d)", filter_id, (filter_id >> 0) & 1, (filter_id >> 1) & 1, (filter_id >> 2) & 1);
    return ESP_OK;
}
