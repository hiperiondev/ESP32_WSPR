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

// Compile-time guard: all three address pins must be distinct to avoid bus conflicts
static_assert(CONFIG_FILTER_GPIO_A != CONFIG_FILTER_GPIO_B, "FILTER_GPIO_A and FILTER_GPIO_B are the same pin — fix in menuconfig");
static_assert(CONFIG_FILTER_GPIO_A != CONFIG_FILTER_GPIO_C, "FILTER_GPIO_A and FILTER_GPIO_C are the same pin — fix in menuconfig");
static_assert(CONFIG_FILTER_GPIO_B != CONFIG_FILTER_GPIO_C, "FILTER_GPIO_B and FILTER_GPIO_C are the same pin — fix in menuconfig");

static const char *TAG = "gpio_filter";

// Local aliases for the three GPIO address-bus pins
#define GPIO_A CONFIG_FILTER_GPIO_A // address bit 0 (LSB)
#define GPIO_B CONFIG_FILTER_GPIO_B // address bit 1
#define GPIO_C CONFIG_FILTER_GPIO_C // address bit 2 (MSB)

esp_err_t gpio_filter_init(void) {
    // Configure all three LPF-select pins as push-pull outputs, no pull resistors
    gpio_config_t io = {
        .pin_bit_mask = BIT64(GPIO_A) | BIT64(GPIO_B) | BIT64(GPIO_C),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    // Drive all address lines low immediately: relay board starts in state 0 (filter 0)
    if (err == ESP_OK)
        gpio_filter_select(0);
    ESP_LOGI(TAG, "Filter GPIOs: A=%d B=%d C=%d", GPIO_A, GPIO_B, GPIO_C);
    return err;
}

esp_err_t gpio_filter_select(uint8_t filter_id) {
    // Mask to 3 bits; bits above bit 2 are silently ignored
    filter_id &= 0x07;

    // Decompose the 3-bit address and write each bit to its GPIO.
    // The external relay/switch decoder (e.g. 74HC138 or CD4051) converts
    // this binary address into a one-of-eight selection of the LPF section.
    // After this call the caller must wait CONFIG_WSPR_LPF_SETTLE_MS for the
    // relay contacts to close before enabling the oscillator RF output.
    gpio_set_level(GPIO_A, (filter_id >> 0) & 1);
    gpio_set_level(GPIO_B, (filter_id >> 1) & 1);
    gpio_set_level(GPIO_C, (filter_id >> 2) & 1);
    ESP_LOGD(TAG, "Filter selected: %d (A=%d B=%d C=%d)", filter_id, (filter_id >> 0) & 1, (filter_id >> 1) & 1, (filter_id >> 2) & 1);
    return ESP_OK;
}
