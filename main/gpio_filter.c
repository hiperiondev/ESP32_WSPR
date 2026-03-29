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

#include <assert.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "gpio_filter.h"

// Guard the three filter pins against each other: wiring two address
// lines to the same GPIO turns the BCD decoder into a short circuit.
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
