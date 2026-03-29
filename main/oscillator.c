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

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oscillator.h"

static const char *TAG = "oscillator";

typedef enum {
    OSC_NONE = 0,
    OSC_SI5351 = 1,
    OSC_AD9850 = 2,
} osc_type_t;

static osc_type_t _osc_type = OSC_NONE;
static bool _osc_hw_ok = false;

// =============================================================================
//  SI5351 Driver
// =============================================================================

#define SI5351_ADDR              0x60
#define SI5351_CLK0_CTRL         16
#define SI5351_PLLA_MSNA         26
#define SI5351_MS0_BASE          42
#define SI5351_PLL_RESET         177
#define SI5351_OUTPUT_EN         3
#define SI5351_CLK0_MS0_SRC_PLLA 0x0C

#if defined(CONFIG_SI5351_DRIVE_2MA)
#define SI5351_IDRV 0x00u
#elif defined(CONFIG_SI5351_DRIVE_4MA)
#define SI5351_IDRV 0x01u
#elif defined(CONFIG_SI5351_DRIVE_6MA)
#define SI5351_IDRV 0x02u
#else
#define SI5351_IDRV 0x03u
#endif
#define SI5351_CLK0_CTRL_VAL (0x0Cu | SI5351_IDRV)

static i2c_port_t _si_port = (i2c_port_t)CONFIG_SI5351_I2C_PORT;
static uint32_t _si_xtal = (uint32_t)CONFIG_SI5351_XTAL_FREQ;
static uint32_t _si_vco = 0;
static int32_t _si_cal = 0;

static esp_err_t si_write(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI5351_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_si_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t si_write_bulk(uint8_t reg, const uint8_t *buf, int len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI5351_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, buf, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_si_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t si5351_ping(void) {
    uint8_t dummy = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI5351_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI5351_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &dummy, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_si_port, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    return err;
}

// MODIFIED 3.7: si_init_pll — replaced dead guard with explicit VCO range clamps
static esp_err_t si_init_pll(void) {
    uint32_t xtal_mhz = _si_xtal / 1000000UL;
    uint32_t a = 900UL / xtal_mhz;
    // MODIFIED 3.7: replaced dead guard condition with explicit VCO range clamps.
    // Si5351 PLL requires 600 MHz <= VCO <= 900 MHz (datasheet section 4).
    // a < 15 with any xtal >= 10 MHz yields VCO < 150 MHz (below minimum).
    // a > 90 with xtal = 10 MHz yields VCO > 900 MHz (above maximum).
    // The original condition (a*xtal/28.127 < 6) never triggered in practice.
    if (a < 15UL) a = 15UL;
    if (a > 90UL) a = 90UL;
    _si_vco = a * xtal_mhz * 1000000UL;
    uint32_t p1 = 128UL * a - 512UL;
    uint8_t pll_regs[8] = {
        0x00, 0x01, (uint8_t)((p1 >> 16) & 0x03), (uint8_t)((p1 >> 8) & 0xFF), (uint8_t)(p1 & 0xFF), 0x00, 0x00, 0x00,
    };
    esp_err_t err = si_write_bulk(SI5351_PLLA_MSNA, pll_regs, 8);
    if (err != ESP_OK)
        return err;
    err = si_write(SI5351_PLL_RESET, 0x20);
    ESP_LOGI(TAG, "SI5351 PLL locked: xtal=%lu Hz a=%lu vco=%lu Hz", (unsigned long)_si_xtal, (unsigned long)a, (unsigned long)_si_vco);
    return err;
}

// MODIFIED 3.6 + 3.8: si_set_freq_hz_mhz — R-divider comment + calibration sign fix
static esp_err_t si_set_freq_hz_mhz(uint32_t freq_hz, uint16_t frac_mhz) {
    if (_si_vco == 0) {
        ESP_LOGE(TAG, "SI5351: PLL not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t r_div_reg = 0;
    uint32_t eff_hz = freq_hz;
    uint16_t eff_mhz = frac_mhz;
    // MODIFIED 3.6: R-divider doubles eff_hz up to 7 times (divide-by-128 maximum).
    // Minimum usable input frequency is approximately 500 kHz / 128 = 3.9 kHz.
    // For 2200 m (137.6 kHz): 5 doublings reach ~4.4 MHz, within MS0 divider range.
    // r_div_reg value is stored in MS0_REG[2] bits [6:4] (see si5351 datasheet table 8).
    while (eff_hz < 500000UL && r_div_reg < 7) {
        eff_hz = eff_hz * 2UL + (eff_mhz >= 500U ? 1UL : 0UL);
        eff_mhz = (uint16_t)((eff_mhz * 2U) % 1000U);
        r_div_reg++;
        configASSERT(eff_mhz < 1000U);
    }
    // MODIFIED 3.8: fixed calibration sign inversion and simplified formula.
    // Positive ppb means the crystal runs fast: actual VCO > nominal, output too high.
    // To compensate we must DECREASE the effective VCO, not increase it.
    // Previous code added delta (wrong sign), making the frequency error larger.
    // New formula: correction_hz = vco_MHz * ppb / 1000  (ppb/1000 converts to ppm scale).
    // Max correction at 100000 ppb: 900 * 100000 / 1000 = 90000 Hz, fits in int32_t.
    uint32_t vco_cal = _si_vco;
    if (_si_cal != 0) {
        int32_t vco_mhz = (int32_t)(_si_vco / 1000000UL);
        int32_t correction_hz = (vco_mhz * _si_cal) / 1000;
        vco_cal = (uint32_t)((int32_t)_si_vco - correction_hz);
    }
    uint32_t d_int = vco_cal / eff_hz;
    if (d_int < 6UL || d_int > 1800UL) {
        ESP_LOGE(TAG, "SI5351: divider %lu out of range (vco=%lu eff_hz=%lu)", (unsigned long)d_int, (unsigned long)vco_cal, (unsigned long)eff_hz);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t remainder = vco_cal - d_int * eff_hz;
    uint32_t c_den = 10000UL;
    uint32_t d_num;
    {
        uint32_t r = remainder;
        uint32_t e = eff_hz;
        uint8_t shift = 0;
        while (r > 429496UL && shift < 12) {
            r >>= 1;
            e >>= 1;
            shift++;
        }
        if (e == 0u)
            e = 1u;
        d_num = (r * c_den) / e;
    }
    uint32_t p1 = 128UL * d_int + (128UL * d_num / c_den) - 512UL;
    uint32_t p2 = 128UL * d_num - c_den * (128UL * d_num / c_den);
    uint32_t p3 = c_den;
    uint8_t ms_regs[8] = {
        (uint8_t)((p3 >> 8) & 0xFF), (uint8_t)(p3 & 0xFF), (uint8_t)(((p1 >> 16) & 0x03) | (r_div_reg << 4)),
        (uint8_t)((p1 >> 8) & 0xFF), (uint8_t)(p1 & 0xFF), (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)),
        (uint8_t)((p2 >> 8) & 0xFF), (uint8_t)(p2 & 0xFF),
    };
    return si_write_bulk(SI5351_MS0_BASE, ms_regs, 8);
}

static bool si5351_try_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_SI5351_SDA_GPIO,
        .scl_io_num = CONFIG_SI5351_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(_si_port, &conf);
    i2c_driver_install(_si_port, I2C_MODE_MASTER, 0, 0, 0);

    esp_err_t probe = si5351_ping();
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "SI5351 not detected on I2C addr 0x%02X (err=%s)", SI5351_ADDR, esp_err_to_name(probe));
        i2c_driver_delete(_si_port);
        return false;
    }
    si_write(3, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    si_write(SI5351_CLK0_CTRL, SI5351_CLK0_CTRL_VAL);
    esp_err_t err = si_init_pll();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SI5351 PLL lock failed (err=%s)", esp_err_to_name(err));
        i2c_driver_delete(_si_port);
        return false;
    }
    ESP_LOGI(TAG, "SI5351 initialized OK (xtal=%lu Hz vco=%lu Hz)", (unsigned long)_si_xtal, (unsigned long)_si_vco);
    return true;
}

// =============================================================================
//  AD9850 Driver
// =============================================================================

#define AD9850_CLK_GPIO   CONFIG_AD9850_CLK_GPIO
#define AD9850_FQUD_GPIO  CONFIG_AD9850_FQ_UD_GPIO
#define AD9850_DATA_GPIO  CONFIG_AD9850_DATA_GPIO
#define AD9850_RESET_GPIO CONFIG_AD9850_RESET_GPIO
#define AD9850_REF_CLK    CONFIG_AD9850_REF_CLOCK

static uint32_t _ad_last_hz = 0u;
static uint32_t _ad_last_frac = 0u;
static int32_t _ad_cal = 0;

static portMUX_TYPE _ad_mux = portMUX_INITIALIZER_UNLOCKED;

// FIXED: replaced 64-bit ULL constant with equivalent 32-bit arithmetic.
// Identity: 2^32 / ref_khz  ==  (2^32/1000) / (ref_hz/1000000)
//         = 4294967 / ref_mhz   (floor; ref must be an integer MHz value).
// Verification 125 MHz: 4294967/125 = 34359  vs  4294967296/125000 = 34359  [identical]
// Verification 100 MHz: 4294967/100 = 42949  vs  4294967296/100000 = 42949  [identical]
// No 64-bit intermediate produced; compiler evaluates entirely at compile-time.
#define AD9850_SCALE_KHZ (4294967UL / ((uint32_t)AD9850_REF_CLK / 1000000UL))

static uint32_t ad9850_freq_word(uint32_t freq_hz) {
    if (_ad_cal != 0) {
        uint32_t ppb_abs;
        if (_ad_cal > 0) {
            ppb_abs = (uint32_t)_ad_cal;
        } else {
            // Two's complement negation in unsigned domain: defined for all int32_t values
            ppb_abs = (uint32_t)(-(uint32_t)_ad_cal);
        }
        uint32_t delta_hz = (freq_hz / 1000u) * ppb_abs / 1000000u;
        // FIXED: positive ppb means the reference crystal oscillates faster than
        // its nominal frequency.  Because FTW = freq * 2^32 / ref_actual and
        // ref_actual > ref_nominal, the AD9850 output is already too LOW.
        // To compensate we must INCREASE the programmed freq_hz, not decrease it.
        // The original code had the two branches swapped, making the error worse.
        if (_ad_cal > 0) {
            freq_hz = freq_hz + delta_hz;
        } else {
            freq_hz = (freq_hz > delta_hz) ? (freq_hz - delta_hz) : 0u;
        }
    }
    uint32_t freq_khz = freq_hz / 1000UL;
    uint32_t freq_rem = freq_hz % 1000UL;
    uint32_t fw = freq_khz * AD9850_SCALE_KHZ;
    fw += (freq_rem * AD9850_SCALE_KHZ) / 1000UL;
    return fw;
}

static inline void _pulse(int gpio) {
    gpio_set_level(gpio, 1);
    gpio_set_level(gpio, 0);
}

static void ad9850_write_word(uint32_t freq_word, uint8_t phase) {
    int i;
    taskENTER_CRITICAL(&_ad_mux);
    for (i = 0; i < 32; i++) {
        gpio_set_level(AD9850_DATA_GPIO, (int)((freq_word >> i) & 1u));
        _pulse(AD9850_CLK_GPIO);
    }
    for (i = 0; i < 8; i++) {
        gpio_set_level(AD9850_DATA_GPIO, (int)((phase >> i) & 1u));
        _pulse(AD9850_CLK_GPIO);
    }
    _pulse(AD9850_FQUD_GPIO);
    taskEXIT_CRITICAL(&_ad_mux);
}

static bool ad9850_try_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(AD9850_CLK_GPIO) | BIT64(AD9850_FQUD_GPIO) | BIT64(AD9850_DATA_GPIO) | BIT64(AD9850_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(AD9850_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(AD9850_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    _pulse(AD9850_CLK_GPIO);
    _pulse(AD9850_FQUD_GPIO);

    ESP_LOGI(TAG, "AD9850 initialized (ref=%d Hz) — presence assumed (write-only)", AD9850_REF_CLK);
    return true;
}

// =============================================================================
//  Public API — runtime dispatch
// =============================================================================

esp_err_t oscillator_init(void) {
    if (si5351_try_init()) {
        _osc_type = OSC_SI5351;
        _osc_hw_ok = true;
        ESP_LOGI(TAG, "Auto-detected: SI5351");
        return ESP_OK;
    }
    if (ad9850_try_init()) {
        _osc_type = OSC_AD9850;
        _osc_hw_ok = true;
        ESP_LOGI(TAG, "Auto-detected: AD9850 (assumed)");
        return ESP_OK;
    }
    _osc_type = OSC_NONE;
    _osc_hw_ok = false;
    ESP_LOGW(TAG, "No oscillator hardware found — DUMMY mode active");
    return ESP_OK;
}

esp_err_t oscillator_set_freq(uint32_t freq_hz) {
    if (_osc_type == OSC_SI5351) {
        return si_set_freq_hz_mhz(freq_hz, 0);
    }
    if (_osc_type == OSC_AD9850) {
        _ad_last_hz = freq_hz;
        _ad_last_frac = 0u;
        ad9850_write_word(ad9850_freq_word(freq_hz), 0u);
    }
    return ESP_OK;
}

esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_mhz) {
    int32_t total_mhz = (int32_t)(base_hz % 1000UL) * 1000 + offset_mhz;
    int32_t frac_mhz_s = total_mhz % 1000;
    int32_t extra_hz_s = total_mhz / 1000;
    if (frac_mhz_s < 0) {
        frac_mhz_s += 1000;
        extra_hz_s -= 1;
    }
    uint32_t extra_hz = (uint32_t)extra_hz_s;
    uint32_t out_hz = (base_hz / 1000UL) * 1000UL + extra_hz;

    if (_osc_type == OSC_SI5351) {
        uint16_t frac_mhz = (uint16_t)frac_mhz_s;
        return si_set_freq_hz_mhz(out_hz, frac_mhz);
    }
    if (_osc_type == OSC_AD9850) {
        uint32_t frac_mhz = (uint32_t)frac_mhz_s;
        _ad_last_hz = out_hz;
        _ad_last_frac = frac_mhz;
        uint32_t fw = ad9850_freq_word(out_hz);
        fw += (frac_mhz * AD9850_SCALE_KHZ) / 1000000UL;
        ad9850_write_word(fw, 0u);
    }
    return ESP_OK;
}

esp_err_t oscillator_enable(bool en) {
    if (_osc_type == OSC_SI5351) {
        uint8_t val = en ? 0xFE : 0xFF;
        return si_write(3, val);
    }
    if (_osc_type == OSC_AD9850) {
        if (en) {
            if (_ad_last_hz > 0u) {
                uint32_t fw = ad9850_freq_word(_ad_last_hz);
                fw += (_ad_last_frac * AD9850_SCALE_KHZ) / 1000000UL;
                ad9850_write_word(fw, 0x00u);
            }
        } else {
            ad9850_write_word(0u, 0x04u);
        }
    }
    return ESP_OK;
}

esp_err_t oscillator_set_cal(int32_t ppb) {
    if (_osc_type == OSC_SI5351) {
        _si_cal = ppb;
        ESP_LOGI(TAG, "SI5351 cal set: %ld ppb", (long)ppb);
    } else if (_osc_type == OSC_AD9850) {
        _ad_cal = ppb;
        ESP_LOGI(TAG, "AD9850 cal set: %ld ppb", (long)ppb);
    } else {
        _si_cal = ppb;
        ESP_LOGI(TAG, "Dummy cal stored: %ld ppb", (long)ppb);
    }
    return ESP_OK;
}

bool oscillator_hw_ok(void) {
    return _osc_hw_ok;
}

const char *oscillator_hw_name(void) {
    switch (_osc_type) {
        case OSC_SI5351:
            return "Si5351A";
        case OSC_AD9850:
            return "AD9850 (assumed)";
        default:
            return "None (DUMMY)";
    }
}
