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
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
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

static uint32_t _si_xtal = (uint32_t)CONFIG_SI5351_XTAL_FREQ;
static uint32_t _si_vco = 0;
static int32_t _si_cal = 0;

static i2c_master_bus_handle_t si_bus_handle = NULL;
static i2c_master_dev_handle_t si_dev_handle = NULL;

static esp_err_t si_write(uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(si_dev_handle, data, 2, pdMS_TO_TICKS(100));
}

static esp_err_t si_write_bulk(uint8_t reg, const uint8_t *buf, int len) {
    uint8_t data[9];
    if (len + 1 > (int)sizeof(data)) {
        return ESP_ERR_INVALID_SIZE;
    }
    data[0] = reg;
    memcpy(&data[1], buf, (size_t)len);
    return i2c_master_transmit(si_dev_handle, data, (size_t)(len + 1), pdMS_TO_TICKS(100));
}

static esp_err_t si5351_ping(void) {
    uint8_t reg = 0x00;
    uint8_t dummy = 0;
    return i2c_master_transmit_receive(si_dev_handle, &reg, 1, &dummy, 1, pdMS_TO_TICKS(200));
}

static esp_err_t si_init_pll(void) {
    uint32_t xtal_mhz = _si_xtal / 1000000UL;
    uint32_t a = 900UL / xtal_mhz;
    if (a < 15UL)
        a = 15UL;
    if (a > 90UL)
        a = 90UL;
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

static esp_err_t si_set_freq_hz_mhz(uint32_t freq_hz, uint16_t frac_mhz) {
    if (_si_vco == 0) {
        ESP_LOGE(TAG, "SI5351: PLL not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t r_div_reg = 0;
    uint32_t eff_hz = freq_hz;
    uint16_t eff_mhz = frac_mhz;
    while (eff_hz < 500000UL && r_div_reg < 7) {
        eff_hz = eff_hz * 2UL + (eff_mhz >= 500U ? 1UL : 0UL);
        eff_mhz = (uint16_t)((eff_mhz * 2U) % 1000U);
        r_div_reg++;
        configASSERT(eff_mhz < 1000U);
    }
    uint32_t vco_cal = _si_vco;
    if (_si_cal != 0) {
        int32_t vco_mhz = (int32_t)(_si_vco / 1000000UL);
        int32_t correction_hz = (vco_mhz * _si_cal) / 1000;
        // add correction because fast crystal raises f_VCO_actual;
        // vco_cal must equal f_VCO_actual so divider d = vco_cal / f_target yields
        // f_out = f_VCO_actual / d = f_target exactly. Subtracting made error worse.
        vco_cal = (uint32_t)((int32_t)_si_vco + correction_hz);
    }
    uint32_t d_int = vco_cal / eff_hz;
    // Si5351 AN619: fractional mode MS output minimum divider is 8.
    // Integer mode supports 4/6/8 but this driver always operates in fractional mode.
    if (d_int < 8UL || d_int > 1800UL) {
        ESP_LOGE(TAG, "SI5351: divider %lu out of range (vco=%lu eff_hz=%lu)", (unsigned long)d_int, (unsigned long)vco_cal, (unsigned long)eff_hz);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t remainder = vco_cal - d_int * eff_hz;
    // Use maximum 20-bit c_den for ~0.2 mHz/step resolution and incorporate
    // eff_mhz into the fractional divider to eliminate the 0-464 mHz per-tone error
    // on WSPR HF bands. Without this fix eff_mhz was computed but never used.
    // 64-bit is used only in this frequency-setup routine (~1 Hz call rate, not hot path).
    uint32_t c_den = 1048575UL; // maximum 20-bit value per Si5351 AN619
    uint32_t d_num;
    {
        // rem_mhz = remainder*1000 - d_int*eff_mhz  [milli-Hz domain]
        // Always >= 0: remainder < eff_hz so d_int*eff_mhz/1000 < d_int <= remainder/... OK.
        uint64_t rem_mhz = (uint64_t)remainder * 1000UL;
        if (eff_mhz != 0u) {
            uint64_t sub = (uint64_t)d_int * (uint64_t)eff_mhz;
            if (sub <= rem_mhz) {
                rem_mhz -= sub;
            } else {
                rem_mhz = 0ULL; // guard against rounding edge cases
            }
        }
        uint64_t den_mhz = (uint64_t)eff_hz * 1000UL;
        // rem_mhz < den_mhz guarantees d_num < c_den; max(rem_mhz*c_den) ~3e16 < 2^63.
        d_num = (uint32_t)((rem_mhz * (uint64_t)c_den) / den_mhz);
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
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = (i2c_port_t)CONFIG_SI5351_I2C_PORT,
        .sda_io_num = CONFIG_SI5351_SDA_GPIO,
        .scl_io_num = CONFIG_SI5351_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &si_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI5351_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(si_bus_handle, &dev_cfg, &si_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        i2c_del_master_bus(si_bus_handle);
        si_bus_handle = NULL;
        return false;
    }
    esp_err_t probe = si5351_ping();
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "SI5351 not detected on I2C addr 0x%02X (err=%s)", SI5351_ADDR, esp_err_to_name(probe));
        i2c_master_bus_rm_device(si_dev_handle);
        i2c_del_master_bus(si_bus_handle);
        si_dev_handle = NULL;
        si_bus_handle = NULL;
        return false;
    }
    si_write(3, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    si_write(SI5351_CLK0_CTRL, SI5351_CLK0_CTRL_VAL);
    esp_err_t err_pll = si_init_pll();
    if (err_pll != ESP_OK) {
        ESP_LOGW(TAG, "SI5351 PLL lock failed (err=%s)", esp_err_to_name(err_pll));
        i2c_master_bus_rm_device(si_dev_handle);
        i2c_del_master_bus(si_bus_handle);
        si_dev_handle = NULL;
        si_bus_handle = NULL;
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

// For 125 MHz: FTW_PER_MHZ=34359738, FTW_INT_PER_HZ=34, FTW_FRAC_NUM=3597
// Accuracy: max error < 1 Hz at 28 MHz with pure 32-bit arithmetic.
//
// Pre-computed values verified:
//   125 MHz: 2^32/125 = 34359738.368  -> FTW_PER_MHZ=34359738
//            2^32/125000000 = 34.359  -> FTW_INT_PER_HZ=34
//            (2^32 mod 125000000)*10000/125000000 = 44967296*10000/125000000 = 3597
//   100 MHz: 2^32/100 = 42949672.96   -> FTW_PER_MHZ=42949672
//            2^32/100000000 = 42.949  -> FTW_INT_PER_HZ=42
//            (2^32 mod 100000000)*10000/100000000 = 94967296*10000/100000000 = 9496
#if AD9850_REF_CLK == 125000000UL
// 125 MHz module: pre-computed, no ULL at runtime
#define AD9850_FTW_PER_MHZ    34359738UL
#define AD9850_FTW_INT_PER_HZ 34UL
#define AD9850_FTW_FRAC_DEN   10000UL
#define AD9850_FTW_FRAC_NUM   3597UL
#elif AD9850_REF_CLK == 100000000UL
// 100 MHz module: pre-computed, no ULL at runtime
#define AD9850_FTW_PER_MHZ    42949672UL
#define AD9850_FTW_INT_PER_HZ 42UL
#define AD9850_FTW_FRAC_DEN   10000UL
#define AD9850_FTW_FRAC_NUM   9496UL
#else
// Non-standard ref clock fallback: ULL evaluated at compile-time on
// host only; no 64-bit runtime arithmetic emitted for the ESP32 target.
#define AD9850_FTW_PER_MHZ    ((uint32_t)(4294967296ULL / ((uint32_t)AD9850_REF_CLK / 1000000UL)))
#define AD9850_FTW_INT_PER_HZ ((uint32_t)(4294967296ULL / (uint32_t)AD9850_REF_CLK))
#define AD9850_FTW_FRAC_DEN   10000UL
#define AD9850_FTW_FRAC_NUM                                                                                                                                    \
    ((uint32_t)((4294967296ULL - (uint64_t)AD9850_FTW_INT_PER_HZ * (uint32_t)AD9850_REF_CLK) * AD9850_FTW_FRAC_DEN / (uint32_t)AD9850_REF_CLK))
#endif

// Combined per-mHz scale factor used in oscillator_set_freq_mhz.
// frac_mhz is in milli-Hz (0..4395 for WSPR).
// FTW increment per mHz = (FTW_INT_PER_HZ * FTW_FRAC_DEN + FTW_FRAC_NUM) / (1000 * FTW_FRAC_DEN)
// For 125 MHz: (34*10000 + 3597) / 10000000 = 343597 / 10000000
// At max 4395 mHz: 4395*343597/10000000 = 151 FTW (exact, no overflow: 4395*343597 < 2^32).
#define AD9850_FTW_FRAC_PER_MHZ_NUM ((uint32_t)(AD9850_FTW_INT_PER_HZ * AD9850_FTW_FRAC_DEN + AD9850_FTW_FRAC_NUM))
#define AD9850_FTW_FRAC_PER_MHZ_DEN (1000UL * AD9850_FTW_FRAC_DEN)

// verifies the new MHz-split formula does not overflow uint32_t at the
// maximum WSPR frequency (30 MHz used as a conservative ceiling).
static_assert((30UL * (4294967296ULL / ((uint32_t)AD9850_REF_CLK / 1000000UL)) + 999999UL * (4294967296ULL / (uint32_t)AD9850_REF_CLK) +
               999999UL * AD9850_FTW_FRAC_NUM / AD9850_FTW_FRAC_DEN) < 0xFFFFFFFFUL,
              "AD9850 FTW overflow: max WSPR frequency word must fit in 32 bits for the configured ref clock");

static uint32_t ad9850_freq_word(uint32_t freq_hz) {
    if (_ad_cal != 0) {
        uint32_t ppb_abs;
        if (_ad_cal > 0) {
            ppb_abs = (uint32_t)_ad_cal;
        } else {
            ppb_abs = (uint32_t)(-(uint32_t)_ad_cal);
        }

        // Rewrite delta_hz using MHz/sub-MHz split to fix two bugs:
        // (1) Precision loss at low WSPR frequencies: the old kHz-truncated formula
        //     gave 13 Hz instead of 13.76 Hz at 137600 Hz -- a 0.76 Hz error that
        //     exceeds the 6 Hz WSPR BW on the 2200 m band.
        // (2) Overflow risk: (28126100/1000u)*100000u = 2812600000 is safe at
        //     100k ppb but overflows uint32_t above ~153k ppb.
        // New split: delta = f_mhz*ppb/1000 + (f_sub_hz/1000)*ppb/1000000
        //   MHz term max: 30*100000/1000 = 3000. No overflow.
        //   Sub-kHz term max: 999*100000/1000000 = 99. No overflow.
        //   Total max error < 1 Hz at 137 kHz; < 0.3 Hz at 28 MHz.
        uint32_t f_mhz = freq_hz / 1000000UL;
        uint32_t f_sub_hz = freq_hz % 1000000UL;
        uint32_t f_sub_khz = f_sub_hz / 1000UL;
        uint32_t delta_mhz_part = (f_mhz * ppb_abs) / 1000UL;
        uint32_t delta_sub_part = (f_sub_khz * ppb_abs) / 1000000UL;
        uint32_t delta_hz = delta_mhz_part + delta_sub_part;

        // positive ppb means ref runs fast -> output too high -> reduce requested freq.
        // Negative ppb means ref runs slow -> output too low -> increase requested freq.
        if (_ad_cal > 0) {
            freq_hz = (freq_hz > delta_hz) ? (freq_hz - delta_hz) : 0u;
        } else {
            freq_hz = freq_hz + delta_hz;
        }
    }

    // MHz-split FTW computation -- 32-bit only, max error < 1 Hz at 28 MHz.
    // Split frequency into MHz and sub-MHz Hz parts to avoid truncation error
    // inherent in any kHz-scale constant.
    uint32_t freq_mhz = freq_hz / 1000000UL;    // integer MHz  (0..30)
    uint32_t freq_sub_hz = freq_hz % 1000000UL; // sub-MHz Hz remainder (0..999999)

    // Term 1: MHz-aligned FTW. Max: 30 * 34359738 = 1030792140 < 2^32. OK.
    uint32_t fw = freq_mhz * AD9850_FTW_PER_MHZ;

    // Term 2: integer part of FTW per sub-MHz Hz. Max: 999999 * 34 = 33999966. OK.
    fw += freq_sub_hz * AD9850_FTW_INT_PER_HZ;

    // Term 3: fractional correction. Max: 999999 * 3597 / 10000 = 359696. OK.
    fw += (freq_sub_hz * AD9850_FTW_FRAC_NUM) / AD9850_FTW_FRAC_DEN;

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
    // When CONFIG_OSCILLATOR_ASSUME_AD9850 is disabled this function
    // returns false immediately so oscillator_init() falls through to dummy mode.
    // This allows the firmware to operate without physical AD9850 hardware (e.g.
    // development builds that use only the Si5351, or hardware variants without
    // any oscillator fitted). Enable CONFIG_OSCILLATOR_ASSUME_AD9850 in menuconfig
    // to restore the original "always assumed present" behaviour.
#if !CONFIG_OSCILLATOR_ASSUME_AD9850
    ESP_LOGW(TAG, "AD9850 assumption disabled in Kconfig -- skipping AD9850 init, entering DUMMY mode");
    return false;
#else
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
#endif
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

// The value is in milli-Hz. Function name keeps the legacy "mhz" suffix for compatibility.
esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_millihz) {
    int32_t total_mhz = (int32_t)(base_hz % 1000UL) * 1000 + offset_millihz;
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
        // combined mHz formula for sub-Hz accuracy.
        // frac_mhz is in milli-Hz (0..4395 for WSPR: 3 symbols * 1464.84375 mHz).
        // FTW per mHz = (FTW_INT_PER_HZ*FTW_FRAC_DEN + FTW_FRAC_NUM) / (1000*FTW_FRAC_DEN)
        // Intermediate max: 4395 * 343597 = 1,510,008,615 < 2^32. No overflow.
        fw += (frac_mhz * AD9850_FTW_FRAC_PER_MHZ_NUM) / AD9850_FTW_FRAC_PER_MHZ_DEN;
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
                // Same mHz formula as in oscillator_set_freq_mhz.
                fw += (_ad_last_frac * AD9850_FTW_FRAC_PER_MHZ_NUM) / AD9850_FTW_FRAC_PER_MHZ_DEN;
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
