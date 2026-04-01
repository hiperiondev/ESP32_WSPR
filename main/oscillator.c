/**
 * @file oscillator.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
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
static volatile bool _osc_tx_active = false;
static bool _cal_pending = false;
static int32_t _cal_pending_ppb = 0;

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
    if (_si_xtal < 10000000UL || _si_xtal > 40000000UL) {
        ESP_LOGE(TAG, "SI5351: xtal %lu Hz out of supported range 10-40 MHz", (unsigned long)_si_xtal);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t xtal_mhz = _si_xtal / 1000000UL;
    uint32_t a = 875UL / xtal_mhz;
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

typedef struct {
    uint32_t d_int;      // even integer output divider (MS0 set to integer mode)
    uint32_t pll_a;      // PLL integer multiplier: VCO = xtal * (a + b/c)
    uint32_t pll_c;      // PLL fractional denominator (1048575 for max resolution)
    uint32_t pll_b_base; // PLL fractional numerator for tone offset = 0 mHz
    // 3-term decomposition of delta_b per mHz
    uint32_t b_step_int;  // floor(raw_num / full_den), e.g. 2 for 20m@25MHz (d_int=62)
    uint32_t b_step2_int; // floor((raw_num % full_den) / xtal_kHz), e.g. 600 for 20m@25MHz
    uint32_t b_step2_rem; // (raw_num % full_den) % xtal_kHz, e.g. 11650 for 20m@25MHz
    uint32_t xtal_kHz;    // xtal_Hz / 1000, e.g. 25000
    uint32_t full_den;    // xtal_kHz * 1000 = xtal_Hz, e.g. 25000000
    uint8_t r_div_reg;    // R-divider exponent written to MS0 (0..7)
    bool valid;           // true after si_cache_band() succeeds
} si5351_band_cache_t;

static si5351_band_cache_t _si_cache = { 0 };
static uint32_t _si_last_set_hz = 0u;

static esp_err_t si_write_pll_regs(uint32_t pll_a, uint32_t pll_b, uint32_t pll_c) {
    uint32_t floor_128b_c = (128u * pll_b) / pll_c; // floor(128*b/c), 32-bit
    uint32_t p1 = 128u * pll_a + floor_128b_c - 512u;
    uint32_t p2 = 128u * pll_b - pll_c * floor_128b_c;
    uint32_t p3 = pll_c;
    uint8_t regs[8] = {
        (uint8_t)((p3 >> 8) & 0xFF), (uint8_t)(p3 & 0xFF), (uint8_t)((p1 >> 16) & 0x03),
        (uint8_t)((p1 >> 8) & 0xFF), (uint8_t)(p1 & 0xFF), (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)),
        (uint8_t)((p2 >> 8) & 0xFF), (uint8_t)(p2 & 0xFF),
    };
    return si_write_bulk(SI5351_PLLA_MSNA, regs, 8);
}

static esp_err_t si_write_pll_p1p2_only(uint32_t pll_a, uint32_t pll_b, uint32_t pll_c) {
    uint32_t floor_128b_c = (128u * pll_b) / pll_c;
    uint32_t p1 = 128u * pll_a + floor_128b_c - 512u;
    uint32_t p2 = 128u * pll_b - pll_c * floor_128b_c;

    uint8_t regs[6] = {
        (uint8_t)((p1 >> 16) & 0x03u), (uint8_t)((p1 >> 8) & 0xFFu), (uint8_t)(p1 & 0xFFu), (uint8_t)(((pll_c >> 12) & 0xF0u) | ((p2 >> 16) & 0x0Fu)),
        (uint8_t)((p2 >> 8) & 0xFFu),  (uint8_t)(p2 & 0xFFu),
    };

    return si_write_bulk(SI5351_PLLA_MSNA + 2u, regs, 6);
}

static esp_err_t si_write_ms0_integer(uint32_t d_int, uint8_t r_div_reg) {
    uint32_t p1 = 128u * d_int - 512u;
    uint8_t regs[8] = {
        0x00,
        0x01,
        // Reg 44: MS0_INT (bit 6) | R0_DIV (bits [6:4]) | P1[17:16] (bits [1:0])
        (uint8_t)(0x40u | ((r_div_reg & 0x07u) << 4) | ((p1 >> 16) & 0x03u)),
        (uint8_t)((p1 >> 8) & 0xFF),
        (uint8_t)(p1 & 0xFF),
        0x00,
        0x00,
        0x00,
    };

    return si_write_bulk(SI5351_MS0_BASE, regs, 8);
}

static esp_err_t si_cache_band(uint32_t freq_hz) {
    if (_si_vco == 0) {
        ESP_LOGE(TAG, "SI5351: PLL not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t vco_cal = _si_vco;
    if (_si_cal != 0) {
        uint32_t vco_khz = _si_vco / 1000u;
        int32_t ppb_s = _si_cal;
        int32_t corr;
        if (ppb_s >= 0) {
            uint32_t p = (uint32_t)ppb_s;
            corr = (int32_t)((vco_khz * (p / 1000u)) + (vco_khz * (p % 1000u) / 1000u));
        } else {
            uint32_t p = (uint32_t)(-ppb_s);
            corr = -(int32_t)((vco_khz * (p / 1000u)) + (vco_khz * (p % 1000u) / 1000u));
        }
        vco_cal = (uint32_t)((int32_t)_si_vco + corr);
    }

    uint8_t r_div_reg = 0;
    uint32_t eff_hz = freq_hz;

    while (eff_hz < 500000UL && r_div_reg < 7) {
        eff_hz = eff_hz * 2UL;
        r_div_reg++;

        if (eff_hz == 0u) {
            ESP_LOGE(TAG, "SI5351: eff_hz overflow in R-div loop");
            return ESP_ERR_INVALID_STATE;
        }
    }

    uint32_t d_int = vco_cal / eff_hz;
    if (d_int & 1u) {
        d_int &= ~1u;
    }

    if (d_int < 8UL)
        d_int = 8UL;

    if (d_int > 2048UL) {
        ESP_LOGE(TAG, "SI5351: divider %lu out of range after even-force (vco=%lu eff_hz=%lu)", (unsigned long)d_int, (unsigned long)vco_cal,
                 (unsigned long)eff_hz);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pll_c = 1048575UL;
    uint64_t vco_target = (uint64_t)eff_hz * (uint64_t)d_int;
    uint32_t pll_a = (uint32_t)(vco_target / (uint64_t)_si_xtal);
    uint64_t vco_remainder = vco_target - (uint64_t)pll_a * (uint64_t)_si_xtal;
    uint32_t pll_b_base = (uint32_t)((vco_remainder * (uint64_t)pll_c + (uint64_t)_si_xtal / 2ULL) / (uint64_t)_si_xtal);
    if (pll_b_base >= pll_c)
        pll_b_base = pll_c - 1u;

    uint32_t xtal_kHz = _si_xtal / 1000UL;
    uint32_t full_den = xtal_kHz * 1000UL;
    uint32_t raw_num = d_int * pll_c;
    uint32_t b_step_int = raw_num / full_den;
    uint32_t b_step_rem1 = raw_num % full_den;
    uint32_t b_step2_int = b_step_rem1 / xtal_kHz;
    uint32_t b_step2_rem = b_step_rem1 % xtal_kHz;

    if (pll_a < 15UL || pll_a > 90UL) {
        ESP_LOGE(TAG, "SI5351: PLL multiplier %lu out of range for eff_hz=%lu d_int=%lu", (unsigned long)pll_a, (unsigned long)eff_hz, (unsigned long)d_int);
        return ESP_ERR_INVALID_ARG;
    }

    _si_cache.d_int = d_int;
    _si_cache.pll_a = pll_a;
    _si_cache.pll_c = pll_c;
    _si_cache.pll_b_base = pll_b_base;
    _si_cache.b_step_int = b_step_int;
    _si_cache.b_step2_int = b_step2_int;
    _si_cache.b_step2_rem = b_step2_rem;
    _si_cache.xtal_kHz = xtal_kHz;
    _si_cache.full_den = full_den;
    _si_cache.r_div_reg = r_div_reg;
    _si_cache.valid = true;

    ESP_LOGI(TAG, "SI5351 band cache: d=%lu pll_a=%lu pll_b_base=%lu pll_c=%lu r_div=%u b_step_int=%lu b_step2_int=%lu b_step2_rem=%lu", (unsigned long)d_int,
             (unsigned long)pll_a, (unsigned long)pll_b_base, (unsigned long)pll_c, (unsigned)r_div_reg, (unsigned long)b_step_int, (unsigned long)b_step2_int,
             (unsigned long)b_step2_rem);

    esp_err_t err = si_write_ms0_integer(d_int, r_div_reg);
    if (err != ESP_OK)
        return err;

    err = si_write_pll_regs(pll_a, pll_b_base, pll_c);
    if (err != ESP_OK)
        return err;

    return si_write(SI5351_PLL_RESET, 0x20);
}

static esp_err_t si_apply_tone(uint32_t tone_millihz) {
    if (!_si_cache.valid) {
        ESP_LOGE(TAG, "SI5351: si_apply_tone called before si_cache_band");
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t scaled_tone = tone_millihz << _si_cache.r_div_reg;
    uint32_t delta_b = scaled_tone * _si_cache.b_step_int;
    delta_b += (scaled_tone * _si_cache.b_step2_int) / 1000UL;
    delta_b += (scaled_tone * _si_cache.b_step2_rem) / _si_cache.full_den;
    uint32_t pll_b = _si_cache.pll_b_base + delta_b;

    if (pll_b >= _si_cache.pll_c) {
        pll_b = _si_cache.pll_c - 1u;
    }

    return si_write_pll_p1p2_only(_si_cache.pll_a, pll_b, _si_cache.pll_c);
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

#if AD9850_REF_CLK == 125000000UL
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

#define AD9850_FTW_FRAC_PER_MHZ_NUM ((uint32_t)(AD9850_FTW_INT_PER_HZ * AD9850_FTW_FRAC_DEN + AD9850_FTW_FRAC_NUM))
#define AD9850_FTW_FRAC_PER_MHZ_DEN (1000UL * AD9850_FTW_FRAC_DEN)

static_assert((30UL * (4294967296ULL / ((uint32_t)AD9850_REF_CLK / 1000000UL)) + 999999UL * (4294967296ULL / (uint32_t)AD9850_REF_CLK) +
               999999UL * AD9850_FTW_FRAC_NUM / AD9850_FTW_FRAC_DEN) < 0xFFFFFFFFUL,
              "AD9850 FTW overflow: max WSPR frequency word must fit in 32 bits for the configured ref clock");

static uint32_t _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ;
static uint32_t _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ;

static uint32_t ad9850_freq_word(uint32_t freq_hz) {
    uint32_t freq_mhz = freq_hz / 1000000UL;    // integer MHz  (0..30)
    uint32_t freq_sub_hz = freq_hz % 1000000UL; // sub-MHz Hz remainder (0..999999)
    uint32_t fw = freq_mhz * _ad_ftw_per_mhz_cal;

    fw += freq_sub_hz * _ad_ftw_int_per_hz_cal;
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
        if (freq_hz == _si_last_set_hz && _si_cache.valid) {
            return si_apply_tone(0u);
        }

        esp_err_t err = si_cache_band(freq_hz);
        if (err == ESP_OK)
            _si_last_set_hz = freq_hz;
        return err;
    }
    if (_osc_type == OSC_AD9850) {
        _ad_last_hz = freq_hz;
        _ad_last_frac = 0u;
        ad9850_write_word(ad9850_freq_word(freq_hz), 0u);
    }
    return ESP_OK;
}

esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_millihz) {
    if (_osc_type == OSC_SI5351) {

        uint32_t tone_mhz = (offset_millihz >= 0) ? (uint32_t)offset_millihz : 0u;
        return si_apply_tone(tone_mhz);
    }

    if (_osc_type == OSC_AD9850) {
        int32_t total_mhz = (int32_t)(base_hz % 1000UL) * 1000 + offset_millihz;
        int32_t frac_mhz_s = total_mhz % 1000;
        int32_t extra_hz_s = total_mhz / 1000;
        if (frac_mhz_s < 0) {
            frac_mhz_s += 1000;
            extra_hz_s -= 1;
        }

        if (extra_hz_s < 0) {
            ESP_LOGE(TAG,
                     "AD9850: extra_hz_s=%ld negative after normalization "
                     "(base_hz=%lu offset_mhz=%ld) -- skipping TX",
                     (long)extra_hz_s, (unsigned long)base_hz, (long)offset_millihz);
            return ESP_ERR_INVALID_ARG;
        }
        uint32_t extra_hz = (uint32_t)extra_hz_s;
        uint32_t out_hz = (base_hz / 1000UL) * 1000UL + extra_hz;
        uint32_t frac_mhz = (uint32_t)frac_mhz_s;
        _ad_last_hz = out_hz;
        _ad_last_frac = frac_mhz;
        uint32_t fw = ad9850_freq_word(out_hz);

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
                fw += (_ad_last_frac * AD9850_FTW_FRAC_PER_MHZ_NUM) / AD9850_FTW_FRAC_PER_MHZ_DEN;
                ad9850_write_word(fw, 0x00u);
            }
        } else {
            ad9850_write_word(0u, 0x04u);
        }
    }
    return ESP_OK;
}

void oscillator_tx_begin(void) {
    _osc_tx_active = true;
}

void oscillator_tx_end(void) {
    _osc_tx_active = false;
    if (_cal_pending) {
        _cal_pending = false;
        _si_cal = _cal_pending_ppb;
        _si_cache.valid = false;
        _si_last_set_hz = 0u; // force full si_cache_band() on next oscillator_set_freq()
        ESP_LOGI(TAG, "SI5351 deferred cal applied: %ld ppb", (long)_cal_pending_ppb);
    }
}

esp_err_t oscillator_set_cal(int32_t ppb) {
    if (_osc_type == OSC_SI5351) {
        if (_osc_tx_active) {
            _cal_pending = true;
            _cal_pending_ppb = ppb;
            ESP_LOGW(TAG, "SI5351 cal deferred (TX active): %ld ppb", (long)ppb);
            return ESP_OK;
        }

        _si_cal = ppb;

        _si_cache.valid = false;
        _si_last_set_hz = 0u;
        ESP_LOGI(TAG, "SI5351 cal set: %ld ppb", (long)ppb);
    } else if (_osc_type == OSC_AD9850) {
        _ad_cal = ppb;

        uint32_t ppb_abs = (ppb >= 0) ? (uint32_t)ppb : (uint32_t)(-(int32_t)ppb);
        uint32_t delta_per_mhz = (uint32_t)(((uint64_t)AD9850_FTW_PER_MHZ * (uint64_t)ppb_abs) / 1000000ULL);
        uint32_t delta_per_hz = (uint32_t)(((uint64_t)AD9850_FTW_INT_PER_HZ * (uint64_t)ppb_abs) / 1000000ULL);
        if (ppb > 0) {
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ - delta_per_mhz;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ - delta_per_hz;
        } else if (ppb < 0) {
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ + delta_per_mhz;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ + delta_per_hz;
        } else {
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ;
        }
        ESP_LOGI(TAG, "AD9850 cal set: %ld ppb -> FTW/MHz=%lu FTW/Hz=%lu", (long)ppb, (unsigned long)_ad_ftw_per_mhz_cal,
                 (unsigned long)_ad_ftw_int_per_hz_cal);
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
