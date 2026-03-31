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
    // Reject crystal frequencies outside the Si5351A supported range
    // (10-40 MHz per datasheet). A value like 50 MHz or 100 MHz would produce a
    // PLL multiplier inside the valid 15-90 range but drive the VCO above 900 MHz
    // spec, causing silent lock failure or wrong output frequency.
    if (_si_xtal < 10000000UL || _si_xtal > 40000000UL) {
        ESP_LOGE(TAG, "SI5351: xtal %lu Hz out of supported range 10-40 MHz", (unsigned long)_si_xtal);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t xtal_mhz = _si_xtal / 1000000UL;
    // Target 875 MHz instead of 900 MHz (Si5351A VCO absolute max per AN619).
    // Operating at exactly 900 MHz means a crystal running even slightly fast, or any
    // temperature-induced VCO gain shift, can push the VCO above spec, increasing the
    // probability of PLL loss-of-lock.  875 MHz gives a comfortable 2.8 % safety margin.
    // For 25 MHz xtal: a = 35, VCO = 875 MHz.  For 27 MHz xtal: a = 32, VCO = 864 MHz.
    // Frequency resolution impact is negligible for WSPR (PLL step ~23 mHz, far below
    // the 1465 mHz WSPR tone spacing).
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

// the output divider (MS0) to an even integer at band-select time,
// and modulate the PLL feedback numerator (PLLA MSNA) for each WSPR symbol.
// This approach (PLL-fractional FSK) gives ~23 mHz resolution on ALL bands
// including 10m and 12m, because PLL step = f_xtal / c_pll ~= 25e6 / 1048575
// ~= 23.8 mHz -- completely independent of the output divider ratio.
//
// Per-symbol update writes only PLLA MSNA registers (8 bytes, 1 I2C transaction).
// The output divider MS0 registers are written once per band change.
// Per-band cache: set once by si_cache_band(), used by si_apply_tone()
//
// formula:
//   delta_b = tone_mHz * d_int * pll_c / (xtal_Hz * 1000)
//
// 3-term decomposition (all multiplications stay <= ~4395 * max_b_step2_int < 2^32):
//   full_den  = xtal_kHz * 1000 [= xtal_Hz, e.g. 25,000,000 for 25 MHz]
//               (divide num & den by 1000 to keep full_den in 32-bit)
//   raw_num   = d_int * pll_c         [max 2048*1048575 ~2.15e9 < 2^32]
//   b_step_int  = raw_num / full_den  [floor, e.g. 2 for 20m@25MHz with d_int=62]
//   b_step2_int = (raw_num % full_den) / xtal_kHz
//   b_step2_rem = (raw_num % full_den) % xtal_kHz
//
// At symbol time (si_apply_tone):
//   delta_b = tone_mHz * b_step_int
//           + tone_mHz * b_step2_int / 1000
//           + tone_mHz * b_step2_rem / full_den
//
// Example below uses actual 20m values with 25 MHz xtal (VCO=875 MHz, d_int=62):
//   raw_num = 62*1048575 = 65011650; full_den = 25000000
//   b_step_int  = 65011650/25000000 = 2
//   b_step_rem1 = 65011650%25000000 = 15011650
//   b_step2_int = 15011650/25000   = 600
//   b_step2_rem = 15011650%25000   = 11650
// Max intermediate products (tone=4395 mHz, 20m@25MHz, d_int=62):
//   term1: 4395 * 2             = 8790              < 2^32. Safe.
//   term2: 4395 * 600 / 1000   = 2637               < 2^32. Safe.
//   term3: 4395 * 11650 / 25M  = 2                  < 2^32. Safe.
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
// MODIFIED (Bug 5): cache the last frequency passed to oscillator_set_freq().
// When the same frequency is requested again and _si_cache.valid is true,
// si_cache_band() (17 I2C writes + PLL reset, ~2 ms) is skipped entirely.
// Only si_apply_tone(0) is called to re-zero the tone offset to carrier centre.
// Reset to 0 by oscillator_set_cal() so a calibration change forces full recache.
static uint32_t _si_last_set_hz = 0u;

// Write PLLA MSNA fractional multiplier registers (8 bytes, regs 26-33).
// p3=c, p1=128*a + floor(128*b/c) - 512, p2=128*b - c*floor(128*b/c).
// All arithmetic is 32-bit; b < c < 1048576 so 128*b < 134M < 2^32. Safe.
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

// Write MS0 output divider registers for an even integer divider d_int.
// Integer mode gives best jitter. p1=128*d-512, p2=0, p3=1.
// r_div_reg is the R-divider exponent (0..7) written into bits [6:4] of reg 44.
static esp_err_t si_write_ms0_integer(uint32_t d_int, uint8_t r_div_reg) {
    uint32_t p1 = 128u * d_int - 512u;
    uint8_t regs[8] = {
        0x00, 0x01, (uint8_t)((((p1 >> 16) & 0x03)) | (r_div_reg << 4)), (uint8_t)((p1 >> 8) & 0xFF), (uint8_t)(p1 & 0xFF), 0x00, 0x00, 0x00,
    };
    // Also set CLK0 to integer mode: bit 6 of CLK0_CTRL = 1 for integer mode.
    // We only set the MS0_INT flag (reg 22 bit 6); CLK0_CTRL (reg 16) drive
    // settings were already configured in si5351_try_init().
    // Note: MS0 integer mode bit is reg 22 bit 6 on Si5351A.
    si_write(22, 0x40); // MS0 integer mode enable (reg 22, bit 6)
    return si_write_bulk(SI5351_MS0_BASE, regs, 8);
}

// Cache band parameters for per-symbol 32-bit-only PLL numerator updates.
// Called ONCE per band change (64-bit arithmetic is acceptable here).
// All intermediate products during cache setup use 64-bit (acceptable: called
// once per band change ~120 s). Per-symbol arithmetic in si_apply_tone() is 32-bit only.
static esp_err_t si_cache_band(uint32_t freq_hz) {
    if (_si_vco == 0) {
        ESP_LOGE(TAG, "SI5351: PLL not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Apply crystal calibration to derive effective VCO reference.
    // kHz-split formula to achieve 1 Hz correction resolution at 1 ppb input. splits ppb
    // into whole-ppm and sub-ppm parts, keeping all intermediate products in 32-bit:
    //   max (vco_khz * (ppb/1000)):  900000 * 100    =  90 000 000  < 2^32. Safe.
    //   max (vco_khz * (ppb%1000)):  900000 * 999    = 899 100 000  < 2^32. Safe.
    //   after /1000:                                  =     899 100  < 2^32. Safe.
    // Covers the full practical range of +/-100 000 ppb (+/-100 ppm).
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

    // R-divider: for sub-500 kHz bands, scale up the effective frequency
    // so the output divider stays in a valid range (8..2048).
    uint8_t r_div_reg = 0;
    uint32_t eff_hz = freq_hz;

    // Replace configASSERT with defensive clamp and error return.
    // The loop doubles eff_hz until it reaches >= 500 kHz or R-div hits max.
    // The guard condition (eff_hz remains < 500000 after r_div_reg==7) is
    // essentially unreachable for any valid WSPR band (minimum 137.6 kHz *
    // 2^7 = 17.6 MHz) but we protect with an explicit check instead of abort().
    while (eff_hz < 500000UL && r_div_reg < 7) {
        eff_hz = eff_hz * 2UL;
        r_div_reg++;
        // Defensive guard: eff_hz must not overflow or wrap to zero.
        // This is unreachable in normal operation but prevents undefined behaviour.
        if (eff_hz == 0u) {
            ESP_LOGE(TAG, "SI5351: eff_hz overflow in R-div loop");
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Choose even integer output divider. AN619 recommends even integers for
    // best jitter when MS0 is in integer mode.
    // d_int = round_down_even(vco_cal / eff_hz)
    uint32_t d_int = vco_cal / eff_hz;
    if (d_int < 8UL || d_int > 2048UL) {
        ESP_LOGE(TAG, "SI5351: divider %lu out of range (vco=%lu eff_hz=%lu)", (unsigned long)d_int, (unsigned long)vco_cal, (unsigned long)eff_hz);
        return ESP_ERR_INVALID_ARG;
    }
    // Force even integer for integer-mode operation (best phase noise).
    if (d_int & 1u) {
        d_int &= ~1u; // round down to even
        if (d_int < 8UL)
            d_int = 8UL;
    }

    // PLL fractional denominator: use maximum 20-bit value for finest resolution.
    // Step size = f_xtal / c_pll ~= 25e6 / 1048575 ~= 23.84 mHz -- uniform
    // across all bands because it depends only on xtal, not on d_int.
    uint32_t pll_c = 1048575UL;

    // PLL integer multiplier: a = VCO_target / xtal (integer part).
    // VCO_target = eff_hz * d_int (exact integer, no rounding needed here).
    // For 25 MHz xtal, d_int=62, eff_hz=14098600: VCO_target=874113200, pll_a=34.
    // 64-bit is used here because eff_hz*d_int can reach ~900 MHz which
    // exceeds 32-bit range (max 4.29 GHz is fine but the intermediate
    // pll_a computation needs care for the fractional part).
    // This function is called once per band change (~120 s interval), so
    // 64-bit cost here is acceptable.
    uint64_t vco_target = (uint64_t)eff_hz * (uint64_t)d_int;     // exact integer VCO target
    uint32_t pll_a = (uint32_t)(vco_target / (uint64_t)_si_xtal); // integer part of multiplier

    // pll_b_base = round( (vco_target - pll_a * xtal) * pll_c / xtal )
    // This is the numerator for tone_offset = 0 (carrier centre).
    uint64_t vco_remainder = vco_target - (uint64_t)pll_a * (uint64_t)_si_xtal;
    uint32_t pll_b_base = (uint32_t)((vco_remainder * (uint64_t)pll_c) / (uint64_t)_si_xtal);

    // Compute 3-term decomposition for per-symbol delta_b.
    // Correct formula: delta_b = tone_mHz * d_int * pll_c / (xtal_Hz * 1000)
    //
    // divide numerator and denominator by 1000 so full_den fits in 32-bit:
    //   raw_num  = d_int * pll_c              [max 2048*1048575 ~2.15e9 < 2^32]
    //   full_den = xtal_kHz * 1000            [e.g. 25000*1000 = 25,000,000 < 2^32]
    // Then decompose into 3 terms to avoid any single product overflowing 32-bit:
    //   b_step_int  = raw_num / full_den      [floor of first level]
    //   b_step2_int = (raw_num%full_den) / xtal_kHz  [floor of second level]
    //   b_step2_rem = (raw_num%full_den) % xtal_kHz  [sub-kHz remainder]
    //
    // example values corrected for 875 MHz VCO (d_int=62 for 20m@25MHz):
    //   raw_num = 62*1048575 = 65011650
    //   b_step_int  = 65011650/25000000 = 2
    //   b_step_rem1 = 65011650%25000000 = 15011650
    //   b_step2_int = 15011650/25000   = 600
    //   b_step2_rem = 15011650%25000   = 11650
    // (The old comments showed d_int=36 values corresponding to 900 MHz VCO,
    //  which is no longer used. The 875 MHz target in si_init_pll() gives
    //  d_int=62 for 20m with a 25 MHz xtal, not d_int=36.)
    uint32_t xtal_kHz = _si_xtal / 1000UL;         // e.g. 25000
    uint32_t full_den = xtal_kHz * 1000UL;         // e.g. 25000000 (fits in 32-bit)
    uint32_t raw_num = d_int * pll_c;              // e.g. 62*1048575 = 65011650 for 20m
    uint32_t b_step_int = raw_num / full_den;      // e.g. 65011650/25000000 = 2 for 20m
    uint32_t b_step_rem1 = raw_num % full_den;     // e.g. 15011650 for 20m
    uint32_t b_step2_int = b_step_rem1 / xtal_kHz; // e.g. 15011650/25000 = 600 for 20m
    uint32_t b_step2_rem = b_step_rem1 % xtal_kHz; // e.g. 15011650%25000 = 11650 for 20m

    // Clamp pll_a to valid PLL multiplier range (15..90 per Si5351 spec).
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

    // Write the fixed integer output divider to MS0 registers (once per band).
    esp_err_t err = si_write_ms0_integer(d_int, r_div_reg);
    if (err != ESP_OK)
        return err;

    // Program PLL for tone offset = 0 and reset PLL to lock.
    err = si_write_pll_regs(pll_a, pll_b_base, pll_c);
    if (err != ESP_OK)
        return err;

    // Reset PLLA to ensure lock after changing divider.
    return si_write(SI5351_PLL_RESET, 0x20);
}

// Apply a WSPR tone offset (in mHz) by updating only the PLL numerator.
// Called 162 times per transmission -- ALL arithmetic is 32-bit only.
// PLL-based FSK gives uniform ~23.8 mHz step on ALL bands including 10m/12m,
// eliminating the 14% tone-spacing error caused by the coarse output-divider steps.
//
// 3-term decomposition that stays within 32-bit and produces
// the right delta_b (~4 counts per WSPR tone step at 14 MHz / 25 MHz xtal).
//
// values for 20m with 25 MHz xtal (VCO=875 MHz, d_int=62):
//   b_step_int=2, b_step2_int=600, b_step2_rem=11650, full_den=25000000
//
// Max per-product analysis (worst-case: tone=4395 mHz, 20m@25MHz, d_int=62):
//   term1: 4395 * b_step_int(2)             = 8790              < 2^32. Safe.
//   term2: 4395 * b_step2_int(600) / 1000   = 2637000 / 1000
//                                            = 2637              < 2^32. Safe.
//   term3: 4395 * b_step2_rem(11650) / 25M  = 51201750 / 25M
//                                            = 2                 < 2^32. Safe.
//   delta_b for tone=1465 (symbol 1): 2+878+1 = 881 (approx; exact value band-specific).
//
// 10m is the highest frequency band (d_int=30, b_step2_int=258, b_step2_rem=7250):
//   term2 num: 4395*258 = 1133910 < 2^32. Safe.
//   term3 num: 4395*7250 = 31863750 < 2^32. Safe.
// All bands safe with 32-bit-only arithmetic.
static esp_err_t si_apply_tone(uint32_t tone_millihz) {
    if (!_si_cache.valid) {
        ESP_LOGE(TAG, "SI5351: si_apply_tone called before si_cache_band");
        return ESP_ERR_INVALID_STATE;
    }

    // FIXED: scale tone by 2^r_div_reg to compensate for the R-divider.
    // The R-divider in si_cache_band() divides the MS0 output by 2^r_div_reg.
    // d_int was computed from eff_hz = freq_hz * 2^r_div_reg, so the 3-term
    // decomposition already contains a factor of 1/2^r_div_reg relative to the
    // physical RF output. Without this correction the tone spacing on 630m is
    // 0.732 Hz (2x too small) and on 2200m is 0.366 Hz (4x too small) instead
    // of the required WSPR 1.4648 Hz. Both conditions make the signal completely
    // undecodable by WSJT-X or any other WSPR decoder worldwide.
    //
    // r_div_reg = 0: no scaling (160m through 10m, all standard WSPR bands)
    // r_div_reg = 1: x2 scaling (630m at 475.7 kHz)
    // r_div_reg = 2: x4 scaling (2200m at 137.6 kHz)
    //
    // Overflow analysis for worst case (2200m, r_div_reg=2, max WSPR tone=4395 mHz):
    //   scaled_tone = 4395 << 2 = 17580 mHz
    //   For 2200m @25 MHz xtal (vco=875 MHz, eff_hz=550400 Hz):
    //     d_int = 875000000/550400 = 1589
    //     raw_num = 1589 * 1048575 = 1,666,185,825
    //     b_step_int  = 1666185825 / 25000000 = 66
    //     b_step2_int = (1666185825 % 25000000) / 25000 = 647
    //     b_step2_rem = (1666185825 % 25000000) % 25000 = 10825
    //   term1: 17580 * 66             = 1,160,280         < 2^32. Safe.
    //   term2: 17580 * 647 / 1000     = 11,378            < 2^32. Safe.
    //   term3: 17580 * 10825 / 25e6   = 7                 < 2^32. Safe.
    uint32_t scaled_tone = tone_millihz << _si_cache.r_div_reg;

    // 3-term delta_b computation using scaled_tone (all 32-bit arithmetic).
    // All intermediates fit in uint32_t; see overflow analysis in comment above.
    // term1: integer part (floor level 1)
    uint32_t delta_b = scaled_tone * _si_cache.b_step_int;
    // term2: integer part of the remainder at xtal_kHz granularity, scaled /1000
    delta_b += (scaled_tone * _si_cache.b_step2_int) / 1000UL;
    // term3: sub-xtal_kHz remainder divided by full_den
    delta_b += (scaled_tone * _si_cache.b_step2_rem) / _si_cache.full_den;

    uint32_t pll_b = _si_cache.pll_b_base + delta_b;
    // Clamp: defensive only -- with correct delta_b values pll_b should not
    // exceed pll_c for any valid WSPR tone on any supported band.
    if (pll_b >= _si_cache.pll_c) {
        pll_b = _si_cache.pll_c - 1u;
    }

    // Write only the PLL MSNA registers (8 bytes). MS0 is not touched.
    // No PLL reset required for small frequency steps -- the PLL tracks
    // the new numerator within one reference cycle (~40 ns for 25 MHz xtal).
    return si_write_pll_regs(_si_cache.pll_a, pll_b, _si_cache.pll_c);
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

        // Three-term decomposition:
        //   f_mhz         = freq_hz / 1e6              (integer MHz)
        //   f_sub_khz     = (freq_hz % 1e6) / 1000     (integer kHz below MHz boundary)
        //   f_sub_hz_rem  = (freq_hz % 1e6) % 1000     (integer Hz below kHz boundary)
        //
        //   delta = f_mhz * ppb / 1000
        //         + f_sub_khz * ppb / 1e6
        //         + f_sub_hz_rem * ppb / 1e9            (in Hz, computed as ppb/1e6 * 1e-3)
        //
        // Overflow analysis (at ppb_abs = 100000, worst-case):
        //   MHz term:         30 * 100000 / 1000       = 3000              < 2^32. Safe.
        //   kHz term:        999 * 100000 / 1000000    = 99                < 2^32. Safe.
        //   sub-Hz term:     999 * 100000 / 1000000    = 99                < 2^32. Safe.
        //     (sub-Hz term formula: f_sub_hz_rem * ppb_abs / 1000000,
        //      which is Hz-scale; max 999 * 100000 = 99900000 < 2^32. Safe.)
        //
        // The sub-kHz term contribution at 475700 Hz with 1 ppb:
        //   f_sub_hz_rem = 700; delta_sub_hz_rem = 700 * 1 / 1000000 = 0 Hz (rounds down).
        // At 1 ppb the entire correction is sub-mHz, which is below the AD9850 FTW
        // resolution (~0.03 Hz at 125 MHz).
        uint32_t f_mhz = freq_hz / 1000000UL;
        uint32_t f_sub_hz = freq_hz % 1000000UL;
        uint32_t f_sub_khz = f_sub_hz / 1000UL;
        // Extract the sub-kHz Hz remainder that was previously dropped.
        uint32_t f_sub_hz_rem = f_sub_hz % 1000UL;
        uint32_t delta_mhz_part = (f_mhz * ppb_abs) / 1000UL;
        uint32_t delta_sub_part = (f_sub_khz * ppb_abs) / 1000000UL;
        // sub-kHz correction term.
        // f_sub_hz_rem * ppb_abs / 1e9 (in Hz) = f_sub_hz_rem * ppb_abs / 1000000 (truncated to Hz).
        // Max intermediate product: 999 * 100000 = 99900000 < 2^32. No overflow.
        uint32_t delta_sub_hz_rem = (f_sub_hz_rem * ppb_abs) / 1000000UL;
        uint32_t delta_hz = delta_mhz_part + delta_sub_part + delta_sub_hz_rem;

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

// oscillator_set_freq now calls si_cache_band() which sets up
// the integer output divider and PLL base numerator for the new band.
// The old si_set_freq_hz_mhz() (which used 64-bit ops per symbol) is removed.
// MODIFIED (Bug 5): si_cache_band() is now skipped when freq_hz matches the
// last programmed frequency and the band cache is still valid.  This avoids
// 17 unnecessary I2C writes and one PLL reset (~2 ms total) on every same-band
// TX slot.  For a fixed-band station every 2-minute pre-arm call previously
// triggered a full PLL re-lock; now it falls through to si_apply_tone(0)
// which writes only 8 bytes to zero the tone offset to carrier centre.
esp_err_t oscillator_set_freq(uint32_t freq_hz) {
    if (_osc_type == OSC_SI5351) {
        // MODIFIED (Bug 5): skip the expensive si_cache_band() path when the
        // frequency has not changed and the band cache is still valid.
        // si_apply_tone(0) re-zeros the PLL numerator to the carrier centre
        // using only 8 I2C bytes and no PLL reset -- safe because the PLL
        // multiplier (pll_a, pll_c, pll_b_base) is identical to the cached state.
        // The check on _si_cache.valid ensures we never call si_apply_tone()
        // before si_cache_band() has populated the cache (e.g. cold start,
        // or after a calibration change clears the valid flag).
        if (freq_hz == _si_last_set_hz && _si_cache.valid) {
            return si_apply_tone(0u);
        }
        // Frequency changed (new band, first call, or post-calibration).
        // Run full si_cache_band(): programs MS0 integer divider, PLL base
        // numerator, and issues one PLL reset to guarantee phase lock.
        esp_err_t err = si_cache_band(freq_hz);
        if (err == ESP_OK)
            _si_last_set_hz = freq_hz; // MODIFIED (Bug 5): record freq on success only
        return err;
    }
    if (_osc_type == OSC_AD9850) {
        _ad_last_hz = freq_hz;
        _ad_last_frac = 0u;
        ad9850_write_word(ad9850_freq_word(freq_hz), 0u);
    }
    return ESP_OK;
}

// oscillator_set_freq_mhz now calls si_apply_tone() for Si5351,
// which uses only 32-bit arithmetic per symbol (162 calls per TX window).
// The value is in milli-Hz. Function name keeps the legacy "mhz" suffix for compatibility.
esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_millihz) {
    if (_osc_type == OSC_SI5351) {
        // si_apply_tone requires a non-negative mHz offset.
        // WSPR tone offsets are always >= 0 (symbols 0-3 map to 0..4395 mHz).
        // Guard against negative values (should not occur in normal operation).
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
        // MODIFIED (Bug 8): guard against negative extra_hz_s before unsigned cast.
        // Should not occur with valid WSPR inputs (offset_millihz >= 0, base_hz >= 137600 Hz).
        // Protects against future misuse or calibration arithmetic underflow: an unchecked
        // cast of a negative extra_hz_s to uint32_t wraps to ~2^32, corrupting out_hz
        // to approximately 4 GHz which causes the AD9850 to receive a garbage frequency
        // word and output an unpredictable frequency.
        if (extra_hz_s < 0) {
            ESP_LOGE(TAG, "AD9850: extra_hz_s=%ld negative after normalization "
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
        // Invalidate the band cache so the next oscillator_set_freq() call
        // recomputes the PLL with the updated calibration offset.
        _si_cache.valid = false;
        _si_last_set_hz = 0u; // MODIFIED (Bug 5): force full si_cache_band() on next set_freq call
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
