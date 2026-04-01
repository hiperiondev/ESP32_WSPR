/**
 * @file oscillator.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez  (lu3vea@gmail.com)
 * @brief Runtime-detected RF oscillator driver — Si5351A or AD9850.
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

// Runtime oscillator type selected during oscillator_init()
typedef enum {
    OSC_NONE = 0,   // no hardware detected; all operations are no-ops
    OSC_SI5351 = 1, // Silicon Labs Si5351A clock generator (I2C)
    OSC_AD9850 = 2, // Analog Devices AD9850 DDS synthesizer (bit-bang serial)
} osc_type_t;

static osc_type_t _osc_type = OSC_NONE;
static bool _osc_hw_ok = false;              // true when real hardware was detected
static volatile bool _osc_tx_active = false; // true during a WSPR symbol loop
// Deferred calibration: stores a ppb value received while TX is in progress
static bool _cal_pending = false;
static int32_t _cal_pending_ppb = 0;

// =============================================================================
//  SI5351 Driver
// =============================================================================
//
// The Si5351A is a programmable clock generator from Silicon Labs.
// Architecture: XTAL -> PLL (feedback Multisynth, VCO 600-900 MHz) -> Output Multisynth
//               -> optional R-divider -> CLK0 output pin.
//
// For WSPR the strategy is:
//   1. Fix the output Multisynth (MS0) to an even integer divider for low jitter.
//   2. Vary the PLL fractional numerator (b) between symbols to produce the
//      sub-Hz tone offsets required by WSPR (1.4648 Hz steps).
//   This avoids resetting the PLL on every symbol, which would cause a phase
//   discontinuity and glitch.  Keeping MS0 integer and dithering only the PLL
//   numerator produces continuous-phase FSK as required by the WSPR protocol.
//
// References:
//   - Si5351A datasheet (Skyworks)
//   - AN619: Manually Generating an Si5351 Register Map

// I2C address of the Si5351A (factory default, not configurable)
#define SI5351_ADDR 0x60
// Register addresses (from Si5351 datasheet Table 1)
#define SI5351_CLK0_CTRL         16   // CLK0 control: power, invert, source, drive
#define SI5351_PLLA_MSNA         26   // PLLA Multisynth (feedback) registers 26-33
#define SI5351_MS0_BASE          42   // Multisynth0 (output) registers 42-49
#define SI5351_PLL_RESET         177  // PLL reset register; bit 5 = reset PLLA
#define SI5351_OUTPUT_EN         3    // Output enable control register
#define SI5351_CLK0_MS0_SRC_PLLA 0x0C // CLK0 sourced from PLLA, integer mode bit

// Drive current selection for CLK0 output (bits 1:0 of CLK0 control register)
#if defined(CONFIG_SI5351_DRIVE_2MA)
#define SI5351_IDRV 0x00u // 2 mA ~ +3 dBm into 50 Ω
#elif defined(CONFIG_SI5351_DRIVE_4MA)
#define SI5351_IDRV 0x01u // 4 mA ~ +7 dBm into 50 Ω
#elif defined(CONFIG_SI5351_DRIVE_6MA)
#define SI5351_IDRV 0x02u // 6 mA ~ +10 dBm into 50 Ω
#else
#define SI5351_IDRV 0x03u // 8 mA ~ +13 dBm into 50 Ω (default)
#endif
// Complete CLK0 control byte: PLLA source (bits 3:2) | drive current (bits 1:0)
#define SI5351_CLK0_CTRL_VAL (0x0Cu | SI5351_IDRV)

// Crystal frequency and runtime state for the Si5351 driver
static uint32_t _si_xtal = (uint32_t)CONFIG_SI5351_XTAL_FREQ;
static uint32_t _si_vco = 0; // VCO frequency computed by si_init_pll()
static int32_t _si_cal = 0;  // crystal calibration offset in ppb

// I2C bus and device handles (ESP-IDF new I2C master API)
static i2c_master_bus_handle_t si_bus_handle = NULL;
static i2c_master_dev_handle_t si_dev_handle = NULL;

/**
 * @brief Write a single byte to an Si5351 register over I2C.
 * @param reg Register address.
 * @param val Byte value to write.
 * @return ESP_OK on success.
 */
static esp_err_t si_write(uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(si_dev_handle, data, 2, pdMS_TO_TICKS(100));
}

/**
 * @brief Write a block of consecutive Si5351 registers in a single I2C transaction.
 *
 * Packs the register address and data bytes into a local buffer and sends them
 * in one I2C transfer to minimise bus overhead. Limited to 8 data bytes because
 * all Si5351 register blocks relevant to WSPR (MSNA and MS0) are 8 bytes wide.
 *
 * @param reg  Starting register address.
 * @param buf  Data bytes to write.
 * @param len  Number of bytes (must be <= 8).
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if len > 8.
 */
static esp_err_t si_write_bulk(uint8_t reg, const uint8_t *buf, int len) {
    uint8_t data[9]; // 1 address byte + up to 8 data bytes
    if (len + 1 > (int)sizeof(data)) {
        return ESP_ERR_INVALID_SIZE;
    }
    data[0] = reg;
    memcpy(&data[1], buf, (size_t)len);
    return i2c_master_transmit(si_dev_handle, data, (size_t)(len + 1), pdMS_TO_TICKS(100));
}

/**
 * @brief Probe the Si5351 by reading register 0 via I2C.
 *
 * A successful ACK on the write and a valid read response confirms that
 * an Si5351A is present on the I2C bus at address 0x60.
 *
 * @return ESP_OK if the device responded; an I2C error code otherwise.
 */
static esp_err_t si5351_ping(void) {
    uint8_t reg = 0x00;
    uint8_t dummy = 0;
    return i2c_master_transmit_receive(si_dev_handle, &reg, 1, &dummy, 1, pdMS_TO_TICKS(200));
}

/**
 * @brief Compute and program the PLLA integer-only feedback Multisynth (MSNA).
 *
 * The Si5351 VCO must operate in the range 600-900 MHz (per datasheet).
 * This function selects the smallest integer multiplier 'a' that keeps
 * VCO = a * xtal inside that window, then programs the MSNA registers
 * with b=0, c=1 (pure integer mode) for minimum jitter.
 *
 * AN619 Equation for MSNA integer mode:
 *   P1 = 128 * a - 512
 *   P2 = 0  (b = 0)
 *   P3 = 1  (c = 1)
 * Written to registers 26-33 (MSNA base).
 *
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG if xtal is out of range.
 */
static esp_err_t si_init_pll(void) {
    // Verify crystal is within the Si5351 supported input range
    if (_si_xtal < 10000000UL || _si_xtal > 40000000UL) {
        ESP_LOGE(TAG, "SI5351: xtal %lu Hz out of supported range 10-40 MHz", (unsigned long)_si_xtal);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t xtal_mhz = _si_xtal / 1000000UL;
    // Choose multiplier 'a' to target ~875 MHz VCO (mid-range for headroom)
    uint32_t a = 875UL / xtal_mhz;
    if (a < 15UL)
        a = 15UL; // MSNA minimum integer multiplier per AN619
    if (a > 90UL)
        a = 90UL; // MSNA maximum integer multiplier per AN619
    _si_vco = a * xtal_mhz * 1000000UL;
    // P1 for integer-only PLL: P1 = 128*a - 512, P2=0, P3=1 (b=0, c=1)
    uint32_t p1 = 128UL * a - 512UL;
    uint8_t pll_regs[8] = {
        0x00, 0x01, (uint8_t)((p1 >> 16) & 0x03), (uint8_t)((p1 >> 8) & 0xFF), (uint8_t)(p1 & 0xFF), 0x00, 0x00, 0x00,
    };
    esp_err_t err = si_write_bulk(SI5351_PLLA_MSNA, pll_regs, 8);
    if (err != ESP_OK)
        return err;
    // Reset PLLA to lock to the new multiplier (bit 5 of register 177)
    err = si_write(SI5351_PLL_RESET, 0x20);
    ESP_LOGI(TAG, "SI5351 PLL locked: xtal=%lu Hz a=%lu vco=%lu Hz", (unsigned long)_si_xtal, (unsigned long)a, (unsigned long)_si_vco);
    return err;
}

// Band parameter cache for the Si5351 tone-generation scheme.
// Pre-computed once per band change in si_cache_band(); used in si_apply_tone()
// to update only the PLL fractional numerator (b) without touching the divider.
// This avoids the PLL reset glitch that would occur if we reprogrammed everything
// on each of the 162 WSPR symbols.
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
static uint32_t _si_last_set_hz = 0u; // avoids re-running si_cache_band() on repeated calls

/**
 * @brief Write all 8 PLL Multisynth (MSNA) registers for fractional mode.
 *
 * Computes AN619 parameters P1, P2, P3 from the a+b/c fractional divider
 * and writes them to registers 26-33.
 * P1 = 128*a + floor(128*b/c) - 512
 * P2 = 128*b - c * floor(128*b/c)
 * P3 = c
 *
 * @param pll_a Integer part of the PLL multiplier (15-90).
 * @param pll_b Fractional numerator (0 to pll_c-1).
 * @param pll_c Fractional denominator (typically 1048575 for max resolution).
 * @return ESP_OK on success.
 */
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

/**
 * @brief Write only P1 and P2 PLL register fields (skip P3 / denominator).
 *
 * During WSPR symbol transmission, only the PLL numerator (b) changes between
 * symbols; the denominator (c = P3) is constant for the entire transmission.
 * Updating only the 6 bytes that encode P1 and P2 halves the I2C traffic per
 * symbol compared to writing all 8 bytes and avoids touching the P3 field,
 * which would needlessly interrupt the PLL phase-lock loop.
 *
 * Writes to MSNA registers 28-33 (skipping registers 26-27 which hold P3[15:8]
 * and P3[7:0] — the denominator that doesn't change between tones).
 *
 * @param pll_a Integer part of the PLL multiplier.
 * @param pll_b New fractional numerator for this tone.
 * @param pll_c Fractional denominator (unchanged; used only for the calculation).
 * @return ESP_OK on success.
 */
static esp_err_t si_write_pll_p1p2_only(uint32_t pll_a, uint32_t pll_b, uint32_t pll_c) {
    uint32_t floor_128b_c = (128u * pll_b) / pll_c;
    uint32_t p1 = 128u * pll_a + floor_128b_c - 512u;
    uint32_t p2 = 128u * pll_b - pll_c * floor_128b_c;

    uint8_t regs[6] = {
        (uint8_t)((p1 >> 16) & 0x03u), (uint8_t)((p1 >> 8) & 0xFFu), (uint8_t)(p1 & 0xFFu), (uint8_t)(((pll_c >> 12) & 0xF0u) | ((p2 >> 16) & 0x0Fu)),
        (uint8_t)((p2 >> 8) & 0xFFu),  (uint8_t)(p2 & 0xFFu),
    };

    // Start write at MSNA+2 (register 28) to skip the P3 bytes
    return si_write_bulk(SI5351_PLLA_MSNA + 2u, regs, 6);
}

/**
 * @brief Program the MS0 output Multisynth in integer mode with an R-divider.
 *
 * Sets MS0 to integer mode (MS0_INT bit = 1, b=0, c=1) for minimum jitter.
 * The R-divider extends the frequency range down to 8 kHz by dividing the
 * MS0 output by 2^r_div_reg (1, 2, 4, 8, 16, 32, 64, or 128).
 *
 * AN619 P1 for integer-only MS0: P1 = 128*d_int - 512  (b=0, floor(128*0/1)=0)
 * Register 44 layout (AN619 Table 9):
 *   Bit 7:   Reserved (0)
 *   Bit 6:   MS0_INT (1 = integer mode)
 *   Bits 5:3 R0_DIV[2:0] (R-divider exponent, divide by 2^r_div_reg)
 *   Bit 2:   Reserved (0)
 *   Bits 1:0 MS0_P1[17:16]
 *
 * @param d_int     Even integer output divider (8-2048 even, or 4/6/8).
 * @param r_div_reg R-divider exponent 0-7 (output division = 2^r_div_reg).
 * @return ESP_OK on success.
 */
static esp_err_t si_write_ms0_integer(uint32_t d_int, uint8_t r_div_reg) {
    uint32_t p1 = 128u * d_int - 512u;
    uint8_t regs[8] = {
        0x00,
        0x01,
        // Reg 44: bit6=MS0_INT | bits[5:3]=R0_DIV[2:0] | bits[1:0]=P1[17:16]
        (uint8_t)(0x40u | ((r_div_reg & 0x07u) << 3) | ((p1 >> 16) & 0x03u)),
        (uint8_t)((p1 >> 8) & 0xFF),
        (uint8_t)(p1 & 0xFF),
        0x00,
        0x00,
        0x00,
    };

    return si_write_bulk(SI5351_MS0_BASE, regs, 8);
}

// Pure 32-bit helper: compute floor(vco_rem * pll_c / xtal).
// Algorithm: binary long division producing 20 quotient bits (pll_c = 2^20 - 1).
//   Each step: scaled_rem *= 2; if >= xtal, set bit and subtract xtal.
//   scaled_rem < xtal always, so scaled_rem*2 < 2*40e6 = 80e6 < 2^27 -- no overflow.
//   After 20 steps, result = floor(vco_rem * 2^20 / xtal).
//   Adjust for pll_c = 2^20-1 (not 2^20): subtract 1 when final remainder < vco_rem.
//   Max error vs 64-bit rounded result: 1 LSB = xtal/pll_c <= 38 mHz.
//   WSPR tone spacing = 1464 mHz, so the error is negligible.
static uint32_t si_muldiv_pllb_32(uint32_t vco_rem, uint32_t xtal) {
    uint32_t scaled_rem = vco_rem;
    uint32_t result = 0;
    int i;
    // 20 iterations: compute floor(vco_rem * 2^20 / xtal) via shift-and-subtract
    for (i = 19; i >= 0; i--) {
        scaled_rem <<= 1; // safe: scaled_rem < xtal <= 40e6; *2 <= 80e6 < 2^27
        if (scaled_rem >= xtal) {
            result |= (1u << i);
            scaled_rem -= xtal;
        }
    }
    // Correct for pll_c = 2^20-1 vs the 2^20 implied by 20-bit binary division.
    // floor(vco_rem*(2^20-1)/xtal) = floor(vco_rem*2^20/xtal) - (remainder < vco_rem ? 1 : 0)
    if (scaled_rem < vco_rem) {
        result--;
    }
    return result;
}

/**
 * @brief Pre-compute all parameters needed to shift WSPR tones for a given band.
 *
 * Called once when the transmit frequency changes (band hop or first use).
 * Computes the best even-integer MS0 divider and the fractional PLL parameters
 * that place the carrier at freq_hz, then caches the incremental delta_b
 * value that must be added per milli-Hz of tone offset.
 *
 * The key insight for WSPR: each 4-FSK tone is only ~1.46 Hz away from the
 * previous one, so instead of recomputing full PLL parameters per symbol,
 * we compute a linear approximation delta_b = d_int * pll_c / xtal_hz and
 * add scaled multiples of it to pll_b_base for each symbol.  This keeps the
 * per-symbol I2C write to just 6 bytes (P1 + P2 fields only).
 *
 * Crystal calibration (ppb) is applied by scaling the effective VCO frequency
 * before the divider calculation, so the output frequency tracks the correction.
 *
 * @param freq_hz  Desired carrier frequency in Hz (base tone, no offset yet).
 * @return ESP_OK on success; error code if frequency is out of Si5351 range.
 */
static esp_err_t si_cache_band(uint32_t freq_hz) {
    if (_si_vco == 0) {
        ESP_LOGE(TAG, "SI5351: PLL not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Guard against zero frequency at function entry.
    // If freq_hz is 0, eff_hz stays 0 through all 7 iterations of the R-divider
    // doubling loop (0 * 2 == 0 always), so the loop exits with eff_hz still 0.
    // The subsequent vco_cal / eff_hz then triggers a fatal Xtensa divide-by-zero
    // hardware exception (no C undefined-behavior sanitizer catches this on-target).
    // The in-loop "eff_hz == 0" check only detects non-zero values that wrap to 0
    // on overflow; it cannot catch the zero-at-entry case.
    if (freq_hz == 0u) {
        ESP_LOGE(TAG, "SI5351: zero frequency requested");
        return ESP_ERR_INVALID_ARG;
    }

    // Apply crystal calibration: shift the effective VCO by the ppb correction.
    // A positive ppb means the crystal runs fast, so we lower the effective VCO
    // to compensate and produce the correct output frequency.
    uint32_t vco_cal = _si_vco;
    if (_si_cal != 0) {
        // corr_Hz = VCO_Hz * ppb / 1,000,000,000
        // Decompose VCO into MHz and kHz-remainder parts to stay in 32-bit:
        //   VCO_Hz * ppb / 1e9
        //   = (vco_mhz * 1e6 + vco_khz_rem * 1e3) * ppb / 1e9
        //   = vco_mhz * ppb / 1e3  +  vco_khz_rem * ppb / 1e6
        // Range check: 900 MHz * 100000 ppb / 1000 = 90,000,000 — fits uint32_t.
        int32_t ppb_s = _si_cal;
        uint32_t ppb_abs = (ppb_s >= 0) ? (uint32_t)ppb_s : (uint32_t)(-ppb_s);
        uint32_t vco_mhz = _si_vco / 1000000u;
        uint32_t vco_khz_rem = (_si_vco % 1000000u) / 1000u;
        int32_t corr = (int32_t)(vco_mhz * ppb_abs / 1000u) + (int32_t)(vco_khz_rem * ppb_abs / 1000000u);
        if (ppb_s < 0)
            corr = -corr;
        vco_cal = (uint32_t)((int32_t)_si_vco - corr);
    }

    // R-divider: scale up frequencies below 500 kHz by powers of 2 until they
    // are within the MS0 valid range (500 kHz to 200 MHz).  r_div_reg is the
    // exponent written to the register; the hardware divides by 2^r_div_reg.
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

    // Compute the output Multisynth divider d_int = VCO / eff_hz.
    // Force to an even integer: odd dividers add additional jitter in the Si5351.
    uint32_t d_int = vco_cal / eff_hz;
    if (d_int & 1u) {
        d_int &= ~1u; // round down to even
    }

    // Clamp to valid MS0 integer-mode range (8-2048 for even values)
    if (d_int < 8UL)
        d_int = 8UL;

    if (d_int > 2048UL) {
        ESP_LOGE(TAG, "SI5351: divider %lu out of range after even-force (vco=%lu eff_hz=%lu)", (unsigned long)d_int, (unsigned long)vco_cal,
                 (unsigned long)eff_hz);
        return ESP_ERR_INVALID_ARG;
    }

    // Compute the PLL fractional numerator for the base tone (offset = 0 mHz).
    // Use c = 1048575 (maximum) for the finest possible frequency resolution.
    // The PLL VCO target is eff_hz * d_int; the fractional part comes from the
    // remainder when dividing VCO_target by xtal_Hz.
    uint32_t pll_c = 1048575UL;
    // Compute vco_target in pure 32-bit.
    // PROOF it fits: eff_hz * d_int ≈ vco_cal ≈ 870 MHz.
    // Upper bound: eff_hz*d_int <= vco_cal + eff_hz <= 870,087,000 + 28,127,600 = 898,214,600
    // 898,214,600 < 2^30 (1,073,741,824) -- safe for uint32_t.
    uint32_t vco_target = eff_hz * d_int;
    uint32_t pll_a = vco_target / _si_xtal;
    uint32_t vco_remainder = vco_target - pll_a * _si_xtal;
    // Compute pll_b_base via pure 32-bit binary long division.
    // si_muldiv_pllb_32() replaces (vco_remainder * pll_c + xtal/2) / xtal which
    // requires 64-bit intermediate (vco_remainder * 1048575 up to ~41.9e12).
    // Max error vs original rounded 64-bit result: 1 LSB (floor vs round, diff <= 1).
    // 1 LSB = xtal/pll_c = 23.8 mHz (25 MHz xtal) -- negligible vs 1464 mHz tone step.
    uint32_t pll_b_base = si_muldiv_pllb_32(vco_remainder, _si_xtal);
    if (pll_b_base >= pll_c)
        pll_b_base = pll_c - 1u;

    // Pre-compute the b-increment per milli-Hz of tone offset.
    // delta_b = d_int * pll_c / xtal_Hz  (per Hz).
    // Decomposed into integer parts to stay in 32-bit arithmetic at runtime:
    //   b_step_int   = floor(d_int * pll_c / xtal_Hz)        [per Hz, integer part]
    //   b_step2_int  = floor(remainder / xtal_kHz)            [per Hz, sub-part]
    //   b_step2_rem  = remainder % xtal_kHz                   [per Hz, residual]
    // si_apply_tone() multiplies these by the scaled tone offset in milli-Hz.
    uint32_t xtal_kHz = _si_xtal / 1000UL;
    uint32_t full_den = xtal_kHz * 1000UL; // = xtal_Hz
    uint32_t raw_num = d_int * pll_c;
    uint32_t b_step_int = raw_num / full_den;
    uint32_t b_step_rem1 = raw_num % full_den;
    uint32_t b_step2_int = b_step_rem1 / xtal_kHz;
    uint32_t b_step2_rem = b_step_rem1 % xtal_kHz;

    // Verify the PLL multiplier is within the Si5351 valid range (15-90)
    if (pll_a < 15UL || pll_a > 90UL) {
        ESP_LOGE(TAG, "SI5351: PLL multiplier %lu out of range for eff_hz=%lu d_int=%lu", (unsigned long)pll_a, (unsigned long)eff_hz, (unsigned long)d_int);
        return ESP_ERR_INVALID_ARG;
    }

    // Store all computed values in the cache for use by si_apply_tone()
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

    // Program the output Multisynth divider (MS0) in integer mode with the R-divider
    esp_err_t err = si_write_ms0_integer(d_int, r_div_reg);
    if (err != ESP_OK)
        return err;

    // Program the full PLL Multisynth (MSNA) with the base numerator
    err = si_write_pll_regs(pll_a, pll_b_base, pll_c);
    if (err != ESP_OK)
        return err;

    // Reset PLLA to lock to the new parameters
    return si_write(SI5351_PLL_RESET, 0x20);
}

/**
 * @brief Apply a WSPR tone offset to the Si5351 PLL by updating only pll_b.
 *
 * Computes the new PLL fractional numerator for the given tone_millihz offset
 * from the base frequency using the cached b-step values, then writes only the
 * P1 and P2 register fields (6 bytes) without touching the P3 denominator or
 * the MS0 divider.  This produces a continuous-phase frequency change with
 * no PLL glitch, as required by the WSPR 4-FSK modulation scheme.
 *
 * The R-divider exponent is factored in by left-shifting tone_millihz:
 *   scaled_tone = tone_millihz << r_div_reg
 * because the MS0 R-divider reduces the effective step size by 2^r_div_reg.
 *
 * @param tone_millihz  Tone offset in milli-Hz above the base frequency (0 to
 *                      ~4395 mHz for WSPR tones 0-3: 0, 1465, 2930, 4395 mHz).
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if the cache is not valid.
 */
static esp_err_t si_apply_tone(uint32_t tone_millihz) {
    if (!_si_cache.valid) {
        ESP_LOGE(TAG, "SI5351: si_apply_tone called before si_cache_band");
        return ESP_ERR_INVALID_STATE;
    }

    // Scale the milli-Hz offset to account for the R-divider
    uint32_t scaled_tone = tone_millihz << _si_cache.r_div_reg;

    // Compute delta_b using the 3-term decomposition to stay within 32-bit arithmetic
    uint32_t delta_b = scaled_tone * _si_cache.b_step_int;
    delta_b += (scaled_tone * _si_cache.b_step2_int) / 1000UL;
    delta_b += (scaled_tone * _si_cache.b_step2_rem) / _si_cache.full_den;
    uint32_t pll_b = _si_cache.pll_b_base + delta_b;

    // Clamp to valid range [0, pll_c - 1]
    if (pll_b >= _si_cache.pll_c) {
        pll_b = _si_cache.pll_c - 1u;
    }

    // Write only the P1+P2 fields; skip P3 (denominator doesn't change)
    return si_write_pll_p1p2_only(_si_cache.pll_a, pll_b, _si_cache.pll_c);
}

/**
 * @brief Attempt to detect and initialize the Si5351A clock generator.
 *
 * Creates an I2C master bus, adds the Si5351 device at address 0x60,
 * probes with a register 0 read, then configures CLK0 and locks the PLL.
 * Cleans up and returns false if any step fails.
 *
 * @return true if the Si5351 was found and initialized successfully.
 */
static bool si5351_try_init(void) {
    // Create the I2C master bus on the configured port and GPIO pins
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
    // Add the Si5351 device at 400 kHz fast-mode I2C
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI5351_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(si_bus_handle, &dev_cfg, &si_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        i2c_del_master_bus(si_bus_handle);
        return false;
    }
    // Probe: read register 0 to confirm the Si5351 is physically present
    esp_err_t probe = si5351_ping();
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "SI5351 not detected on I2C addr 0x%02X (err=%s)", SI5351_ADDR, esp_err_to_name(probe));
        i2c_master_bus_rm_device(si_dev_handle);
        i2c_del_master_bus(si_bus_handle);
        si_dev_handle = NULL;
        si_bus_handle = NULL;
        return false;
    }
    // Disable all outputs while configuring (register 3: output enable, 1=disabled)
    si_write(3, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Configure CLK0: source PLLA, integer mode, chosen drive current
    si_write(SI5351_CLK0_CTRL, SI5351_CLK0_CTRL_VAL);
    // Lock PLLA to the crystal with an integer multiplier
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
//
// The AD9850 is a complete Direct Digital Synthesis (DDS) chip from Analog Devices.
// It contains a 32-bit phase accumulator, a sine ROM, a 10-bit DAC, and a comparator.
// The output frequency is set by the Frequency Tuning Word (FTW):
//
//   f_out = FTW * f_ref / 2^32
//
// Therefore:  FTW = f_out * 2^32 / f_ref
//
// With f_ref = 125 MHz:  FTW = f_out * 4294967296 / 125000000
//                             = f_out * 34.35973...
// Resolution = f_ref / 2^32 = 125e6 / 4294967296 ≈ 0.0291 Hz per LSB.
//
// Programming is done serially: 32 bits of FTW (LSB first) followed by 8 bits
// of phase/control (LSB first), then a pulse on FQ_UD to latch the new word.
// The AD9850 has no readback capability; its presence can only be assumed.
//
// References:
//   - AD9850 datasheet, Rev. H (Analog Devices)

#define AD9850_CLK_GPIO   CONFIG_AD9850_CLK_GPIO   // W_CLK: serial shift clock
#define AD9850_FQUD_GPIO  CONFIG_AD9850_FQ_UD_GPIO // FQ_UD: frequency update strobe
#define AD9850_DATA_GPIO  CONFIG_AD9850_DATA_GPIO  // D7: serial data input (LSB first)
#define AD9850_RESET_GPIO CONFIG_AD9850_RESET_GPIO // RESET: active-high master reset
#define AD9850_REF_CLK    CONFIG_AD9850_REF_CLOCK  // Reference oscillator frequency (Hz)

// Last programmed frequency state (needed to re-enable from power-down)
static uint32_t _ad_last_hz = 0u;
static uint32_t _ad_last_frac = 0u; // sub-Hz fractional part in milli-Hz / 1000
static int32_t _ad_cal = 0;         // calibration offset in ppb
// Critical-section lock for the bit-bang serial interface (called from normal task context)
static portMUX_TYPE _ad_mux = portMUX_INITIALIZER_UNLOCKED;

// Pre-computed Frequency Tuning Word constants for supported reference clocks.
// The FTW is split into an integer per-MHz part and a fractional per-Hz part
// to keep all arithmetic in 32-bit integers and avoid ULL at runtime on ESP32.
//
// FTW_PER_MHZ = floor(2^32 / (f_ref / 1e6))
// FTW_INT_PER_HZ = floor(2^32 / f_ref)
// FTW_FRAC_NUM / FTW_FRAC_DEN = fractional residual of FTW_INT_PER_HZ
#if AD9850_REF_CLK == 125000000UL
// 125 MHz module (most common): FTW = freq_Hz * 4294967296 / 125000000
#define AD9850_FTW_PER_MHZ    34359738UL // floor(2^32 / 125) = 34359738.368
#define AD9850_FTW_INT_PER_HZ 34UL       // floor(2^32 / 125e6)
#define AD9850_FTW_FRAC_DEN   10000UL
#define AD9850_FTW_FRAC_NUM   3597UL // 0.368 * 10000 ≈ 3597
#elif AD9850_REF_CLK == 100000000UL
// 100 MHz module: pre-computed, no ULL at runtime
#define AD9850_FTW_PER_MHZ    42949672UL // floor(2^32 / 100)
#define AD9850_FTW_INT_PER_HZ 42UL       // floor(2^32 / 100e6)
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

// Combined FTW increment per milli-Hz of tone offset, for sub-Hz AD9850 updates
#define AD9850_FTW_FRAC_PER_MHZ_NUM ((uint32_t)(AD9850_FTW_INT_PER_HZ * AD9850_FTW_FRAC_DEN + AD9850_FTW_FRAC_NUM))
#define AD9850_FTW_FRAC_PER_MHZ_DEN (1000UL * AD9850_FTW_FRAC_DEN)

// Compile-time sanity check: maximum WSPR frequency FTW must fit in 32 bits
static_assert((30UL * (4294967296ULL / ((uint32_t)AD9850_REF_CLK / 1000000UL)) + 999999UL * (4294967296ULL / (uint32_t)AD9850_REF_CLK) +
               999999UL * AD9850_FTW_FRAC_NUM / AD9850_FTW_FRAC_DEN) < 0xFFFFFFFFUL,
              "AD9850 FTW overflow: max WSPR frequency word must fit in 32 bits for the configured ref clock");

// Calibrated FTW constants (updated by oscillator_set_cal())
static uint32_t _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ;
static uint32_t _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ;

/**
 * @brief Compute the 32-bit AD9850 Frequency Tuning Word for a given frequency.
 *
 * Splits the frequency into integer MHz and sub-MHz Hz parts to keep all
 * arithmetic in 32-bit integers.  The fractional residual (FTW_FRAC_NUM /
 * FTW_FRAC_DEN per Hz) is added to correct the truncation error.
 *
 * FTW = freq_MHz * FTW_PER_MHZ
 *      + freq_sub_Hz * FTW_INT_PER_HZ
 *      + freq_sub_Hz * FTW_FRAC_NUM / FTW_FRAC_DEN
 *
 * @param freq_hz  Output frequency in Hz.
 * @return 32-bit Frequency Tuning Word to load into the AD9850.
 */
static uint32_t ad9850_freq_word(uint32_t freq_hz) {
    uint32_t freq_mhz = freq_hz / 1000000UL;    // integer MHz  (0..30)
    uint32_t freq_sub_hz = freq_hz % 1000000UL; // sub-MHz Hz remainder (0..999999)
    uint32_t fw = freq_mhz * _ad_ftw_per_mhz_cal;

    fw += freq_sub_hz * _ad_ftw_int_per_hz_cal;
    fw += (freq_sub_hz * AD9850_FTW_FRAC_NUM) / AD9850_FTW_FRAC_DEN;

    return fw;
}

/**
 * @brief Pulse a GPIO high then low (one rising+falling edge).
 * @param gpio GPIO number to pulse.
 */
static inline void _pulse(int gpio) {
    gpio_set_level(gpio, 1);
    gpio_set_level(gpio, 0);
}

/**
 * @brief Shift a 40-bit word (32-bit FTW + 8-bit phase/control) into the AD9850.
 *
 * The AD9850 accepts serial data LSB first on the D7 (DATA) pin, clocked by
 * rising edges on W_CLK. After all 40 bits are shifted in, a rising edge on
 * FQ_UD latches the word into the active frequency register.
 * The entire operation is wrapped in a FreeRTOS critical section to prevent
 * preemption from corrupting the bit-bang timing.
 *
 * @param freq_word 32-bit Frequency Tuning Word.
 * @param phase     8-bit phase/control byte (bits[7:3] = phase, bit[2] = power-down).
 *                  Pass 0x00 for normal operation; 0x04 to power down the DAC.
 */
static void ad9850_write_word(uint32_t freq_word, uint8_t phase) {
    int i;
    taskENTER_CRITICAL(&_ad_mux);
    // Shift out 32-bit FTW, LSB first
    for (i = 0; i < 32; i++) {
        gpio_set_level(AD9850_DATA_GPIO, (int)((freq_word >> i) & 1u));
        _pulse(AD9850_CLK_GPIO);
    }
    // Shift out 8-bit phase/control byte, LSB first
    for (i = 0; i < 8; i++) {
        gpio_set_level(AD9850_DATA_GPIO, (int)((phase >> i) & 1u));
        _pulse(AD9850_CLK_GPIO);
    }
    // Latch the 40-bit word into the AD9850 frequency register
    _pulse(AD9850_FQUD_GPIO);
    taskEXIT_CRITICAL(&_ad_mux);
}

/**
 * @brief Attempt to initialize the AD9850 DDS via GPIO bit-bang.
 *
 * The AD9850 has no readback interface, so its presence cannot be confirmed.
 * When CONFIG_OSCILLATOR_ASSUME_AD9850 is disabled, this function returns false
 * to allow DUMMY mode on boards without an AD9850 fitted.
 *
 * Reset sequence per datasheet: RESET high -> low, then pulse CLK, then pulse FQ_UD.
 *
 * @return true if initialization was performed (AD9850 assumed present).
 */
static bool ad9850_try_init(void) {
#if !CONFIG_OSCILLATOR_ASSUME_AD9850
    ESP_LOGW(TAG, "AD9850 assumption disabled in Kconfig -- skipping AD9850 init, entering DUMMY mode");
    return false;
#else
    // Configure all four AD9850 control GPIOs as push-pull outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(AD9850_CLK_GPIO) | BIT64(AD9850_FQUD_GPIO) | BIT64(AD9850_DATA_GPIO) | BIT64(AD9850_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // Master reset: assert RESET high for at least 1 ms, then release
    gpio_set_level(AD9850_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(AD9850_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    // Clock and FQ_UD pulses complete the serial-mode reset sequence
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
    // Try Si5351 first (I2C ACK confirms hardware presence)
    if (si5351_try_init()) {
        _osc_type = OSC_SI5351;
        _osc_hw_ok = true;
        ESP_LOGI(TAG, "Auto-detected: SI5351");
        return ESP_OK;
    }
    // Try AD9850 (write-only; assumed present if Kconfig allows it)
    if (ad9850_try_init()) {
        _osc_type = OSC_AD9850;
        _osc_hw_ok = true;
        ESP_LOGI(TAG, "Auto-detected: AD9850 (assumed)");
        return ESP_OK;
    }
    // No hardware found: enter DUMMY mode (all operations silently succeed)
    _osc_type = OSC_NONE;
    _osc_hw_ok = false;
    ESP_LOGW(TAG, "No oscillator hardware found — DUMMY mode active");
    return ESP_OK;
}

esp_err_t oscillator_set_freq(uint32_t freq_hz) {
    if (_osc_type == OSC_SI5351) {
        // Avoid redundant si_cache_band() calls if the frequency hasn't changed
        if (freq_hz == _si_last_set_hz && _si_cache.valid) {
            return si_apply_tone(0u); // reapply base tone (offset = 0)
        }

        // Recompute dividers and PLL parameters for the new band/frequency
        esp_err_t err = si_cache_band(freq_hz);
        if (err == ESP_OK)
            _si_last_set_hz = freq_hz;
        return err;
    }
    if (_osc_type == OSC_AD9850) {
        // For the AD9850, compute and write the FTW directly
        _ad_last_hz = freq_hz;
        _ad_last_frac = 0u;
        ad9850_write_word(ad9850_freq_word(freq_hz), 0u);
    }
    return ESP_OK;
}

esp_err_t oscillator_set_freq_mhz(uint32_t base_hz, int32_t offset_millihz) {
    if (_osc_type == OSC_SI5351) {
        // For Si5351: update only the PLL numerator b for the new tone offset.
        // The base_hz parameter is not needed here because the band cache already
        // encodes the base frequency; only the offset changes between WSPR symbols.
        uint32_t tone_mhz = (offset_millihz >= 0) ? (uint32_t)offset_millihz : 0u;
        return si_apply_tone(tone_mhz);
    }

    if (_osc_type == OSC_AD9850) {
        // For AD9850: combine base_hz and milli-Hz offset into a new FTW.
        // The sub-kHz part of base_hz and the offset are merged carefully to
        // maintain milli-Hz precision without 64-bit arithmetic at runtime.
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
        // Compute integer-Hz FTW then add the milli-Hz fractional increment.
        // Use _ad_ftw_int_per_hz_cal (calibrated at runtime by
        // oscillator_set_cal()) instead of the compile-time AD9850_FTW_FRAC_PER_MHZ_NUM
        // which embeds the uncalibrated AD9850_FTW_INT_PER_HZ constant.
        // After a ppb calibration, _ad_ftw_int_per_hz_cal diverges from
        // AD9850_FTW_INT_PER_HZ; using the uncalibrated constant would apply
        // a slightly wrong FTW delta for each mHz tone offset, causing systematic
        // tone-frequency errors proportional to both the ppb correction and the
        // mHz offset. The calibrated numerator is computed at runtime as:
        //   _ad_ftw_int_per_hz_cal * AD9850_FTW_FRAC_DEN + AD9850_FTW_FRAC_NUM
        // AD9850_FTW_FRAC_DEN (10000) and AD9850_FTW_FRAC_NUM are invariant.
        // Overflow check: max _ad_ftw_int_per_hz_cal ~34, *10000 = 340000,
        // +3597 = 343597; *frac_mhz_max(999) = 343,253,403 < 2^32. Safe.
        uint32_t fw = ad9850_freq_word(out_hz);
        uint32_t ad_ftw_frac_num_cal = _ad_ftw_int_per_hz_cal * AD9850_FTW_FRAC_DEN + AD9850_FTW_FRAC_NUM;
        fw += (frac_mhz * ad_ftw_frac_num_cal) / AD9850_FTW_FRAC_PER_MHZ_DEN;

        ad9850_write_word(fw, 0u);
    }

    return ESP_OK;
}

esp_err_t oscillator_enable(bool en) {
    if (_osc_type == OSC_SI5351) {
        // Register 3 (Output Enable Control): bit 0 = CLK0 disable (active-low logic).
        // 0xFE (1111 1110) enables CLK0; 0xFF disables all outputs.
        uint8_t val = en ? 0xFE : 0xFF;
        return si_write(3, val);
    }
    if (_osc_type == OSC_AD9850) {
        if (en) {
            // Restore the last programmed frequency when coming out of power-down
            if (_ad_last_hz > 0u) {
                uint32_t fw = ad9850_freq_word(_ad_last_hz);
                fw += (_ad_last_frac * AD9850_FTW_FRAC_PER_MHZ_NUM) / AD9850_FTW_FRAC_PER_MHZ_DEN;
                ad9850_write_word(fw, 0x00u); // phase/control = 0: normal operation
            }
        } else {
            // Power down: FTW = 0, control byte bit 2 = 1 (power-down enable)
            ad9850_write_word(0u, 0x04u);
        }
    }
    return ESP_OK;
}

void oscillator_tx_begin(void) {
    // Mark start of TX burst; defers calibration changes until transmission ends
    _osc_tx_active = true;
}

void oscillator_tx_end(void) {
    _osc_tx_active = false;
    // Apply any calibration update that arrived while the TX was in progress
    if (_cal_pending) {
        _cal_pending = false;
        _si_cal = _cal_pending_ppb;
        // Invalidate the band cache so si_cache_band() runs again with new cal
        _si_cache.valid = false;
        _si_last_set_hz = 0u; // force full si_cache_band() on next oscillator_set_freq()
        ESP_LOGI(TAG, "SI5351 deferred cal applied: %ld ppb", (long)_cal_pending_ppb);
    }
}

esp_err_t oscillator_set_cal(int32_t ppb) {
    if (_osc_type == OSC_SI5351) {
        // Defer calibration if a TX is in progress to avoid mid-symbol frequency jumps
        if (_osc_tx_active) {
            _cal_pending = true;
            _cal_pending_ppb = ppb;
            ESP_LOGW(TAG, "SI5351 cal deferred (TX active): %ld ppb", (long)ppb);
            return ESP_OK;
        }

        _si_cal = ppb;
        // Invalidate cache so next oscillator_set_freq() recomputes with new cal
        _si_cache.valid = false;
        _si_last_set_hz = 0u;
        ESP_LOGI(TAG, "SI5351 cal set: %ld ppb", (long)ppb);
    } else if (_osc_type == OSC_AD9850) {
        // For AD9850: adjust the pre-computed FTW constants proportionally.
        // A positive ppb means the crystal runs fast, so we decrease the FTW
        // constants to output a slightly lower frequency and compensate.
        _ad_cal = ppb;

        // delta = FTW_constant * ppb / 1,000,000,000
        // Split into two 32-bit-safe multiplications to avoid 64-bit arithmetic:
        //   FTW * ppb / 1e9 = (FTW / 1000) * (ppb / 1000)
        //                   + (FTW / 1000) * (ppb % 1000) / 1000
        //                   + (FTW % 1000) * (ppb / 1000) / 1000
        // For AD9850_FTW_PER_MHZ ≈ 34,359,738 and ppb <= 100,000:
        //   largest intermediate: (34359738/1000) * (100000/1000) = 34359 * 100 = 3,435,900 — fits uint32_t.
        uint32_t ppb_abs = (ppb >= 0) ? (uint32_t)ppb : (uint32_t)(-(int32_t)ppb);
        uint32_t ftw_mhz_q = AD9850_FTW_PER_MHZ / 1000u;
        uint32_t ftw_mhz_r = AD9850_FTW_PER_MHZ % 1000u;
        uint32_t ppb_q = ppb_abs / 1000u;
        uint32_t ppb_r = ppb_abs % 1000u;
        uint32_t delta_per_mhz = ftw_mhz_q * ppb_q + (ftw_mhz_q * ppb_r) / 1000u + (ftw_mhz_r * ppb_q) / 1000u;

        uint32_t ftw_hz_q = AD9850_FTW_INT_PER_HZ / 1000u;
        uint32_t ftw_hz_r = AD9850_FTW_INT_PER_HZ % 1000u;
        uint32_t delta_per_hz = ftw_hz_q * ppb_q + (ftw_hz_q * ppb_r) / 1000u + (ftw_hz_r * ppb_q) / 1000u;

        if (ppb > 0) {
            // Crystal runs fast: reduce FTW to output a lower (corrected) frequency
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ - delta_per_mhz;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ - delta_per_hz;
        } else if (ppb < 0) {
            // Crystal runs slow: increase FTW to output a higher (corrected) frequency
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ + delta_per_mhz;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ + delta_per_hz;
        } else {
            // Zero correction: restore nominal FTW constants
            _ad_ftw_per_mhz_cal = AD9850_FTW_PER_MHZ;
            _ad_ftw_int_per_hz_cal = AD9850_FTW_INT_PER_HZ;
        }
        ESP_LOGI(TAG, "AD9850 cal set: %ld ppb -> FTW/MHz=%lu FTW/Hz=%lu", (long)ppb, (unsigned long)_ad_ftw_per_mhz_cal,
                 (unsigned long)_ad_ftw_int_per_hz_cal);
    } else {
        // DUMMY mode: store the value so it can be retrieved later
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
