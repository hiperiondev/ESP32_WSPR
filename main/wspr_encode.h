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

#pragma once

#include <stdint.h>

/**
 * @file wspr_encode.h
 * @brief WSPR Type-1 message encoder.
 *
 * Encodes a callsign, 4-character Maidenhead locator, and TX power level into
 * the 162-symbol 4-FSK sequence defined by the Weak Signal Propagation Reporter
 * (WSPR) protocol, revision 2.
 *
 * @par Protocol summary (G4JNT / K1JT specification)
 * A WSPR Type-1 message carries:
 *  - A 28-bit packed callsign (@c n_call).
 *  - A 15-bit packed locator (@c n_loc).
 *  - A 7-bit packed power value (@c n_pwr = dBm + 64).
 *
 * The 50 data bits are convolutionally encoded with a rate-1/2, K=32 code
 * (polynomials @c G1=0xF2D05351, @c G2=0xE4613C47) producing 162 coded bits.
 * A bit-reversal interleaver then permutes the coded bits, after which each
 * bit is combined with the corresponding element of a fixed 162-bit sync
 * vector to produce the final 4-FSK symbol (0–3):
 *
 * @code
 *   symbol[i] = 2 * interleaved_bit[i] + sync[i]
 * @endcode
 *
 * @par Tone spacing and timing
 *  - Tone spacing : 12000 / 8192 Hz ≈ 1.4648 Hz
 *  - Symbol period: 8192 / 12000 s ≈ 682.667 ms
 *  - Total TX time: 162 symbols ≈ 110.6 s
 *  - Carrier offset: dial frequency + 1500 Hz (audio passband centre)
 *
 * @par Callsign packing rules
 * The encoder follows the G4JNT packing standard:
 *  1. The callsign is right-padded with spaces to 6 characters.
 *  2. If the second character (index 1) is a letter, a leading space is
 *     prepended (so @c "G4JNT" becomes @c " G4JNT").
 *  3. Character at position 2 (0-indexed) **must** be a digit after the
 *     above normalisation.
 *  4. Positions 0–1 use the 37-symbol alphabet (0–9, A–Z, space).
 *  5. Positions 3–5 use the 27-symbol suffix alphabet (A–Z, space).
 *
 * @par Locator encoding
 * Only 4-character Maidenhead grid squares are supported (WSPR Type-1 format).
 * A 6-character sub-square will cause @ref wspr_encode() to return @c -1.
 *
 * @par Power encoding
 * The power value is rounded to the nearest valid WSPR level from the set:
 * { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 }.
 *
 * @par Implementation notes
 * The entire encoder uses 32-bit integer arithmetic only; no floating-point
 * operations are performed, making it suitable for the ESP32 Xtensa LX6 core
 * without the FPU penalty of @c -mfloat-abi=softfp.
 */

/**
 * @defgroup wspr_encode_api WSPR encoder API
 * @{
 */

/**
 * @brief Total number of 4-FSK symbols in one WSPR Type-1 transmission.
 *
 * 162 = 81 encoded input bits × rate-1/2 convolutional code.
 * (50 data bits + 31 convolutional tail bits) × 2 = 162 output bits / symbols.
 */
#define WSPR_SYMBOLS 162

/**
 * @brief Encode a WSPR Type-1 message into 162 4-FSK symbols.
 *
 * Performs the complete WSPR encoding pipeline:
 *  1. Pack callsign → 28-bit integer (@c n_call).
 *  2. Pack locator  → 15-bit integer (@c n_loc).
 *  3. Pack power    → 7-bit integer  (@c n_pwr).
 *  4. Assemble 50-bit message into 7 bytes (left-justified, 6 padding bits).
 *  5. Convolutional encode (K=32, rate 1/2) → 162 coded bits.
 *  6. Bit-reversal interleave → 162 permuted bits.
 *  7. Combine with sync vector → 162 symbols in range [0, 3].
 *
 * The output symbols are written into the caller-supplied buffer @p symbols.
 * Each byte holds one symbol value (0, 1, 2, or 3) corresponding to the four
 * WSPR tone offsets:
 *
 * @code
 *   tone_offset_Hz = symbol * (12000.0 / 8192.0)   // ≈ 0, 1.46, 2.93, 4.39 Hz
 * @endcode
 *
 * The actual output frequency for symbol @c i is therefore:
 * @code
 *   f_out = dial_freq_Hz + 1500 + symbols[i] * 375000 / 256  (milli-Hz arithmetic)
 * @endcode
 *
 * @param[in]  callsign   Null-terminated callsign string (1–6 alphanumeric
 *                        characters).  Leading spaces are inserted
 *                        automatically when required.  Case-insensitive.
 *                        Examples: @c "N0CALL", @c "G4JNT", @c "VK2ABC".
 * @param[in]  locator    Null-terminated 4-character Maidenhead grid square.
 *                        Must match @c [A-Ra-r][A-Ra-r][0-9][0-9].
 *                        6-character sub-squares are rejected.
 *                        Examples: @c "AA00", @c "GF05", @c "EM73".
 * @param[in]  power_dbm  TX power in dBm.  Must be one of the 19 valid WSPR
 *                        levels; values not in the list are silently rounded
 *                        to the nearest valid entry.
 * @param[out] symbols    Output buffer of exactly @ref WSPR_SYMBOLS bytes.
 *                        On success, each byte is in the range [0, 3].
 *                        The buffer must be allocated by the caller.
 *
 * @retval  0  Encoding succeeded; @p symbols is ready to transmit.
 * @retval -1  Input validation failed:
 *             - @p callsign is NULL, empty, longer than 6 characters, contains
 *               invalid characters, or cannot be normalised to a valid WSPR
 *               callsign (position 2 not a digit after padding).
 *             - @p locator is NULL, shorter than 4 characters, has invalid
 *               characters, or contains a 6-character sub-square.
 */
int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]);

/** @} */
