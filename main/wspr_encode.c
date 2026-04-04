/**
 * @file wspr_encode.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @brief WSPR message encoder — Type-1, Type-2, and Type-3 messages.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

// WSPR (Weak Signal Propagation Reporter) encoding overview — G4JNT / K1JT spec
//
// A WSPR transmission carries 50 bits of payload packed from three source fields:
//   28 bits — callsign (n_call)
//   15 bits — Maidenhead locator (n_loc)
//    7 bits — TX power in dBm encoded as (dBm + 64)  (n_pwr)
//
// The 50-bit payload is processed through this pipeline:
//   1. Pack  : compact source fields into a 7-byte (56-bit) array, MSB-first,
//              left-aligned.  Only bits 0..49 carry data; bits 50..80 are zero
//              (31 tail bits that flush the convolutional encoder shift registers).
//   2. Convolve : rate-1/2 non-recursive convolutional code, constraint length K=32,
//              generators 0xF2D05351 and 0xE4613C47.  81 input bits → 162 output bits.
//   3. Interleave : bit-reversal permutation of the 162 coded bits to scatter
//              burst errors into random-error patterns the FEC can correct.
//   4. Sync merge : symbol[i] = 2 * data_bit[i] + SYNC[i], producing 162 values
//              in the range 0..3 ready for 4-FSK modulation.
//
// Modulation parameters (WSPR 2.0 spec, K1JT):
//   Keying rate   : 12000 / 8192 ≈ 1.4648 baud
//   Tone spacing  : 1.4648 Hz  (375000 / 256 mHz per tone index unit)
//   Symbol period : 8192 / 12000 ≈ 0.6827 s
//   Total TX time : 162 × 0.6827 ≈ 110.6 s
//   Occupied BW   : ~6 Hz
//   Start time    : 1 s after even UTC minute (hh:mm:01)

#include <ctype.h>
#include <string.h>

#include "esp_log.h"
#include "wspr_encode.h"

static const char *TAG = "wspr_enc";

// WSPR 162-bit pseudo-random synchronisation vector (G4JNT / K1JT spec).
// Each bit is the LSB of the corresponding 4-FSK symbol; the data bit occupies the MSB.
// The vector has good autocorrelation properties so receivers can determine the
// exact start of the frame even at signal-to-noise ratios as low as −28 dB in 2.5 kHz BW.
// Merging formula:  symbol[i] = 2 * data_bit[i] + SYNC[i]   (values 0..3)
static const uint8_t SYNC[WSPR_SYMBOLS] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0,
                                            0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1,
                                            0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
                                            1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 };

// ---------------------------------------------------------------------------
// Character value helpers (shared by all message types)
// ---------------------------------------------------------------------------

/**
 * @brief Map a single callsign character to its WSPR integer value (0..36).
 *
 * The WSPR character set has 37 members:
 *   '0'..'9' → 0..9
 *   'A'..'Z' → 10..35  (input is forced to upper-case)
 *   ' '      → 36
 * Any character outside this set returns -1 (invalid).
 *
 * These values are used when building n_call with the polynomial
 *   n = n * 36 * 10 * 27 * 27 * 27 coefficient chain (see pack_callsign_type1).
 *
 * @param c  Input character (any case for letters).
 * @return   WSPR value 0..36, or -1 for an invalid character.
 */
static int char_to_wspr(char c) {
    c = toupper((unsigned char)c);
    if (c >= '0' && c <= '9')
        return (int)(c - '0');
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A') + 10;
    if (c == ' ')
        return 36;
    return -1;
}

/**
 * @brief Map a callsign suffix/tail character to its 27-value field index (0..26).
 *
 * Positions 3, 4, 5 of a padded WSPR callsign can only be letters (A..Z) or
 * space, giving 27 possible values.  Numbers are not allowed in these positions.
 *   'A'..'Z' → 0..25
 *   ' '      → 26
 *
 * @param c  Input character (any case for letters).
 * @return   Index 0..26 (26 = space / blank).
 */
static int suffix_val(char c) {
    c = toupper((unsigned char)c);
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A');
    return 26; // space
}

/**
 * @brief Return the decimal digit value of a character, or -1 if not a digit.
 *
 * Used to validate and extract the numeric characters in Maidenhead locators
 * and in callsign position 2 (which must always be a digit per WSPR rules).
 *
 * @param c  Character to test ('0'..'9').
 * @return   0..9 for valid digits, -1 otherwise.
 */
static int digit_val(char c) {
    return (c >= '0' && c <= '9') ? (c - '0') : -1;
}

/**
 * @brief Pack a Type-1 (simple) callsign into the 28-bit WSPR integer n_call.
 *
 * WSPR Type-1 callsigns must satisfy these constraints (G4JNT / K1JT spec):
 *   Position 0 : any of 37 values (A-Z, 0-9, space)
 *   Position 1 : any of 36 values (A-Z, 0-9; NOT space)
 *   Position 2 : must be a digit 0-9  (10 values only)
 *   Positions 3-5 : letters or space  (27 values each)
 *
 * If the third character is not a digit the callsign is right-shifted by one
 * position and a leading space is prepended, so e.g. "G4JNT" → " G4JNT".
 * Short callsigns are space-padded to 6 characters on the right.
 *
 * The integer n_call is built by the polynomial:
 *   n = c0
 *   n = n*36 + c1
 *   n = n*10 + c2
 *   n = n*27 + c3
 *   n = n*27 + c4
 *   n = n*27 + c5
 * where c0..c5 are the WSPR character values from char_to_wspr() / suffix_val().
 * Maximum value: 37*36*10*27*27*27 = 262,177,560  (<  2^28 = 268,435,456).
 *
 * @param cs   NUL-terminated callsign string (1-6 characters, no '/').
 * @param out  Receives the packed 28-bit integer n_call on success.
 * @return 0 on success, -1 if the callsign cannot be packed (too long,
 *         contains '/', or position 2 cannot be made a digit).
 */
static int pack_callsign_type1(const char *cs, uint32_t *out) {
    // Start with a 6-character space-padded buffer
    char buf[7] = "      ";
    int len = (int)strlen(cs);

    if (len < 1 || len > 6)
        return -1;
    memcpy(buf, cs, (size_t)len);
    buf[6] = '\0';

    // Compound callsigns (with '/') must not reach this function
    for (int i = 0; i < 6; i++) {
        if (buf[i] == '/')
            return -1;
    }

    // WSPR rule: position 2 must be a digit.
    // If not, prepend a space (shift right) — handles e.g. "G4JNT" → " G4JNT".
    if (!isdigit((unsigned char)buf[2])) {
        if (len > 5)
            return -1; // no room to shift
        for (int i = len; i > 0; i--)
            buf[i] = buf[i - 1];
        buf[0] = ' ';
    }

    // After the shift attempt, position 2 must now be a digit
    if (!isdigit((unsigned char)buf[2]))
        return -1;

    // Extract per-position values using the WSPR encoding alphabet
    int c0 = char_to_wspr(buf[0]); // 37-value field (space allowed)
    int c1 = char_to_wspr(buf[1]); // 36-value field (no space: value <= 35)
    int c2 = (int)(buf[2] - '0');  // digit only — 10 values
    int c3 = suffix_val(buf[3]);   // letter or space — 27 values
    int c4 = suffix_val(buf[4]);   // letter or space — 27 values
    int c5 = suffix_val(buf[5]);   // letter or space — 27 values

    if (c0 < 0 || c1 < 0)
        return -1;

    // c1 must not be a space (space = 36), enforced by the 36-value field limit
    if (c1 > 35)
        return -1;

    // Build the 28-bit integer with the mixed-radix polynomial (G4JNT spec)
    uint32_t n = (uint32_t)c0;
    n = n * 36u + (uint32_t)c1;
    n = n * 10u + (uint32_t)c2;
    n = n * 27u + (uint32_t)c3;
    n = n * 27u + (uint32_t)c4;
    n = n * 27u + (uint32_t)c5;
    *out = n;

    return 0;
}

/**
 * @brief Pack a 4-character Maidenhead locator into the 15-bit WSPR integer n_loc.
 *
 * Maidenhead grid squares are encoded with the formula from the G4JNT spec:
 *   n_loc = (179 - 10*(Loc0 - 'A') - D0) * 180 + (10*(Loc1 - 'A') + D1)
 * where Loc0, Loc1 are the field letters (A..R, 18 values each) and D0, D1
 * are the square digits (0..9).
 *
 * The range AA00..RR99 maps to values 179..32220, all fitting in 15 bits
 * (2^15 = 32768 > 32220).  The "reversed" arithmetic (179 - ...) means that
 * lower field letters produce higher n_loc values, matching the original WSPR
 * source convention.
 *
 * @param loc  NUL-terminated 4-character locator string, e.g. "GF05".
 *             Characters 0-1 must be A..R (case-insensitive).
 *             Characters 2-3 must be digits 0..9.
 * @param out  Receives the packed 15-bit integer n_loc on success.
 * @return 0 on success, -1 if the locator is not exactly 4 characters or
 *         contains out-of-range characters.
 */
static int pack_locator4(const char *loc, uint32_t *out) {
    size_t len = strlen(loc);

    // Exactly 4 characters required for a standard WSPR locator
    if (len != 4)
        return -1;

    char c0 = toupper((unsigned char)loc[0]); // field letter 1 (A..R)
    char c1 = toupper((unsigned char)loc[1]); // field letter 2 (A..R)
    int d0 = digit_val(loc[2]);               // square digit 1 (0..9)
    int d1 = digit_val(loc[3]);               // square digit 2 (0..9)

    if (c0 < 'A' || c0 > 'R')
        return -1;

    if (c1 < 'A' || c1 > 'R')
        return -1;

    if (d0 < 0 || d1 < 0)
        return -1;

    // G4JNT locator formula: compresses (field1, field2, digit1, digit2) into 15 bits
    uint32_t n = (179u - 10u * (uint32_t)(c0 - 'A') - (uint32_t)d0) * 180u + (10u * (uint32_t)(c1 - 'A') + (uint32_t)d1);
    *out = n;

    return 0;
}

/**
 * @brief Round a dBm value to the nearest WSPR-legal power level and encode it.
 *
 * WSPR specifies 19 legal TX power values (those ending in 0, 3, or 7 dBm):
 *   0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60
 * Any other value is rounded to the nearest legal entry.
 *
 * The encoded power field n_pwr = rounded_dBm + 64, giving a 7-bit integer in
 * the range 64..124.  Adding 64 ensures the value fits within the 7-bit power
 * field of the packed message (M = M1 * 128 + n_pwr, per the G4JNT spec).
 *
 * For Type-2 / Type-3 messages the caller forces n_pwr to an odd number after
 * this call (an odd power code signals a compound-callsign or 6-char-locator
 * message to WSPR decoders).
 *
 * @param dbm  Desired TX power in dBm (0..60 typical).
 * @param out  Receives the encoded power integer (rounded_dBm + 64).
 * @return Always 0 (the rounding loop always finds a best match).
 */
static int pack_power(int dbm, uint32_t *out) {
    // The 19 WSPR-legal dBm values (0, 3, 7, 10, ... 60)
    static const int valid[] = { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 };

    // Exact match: common case for well-configured stations
    for (int i = 0; i < 19; i++) {
        if (valid[i] == dbm) {
            *out = (uint32_t)(dbm + 64);
            return 0;
        }
    }

    // No exact match: find the nearest legal value (minimum absolute distance)
    int best = valid[0], bd = 999;
    for (int i = 0; i < 19; i++) {
        int d = dbm - valid[i];
        if (d < 0)
            d = -d;
        if (d < bd) {
            bd = d;
            best = valid[i];
        }
    }
    *out = (uint32_t)(best + 64);

    return 0;
}

/**
 * @brief Bit-pack the three WSPR field integers into the 7-byte message array.
 *
 * The WSPR message occupies exactly 50 bits, stored MSB-first in a 7-byte
 * array (56 bits total; the 6 LSBs of byte 6 and bytes 7..10 are zero).
 * Only bytes 0..6 are used; the downstream convolutional encoder reads 81
 * bits (50 data + 31 trailing zeros) from this array.
 *
 * Bit layout (from G4JNT / K1JT spec):
 *   Bits 0..27  (28 bits) : n_call  — packed callsign
 *   Bits 28..42 (15 bits) : n_loc   — packed locator
 *   Bits 43..49  (7 bits) : n_pwr   — encoded power
 *
 * Byte assignment:
 *   msg[0] = n_call[27:20]          (8 MSBs of callsign)
 *   msg[1] = n_call[19:12]
 *   msg[2] = n_call[11:4]
 *   msg[3] = n_call[3:0] | n_loc[14:11]   (4 LSBs call + 4 MSBs locator)
 *   msg[4] = n_loc[10:3]
 *   msg[5] = n_loc[2:0]  | n_pwr[6:2]     (3 LSBs locator + 5 MSBs power)
 *   msg[6] = n_pwr[1:0]  << 6             (2 LSBs power in bits 7:6)
 *
 * @param n_call  28-bit packed callsign integer.
 * @param n_loc   15-bit packed locator integer.
 * @param n_pwr    7-bit encoded power integer (dBm + 64).
 * @param msg     Output byte array of exactly 7 bytes.
 */
static void pack_message(uint32_t n_call, uint32_t n_loc, uint32_t n_pwr, uint8_t msg[7]) {
    msg[0] = (uint8_t)(n_call >> 20);
    msg[1] = (uint8_t)(n_call >> 12);
    msg[2] = (uint8_t)(n_call >> 4);
    msg[3] = (uint8_t)((n_call & 0x0Fu) << 4) | (uint8_t)((n_loc >> 11) & 0x0Fu);
    msg[4] = (uint8_t)(n_loc >> 3);
    msg[5] = (uint8_t)((n_loc & 0x07u) << 5) | (uint8_t)((n_pwr >> 2) & 0x1Fu);
    msg[6] = (uint8_t)((n_pwr & 0x03u) << 6);
}

// Convolutional encoder generator polynomials (K1JT / G4JNT spec).
// These are the feedback tap masks for the two 32-bit shift registers.
// The constraint length K=32 means 31 tail bits are needed to flush the registers,
// giving 50 data bits + 31 tail = 81 input bits → 162 output bits (rate-1/2).
#define POLY1 0xF2D05351UL
#define POLY2 0xE4613C47UL

/**
 * @brief Apply the WSPR rate-1/2 convolutional encoder to the 7-byte message.
 *
 * The encoder implements a non-recursive convolutional code with:
 *   Constraint length K  = 32  (31 memory bits)
 *   Code rate            = 1/2 (2 output bits per input bit)
 *   Generator polynomial 0 : 0xF2D05351
 *   Generator polynomial 1 : 0xE4613C47
 *
 * Processing loop (G4JNT spec):
 *   For each of the 81 input bits (50 data + 31 zero tail):
 *     1. Shift the bit into the LSB of the 32-bit shift register (sr).
 *     2. AND sr with POLY1; XOR all 32 result bits together (parity) → bit p1.
 *     3. AND sr with POLY2; XOR all 32 result bits together (parity) → bit p2.
 *     4. Append p1 then p2 to the output array enc[].
 * Output: 81 × 2 = 162 single-bit values in enc[].
 *
 * The 31 tail bits (zeros) ensure both shift registers return to the all-zero
 * state, maximising the FEC capability at the end of the frame.
 *
 * Parity computation uses the XOR folding trick:
 *   p ^= (p >> 16); p ^= (p >> 8); p ^= (p >> 4);
 *   p ^= (p >> 2);  p ^= (p >> 1); p &= 1;
 * which is equivalent to XOR-ing all 32 bits without a loop.
 *
 * @param msg  7-byte input message from pack_message().
 * @param enc  Output array of WSPR_SYMBOLS (162) bytes; each byte is 0 or 1.
 */
static void conv_encode(const uint8_t msg[7], uint8_t enc[WSPR_SYMBOLS]) {
    uint32_t sr = 0; // 32-bit shift register shared by both generators
    int out_idx = 0;
    for (int bit_idx = 0; bit_idx < 81; bit_idx++) {
        uint8_t in_bit;
        if (bit_idx < 50) {
            // Data bits 0..49: read MSB-first from the 7-byte message array
            int byte_idx = bit_idx / 8;
            int bit_pos = 7 - (bit_idx % 8); // MSB of each byte is bit 0
            in_bit = (msg[byte_idx] >> bit_pos) & 1;
        } else {
            // Tail bits 50..80: zero-flush to fully drain both shift registers
            in_bit = 0;
        }

        // Shift new bit into the shift register (both generators share the same sr)
        sr = (sr << 1) | in_bit;

        // Generator 0: parity of (sr AND POLY1)
        uint32_t p1 = sr & POLY1;
        uint32_t p2 = sr & POLY2;

        // XOR-fold p1 to a single parity bit (popcount mod 2)
        p1 ^= (p1 >> 16);
        p1 ^= (p1 >> 8);
        p1 ^= (p1 >> 4);
        p1 ^= (p1 >> 2);
        p1 ^= (p1 >> 1);
        p1 &= 1;

        // Generator 1: parity of (sr AND POLY2)
        p2 ^= (p2 >> 16);
        p2 ^= (p2 >> 8);
        p2 ^= (p2 >> 4);
        p2 ^= (p2 >> 2);
        p2 ^= (p2 >> 1);
        p2 &= 1;

        // Output p1 first, then p2 (matches the G4JNT / K1JT generator ordering)
        if (out_idx < WSPR_SYMBOLS)
            enc[out_idx++] = (uint8_t)p1;
        if (out_idx < WSPR_SYMBOLS)
            enc[out_idx++] = (uint8_t)p2;
    }
}

/**
 * @brief Reverse all 8 bits of a byte (bit-mirror).
 *
 * This is the core of the WSPR bit-reversal interleaver.  The WSPR spec
 * uses the bit-reversed value of an 8-bit address I as the destination
 * index J for each encoded bit:  D[J] = S[P]  (G4JNT spec).
 *
 * The reversal is done with three in-place swap operations on nibbles,
 * pairs, and individual bits — no lookup table required.
 *
 * Examples: reverse8(0x01) = 0x80,  reverse8(0x13) = 0xC8.
 *
 * @param v  Byte to reverse.
 * @return   Bit-reversed byte.
 */
static uint8_t reverse8(uint8_t v) {
    // Swap nibbles (bits 7:4 ↔ bits 3:0)
    v = ((v & 0xF0) >> 4) | ((v & 0x0F) << 4);
    // Swap adjacent pairs (bits 7:6,3:2 ↔ bits 5:4,1:0)
    v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
    // Swap adjacent single bits
    v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
    return v;
}

/**
 * @brief Apply the WSPR bit-reversal interleaver to the 162 encoded bits.
 *
 * Burst errors on radio channels tend to corrupt several consecutive coded
 * bits, which defeats the FEC.  Interleaving spreads the positions of
 * logically adjacent bits in time so that a burst affecting physical positions
 * P, P+1, P+2, ... hits encoded bits at widely separated logical positions,
 * making the error pattern appear random to the convolutional decoder.
 *
 * WSPR interleaving algorithm (G4JNT spec):
 *   P = 0  (source pointer into enc[])
 *   for I = 0 to 255:
 *     J = bit_reverse_8(I)         // 8-bit mirror of I
 *     if J < 162:
 *       il[J] = enc[P++]           // place source bit at bit-reversed destination
 *     stop when P == 162
 *
 * This visits all 256 possible 8-bit addresses in natural order and accepts
 * only those whose bit-reversed value falls within [0, 161].  The 162 source
 * bits are scattered to non-sequential destinations, breaking burst correlation.
 *
 * @param enc  Input: 162 convolutional-encoded bits (each 0 or 1).
 * @param il   Output: 162 interleaved bits in their final WSPR positions.
 */
static void interleave(const uint8_t enc[WSPR_SYMBOLS], uint8_t il[WSPR_SYMBOLS]) {
    memset(il, 0, WSPR_SYMBOLS);
    int p = 0; // source pointer
    for (int i = 0; i < 256 && p < WSPR_SYMBOLS; i++) {
        int j = (int)reverse8((uint8_t)i); // bit-reversed destination index
        if (j < WSPR_SYMBOLS) {
            il[j] = enc[p++];
        }
    }
}

/**
 * @brief Run the full WSPR encode pipeline on a packed 7-byte message.
 *
 * Combines convolutional encoding, bit-reversal interleaving, and sync-vector
 * merging into the final 162-symbol output array.  This function is the
 * common exit path for all message types (Type-1, Type-2, Type-3).
 *
 * Final symbol formula (G4JNT / K1JT spec):
 *   symbol[i] = 2 * interleaved_bit[i] + SYNC[i]    for i = 0..161
 * Each symbol carries both a sync bit (LSB) and a data bit (MSB), giving
 * the 4-FSK tone index (0..3) that drives the oscillator.
 *
 * @param msg      7-byte packed message from pack_message().
 * @param symbols  Output: 162 4-FSK tone indices (0..3).
 */
static void encode_and_interleave(const uint8_t msg[7], uint8_t symbols[WSPR_SYMBOLS]) {
    uint8_t enc[WSPR_SYMBOLS]; // convolutional-encoded bits
    conv_encode(msg, enc);
    uint8_t il[WSPR_SYMBOLS]; // interleaved bits
    interleave(enc, il);
    // Merge data with sync vector: symbol = 2*data + sync  (range 0..3)
    for (int i = 0; i < WSPR_SYMBOLS; i++)
        symbols[i] = 2 * il[i] + SYNC[i];
}

/**
 * @brief Parse a compound callsign (containing '/') into base, prefix, and suffix parts.
 *
 * WSPR Type-2 messages support compound callsigns of the forms:
 *   PREFIX/BASECALL   — prefix of 1-3 alphanumeric characters before the slash
 *   BASECALL/SUFFIX   — suffix of 1-2 characters after the slash
 *                         (single letter or digit, or two digits)
 *
 * Doubly-compounded callsigns (two slashes, e.g. PJ4/K1ABC/P) are rejected
 * because the 15-bit locator field can only hold one modifier at a time.
 *
 * The function populates exactly one of (*has_prefix, *has_suffix) = (1, 0)
 * or (0, 1) depending on which form is matched.  The 3-character prefix buffer
 * is space-padded on the left if the prefix is shorter than 3 characters.
 *
 * @param cs          NUL-terminated compound callsign (must contain '/').
 * @param base_call   Output: base callsign without prefix/suffix (7-byte buf).
 * @param prefix      Output: 3-character null-terminated prefix (left-padded).
 * @param suffix      Output: 1-2 character null-terminated suffix.
 * @param has_prefix  Output: 1 if a prefix was found, 0 otherwise.
 * @param has_suffix  Output: 1 if a suffix was found, 0 otherwise.
 * @return 0 on success, -1 if the callsign cannot be parsed as a valid
 *         compound callsign.
 */
static int parse_compound_callsign(const char *cs, char base_call[8], char prefix[4], char suffix[3], int *has_prefix, int *has_suffix) {
    // Locate the single '/' separator; reject if more than one is found
    const char *slash = NULL;
    for (const char *p = cs; *p; p++) {
        if (*p == '/') {
            if (slash != NULL)
                return -1; // doubly-compounded callsigns not supported
            slash = p;
        }
    }
    if (slash == NULL)
        return -1; // no slash found

    *has_prefix = 0;
    *has_suffix = 0;
    memset(prefix, 0, 4);
    memset(suffix, 0, 3);
    memset(base_call, 0, 8);

    int total_len = (int)strlen(cs);
    int slash_pos = (int)(slash - cs);
    int before_len = slash_pos;                // characters before '/'
    int after_len = total_len - slash_pos - 1; // characters after '/'

    // check suffix form FIRST when after_len <= 2.
    // WSPR 2.0 Appendix B defines a suffix as 1 alphanumeric char or 2 digits.
    // Preferring suffix prevents ambiguous splits like AB/37 (before=2, after=2)
    // from being misclassified as PREFIX/BASE instead of the correct BASE/SUFFIX.
    // If the suffix characters fail validation we fall through to prefix form.
    if (after_len >= 1 && after_len <= 2 && before_len >= 1 && before_len <= 6) {
        const char *suf = slash + 1;
        bool suffix_valid;
        if (after_len == 1) {
            // Single character: any alphanumeric qualifies as a suffix
            suffix_valid = (isalnum((unsigned char)suf[0]) != 0);
        } else {
            // Two-character suffix must be exactly two digits per WSPR 2.0 Appendix B
            suffix_valid = (isdigit((unsigned char)suf[0]) != 0) && (isdigit((unsigned char)suf[1]) != 0);
        }
        if (suffix_valid) {
            for (int i = 0; i < after_len; i++)
                suffix[i] = toupper((unsigned char)suf[i]);
            suffix[after_len] = '\0';
            for (int i = 0; i < before_len; i++)
                base_call[i] = toupper((unsigned char)cs[i]);
            base_call[before_len] = '\0';
            *has_suffix = 1;
            return 0;
        }
        // Suffix characters invalid; fall through to try prefix form below
    }

    // Prefix form: PREFIX/BASECALL  (prefix 1-3 chars, base 1-6 chars)
    if (before_len <= 3 && before_len >= 1 && after_len >= 1 && after_len <= 6) {
        // Validate prefix characters — must be alphanumeric
        for (int i = 0; i < before_len; i++) {
            if (!isalnum((unsigned char)cs[i]))
                return -1;
        }
        // Left-pad prefix to 3 characters with spaces
        int pad = 3 - before_len;
        for (int i = 0; i < pad; i++)
            prefix[i] = ' ';
        for (int i = 0; i < before_len; i++)
            prefix[pad + i] = toupper((unsigned char)cs[i]);
        prefix[3] = '\0';

        if (after_len > 6)
            return -1;
        // The portion after '/' is the base callsign
        for (int i = 0; i < after_len; i++)
            base_call[i] = toupper((unsigned char)(slash + 1)[i]);
        base_call[after_len] = '\0';
        *has_prefix = 1;
        return 0;
    }

    return -1; // does not match prefix or suffix form
}

/**
 * @brief Encode a compound callsign (with '/') as a WSPR Type-2 message.
 *
 * Type-2 messages use the 28-bit callsign field for the base callsign (packed
 * via pack_callsign_type1) and the 15-bit locator field for the prefix or suffix
 * code (n_loc).  The power code is forced to an odd number — this is the
 * signal to WSPR decoders that the message is Type-2 rather than Type-1.
 *
 * Prefix encoding (WSPR 2.0 Appendix B):
 *   Each prefix character maps to a WSPR value (char_to_wspr), giving 37 values.
 *   n_pfx = v0 * 37^2 + v1 * 37 + v2
 *   n_loc_candidate = n_pfx * 22 + 11
 *   If n_loc_candidate > 32767 the prefix overflows the 15-bit field → return -1.
 *
 * Suffix encoding:
 *   Single letter A-Z  : n_loc = letter_index (0..25)
 *   Single digit  0-9  : n_loc = 26 + digit (26..35)
 *   Two digits DD      : n_loc = 26 + DD    (26..125, but DD ≤ 99 in practice)
 *   Empty / other      : n_loc = 36
 *
 * Power encoding for Type-2 / Type-3:
 *   The rounded power code p is made odd:  n_pwr = (p/2)*2 + 1
 *   An odd power code in the 7-bit field is the Type-2 identifier per K1JT spec.
 *
 * @param cs          NUL-terminated compound callsign.
 * @param power_dbm   TX power in dBm (rounded internally).
 * @param n_call_out  Receives the 28-bit packed base callsign.
 * @param n_loc_out   Receives the 15-bit prefix/suffix code.
 * @param n_pwr_out   Receives the odd-forced 7-bit power code.
 * @return 0 on success, -1 if the callsign cannot be parsed or the prefix
 *         overflows the 15-bit locator field.
 */
static int pack_callsign_type2(const char *cs, int power_dbm, uint32_t *n_call_out, uint32_t *n_loc_out, uint32_t *n_pwr_out) {
    char base_call[8], prefix[4], suffix[3];
    int has_prefix = 0, has_suffix = 0;
    if (parse_compound_callsign(cs, base_call, prefix, suffix, &has_prefix, &has_suffix) < 0)
        return -1;

    // Pack the base callsign into the standard 28-bit callsign field
    uint32_t n_call = 0;
    if (pack_callsign_type1(base_call, &n_call) < 0)
        return -1;

    // force pwr_code (already = dBm + 64) to odd directly,
    // preserving the +64 base offset required by the WSPR spec.
    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    uint32_t n_pwr = (pwr_code % 2u == 0u) ? (pwr_code + 1u) : pwr_code;

    uint32_t n_loc = 0;
    if (has_prefix) {
        // Encode the 3-character prefix into the 15-bit locator field.
        // The multiplier 22 and offset 11 are from the WSPR 2.0 Appendix B spec.
        int v0 = char_to_wspr(prefix[0]);
        int v1 = char_to_wspr(prefix[1]);
        int v2 = char_to_wspr(prefix[2]);
        if (v0 < 0 || v1 < 0 || v2 < 0)
            return -1;
        uint32_t n_pfx = (uint32_t)v0 * 37u * 37u + (uint32_t)v1 * 37u + (uint32_t)v2;
        uint32_t n_loc_candidate = n_pfx * 22u + 11u;
        // A result > 32767 overflows the 15-bit field; caller will fall back to Type-1
        if (n_loc_candidate > 32767u) {
            return -1;
        }
        n_loc = n_loc_candidate;
    } else {
        // Encode the suffix into the 15-bit field per WSPR 2.0 Appendix B
        int suf_len = (int)strlen(suffix);
        int ntype;
        if (suf_len == 1) {
            if (isdigit((unsigned char)suffix[0])) {
                // Single digit '0'..'9' → 26..35
                ntype = 26 + (suffix[0] - '0');
            } else {
                // Single letter 'A'..'Z' → 0..25
                ntype = toupper((unsigned char)suffix[0]) - 'A';
            }
        } else if (suf_len == 2) {
            // Two digits DD → 26 + DD
            int dd = (suffix[0] - '0') * 10 + (suffix[1] - '0');
            ntype = 26 + dd;
        } else {
            ntype = 36; // empty or unrecognised suffix
        }
        n_loc = (uint32_t)ntype;
    }

    *n_call_out = n_call;
    *n_loc_out = n_loc;
    *n_pwr_out = n_pwr;
    return 0;
}

/**
 * @brief Compute the 15-bit WSPR callsign hash used in Type-3 messages.
 *
 * WSPR Type-3 messages do not transmit the callsign directly.  Instead, a
 * 15-bit hash of the callsign is placed in the locator field so that
 * receiving stations can link the Type-3 message (which carries the 6-char
 * locator) to a previously received Type-1 or Type-2 message from the same
 * station.  The 15-bit length means collisions are rare but not impossible
 * (probability ~1/32768 for any two different callsigns).
 *
 * Hash algorithm (WSPR 2.0 Appendix B / K1JT source):
 *   n = 0
 *   for each character c in the callsign (upper-cased):
 *     v = WSPR_char_value(c)   (digits 0-9, letters 10-35, other 36)
 *     n = n * 37 + v
 *   hash15 = ((n >> 15) XOR n) & 0x7FFF
 *
 * The XOR folding of the upper and lower 15-bit halves improves avalanche
 * behaviour over the simple modulo-32768 reduction.
 *
 * @param cs  NUL-terminated callsign string (full compound form if applicable).
 * @return    15-bit hash value (0..32767).
 */
static uint32_t callsign_hash15(const char *cs) {
    // Split the 30-bit accumulator into two 15-bit halves (n_hi, n_lo) so
    // that each intermediate product stays well within uint32_t:
    //   n_lo * 37 + v  <= 32767*37+36 = 1,212,415 < 2^21 (safe)
    //   n_hi * 37 + carry <= 32767*37+37 = 1,212,416 < 2^21 (safe)
    // The final XOR-fold (n_hi ^ n_lo) is identical to ((n>>15)^n)&0x7FFF of
    // the K1JT 64-bit reference because n_hi == n>>15 and n_lo == n&0x7FFF.
    uint32_t n_hi = 0, n_lo = 0;
    for (int i = 0; cs[i] != '\0'; i++) {
        char c = toupper((unsigned char)cs[i]);
        uint32_t v;
        if (c >= '0' && c <= '9')
            v = (uint32_t)(c - '0');
        else if (c >= 'A' && c <= 'Z')
            v = (uint32_t)(c - 'A') + 10u;
        else
            v = 36u; // space or any other character
        // Accumulate n = n*37 + v (mod 2^30) using two 15-bit halves.
        uint32_t lo_new = n_lo * 37u + v;     // max 1,212,415 -- fits uint32_t
        uint32_t carry = lo_new >> 15;        // carry into the upper half
        n_lo = lo_new & 0x7FFFu;              // keep bits[14:0]
        uint32_t hi_new = n_hi * 37u + carry; // max 1,212,416 -- fits uint32_t
        n_hi = hi_new & 0x7FFFu;              // keep bits[29:15] of n
    }
    // XOR-fold: n_hi == n>>15, n_lo == n&0x7FFF, so result matches K1JT reference.
    return (n_hi ^ n_lo) & 0x7FFFu;
}

/**
 * @brief Pack a 6-character Maidenhead sub-square into the 28-bit Type-3 callsign field.
 *
 * WSPR Type-3 messages carry a 6-character Maidenhead locator (e.g. "GF05ab")
 * in the 28-bit callsign field (n_call).  The encoding (from K1JT source /
 * WSPR 2.0 Appendix B) is:
 *
 *   ng     = 32768 + (179 - 10*(G0-'A') - D0) * 180 + (10*(G1-'A') + D1)
 *   nsubsq = (G4-'A') * 24 + (G5-'A')
 *   n_call = ng * 576 + nsubsq
 *
 * where G0,G1 are field letters (A..R, value 0..17), D0,D1 are square digits
 * (0..9), G4,G5 are sub-square letters (A..X, value 0..23, case-insensitive).
 *
 * The base offset 32768 (= 2^15) places the Type-3 value above the 32767
 * maximum of the 4-character locator encoding, allowing decoders to
 * distinguish the two uses of the 28-bit callsign field.
 *
 * Maximum n_call:
 *   ng_max     = 32768 + 179 * 180 + 10*17 + 9 = 32768 + 32220 + 179 = 65167
 *   nsubsq_max = 23 * 24 + 23 = 575
 *   n_call_max = 65167 * 576 + 575 = 37,536,767  (fits in 26 bits, < 2^28)
 *
 * @param loc6       NUL-terminated 6-character locator string (A-R, A-R, 0-9, 0-9, A-X, A-X).
 * @param n_call_out Receives the packed 28-bit value on success.
 * @return 0 on success, -1 if the locator is not exactly 6 characters or
 *         any character is out of range.
 */
static int pack_loc6_type3(const char *loc6, uint32_t *n_call_out) {
    if (strlen(loc6) != 6)
        return -1;
    char g0 = toupper((unsigned char)loc6[0]); // field letter 1 (A..R)
    char g1 = toupper((unsigned char)loc6[1]); // field letter 2 (A..R)
    int g2 = digit_val(loc6[2]);               // square digit 1 (0..9)
    int g3 = digit_val(loc6[3]);               // square digit 2 (0..9)
    char g4 = toupper((unsigned char)loc6[4]); // subsquare letter 1 (A..X)
    char g5 = toupper((unsigned char)loc6[5]); // subsquare letter 2 (A..X)

    if (g0 < 'A' || g0 > 'R')
        return -1;
    if (g1 < 'A' || g1 > 'R')
        return -1;
    if (g2 < 0 || g3 < 0)
        return -1;
    if (g4 < 'A' || g4 > 'X')
        return -1;
    if (g5 < 'A' || g5 > 'X')
        return -1;

    // ng: 4-character portion packed with the same formula as pack_locator4,
    //     but offset by 32768 to signal a Type-3 message to the decoder.
    uint32_t ng = 32768u + (179u - 10u * (uint32_t)(g0 - 'A') - (uint32_t)g2) * 180u + (10u * (uint32_t)(g1 - 'A') + (uint32_t)g3);
    // nsubsq: sub-square pair (A..X × A..X) = 24 × 24 = 576 possible values
    uint32_t nsubsq = (uint32_t)(g4 - 'A') * 24u + (uint32_t)(g5 - 'A');
    *n_call_out = ng * 576u + nsubsq;

    return 0;
}

/**
 * @brief Test whether a callsign is compound (contains a '/' character).
 *
 * Used to route encoding to Type-1 (simple) or Type-2 (compound) path without
 * any allocation.
 *
 * @param cs  NUL-terminated callsign string.
 * @return 1 if the string contains '/', 0 otherwise.
 */
static int is_compound_callsign(const char *cs) {
    for (int i = 0; cs[i] != '\0'; i++) {
        if (cs[i] == '/')
            return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;

    if (is_compound_callsign(callsign)) {
        // --- Type-2 path: compound callsign (contains '/') ---
        // Try to encode as a proper Type-2 message (base call + prefix or suffix)
        uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
        if (pack_callsign_type2(callsign, power_dbm, &n_call, &n_loc, &n_pwr) == 0) {
            uint8_t msg[7];
            pack_message(n_call, n_loc, n_pwr, msg);
            encode_and_interleave(msg, symbols);
            return 0;
        }

        // Type-2 failed (e.g. prefix overflows the 15-bit locator field).
        // Fall back: extract the longer part of the callsign (before or after '/')
        // and encode it as a Type-1 message with the first 4 locator characters.
        const char *slash = NULL;
        for (const char *p = callsign; *p; p++) {
            if (*p == '/') {
                slash = p;
                break;
            }
        }

        // Choose the longer segment (before or after '/') as the base callsign
        const char *base_start = callsign;
        int base_len = (int)strlen(callsign);
        if (slash != NULL) {
            int before = (int)(slash - callsign);
            int after = (int)strlen(slash + 1);
            if (after > before) {
                base_start = slash + 1;
                base_len = after;
            } else {
                base_len = before;
            }
        }

        // Truncate to at most 6 characters for the Type-1 field
        char base_buf[7] = { 0 };
        int copy_len = (base_len > 6) ? 6 : base_len;
        for (int i = 0; i < copy_len; i++)
            base_buf[i] = base_start[i];

        if (pack_callsign_type1(base_buf, &n_call) == 0) {
            char loc4[5];
            size_t loc_len = strlen(locator);
            if (loc_len >= 4) {
                // Use only the first 4 characters of the locator for Type-1 fallback
                loc4[0] = locator[0];
                loc4[1] = locator[1];
                loc4[2] = locator[2];
                loc4[3] = locator[3];
                loc4[4] = '\0';
                if (pack_locator4(loc4, &n_loc) == 0) {
                    ESP_LOGW(TAG, "Type-2 prefix overflow for '%s'; TX as Type-1 base callsign '%s'", callsign, base_buf);
                    pack_power(power_dbm, &n_pwr);
                    uint8_t msg[7];
                    pack_message(n_call, n_loc, n_pwr, msg);
                    encode_and_interleave(msg, symbols);
                    return 0;
                }
            }
        }

        return -1; // all fallbacks exhausted
    }

    // --- Type-1 path: simple callsign (no '/') ---
    // Handles both 4-char locator (pure Type-1) and 6-char locator
    // (Type-1 primary; the caller must separately produce the Type-3 companion).
    uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
    if (pack_callsign_type1(callsign, &n_call) < 0)
        return -1;

    // For 6-character locators only the first 4 chars are used in this (Type-1) message;
    // the full 6-char locator is conveyed by the companion Type-3 transmission.
    char loc4[5];
    size_t loc_len = strlen(locator);
    if (loc_len < 4)
        return -1;

    loc4[0] = locator[0];
    loc4[1] = locator[1];
    loc4[2] = locator[2];
    loc4[3] = locator[3];
    loc4[4] = '\0';
    if (pack_locator4(loc4, &n_loc) < 0)
        return -1;

    pack_power(power_dbm, &n_pwr);
    uint8_t msg[7];
    pack_message(n_call, n_loc, n_pwr, msg);
    encode_and_interleave(msg, symbols);
    return 0;
}

wspr_msg_type_t wspr_encode_type(const char *callsign, const char *locator) {
    if (!callsign || !locator)
        return WSPR_MSG_TYPE_1;

    if (is_compound_callsign(callsign)) {
        // Probe pack_callsign_type2 with a dummy power level to check feasibility
        uint32_t nc = 0, nl = 0, np = 0;
        if (pack_callsign_type2(callsign, 20, &nc, &nl, &np) == 0)
            return WSPR_MSG_TYPE_2; // proper Type-2 encoding possible

        // Prefix overflow or parse failure → fall back to Type-1 encoding
        return WSPR_MSG_TYPE_1;
    }

    // Simple callsign with 6-character locator → alternating Type-1 + Type-3
    if (strlen(locator) >= 6)
        return WSPR_MSG_TYPE_3;

    // Standard simple callsign + 4-char locator → single Type-1 message
    return WSPR_MSG_TYPE_1;
}

int wspr_encode_type3(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;

    // Type-3 messages require exactly a 6-character Maidenhead sub-square locator
    if (strlen(locator) != 6)
        return -1;

    // The 6-character locator is packed into the 28-bit callsign field
    uint32_t n_call = 0;
    if (pack_loc6_type3(locator, &n_call) < 0)
        return -1;

    // The 15-bit locator field carries the callsign hash so decoders can link
    // this Type-3 message to the preceding Type-1 or Type-2 from the same station
    uint32_t n_loc = callsign_hash15(callsign);

    // Force pwr_code (already = dBm + 64) to odd directly,
    // preserving the +64 base offset required by the WSPR spec.
    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    uint32_t n_pwr = (pwr_code % 2u == 0u) ? (pwr_code + 1u) : pwr_code;

    uint8_t msg[7];
    pack_message(n_call, n_loc, n_pwr, msg);
    encode_and_interleave(msg, symbols);

    return 0;
}
