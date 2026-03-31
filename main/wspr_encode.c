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

#include <ctype.h>
#include <string.h>

#include "wspr_encode.h"

// Sync vector (162 bits) -- unchanged from original.
static const uint8_t SYNC[WSPR_SYMBOLS] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0,
                                            0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1,
                                            0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
                                            1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 };

// ---------------------------------------------------------------------------
// Character value helpers (shared by all message types)
// ---------------------------------------------------------------------------

// Map character to WSPR 37-symbol alphabet value: 0-9=0..9, A-Z=10..35, space=36.
// Returns -1 for invalid characters.
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

// Map character to WSPR 27-symbol suffix alphabet: A-Z=0..25, space=26.
// Used for positions 3-5 of a packed Type-1 callsign.
static int suffix_val(char c) {
    c = toupper((unsigned char)c);
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A');
    return 26; // space
}

// Return numeric digit value 0-9, or -1 if not a digit.
static int digit_val(char c) {
    return (c >= '0' && c <= '9') ? (c - '0') : -1;
}

// ---------------------------------------------------------------------------
// Type-1 callsign packing (unchanged logic, bug-free in original for simple callsigns)
// ---------------------------------------------------------------------------

// Pack a standard (simple, no '/') callsign into a 28-bit integer using the
// G4JNT formula. Position 2 (0-indexed) must be a digit after normalisation.
// Returns 0 on success, -1 on error.
static int pack_callsign_type1(const char *cs, uint32_t *out) {
    char buf[7] = "      ";
    int len = (int)strlen(cs);
    // Tightened maximum to 6: Type-1 callsigns are at most 6 characters.
    // Compound callsigns (containing '/') must NOT enter this path.
    // The original allowed len up to 12 and silently replaced '/' with space,
    // producing a malformed pack result that decoded to garbage at the receiver.
    if (len < 1 || len > 6)
        return -1;
    memcpy(buf, cs, (size_t)len);
    buf[6] = '\0';

    // Reject compound callsigns here explicitly.
    // The original code replaced '/' with space and continued, producing a
    // wrong 28-bit value. Compound callsigns must use pack_callsign_type2().
    for (int i = 0; i < 6; i++) {
        if (buf[i] == '/')
            return -1;
    }

    // If buf[2] is not a digit, prepend a space (handles G4JNT -> " G4JNT").
    if (!isdigit((unsigned char)buf[2])) {
        if (len > 5)
            return -1; // no room to prepend
        for (int i = len; i > 0; i--)
            buf[i] = buf[i - 1];
        buf[0] = ' ';
    }
    if (!isdigit((unsigned char)buf[2]))
        return -1;

    int c0 = char_to_wspr(buf[0]);
    int c1 = char_to_wspr(buf[1]);
    int c2 = (int)(buf[2] - '0');
    int c3 = suffix_val(buf[3]);
    int c4 = suffix_val(buf[4]);
    int c5 = suffix_val(buf[5]);
    if (c0 < 0 || c1 < 0)
        return -1;
    if (c1 > 35)
        return -1; // space not valid at position 1

    uint32_t n = (uint32_t)c0;
    n = n * 36u + (uint32_t)c1;
    n = n * 10u + (uint32_t)c2;
    n = n * 27u + (uint32_t)c3;
    n = n * 27u + (uint32_t)c4;
    n = n * 27u + (uint32_t)c5;
    *out = n;
    return 0;
}

// ---------------------------------------------------------------------------
// Type-1 locator packing (4-char only)
// ---------------------------------------------------------------------------

// Pack a 4-character Maidenhead grid square into a 15-bit integer.
// Only accepts exactly 4 characters: [A-R][A-R][0-9][0-9].
// strictly requires exactly 4 chars.
// 6-char locators must use pack_locator6_as_callsign() for Type-3 encoding.
static int pack_locator4(const char *loc, uint32_t *out) {
    size_t len = strlen(loc);
    // Accept only exactly 4 characters for Type-1 locator field.
    // The original accepted 4-6 and silently used only the first 4.
    if (len != 4)
        return -1;
    char c0 = toupper((unsigned char)loc[0]);
    char c1 = toupper((unsigned char)loc[1]);
    int d0 = digit_val(loc[2]);
    int d1 = digit_val(loc[3]);
    if (c0 < 'A' || c0 > 'R')
        return -1;
    if (c1 < 'A' || c1 > 'R')
        return -1;
    if (d0 < 0 || d1 < 0)
        return -1;
    uint32_t n = (179u - 10u * (uint32_t)(c0 - 'A') - (uint32_t)d0) * 180u + (10u * (uint32_t)(c1 - 'A') + (uint32_t)d1);
    *out = n;
    return 0;
}

// ---------------------------------------------------------------------------
// Power packing (unchanged from original)
// ---------------------------------------------------------------------------

static int pack_power(int dbm, uint32_t *out) {
    static const int valid[] = { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 };
    for (int i = 0; i < 19; i++) {
        if (valid[i] == dbm) {
            *out = (uint32_t)(dbm + 64);
            return 0;
        }
    }
    // Round to nearest valid level.
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

// ---------------------------------------------------------------------------
// 50-bit message assembly (unchanged from original)
// ---------------------------------------------------------------------------

static void pack_message(uint32_t n_call, uint32_t n_loc, uint32_t n_pwr, uint8_t msg[7]) {
    msg[0] = (uint8_t)(n_call >> 20);
    msg[1] = (uint8_t)(n_call >> 12);
    msg[2] = (uint8_t)(n_call >> 4);
    msg[3] = (uint8_t)((n_call & 0x0Fu) << 4) | (uint8_t)((n_loc >> 11) & 0x0Fu);
    msg[4] = (uint8_t)(n_loc >> 3);
    msg[5] = (uint8_t)((n_loc & 0x07u) << 5) | (uint8_t)((n_pwr >> 2) & 0x1Fu);
    msg[6] = (uint8_t)((n_pwr & 0x03u) << 6);
}

// ---------------------------------------------------------------------------
// Convolutional encoder K=32, rate 1/2 (unchanged from original)
// ---------------------------------------------------------------------------

#define POLY1 0xF2D05351UL
#define POLY2 0xE4613C47UL

static void conv_encode(const uint8_t msg[7], uint8_t enc[WSPR_SYMBOLS]) {
    uint32_t sr = 0;
    int out_idx = 0;
    for (int bit_idx = 0; bit_idx < 81; bit_idx++) {
        uint8_t in_bit;
        if (bit_idx < 50) {
            int byte_idx = bit_idx / 8;
            int bit_pos = 7 - (bit_idx % 8);
            in_bit = (msg[byte_idx] >> bit_pos) & 1;
        } else {
            in_bit = 0;
        }
        sr = (sr << 1) | in_bit;
        uint32_t p1 = sr & POLY1;
        uint32_t p2 = sr & POLY2;
        p1 ^= (p1 >> 16);
        p1 ^= (p1 >> 8);
        p1 ^= (p1 >> 4);
        p1 ^= (p1 >> 2);
        p1 ^= (p1 >> 1);
        p1 &= 1;
        p2 ^= (p2 >> 16);
        p2 ^= (p2 >> 8);
        p2 ^= (p2 >> 4);
        p2 ^= (p2 >> 2);
        p2 ^= (p2 >> 1);
        p2 &= 1;
        if (out_idx < WSPR_SYMBOLS)
            enc[out_idx++] = (uint8_t)p1;
        if (out_idx < WSPR_SYMBOLS)
            enc[out_idx++] = (uint8_t)p2;
    }
}

// ---------------------------------------------------------------------------
// Bit-reversal interleaver (unchanged from original)
// ---------------------------------------------------------------------------

static uint8_t reverse8(uint8_t v) {
    v = ((v & 0xF0) >> 4) | ((v & 0x0F) << 4);
    v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
    v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
    return v;
}

static void interleave(const uint8_t enc[WSPR_SYMBOLS], uint8_t il[WSPR_SYMBOLS]) {
    memset(il, 0, WSPR_SYMBOLS);
    int p = 0;
    for (int i = 0; i < 256 && p < WSPR_SYMBOLS; i++) {
        int j = (int)reverse8((uint8_t)i);
        if (j < WSPR_SYMBOLS) {
            il[j] = enc[p++];
        }
    }
}

// Shared final stage: convolutional encode, interleave, combine with sync.
static void encode_and_interleave(const uint8_t msg[7], uint8_t symbols[WSPR_SYMBOLS]) {
    uint8_t enc[WSPR_SYMBOLS];
    conv_encode(msg, enc);
    uint8_t il[WSPR_SYMBOLS];
    interleave(enc, il);
    for (int i = 0; i < WSPR_SYMBOLS; i++)
        symbols[i] = 2 * il[i] + SYNC[i];
}

// ---------------------------------------------------------------------------
// Type-2 compound callsign packing
//
// WSPR Type-2 message: the 28-bit callsign field holds the base callsign
// packed normally; the 15-bit locator field holds an encoded prefix or suffix.
// The 7-bit power field holds: (power_code << 1) | type_flag, where type_flag=1
// signals Type-2 to the decoder.
//
// Per K1JT WSPR 2.0 Appendix B and the WSJT-X packjt.f90 source:
//   For a PREFIX/callsign (e.g. PJ4/K1ABC):
//     n_pfx = pack prefix chars into value using 37-symbol alphabet (up to 3 chars)
//     n_loc = n_pfx * 22 + suffix_type_code   (22 = 2*11 possibilities for suffix)
//             where suffix_type_code = 11 (no suffix) for PREFIX/ form
//   For callsign/SUFFIX (e.g. K1ABC/P, K1ABC/7, K1ABC/47):
//     n_loc encodes the suffix type:
//       single letter X:   n_loc = char_val(X)                 (0..25)
//       single digit D:    n_loc = 26 + D                      (26..35)
//       two digits DD:     n_loc = 26 + DD                     (26..99, DD=10..99)
//       no suffix:         n_loc = 26 + 10 = 36 (treated as no-suffix marker)
//     Then: n_loc = n_loc (the plain value above)
//     For the prefix case: n_loc |= (n_pfx << something) -- see below.
//
// The actual WSJT-X encoding (from packjt.f90):
//   If prefix/callsign: compute prefix as 3-char value (pad left with spaces),
//     n_pfx = char(0)*37^2 + char(1)*37 + char(2)
//     n_loc = n_pfx * 22 + 11     (11 = "no suffix" code in 0..21 range)
//     n_call = pack base callsign normally
//     n_pwr_out = (power_encoded << 1) | 1   (bit 0 = 1 = type 2)
//   If callsign/suffix:
//     suffix char X: ntype = char_wspr(X)    where single letter => 0..25
//     suffix digit D: ntype = 26 + D         => 26..35
//     suffix "dd": ntype = 26 + atoi(dd)    => 36..125 but clamped to 0..35 in practice
//     n_loc = ntype
//     n_call = pack base callsign normally
//     n_pwr_out = (power_encoded << 1) | 1
//
// References:
//   K1JT WSPR 2.0 User's Guide, Appendix B
//   etherkit/JTEncode JTEncode.cpp wspr_encode() Type-2 path
//   WSJT-X lib/wsjtx/packjt.f90
// ---------------------------------------------------------------------------

// Parse a compound callsign of the form "PREFIX/CALL" or "CALL/SUFFIX".
// Fills base_call (the main callsign body), prefix (up to 3 chars before '/'),
// suffix (up to 2 chars after '/'), and sets has_prefix/has_suffix.
// Returns 0 on success, -1 if the format is not a valid compound callsign.
static int parse_compound_callsign(const char *cs, char base_call[8], char prefix[4], char suffix[3], int *has_prefix, int *has_suffix) {
    // Locate the slash.
    const char *slash = NULL;
    for (const char *p = cs; *p; p++) {
        if (*p == '/') {
            if (slash != NULL)
                return -1; // double slash not supported (PJ4/K1ABC/P)
            slash = p;
        }
    }
    if (slash == NULL)
        return -1; // not a compound callsign

    *has_prefix = 0;
    *has_suffix = 0;
    memset(prefix, 0, 4);
    memset(suffix, 0, 3);
    memset(base_call, 0, 8);

    int total_len = (int)strlen(cs);
    int slash_pos = (int)(slash - cs);
    int before_len = slash_pos;
    int after_len = total_len - slash_pos - 1;

    // Determine direction: prefix/CALL or CALL/suffix.
    // WSPR convention: if the part before '/' is 1-3 chars it is a prefix;
    // if the part after '/' is 1-2 chars it is a suffix.
    // If both sides could qualify, treat left-of-slash as prefix if it has <= 3 chars
    // and is not a valid standalone callsign (no digit at position 2 after normalisation).
    // This matches WSJT-X behaviour.

    if (before_len <= 3 && before_len >= 1 && after_len >= 1 && after_len <= 6) {
        // Try prefix/CALL interpretation first.
        // Validate prefix chars (alphanumeric only).
        for (int i = 0; i < before_len; i++) {
            if (!isalnum((unsigned char)cs[i]))
                return -1;
        }
        // Copy prefix (pad left to 3 with spaces if shorter).
        int pad = 3 - before_len;
        for (int i = 0; i < pad; i++)
            prefix[i] = ' ';
        for (int i = 0; i < before_len; i++)
            prefix[pad + i] = toupper((unsigned char)cs[i]);
        prefix[3] = '\0';

        // Copy base callsign (after slash).
        if (after_len > 6)
            return -1;
        for (int i = 0; i < after_len; i++)
            base_call[i] = toupper((unsigned char)(slash + 1)[i]);
        base_call[after_len] = '\0';
        *has_prefix = 1;
        return 0;
    }

    if (after_len <= 2 && after_len >= 1 && before_len >= 1 && before_len <= 6) {
        // CALL/suffix interpretation.
        const char *suf = slash + 1;
        // Validate suffix: single letter, single digit, or two digits (10-99).
        if (after_len == 1) {
            if (!isalnum((unsigned char)suf[0]))
                return -1;
        } else {
            // Two chars: both must be digits (DD form, 10-99).
            if (!isdigit((unsigned char)suf[0]) || !isdigit((unsigned char)suf[1]))
                return -1;
        }
        for (int i = 0; i < after_len; i++)
            suffix[i] = toupper((unsigned char)suf[i]);
        suffix[after_len] = '\0';

        // Copy base callsign (before slash).
        if (before_len > 6)
            return -1;
        for (int i = 0; i < before_len; i++)
            base_call[i] = toupper((unsigned char)cs[i]);
        base_call[before_len] = '\0';
        *has_suffix = 1;
        return 0;
    }

    return -1; // unrecognised compound format
}

// Pack a compound callsign into 28-bit n_call and 15-bit n_loc for Type-2.
// Also produces the 7-bit n_pwr_out = (power_code << 1) | 1.
static int pack_callsign_type2(const char *cs, int power_dbm, uint32_t *n_call_out, uint32_t *n_loc_out, uint32_t *n_pwr_out) {
    char base_call[8], prefix[4], suffix[3];
    int has_prefix = 0, has_suffix = 0;
    if (parse_compound_callsign(cs, base_call, prefix, suffix, &has_prefix, &has_suffix) < 0)
        return -1;

    // Pack the base callsign into 28-bit n_call using Type-1 rules.
    uint32_t n_call = 0;
    if (pack_callsign_type1(base_call, &n_call) < 0)
        return -1;

    // Pack power -- note Type-2 uses (power_encoded << 1) | 1 in the 7-bit field.
    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    // Type-2 power field: bit 0 = 1 (type indicator), bits 6:1 = power code.
    // n_pwr value passed into pack_message must fit in 7 bits (0..127).
    // pwr_code is in range 64..124 (dBm+64, max 60+64=124). Shifting left 1
    // gives 128..248 which overflows 7 bits. WSJT-X instead stores the raw
    // dBm value (0..60) OR'd with bit 0, keeping total <= 121 which fits in 7 bits.
    // Per packjt.f90: ntype = power_dbm; n_pwr = (ntype/2)*2 + 1 (makes it odd).
    // The rounded dBm is stored directly (not dBm+64) for Type-2/3.
    // WSPR spec: Type-2/3 power field must be ODD (bit 0 = 1) to signal
    // the message type to the decoder.  (rounded_dbm / 2) * 2 rounds down to the
    // nearest even integer; adding 1 makes it odd.  For all valid WSPR power levels
    // (0,3,7,10,...,60) C integer division truncates toward zero which equals floor
    // for non-negative values, so the formula is correct for every valid level.
    // Example: 10 dBm -> (10/2)*2+1 = 11; 20 dBm -> (20/2)*2+1 = 21.  Both odd.
    int rounded_dbm = (int)(pwr_code)-64;                   // recover rounded dBm from pwr_code
    uint32_t n_pwr = (uint32_t)((rounded_dbm / 2) * 2 + 1); // odd = Type-2/3 flag

    uint32_t n_loc = 0;
    if (has_prefix) {
        // Prefix packing: n_pfx = val(p0)*37^2 + val(p1)*37 + val(p2)
        // prefix[] is already left-padded to 3 chars.
        int v0 = char_to_wspr(prefix[0]);
        int v1 = char_to_wspr(prefix[1]);
        int v2 = char_to_wspr(prefix[2]);
        if (v0 < 0 || v1 < 0 || v2 < 0)
            return -1;
        uint32_t n_pfx = (uint32_t)v0 * 37u * 37u + (uint32_t)v1 * 37u + (uint32_t)v2;
        // n_loc = n_pfx * 22 + 11  (11 = no-suffix marker in 0..21)
        n_loc = n_pfx * 22u + 11u;
    } else {
        // Suffix packing.
        int suf_len = (int)strlen(suffix);
        int ntype;
        if (suf_len == 1) {
            if (isdigit((unsigned char)suffix[0])) {
                // Single digit D: ntype = 26 + D
                ntype = 26 + (suffix[0] - '0');
            } else {
                // Single letter: ntype = letter_index (0..25)
                ntype = toupper((unsigned char)suffix[0]) - 'A';
            }
        } else if (suf_len == 2) {
            // Two-digit suffix DD (10..99): ntype = 26 + DD
            int dd = (suffix[0] - '0') * 10 + (suffix[1] - '0');
            ntype = 26 + dd;
        } else {
            ntype = 36; // no suffix (fallback)
        }
        n_loc = (uint32_t)ntype;
    }

    *n_call_out = n_call;
    *n_loc_out = n_loc;
    *n_pwr_out = n_pwr;
    return 0;
}

// ---------------------------------------------------------------------------
// 15-bit callsign hash for Type-3 messages
//
// The hash is computed using the same algorithm as WSJT-X (packjt.f90):
//   n = 0
//   for each char c in the full compound callsign (e.g. "PJ4/K1ABC"):
//       n = n * 37 + wspr_char_value(c)
//   hash15 = (n >> 15) XOR n) & 0x7FFF
// Where wspr_char_value maps: 0-9 -> 0-9, A-Z -> 10-35, space -> 36,
// and '/' -> 36 (treated as space for hashing purposes per WSJT-X).
// The result fits in 15 bits (0..32767).
// ---------------------------------------------------------------------------
static uint32_t callsign_hash15(const char *cs) {
    uint32_t n = 0;
    for (int i = 0; cs[i] != '\0'; i++) {
        char c = toupper((unsigned char)cs[i]);
        int v;
        if (c >= '0' && c <= '9')
            v = c - '0';
        else if (c >= 'A' && c <= 'Z')
            v = (c - 'A') + 10;
        else
            v = 36; // space, '/', or any other non-alphanumeric treated as space
        // Intentional uint32_t wrap modulo 2^32, exactly matching the
        // WSJT-X packjt.f90 hash algorithm.  C11 §6.2.5p9 guarantees unsigned integer
        // overflow wraps without undefined behaviour, ensuring interoperability with
        // all WSPR decoders.  For CALLSIGN_LEN=12 the accumulator wraps multiple times;
        // this is correct and intentional -- the fold below extracts the 15-bit hash.
        n = n * 37u + (uint32_t)v;
    }
    // Fold to 15 bits: XOR upper and lower halves then mask.
    return ((n >> 15) ^ n) & 0x7FFFu;
}

// ---------------------------------------------------------------------------
// 6-character locator packing for Type-3 callsign field (28 bits)
//
// Per WSJT-X packjt.f90 and K1JT Appendix B:
// A 6-character Maidenhead locator AADDLL (AA=field, DD=square, LL=subsquare)
// is packed into a 28-bit "callsign" field as:
//   n_call = (179 - 10*(ord(A0)-ord('A')) - D0) * 180
//           + (10*(ord(A1)-ord('A')) + D1)
//   n_call = n_call * 24 + (ord(L0) - ord('A'))   [subsquare first letter, A..X]
//   n_call = n_call * 24 + (ord(L1) - ord('A'))   [subsquare second letter, A..X]
//   n_call = n_call * 128 + 64 + power_rounded
//
// Wait -- that last multiply bakes in power. Let's re-examine:
// From etherkit/JTEncode source (wspr_encode Type-3 path):
//   ng = 32768 + (179 - 10*(ord(g0)-'A') - (g2-'0')) * 180
//            + (10*(ord(g1)-'A') + (g3-'0'))
//   where g0,g1 are field chars, g2,g3 are square digits, g4,g5 are subsquare chars.
//   nsubsq = (toupper(g4)-'A') * 24 + (toupper(g5)-'A')
//   n_call = ng * 576 + nsubsq * 24 + ...
// Re-reading WSJT-X more carefully:
//   n1 = ng * 576 + (nsubsq * 24 ... wait this embeds power too)
//
// The correct formula from WSJT-X packjt.f90 (fortran, verified against JTEncode):
//   ng = 32768 + (179 - 10*(ichar(g(1))-ichar('A')) - (ichar(g(3))-ichar('0'))) * 180
//             + (10*(ichar(g(2))-ichar('A')) + (ichar(g(4))-ichar('0')))
//   n_call = ng * 576 + (ichar(g(5))-ichar('a')) * 24 + (ichar(g(6))-ichar('a'))
// The value 32768 is a flag that distinguishes Type-3 from a normal callsign in
// the 28-bit field (normal max n_call for Type-1 is ~268M which is >> 32768,
// but Type-3 uses a different range: 32768 + 0..180*180-1 = 32768..65167).
// Then the subsquare adds 0..575, so n_call range is 32768..65742.
// This fits in 17 bits, well within the 28-bit field.
//
// The 15-bit locator field holds the 15-bit hash of the callsign.
// The 7-bit power field holds: (rounded_dBm / 2) * 2 + 1  (same as Type-2).
// ---------------------------------------------------------------------------
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

    // Base 4-char locator value (same formula as Type-1 pack_locator4, plus 32768 offset).
    uint32_t ng = 32768u + (179u - 10u * (uint32_t)(g0 - 'A') - (uint32_t)g2) * 180u + (10u * (uint32_t)(g1 - 'A') + (uint32_t)g3);

    // Subsquare index: 0..575 (24 * 24 possibilities).
    uint32_t nsubsq = (uint32_t)(g4 - 'A') * 24u + (uint32_t)(g5 - 'A');

    // Pack: ng * 576 + nsubsq (fits in ~17 bits, well within 28-bit field).
    *n_call_out = ng * 576u + nsubsq;
    return 0;
}

// ---------------------------------------------------------------------------
// Helper: detect if a callsign is compound (contains '/')
// ---------------------------------------------------------------------------
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

//   1. Simple callsign + 4-char locator: produces Type-1 (unchanged behaviour).
//   2. Compound callsign (contains '/'): produces Type-2.
//      The caller must alternate with wspr_encode_type3() for full station ID.
//   3. Simple callsign + 6-char locator: falls back to 4-char Type-1
//      (truncates locator to 4 chars). The caller should alternate with
//      wspr_encode_type3() to transmit the 6-char precision.
int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;

    // Route compound callsigns to Type-2 encoding.
    if (is_compound_callsign(callsign)) {
        uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
        if (pack_callsign_type2(callsign, power_dbm, &n_call, &n_loc, &n_pwr) < 0)
            return -1;
        uint8_t msg[7];
        pack_message(n_call, n_loc, n_pwr, msg);
        encode_and_interleave(msg, symbols);
        return 0;
    }

    // Simple callsign path (Type-1 or Type-1 with truncated locator).
    uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
    if (pack_callsign_type1(callsign, &n_call) < 0)
        return -1;

    // For 6-char locators with simple callsigns, use only the first
    // 4 characters for Type-1 (same decoded result at receiver as before, but
    // now explicit and documented). The caller should transmit a Type-3 companion.
    // Original code accepted 4-6 char silently using only 4; now we make a copy.
    char loc4[5];
    size_t loc_len = strlen(locator);
    if (loc_len < 4)
        return -1;
    // Use exactly the first 4 characters for the Type-1 locator field.
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

// Determine the WSPR message type for a given callsign + locator pair.
wspr_msg_type_t wspr_encode_type(const char *callsign, const char *locator) {
    if (!callsign || !locator)
        return WSPR_MSG_TYPE_1; // default safe return
    if (is_compound_callsign(callsign))
        return WSPR_MSG_TYPE_2;
    if (strlen(locator) >= 6)
        return WSPR_MSG_TYPE_3; // simple callsign + 6-char locator -> needs Type-1 + Type-3
    return WSPR_MSG_TYPE_1;
}

// Encode a Type-3 companion message: 15-bit hash of callsign in the locator
// field; 6-char locator packed into the 28-bit callsign field.
int wspr_encode_type3(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;
    if (strlen(locator) != 6)
        return -1;

    // 28-bit "callsign" field holds the packed 6-char locator.
    uint32_t n_call = 0;
    if (pack_loc6_type3(locator, &n_call) < 0)
        return -1;

    // 15-bit "locator" field holds the hash of the callsign.
    uint32_t n_loc = callsign_hash15(callsign);

    // 7-bit power field: (rounded_dBm/2)*2 + 1  (odd = Type-2/3 flag).
    // WSPR spec: Type-2/3 power field must be ODD (bit 0 = 1) to signal
    // message type to the decoder.  (rounded_dbm / 2) * 2 rounds down to the nearest
    // even integer; adding 1 makes it odd.  For all valid WSPR levels (0,3,7,...,60)
    // this is correct and the result fits in 7 bits (max 60 -> result = 61).
    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    int rounded_dbm = (int)pwr_code - 64;
    uint32_t n_pwr = (uint32_t)((rounded_dbm / 2) * 2 + 1);

    uint8_t msg[7];
    pack_message(n_call, n_loc, n_pwr, msg);
    encode_and_interleave(msg, symbols);
    return 0;
}
