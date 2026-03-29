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

/* ── Sync vector ─────────────────────────────────────────────────────────── */
static const uint8_t SYNC[WSPR_SYMBOLS] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0,
                                            0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1,
                                            0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
                                            1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 };

// Callsign packing per G4JNT WSPR specification:
//   - Pad to 6 characters with spaces on the right
//   - 0-9 maps to 0..9, A-Z maps to 10..35, space maps to 36
//   - Position 2 (0-indexed) MUST be a digit
//   - Pack: N=c0; N=N*36+c1; N=N*10+c2; N=N*27+c3; N=N*27+c4; N=N*27+c5

static int char_to_wspr(char c) {
    c = toupper((unsigned char)c);
    if (c >= '0' && c <= '9')
        return (int)(c - '0'); // 0..9
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A') + 10; // 10..35
    if (c == ' ')
        return 36;
    return -1;
}

// Suffix chars for positions 3-5 only: A-Z=0..25, space=26.
// These positions use a different 27-symbol alphabet than positions 0-1.
static int suffix_val(char c) {
    c = toupper((unsigned char)c);
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A');
    return 26; // space
}

static int digit_val(char c) {
    return (c >= '0' && c <= '9') ? c - '0' : -1;
}

static int pack_callsign(const char *cs, uint32_t *out) {
    char buf[7] = "      ";
    int len = (int)strlen(cs);
    if (len < 1 || len > 6)
        return -1;
    // WSPR rule: position 2 (0-indexed) MUST be a digit after normalization.
    // Right-pad callsign into buf first, then decide whether a prepend is needed.
    memcpy(buf, cs, (size_t)len);
    buf[6] = '\0';
    // If buf[2] is not a digit, shift right one position and prepend a space.
    // This handles one-letter-prefix callsigns like G4JNT -> " G4JNT",
    // W1AW -> " W1AW ", K1JT -> " K1JT ".
    if (!isdigit((unsigned char)buf[2])) {
        if (len > 5)
            return -1; // no room to prepend space
        // shift right from index len down to 1, then set buf[0]=' '
        for (int i = len; i > 0; i--)
            buf[i] = buf[i - 1];
        buf[0] = ' ';
    }
    // Position 2 must be a digit after normalization
    if (!isdigit((unsigned char)buf[2]))
        return -1;
    // Positions 0-1: full 37-symbol alphabet (0-9=0..9, A-Z=10..35, space=36)
    int c0 = char_to_wspr(buf[0]);
    int c1 = char_to_wspr(buf[1]);
    int c2 = (int)(buf[2] - '0'); // guaranteed digit 0-9
    // Positions 3-5: 27-symbol suffix alphabet (A-Z=0..25, space=26)
    int c3 = suffix_val(buf[3]);
    int c4 = suffix_val(buf[4]);
    int c5 = suffix_val(buf[5]);
    if (c0 < 0 || c1 < 0)
        return -1;
    // c1 can never be space: a one-letter prefix like "W" at buf[0] after prepend
    // leaves buf[1] as the first letter of the original callsign (e.g. '1' digit or
    // a letter). Space at buf[1] is impossible for any valid ham callsign.
    // The guard is kept as a safety net only.
    if (c1 > 35)
        return -1;

    // G4JNT standard packing formula
    uint32_t n = (uint32_t)c0;
    n = n * 36u + (uint32_t)c1;
    n = n * 10u + (uint32_t)c2;
    n = n * 27u + (uint32_t)c3;
    n = n * 27u + (uint32_t)c4;
    n = n * 27u + (uint32_t)c5;
    *out = n;
    return 0;
}

// accept LOCATOR_LEN=7 buffer (up to 6-char locators from web UI).
// Type-1 encoder uses only the first 4 characters (grid square + subsquare digit pair).
// Characters at positions 4-5 are accepted if they are spaces or absent; any
// non-space content at position 4+ indicates a 6-char locator which is not
// supported by the Type-1 message format - return error rather than silently
// truncating, so the caller can warn the user.
static int pack_locator(const char *loc, uint32_t *out) {
    size_t len = strlen(loc);
    if (len < 4)
        return -1;
    // Reject 6-char (extended) locators: positions 4-5 must be absent or space.
    // Type-1 WSPR encodes only a 4-char Maidenhead square; a 6-char locator
    // cannot be represented and would be silently truncated without this guard.
    if (len > 4) {
        for (size_t i = 4; i < len; i++) {
            if (loc[i] != ' ')
                return -1;
        }
    }
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

/* ── Power packing ───────────────────────────────────────────────────────── */
static int pack_power(int dbm, uint32_t *out) {
    static const int valid[] = { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 };
    for (int i = 0; i < 19; i++) {
        if (valid[i] == dbm) {
            *out = (uint32_t)(dbm + 64);
            return 0;
        }
    }
    /* Round to nearest valid */
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

static void pack_message(uint32_t n_call, uint32_t n_loc, uint32_t n_pwr, uint8_t msg[7]) {
    // 50-bit message left-justified in 7 bytes (56-bit space).
    // Bit layout (bit 55 = MSB of msg[0] bit 7):
    //   msg[0] = n_call[27:20]
    //   msg[1] = n_call[19:12]
    //   msg[2] = n_call[11:4]
    //   msg[3] = n_call[3:0]  | n_loc[14:11]
    //   msg[4] = n_loc[10:3]
    //   msg[5] = n_loc[2:0]   | n_pwr[6:2]
    //   msg[6] = n_pwr[1:0]   | 000000  (padding, never read by conv_encode)
    msg[0] = (uint8_t)(n_call >> 20);
    msg[1] = (uint8_t)(n_call >> 12);
    msg[2] = (uint8_t)(n_call >> 4);
    msg[3] = (uint8_t)((n_call & 0x0Fu) << 4) | (uint8_t)((n_loc >> 11) & 0x0Fu);
    msg[4] = (uint8_t)(n_loc >> 3);
    msg[5] = (uint8_t)((n_loc & 0x07u) << 5) | (uint8_t)((n_pwr >> 2) & 0x1Fu);
    msg[6] = (uint8_t)((n_pwr & 0x03u) << 6);
}

/* ── Convolutional encoder (K=32, rate 1/2) ──────────────────────────────── */
/* Polynomials: G1=0xF2D05351, G2=0xE4613C47 */
#define POLY1 0xF2D05351UL
#define POLY2 0xE4613C47UL

static void conv_encode(const uint8_t msg[7], uint8_t enc[WSPR_SYMBOLS]) {
    /* Input: 50 data bits + 31 tail bits = 81 bits → 162 output bits */
    uint32_t sr = 0;
    int out_idx = 0;

    for (int bit_idx = 0; bit_idx < 81; bit_idx++) {
        uint8_t in_bit;
        if (bit_idx < 50) {
            /* Extract bit from message (MSB first) */
            int byte_idx = bit_idx / 8;
            int bit_pos = 7 - (bit_idx % 8);
            in_bit = (msg[byte_idx] >> bit_pos) & 1;
        } else {
            in_bit = 0; /* tail bits */
        }

        sr = (sr << 1) | in_bit;

        /* Parity of (sr & POLY) = popcount & 1 */
        uint32_t p1 = sr & POLY1;
        uint32_t p2 = sr & POLY2;

        /* Population count parity */
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

/* ── Bit-reversal interleaver ────────────────────────────────────────────── */
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

/* ── Public API ──────────────────────────────────────────────────────────── */
int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    uint32_t n_call, n_loc, n_pwr;

    if (pack_callsign(callsign, &n_call) < 0)
        return -1;
    if (pack_locator(locator, &n_loc) < 0)
        return -1;
    pack_power(power_dbm, &n_pwr);

    uint8_t msg[7];
    pack_message(n_call, n_loc, n_pwr, msg);

    uint8_t enc[WSPR_SYMBOLS];
    conv_encode(msg, enc);

    uint8_t il[WSPR_SYMBOLS];
    interleave(enc, il);

    /* symbols[i] = 2*il[i] + SYNC[i] */
    for (int i = 0; i < WSPR_SYMBOLS; i++) {
        symbols[i] = 2 * il[i] + SYNC[i];
    }
    return 0;
}
