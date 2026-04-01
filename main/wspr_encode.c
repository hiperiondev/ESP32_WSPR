/**
 * @file wspr_encode.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <ctype.h>
#include <string.h>

#include "esp_log.h"
#include "wspr_encode.h"

static const char *TAG = "wspr_enc";

// Sync vector (162 bits)
static const uint8_t SYNC[WSPR_SYMBOLS] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0,
                                            0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1,
                                            0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
                                            1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 };

// ---------------------------------------------------------------------------
// Character value helpers (shared by all message types)
// ---------------------------------------------------------------------------

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

static int suffix_val(char c) {
    c = toupper((unsigned char)c);
    if (c >= 'A' && c <= 'Z')
        return (int)(c - 'A');
    return 26; // space
}

static int digit_val(char c) {
    return (c >= '0' && c <= '9') ? (c - '0') : -1;
}

static int pack_callsign_type1(const char *cs, uint32_t *out) {
    char buf[7] = "      ";
    int len = (int)strlen(cs);

    if (len < 1 || len > 6)
        return -1;
    memcpy(buf, cs, (size_t)len);
    buf[6] = '\0';

    for (int i = 0; i < 6; i++) {
        if (buf[i] == '/')
            return -1;
    }

    if (!isdigit((unsigned char)buf[2])) {
        if (len > 5)
            return -1;
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
        return -1;

    uint32_t n = (uint32_t)c0;
    n = n * 36u + (uint32_t)c1;
    n = n * 10u + (uint32_t)c2;
    n = n * 27u + (uint32_t)c3;
    n = n * 27u + (uint32_t)c4;
    n = n * 27u + (uint32_t)c5;
    *out = n;

    return 0;
}

static int pack_locator4(const char *loc, uint32_t *out) {
    size_t len = strlen(loc);

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

static int pack_power(int dbm, uint32_t *out) {
    static const int valid[] = { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 };
    for (int i = 0; i < 19; i++) {
        if (valid[i] == dbm) {
            *out = (uint32_t)(dbm + 64);
            return 0;
        }
    }

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
    msg[0] = (uint8_t)(n_call >> 20);
    msg[1] = (uint8_t)(n_call >> 12);
    msg[2] = (uint8_t)(n_call >> 4);
    msg[3] = (uint8_t)((n_call & 0x0Fu) << 4) | (uint8_t)((n_loc >> 11) & 0x0Fu);
    msg[4] = (uint8_t)(n_loc >> 3);
    msg[5] = (uint8_t)((n_loc & 0x07u) << 5) | (uint8_t)((n_pwr >> 2) & 0x1Fu);
    msg[6] = (uint8_t)((n_pwr & 0x03u) << 6);
}

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

static void encode_and_interleave(const uint8_t msg[7], uint8_t symbols[WSPR_SYMBOLS]) {
    uint8_t enc[WSPR_SYMBOLS];
    conv_encode(msg, enc);
    uint8_t il[WSPR_SYMBOLS];
    interleave(enc, il);
    for (int i = 0; i < WSPR_SYMBOLS; i++)
        symbols[i] = 2 * il[i] + SYNC[i];
}

static int parse_compound_callsign(const char *cs, char base_call[8], char prefix[4], char suffix[3], int *has_prefix, int *has_suffix) {
    const char *slash = NULL;
    for (const char *p = cs; *p; p++) {
        if (*p == '/') {
            if (slash != NULL)
                return -1;
            slash = p;
        }
    }
    if (slash == NULL)
        return -1;

    *has_prefix = 0;
    *has_suffix = 0;
    memset(prefix, 0, 4);
    memset(suffix, 0, 3);
    memset(base_call, 0, 8);

    int total_len = (int)strlen(cs);
    int slash_pos = (int)(slash - cs);
    int before_len = slash_pos;
    int after_len = total_len - slash_pos - 1;

    if (before_len <= 3 && before_len >= 1 && after_len >= 1 && after_len <= 6) {
        for (int i = 0; i < before_len; i++) {
            if (!isalnum((unsigned char)cs[i]))
                return -1;
        }
        int pad = 3 - before_len;
        for (int i = 0; i < pad; i++)
            prefix[i] = ' ';
        for (int i = 0; i < before_len; i++)
            prefix[pad + i] = toupper((unsigned char)cs[i]);
        prefix[3] = '\0';

        if (after_len > 6)
            return -1;
        for (int i = 0; i < after_len; i++)
            base_call[i] = toupper((unsigned char)(slash + 1)[i]);
        base_call[after_len] = '\0';
        *has_prefix = 1;
        return 0;
    }

    if (after_len <= 2 && after_len >= 1 && before_len >= 1 && before_len <= 6) {
        const char *suf = slash + 1;
        if (after_len == 1) {
            if (!isalnum((unsigned char)suf[0]))
                return -1;
        } else {
            if (!isdigit((unsigned char)suf[0]) || !isdigit((unsigned char)suf[1]))
                return -1;
        }
        for (int i = 0; i < after_len; i++)
            suffix[i] = toupper((unsigned char)suf[i]);
        suffix[after_len] = '\0';

        if (before_len > 6)
            return -1;
        for (int i = 0; i < before_len; i++)
            base_call[i] = toupper((unsigned char)cs[i]);
        base_call[before_len] = '\0';
        *has_suffix = 1;
        return 0;
    }

    return -1;
}

static int pack_callsign_type2(const char *cs, int power_dbm, uint32_t *n_call_out, uint32_t *n_loc_out, uint32_t *n_pwr_out) {
    char base_call[8], prefix[4], suffix[3];
    int has_prefix = 0, has_suffix = 0;
    if (parse_compound_callsign(cs, base_call, prefix, suffix, &has_prefix, &has_suffix) < 0)
        return -1;

    uint32_t n_call = 0;
    if (pack_callsign_type1(base_call, &n_call) < 0)
        return -1;

    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    int rounded_dbm = (int)(pwr_code)-64;
    uint32_t n_pwr = (uint32_t)((rounded_dbm / 2) * 2 + 1);

    uint32_t n_loc = 0;
    if (has_prefix) {
        int v0 = char_to_wspr(prefix[0]);
        int v1 = char_to_wspr(prefix[1]);
        int v2 = char_to_wspr(prefix[2]);
        if (v0 < 0 || v1 < 0 || v2 < 0)
            return -1;
        uint32_t n_pfx = (uint32_t)v0 * 37u * 37u + (uint32_t)v1 * 37u + (uint32_t)v2;
        uint32_t n_loc_candidate = n_pfx * 22u + 11u;
        if (n_loc_candidate > 32767u) {
            return -1;
        }
        n_loc = n_loc_candidate;
    } else {
        int suf_len = (int)strlen(suffix);
        int ntype;
        if (suf_len == 1) {
            if (isdigit((unsigned char)suffix[0])) {
                ntype = 26 + (suffix[0] - '0');
            } else {
                ntype = toupper((unsigned char)suffix[0]) - 'A';
            }
        } else if (suf_len == 2) {
            int dd = (suffix[0] - '0') * 10 + (suffix[1] - '0');
            ntype = 26 + dd;
        } else {
            ntype = 36;
        }
        n_loc = (uint32_t)ntype;
    }

    *n_call_out = n_call;
    *n_loc_out = n_loc;
    *n_pwr_out = n_pwr;
    return 0;
}

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
            v = 36;

        n = n * 37u + (uint32_t)v;
    }

    return ((n >> 15) ^ n) & 0x7FFFu;
}

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

    uint32_t ng = 32768u + (179u - 10u * (uint32_t)(g0 - 'A') - (uint32_t)g2) * 180u + (10u * (uint32_t)(g1 - 'A') + (uint32_t)g3);
    uint32_t nsubsq = (uint32_t)(g4 - 'A') * 24u + (uint32_t)(g5 - 'A');
    *n_call_out = ng * 576u + nsubsq;

    return 0;
}

static int is_compound_callsign(const char *cs) {
    for (int i = 0; cs[i] != '\0'; i++) {
        if (cs[i] == '/')
            return 1;
    }
    return 0;
}

int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;

    if (is_compound_callsign(callsign)) {
        uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
        if (pack_callsign_type2(callsign, power_dbm, &n_call, &n_loc, &n_pwr) == 0) {
            uint8_t msg[7];
            pack_message(n_call, n_loc, n_pwr, msg);
            encode_and_interleave(msg, symbols);
            return 0;
        }

        const char *slash = NULL;
        for (const char *p = callsign; *p; p++) {
            if (*p == '/') {
                slash = p;
                break;
            }
        }

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

        char base_buf[7] = { 0 };
        int copy_len = (base_len > 6) ? 6 : base_len;
        for (int i = 0; i < copy_len; i++)
            base_buf[i] = base_start[i];

        if (pack_callsign_type1(base_buf, &n_call) == 0) {
            char loc4[5];
            size_t loc_len = strlen(locator);
            if (loc_len >= 4) {
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

        return -1;
    }

    uint32_t n_call = 0, n_loc = 0, n_pwr = 0;
    if (pack_callsign_type1(callsign, &n_call) < 0)
        return -1;

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
        uint32_t nc = 0, nl = 0, np = 0;
        if (pack_callsign_type2(callsign, 20, &nc, &nl, &np) == 0)
            return WSPR_MSG_TYPE_2;

        return WSPR_MSG_TYPE_1;
    }
    if (strlen(locator) >= 6)
        return WSPR_MSG_TYPE_3;

    return WSPR_MSG_TYPE_1;
}

int wspr_encode_type3(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]) {
    if (!callsign || !locator)
        return -1;

    if (strlen(locator) != 6)
        return -1;

    uint32_t n_call = 0;
    if (pack_loc6_type3(locator, &n_call) < 0)
        return -1;

    uint32_t n_loc = callsign_hash15(callsign);
    uint32_t pwr_code = 0;
    pack_power(power_dbm, &pwr_code);
    int rounded_dbm = (int)pwr_code - 64;
    uint32_t n_pwr = (uint32_t)((rounded_dbm / 2) * 2 + 1);

    uint8_t msg[7];
    pack_message(n_call, n_loc, n_pwr, msg);
    encode_and_interleave(msg, symbols);

    return 0;
}
