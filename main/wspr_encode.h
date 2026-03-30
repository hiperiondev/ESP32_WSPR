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

#include <stdbool.h>
#include <stdint.h>

// WSPR message type enumeration returned by wspr_encode_type().
// Callers must inspect this value to determine whether a second alternating
// transmission (Type-3 companion) is required.
typedef enum {
    WSPR_MSG_TYPE_1 = 1, // standard: simple callsign + 4-char locator + power
    WSPR_MSG_TYPE_2 = 2, // compound callsign + power (no locator; pair with Type-3)
    WSPR_MSG_TYPE_3 = 3, // hashed callsign + 6-char locator + power (companion to Type-2 or Type-1+6loc)
} wspr_msg_type_t;

// Total number of 4-FSK symbols in one WSPR transmission (all types).
// 162 = (50 data bits + 31 tail bits) * 2 (rate-1/2 code).
#define WSPR_SYMBOLS 162

// Encode a WSPR message into 162 4-FSK symbols (original signature preserved).
// he function fully handles compound callsigns (Type-2) and
// 6-character locators (Type-3) instead of silently corrupting them.
// For Type-1 (simple callsign + 4-char locator): encodes the normal Type-1 message.
// For compound callsign (callsign contains '/'): encodes the Type-2 message
//   (compound callsign in 28-bit field + suffix/prefix in 15-bit field).
//   The caller MUST also transmit the companion Type-3 message (wspr_encode_type3).
// For 6-char locator with simple callsign: encodes the Type-1 message using only
//   the first 4 characters of the locator for WSPRnet compatibility.
//   The caller SHOULD also transmit the companion Type-3 message for full precision.
// Returns 0 on success, -1 on invalid input.
// Use wspr_encode_type() to determine which message type was produced
// and whether a companion Type-3 transmission is needed.
int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]);

// Determine the WSPR message type that wspr_encode() would produce for given inputs,
// without actually encoding. Used by the caller to decide whether alternating
// Type-2 + Type-3 transmissions are needed.
// Returns WSPR_MSG_TYPE_1 for a plain callsign with up to 4-char locator.
// Returns WSPR_MSG_TYPE_2 for a compound callsign (contains '/').
// Returns WSPR_MSG_TYPE_3 if callsign is simple but locator is 6 characters
//   (caller wants the 6-char precision path: Type-1 + Type-3 alternation).
wspr_msg_type_t wspr_encode_type(const char *callsign, const char *locator);

// Encode the Type-3 companion message: hashed callsign in 15-bit locator field,
// 6-character locator packed into the 28-bit callsign field.
// Used as the alternating second transmission when:
//   - The primary message is Type-2 (compound callsign), OR
//   - The primary message is Type-1 but a 6-char locator is provided.
// Parameters:
//   callsign   - the FULL compound callsign string, e.g. "PJ4/K1ABC" or "K1ABC".
//                Used only to compute the 15-bit hash; the plain callsign is hashed
//                even when it was originally transmitted in compound form.
//   locator    - 6-character Maidenhead locator, e.g. "FN42AX".
//                Must be exactly 6 characters: [A-R][A-R][0-9][0-9][a-xA-X][a-xA-X].
//   power_dbm  - TX power in dBm (same rounding rules as wspr_encode()).
//   symbols    - output buffer of WSPR_SYMBOLS bytes; filled on success.
// Returns 0 on success, -1 on invalid input.
int wspr_encode_type3(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]);
