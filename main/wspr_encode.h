/**
 * @file wspr_encode.h
 * @brief WSPR message encoder — Type-1, Type-2, and Type-3 messages.
 * @copyright 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This module encodes WSPR (Weak Signal Propagation Reporter) messages into
 * arrays of 162 4-FSK symbols ready to be fed directly to the oscillator
 * driver.  All three standardised WSPR message types are supported:
 *
 * @par Message types
 * | Type | Callsign format    | Locator precision | Power field    | Companion? |
 * |------|--------------------|-------------------|----------------|------------|
 * | 1    | Simple (≤ 6 chars) | 4-char grid (DDLL)| Rounded dBm    | No         |
 * | 2    | Compound (has '/')  | None (via Type-3) | Odd dBm code   | Yes (Type-3)|
 * | 3    | 15-bit hash        | 6-char (DDLLSS)   | Odd dBm code   | Companion  |
 *
 * @par Type-1 messages
 * The standard WSPR message type.  Encodes a simple callsign (up to 6 characters,
 * digit at position 2, e.g. @c "LU3VEA", @c "W1AW", @c "G4JNT"), a 4-character
 * Maidenhead locator (e.g. @c "GF05"), and a TX power rounded to the nearest
 * legal WSPR value.  If a 6-character locator is supplied, only the first 4
 * characters are used for the Type-1 encoding; the caller should also transmit
 * a companion Type-3 message (using @ref wspr_encode_type3()) to convey the
 * sub-square precision.
 *
 * @par Type-2 messages
 * Used when the callsign contains a slash (@c '/'), indicating a compound
 * callsign with a prefix or suffix (e.g. @c "PJ4/K1ABC", @c "K1ABC/P").
 * The compound callsign occupies the 28-bit callsign field and the 15-bit
 * locator field carries the prefix/suffix encoded value.  A companion Type-3
 * message must be transmitted on the following even-minute slot to convey the
 * 6-character locator.  The TX parity alternation logic in @c wspr_transmit()
 * (@c main.c) handles the two-slot alternation automatically.
 *
 * @par Type-3 messages
 * The companion message for Type-2 and for Type-1 with 6-character locator.
 * Encodes the full 6-character Maidenhead sub-square locator in the 28-bit
 * callsign field and a 15-bit CRC hash of the callsign in the locator field,
 * allowing decoding software to associate the Type-2 and Type-3 messages.
 *
 * @par Encoding pipeline
 * Both @ref wspr_encode() and @ref wspr_encode_type3() run the same pipeline:
 *  1. Pack the three source fields into a 50-bit message word (7 bytes).
 *  2. Apply a rate-1/2 convolutional encoder with generators 0xF2D05351 and
 *     0xE4613C47, producing 162 code bits (81 input bits × 2).
 *  3. Interleave the 162 code bits using a bit-reversal permutation.
 *  4. XOR with the 162-bit WSPR synchronisation vector (SYNC[]).
 *  5. Write each of the 162 resulting values (0–3) as a 4-FSK symbol index
 *     into the output @p symbols array.
 *
 * @par Power rounding
 * WSPR specifies 19 legal TX power values (dBm): 0, 3, 7, 10, 13, 17, 20, 23,
 * 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60.  Any power value not in this set
 * is silently rounded to the nearest legal value before encoding.  For Type-2
 * and Type-3 messages the power code is forced to an odd number to signal the
 * special message type to decoding software.
 *
 * @par Thread safety
 * All functions are stateless (no global mutable state); they are safe to call
 * from any task.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup wspr_encode_api WSPR encoder API
 * @{
 */

// MODIFIED: replaced Doxygen block comments with // style throughout; added clarifying
// note for WSPR_MSG_TYPE_3 to document that the primary TX is still Type-1, not Type-3.

// WSPR message type returned by wspr_encode_type() for a given callsign+locator pair.
// Callers must inspect this value to decide whether a companion Type-3 slot is needed.
typedef enum {
    // Standard Type-1 message: simple callsign (no '/') + 4-char locator.
    // The full callsign, 4-character grid square, and rounded TX power are encoded.
    // No companion message is needed; transmit this type every available slot.
    WSPR_MSG_TYPE_1 = 1,

    // Type-2 compound-callsign message: callsign contains '/' (e.g. "PJ4/K1ABC").
    // The 28-bit field encodes the base callsign; the 15-bit field encodes the
    // prefix or suffix indicator. No locator is included in the Type-2 frame.
    // A companion Type-3 message (wspr_encode_type3) MUST be transmitted on the
    // following even-minute slot to provide the 6-character locator.
    // The scheduler alternates: parity==0 -> Type-2, parity==1 -> Type-3.
    WSPR_MSG_TYPE_2 = 2,

    // Returned by wspr_encode_type() when a SIMPLE callsign + 6-char locator is used.
    // NOTE: this value signals "alternation needed" — the PRIMARY transmission on
    // parity==0 slots is still a Type-1 message (first 4 locator chars only).
    // A companion Type-3 message (wspr_encode_type3) follows on parity==1 slots to
    // convey the full 6-character sub-square locator and the callsign hash.
    // The scheduler alternates: parity==0 -> Type-1 (4-char loc), parity==1 -> Type-3.
    WSPR_MSG_TYPE_3 = 3,
} wspr_msg_type_t;

/**
 * @brief Total number of 4-FSK symbols in one WSPR transmission (all types).
 *
 * Derived from the WSPR specification: 50 data bits + 31 tail bits = 81 bits
 * fed into a rate-1/2 convolutional encoder, producing 162 output bits.
 * Each output bit, after interleaving and XOR with the sync vector, becomes
 * one 4-FSK symbol (2 bits: data bit XOR sync bit).
 *
 * A single WSPR transmission (162 symbols × 8192/12000 s/symbol) lasts
 * approximately 110.6 seconds.  The symbol period used in this firmware is
 * exactly 2 048 000 µs / 3 = 682 666.67 µs per symbol.
 */
#define WSPR_SYMBOLS 162

/**
 * @brief Encode a WSPR message into 162 4-FSK symbols.
 *
 * Determines the appropriate WSPR message type from the callsign and locator,
 * packs the input fields, runs the convolutional encoder, bit-reversal
 * interleaver, and sync-vector XOR, then writes the resulting symbol array.
 *
 * @par Callsign handling
 *  - Simple callsign (no @c '/'): encoded as Type-1 using the standard 28-bit
 *    packing (up to 6 alphanumeric characters, digit required at position 2).
 *    If the callsign is shorter than 6 characters and does not have a digit at
 *    position 2, a leading space is prepended automatically.
 *  - Compound callsign (contains @c '/'): encoded as Type-2 using
 *    @c pack_callsign_type2().  If the prefix overflows the 15-bit field
 *    (n_loc > 32767), the function falls back to encoding the base callsign
 *    portion as a Type-1 message (with a log warning) rather than failing.
 *
 * @par Locator handling
 *  - 4-character locator: used directly in the 15-bit locator field (Type-1).
 *  - 6-character locator with simple callsign: only the first 4 characters are
 *    packed into this (Type-1) message.  The caller should also transmit the
 *    companion Type-3 message produced by @ref wspr_encode_type3() to convey
 *    sub-square precision.  Use @ref wspr_encode_type() to determine whether
 *    a Type-3 companion is needed.
 *
 * @par Power rounding
 * @p power_dbm is rounded to the nearest value in the set
 * {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60}.
 * For Type-2 encoding the rounded power code is forced odd as required by the
 * WSPR protocol.
 *
 * @param[in]  callsign   NUL-terminated amateur callsign string.
 *                        Simple: 1–6 alphanumeric chars, digit at position 2
 *                        (e.g. @c "LU3VEA", @c "W1AW").
 *                        Compound: PREFIX/CALL or CALL/SUFFIX
 *                        (e.g. @c "PJ4/K1ABC", @c "K1ABC/P").
 *                        Must not be @c NULL.
 * @param[in]  locator    NUL-terminated Maidenhead locator.
 *                        Must be 4 characters (e.g. @c "GF05") for pure Type-1,
 *                        or 6 characters (e.g. @c "GF05ab") when sub-square
 *                        precision is desired (pair with @ref wspr_encode_type3()).
 *                        Must not be @c NULL.
 * @param[in]  power_dbm  Transmit power in dBm.  Rounded to the nearest legal
 *                        WSPR value.  Practical range: 0–60 dBm.
 * @param[out] symbols    Output buffer of @ref WSPR_SYMBOLS bytes.  Each byte
 *                        receives a value in the range 0–3 representing the
 *                        4-FSK tone index for that symbol position.
 *                        The oscillator driver converts tone index @p t to a
 *                        milli-Hz offset of @c t × 375000 / 256 ≈ t × 1464.844 mHz.
 *                        Must not be @c NULL.
 *
 * @return @c 0 on success.
 * @return @c -1 if @p callsign or @p locator is @c NULL, if the callsign cannot
 *         be encoded in any supported format, or if the locator is shorter than
 *         4 characters.
 */
int wspr_encode(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]);

/**
 * @brief Determine which WSPR message type @ref wspr_encode() would produce.
 *
 * Performs the same type-selection logic as @ref wspr_encode() without
 * computing the symbol array.  Used by the scheduler in @c wspr_transmit()
 * (@c main.c) to decide:
 *  - @ref WSPR_MSG_TYPE_1: transmit a single Type-1 message every slot; no
 *    alternation needed.
 *  - @ref WSPR_MSG_TYPE_2: alternate between Type-2 (parity = 0) and Type-3
 *    (parity = 1) on successive even-minute slots.
 *  - @ref WSPR_MSG_TYPE_3: alternate between Type-1 (parity = 0, first 4
 *    locator chars) and Type-3 (parity = 1, full 6-char locator) on successive
 *    slots when a simple callsign with a 6-character locator is configured.
 *
 * @param[in] callsign  NUL-terminated callsign string.  May be @c NULL; if so,
 *                      @ref WSPR_MSG_TYPE_1 is returned.
 * @param[in] locator   NUL-terminated locator string.  May be @c NULL; if so,
 *                      @ref WSPR_MSG_TYPE_1 is returned.
 *
 * @return @ref WSPR_MSG_TYPE_1 — simple callsign with 4-char or shorter locator.
 * @return @ref WSPR_MSG_TYPE_2 — compound callsign that encodes successfully
 *                                 as Type-2 (@c pack_callsign_type2() succeeds).
 * @return @ref WSPR_MSG_TYPE_3 — simple callsign with a 6-character locator
 *                                 (the Type-1 + Type-3 alternation path).
 */
wspr_msg_type_t wspr_encode_type(const char *callsign, const char *locator);

/**
 * @brief Encode the Type-3 companion message.
 *
 * Produces the second slot of the two-slot alternating sequence used for:
 *  - Compound callsigns (Type-2 primary + Type-3 companion), and
 *  - Simple callsigns with a 6-character locator (Type-1 primary + Type-3 companion).
 *
 * @par Type-3 field encoding
 * The 28-bit "callsign" field carries the 6-character Maidenhead sub-square,
 * packed as:
 * @code
 *   ng     = 32768 + (179 - 10*(G0-'A') - D0) * 180 + (10*(G1-'A') + D1)
 *   nsubsq = (G4-'A') * 24 + (G5-'A')
 *   n_call = ng * 576 + nsubsq
 * @endcode
 * where @c G0,G1 are the field letters (A–R), @c D0,D1 are the square digits (0–9),
 * and @c G4,G5 are the sub-square letters (A–X, case-insensitive).
 *
 * The 15-bit "locator" field carries a 15-bit CRC-style hash of the callsign
 * (@c callsign_hash15()), allowing decoding software to link the Type-3 message
 * to a previously received Type-1 or Type-2 message from the same station.
 *
 * The power code is forced odd (Type-2/3 indicator) as required by the WSPR
 * specification.
 *
 * @param[in]  callsign   NUL-terminated callsign string used @em only to
 *                        compute the 15-bit hash.  This must be the full
 *                        callsign (including any @c '/' prefix or suffix),
 *                        e.g. @c "PJ4/K1ABC" or @c "K1ABC".
 *                        Must not be @c NULL.
 * @param[in]  locator    NUL-terminated 6-character Maidenhead sub-square
 *                        locator, e.g. @c "GF05ab".
 *                        Must be exactly 6 characters:
 *                        [A-R][A-R][0-9][0-9][a-xA-X][a-xA-X].
 *                        Must not be @c NULL.
 * @param[in]  power_dbm  Transmit power in dBm.  Rounded to the nearest legal
 *                        WSPR value; the code is then forced odd for Type-3.
 * @param[out] symbols    Output buffer of @ref WSPR_SYMBOLS bytes.  Filled
 *                        with the encoded Type-3 symbol sequence.
 *                        Must not be @c NULL.
 *
 * @return @c 0 on success.
 * @return @c -1 if @p callsign or @p locator is @c NULL, if @p locator is not
 *         exactly 6 characters, or if any locator character is out of range.
 */
int wspr_encode_type3(const char *callsign, const char *locator, int power_dbm, uint8_t symbols[WSPR_SYMBOLS]);

/** @} */
