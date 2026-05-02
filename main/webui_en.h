/**
 * @file webui_en.h
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @brief WEBUI english translation.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#pragma once

// HTML lang attribute value
#define WEBUI_HTML_LANG "en"

#define WEBUI_HTTP_AUTH_WARNING "The configuration interface has no authentication. Use on a trusted local network only."

// Station card
#define WEBUI_CARD_STATION_TITLE "Station"
#define WEBUI_LABEL_CALLSIGN     "Callsign"
// Clarify that 6-char locators are stored and transmitted via
// alternating Type-1 (4-char) + Type-3 (6-char) slots for sub-square precision.
#define WEBUI_LABEL_LOCATOR "Maidenhead Locator (4 char. = Type-1 only; 6 char. = alternating Type-1 + Type-3)"
// Status panel label for the locator-truncation notice row
#define WEBUI_STATUS_LOC_NOTE_LABEL "Locator precision"
#define WEBUI_JS_LOC_TRUNCATED_NOTE "6-char stored, Type-1 (4-char) + Type-3 (6-char) alternating TX"
#define WEBUI_LABEL_POWER           "TX Power (dBm)"
#define WEBUI_LABEL_XTAL_CAL        "XTAL Calibration (ppb)"
#define WEBUI_HINT_XTAL_CAL         "Crystal offset in ppb. 0=no correction."
#define WEBUI_BTN_SAVE              "&#128190; Save configuration"

// "From GPS" button label — shown to the right of the Maidenhead locator input.
// Button is enabled when GPS module is detected, disabled when NTP mode is active.
#define WEBUI_BTN_FROM_GPS "&#128204; GPS"

// WiFi card
#define WEBUI_CARD_WIFI_TITLE "WiFi Network (STA mode)"
#define WEBUI_BTN_SCAN_LABEL  "&#128268; Scan WiFi networks..."
#define WEBUI_LABEL_PASSWORD  "Password"
#define WEBUI_SHOW_PASS_TEXT  "Show password"
#define WEBUI_LABEL_NTP       "NTP Server"
#define WEBUI_HINT_NO_CRED    "No credentials &#8594; AP mode (192.168.4.1)"

// Bands card
#define WEBUI_CARD_BANDS_TITLE "Active bands"

// IARU region card strings
#define WEBUI_CARD_IARU_TITLE   "IARU Region &amp; 60 m Frequency"
#define WEBUI_LABEL_IARU_REGION "IARU Region"
#define WEBUI_IARU_REGION_1     "Region 1 &mdash; Europe, Africa, Middle East (5.2886 MHz)"
#define WEBUI_IARU_REGION_2     "Region 2 &mdash; Americas (5.3465 MHz)"
#define WEBUI_IARU_REGION_3     "Region 3 &mdash; Asia, Pacific (5.3670 MHz)"
#define WEBUI_HINT_IARU         "Selects the 60 m WSPR dial frequency for your ITU/IARU zone. All other bands use the same frequency worldwide."

// Hop card
#define WEBUI_CARD_HOP_TITLE     "Frequency hopping"
#define WEBUI_TOGGLE_HOP_LABEL   "Enable automatic hopping"
#define WEBUI_LABEL_HOP_INTERVAL "Interval (s):"
#define WEBUI_HINT_HOP           "The transmitter rotates through selected bands every N seconds (min. 120 s = 1 TX)."

// Duty cycle card
#define WEBUI_CARD_DUTY_TITLE  "TX Duty Cycle"
#define WEBUI_LABEL_DUTY       "Active TX slots percentage (0-100 %)"
#define WEBUI_HINT_DUTY_INLINE "&nbsp;% (0=never, 20=WSPR standard, 100=always)"
#define WEBUI_HINT_DUTY        "20% transmits 1 in 5 slots (WSPR recommended). 100% uses all available slots."

// TX control card
#define WEBUI_CARD_TX_TITLE "TX Control"

// Status row labels
#define WEBUI_STATUS_HW_LABEL   "RF Hardware"
#define WEBUI_STATUS_TIME_LABEL "Time synchronization"
#define WEBUI_STATUS_BAND_LABEL "Current band"
#define WEBUI_STATUS_FREQ_LABEL "Frequency"
#define WEBUI_STATUS_NEXT_LABEL "Next TX"
#define WEBUI_STATUS_TX_LABEL   "Active TX"
#define WEBUI_STATUS_SYM_LABEL  "Symbol"

// PSKReporter map link
#define WEBUI_WSPR_LINK_TEXT "&#127760; View my spots on PSKReporter &#8599;"

// JS strings
#define WEBUI_JS_BTN_TX_START     "Enable TX"
#define WEBUI_JS_BTN_TX_STOP      "Stop TX"
#define WEBUI_JS_ERR_LOAD         "Error loading config: "
#define WEBUI_JS_SAVED            "Configuration saved"
#define WEBUI_JS_ERR_TX           "TX Error: "
#define WEBUI_JS_NOT_SYNCED       "Not synchronized"
#define WEBUI_JS_TRANSMITTING     "Transmitting"
#define WEBUI_JS_SCANNING_TEXT    "Scanning..."
#define WEBUI_JS_BTN_SCAN_RESTORE "\xf0\x9f\x94\x8c Scan WiFi networks..."
#define WEBUI_JS_NO_NETS          "No networks found"
#define WEBUI_JS_HIDDEN           "(hidden)"
#define WEBUI_JS_ERR_SCAN         "WiFi scan error: "
#define WEBUI_JS_PASS_SAVED       "(saved password)"
#define WEBUI_JS_CONFIRM_RESET    "Restart ESP32?"
#define WEBUI_JS_RESTARTING       "Restarting..."

#define WEBUI_REBOOT_INFO_PREFIX  "Boot: "
#define WEBUI_REBOOT_CAUSE_PREFIX " \u2502 Cause: "

#define WEBUI_JS_ERR_CALLSIGN "Invalid callsign (3-11 chars, A-Z 0-9 and space only)"
#define WEBUI_JS_ERR_LOCATOR  "Invalid locator (4 or 6 chars: AA00 or AA00AA format, e.g. GF05 or GF05ab)"

// JS toast message when GPS locator fetch fails
#define WEBUI_JS_GPS_LOC_ERR "GPS position not yet available"

// Tone test button labels
#define WEBUI_JS_BTN_TONE_START "Test Tone"
#define WEBUI_JS_BTN_TONE_STOP  "Stop Tone"
#define WEBUI_JS_TONE_FREQ_HINT "kHz"
#define WEBUI_JS_TONE_FREQ_ERR  "Invalid frequency (0.1 to "

// label shown above the measured-frequency input in the Station card
#define WEBUI_LABEL_TONE_RECV "Test tone received"
