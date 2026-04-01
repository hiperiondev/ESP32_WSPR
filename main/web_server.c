/**
 * @file web_server.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/semphr.h"

#include "config.h"
#include "oscillator.h"
#include "time_sync.h"
#include "version.h"
#include "web_server.h"
#include "webui_strings.h"
#include "wifi_manager.h"

// HTTP Basic Authentication block — compiled in only when CONFIG_WSPR_HTTP_AUTH_ENABLE is set
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
#include "mbedtls/base64.h"

// When auth is active the warning banner is suppressed (empty string replaces it)
#define HTTP_AUTH_WARNING ""

/**
 * @brief Validate the HTTP Basic Authentication header of an incoming request.
 *
 * Extracts the "Authorization" header, verifies the "Basic " prefix, decodes
 * the Base64 payload with mbedTLS, and compares the resulting "user:pass"
 * string against the credentials stored in CONFIG_WSPR_HTTP_AUTH_USER and
 * CONFIG_WSPR_HTTP_AUTH_PASS (set in menuconfig).
 *
 * @param[in] req  Incoming httpd request handle.
 * @return true  — credentials are present and match.
 * @return false — header is absent, malformed, or credentials do not match.
 */
static bool check_auth(httpd_req_t *req) {
    char auth_hdr[160] = { 0 };
    // Retrieve the raw Authorization header value
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_hdr, sizeof(auth_hdr)) != ESP_OK)
        return false;

    // Only the "Basic" scheme is supported; reject Bearer and other schemes
    if (strncmp(auth_hdr, "Basic ", 6) != 0)
        return false;

    // Decode Base64 payload that follows the "Basic " prefix
    unsigned char decoded[128] = { 0 };
    size_t decoded_len = 0;
    const unsigned char *b64 = (const unsigned char *)(auth_hdr + 6);
    int rc = mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &decoded_len, b64, strlen(auth_hdr + 6));
    if (rc != 0)
        return false;
    decoded[decoded_len] = '\0';
    // Build expected "user:pass" string and compare with the decoded value
    char expected[160];
    snprintf(expected, sizeof(expected), "%s:%s", CONFIG_WSPR_HTTP_AUTH_USER, CONFIG_WSPR_HTTP_AUTH_PASS);
    return strcmp((char *)decoded, expected) == 0;
}

/**
 * @brief Send a 401 Unauthorized response that triggers the browser credential dialog.
 *
 * Sets the HTTP status to 401, adds a WWW-Authenticate header with the realm
 * "WSPR", and sends a plain-text body.  The browser's built-in Basic Auth
 * dialog will appear on the next request.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always (the error is encoded in the HTTP response, not the return value).
 */
static esp_err_t send_auth_challenge(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    // WWW-Authenticate header instructs the browser to prompt for credentials
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"WSPR\"");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "401 Unauthorized", -1);
    return ESP_OK;
}

#else
// When auth is disabled, inject a visible warning banner into the HTML page
#define HTTP_AUTH_WARNING "<p style='text-align:center;color:#92400e;font-size:.78em;margin-bottom:4px'> &#9888; " WEBUI_HTTP_AUTH_WARNING "</p>"
#endif

static const char *TAG = "web";
// Handle returned by httpd_start(); NULL when the server is not running
static httpd_handle_t _srv = NULL;
// Pointer to the live configuration structure shared with scheduler and status tasks
static wspr_config_t *_cfg = NULL;

// Mutex protecting _status (written by status_task, read by h_status handler)
static SemaphoreHandle_t _status_mutex = NULL;
// Mutex protecting *_cfg (shared by httpd task, scheduler_task, status_task)
static SemaphoreHandle_t _cfg_mutex = NULL;

// ─────────────────────────────────────────────────────────────────────────────
// Embedded single-page application (SPA)
// The entire HTML/CSS/JS UI is stored as a compile-time string literal.
// No filesystem or SPIFFS partition is needed; the page is served directly
// from flash.  Language strings are injected at compile time via the
// WEBUI_* macros defined in webui_strings.h (English or Spanish).
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] =
    "<!DOCTYPE html><html lang='" WEBUI_HTML_LANG "'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WSPR Transmitter</title>"
    "<style>"
    // CSS custom properties — define the colour palette for light-mode rendering
    ":root{--bg:#f5f4f0;--card:#ffffff;--border:#d0cfc9;--accent:#1a56db;"
    "--green:#1a7f37;--red:#cf222e;--text:#1c1c1c;--sub:#57534e}"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;"
    "min-height:100vh;padding:20px}"
    "h1{text-align:center;color:var(--accent);margin-bottom:6px;font-size:1.6em}"
    // Subtitle row below the title — shows boot time and reset cause
    ".reboot-info{text-align:center;color:var(--sub);font-size:.78em;margin-bottom:16px;"
    "min-height:1.2em;letter-spacing:.01em}"
    // Responsive card grid: two columns on wide screens, single column on mobile
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;max-width:900px;margin:0 auto}"
    ".card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:20px}"
    ".card h2{color:var(--accent);font-size:1em;margin-bottom:14px;display:flex;align-items:center;gap:8px}"
    "label{display:block;color:var(--sub);font-size:.8em;margin-bottom:4px;margin-top:12px}"
    "label:first-child{margin-top:0}"
    "input[type=text],input[type=number],input[type=password],select{"
    "width:100%;background:var(--bg);border:1px solid var(--border);color:var(--text);"
    "padding:8px 10px;border-radius:6px;font-size:.9em;outline:none;transition:.2s}"
    "input:focus,select:focus{border-color:var(--accent)}"
    // Band selection grid: 3 columns for the 12 WSPR bands (2200m through 10m)
    ".bands{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:4px}"
    ".band-cb{display:flex;align-items:center;gap:6px;background:var(--bg);"
    "border:1px solid var(--border);border-radius:6px;padding:6px 8px;cursor:pointer;"
    "transition:.2s;font-size:.85em}"
    ".band-cb:hover{border-color:var(--accent)}"
    ".band-cb input{width:16px;height:16px;cursor:pointer;accent-color:var(--accent)}"
    // iOS-style toggle switch used for hop enable and similar boolean settings
    ".toggle{display:flex;align-items:center;gap:10px;margin-top:14px}"
    ".switch{position:relative;width:46px;height:26px;flex-shrink:0}"
    ".switch input{opacity:0;width:0;height:0}"
    ".slider{position:absolute;inset:0;background:#b0aca6;border-radius:13px;"
    "transition:.3s;cursor:pointer}"
    ".slider:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;"
    "background:#fff;border-radius:50%;transition:.3s}"
    "input:checked+.slider{background:var(--accent)}"
    "input:checked+.slider:before{transform:translateX(20px)}"
    ".row{display:flex;gap:10px;align-items:center;margin-top:6px}"
    "button{padding:10px 20px;border:none;border-radius:6px;cursor:pointer;"
    "font-size:.9em;font-weight:600;transition:.2s}"
    ".btn-save{background:#b2f0e8;color:#0d4a42;width:100%;margin-top:18px}"
    ".btn-save:hover{background:#89e6d8;}"
    // TX button styling: green=idle, red=active
    ".btn-tx{padding:10px 20px;font-size:.9em;font-weight:600;border:none;"
    "border-radius:6px;cursor:pointer;transition:.2s}"
    ".btn-tx.on{background:var(--red);color:#fff}"
    ".btn-tx.off{background:var(--green);color:#000}"
    // Status panel inside the TX Control card
    ".status-box{background:var(--bg);border:1px solid var(--border);border-radius:8px;"
    "padding:14px;margin-top:14px}"
    ".status-row{display:flex;justify-content:space-between;padding:4px 0;"
    "border-bottom:1px solid var(--border);font-size:.85em}"
    ".status-row:last-child{border:none}"
    ".status-row span:first-child{color:var(--sub)}"
    // Coloured pill badges: ok=green, warn=amber, err=red
    ".badge{padding:2px 8px;border-radius:10px;font-size:.75em;font-weight:700}"
    ".badge.ok{background:#d1fae5;color:var(--green)}"
    ".badge.warn{background:#fef3c7;color:#92400e}"
    ".badge.err{background:#fee2e2;color:var(--red)}"
    // Transient notification toast — auto-dismisses after 3 s
    ".toast{position:fixed;top:20px;right:20px;background:var(--card);"
    "border:1px solid var(--border);border-radius:8px;padding:12px 20px;"
    "display:none;animation:fadein .3s}"
    "@keyframes fadein{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:none}}"
    ".hop-row{display:flex;align-items:center;gap:10px;margin-top:10px}"
    // WiFi scan button and dropdown list styles
    ".btn-scan{background:#e8e7e3;color:var(--accent);border:1px solid var(--border);"
    "padding:7px 14px;border-radius:6px;font-size:.82em;font-weight:600;"
    "cursor:pointer;margin-top:8px;transition:.2s;width:100%}"
    ".btn-scan:hover{border-color:var(--accent);background:#dddcda}"
    ".btn-scan:disabled{opacity:.4;cursor:default}"
    ".scan-list{margin-top:8px;display:none;max-height:180px;overflow-y:auto;"
    "border:1px solid var(--border);border-radius:6px;background:var(--bg)}"
    ".scan-item{display:flex;justify-content:space-between;align-items:center;"
    "padding:7px 10px;border-bottom:1px solid var(--border);cursor:pointer;"
    "font-size:.84em;transition:.15s}"
    ".scan-item:last-child{border:none}"
    ".scan-item:hover{background:#e8e7e3;color:var(--accent)}"
    ".scan-ssid{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}"
    ".scan-rssi{color:var(--sub);font-size:.78em;margin-left:8px;white-space:nowrap}"
    ".scan-lock{margin-left:6px;font-size:.85em}"
    ".pass-toggle{display:flex;align-items:center;gap:6px;margin-top:6px;font-size:.8em;color:var(--sub);cursor:pointer;user-select:none}"
    ".pass-toggle input[type=checkbox]{width:14px;height:14px;cursor:pointer;accent-color:var(--accent);flex-shrink:0}"
    // Reset button — styled in red to signal a destructive action
    ".btn-reset{background:#fff0f0;color:var(--red);border:1px solid var(--red);"
    "padding:5px 16px;border-radius:6px;font-size:.82em;font-weight:700;"
    "cursor:pointer;transition:.2s;display:block;margin:0 auto 20px}"
    ".btn-reset:hover{background:var(--red);color:#fff}"
    "</style></head><body>"
    "<h1>&#128225; WSPR Transmitter</h1>"
    // Firmware version string is embedded at compile time from version.h
    "<p style='text-align:center;color:#92400e;font-size:.78em;margin-bottom:4px'> FW Ver: " FW_VERSION_STRING "</p>" HTTP_AUTH_WARNING
    // Populated by pollStatus() JS once boot_time_str / reboot_reason are available
    "<div id='reboot_info' class='reboot-info'></div>"
    "<button class='btn-reset' onclick='resetESP()'>&#9211; Reset ESP32</button>"
    "<div class='grid'>"

    // ── Station card ──────────────────────────────────────────────────────────
    // Callsign: up to 11 chars (e.g. "LU3VEA", "PJ4/K1ABC").
    // WSPR encodes callsigns in 28 bits; simple calls fit Type-1, compound calls (with '/')
    // trigger Type-2 encoding with a companion Type-3 for the 6-char locator.
    "<div class='card'>"
    "<h2>&#128752; " WEBUI_CARD_STATION_TITLE "</h2>"
    "<label>" WEBUI_LABEL_CALLSIGN "</label>"
    "<input id='callsign' type='text' maxlength='11' placeholder='LU1ABC'>"
    // Maidenhead locator: 4-char (DDLL) for Type-1, 6-char (DDLLSS) for sub-square precision.
    // A 6-char locator causes the scheduler to alternate Type-1/Type-3 every TX slot.
    "<label>" WEBUI_LABEL_LOCATOR "</label>"
    "<input id='locator' type='text' maxlength='6' placeholder='GF05'>"
    // TX power: WSPR legal values are 0,3,7,10…60 dBm; the JS rounds to the nearest legal value
    "<label>" WEBUI_LABEL_POWER "</label>"
    "<input id='power' type='number' min='0' max='60' step='3'>"
    // Crystal calibration: signed ppb offset applied to the Si5351 PLL to correct TCXO/XO error.
    // Accurate frequency is critical for WSPR: the entire protocol bandwidth is only 200 Hz wide,
    // and the four FSK tones are separated by only ~1.46 Hz each.
    "<label>" WEBUI_LABEL_XTAL_CAL "</label>"
    "<input id='xtal_cal' type='number' min='-100000' max='100000' step='100'>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:4px'>" WEBUI_HINT_XTAL_CAL "</p>"
    "<button class='btn-save' onclick='save()'>" WEBUI_BTN_SAVE "</button>"
    "</div>"

    // ── WiFi card ─────────────────────────────────────────────────────────────
    // NTP synchronisation requires a working STA connection.
    // Without credentials the device starts in soft-AP mode (192.168.4.1).
    "<div class='card'>"
    "<h2>&#128246; " WEBUI_CARD_WIFI_TITLE "</h2>"
    "<label>SSID</label>"
    "<input id='wifi_ssid' type='text' maxlength='32' placeholder='Mi_Red'>"
    // Scan button triggers GET /api/wifi_scan and populates the dropdown
    "<button class='btn-scan' id='scan_btn' onclick='scanWifi()'>" WEBUI_BTN_SCAN_LABEL "</button>"
    "<div class='scan-list' id='scan_list'></div>"
    "<label>" WEBUI_LABEL_PASSWORD "</label>"
    // Password field: oninput sets _passEdited so the JS only sends it when changed
    "<input id='wifi_pass' type='password' maxlength='64' placeholder='&#8226;&#8226;&#8226;&#8226;&#8226;&#8226;&#8226;&#8226;' oninput='_passEdited=true'>"
    "<div class='pass-toggle' onclick='togglePassVis()'>"
    "<input type='checkbox' id='show_pass' onclick='event.stopPropagation();togglePassVis()'>"
    "<span>" WEBUI_SHOW_PASS_TEXT "</span>"
    "</div>"
    "<label>" WEBUI_LABEL_NTP "</label>"
    "<input id='ntp_server' type='text' maxlength='63' placeholder='pool.ntp.org'>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_NO_CRED "</p>"
    "</div>"

    // ── Active bands card ─────────────────────────────────────────────────────
    // The 12 WSPR bands from 2200m to 10m are displayed as checkboxes.
    // The enabled set is stored in cfg.band_enabled[] (one bool per band index).
    // When hop is enabled the scheduler cycles through the enabled bands every
    // hop_interval_sec seconds.
    "<div class='card'>"
    "<h2>&#128251; " WEBUI_CARD_BANDS_TITLE "</h2>"
    "<div class='bands' id='bands'></div>"
    "</div>"

    // ── IARU region card ──────────────────────────────────────────────────────
    // The 60 m WSPR frequency differs between IARU regions to avoid interference
    // with local secondary services in each ITU zone:
    //   Region 1 (EU/AF/ME): 5.2886 MHz
    //   Region 2 (Americas): 5.3465 MHz
    //   Region 3 (AS/PAC):   5.3670 MHz
    // All other bands use the same worldwide frequencies.
    "<div class='card'>"
    "<h2>&#127758; " WEBUI_CARD_IARU_TITLE "</h2>"
    "<label>" WEBUI_LABEL_IARU_REGION "</label>"
    "<select id='iaru_region'>"
    "<option value='1'>" WEBUI_IARU_REGION_1 "</option>"
    "<option value='2'>" WEBUI_IARU_REGION_2 "</option>"
    "<option value='3'>" WEBUI_IARU_REGION_3 "</option>"
    "</select>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_IARU "</p>"
    "</div>"

    // ── Frequency hopping card ────────────────────────────────────────────────
    // When hop is enabled the scheduler moves to the next enabled band after
    // hop_interval_sec seconds (minimum 120 s = one full WSPR TX slot of 110.6 s).
    // This distributes spots across bands to increase propagation coverage.
    "<div class='card'>"
    "<h2>&#128260; " WEBUI_CARD_HOP_TITLE "</h2>"
    "<div class='toggle'>"
    "<label class='switch'><input type='checkbox' id='hop_en'><span class='slider'></span></label>"
    "<span>" WEBUI_TOGGLE_HOP_LABEL "</span></div>"
    "<div class='hop-row'>"
    "<label style='margin:0;white-space:nowrap'>" WEBUI_LABEL_HOP_INTERVAL "</label>"
    "<input id='hop_interval' type='number' min='120' max='86400' step='120' style='width:100px'>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_HOP "</p>"
    "</div>"

    // ── TX duty cycle card ────────────────────────────────────────────────────
    // WSPR recommends a 20 % duty cycle (1 in 5 slots).  Using 100 % is legal
    // but considered poor practice as it prevents other stations from being heard.
    // The duty cycle is enforced in the scheduler via a pseudo-random slot selector
    // seeded from the system clock.
    "<div class='card'>"
    "<h2>&#128202; " WEBUI_CARD_DUTY_TITLE "</h2>"
    "<label>" WEBUI_LABEL_DUTY "</label>"
    "<div class='row'>"
    "<input id='tx_duty_pct' type='number' min='0' max='100' step='5' style='width:80px'>"
    "<span style='color:var(--sub);font-size:.85em'>" WEBUI_HINT_DUTY_INLINE "</span>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_DUTY "</p>"
    "</div>"

    // ── TX control / live status card ─────────────────────────────────────────
    // pollStatus() fetches GET /api/status every 2 s and updates all status rows.
    // The TX button calls POST /api/tx_toggle which atomically flips tx_enabled
    // and persists the change to NVS.
    "<div class='card'>"
    "<h2>&#9889; " WEBUI_CARD_TX_TITLE "</h2>"
    "<div class='toggle'>"
    "<button class='btn-tx off' id='tx_btn' onclick='toggleTx()'>" WEBUI_JS_BTN_TX_START "</button>"
    "</div>"
    "<div class='status-box'>"
    // RF hardware row: shows oscillator chip name (e.g. "Si5351A") and ok/error badge
    "<div class='status-row'><span>" WEBUI_STATUS_HW_LABEL "</span><span id='s_hw'><span class='badge warn'>---</span></span></div>"
    // Time sync row: shows UTC time string once NTP/GPS is locked
    "<div class='status-row'><span>" WEBUI_STATUS_TIME_LABEL "</span><span id='s_time'><span class='badge warn'>---</span></span></div>"
    // Current band row: shows the WSPR band currently selected (e.g. "20m")
    "<div class='status-row'><span>" WEBUI_STATUS_BAND_LABEL "</span><span id='s_band'>---</span></div>"
    // Frequency row: shows the dial frequency in MHz (e.g. "14.0971 MHz")
    "<div class='status-row'><span>" WEBUI_STATUS_FREQ_LABEL "</span><span id='s_freq'>---</span></div>"
    // Next TX countdown: seconds until the next even-minute TX window opens
    "<div class='status-row'><span>" WEBUI_STATUS_NEXT_LABEL "</span><span id='s_next'>---</span></div>"
    // Active TX row: ON (green) while the 162-symbol loop is running (~110.6 s)
    "<div class='status-row'><span>" WEBUI_STATUS_TX_LABEL "</span><span id='s_tx'><span class='badge err'>OFF</span></span></div>"
    // Symbol index row: counts 0-161 during transmission, giving a progress indicator
    "<div class='status-row'><span>" WEBUI_STATUS_SYM_LABEL "</span><span id='s_sym'>---</span></div>"
    "</div>"
    // Link to WSPRnet spot database, pre-filled with the configured callsign
    "<div style='margin-top:12px;text-align:center'>"
    "<a id='wspr_link' href='https://wsprnet.org' target='_blank' rel='noopener'"
    " style='color:var(--accent);font-size:.85em;text-decoration:none'>" WEBUI_WSPR_LINK_TEXT "</a></div>"
    "</div>"

    "</div>"
    "<div class='toast' id='toast'></div>"

    // ── Embedded JavaScript ───────────────────────────────────────────────────
    "<script>"
    // Band name list matches wspr_band_t enum order defined in config.h
    "const BANDS=['2200m','630m','160m','80m','60m','40m','30m','20m','17m','15m','12m','10m'];"
    "let cfg={};"

    // _hasPass: true when the server has a non-empty wifi_pass stored (shown as placeholder)
    "let _hasPass=false;"
    // _passEdited: set to true when the user types in the password field
    "let _passEdited=false;"
    // _rebootInfoSet: prevents overwriting the boot info subtitle on every poll cycle
    "let _rebootInfoSet=false;"
    // _settingsResetShown: ensures the factory-reset warning toast fires only once
    "let _settingsResetShown=false;"

    // Build the band checkbox grid dynamically from the BANDS array and enabled flags
    "function buildBands(enabled){"
    "const c=document.getElementById('bands');c.innerHTML='';"
    "BANDS.forEach((b,i)=>{"
    "const d=document.createElement('label');"
    "d.className='band-cb';"
    "d.innerHTML=`<input type='checkbox' value='${i}' ${enabled[i]?'checked':''}>${b}`;"
    "c.appendChild(d);});}"

    // Load current config from GET /api/config and populate all form fields
    "async function loadCfg(){"
    "try{"
    "const r=await fetch('/api/config');cfg=await r.json();"
    "document.getElementById('callsign').value=cfg.callsign||'';"
    "document.getElementById('locator').value=cfg.locator||'';"
    "document.getElementById('power').value=cfg.power_dbm||23;"
    "document.getElementById('xtal_cal').value=cfg.xtal_cal_ppb!=null?cfg.xtal_cal_ppb:0;"
    "document.getElementById('wifi_ssid').value=cfg.wifi_ssid||'';"
    // Never pre-fill the password field; show a placeholder when a saved password exists
    "document.getElementById('wifi_pass').value='';"
    "_hasPass=!!cfg.has_pass;"
    "_passEdited=false;"
    "document.getElementById('show_pass').checked=false;"
    "document.getElementById('wifi_pass').type='password';"
    "document.getElementById('wifi_pass').placeholder=_hasPass?'" WEBUI_JS_PASS_SAVED "':'\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022';"
    "document.getElementById('ntp_server').value=cfg.ntp_server||'pool.ntp.org';"
    "document.getElementById('hop_en').checked=!!cfg.hop_enabled;"
    "document.getElementById('hop_interval').value=cfg.hop_interval_sec||120;"
    "document.getElementById('tx_duty_pct').value=cfg.tx_duty_pct!=null?cfg.tx_duty_pct:20;"
    "document.getElementById('iaru_region').value=cfg.iaru_region||1;"
    "buildBands(cfg.band_enabled||Array(12).fill(false));"
    "updateTxBtn(cfg.tx_enabled);"
    // Build the WSPRnet spot-viewer link for the current callsign
    "const cs=(cfg.callsign||'').trim();"
    "if(cs){"
    "document.getElementById('wspr_link').href="
    "'https://wsprnet.org/drupal/wsprnet/spots?callsign='+encodeURIComponent(cs);"
    "}"
    "}catch(e){toast('" WEBUI_JS_ERR_LOAD "'+e,'err');}}"

    // Collect form values, validate, and POST to /api/config
    "async function save(){"
    "const bands=Array.from(document.querySelectorAll('#bands input')).map(i=>i.checked);"
    "const body={"
    "callsign:document.getElementById('callsign').value.toUpperCase().trim(),"
    "locator:document.getElementById('locator').value.toUpperCase().trim(),"
    "power_dbm:parseInt(document.getElementById('power').value)||23,"
    "xtal_cal_ppb:parseInt(document.getElementById('xtal_cal').value)||0,"
    "wifi_ssid:document.getElementById('wifi_ssid').value,"
    "ntp_server:document.getElementById('ntp_server').value||'pool.ntp.org',"
    "hop_enabled:document.getElementById('hop_en').checked,"
    "hop_interval_sec:parseInt(document.getElementById('hop_interval').value)||120,"
    "tx_duty_pct:parseInt(document.getElementById('tx_duty_pct').value)||20,"
    "iaru_region:parseInt(document.getElementById('iaru_region').value)||1,"
    "band_enabled:bands"
    "};"

    // Only include wifi_pass in the payload when the user has edited it
    "if(_passEdited){"
    "body.wifi_pass=document.getElementById('wifi_pass').value;"
    "}"

    // Client-side callsign validation mirrors the server-side validate_callsign() rules:
    // 3-11 characters, only A-Z, 0-9, space, and at most one '/'.
    "if(!body.callsign||body.callsign.length<3||body.callsign.length>11||"
    "!/^[A-Z0-9 /]{3,11}$/.test(body.callsign)||"
    "(body.callsign.match(/\\//g)||[]).length>1){"
    "toast('\\u274c " WEBUI_JS_ERR_CALLSIGN "','err');return;}"
    // Client-side Maidenhead locator validation: field letters A-R, digits 0-9, optional sub-square A-X
    "if(!body.locator||!/^[A-Ra-r]{2}[0-9]{2}([A-Xa-x]{2})?$/.test(body.locator)){"
    "toast('\\u274c " WEBUI_JS_ERR_LOCATOR "','err');return;}"
    // WSPR power validation: round to the nearest of the 19 legal dBm values
    // Legal values: 0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60
    "const validPowers=[0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60];"
    "if(!validPowers.includes(body.power_dbm)){"
    "let best=validPowers[0];let minDiff=Math.abs(body.power_dbm-best);for(let v of validPowers){let "
    "diff=Math.abs(body.power_dbm-v);if(diff<minDiff){minDiff=diff;best=v;}}"
    "body.power_dbm=best;"
    "toast('\\u26a0 Power rounded to '+best+' dBm (valid WSPR level)','warn');"
    "}"
    "try{"
    "const r=await fetch('/api/config',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify(body)});"
    "const j=await r.json();"
    "toast(j.ok?'\\u2705 " WEBUI_JS_SAVED "':'\\u274c Error: '+j.err,j.ok?'ok':'err');"
    // Reload config after a successful save to synchronise form fields with stored values
    "if(j.ok){_passEdited=false;loadCfg();}"
    "}catch(e){toast('Error: '+e,'err');}}"

    // Toggle TX enable: POST to /api/tx_toggle and update the button appearance
    "async function toggleTx(){"
    "try{"
    "const r=await fetch('/api/tx_toggle',{method:'POST'});"
    "const j=await r.json();"
    "updateTxBtn(j.tx_enabled);"
    "}catch(e){toast('" WEBUI_JS_ERR_TX "'+e,'err');}}"

    // Update TX button label and colour class to reflect the current tx_enabled state
    "function updateTxBtn(on){"
    "const b=document.getElementById('tx_btn');"
    "b.textContent=on?'" WEBUI_JS_BTN_TX_STOP "':'" WEBUI_JS_BTN_TX_START "';"
    "b.className='btn-tx '+(on?'on':'off');}"

    // Poll GET /api/status every 2 s and update all status row values
    "async function pollStatus(){"
    "try{"
    "const r=await fetch('/api/status');const s=await r.json();"
    // RF hardware badge: green if oscillator chip detected, red if running in dummy mode
    "document.getElementById('s_hw').innerHTML=s.hw_ok?"
    "`<span class='badge ok'>${s.hw_name}</span>`:"
    "`<span class='badge err'>${s.hw_name} \\u26a0</span>`;"
    // Time sync badge: green with UTC time string once NTP/GPS locked
    "document.getElementById('s_time').innerHTML=s.time_ok?"
    "`<span class='badge ok'>${s.time_str}</span>`:"
    "`<span class='badge warn'>" WEBUI_JS_NOT_SYNCED "</span>`;"
    "document.getElementById('s_band').textContent=s.band||'---';"
    "document.getElementById('s_freq').textContent=s.freq_str||'---';"
    // Countdown to next even-minute TX window; shows "Transmitting" when next_tx_sec <= 0
    "document.getElementById('s_next').textContent=s.next_tx_sec>0?s.next_tx_sec+'s':'" WEBUI_JS_TRANSMITTING "';"
    "document.getElementById('s_tx').innerHTML=s.tx_active?"
    "`<span class='badge ok'>ON</span>`:`<span class='badge err'>OFF</span>`;"
    // Symbol counter: shows n/162 during an active TX, "---" when idle
    "document.getElementById('s_sym').textContent=s.tx_active?(s.symbol_idx+'/162'):'---';"
    "updateTxBtn(s.tx_enabled);"
    // Populate boot info subtitle once (not on every poll) after the server provides it
    "if(!_rebootInfoSet&&(s.boot_time_str||s.reboot_reason)){"
    "let ri='';"
    "if(s.boot_time_str)ri+='" WEBUI_REBOOT_INFO_PREFIX "'+s.boot_time_str;"
    "if(s.reboot_reason)ri+='" WEBUI_REBOOT_CAUSE_PREFIX "'+s.reboot_reason;"
    "document.getElementById('reboot_info').textContent=ri;"
    "_rebootInfoSet=true;}"
    // Warn the user once if NVS settings were reset to factory defaults on boot
    "if(!_settingsResetShown&&s.settings_reset){"
    "toast('\\u26a0 Settings were reset to factory defaults!','warn');"
    "_settingsResetShown=true;}"
    "}catch(e){}}"

    // Display a transient notification toast for 3 s; colour depends on type (ok/warn/err)
    "function toast(msg,type){"
    "const t=document.getElementById('toast');t.textContent=msg;"
    "t.style.display='block';t.style.borderColor=type==='ok'?'var(--green)':type==='warn'?'#92400e':'var(--red)';"
    "setTimeout(()=>t.style.display='none',3000);}"

    // Trigger a blocking WiFi scan via GET /api/wifi_scan and render results as a dropdown
    "async function scanWifi(){"
    "const btn=document.getElementById('scan_btn');"
    "const lst=document.getElementById('scan_list');"
    "btn.disabled=true;"
    "btn.textContent='\\u23f3 " WEBUI_JS_SCANNING_TEXT "';"
    "lst.style.display='none';"
    "lst.innerHTML='';"
    "try{"
    "const r=await fetch('/api/wifi_scan');"
    "if(!r.ok){throw new Error('HTTP '+r.status);}"
    "const aps=await r.json();"
    "if(!aps||aps.length===0){"
    "lst.innerHTML=\"<div style='padding:10px;color:var(--sub);font-size:.84em;text-align:center'>" WEBUI_JS_NO_NETS "</div>\";"
    "lst.style.display='block';"
    "}else{"
    // Sort by descending RSSI so the strongest networks appear first
    "aps.sort((a,b)=>b.rssi-a.rssi);"
    "aps.forEach(ap=>{"
    "const row=document.createElement('div');"
    "row.className='scan-item';"
    // RSSI colour: green > -60 dBm, amber > -75 dBm, red otherwise
    "const rssiColor=ap.rssi>=-60?'var(--green)':ap.rssi>=-75?'#e3b341':'var(--red)';"
    "const lock=ap.auth?'&#128274;':'';"
    "row.innerHTML=`<span class='scan-ssid'>${ap.ssid||'" WEBUI_JS_HIDDEN "'}</span>`"
    "+`<span class='scan-rssi' style='color:${rssiColor}'>${ap.rssi} dBm</span>`"
    "+`<span class='scan-lock'>${lock}</span>`;"
    // Clicking a scan result populates the SSID field and focuses the password input
    "row.onclick=()=>{"
    "document.getElementById('wifi_ssid').value=ap.ssid||'';"
    "lst.style.display='none';"
    "document.getElementById('wifi_pass').focus();"
    "};"
    "lst.appendChild(row);"
    "});"
    "lst.style.display='block';"
    "}"
    "}catch(e){"
    "toast('" WEBUI_JS_ERR_SCAN "'+e,'err');"
    "}finally{"
    "btn.disabled=false;"
    "btn.textContent='" WEBUI_JS_BTN_SCAN_RESTORE "';"
    "}}"

    // Show/hide password field — displays placeholder text when a saved password exists
    // and the user has not typed a new one
    "function togglePassVis(){"
    "const cb=document.getElementById('show_pass');"
    "const inp=document.getElementById('wifi_pass');"
    "if(cb.checked){"
    "if(!_passEdited&&_hasPass){"
    "inp.type='text';"
    "inp.value='" WEBUI_JS_PASS_SAVED "';"
    "inp.style.color='var(--sub)';"
    "}else{"
    "inp.type='text';"
    "inp.style.color='';"
    "}"
    "}else{"
    "if(!_passEdited&&_hasPass){"
    "inp.value='';"
    "}"
    "inp.style.color='';"
    "inp.type='password';"
    "inp.placeholder=_hasPass&&!_passEdited?'" WEBUI_JS_PASS_SAVED "':'\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022\\u2022';"
    "}"
    "}"

    // Confirm and POST to /api/reset; reload page after 4 s to reconnect after reboot
    "async function resetESP(){"
    "if(!confirm('" WEBUI_JS_CONFIRM_RESET "'))return;"
    "try{await fetch('/api/reset',{method:'POST'});}catch(e){}"
    "toast('" WEBUI_JS_RESTARTING "','warn');"
    "setTimeout(()=>location.reload(),4000);}"

    // Bootstrap: load config on page load, then poll status every 2 s
    "loadCfg();setInterval(pollStatus,2000);"
    "</script></body></html>";

// ─────────────────────────────────────────────────────────────────────────────
// Internal status snapshot structure
// Written by web_server_update_status() and web_server_set_hw_status() under
// _status_mutex; read by h_status() under the same mutex.
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    bool time_ok;             // true once NTP/GPS has provided a valid UTC time
    char time_str[24];        // UTC time as human-readable string, e.g. "14:23:05 UTC"
    char band[8];             // current WSPR band name, e.g. "20m"
    char freq_str[20];        // dial frequency string, e.g. "14.0971 MHz"
    int32_t next_tx_sec;      // seconds until next even-minute TX window (-1 = not synced)
    bool tx_active;           // true while the 162-symbol transmission loop is running
    bool tx_enabled;          // shadow of cfg->tx_enabled for the status response
    int symbol_idx;           // WSPR symbol index currently being transmitted (0-161)
    bool hw_ok;               // true if a real oscillator chip was detected
    char hw_name[32];         // oscillator chip name string, e.g. "Si5351A"
    char reboot_time_str[32]; // formatted UTC time of the last boot
    char reboot_reason[32];   // human-readable reset cause, e.g. "Power-on"
} wspr_status_t;

// Module-level status cache; zero-initialised at startup
static wspr_status_t _status = { 0 };

void web_server_update_status(bool time_ok, const char *time_str, const char *band, const char *freq_str, int32_t next_tx_sec, bool tx_active, bool tx_enabled,
                              int symbol_idx) {
    // Acquire the status mutex with a short timeout to avoid blocking the scheduler
    if (_status_mutex && xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _status.time_ok = time_ok;
        _status.tx_active = tx_active;
        _status.tx_enabled = tx_enabled;
        _status.next_tx_sec = next_tx_sec;
        _status.symbol_idx = symbol_idx;
        // Only update string fields when the caller provides a non-NULL pointer
        if (time_str) {
            strncpy(_status.time_str, time_str, sizeof(_status.time_str) - 1);
            _status.time_str[sizeof(_status.time_str) - 1] = '\0';
        }
        if (band) {
            strncpy(_status.band, band, sizeof(_status.band) - 1);
            _status.band[sizeof(_status.band) - 1] = '\0';
        }
        if (freq_str) {
            strncpy(_status.freq_str, freq_str, sizeof(_status.freq_str) - 1);
            _status.freq_str[sizeof(_status.freq_str) - 1] = '\0';
        }
        xSemaphoreGive(_status_mutex);
    }
}

void web_server_set_hw_status(bool hw_ok, const char *hw_name) {
    // Called from app_main() before the server is fully started; write directly if mutex not yet created
    if (_status_mutex == NULL) {
        _status.hw_ok = hw_ok;
        if (hw_name) {
            strncpy(_status.hw_name, hw_name, sizeof(_status.hw_name) - 1);
            _status.hw_name[sizeof(_status.hw_name) - 1] = '\0';
        }
        return;
    }

    // Use a longer timeout here since hw status is set only once at startup
    if (xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "hw_status: mutex timeout, skipping update");
        return;
    }
    _status.hw_ok = hw_ok;
    if (hw_name) {
        strncpy(_status.hw_name, hw_name, sizeof(_status.hw_name) - 1);
        _status.hw_name[sizeof(_status.hw_name) - 1] = '\0';
    }
    xSemaphoreGive(_status_mutex);
    ESP_LOGI(TAG, "HW status: %s -- %s", hw_ok ? "OK" : "DUMMY", _status.hw_name);
}

void web_server_set_reboot_info(const char *boot_time_str, const char *reason_str) {
    // Both fields are optional: NULL leaves the previously stored value unchanged
    if (_status_mutex && xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (boot_time_str) {
            strncpy(_status.reboot_time_str, boot_time_str, sizeof(_status.reboot_time_str) - 1);
            _status.reboot_time_str[sizeof(_status.reboot_time_str) - 1] = '\0';
        }
        if (reason_str) {
            strncpy(_status.reboot_reason, reason_str, sizeof(_status.reboot_reason) - 1);
            _status.reboot_reason[sizeof(_status.reboot_reason) - 1] = '\0';
        }
        xSemaphoreGive(_status_mutex);
    }
}

void web_server_cfg_lock(void) {
    if (_cfg_mutex) {
        // Try a 500 ms timed wait first; if it fails log a deadlock warning and wait forever
        if (xSemaphoreTake(_cfg_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "cfg_mutex timeout -- possible deadlock, retrying with infinite wait");
            xSemaphoreTake(_cfg_mutex, portMAX_DELAY);
        }
    }
}

void web_server_cfg_unlock(void) {
    if (_cfg_mutex) {
        xSemaphoreGive(_cfg_mutex);
    }
}

/**
 * @brief Validate an amateur radio callsign string for use in a WSPR message.
 *
 * WSPR callsign rules (derived from the ITU callsign format and WSPR protocol spec):
 *  - Simple callsigns: 3-6 alphanumeric characters, digit required at position 2
 *    when packed via the standard 28-bit Type-1 scheme (e.g. "W1AW", "LU3VEA").
 *  - Compound callsigns: up to 11 characters with exactly one '/' separator
 *    (e.g. "PJ4/K1ABC", "K1ABC/P"), encoded as Type-2 + companion Type-3.
 *  - Spaces are permitted (some callsigns require a leading space in the 28-bit field).
 *
 * This function performs a permissive check: it validates length (3-11), character
 * set (A-Z, 0-9, space, '/'), and slash count (<= 1).  Deeper structural validation
 * (digit position, prefix/suffix validity) is deferred to wspr_encode().
 *
 * @param[in] s  NUL-terminated callsign string to validate.
 * @return true  — string is syntactically acceptable as a WSPR callsign.
 * @return false — string is too short, too long, contains illegal characters,
 *                 or has more than one slash.
 */
static bool validate_callsign(const char *s) {
    int len = (int)strlen(s);

    // Length must be at least 3 (minimum ITU callsign) and fit in CALLSIGN_LEN
    if (len < 3 || len > CALLSIGN_LEN - 1)
        return false;
    int slash_count = 0;
    for (int i = 0; i < len; i++) {
        char c = (char)toupper((unsigned char)s[i]);

        // Permitted characters: letters, digits, space (for WSPR padding), and one '/'
        if (!isalnum((unsigned char)c) && c != ' ' && c != '/')
            return false;

        if (c == '/')
            slash_count++;
    }

    // More than one slash is invalid (only PREFIX/CALL or CALL/SUFFIX forms are legal)
    if (slash_count > 1)
        return false;

    return true;
}

/**
 * @brief Validate a Maidenhead grid locator string for use in a WSPR message.
 *
 * WSPR supports two locator precisions:
 *  - 4-character locator (e.g. "GF05"): field letters A-R followed by two digits 0-9.
 *    Encoded in 15 bits of the Type-1 message.  Resolution: approx. 1° lat × 2° lon.
 *  - 6-character locator (e.g. "GF05ab"): 4-char base plus a sub-square pair (A-X).
 *    The first 4 chars go into the Type-1 message; the full 6 chars are encoded in a
 *    companion Type-3 message to provide ~2.5 km precision.
 *
 * @param[in] s  NUL-terminated Maidenhead locator string.
 * @return true  — string is a valid 4- or 6-character Maidenhead locator.
 * @return false — string has wrong length or contains characters outside the valid ranges.
 */
static bool validate_locator(const char *s) {
    size_t len = strlen(s);

    // Only 4-char (Type-1) or 6-char (Type-1 + Type-3 companion) locators are accepted
    if (len != 4 && len != 6)
        return false;

    // First two characters: field letters, range A-R (18 fields worldwide)
    char c0 = (char)toupper((unsigned char)s[0]);
    char c1 = (char)toupper((unsigned char)s[1]);
    if (c0 < 'A' || c0 > 'R')
        return false;

    if (c1 < 'A' || c1 > 'R')
        return false;

    // Third and fourth characters: square digits 0-9
    if (!isdigit((unsigned char)s[2]))
        return false;

    if (!isdigit((unsigned char)s[3]))
        return false;

    // Fifth and sixth characters (sub-square): letters A-X (24 sub-squares per square)
    if (len == 6) {
        char c4 = (char)toupper((unsigned char)s[4]);
        char c5 = (char)toupper((unsigned char)s[5]);
        if (c4 < 'A' || c4 > 'X')
            return false;
        if (c5 < 'A' || c5 > 'X')
            return false;
    }
    return true;
}

/**
 * @brief HTTP GET / handler — serves the embedded single-page application HTML.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always.
 */
static esp_err_t h_index(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    httpd_resp_set_type(req, "text/html");
    // Send the compile-time embedded HTML; -1 tells httpd to use strlen (but we pass exact size)
    httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
    return ESP_OK;
}

/**
 * @brief HTTP GET /api/config handler — serialises the live config to JSON.
 *
 * Acquires the config mutex, copies all fields into a cJSON object, releases
 * the mutex, then sends the serialised JSON.  The wifi_pass field is never
 * included; instead a boolean "has_pass" signals whether a password is stored.
 * This prevents credentials from being exposed over HTTP.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always.
 */
static esp_err_t h_get_config(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    web_server_cfg_lock();
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "callsign", _cfg->callsign);
    cJSON_AddStringToObject(j, "locator", _cfg->locator);

    // Use raw number strings for integer fields to avoid cJSON float formatting
    char _pwr_str[8];
    snprintf(_pwr_str, sizeof(_pwr_str), "%u", (unsigned)_cfg->power_dbm);
    cJSON_AddRawToObject(j, "power_dbm", _pwr_str);
    cJSON_AddStringToObject(j, "wifi_ssid", _cfg->wifi_ssid);
    // Expose only whether a password is set — never send the actual password
    cJSON_AddBoolToObject(j, "has_pass", _cfg->wifi_pass[0] != '\0');
    cJSON_AddStringToObject(j, "ntp_server", _cfg->ntp_server);
    cJSON_AddBoolToObject(j, "hop_enabled", _cfg->hop_enabled);

    char _hop_str[16];
    snprintf(_hop_str, sizeof(_hop_str), "%lu", (unsigned long)_cfg->hop_interval_sec);
    cJSON_AddRawToObject(j, "hop_interval_sec", _hop_str);
    cJSON_AddBoolToObject(j, "tx_enabled", _cfg->tx_enabled);

    char _duty_str[8];
    snprintf(_duty_str, sizeof(_duty_str), "%u", (unsigned)_cfg->tx_duty_pct);
    cJSON_AddRawToObject(j, "tx_duty_pct", _duty_str);

    // Crystal calibration offset in ppb (parts per billion); signed integer
    char _ppb_str[16];
    snprintf(_ppb_str, sizeof(_ppb_str), "%ld", (long)_cfg->xtal_cal_ppb);
    cJSON_AddRawToObject(j, "xtal_cal_ppb", _ppb_str);

    char _reg_str[4];
    snprintf(_reg_str, sizeof(_reg_str), "%u", (unsigned)_cfg->iaru_region);
    cJSON_AddRawToObject(j, "iaru_region", _reg_str);

    // Encode band_enabled[] as a JSON boolean array, one entry per wspr_band_t value
    cJSON *bands = cJSON_CreateArray();
    for (int i = 0; i < BAND_COUNT; i++)
        cJSON_AddItemToArray(bands, cJSON_CreateBool(_cfg->band_enabled[i]));
    cJSON_AddItemToObject(j, "band_enabled", bands);
    web_server_cfg_unlock();

    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(j);

    return ESP_OK;
}

/**
 * @brief HTTP POST /api/config handler — parses a JSON body and updates the live config.
 *
 * Execution sequence:
 *  1. Read the full request body (limited to 1023 bytes to protect the stack).
 *  2. Parse the JSON payload with cJSON.
 *  3. Validate callsign and locator server-side (mirrors the JS validation).
 *  4. Acquire the config mutex and update all present fields.
 *  5. Release the mutex and apply side effects outside the critical section:
 *     - oscillator_set_cal() if xtal_cal_ppb changed.
 *     - time_sync_restart_ntp() if ntp_server changed (NTP mode only).
 *  6. Persist the new config to NVS via config_save().
 *  7. Return a JSON {"ok":true} or {"ok":false,"err":"..."} response.
 *
 * The config mutex is held only while fields are being copied, not during NVS
 * writes or oscillator I2C transactions, to minimise lock contention.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK on success (HTTP error encoded in response body).
 * @return ESP_FAIL if the socket receive fails with a non-timeout error.
 */
static esp_err_t h_post_config(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    char buf[1024] = { 0 };
    int total = req->content_len;

    // Reject oversized bodies before reading to prevent stack overflow
    if (total >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "request body too large");
        return ESP_OK;
    }

    // Read the full body in a loop; httpd may deliver it in multiple chunks
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error");
            return ESP_FAIL;
        }
        received += r;
    }

    cJSON *j = cJSON_Parse(buf);
    if (!j) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"err\":\"JSON parse error\"}", -1);
        return ESP_OK;
    }

    // Server-side validation: reject invalid callsign or locator before touching config
    cJSON *v_cs = cJSON_GetObjectItem(j, "callsign");
    cJSON *v_loc = cJSON_GetObjectItem(j, "locator");
    if (v_cs && cJSON_IsString(v_cs) && !validate_callsign(v_cs->valuestring)) {
        cJSON_Delete(j);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"err\":\"Invalid callsign\"}", -1);
        return ESP_OK;
    }
    if (v_loc && cJSON_IsString(v_loc) && !validate_locator(v_loc->valuestring)) {
        cJSON_Delete(j);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"err\":\"Invalid locator (4 or 6-char Maidenhead required)\"}", -1);
        return ESP_OK;
    }

    // Take the config mutex before modifying any shared config fields
    web_server_cfg_lock();

    // Snapshot the old NTP server name so we can detect a change after the mutex is released
    char old_ntp_snap[sizeof(_cfg->ntp_server)];
    strncpy(old_ntp_snap, _cfg->ntp_server, sizeof(old_ntp_snap) - 1);
    old_ntp_snap[sizeof(old_ntp_snap) - 1] = '\0';

    cJSON *v;
    if (v_cs && cJSON_IsString(v_cs)) {
        strncpy(_cfg->callsign, v_cs->valuestring, CALLSIGN_LEN - 1);
        _cfg->callsign[CALLSIGN_LEN - 1] = '\0';
        // Normalise to uppercase — WSPR encoding expects uppercase callsigns
        for (int _i = 0; _cfg->callsign[_i] != '\0'; _i++)
            _cfg->callsign[_i] = (char)toupper((unsigned char)_cfg->callsign[_i]);
    }
    if (v_loc && cJSON_IsString(v_loc)) {
        strncpy(_cfg->locator, v_loc->valuestring, LOCATOR_LEN - 1);
        _cfg->locator[LOCATOR_LEN - 1] = '\0';
        // Normalise locator to uppercase; sub-square characters are case-insensitive in WSPR
        for (int _i = 0; _cfg->locator[_i] != '\0'; _i++)
            _cfg->locator[_i] = (char)toupper((unsigned char)_cfg->locator[_i]);
    }

    // TX power: clamp to legal WSPR range 0-60 dBm; wspr_encode() rounds to nearest valid value
    if ((v = cJSON_GetObjectItem(j, "power_dbm")) && cJSON_IsNumber(v)) {
        int pwr = v->valueint;
        if (pwr < 0)
            pwr = 0;
        if (pwr > 60)
            pwr = 60;
        _cfg->power_dbm = (uint8_t)pwr;
    }

    if ((v = cJSON_GetObjectItem(j, "wifi_ssid")) && cJSON_IsString(v)) {
        strncpy(_cfg->wifi_ssid, v->valuestring, sizeof(_cfg->wifi_ssid) - 1);
        _cfg->wifi_ssid[sizeof(_cfg->wifi_ssid) - 1] = '\0';
    }

    // Password is only updated when the client explicitly sends the field
    if ((v = cJSON_GetObjectItem(j, "wifi_pass")) && cJSON_IsString(v)) {
        strncpy(_cfg->wifi_pass, v->valuestring, sizeof(_cfg->wifi_pass) - 1);
        _cfg->wifi_pass[sizeof(_cfg->wifi_pass) - 1] = '\0';
    }

    if ((v = cJSON_GetObjectItem(j, "ntp_server")) && cJSON_IsString(v)) {
        strncpy(_cfg->ntp_server, v->valuestring, sizeof(_cfg->ntp_server) - 1);
        _cfg->ntp_server[sizeof(_cfg->ntp_server) - 1] = '\0';
    }

    if ((v = cJSON_GetObjectItem(j, "hop_enabled")) && cJSON_IsBool(v))
        _cfg->hop_enabled = cJSON_IsTrue(v);

    // Hop interval: minimum 120 s (one WSPR TX slot = 110.6 s, rounded up)
    if ((v = cJSON_GetObjectItem(j, "hop_interval_sec")) && cJSON_IsNumber(v)) {
        uint32_t interval = (uint32_t)v->valueint;
        if (interval < 120u)
            interval = 120u;
        if (interval > 86400u)
            interval = 86400u;
        _cfg->hop_interval_sec = interval;
    }

    // TX duty cycle: 0=never transmit, 20=WSPR standard (1-in-5), 100=every slot
    if ((v = cJSON_GetObjectItem(j, "tx_duty_pct")) && cJSON_IsNumber(v)) {
        int pct = v->valueint;
        if (pct < 0)
            pct = 0;
        if (pct > 100)
            pct = 100;
        _cfg->tx_duty_pct = (uint8_t)pct;
    }

    // Crystal calibration: signed ppb offset, clamped to ±100000 ppb (±100 ppm)
    // WSPR requires very accurate frequency; the Si5351 PLL applies this offset at runtime
    if ((v = cJSON_GetObjectItem(j, "xtal_cal_ppb")) && cJSON_IsNumber(v)) {
        int32_t ppb = (int32_t)v->valueint;
        if (ppb < -100000)
            ppb = -100000;
        if (ppb > 100000)
            ppb = 100000;
        _cfg->xtal_cal_ppb = ppb;
    }

    // IARU region: 1=EU/AF/ME, 2=Americas, 3=AS/Pacific; affects 60 m dial frequency only
    if ((v = cJSON_GetObjectItem(j, "iaru_region")) && cJSON_IsNumber(v)) {
        int region = v->valueint;
        if (region < 1 || region > 3)
            region = 1;
        _cfg->iaru_region = (uint8_t)region;
    }

    // Replace the entire band_enabled[] array from the JSON array
    cJSON *bands = cJSON_GetObjectItem(j, "band_enabled");
    if (cJSON_IsArray(bands)) {
        memset(_cfg->band_enabled, 0, sizeof(_cfg->band_enabled));
        int n = cJSON_GetArraySize(bands);

        if (n > BAND_COUNT)
            n = BAND_COUNT;

        for (int i = 0; i < n; i++)
            _cfg->band_enabled[i] = cJSON_IsTrue(cJSON_GetArrayItem(bands, i));
    }

    // Signal to scheduler_task that the band list has changed so it can update immediately
    _cfg->bands_changed = true;

    // Take a local snapshot for use outside the critical section
    wspr_config_t cfg_snap = *_cfg;
    int32_t new_cal_ppb = cfg_snap.xtal_cal_ppb;
    web_server_cfg_unlock();

    // Apply the new calibration to the oscillator driver (I2C write — must be outside mutex)
    oscillator_set_cal(new_cal_ppb);

    // Restart the NTP client only if the server hostname actually changed
#ifdef CONFIG_WSPR_TIME_NTP
    if (strncmp(old_ntp_snap, cfg_snap.ntp_server, sizeof(cfg_snap.ntp_server)) != 0) {
        time_sync_restart_ntp(cfg_snap.ntp_server);
    }
#endif

    cJSON_Delete(j);

    // Persist the updated config atomically to NVS flash
    esp_err_t err = config_save(&cfg_snap);
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", err == ESP_OK);
    if (err != ESP_OK)
        cJSON_AddStringToObject(resp, "err", esp_err_to_name(err));
    char *s = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(resp);
    return ESP_OK;
}

/**
 * @brief HTTP POST /api/tx_toggle handler — atomically toggles the TX enable flag.
 *
 * Flips cfg->tx_enabled under the config mutex, saves the new state to NVS,
 * then returns a JSON {"tx_enabled": true/false} response.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always.
 */
static esp_err_t h_tx_toggle(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    wspr_config_t cfg_snap;
    // Atomically flip tx_enabled and capture the new state
    web_server_cfg_lock();
    _cfg->tx_enabled = !_cfg->tx_enabled;
    bool enabled = _cfg->tx_enabled;
    cfg_snap = *_cfg;
    web_server_cfg_unlock();

    // Persist the change immediately so TX state survives a reboot
    config_save(&cfg_snap);

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"tx_enabled\":%s}", enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));

    return ESP_OK;
}

/**
 * @brief HTTP GET /api/status handler — returns a JSON snapshot of runtime state.
 *
 * Takes a consistent copy of _status under _status_mutex (10 ms timeout) and
 * serialises it.  Fields include hardware status, time sync, current band,
 * frequency, next TX countdown, active TX flag, symbol index, and boot info.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always.
 */
static esp_err_t h_status(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    // Copy the status snapshot atomically under the mutex
    wspr_status_t snap = { 0 };
    if (_status_mutex && xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snap = _status;
        xSemaphoreGive(_status_mutex);
    }

    cJSON *j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "hw_ok", snap.hw_ok);
    cJSON_AddStringToObject(j, "hw_name", snap.hw_name[0] ? snap.hw_name : "---");
    cJSON_AddBoolToObject(j, "time_ok", snap.time_ok);
    cJSON_AddStringToObject(j, "time_str", snap.time_str);
    cJSON_AddStringToObject(j, "band", snap.band);
    cJSON_AddStringToObject(j, "freq_str", snap.freq_str);

    // Use raw integer strings for numeric fields to avoid floating-point rendering
    char _ntx_str[16];
    snprintf(_ntx_str, sizeof(_ntx_str), "%ld", (long)snap.next_tx_sec);
    cJSON_AddRawToObject(j, "next_tx_sec", _ntx_str);
    cJSON_AddBoolToObject(j, "tx_active", snap.tx_active);
    cJSON_AddBoolToObject(j, "tx_enabled", snap.tx_enabled);

    char _sym_str[8];
    snprintf(_sym_str, sizeof(_sym_str), "%d", snap.symbol_idx);
    cJSON_AddRawToObject(j, "symbol_idx", _sym_str);
    cJSON_AddStringToObject(j, "boot_time_str", snap.reboot_time_str);
    cJSON_AddStringToObject(j, "reboot_reason", snap.reboot_reason);
    // Inform the client if settings were reset to factory defaults on this boot
    cJSON_AddBoolToObject(j, "settings_reset", config_was_reset());

    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

/**
 * @brief HTTP GET /api/wifi_scan handler — triggers a blocking WiFi scan.
 *
 * Delegates to wifi_manager_scan() which performs the scan (1-2 s), builds a
 * heap-allocated JSON array, and returns it.  The handler sends the array and
 * frees the buffer.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK always.
 */
static esp_err_t h_wifi_scan(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    char *json = wifi_manager_scan();
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scan alloc failed");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, (ssize_t)strlen(json));
    // wifi_manager_scan() allocates on heap; caller is responsible for freeing
    free(json);

    return ESP_OK;
}

/**
 * @brief HTTP POST /api/reset handler — sends a response then reboots the ESP32.
 *
 * A 200 ms delay is inserted between the HTTP response and esp_restart() to
 * allow the TCP stack to flush the response to the client before the network
 * interface is torn down.
 *
 * @param[in] req  Incoming httpd request handle.
 * @return ESP_OK (never actually reached after esp_restart()).
 */
static esp_err_t h_reset(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    // Send success response before restarting so the client knows the request was received
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", -1);
    // Brief delay to allow TCP to deliver the response before the stack is destroyed
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();

    return ESP_OK;
}

esp_err_t web_server_start(wspr_config_t *cfg) {
    _cfg = cfg;

    // Create the config mutex first; scheduler and status tasks must not start before this
    if (_cfg_mutex == NULL) {
        _cfg_mutex = xSemaphoreCreateMutex();
        if (_cfg_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create cfg mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    // Create the status mutex used to protect _status between h_status() and update callers
    if (_status_mutex == NULL) {
        _status_mutex = xSemaphoreCreateMutex();
        if (_status_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create status mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    // Configure the httpd server: allow 10 URI handlers, increase stack for JSON processing
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.max_uri_handlers = 10;
    hcfg.stack_size = 8192;

    if (httpd_start(&_srv, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    // Register all REST endpoints in the route table
    static const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = h_index },
        { .uri = "/api/config", .method = HTTP_GET, .handler = h_get_config },
        { .uri = "/api/config", .method = HTTP_POST, .handler = h_post_config },
        { .uri = "/api/tx_toggle", .method = HTTP_POST, .handler = h_tx_toggle },
        { .uri = "/api/status", .method = HTTP_GET, .handler = h_status },
        { .uri = "/api/wifi_scan", .method = HTTP_GET, .handler = h_wifi_scan },
        { .uri = "/api/reset", .method = HTTP_POST, .handler = h_reset },
    };

    for (int i = 0; i < 7; i++)
        httpd_register_uri_handler(_srv, &routes[i]);

    ESP_LOGI(TAG, "HTTP server started");

    return ESP_OK;
}

esp_err_t web_server_stop(void) {
    if (_srv) {
        httpd_stop(_srv);
        _srv = NULL;
    }

    return ESP_OK;
}
