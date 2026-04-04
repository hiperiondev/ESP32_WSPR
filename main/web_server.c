/**
 * @file web_server.c
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez (lu3vea@gmail.com)
 * @brief Embedded HTTP configuration and status server.
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#include <ctype.h>
#include <math.h>
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

static bool check_auth(httpd_req_t *req) {
    char auth_hdr[160] = { 0 };
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_hdr, sizeof(auth_hdr)) != ESP_OK)
        return false;
    if (strncmp(auth_hdr, "Basic ", 6) != 0)
        return false;
    unsigned char decoded[128] = { 0 };
    size_t decoded_len = 0;
    const unsigned char *b64 = (const unsigned char *)(auth_hdr + 6);
    int rc = mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &decoded_len, b64, strlen(auth_hdr + 6));
    if (rc != 0)
        return false;
    decoded[decoded_len] = '\0';
    char expected[160];
    snprintf(expected, sizeof(expected), "%s:%s", CONFIG_WSPR_HTTP_AUTH_USER, CONFIG_WSPR_HTTP_AUTH_PASS);
    return strcmp((char *)decoded, expected) == 0;
}

static esp_err_t send_auth_challenge(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
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
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] =
    "<!DOCTYPE html><html lang='" WEBUI_HTML_LANG "'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WSPR Transmitter</title>"
    "<style>"
    ":root{--bg:#f5f4f0;--card:#ffffff;--border:#d0cfc9;--accent:#1a56db;"
    "--green:#1a7f37;--red:#cf222e;--text:#1c1c1c;--sub:#57534e}"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;"
    "min-height:100vh;padding:20px}"
    "h1{text-align:center;color:var(--accent);margin-bottom:6px;font-size:1.6em}"
    ".reboot-info{text-align:center;color:var(--sub);font-size:.78em;margin-bottom:16px;"
    "min-height:1.2em;letter-spacing:.01em}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;max-width:900px;margin:0 auto}"
    ".card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:20px}"
    ".card h2{color:var(--accent);font-size:1em;margin-bottom:14px;display:flex;align-items:center;gap:8px}"
    "label{display:block;color:var(--sub);font-size:.8em;margin-bottom:4px;margin-top:12px}"
    "label:first-child{margin-top:0}"
    "input[type=text],input[type=number],input[type=password],select{"
    "width:100%;background:var(--bg);border:1px solid var(--border);color:var(--text);"
    "padding:8px 10px;border-radius:6px;font-size:.9em;outline:none;transition:.2s}"
    "input:focus,select:focus{border-color:var(--accent)}"
    ".bands{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:4px}"
    ".band-cb{display:flex;align-items:center;gap:6px;background:var(--bg);"
    "border:1px solid var(--border);border-radius:6px;padding:6px 8px;cursor:pointer;"
    "transition:.2s;font-size:.85em}"
    ".band-cb:hover{border-color:var(--accent)}"
    ".band-cb input{width:16px;height:16px;cursor:pointer;accent-color:var(--accent)}"
    // override the generic label element rule so all band-cb boxes have
    // the same margin (zero) and text color regardless of DOM position (first-child
    // or not). Without this, label{margin-top:12px} applied to non-first band items
    // while label:first-child set margin-top:0 only for 2200m, making it appear
    // shorter than the others inside the CSS grid rows.
    "label.band-cb{margin:0;color:var(--text)}"
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
    ".btn-tx{padding:10px 20px;font-size:.9em;font-weight:600;border:none;"
    "border-radius:6px;cursor:pointer;transition:.2s}"
    ".btn-tx.on{background:var(--red);color:#fff}"
    ".btn-tx.off{background:var(--green);color:#000}"
    // tone button light red (#fca5a5) when active, grey (#d1d5db) when idle
    ".btn-tone{padding:10px 16px;font-size:.88em;font-weight:600;border:none;"
    "border-radius:6px;cursor:pointer;transition:.2s;white-space:nowrap}"
    ".btn-tone.on{background:#fca5a5;color:#7f1d1d}"
    ".btn-tone.off{background:#d1d5db;color:#374151}"
    // tone-ctrl-row places button and kHz input side-by-side in one row
    ".tone-ctrl-row{display:flex;gap:8px;align-items:center;margin-top:10px}"
    ".tone-ctrl-row input{width:100px;flex-shrink:0}"
    // xtal-cal-row: ppb input + measured kHz input + Auto button in one line
    ".xtal-cal-row{display:flex;gap:6px;align-items:center;margin-top:2px}"
    ".xtal-cal-row input{flex-shrink:0}"
    ".btn-auto{flex-shrink:0;padding:6px 12px;font-size:.82em;font-weight:700;"
    "border-radius:6px;border:1px solid var(--accent);background:#eef2ff;"
    "color:var(--accent);cursor:pointer;white-space:nowrap;transition:.2s}"
    ".btn-auto:hover{background:#dde6ff}"
    ".status-box{background:var(--bg);border:1px solid var(--border);border-radius:8px;"
    "padding:14px;margin-top:14px}"
    ".status-row{display:flex;justify-content:space-between;padding:4px 0;"
    "border-bottom:1px solid var(--border);font-size:.85em}"
    ".status-row:last-child{border:none}"
    ".status-row span:first-child{color:var(--sub)}"
    ".badge{padding:2px 8px;border-radius:10px;font-size:.75em;font-weight:700}"
    ".badge.ok{background:#d1fae5;color:var(--green)}"
    ".badge.warn{background:#fef3c7;color:#92400e}"
    ".badge.err{background:#fee2e2;color:var(--red)}"
    ".toast{position:fixed;top:20px;right:20px;background:var(--card);"
    "border:1px solid var(--border);border-radius:8px;padding:12px 20px;"
    "display:none;animation:fadein .3s}"
    "@keyframes fadein{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:none}}"
    ".hop-row{display:flex;align-items:center;gap:10px;margin-top:10px}"
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
    ".btn-reset{background:#fff0f0;color:var(--red);border:1px solid var(--red);"
    "padding:5px 16px;border-radius:6px;font-size:.82em;font-weight:700;"
    "cursor:pointer;transition:.2s;display:block;margin:0 auto 20px}"
    ".btn-reset:hover{background:var(--red);color:#fff}"
    // Locator row — flex container holds the input and the GPS button side by side.
    // The input grows to fill available space; the button is fixed-width.
    // The combined width matches the callsign input above it (full card width).
    ".loc-row{display:flex;gap:6px;align-items:stretch;width:100%}"
    ".loc-row input{flex:1;min-width:0}"
    // GPS button inside the locator row.
    // Enabled = accent border+text; disabled = muted appearance, no cursor.
    ".btn-gps{flex-shrink:0;padding:0 12px;height:38px;font-size:.82em;font-weight:700;"
    "border-radius:6px;border:1px solid var(--border);background:var(--bg);"
    "color:var(--sub);cursor:default;transition:.2s;white-space:nowrap}"
    ".btn-gps.gps-ok{border-color:var(--accent);color:var(--accent);background:#eef2ff;cursor:pointer}"
    ".btn-gps.gps-ok:hover{background:#dde6ff;border-color:var(--accent)}"
    ".btn-gps:disabled{opacity:.45;cursor:default}"
    "</style></head><body>"
    "<h1>&#128225; WSPR Transmitter</h1>"
    "<p style='text-align:center;color:#92400e;font-size:.78em;margin-bottom:4px'> FW Ver: " FW_VERSION_STRING "</p>" HTTP_AUTH_WARNING
    "<div id='reboot_info' class='reboot-info'></div>"
    "<button class='btn-reset' onclick='resetESP()'>&#9211; Reset ESP32</button>"
    "<div class='grid'>"

    // ── Station card ──────────────────────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#128752; " WEBUI_CARD_STATION_TITLE "</h2>"
    "<label>" WEBUI_LABEL_CALLSIGN "</label>"
    "<input id='callsign' type='text' maxlength='11' placeholder='LU1ABC'>"
    "<label>" WEBUI_LABEL_LOCATOR "</label>"
    // locator row — input + GPS button side-by-side.
    // The outer .loc-row div makes both elements the same height and fills the card width.
    // The input has flex:1 so it takes all remaining space after the fixed-width button.
    // This mirrors the callsign input width so the card looks symmetric.
    "<div class='loc-row'>"
    "<input id='locator' type='text' maxlength='6' placeholder='GF05'>"
    // GPS button — disabled by default; JS enables it when /api/status
    // reports gps_active=true (i.e. GPS module was auto-detected at boot).
    "<button id='btn_gps' class='btn-gps' disabled onclick='fillFromGPS()'>" WEBUI_BTN_FROM_GPS "</button>"
    "</div>"
    "<label>" WEBUI_LABEL_POWER "</label>"
    "<input id='power' type='number' min='0' max='60' step='3'>"
    "<label>" WEBUI_LABEL_XTAL_CAL "</label>"
    // xtal-cal-row — ppb input (narrow) + measured kHz input + Auto button
    // [MODIFIED] Corrected comment: * 1e9 (ppb = parts per BILLION, not ppm).
    // Auto button computes ppb = (measured_khz - nominal_khz)/nominal_khz*1e9
    // positive ppb = crystal fast (output too high) -> firmware lowers VCO to compensate
    "<div class='xtal-cal-row'>"
    "<input id='xtal_cal' type='number' min='-200000' max='200000' step='100' style='width:110px'>"
    "<input id='meas_freq' type='number' min='0.001' max='30000' step='0.001' "
    "placeholder='kHz' style='width:100px' title='Measured tone frequency in kHz'>"
    "<button class='btn-auto' onclick='autoCalibrate()' title='Auto-compute ppb from measured vs nominal tone frequency'>"
    "Auto</button>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:4px'>" WEBUI_HINT_XTAL_CAL "</p>"
    "<button class='btn-save' onclick='save()'>" WEBUI_BTN_SAVE "</button>"
    "</div>"

    // ── WiFi card ─────────────────────────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#128246; " WEBUI_CARD_WIFI_TITLE "</h2>"
    "<label>SSID</label>"
    "<input id='wifi_ssid' type='text' maxlength='32' placeholder='Mi_Red'>"
    "<button class='btn-scan' id='scan_btn' onclick='scanWifi()'>" WEBUI_BTN_SCAN_LABEL "</button>"
    "<div class='scan-list' id='scan_list'></div>"
    "<label>" WEBUI_LABEL_PASSWORD "</label>"
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
    "<div class='card'>"
    "<h2>&#128251; " WEBUI_CARD_BANDS_TITLE "</h2>"
    "<div class='bands' id='bands'></div>"
    "</div>"

    // ── IARU region card ──────────────────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#127758; " WEBUI_CARD_IARU_TITLE "</h2>"
    "<label>" WEBUI_LABEL_IARU_REGION "</label>"
    "<select id='iaru_region' onchange='sendLiveUpdate()'>"
    "<option value='1'>" WEBUI_IARU_REGION_1 "</option>"
    "<option value='2'>" WEBUI_IARU_REGION_2 "</option>"
    "<option value='3'>" WEBUI_IARU_REGION_3 "</option>"
    "</select>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_IARU "</p>"
    "</div>"

    // ── Frequency hopping card ────────────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#128260; " WEBUI_CARD_HOP_TITLE "</h2>"
    "<div class='toggle'>"
    "<label class='switch'><input type='checkbox' id='hop_en' onchange='sendLiveUpdate()'><span class='slider'></span></label>"
    "<span>" WEBUI_TOGGLE_HOP_LABEL "</span></div>"
    "<div class='hop-row'>"
    "<label style='margin:0;white-space:nowrap'>" WEBUI_LABEL_HOP_INTERVAL "</label>"
    "<input id='hop_interval' type='number' min='120' max='86400' step='120' style='width:100px' onchange='sendLiveUpdate()'>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_HOP "</p>"
    "</div>"

    // ── TX duty cycle card ────────────────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#128202; " WEBUI_CARD_DUTY_TITLE "</h2>"
    "<label>" WEBUI_LABEL_DUTY "</label>"
    "<div class='row'>"
    "<input id='tx_duty_pct' type='number' min='0' max='100' step='5' style='width:80px' onchange='sendLiveUpdate()'>"
    "<span style='color:var(--sub);font-size:.85em'>" WEBUI_HINT_DUTY_INLINE "</span>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_DUTY "</p>"
    "</div>"

    // ── TX control / live status card ─────────────────────────────────────────
    "<div class='card'>"
    "<h2>&#9889; " WEBUI_CARD_TX_TITLE "</h2>"
    // TX enable button on its own row
    "<div class='toggle'>"
    "<button class='btn-tx off' id='tx_btn' onclick='toggleTx()'>" WEBUI_JS_BTN_TX_START "</button>"
    "</div>"
    // tone button + kHz input inline in one row (tone-ctrl-row)
    "<div class='tone-ctrl-row'>"
    "<button class='btn-tone off' id='tone_btn' onclick='toggleTone()'>" WEBUI_JS_BTN_TONE_START "</button>"
    "<input id='tone_freq' type='number' min='0.1' max='30000' step='0.1' "
    "placeholder='kHz' value='7040.1'>"
    "<span style='color:var(--sub);font-size:.82em'>" WEBUI_JS_TONE_FREQ_HINT "</span>"
    "</div>"
    "<div class='status-box'>"
    "<div class='status-row'><span>" WEBUI_STATUS_HW_LABEL "</span><span id='s_hw'><span class='badge warn'>---</span></span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_TIME_LABEL "</span><span id='s_time'><span class='badge warn'>---</span></span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_BAND_LABEL "</span><span id='s_band'>---</span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_FREQ_LABEL "</span><span id='s_freq'>---</span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_NEXT_LABEL "</span><span id='s_next'>---</span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_TX_LABEL "</span><span id='s_tx'><span class='badge err'>OFF</span></span></div>"
    "<div class='status-row'><span>" WEBUI_STATUS_SYM_LABEL "</span><span id='s_sym'>---</span></div>"
    "</div>"
    "<div style='margin-top:12px;text-align:center'>"
    // default href points to PSKReporter map (updated by loadCfg() with full URL)
    "<a id='wspr_link' href='https://pskreporter.info/pskmap.html' target='_blank' rel='noopener'"
    " style='color:var(--accent);font-size:.85em;text-decoration:none'>" WEBUI_WSPR_LINK_TEXT "</a></div>"
    "</div>"

    "</div>"
    "<div class='toast' id='toast'></div>"

    // ── Embedded JavaScript ───────────────────────────────────────────────────
    "<script>"
    "const BANDS=['2200m','630m','160m','80m','60m','40m','30m','20m','17m','15m','12m','10m'];"
    "let cfg={};"
    "let _hasPass=false;"
    "let _passEdited=false;"
    "let _rebootInfoSet=false;"
    "let _settingsResetShown=false;"
    // Compile-time string constants for the tone button labels
    "const TONE_START='" WEBUI_JS_BTN_TONE_START "';"
    "const TONE_STOP='" WEBUI_JS_BTN_TONE_STOP "';"
    // _gpsActive tracks whether a GPS module was detected at boot.
    // Initialised to false; set to true when /api/status returns gps_active=true.
    // Controls the enabled/disabled state and visual style of the GPS button.
    "let _gpsActive=false;"
    // _toneFreqKhz stores the last known nominal tone frequency in kHz.
    // Updated on every status poll from s.tone_freq_khz.
    // Used by autoCalibrate() as the nominal reference frequency.
    "let _toneFreqKhz=0;"

    // convert a 4- or 6-character Maidenhead locator to the approximate
    // center lat/lon (decimal degrees) used to set the PSKReporter map center.
    // Algorithm follows the ITU/IARU Maidenhead spec:
    //   field  : lon_adj/20, lat_adj/10     (letters A-R, 18 fields each)
    //   square : digit0..1                  (digits 0-9 within the field)
    //   subsq  : (lon%2)/2*24, (lat%1)*24   (letters A-X, 24 subsquares)
    // The center of the selected square/subsquare is returned (+1 lon, +0.5 lat
    // for 4-char; +1/24 lon, +0.5/24 lat for 6-char).
    "function maidenToLatLon(loc){"
    "if(!loc||loc.length<4)return{lat:'0.00',lon:'0.00'};"
    "const u=loc.toUpperCase();"
    "let lon=(u.charCodeAt(0)-65)*20-180;"
    "let lat=(u.charCodeAt(1)-65)*10-90;"
    "lon+=parseInt(u[2],10)*2;"
    "lat+=parseInt(u[3],10);"
    "if(loc.length>=6){"
    // subsquare: each letter A-X divides 2-degree lon / 1-degree lat into 24 parts
    "lon+=(u.charCodeAt(4)-65)*(2.0/24.0)+(1.0/24.0);"
    "lat+=(u.charCodeAt(5)-65)*(1.0/24.0)+(0.5/24.0);"
    "}else{"
    // center of the 2-degree x 1-degree grid square
    "lon+=1.0;lat+=0.5;"
    "}"
    "return{lat:lat.toFixed(2),lon:lon.toFixed(2)};}"

    "async function sendLiveUpdate(){"
    "const bands=Array.from(document.querySelectorAll('#bands input')).map(i=>i.checked);"
    "const body={"
    "band_enabled:bands,"
    "iaru_region:parseInt(document.getElementById('iaru_region').value)||1,"
    "tx_duty_pct:parseInt(document.getElementById('tx_duty_pct').value)||20,"
    "hop_enabled:document.getElementById('hop_en').checked,"
    "hop_interval_sec:parseInt(document.getElementById('hop_interval').value)||120"
    "};"
    "try{await fetch('/api/live_update',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify(body)});}catch(e){}}"

    "function buildBands(enabled){"
    "const c=document.getElementById('bands');c.innerHTML='';"
    "BANDS.forEach((b,i)=>{"
    "const d=document.createElement('label');"
    "d.className='band-cb';"
    "d.innerHTML=`<input type='checkbox' value='${i}' ${enabled[i]?'checked':''}>${b}`;"
    "d.querySelector('input').addEventListener('change',sendLiveUpdate);"
    "c.appendChild(d);});}"

    "async function loadCfg(){"
    "try{"
    "const r=await fetch('/api/config');cfg=await r.json();"
    "document.getElementById('callsign').value=cfg.callsign||'';"
    "document.getElementById('locator').value=cfg.locator||'';"
    "document.getElementById('power').value=cfg.power_dbm||23;"
    "document.getElementById('xtal_cal').value=cfg.xtal_cal_ppb!=null?cfg.xtal_cal_ppb:0;"
    "document.getElementById('wifi_ssid').value=cfg.wifi_ssid||'';"
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
    "const cs=(cfg.callsign||'').trim();"
    // build PSKReporter map URL with callsign + lat/lon derived from
    // the Maidenhead locator. Format:
    //   https://pskreporter.info/pskmap.html#preset&callsign=CS&txrx=tx&mode=WSPR&mapCenter=LAT,LON
    "const loc=(cfg.locator||'').trim();"
    "if(cs){"
    "const ll=maidenToLatLon(loc);"
    "document.getElementById('wspr_link').href='https://pskreporter.info/"
    "pskmap.html#preset&callsign='+encodeURIComponent(cs)+'&txrx=tx&mode=WSPR&mapCenter='+ll.lat+','+ll.lon;"
    "}"
    "}catch(e){toast('" WEBUI_JS_ERR_LOAD "'+e,'err');}}"

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
    "if(_passEdited){"
    "body.wifi_pass=document.getElementById('wifi_pass').value;"
    "}"
    "if(!body.callsign||body.callsign.length<3||body.callsign.length>11||"
    "!/^[A-Z0-9 /]{3,11}$/.test(body.callsign)||"
    "(body.callsign.match(/\\//g)||[]).length>1){"
    "toast('\\u274c " WEBUI_JS_ERR_CALLSIGN "','err');return;}"
    "if(!body.locator||!/^[A-Ra-r]{2}[0-9]{2}([A-Xa-x]{2})?$/.test(body.locator)){"
    "toast('\\u274c " WEBUI_JS_ERR_LOCATOR "','err');return;}"
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
    "if(j.ok){_passEdited=false;loadCfg();}"
    "}catch(e){toast('Error: '+e,'err');}}"

    "async function toggleTx(){"
    "try{"
    "const r=await fetch('/api/tx_toggle',{method:'POST'});"
    "const j=await r.json();"
    "updateTxBtn(j.tx_enabled);"
    "}catch(e){toast('" WEBUI_JS_ERR_TX "'+e,'err');}}"

    "function updateTxBtn(on){"
    "const b=document.getElementById('tx_btn');"
    "b.textContent=on?'" WEBUI_JS_BTN_TX_STOP "':'" WEBUI_JS_BTN_TX_START "';"
    "b.className='btn-tx '+(on?'on':'off');}"

    // updateToneBtn — syncs tone button visual state from poll or toggle response.
    // active=true: light red background, stop label. active=false: grey, start label.
    // freq parameter (optional) updates the kHz input when provided and non-zero.
    "function updateToneBtn(active,freq){"
    "const b=document.getElementById('tone_btn');"
    "b.textContent=active?TONE_STOP:TONE_START;"
    "b.className='btn-tone '+(active?'on':'off');"
    "if(active&&freq&&freq>0){"
    "document.getElementById('tone_freq').value=parseFloat(freq.toFixed(3));}}"

    // autoCalibrate — computes ppb, saves it, then restarts the tone so the
    // new calibration takes effect immediately without manual toggle off/on.
    //
    // Why restart is needed:
    //   oscillator_set_cal() defers the update while _osc_tx_active is true (tone running).
    //   Saving via /api/config calls oscillator_set_cal() but it is queued, not applied.
    //   Stopping the tone calls oscillator_tx_end() which flushes the deferred cal.
    //   Restarting the tone calls oscillator_set_freq() with the corrected VCO/FTW.
    //   Result: corrected frequency is heard without any manual action by the user.
    //
    // [MODIFIED] Corrected comment: * 1e9 (ppb = parts per BILLION, not ppm).
    // Formula: ppb = round((measured_khz - nominal_khz) / nominal_khz * 1e9)
    // positive ppb -> crystal fast (output too high) -> firmware lowers VCO/FTW
    "async function autoCalibrate(){"
    "const measInput=document.getElementById('meas_freq');"
    "const calInput=document.getElementById('xtal_cal');"
    "const meas=parseFloat(measInput.value);"
    // validate measured frequency input
    "if(isNaN(meas)||meas<=0){"
    "toast('\\u274c Enter measured frequency in kHz first','err');return;}"
    "const nom=_toneFreqKhz;"
    // require tone test to be running so nominal frequency is known
    "if(nom<=0){"
    "toast('\\u274c Start Tone Test first so the nominal frequency is known','warn');return;}"
    // [MODIFIED] Corrected comment: * 1e9, not * 1e6. ppb = parts per BILLION.
    // ppb = (measured - nominal) / nominal * 1e9  (kHz units cancel)
    // positive ppb = crystal fast (output above nominal) -> firmware lowers VCO
    // negative ppb = crystal slow (output below nominal) -> firmware raises VCO
    // firmware convention (oscillator.c si_cache_band): vco_cal = vco - corr when ppb>0
    "const ppb=Math.round((meas-nom)/nom*1e9);"
    // clamp to firmware limits +-100000 ppb
    "const clamped=Math.max(-200000,Math.min(200000,ppb));"
    "calInput.value=clamped;"
    // check whether tone is currently active
    "const toneOn=document.getElementById('tone_btn').classList.contains('on');"
    "try{"
    // save config with new ppb — calls oscillator_set_cal() server-side.
    // if tone is running the cal is deferred inside firmware until tone stops.
    "await fetch('/api/config',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({xtal_cal_ppb:clamped})});"
    // if tone is active: stop it so oscillator_tx_end() flushes the deferred cal,
    // then immediately restart at the same frequency — now uses corrected calibration.
    "if(toneOn){"
    "await fetch('/api/tone_toggle',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({active:false,freq_khz:nom})});"
    // 350 ms pause (was 120 ms): the scheduler tone loop runs every
    // 200 ms; 120 ms was not enough to guarantee the scheduler processed the stop
    // before seeing the restart, leaving s_tone_was_active=true and tone_hz
    // unchanged so oscillator_set_freq() was skipped and the cache was not rebuilt
    // with the new calibration. 350 ms ensures the 200 ms loop has completed.
    "await new Promise(r=>setTimeout(r,350));"
    "const r2=await fetch('/api/tone_toggle',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({active:true,freq_khz:nom})});"
    "const j2=await r2.json();"
    "updateToneBtn(!!j2.tone_active,j2.freq_khz);"
    "toast('\\u2705 Cal '+clamped+' ppb applied \\u2014 tone restarted at '+nom.toFixed(3)+' kHz','ok');"
    "}else{"
    "toast('\\u2705 XTAL cal set to '+clamped+' ppb \\u2014 start Tone Test to verify','ok');}"
    "}catch(e){toast('\\u274c autoCalibrate error: '+e,'err');}}"

    // toggleTone — sends POST /api/tone_toggle with freq_khz and active flag.
    // Reads the kHz input, validates it, then toggles the active state derived from
    // the current button class (.on = currently active -> will deactivate, else activate).
    "async function toggleTone(){"
    "const btn=document.getElementById('tone_btn');"
    "const freqInput=document.getElementById('tone_freq');"
    "const freqVal=parseFloat(freqInput.value);"
    "if(isNaN(freqVal)||freqVal<0.1||freqVal>30000){"
    "toast('\\u274c " WEBUI_JS_TONE_FREQ_ERR "','err');return;}"
    // currently active means button has class 'on' -> clicking will stop tone
    "const currentlyActive=btn.classList.contains('on');"
    "const newActive=!currentlyActive;"
    "try{"
    "const r=await fetch('/api/tone_toggle',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({active:newActive,freq_khz:freqVal})});"
    "const j=await r.json();"
    "updateToneBtn(!!j.tone_active,j.freq_khz);"
    "}catch(e){toast('Error: '+e,'err');}}"

    // updateGpsBtn — called from pollStatus() to enable or disable the GPS button.
    // When gpsOk is true the button gets the .gps-ok class (accent color, pointer cursor).
    // When false the button is left disabled with neutral styling.
    "function updateGpsBtn(gpsOk){"
    "const b=document.getElementById('btn_gps');"
    "if(gpsOk&&!_gpsActive){"
    "_gpsActive=true;"
    "b.disabled=false;"
    "b.classList.add('gps-ok');"
    "}else if(!gpsOk&&_gpsActive){"
    "_gpsActive=false;"
    "b.disabled=true;"
    "b.classList.remove('gps-ok');"
    "}}"

    // fillFromGPS — called when user clicks the GPS button.
    // Fetches /api/gps_loc; if valid, converts lat/lon to a 6-char Maidenhead locator
    // entirely in JavaScript (no server-side conversion needed beyond providing the coords).
    // The conversion algorithm follows the ITU/IARU Maidenhead specification:
    //   lon_adj = lon + 180  (shift to 0-360)
    //   lat_adj = lat + 90   (shift to 0-180)
    //   Field:    lon_adj/20,       lat_adj/10         (letters A-R, 18 fields)
    //   Square:   (lon_adj%20)/2,   (lat_adj%10)/1     (digits 0-9)
    //   Subsq:    (lon_adj%2)*12,   (lat_adj%1)*24     (letters A-X, 24 subsquares)
    "async function fillFromGPS(){"
    "if(!_gpsActive)return;"
    "try{"
    "const r=await fetch('/api/gps_loc');"
    "if(!r.ok)throw new Error('HTTP '+r.status);"
    "const g=await r.json();"
    "if(!g.valid){"
    "toast('" WEBUI_JS_GPS_LOC_ERR "','warn');return;"
    "}"
    "const lat=g.lat;"
    "const lon=g.lon;"
    // Maidenhead 6-char computation in JS (integer arithmetic only)
    "const lonAdj=lon+180.0;"
    "const latAdj=lat+90.0;"
    "const f0=String.fromCharCode(65+Math.floor(lonAdj/20));"
    "const f1=String.fromCharCode(65+Math.floor(latAdj/10));"
    "const s0=String(Math.floor((lonAdj%20)/2));"
    "const s1=String(Math.floor(latAdj%10));"
    "const q0=String.fromCharCode(97+Math.floor(((lonAdj%2)*12)));"
    "const q1=String.fromCharCode(97+Math.floor(((latAdj%1)*24)));"
    "const loc6=f0+f1+s0+s1+q0+q1;"
    "document.getElementById('locator').value=loc6.toUpperCase();"
    "}catch(e){"
    "toast('" WEBUI_JS_GPS_LOC_ERR "','warn');"
    "}}"

    "async function pollStatus(){"
    "try{"
    "const r=await fetch('/api/status');const s=await r.json();"
    "document.getElementById('s_hw').innerHTML=s.hw_ok?"
    "`<span class='badge ok'>${s.hw_name}</span>`:"
    "`<span class='badge err'>${s.hw_name} \\u26a0</span>`;"
    "document.getElementById('s_time').innerHTML=s.time_ok?"
    "`<span class='badge ok'>${s.time_str}</span>`:"
    "`<span class='badge warn'>" WEBUI_JS_NOT_SYNCED "</span>`;"
    "document.getElementById('s_band').textContent=s.band||'---';"
    "document.getElementById('s_freq').textContent=s.freq_str||'---';"
    "document.getElementById('s_next').textContent=s.next_tx_sec>0?s.next_tx_sec+'s':'" WEBUI_JS_TRANSMITTING "';"
    "document.getElementById('s_tx').innerHTML=s.tx_active?"
    "`<span class='badge ok'>ON</span>`:`<span class='badge err'>OFF</span>`;"
    "document.getElementById('s_sym').textContent=s.tx_active?(s.symbol_idx+'/162'):'---';"
    "updateTxBtn(s.tx_enabled);"
    // Sync tone button state from status response
    // also store the nominal tone frequency for autoCalibrate()
    "updateToneBtn(!!s.tone_active,s.tone_freq_khz);"
    "if(s.tone_freq_khz&&s.tone_freq_khz>0)_toneFreqKhz=s.tone_freq_khz;"
    // update GPS button state from status poll.
    // gps_active is true when the time source is GPS (auto-detected at boot).
    "updateGpsBtn(!!s.gps_active);"
    "if(!_rebootInfoSet&&(s.boot_time_str||s.reboot_reason)){"
    "let ri='';"
    "if(s.boot_time_str)ri+='" WEBUI_REBOOT_INFO_PREFIX "'+s.boot_time_str;"
    "if(s.reboot_reason)ri+='" WEBUI_REBOOT_CAUSE_PREFIX "'+s.reboot_reason;"
    "document.getElementById('reboot_info').textContent=ri;"
    "_rebootInfoSet=true;}"
    "if(!_settingsResetShown&&s.settings_reset){"
    "toast('\\u26a0 Settings were reset to factory defaults!','warn');"
    "_settingsResetShown=true;}"
    "}catch(e){}}"

    "function toast(msg,type){"
    "const t=document.getElementById('toast');t.textContent=msg;"
    "t.style.display='block';t.style.borderColor=type==='ok'?'var(--green)':type==='warn'?'#92400e':'var(--red)';"
    "setTimeout(()=>t.style.display='none',3000);}"

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
    "aps.sort((a,b)=>b.rssi-a.rssi);"
    "aps.forEach(ap=>{"
    "const row=document.createElement('div');"
    "row.className='scan-item';"
    "const rssiColor=ap.rssi>=-60?'var(--green)':ap.rssi>=-75?'#e3b341':'var(--red)';"
    "const lock=ap.auth?'&#128274;':'';"
    "row.innerHTML=`<span class='scan-ssid'>${ap.ssid||'" WEBUI_JS_HIDDEN "'}</span>`"
    "+`<span class='scan-rssi' style='color:${rssiColor}'>${ap.rssi} dBm</span>`"
    "+`<span class='scan-lock'>${lock}</span>`;"
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

    "async function resetESP(){"
    "if(!confirm('" WEBUI_JS_CONFIRM_RESET "'))return;"
    "try{await fetch('/api/reset',{method:'POST'});}catch(e){}"
    "toast('" WEBUI_JS_RESTARTING "','warn');"
    "setTimeout(()=>location.reload(),4000);}"

    "loadCfg();setInterval(pollStatus,2000);"
    "</script></body></html>";

// ─────────────────────────────────────────────────────────────────────────────
// Internal status snapshot structure
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    bool time_ok;
    char time_str[24];
    char band[8];
    char freq_str[20];
    int32_t next_tx_sec;
    bool tx_active;
    bool tx_enabled;
    int symbol_idx;
    bool hw_ok;
    char hw_name[32];
    char reboot_time_str[32];
    char reboot_reason[32];
} wspr_status_t;

static wspr_status_t _status = { 0 };

void web_server_update_status(bool time_ok, const char *time_str, const char *band, const char *freq_str, int32_t next_tx_sec, bool tx_active, bool tx_enabled,
                              int symbol_idx) {
    if (_status_mutex && xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _status.time_ok = time_ok;
        _status.tx_active = tx_active;
        _status.tx_enabled = tx_enabled;
        _status.next_tx_sec = next_tx_sec;
        _status.symbol_idx = symbol_idx;
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
    if (_status_mutex == NULL) {
        _status.hw_ok = hw_ok;
        if (hw_name) {
            strncpy(_status.hw_name, hw_name, sizeof(_status.hw_name) - 1);
            _status.hw_name[sizeof(_status.hw_name) - 1] = '\0';
        }
        return;
    }
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

static bool validate_callsign(const char *s) {
    int len = (int)strlen(s);
    if (len < 3 || len > CALLSIGN_LEN - 1)
        return false;
    int slash_count = 0;
    for (int i = 0; i < len; i++) {
        char c = (char)toupper((unsigned char)s[i]);
        if (!isalnum((unsigned char)c) && c != ' ' && c != '/')
            return false;
        if (c == '/')
            slash_count++;
    }
    if (slash_count > 1)
        return false;
    return true;
}

static bool validate_locator(const char *s) {
    size_t len = strlen(s);
    if (len != 4 && len != 6)
        return false;
    char c0 = (char)toupper((unsigned char)s[0]);
    char c1 = (char)toupper((unsigned char)s[1]);
    if (c0 < 'A' || c0 > 'R')
        return false;
    if (c1 < 'A' || c1 > 'R')
        return false;
    if (!isdigit((unsigned char)s[2]))
        return false;
    if (!isdigit((unsigned char)s[3]))
        return false;
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

static esp_err_t h_index(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
    return ESP_OK;
}

static esp_err_t h_get_config(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    web_server_cfg_lock();
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "callsign", _cfg->callsign);
    cJSON_AddStringToObject(j, "locator", _cfg->locator);
    char _pwr_str[8];
    snprintf(_pwr_str, sizeof(_pwr_str), "%u", (unsigned)_cfg->power_dbm);
    cJSON_AddRawToObject(j, "power_dbm", _pwr_str);
    cJSON_AddStringToObject(j, "wifi_ssid", _cfg->wifi_ssid);
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
    char _ppb_str[16];
    snprintf(_ppb_str, sizeof(_ppb_str), "%ld", (long)_cfg->xtal_cal_ppb);
    cJSON_AddRawToObject(j, "xtal_cal_ppb", _ppb_str);
    char _reg_str[4];
    snprintf(_reg_str, sizeof(_reg_str), "%u", (unsigned)_cfg->iaru_region);
    cJSON_AddRawToObject(j, "iaru_region", _reg_str);
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

static esp_err_t h_post_config(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    char buf[1024] = { 0 };
    int total = req->content_len;
    if (total >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "request body too large");
        return ESP_OK;
    }
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
    web_server_cfg_lock();
    char old_ntp_snap[sizeof(_cfg->ntp_server)];
    strncpy(old_ntp_snap, _cfg->ntp_server, sizeof(old_ntp_snap) - 1);
    old_ntp_snap[sizeof(old_ntp_snap) - 1] = '\0';
    cJSON *v;
    if (v_cs && cJSON_IsString(v_cs)) {
        strncpy(_cfg->callsign, v_cs->valuestring, CALLSIGN_LEN - 1);
        _cfg->callsign[CALLSIGN_LEN - 1] = '\0';
        for (int _i = 0; _cfg->callsign[_i] != '\0'; _i++)
            _cfg->callsign[_i] = (char)toupper((unsigned char)_cfg->callsign[_i]);
    }
    if (v_loc && cJSON_IsString(v_loc)) {
        strncpy(_cfg->locator, v_loc->valuestring, LOCATOR_LEN - 1);
        _cfg->locator[LOCATOR_LEN - 1] = '\0';
        for (int _i = 0; _cfg->locator[_i] != '\0'; _i++)
            _cfg->locator[_i] = (char)toupper((unsigned char)_cfg->locator[_i]);
    }
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
    if ((v = cJSON_GetObjectItem(j, "hop_interval_sec")) && cJSON_IsNumber(v)) {
        uint32_t interval = (uint32_t)v->valueint;
        if (interval < 120u)
            interval = 120u;
        if (interval > 86400u)
            interval = 86400u;
        _cfg->hop_interval_sec = interval;
    }
    if ((v = cJSON_GetObjectItem(j, "tx_duty_pct")) && cJSON_IsNumber(v)) {
        int pct = v->valueint;
        if (pct < 0)
            pct = 0;
        if (pct > 100)
            pct = 100;
        _cfg->tx_duty_pct = (uint8_t)pct;
    }
    if ((v = cJSON_GetObjectItem(j, "xtal_cal_ppb")) && cJSON_IsNumber(v)) {
        int32_t ppb = (int32_t)v->valueint;
        if (ppb < -200000)
            ppb = -200000;
        if (ppb > 200000)
            ppb = 200000;
        _cfg->xtal_cal_ppb = ppb;
    }
    if ((v = cJSON_GetObjectItem(j, "iaru_region")) && cJSON_IsNumber(v)) {
        int region = v->valueint;
        if (region < 1 || region > 3)
            region = 1;
        _cfg->iaru_region = (uint8_t)region;
    }
    cJSON *bands = cJSON_GetObjectItem(j, "band_enabled");
    if (cJSON_IsArray(bands)) {
        memset(_cfg->band_enabled, 0, sizeof(_cfg->band_enabled));
        int n = cJSON_GetArraySize(bands);
        if (n > BAND_COUNT)
            n = BAND_COUNT;
        for (int i = 0; i < n; i++)
            _cfg->band_enabled[i] = cJSON_IsTrue(cJSON_GetArrayItem(bands, i));
    }
    _cfg->bands_changed = true;
    wspr_config_t cfg_snap = *_cfg;
    int32_t new_cal_ppb = cfg_snap.xtal_cal_ppb;
    web_server_cfg_unlock();
    oscillator_set_cal(new_cal_ppb);

    if (strncmp(old_ntp_snap, cfg_snap.ntp_server, sizeof(cfg_snap.ntp_server)) != 0) {
        time_sync_restart_ntp(cfg_snap.ntp_server);
    }

    cJSON_Delete(j);
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

static esp_err_t h_tx_toggle(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    // [MODIFIED] Removed wspr_config_t cfg_snap and config_save() call.
    // config.h contract: "The flag is not saved to NVS by the toggle endpoint,
    // so it resets to false on every cold boot." Saving here violated that:
    // after a reboot the transmitter would start transmitting immediately without
    // any user action, which is unsafe and against the explicit design intent.
    web_server_cfg_lock();
    _cfg->tx_enabled = !_cfg->tx_enabled;
    bool enabled = _cfg->tx_enabled;
    web_server_cfg_unlock();
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"tx_enabled\":%s}", enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// handler: POST /api/tone_toggle
// Activates or deactivates tone test mode.
// Request body (JSON): {"freq_khz": <float 0.1-30000>, "active": <bool>}
// Response (JSON): {"tone_active": <bool>, "freq_khz": <float>}
// When active=true the scheduler immediately outputs a CW carrier at freq_khz
// and suspends all WSPR scheduling until active=false is sent.
static esp_err_t h_tone_toggle(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    char buf[128] = { 0 };
    int total = req->content_len;
    if (total > 0 && total < (int)sizeof(buf)) {
        int received = 0;
        while (received < total) {
            int r = httpd_req_recv(req, buf + received, total - received);
            if (r <= 0) {
                if (r == HTTPD_SOCK_ERR_TIMEOUT)
                    continue;
                return ESP_FAIL;
            }
            received += r;
        }
    }
    bool new_active = false;
    float new_freq = 7040.1f;
    cJSON *j = cJSON_Parse(buf);
    if (j) {
        cJSON *v;
        if ((v = cJSON_GetObjectItem(j, "active")) && cJSON_IsBool(v))
            new_active = cJSON_IsTrue(v);
        if ((v = cJSON_GetObjectItem(j, "freq_khz")) && cJSON_IsNumber(v))
            new_freq = (float)v->valuedouble;
        cJSON_Delete(j);
    }
    // Clamp to safe oscillator range
    if (new_freq < 0.1f)
        new_freq = 0.1f;
    if (new_freq > 30000.0f)
        new_freq = 30000.0f;
    web_server_cfg_lock();
    _cfg->tone_active = new_active;
    _cfg->tone_freq_khz = new_freq;
    web_server_cfg_unlock();
    ESP_LOGI(TAG, "Tone toggle: active=%s freq=%.3f kHz", new_active ? "ON" : "OFF", (double)new_freq);
    char freq_str[24];
    snprintf(freq_str, sizeof(freq_str), "%.3f", (double)new_freq);
    char resp[96];
    snprintf(resp, sizeof(resp), "{\"tone_active\":%s,\"freq_khz\":%s}", new_active ? "true" : "false", freq_str);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// h_status — adds "gps_active" boolean field to the status JSON response.
// gps_active is true when the time source selected at boot is GPS (not NTP).
// The JS uses this flag to enable/disable the "From GPS" locator button.
static esp_err_t h_status(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
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
    cJSON_AddBoolToObject(j, "settings_reset", config_was_reset());
    cJSON_AddBoolToObject(j, "gps_active", time_sync_source() == TIME_SYNC_GPS);
    // Add tone test mode fields so the UI can sync the button state
    web_server_cfg_lock();
    bool tone_active_snap = _cfg ? _cfg->tone_active : false;
    float tone_freq_snap = _cfg ? _cfg->tone_freq_khz : 0.0f;
    web_server_cfg_unlock();
    cJSON_AddBoolToObject(j, "tone_active", tone_active_snap);
    char _tone_freq_str[24];
    snprintf(_tone_freq_str, sizeof(_tone_freq_str), "%.3f", (double)tone_freq_snap);
    cJSON_AddRawToObject(j, "tone_freq_khz", _tone_freq_str);
    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

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
    free(json);
    return ESP_OK;
}

static esp_err_t h_reset(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", -1);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static esp_err_t h_live_update(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    char buf[512] = { 0 };
    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":true}", -1);
        return ESP_OK;
    }
    if (total >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large");
        return ESP_OK;
    }
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
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
    web_server_cfg_lock();
    cJSON *v;
    if ((v = cJSON_GetObjectItem(j, "iaru_region")) && cJSON_IsNumber(v)) {
        int region = v->valueint;
        if (region < 1 || region > 3)
            region = 1;
        if ((uint8_t)region != _cfg->iaru_region) {
            _cfg->iaru_region = (uint8_t)region;
            ESP_LOGI(TAG, "LIVE UPDATE: iaru_region=%d", region);
        }
    }
    if ((v = cJSON_GetObjectItem(j, "tx_duty_pct")) && cJSON_IsNumber(v)) {
        int pct = v->valueint;
        if (pct < 0)
            pct = 0;
        if (pct > 100)
            pct = 100;
        if ((uint8_t)pct != _cfg->tx_duty_pct) {
            _cfg->tx_duty_pct = (uint8_t)pct;
            ESP_LOGI(TAG, "LIVE UPDATE: tx_duty_pct=%d%%", pct);
        }
    }
    if ((v = cJSON_GetObjectItem(j, "hop_enabled")) && cJSON_IsBool(v)) {
        bool hop = cJSON_IsTrue(v);
        if (hop != _cfg->hop_enabled) {
            _cfg->hop_enabled = hop;
            ESP_LOGI(TAG, "LIVE UPDATE: hop_enabled=%s", hop ? "true" : "false");
        }
    }
    if ((v = cJSON_GetObjectItem(j, "hop_interval_sec")) && cJSON_IsNumber(v)) {
        uint32_t interval = (uint32_t)v->valueint;
        if (interval < 120u)
            interval = 120u;
        if (interval > 86400u)
            interval = 86400u;
        if (interval != _cfg->hop_interval_sec) {
            _cfg->hop_interval_sec = interval;
            ESP_LOGI(TAG, "LIVE UPDATE: hop_interval_sec=%lu", (unsigned long)interval);
        }
    }
    cJSON *bands = cJSON_GetObjectItem(j, "band_enabled");
    if (cJSON_IsArray(bands)) {
        bool new_bands[BAND_COUNT];
        memset(new_bands, 0, sizeof(new_bands));
        int n = cJSON_GetArraySize(bands);
        if (n > BAND_COUNT)
            n = BAND_COUNT;
        for (int i = 0; i < n; i++)
            new_bands[i] = cJSON_IsTrue(cJSON_GetArrayItem(bands, i));
        bool changed = false;
        for (int i = 0; i < BAND_COUNT; i++) {
            if (new_bands[i] != _cfg->band_enabled[i]) {
                changed = true;
                break;
            }
        }
        if (changed) {
            memcpy(_cfg->band_enabled, new_bands, sizeof(_cfg->band_enabled));
            _cfg->bands_changed = true;
            char band_list[64] = { 0 };
            int pos = 0;
            for (int i = 0; i < BAND_COUNT && pos < (int)sizeof(band_list) - 8; i++) {
                if (new_bands[i]) {
                    if (pos > 0)
                        band_list[pos++] = ',';
                    int len = snprintf(band_list + pos, (int)sizeof(band_list) - pos, "%s", BAND_NAME[i]);
                    if (len > 0)
                        pos += len;
                }
            }
            if (pos == 0)
                strncpy(band_list, "none", sizeof(band_list) - 1);
            ESP_LOGI(TAG, "LIVE UPDATE: band_enabled=[%s]", band_list);
        }
    }
    web_server_cfg_unlock();
    cJSON_Delete(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", -1);
    return ESP_OK;
}

// h_gps_loc — new GET /api/gps_loc endpoint.
// Returns JSON: {"valid":true,"lat":<decimal>,"lon":<decimal>}
// when GPS mode is active and a position fix has been received, or
// {"valid":false} otherwise (NTP mode, or GPS but no fix yet).
// The browser-side fillFromGPS() function converts the lat/lon to a
// 6-character Maidenhead grid locator using pure JavaScript math.
static esp_err_t h_gps_loc(httpd_req_t *req) {
#if CONFIG_WSPR_HTTP_AUTH_ENABLE
    if (!check_auth(req))
        return send_auth_challenge(req);
#endif
    gps_position_t pos;
    bool got = time_sync_get_position(&pos);

    cJSON *j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "valid", got && pos.valid);

    if (got && pos.valid) {
        // Encode lat/lon as strings with 6 decimal places to avoid cJSON
        // floating-point representation issues and keep precision for Maidenhead.
        char lat_str[24];
        char lon_str[24];
        // Use integer + fractional decomposition to avoid printf %f portability issues
        // on some ESP-IDF toolchain versions.
        // Format: sign, integer part, ".", 6-digit fractional part.
        double lat = pos.latitude_deg;
        double lon = pos.longitude_deg;
        const char *lat_sign = (lat < 0.0) ? "-" : "";
        const char *lon_sign = (lon < 0.0) ? "-" : "";
        if (lat < 0.0)
            lat = -lat;
        if (lon < 0.0)
            lon = -lon;
        int lat_int = (int)lat;
        int lat_frac = (int)((lat - lat_int) * 1000000.0 + 0.5);
        int lon_int = (int)lon;
        int lon_frac = (int)((lon - lon_int) * 1000000.0 + 0.5);
        // Cap fraction overflow (e.g. 0.9999995 rounds to 1.000000)
        if (lat_frac >= 1000000) {
            lat_int++;
            lat_frac = 0;
        }
        if (lon_frac >= 1000000) {
            lon_int++;
            lon_frac = 0;
        }
        snprintf(lat_str, sizeof(lat_str), "%s%d.%06d", lat_sign, lat_int, lat_frac);
        snprintf(lon_str, sizeof(lon_str), "%s%d.%06d", lon_sign, lon_int, lon_frac);
        // Send as raw numbers so JS receives them as JSON number type (not strings)
        cJSON_AddRawToObject(j, "lat", lat_str);
        cJSON_AddRawToObject(j, "lon", lon_str);
    }

    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

esp_err_t web_server_start(wspr_config_t *cfg) {
    _cfg = cfg;
    if (_cfg_mutex == NULL) {
        _cfg_mutex = xSemaphoreCreateMutex();
        if (_cfg_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create cfg mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    if (_status_mutex == NULL) {
        _status_mutex = xSemaphoreCreateMutex();
        if (_status_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create status mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.max_uri_handlers = 13; // +1 for /api/tone_toggle
    hcfg.stack_size = 8192;
    if (httpd_start(&_srv, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    static const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = h_index },
        { .uri = "/api/config", .method = HTTP_GET, .handler = h_get_config },
        { .uri = "/api/config", .method = HTTP_POST, .handler = h_post_config },
        { .uri = "/api/tx_toggle", .method = HTTP_POST, .handler = h_tx_toggle },
        { .uri = "/api/status", .method = HTTP_GET, .handler = h_status },
        { .uri = "/api/wifi_scan", .method = HTTP_GET, .handler = h_wifi_scan },
        { .uri = "/api/reset", .method = HTTP_POST, .handler = h_reset },
        { .uri = "/api/live_update", .method = HTTP_POST, .handler = h_live_update },
        { .uri = "/api/gps_loc", .method = HTTP_GET, .handler = h_gps_loc },
        // Tone test endpoint
        { .uri = "/api/tone_toggle", .method = HTTP_POST, .handler = h_tone_toggle },
    };
    // Use sizeof to avoid hardcoding route count
    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++)
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
