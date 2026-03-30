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
#include "version.h"
#include "web_server.h"
#include "webui_strings.h"
#include "wifi_manager.h"

#if CONFIG_WSPR_HTTP_AUTH_ENABLE
#include "mbedtls/base64.h"

#define HTTP_AUTH_WARNING

// check_auth: validate Authorization: Basic <b64> header against Kconfig credentials.
// Returns true when credentials match, false when header is absent or wrong.
static bool check_auth(httpd_req_t *req) {
    char auth_hdr[160] = { 0 };
    // ESP-IDF httpd: returns ESP_OK only when the header value fits in the buffer.
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_hdr, sizeof(auth_hdr)) != ESP_OK)
        return false;
    // Basic auth header must start with "Basic " (6 chars).
    if (strncmp(auth_hdr, "Basic ", 6) != 0)
        return false;
    // Decode the base64 credential string that follows "Basic ".
    unsigned char decoded[128] = { 0 };
    size_t decoded_len = 0;
    const unsigned char *b64 = (const unsigned char *)(auth_hdr + 6);
    int rc = mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &decoded_len, b64, strlen(auth_hdr + 6));
    if (rc != 0)
        return false;
    decoded[decoded_len] = '\0';
    // Decoded string must be exactly "user:password".
    char expected[160];
    snprintf(expected, sizeof(expected), "%s:%s", CONFIG_WSPR_HTTP_AUTH_USER, CONFIG_WSPR_HTTP_AUTH_PASS);
    return strcmp((char *)decoded, expected) == 0;
}

// send_auth_challenge: emit HTTP 401 with WWW-Authenticate header so the browser
// shows its native credential prompt.
static esp_err_t send_auth_challenge(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"WSPR\"");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "401 Unauthorized", -1);
    return ESP_OK;
}

#else
#define HTTP_AUTH_WARNING "<p style='text-align:center;color:#92400e;font-size:.78em;margin-bottom:4px'> &#9888; " WEBUI_HTTP_AUTH_WARNING "</p>"
#endif

static const char *TAG = "web";
static httpd_handle_t _srv = NULL;
static wspr_config_t *_cfg = NULL;

static SemaphoreHandle_t _status_mutex = NULL;
static SemaphoreHandle_t _cfg_mutex = NULL;

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
    "</style></head><body>"
    "<h1>&#128225; WSPR Transmitter</h1>" HTTP_AUTH_WARNING "<div id='reboot_info' class='reboot-info'></div>"
    "<button class='btn-reset' onclick='resetESP()'>&#9211; Reset ESP32</button>"
    "<div class='grid'>"

    "<div class='card'>"
    "<h2>&#128752; " WEBUI_CARD_STATION_TITLE "</h2>"
    "<label>" WEBUI_LABEL_CALLSIGN "</label>"
    "<input id='callsign' type='text' maxlength='11' placeholder='LU1ABC'>"
    "<label>" WEBUI_LABEL_LOCATOR "</label>"
    "<input id='locator' type='text' maxlength='6' placeholder='GF05'>"
    "<label>" WEBUI_LABEL_POWER "</label>"
    "<input id='power' type='number' min='0' max='60' step='3'>"
    "<label>" WEBUI_LABEL_XTAL_CAL "</label>"
    "<input id='xtal_cal' type='number' min='-100000' max='100000' step='100'>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:4px'>" WEBUI_HINT_XTAL_CAL "</p>"
    "<button class='btn-save' onclick='save()'>" WEBUI_BTN_SAVE "</button>"
    "</div>"

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

    "<div class='card'>"
    "<h2>&#128251; " WEBUI_CARD_BANDS_TITLE "</h2>"
    "<div class='bands' id='bands'></div>"
    "</div>"

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

    "<div class='card'>"
    "<h2>&#128202; " WEBUI_CARD_DUTY_TITLE "</h2>"
    "<label>" WEBUI_LABEL_DUTY "</label>"
    "<div class='row'>"
    "<input id='tx_duty_pct' type='number' min='0' max='100' step='5' style='width:80px'>"
    "<span style='color:var(--sub);font-size:.85em'>" WEBUI_HINT_DUTY_INLINE "</span>"
    "</div>"
    "<p style='color:var(--sub);font-size:.78em;margin-top:8px'>" WEBUI_HINT_DUTY "</p>"
    "</div>"

    "<div class='card'>"
    "<h2>&#9889; " WEBUI_CARD_TX_TITLE "</h2>"
    "<div class='toggle'>"
    "<button class='btn-tx off' id='tx_btn' onclick='toggleTx()'>" WEBUI_JS_BTN_TX_START "</button>"
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
    "<a id='wspr_link' href='https://wsprnet.org' target='_blank' rel='noopener'"
    " style='color:var(--accent);font-size:.85em;text-decoration:none'>" WEBUI_WSPR_LINK_TEXT "</a></div>"
    "</div>"

    "</div>"
    "<div class='toast' id='toast'></div>"

    "<script>"
    "const BANDS=['2200m','630m','160m','80m','60m','40m','30m','20m','17m','15m','12m','10m'];"
    "let cfg={};"

    "let _hasPass=false;"
    "let _passEdited=false;"
    "let _rebootInfoSet=false;"
    "let _settingsResetShown=false;"

    "function buildBands(enabled){"
    "const c=document.getElementById('bands');c.innerHTML='';"
    "BANDS.forEach((b,i)=>{"
    "const d=document.createElement('label');"
    "d.className='band-cb';"
    "d.innerHTML=`<input type='checkbox' value='${i}' ${enabled[i]?'checked':''}>${b}`;"
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
    "if(cs){"
    "document.getElementById('wspr_link').href="
    "'https://wsprnet.org/drupal/wsprnet/spots?callsign='+encodeURIComponent(cs);"
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

    "if(!body.callsign||body.callsign.length<3||!/^[A-Z0-9 ]{3,11}$/.test(body.callsign)){"
    "toast('\u274c " WEBUI_JS_ERR_CALLSIGN "','err');return;}"
    "if(!body.locator||!/^[A-Ra-r]{2}[0-9]{2}([A-Xa-x]{2})?$/.test(body.locator)){"
    "toast('\u274c " WEBUI_JS_ERR_LOCATOR "','err');return;}"
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
    if (_status_mutex && xSemaphoreTake(_status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _status.hw_ok = hw_ok;
        if (hw_name) {
            strncpy(_status.hw_name, hw_name, sizeof(_status.hw_name) - 1);
            _status.hw_name[sizeof(_status.hw_name) - 1] = '\0';
        }
        xSemaphoreGive(_status_mutex);
    } else {
        _status.hw_ok = hw_ok;
        if (hw_name) {
            strncpy(_status.hw_name, hw_name, sizeof(_status.hw_name) - 1);
            _status.hw_name[sizeof(_status.hw_name) - 1] = '\0';
        }
    }
    ESP_LOGI("web", "HW status: %s — %s", hw_ok ? "OK" : "DUMMY", _status.hw_name);
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
        xSemaphoreTake(_cfg_mutex, portMAX_DELAY);
    }
}

void web_server_cfg_unlock(void) {
    if (_cfg_mutex) {
        xSemaphoreGive(_cfg_mutex);
    }
}

// validate_callsign: now allows '/' for compound callsigns (Type-2).
// Compound form: PREFIX/CALL (e.g. PJ4/K1ABC) or CALL/SUFFIX (e.g. K1ABC/P).
// At most one slash is permitted; wspr_encode() routes compound callsigns to
// Type-2 encoding which handles the slash correctly.
static bool validate_callsign(const char *s) {
    int len = (int)strlen(s);
    // Minimum length is 3 (e.g. "N0X"), maximum is CALLSIGN_LEN-1.
    if (len < 3 || len > CALLSIGN_LEN - 1)
        return false;
    int slash_count = 0;
    for (int i = 0; i < len; i++) {
        char c = (char)toupper((unsigned char)s[i]);
        // Allow '/' in addition to alphanumeric and space.
        if (!isalnum((unsigned char)c) && c != ' ' && c != '/')
            return false;
        if (c == '/')
            slash_count++;
    }
    // At most one slash allowed (PREFIX/CALL/SUFFIX form is not valid WSPR).
    if (slash_count > 1)
        return false;
    return true;
}

// validate_locator: accepts 4-char (DDLL) or 6-char (DDLLSS) Maidenhead locators.
// 4-char locators produce standard Type-1 transmissions.
// 6-char locators enable Type-3 companion transmissions for sub-square precision.
static bool validate_locator(const char *s) {
    size_t len = strlen(s);
    // Accept exactly 4 or exactly 6 characters.
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
    // Validate subsquare letters for 6-char locator (A..X each).
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
    // All integer fields use cJSON_AddRawToObject instead of
    // cJSON_AddNumberToObject to avoid software double-precision FP emulation.
    // The Xtensa LX6 core has a hardware single-precision FPU only; every
    // cJSON_AddNumberToObject call internally stores the value as double and
    // triggers libgcc software FP routines, wasting hundreds of cycles and stack.
    // cJSON_AddRawToObject with an snprintf-formatted string is exact and FP-free.
    {
        // power_dbm: was cJSON_AddNumberToObject (software double FP)
        char _pwr_str[8];
        snprintf(_pwr_str, sizeof(_pwr_str), "%u", (unsigned)_cfg->power_dbm);
        cJSON_AddRawToObject(j, "power_dbm", _pwr_str);
    }
    cJSON_AddStringToObject(j, "wifi_ssid", _cfg->wifi_ssid);
    cJSON_AddBoolToObject(j, "has_pass", _cfg->wifi_pass[0] != '\0');
    cJSON_AddStringToObject(j, "ntp_server", _cfg->ntp_server);
    char _hop_str[16];
    snprintf(_hop_str, sizeof(_hop_str), "%lu", (unsigned long)_cfg->hop_interval_sec);
    cJSON_AddRawToObject(j, "hop_interval_sec", _hop_str);
    cJSON_AddNumberToObject(j, "hop_interval_sec", _cfg->hop_interval_sec);
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
    if (total > (int)(sizeof(buf) - 1))
        total = sizeof(buf) - 1;
    {
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
        if (ppb < -100000)
            ppb = -100000;
        if (ppb > 100000)
            ppb = 100000;
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
        int n = cJSON_GetArraySize(bands);
        if (n > BAND_COUNT)
            n = BAND_COUNT;
        for (int i = 0; i < n; i++)
            _cfg->band_enabled[i] = cJSON_IsTrue(cJSON_GetArrayItem(bands, i));
    }

    _cfg->bands_changed = true; // mark for rebuild when band_enabled or hop_enabled changed via web UI

    wspr_config_t cfg_snap = *_cfg;
    // snapshot the new calibration value before releasing the
    // lock so we can apply it to the oscillator driver without holding the mutex.
    int32_t new_cal_ppb = cfg_snap.xtal_cal_ppb;
    web_server_cfg_unlock();

    // apply the new crystal calibration immediately so it takes
    // effect on the next oscillator_set_freq() call without requiring a reboot.
    // oscillator_set_cal() only stores the ppb value internally; it does not
    // reprogram hardware until the next frequency-set call, making this safe
    // to call from the HTTP task at any time, even during an active TX.
    oscillator_set_cal(new_cal_ppb);

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
    wspr_config_t cfg_snap;
    web_server_cfg_lock();
    _cfg->tx_enabled = !_cfg->tx_enabled;
    bool enabled = _cfg->tx_enabled;
    cfg_snap = *_cfg;
    web_server_cfg_unlock();

    config_save(&cfg_snap);

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"tx_enabled\":%s}", enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t h_status(httpd_req_t *req) {
    // Auth guard: status endpoint is polled by the UI every 2 s.
    // Protect it so that unauthenticated clients receive 401, not live data.
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
    // avoid double FP for integer status fields.
    {
        char _ntx_str[16];
        snprintf(_ntx_str, sizeof(_ntx_str), "%ld", (long)snap.next_tx_sec);
        cJSON_AddRawToObject(j, "next_tx_sec", _ntx_str);
    }
    cJSON_AddBoolToObject(j, "tx_active", snap.tx_active);
    cJSON_AddBoolToObject(j, "tx_enabled", snap.tx_enabled);
    {
        char _sym_str[8];
        snprintf(_sym_str, sizeof(_sym_str), "%d", snap.symbol_idx);
        cJSON_AddRawToObject(j, "symbol_idx", _sym_str);
    }
    // expose reboot info so the UI subtitle is populated from the API
    cJSON_AddStringToObject(j, "boot_time_str", snap.reboot_time_str);
    cJSON_AddStringToObject(j, "reboot_reason", snap.reboot_reason);
    // expose settings_reset flag so the JS shows a one-time
    // toast when NVS config was discarded and defaults were applied this boot.
    cJSON_AddBoolToObject(j, "settings_reset", config_was_reset());
    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

static esp_err_t h_wifi_scan(httpd_req_t *req) {
    // Auth guard: Wi-Fi scan reveals nearby SSIDs; require auth.
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
    // Auth guard: /api/reset causes esp_restart(); must be protected.
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
    hcfg.max_uri_handlers = 10;
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
