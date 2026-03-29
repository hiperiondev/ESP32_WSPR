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

// HTML lang attribute value
#define WEBUI_HTML_LANG "es"

// Station card
#define WEBUI_CARD_STATION_TITLE "Estaci&#243;n"
#define WEBUI_LABEL_CALLSIGN     "Indicativo"
#define WEBUI_LABEL_LOCATOR      "Localizador Maidenhead (4 car.)"
#define WEBUI_LABEL_POWER        "Potencia TX (dBm)"
#define WEBUI_LABEL_XTAL_CAL     "Calibraci&#243;n XTAL (ppb)"
#define WEBUI_HINT_XTAL_CAL      "Compensaci&#243;n del cristal en ppb. 0&#61;sin correcci&#243;n. Efectivo al reiniciar."
#define WEBUI_BTN_SAVE           "&#128190; Guardar configuraci&#243;n"

// WiFi card
#define WEBUI_CARD_WIFI_TITLE "Red WiFi (modo STA)"
#define WEBUI_BTN_SCAN_LABEL  "&#128268; Buscar redes WiFi..."
#define WEBUI_LABEL_PASSWORD  "Contrase&#241;a"
#define WEBUI_SHOW_PASS_TEXT  "Mostrar contrase&#241;a"
#define WEBUI_LABEL_NTP       "Servidor NTP"
#define WEBUI_HINT_NO_CRED    "Sin credenciales &#8594; modo AP (192.168.4.1)"

// Bands card
#define WEBUI_CARD_BANDS_TITLE "Bandas activas"

// IARU region card strings in Spanish
#define WEBUI_CARD_IARU_TITLE   "Regi&#243;n IARU y frecuencia en 60 m"
#define WEBUI_LABEL_IARU_REGION "Regi&#243;n IARU"
#define WEBUI_IARU_REGION_1     "Regi&#243;n 1 &mdash; Europa, &#193;frica, Oriente Medio (5,2886 MHz)"
#define WEBUI_IARU_REGION_2     "Regi&#243;n 2 &mdash; Am&#233;ricas (5,3465 MHz)"
#define WEBUI_IARU_REGION_3     "Regi&#243;n 3 &mdash; Asia, Pac&#237;fico (5,3670 MHz)"
#define WEBUI_HINT_IARU                                                                                                                                        \
    "Selecciona la frecuencia de marcado WSPR en 60 m seg&#250;n tu zona ITU/IARU. El resto de bandas usan la misma frecuencia en todo el mundo."

// Hop card
#define WEBUI_CARD_HOP_TITLE     "Salto de frecuencia"
#define WEBUI_TOGGLE_HOP_LABEL   "Habilitar salto autom&#225;tico"
#define WEBUI_LABEL_HOP_INTERVAL "Intervalo (s):"
#define WEBUI_HINT_HOP           "El transmisor rotar&#225; por las bandas seleccionadas cada N segundos (m&#237;n. 120 s = 1 TX)."

// Duty cycle card
#define WEBUI_CARD_DUTY_TITLE  "Ciclo de trabajo TX"
#define WEBUI_LABEL_DUTY       "Porcentaje de slots TX activos (0-100 %)"
#define WEBUI_HINT_DUTY_INLINE "&nbsp;% (0=nunca, 20=est&#225;ndar WSPR, 100=siempre)"
#define WEBUI_HINT_DUTY        "20% transmite 1 de cada 5 slots (recomendado WSPR). 100% usa todos los slots disponibles."

// TX control card
#define WEBUI_CARD_TX_TITLE "Control TX"

// Status row labels
#define WEBUI_STATUS_HW_LABEL   "Hardware RF"
#define WEBUI_STATUS_TIME_LABEL "Sincronizaci&#243;n horaria"
#define WEBUI_STATUS_BAND_LABEL "Banda actual"
#define WEBUI_STATUS_FREQ_LABEL "Frecuencia"
#define WEBUI_STATUS_NEXT_LABEL "Pr&#243;xima TX"
#define WEBUI_STATUS_TX_LABEL   "TX activa"
#define WEBUI_STATUS_SYM_LABEL  "S&#237;mbolo"

// WSPRnet link
#define WEBUI_WSPR_LINK_TEXT "&#127760; Ver mis spots en WSPRnet &#8599;"

// JS strings
#define WEBUI_JS_BTN_TX_START     "Activar TX"
#define WEBUI_JS_BTN_TX_STOP      "Detener TX"
#define WEBUI_JS_ERR_LOAD         "Error cargando config: "
#define WEBUI_JS_SAVED            "Configuraci\\u00f3n guardada"
#define WEBUI_JS_ERR_TX           "Error TX: "
#define WEBUI_JS_NOT_SYNCED       "Sin sincronizar"
#define WEBUI_JS_TRANSMITTING     "Transmitiendo"
#define WEBUI_JS_SCANNING_TEXT    "Buscando..."
#define WEBUI_JS_BTN_SCAN_RESTORE "\xf0\x9f\x94\x8c Buscar redes WiFi..."
#define WEBUI_JS_NO_NETS          "No se encontraron redes"
#define WEBUI_JS_HIDDEN           "(oculto)"
#define WEBUI_JS_ERR_SCAN         "Error en scan WiFi: "
#define WEBUI_JS_PASS_SAVED       "(contrase\\u00f1a guardada)"
#define WEBUI_JS_CONFIRM_RESET    "\\u00bfReiniciar el ESP32?"
#define WEBUI_JS_RESTARTING       "Reiniciando..."
