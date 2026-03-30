<a name="readme-top"></a>

<div align="center">

[![Logo](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/logo.jpg)](https://github.com/hiperiondev/ESP32_WSPR)

# ESP32\_WSPR

**ESP32 WSPR (Weak Signal Propagation Reporter) Transmitter**

*A fully-featured, standalone WSPR beacon built on ESP-IDF — no Arduino required*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Oscillators](https://img.shields.io/badge/Oscillator-Si5351A%20%7C%20AD9850-red.svg)](#oscillator-hardware)

</div>

---

## Table of Contents

1. [About The Project](#about-the-project)
2. [WSPR Protocol Overview](#wspr-protocol-overview)
3. [Features](#features)
4. [Hardware Requirements](#hardware-requirements)
5. [Architecture & Source Code](#architecture--source-code)
6. [Web Interface (WebUI)](#web-interface-webui)
7. [Configuration Reference](#configuration-reference)
8. [Building & Flashing](#building--flashing)
9. [Menuconfig Options (Kconfig)](#menuconfig-options-kconfig)
10. [Low-Pass Filter Bank](#low-pass-filter-bank)
11. [Oscillator Hardware](#oscillator-hardware)
12. [Time Synchronisation](#time-synchronisation)
13. [Wi-Fi & Networking](#wi-fi--networking)
14. [WSPR Band Frequencies & IARU Regions](#wspr-band-frequencies--iaru-regions)
15. [Frequency Hopping Mode](#frequency-hopping-mode)
16. [TX Duty Cycle](#tx-duty-cycle)
17. [Crystal Calibration](#crystal-calibration)
18. [Implementation Status](#implementation-status)
19. [Roadmap](#roadmap)
20. [Contributing](#contributing)
21. [License](#license)
22. [Contact](#contact)
23. [References](#references)

---

## About The Project

**ESP32\_WSPR** is a complete, standalone WSPR (Weak Signal Propagation Reporter) beacon transmitter firmware built on Espressif's **ESP-IDF** framework for the ESP32 microcontroller family. Unlike most hobby WSPR projects that rely on the Arduino ecosystem, this firmware is written in pure C against the native ESP-IDF APIs, giving it access to FreeRTOS task management, the native SNTP client, the `esp_wifi` stack, `nvs_flash` persistent storage, and the `esp_http_server` web server — all without the overhead of the Arduino HAL.

The project is designed to be production-ready for unattended beacon operation: it encodes WSPR Type-1 messages entirely on-chip, drives an RF oscillator (Si5351A or AD9850) with sub-Hz symbol resolution, selects the correct band-pass / low-pass filter automatically via a 3-bit GPIO bus, synchronises time via NTP or GPS, and exposes a responsive single-page web application for configuration and monitoring. All user settings are persisted in the ESP32's NVS (non-volatile storage) flash partition and survive power cycles.

The WSPR mode occupies roughly 6 Hz of RF bandwidth and can be decoded at signal-to-noise ratios as low as −28 dB in a 2.5 kHz reference bandwidth, making it extremely useful for propagation studies using very low power levels. Once transmitted, reception reports from automated WSPR stations worldwide are automatically uploaded to [WSPRnet](https://www.wsprnet.org), where a mapping interface lets you see exactly how far your signal travelled.

### Key design decisions

- **ESP-IDF only** — no Arduino, no third-party I2C libraries. All oscillator drivers are written from scratch against the IDF `driver/i2c.h` and `driver/gpio.h` APIs.
- **Dual oscillator support** — the firmware auto-detects a Si5351A at boot via I2C ACK probe, then falls back to AD9850 (write-only SPI bit-bang), then falls back to a silent dummy mode if neither is present — so the system never crashes on missing hardware.
- **Integer-only arithmetic** — the entire WSPR encoder, Si5351 PLL divider calculation, and AD9850 tuning-word computation use 32-bit integer maths only. No floating point, no `double`, making the code efficient on the Xtensa LX6 without enabling the soft-float library.
- **Embedded single-file SPA** — the web interface is compiled into the firmware as a C header file; there is no filesystem component and no SPIFFS/LittleFS partition needed.
- **Multilingual WebUI** — English and Spanish UI string tables are provided as separate headers (`webui_en.h` / `webui_es.h`) selected at compile time via Kconfig.
- **Graceful degradation** — missing oscillator hardware, lost Wi-Fi, unavailable NTP, and corrupted NVS blobs are all handled without crashing, with informative log messages and web UI status indicators.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## WSPR Protocol Overview

WSPR (**W**eak **S**ignal **P**ropagation **R**eporter, pronounced *"whisper"*) is a digital amateur radio protocol designed by Joe Taylor (K1JT), Nobel Prize winner in Physics, and originally released in 2008. It is part of the WSJT-X suite and has become one of the most widely used propagation beacon modes in amateur radio.

### What WSPR transmits

A WSPR Type-1 message encodes exactly three pieces of information:

| Field | Description | Encoding width |
|---|---|---|
| **Callsign** | Station identifier (up to 6 alphanumeric chars) | 28 bits |
| **Maidenhead locator** | 4-character grid square (e.g. `GF05`) | 15 bits |
| **TX power** | Transmit power in dBm (one of 19 valid levels) | 7 bits |

Total payload: **50 bits**.

### Encoding pipeline (G4JNT / K1JT specification)

The following pipeline is implemented in `wspr_encode.c`:

```
Input: callsign + locator + power_dBm
        │
        ▼
1. Pack callsign → 28-bit integer (G4JNT standard formula)
   - Pad to 6 chars; prepend space if char[1] is a letter
   - Chars 0-1: 37-symbol alphabet (0-9=0..9, A-Z=10..35, space=36)
   - Char 2:    digit only (0-9)
   - Chars 3-5: 27-symbol suffix alphabet (A-Z=0..25, space=26)
        │
        ▼
2. Pack locator → 15-bit integer
   - Formula: (179 - 10*(c0-'A') - d0) * 180 + (10*(c1-'A') + d1)
        │
        ▼
3. Pack power → 7-bit integer  (power_dBm + 64)
   - Rounded to nearest valid WSPR level
        │
        ▼
4. Assemble 50-bit message into 7 bytes (left-justified, 6 pad bits)
        │
        ▼
5. Convolutional encode (K=32, rate 1/2)
   - Polynomials: G1=0xF2D05351, G2=0xE4613C47
   - Input: 50 data + 31 tail bits = 81 bits → 162 output bits
        │
        ▼
6. Bit-reversal interleave (256-point bit-reversal permutation)
        │
        ▼
7. Combine with 162-bit sync vector:
   symbol[i] = 2 * interleaved_bit[i] + sync[i]    → values 0..3
        │
        ▼
Output: 162 four-FSK symbols (values 0, 1, 2, 3)
```

### RF characteristics

| Parameter | Value |
|---|---|
| Modulation | 4-FSK (4 tones), phase-continuous |
| Emission designator | F1D |
| Tone spacing | 12000 / 8192 Hz ≈ **1.4648 Hz** |
| Symbol period | 8192 / 12000 s ≈ **682.667 ms** |
| Total TX duration | 162 × 682.667 ms ≈ **110.6 seconds** |
| Occupied bandwidth | ~6 Hz (3 × tone spacing) |
| Audio passband offset | **+1500 Hz** above dial frequency |
| Minimum decodable SNR | −28 dB in 2.5 kHz BW |

### Timing

WSPR transmissions **must** begin at the start of an even UTC minute (00:00, 00:02, 00:04, …). The firmware calculates `next_tx = (now - (now % 120)) + 120 + 1` — adding one second to allow relay contacts to settle — and schedules the transmission accordingly. Time accuracy must be better than ±1 second; the firmware waits for NTP or GPS synchronisation before attempting any transmission.

### Duty cycle

The WSPR community standard recommends transmitting in at most 20% of available 2-minute slots, leaving the remainder for receiving. This firmware implements a configurable duty cycle percentage (0–100%) and a random slot-selection mechanism to distribute transmissions across time and avoid systematic collisions with other beacons.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Features

- ✅ **Full WSPR Type-1, 2 and 3 encoder** — callsign packing, convolutional encoding (K=32, rate 1/2), bit-reversal interleaving, sync vector overlay; integer-only arithmetic
- ✅ **Dual oscillator support** — Si5351A (I2C, auto-detected) and AD9850 (GPIO bit-bang, assumed present); graceful dummy mode if neither found
- ✅ **12 WSPR bands** — 2200 m through 10 m (137 kHz to 28 MHz)
- ✅ **IARU Region selection** — Region 1, 2, or 3 for correct 60 m dial frequency
- ✅ **Automatic low-pass filter selection** — 3-bit BCD GPIO bus, 8 filter positions, relay-settle delay
- ✅ **Time sync via NTP** (SNTP, selectable server) or **GPS** (NMEA-0183 GPRMC/GNRMC via UART)
- ✅ **Wi-Fi STA mode** with soft-AP fallback (192.168.4.1) and background reconnect timer
- ✅ **Frequency hopping** — automatic rotation through enabled bands every N seconds (min. 120 s = 1 TX slot)
- ✅ **TX duty cycle** — configurable 0–100% with random slot selection
- ✅ **Crystal calibration** — ±ppb correction stored in NVS, applied to all frequency calculations
- ✅ **Embedded single-page web application** — no SPIFFS, no external files; entirely self-contained
- ✅ **REST API** — JSON endpoints for config read/write, status polling, Wi-Fi scan, TX toggle, system reset
- ✅ **NVS persistent config** — all settings survive power cycles; schema version check with automatic defaults on mismatch
- ✅ **Multilingual UI** — English and Spanish (compile-time selection via Kconfig)
- ✅ **WSPRnet integration** — direct link from the WebUI to your station's spot map
- ✅ **ESP-IDF native** — no Arduino dependency; uses FreeRTOS, esp_wifi, esp_sntp, esp_http_server, nvs_flash, driver/i2c, driver/gpio, driver/uart

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Hardware Requirements

### Minimum

| Component | Specification |
|---|---|
| **Microcontroller** | Any ESP32 module (ESP32-WROOM-32, ESP32-DevKitC, ESP32-WROVER, etc.) |
| **RF Oscillator** | Si5351A breakout board (I2C) **or** AD9850 DDS module (GPIO SPI) |
| **Antenna** | Wire antenna appropriate for the operating band(s) |

### Recommended additional components

| Component | Purpose |
|---|---|
| **Low-pass filter bank** | Harmonic suppression (legally required in most jurisdictions) |
| **BCD decoder / relay driver** | Controlled by 3 GPIO lines (GPIO_A, GPIO_B, GPIO_C) |
| **GPS module** (NMEA UART) | For GPS time sync (alternative to NTP) |
| **Power amplifier** | Increase output beyond the ~10 dBm from the oscillator |
| **3.3 V power supply** | Stable supply for the ESP32 and Si5351 |

### Oscillator wiring

#### Si5351A (preferred)

```
ESP32          Si5351A breakout
GPIO_SDA  ──── SDA  (with 4.7 kΩ pull-up to 3.3 V)
GPIO_SCL  ──── SCL  (with 4.7 kΩ pull-up to 3.3 V)
3.3 V     ──── VCC
GND       ──── GND
CLK0      ──── LPF input (via matching network)
```

- I2C address: `0x60` (fixed on most breakout boards)
- I2C speed: 400 kHz (Fast-Mode)
- Crystal: 25 MHz or 27 MHz (configurable via Kconfig)
- Drive current: 2 mA / 4 mA / 6 mA / 8 mA (configurable via Kconfig)
- Output: square wave, 3.3 V logic levels, ~10 dBm into 50 Ω

#### AD9850 DDS

```
ESP32                   AD9850 module
AD9850_CLK_GPIO    ──── W_CLK
AD9850_FQ_UD_GPIO  ──── FQ_UD
AD9850_DATA_GPIO   ──── D7 / DATA (serial mode)
AD9850_RESET_GPIO  ──── RESET
3.3 V / 5 V        ──── VCC  (check module voltage)
GND                ──── GND
OUT1               ──── LPF input (sine wave output)
```

- Reference clock: 125 MHz (most common AD9850 modules)
- Interface: bit-bang serial (MSB-first, 32-bit frequency word + 8-bit control)
- Frequency tuning word: `FTW = freq_Hz × 2^32 / ref_clk_Hz`
- Output: sine wave, ~1 V peak-to-peak into 50 Ω (much lower than Si5351)

### Filter bank wiring

```
ESP32                BCD decoder / relay driver board
GPIO_A (bit 0)  ──── A input
GPIO_B (bit 1)  ──── B input
GPIO_C (bit 2)  ──── C input
3.3 V           ──── Logic VCC
GND             ──── GND
```

Eight filter positions (0–7) are selected by the 3-bit binary code on GPIO_A/B/C. The firmware selects the correct filter for each band automatically using the `BAND_FILTER[]` table in `config.c`.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Architecture & Source Code

The firmware is organised as a single ESP-IDF component (`main/`). All source files are compiled together via `CMakeLists.txt` using `GLOB_RECURSE`.

### Module map

```
main/
├── CMakeLists.txt          — IDF component registration, dependency list
├── Kconfig.projbuild       — All menuconfig options (pins, oscillator, Wi-Fi, etc.)
│
├── main.c                  — app_main(), scheduler_task(), status_task(), wspr_transmit()
│
├── config.c / config.h     — Persistent config struct, NVS load/save/defaults, band tables
├── oscillator.c / .h       — Unified oscillator API; Si5351A + AD9850 drivers; auto-detect
├── gpio_filter.c / .h      — 3-bit GPIO LPF bank driver
├── time_sync.c / .h        — NTP (SNTP) and GPS (NMEA UART) time sync abstraction
├── wifi_manager.c / .h     — Wi-Fi STA + AP fallback; background reconnect; Wi-Fi scan
├── web_server.c / .h       — HTTP server; REST API; status cache; config mutex
├── wspr_encode.c / .h      — Complete WSPR Type-1 encoder (integer-only)
│
├── webui_strings.h         — Dispatch header: includes webui_en.h or webui_es.h
├── webui_en.h              — English UI string table
└── webui_es.h              — Spanish UI string table
```

### Task structure (FreeRTOS)

The firmware runs three long-lived FreeRTOS tasks after `app_main()` returns:

```
app_main()
  │
  ├─ [initialisation sequence]
  │    config_init() → config_load()
  │    oscillator_init() → oscillator_set_cal()
  │    gpio_filter_init()
  │    wifi_manager_start()
  │    web_server_start()
  │    time_sync_init()
  │
  ├─ xTaskCreate(scheduler_task)   — waits for time sync, computes next TX slot,
  │                                   calls wspr_transmit(), handles hop logic
  │
  └─ xTaskCreate(status_task)      — polls time/status every second,
                                      calls web_server_update_status()
```

### WSPR transmission sequence (`wspr_transmit()`)

```
1. Lock config mutex
2. Encode 162 symbols (wspr_encode)
3. Select LPF via gpio_filter_select(BAND_FILTER[band])
4. vTaskDelay(10 ms) — relay settle time
5. Set oscillator base frequency (dial + 1500 Hz offset)
6. oscillator_enable(true)
7. For each of 162 symbols:
   a. oscillator_set_freq_mhz(base_hz, symbol * 1464843 mHz)
   b. vTaskDelay(683 ms — symbol period)
8. oscillator_enable(false)
9. Unlock config mutex
```

Symbol tone offsets (in milli-Hz): `symbol × (12000000 / 8192) mHz = symbol × 1464.84375 Hz × 1000`

### Configuration structure (`wspr_config_t`)

All settings are stored in a single NVS blob identified by the key `"cfg"` in namespace `"wspr"`. The current schema version is **4**.

```c
typedef struct {
    uint8_t  version;               // Schema version (must equal CONFIG_SCHEMA_VERSION)
    char     callsign[12];          // Amateur callsign
    char     locator[7];            // 4-char Maidenhead grid square
    uint8_t  power_dbm;             // TX power in dBm (WSPR valid levels)
    char     wifi_ssid[33];         // Wi-Fi SSID
    char     wifi_pass[65];         // Wi-Fi password
    bool     band_enabled[12];      // One flag per WSPR band
    bool     hop_enabled;           // Enable automatic frequency hopping
    uint32_t hop_interval_sec;      // Hop interval in seconds (min. 120)
    char     ntp_server[64];        // NTP hostname or IP
    bool     tx_enabled;            // Master TX enable/disable
    uint8_t  tx_duty_pct;           // TX duty cycle (0-100 %)
    int32_t  xtal_cal_ppb;          // Crystal calibration offset in ppb
    uint8_t  iaru_region;           // IARU region: 1, 2, or 3
} wspr_config_t;
```

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Web Interface (WebUI)

The web interface is a single-page application (SPA) served entirely from flash memory as a C string literal embedded in `web_server.c`. No filesystem component (SPIFFS/LittleFS) is needed. The interface auto-refreshes status data every 2 seconds via the `/api/status` endpoint.

### Accessing the WebUI

| Scenario | URL |
|---|---|
| Connected to your home Wi-Fi (STA mode) | `http://<assigned-IP>` |
| No Wi-Fi credentials, AP fallback mode | `http://192.168.4.1` |
| After Wi-Fi scan | IP shown in `/api/status` → `ip` field |

The IP address is also printed to the ESP32 serial console at boot.

### WebUI screenshots

#### Main page overview

The main page is divided into several configuration cards, a live status panel, and a TX control button. All changes require saving with the **Save configuration** button before they take effect. Configuration is persisted immediately to NVS flash.

---

#### Station card

![WebUI Station](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_station.jpg)

The **Station** card (Spanish: *Estación*) configures the WSPR message payload:

- **Callsign** — Your amateur radio callsign (1–6 alphanumeric characters). The encoder automatically handles the G4JNT normalization rule (prepends a space if character 1 is a letter, so `G4JNT` becomes `[sp]G4JNT`).
- **Maidenhead Locator** — 4-character grid square (e.g. `GF05`, `FN20`, `IO91`). 6-character sub-squares are rejected by the encoder since they cannot be represented in a WSPR Type-1 message.
- **TX Power (dBm)** — Transmit power level. Must be one of the 19 valid WSPR levels: 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 dBm. Values outside this set are rounded to the nearest valid entry.
- **XTAL Calibration (ppb)** — Crystal frequency correction in parts-per-billion. A positive value means the crystal runs fast (the firmware lowers the effective frequency to compensate). This value is applied to all oscillator frequency calculations. Changes take effect after a restart.

---

#### Wi-Fi card

![WebUI WiFi](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_wifi.jpg)

The **Wi-Fi Network** card handles connectivity:

- **Scan button** — Triggers a blocking Wi-Fi channel scan (≈2 s) and populates a dropdown list of discovered access points with their SSID, signal strength (RSSI), and security type. Clicking an entry fills the SSID field automatically.
- **Password** — WPA/WPA2 passphrase (show/hide toggle included).
- **NTP Server** — Hostname or IP of the NTP server (default: `pool.ntp.org`). Used only in NTP time-sync mode.
- **No credentials hint** — If the SSID field is left blank, the ESP32 starts in soft-AP mode at `192.168.4.1`. This is useful for initial configuration without a known network.

If STA connection fails, the firmware falls back to AP mode and starts a background reconnect timer (5-minute interval) so the device will reconnect automatically when the home network becomes available again.

---

#### Active Bands card

![WebUI Bands](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_bands.jpg)

The **Active Bands** card presents a checkbox for each of the 12 supported WSPR bands:

| Band | Dial frequency (all regions except 60 m) |
|---|---|
| 2200 m | 137.600 kHz |
| 630 m | 475.700 kHz |
| 160 m | 1,838.100 kHz |
| 80 m | 3,570.100 kHz |
| 60 m | Region-dependent (see IARU Region card) |
| 40 m | 7,040.100 kHz |
| 30 m | 10,140.200 kHz |
| 20 m | 14,097.100 kHz |
| 17 m | 18,106.100 kHz |
| 15 m | 21,096.100 kHz |
| 12 m | 24,926.100 kHz |
| 10 m | 28,126.100 kHz |

Multiple bands can be enabled simultaneously. When frequency hopping is active, the firmware cycles through all enabled bands in order. When hopping is disabled, the firmware transmits only on the first enabled band.

---

#### IARU Region card

![WebUI IARU](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_iaru.jpg)

The **IARU Region & 60 m Frequency** card selects the ITU/IARU administrative region. This affects only the 60 m band dial frequency, which differs between regions due to national spectrum allocations:

| Region | Coverage | 60 m WSPR dial frequency |
|---|---|---|
| **Region 1** | Europe, Africa, Middle East | 5,288.600 kHz |
| **Region 2** | Americas (North, South, Caribbean) | 5,346.500 kHz |
| **Region 3** | Asia, Pacific, Oceania | 5,367.000 kHz |

All other bands use identical dial frequencies worldwide. Always verify that the 60 m frequency is permitted by your national licence authority before transmitting.

---

#### Frequency Hopping card

![WebUI Hop](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_hop.jpg)

The **Frequency Hopping** card enables automatic band rotation:

- **Enable hopping toggle** — When enabled, the transmitter advances to the next enabled band after each hop interval expires.
- **Interval (seconds)** — Minimum 120 seconds (one full WSPR transmission slot). Setting a shorter interval is silently clamped to 120 s.

Example: with 40 m, 20 m, and 15 m enabled and a 240-second hop interval, the sequence is: 40 m → 20 m → 15 m → 40 m → … , with one transmission every 240 s per band.

Hopping requires at least 2 enabled bands to be meaningful. The current band is always visible in the status panel.

---

#### TX Duty Cycle card

![WebUI Duty](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_duty.jpg)

The **TX Duty Cycle** card controls what fraction of available 2-minute WSPR slots are actually used for transmission:

- **0%** — Never transmit (effectively disables the transmitter without unchecking bands)
- **20%** — WSPR community standard; transmits approximately 1 in every 5 slots
- **100%** — Transmit every available slot

The firmware uses a random number generator to decide independently for each slot whether to transmit, so the long-term average approaches the configured percentage without creating fixed patterns.

---

#### TX Control card & Status panel

![WebUI Status](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_status.jpg)

The **TX Control** card contains a single toggle button (**Enable TX** / **Stop TX**) that immediately enables or disables transmission. This is equivalent to changing `tx_enabled` in the configuration and saves to NVS.

The live **Status panel** (updated every 2 seconds) shows:

| Field | Description |
|---|---|
| **RF Hardware** | Detected oscillator (Si5351A, AD9850, or None/DUMMY) |
| **Time synchronization** | UTC time or "Not synchronized" |
| **Current band** | Active WSPR band name |
| **Frequency** | Exact dial frequency in kHz |
| **Next TX** | Countdown in seconds to next transmission slot |
| **Active TX** | Whether a transmission is currently in progress |
| **Symbol** | Current symbol index (0–161) during transmission |
| **Boot time** | UTC timestamp of last reboot |
| **Reset cause** | Hardware reason for last reboot (Power-on, Watchdog, etc.) |

A **WSPRnet** link at the bottom of the page opens the spot map for your callsign directly.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

### REST API

The web server exposes the following endpoints:

| Method | URI | Description |
|---|---|---|
| `GET` | `/` | Returns the full SPA HTML (single 30–40 kB response) |
| `GET` | `/api/config` | Returns current `wspr_config_t` as JSON |
| `POST` | `/api/config` | Updates config from JSON body; saves to NVS |
| `POST` | `/api/tx_toggle` | Toggles `tx_enabled`; saves to NVS |
| `GET` | `/api/status` | Returns live status snapshot as JSON |
| `GET` | `/api/wifi_scan` | Triggers Wi-Fi scan; returns JSON array of APs |
| `POST` | `/api/reset` | Schedules `esp_restart()` |

#### Example: GET /api/status response

```json
{
  "time_ok": true,
  "time": "2026-03-01 14:22:00 UTC",
  "band": "20m",
  "freq": "14097.100 kHz",
  "next_tx_sec": 47,
  "tx_active": false,
  "tx_enabled": true,
  "symbol": 0,
  "hw_ok": true,
  "hw_name": "Si5351A",
  "ip": "192.168.1.42",
  "boot_time": "2026-03-01 08:00:01 UTC",
  "reset_reason": "Power-on"
}
```

#### Example: POST /api/config body

```json
{
  "callsign": "LU1ABC",
  "locator": "GF05",
  "power_dbm": 23,
  "wifi_ssid": "MyNetwork",
  "wifi_pass": "secret",
  "ntp_server": "pool.ntp.org",
  "band_enabled": [false, false, false, false, false, true, false, true, false, false, false, true],
  "hop_enabled": true,
  "hop_interval_sec": 240,
  "tx_enabled": true,
  "tx_duty_pct": 20,
  "xtal_cal_ppb": 0,
  "iaru_region": 2
}
```

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Configuration Reference

### Default values (applied when no NVS config is found)

| Parameter | Default | Notes |
|---|---|---|
| Callsign | `CONFIG_WSPR_DEFAULT_CALLSIGN` | Set in menuconfig |
| Locator | `CONFIG_WSPR_DEFAULT_LOCATOR` | 4-char, set in menuconfig |
| Power | `CONFIG_WSPR_DEFAULT_POWER_DBM` | Set in menuconfig |
| NTP server | `pool.ntp.org` | — |
| Bands enabled | 40 m, 20 m | — |
| TX enabled | `false` | Must be manually enabled after first boot |
| Hop enabled | `false` | — |
| Hop interval | 120 s | — |
| TX duty cycle | 20% | WSPR standard |
| XTAL cal | 0 ppb | No correction |
| IARU region | 1 (Europe) | — |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Building & Flashing

### Prerequisites

- **ESP-IDF v5.0** or later ([Installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- Python 3.8+
- CMake 3.16+
- A serial port with the ESP32 connected

### Clone and configure

```bash
git clone https://github.com/hiperiondev/ESP32_WSPR.git
cd ESP32_WSPR
idf.py set-target esp32
idf.py menuconfig
```

### Configure (menuconfig)

Run `idf.py menuconfig` and navigate to **WSPR Transmitter Configuration** to set all parameters (see [Menuconfig Options](#menuconfig-options-kconfig) below). At minimum, set your callsign, locator, and GPIO pin assignments for your hardware.

### Build

```bash
idf.py build
```

### Flash and monitor

```bash
idf.py flash monitor
```

Or flash and monitor separately:

```bash
idf.py flash
idf.py monitor
```

Use `Ctrl+]` to exit the monitor.

### Partition table

The default ESP-IDF partition table includes a `nvs` partition large enough for the `wspr_config_t` blob. No custom partition table is needed.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Menuconfig Options (Kconfig)

All build-time parameters are exposed through `Kconfig.projbuild` under the menu **"WSPR Transmitter Configuration"**.

### Station defaults

| Option | Symbol | Description |
|---|---|---|
| Default callsign | `CONFIG_WSPR_DEFAULT_CALLSIGN` | Compiled-in default; overridden by NVS |
| Default locator | `CONFIG_WSPR_DEFAULT_LOCATOR` | 4-char Maidenhead; overridden by NVS |
| Default power (dBm) | `CONFIG_WSPR_DEFAULT_POWER_DBM` | Integer; overridden by NVS |

### Oscillator — Si5351A

| Option | Symbol | Default |
|---|---|---|
| I2C port number | `CONFIG_SI5351_I2C_PORT` | 0 |
| SDA GPIO | `CONFIG_SI5351_SDA_GPIO` | 21 |
| SCL GPIO | `CONFIG_SI5351_SCL_GPIO` | 22 |
| Crystal frequency (Hz) | `CONFIG_SI5351_XTAL_FREQ` | 25000000 |
| Output drive current | `CONFIG_SI5351_DRIVE_*MA` | 8 mA |

### Oscillator — AD9850

| Option | Symbol | Default |
|---|---|---|
| CLK GPIO | `CONFIG_AD9850_CLK_GPIO` | 18 |
| FQ_UD GPIO | `CONFIG_AD9850_FQ_UD_GPIO` | 19 |
| DATA GPIO | `CONFIG_AD9850_DATA_GPIO` | 23 |
| RESET GPIO | `CONFIG_AD9850_RESET_GPIO` | 5 |
| Reference clock (Hz) | `CONFIG_AD9850_REF_CLOCK` | 125000000 |

### Low-pass filter GPIO

| Option | Symbol | Default |
|---|---|---|
| Filter address bit 0 (LSB) | `CONFIG_FILTER_GPIO_A` | 25 |
| Filter address bit 1 | `CONFIG_FILTER_GPIO_B` | 26 |
| Filter address bit 2 (MSB) | `CONFIG_FILTER_GPIO_C` | 27 |

A `static_assert` in `gpio_filter.c` enforces that no two filter GPIOs share the same pin number.

### Wi-Fi access point (fallback mode)

| Option | Symbol | Default |
|---|---|---|
| AP SSID | `CONFIG_WSPR_AP_SSID` | `ESP32_WSPR` |
| AP password | `CONFIG_WSPR_AP_PASS` | (empty = open) |

### Time synchronisation

| Option | Symbol | Description |
|---|---|---|
| NTP mode | `CONFIG_WSPR_TIME_NTP` | Use SNTP (Wi-Fi required) |
| GPS mode | `CONFIG_WSPR_TIME_GPS` | Use NMEA UART GPS receiver |

#### GPS-specific options (if GPS mode selected)

| Option | Symbol | Default |
|---|---|---|
| UART port | `CONFIG_GPS_UART_PORT` | 2 |
| RX GPIO | `CONFIG_GPS_RX_GPIO` | 16 |
| TX GPIO | `CONFIG_GPS_TX_GPIO` | 17 |
| Baud rate | `CONFIG_GPS_BAUD_RATE` | 9600 |

### Web UI language

| Option | Symbol |
|---|---|
| English | `CONFIG_WEBUI_LANG_EN` |
| Spanish | `CONFIG_WEBUI_LANG_ES` |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Low-Pass Filter Bank

A low-pass filter (LPF) is **legally required** in virtually all jurisdictions to suppress harmonics before connecting the oscillator output to an antenna. The Si5351A output is a square wave rich in odd harmonics; without filtering, the third harmonic of a 7 MHz signal would appear at 21 MHz, and so on.

The firmware drives a 3-bit binary address bus (GPIO_A = bit 0, GPIO_B = bit 1, GPIO_C = bit 2) that selects one of up to eight filter positions via a BCD decoder (e.g. 74HC138) or a relay-driver board. The mapping from band to filter ID is defined in `BAND_FILTER[]` in `config.c`:

| Filter ID | GPIO C | GPIO B | GPIO A | Bands served |
|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 2200 m, 630 m |
| 1 | 0 | 0 | 1 | 160 m |
| 2 | 0 | 1 | 0 | 80 m, 60 m |
| 3 | 0 | 1 | 1 | 40 m |
| 4 | 1 | 0 | 0 | 30 m |
| 5 | 1 | 0 | 1 | 20 m, 17 m |
| 6 | 1 | 1 | 0 | 15 m, 12 m |
| 7 | 1 | 1 | 1 | 10 m |

After `gpio_filter_select()` is called, the firmware inserts a **10 ms `vTaskDelay()`** before enabling the oscillator output to allow relay contacts to physically settle. This prevents key-clicks and out-of-band emissions during the relay transition.

You can modify the `BAND_FILTER[]` table freely to match your physical LPF hardware layout without touching any other code.

### Recommended LPF design

For each filter, design a **T-network low-pass filter** (rather than a Pi network) for better efficiency. QRP Labs, W3NQN, and numerous online LC filter calculators can generate component values for each cutoff frequency. Typical cutoff frequencies:

| Filter | Bands | Cutoff frequency |
|---|---|---|
| LPF-0 | 2200 m / 630 m | 600 kHz |
| LPF-1 | 160 m | 2.5 MHz |
| LPF-2 | 80 m / 60 m | 6 MHz |
| LPF-3 | 40 m | 8 MHz |
| LPF-4 | 30 m | 12 MHz |
| LPF-5 | 20 m / 17 m | 20 MHz |
| LPF-6 | 15 m / 12 m | 26 MHz |
| LPF-7 | 10 m | 32 MHz |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Oscillator Hardware

### Si5351A (preferred)

The **Si5351A** is a highly versatile I2C-programmable clock generator manufactured by Silicon Laboratories (now Skyworks). It is the preferred oscillator for WSPR beacons because:

- It supports **fractional-N PLL synthesis**, allowing sub-Hz frequency resolution across the entire HF spectrum.
- It is I2C-detectable (ACK probe), so the firmware can confirm its presence at boot.
- It is available on inexpensive breakout boards from Adafruit, QRP Labs, and numerous Chinese suppliers.
- Output power is adequate for direct antenna use with a suitable LPF (~10 dBm = 10 mW).

**Architecture summary (from the Si5351A datasheet):**

The Si5351A consists of a reference oscillator (25 or 27 MHz crystal), two PLLs (PLLA and PLLB) that multiply the crystal to 600–900 MHz, and up to 3 MultiSynth output dividers (CLK0–CLK2). This firmware uses only CLK0 locked to PLLA.

The PLL multiplier `a` is computed as `900 MHz / f_xtal_MHz` (rounded to give VCO ≥ 600 MHz), and the output divider is computed as `VCO_Hz / f_out_Hz` with a fractional numerator/denominator pair (p2, p3) that provides the milli-Hz frequency resolution required for WSPR symbol spacing.

The `R` post-divider is applied for output frequencies below 500 kHz (2200 m and 630 m bands).

### AD9850 DDS

The **AD9850** is a Direct Digital Synthesizer (DDS) IC from Analog Devices, operating with a 125 MHz reference clock. It uses a 32-bit frequency tuning word (FTW) to set the output frequency: `FTW = f_out × 2^32 / f_ref`. The firmware computes this in integer arithmetic using a pre-scaled constant `AD9850_SCALE_KHZ = 2^32 / (f_ref / 1000)` to avoid 64-bit overflow.

Because the AD9850 bus is write-only (no readback), the firmware cannot detect its presence and **always assumes it is present** after the Si5351 probe fails. The output is a 10-bit DAC reconstructed sine wave, much lower amplitude than the Si5351 square wave (~1 V p-p into 50 Ω). An external power amplifier is highly recommended with the AD9850.

The AD9850 bit-bang transfer is protected by a FreeRTOS `portMUX` critical section to prevent interruption by the second ESP32 core mid-transfer.

### Dummy mode

If neither oscillator is found (Si5351 I2C probe fails, AD9850 GPIO init fails), the firmware enters **dummy mode**: all `oscillator_*` calls return `ESP_OK` silently, and the web UI displays a warning. This allows the rest of the firmware (Wi-Fi, WebUI, time sync) to continue operating for debugging purposes.

---

## Time Synchronisation

Accurate UTC time is **essential** for WSPR. Transmissions that start more than ±1–2 seconds off the even-minute boundary will not be decoded. The firmware supports two time sources, selected at compile time via Kconfig.

### NTP mode (`CONFIG_WSPR_TIME_NTP`)

Uses the ESP-IDF SNTP client (`esp_sntp`) to periodically query an NTP server. The SNTP callback sets `_synced = true` and the firmware's `scheduler_task` unblocks. Typical accuracy: 1–50 ms, which is more than sufficient for WSPR.

The NTP server hostname is configurable per-instance from the web UI (default: `pool.ntp.org`). Changes are saved to NVS and applied at next boot.

### GPS mode (`CONFIG_WSPR_TIME_GPS`)

A background FreeRTOS task reads NMEA-0183 sentences from a UART-connected GPS module. It accepts `$GPRMC` and `$GNRMC` sentences (most GPS modules output at least one of these). Each sentence is validated with a full NMEA XOR checksum before parsing. The `TZ` environment variable is forced to `"UTC0"` before any `mktime()` call to prevent local timezone offset being applied to NMEA UTC timestamps.

Typical accuracy: ±1 s (limited by the 1 Hz NMEA sentence rate and UART latency). For higher accuracy, a PPS (pulse-per-second) input could be added in a future revision.

GPS mode is independent of Wi-Fi, making it suitable for portable or remote beacon installations without internet access.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Wi-Fi & Networking

### STA mode (station)

If a Wi-Fi SSID is configured, the firmware attempts to associate with the access point in STA mode. Up to 5 connection retries are made within a 15-second window. On success:
- The firmware proceeds with NTP sync (if configured)
- The web interface becomes accessible at the DHCP-assigned IP
- The IP is shown in the serial console log and in `/api/status`

### AP fallback mode

If no SSID is configured, or if all STA connection attempts fail, the firmware starts a soft access point:
- SSID: `CONFIG_WSPR_AP_SSID` (default `ESP32_WSPR`)
- Password: `CONFIG_WSPR_AP_PASS` (if ≥ 8 characters, uses WPA2-PSK; otherwise open network)
- IP: `192.168.4.1`
- Up to 4 simultaneous clients

### Background reconnect

When in AP-only mode with saved STA credentials, a periodic `esp_timer` fires every **5 minutes** and reattempts the STA connection (transitions to APSTA mode, calls `esp_wifi_connect()`). This allows the beacon to recover automatically when the home router comes back online after a power outage.

### Wi-Fi scan

The `GET /api/wifi_scan` endpoint triggers a blocking scan (~2 s) that returns a JSON array of nearby access points with SSID, RSSI, and security type. In AP-only mode, the scan temporarily elevates to APSTA mode, waits 150 ms for the STA interface to initialise, scans, then returns to AP mode. Results are capped at 20 entries for HTTP payload efficiency.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## WSPR Band Frequencies & IARU Regions

The firmware stores dial frequencies for all 12 bands and all 3 IARU regions in the table `BAND_FREQ_HZ[3][BAND_COUNT]` in `config.c`. The inline function `config_band_freq_hz(region, band)` selects the correct entry.

All frequencies are **dial frequencies** in Hz. The actual transmitted RF frequency is `dial_freq + 1500 Hz + symbol_offset` for each symbol.

### Full frequency table

| Band | Region 1 (EU/Africa) | Region 2 (Americas) | Region 3 (Asia/Pacific) |
|---|---|---|---|
| 2200 m | 137,600 Hz | 137,600 Hz | 137,600 Hz |
| 630 m | 475,700 Hz | 475,700 Hz | 475,700 Hz |
| 160 m | 1,838,100 Hz | 1,838,100 Hz | 1,838,100 Hz |
| 80 m | 3,570,100 Hz | 3,570,100 Hz | 3,570,100 Hz |
| **60 m** | **5,288,600 Hz** | **5,346,500 Hz** | **5,367,000 Hz** |
| 40 m | 7,040,100 Hz | 7,040,100 Hz | 7,040,100 Hz |
| 30 m | 10,140,200 Hz | 10,140,200 Hz | 10,140,200 Hz |
| 20 m | 14,097,100 Hz | 14,097,100 Hz | 14,097,100 Hz |
| 17 m | 18,106,100 Hz | 18,106,100 Hz | 18,106,100 Hz |
| 15 m | 21,096,100 Hz | 21,096,100 Hz | 21,096,100 Hz |
| 12 m | 24,926,100 Hz | 24,926,100 Hz | 24,926,100 Hz |
| 10 m | 28,126,100 Hz | 28,126,100 Hz | 28,126,100 Hz |

**60 m note:** The 60 m WSPR frequency differs between IARU regions due to different national channelisation plans. Region 2 (Americas) uses 5,346.5 kHz per FCC/ARRL coordination. Region 3 uses 5,367.0 kHz per WIA/JARL coordination. Region 1 uses 5,288.6 kHz. Always verify that operation on 60 m is permitted under your national amateur radio licence.

### Selecting your IARU region

Set the region in the **IARU Region** card of the web UI. The selection is saved to NVS. The `iaru_region` field in `wspr_config_t` stores values 1, 2, or 3; an invalid stored value is automatically corrected to 1 at boot.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Frequency Hopping Mode

When `hop_enabled = true`, the `scheduler_task` rotates through the list of enabled bands after each `hop_interval_sec` seconds. The hop state (current band index, time-on-band counter) is maintained in task-local variables and resets on each firmware boot.

**Hop logic:**
1. After a transmission completes, check if `time_on_band >= hop_interval_sec`.
2. If yes, advance the band index to the next enabled band (wrapping around).
3. Call `gpio_filter_select(BAND_FILTER[new_band])`.
4. Update the status cache with the new band name.

If only one band is enabled, hopping is effectively disabled (the single band is always "the next" band). The minimum hop interval is enforced in the UI hint as 120 seconds (one TX slot), though the firmware does not hard-clamp values shorter than this.

---

## TX Duty Cycle

The WSPR protocol recommends that stations transmit no more than 20% of the time, leaving the other 80% for receiving. This is both a good operating practice and a courtesy to other users who may be listening on the same frequency.

The firmware implements duty cycle as a random slot gate: before each 2-minute slot, a random number in [0, 99] is drawn; if it is less than `tx_duty_pct`, the slot is used for transmission. Otherwise, the slot is skipped and the scheduler waits for the next one.

- `tx_duty_pct = 0`: Never transmit.
- `tx_duty_pct = 20`: Transmit ~20% of slots (WSPR standard).
- `tx_duty_pct = 100`: Transmit every slot.

---

## Crystal Calibration

Even high-quality crystals deviate from their nominal frequency by tens to hundreds of parts per million. For WSPR, the transmit frequency must be within ±200 Hz of the dial frequency (the receiver window is only about ±100 Hz), so calibration is important.

The `xtal_cal_ppb` field stores the calibration offset in **parts per billion (ppb)**.

### How calibration is applied

**Si5351A:** The ppb correction is applied to the internal VCO frequency before computing the MS0 output divider:
```
vco_cal = vco_nominal + (vco_nominal / 1e9) * cal_ppb
```
A positive `cal_ppb` means the crystal is fast (output frequency would be too high), so the effective VCO is lowered.

**AD9850:** The ppb correction scales the target frequency before computing the 32-bit tuning word:
```
freq_cal = freq_nominal * (1 - cal_ppb / 1e9)
```

### How to measure your crystal offset

1. Set `xtal_cal_ppb = 0` in the WebUI.
2. Transmit on a well-calibrated band (e.g. 40 m or 20 m).
3. Use a calibrated SDR receiver to measure the actual centre frequency of your signal.
4. Compute: `cal_ppb = (measured_Hz - nominal_Hz) * 1e9 / nominal_Hz`
5. If your signal is 50 Hz high on 14,097,100 Hz: `cal_ppb = 50 * 1e9 / 14097100 ≈ 3547 ppb`
6. Enter a **negative** value to lower the output (the crystal is running fast).
7. Save and verify on the next transmission.

Alternatively, WSPRnet reception reports include the frequency offset in Hz measured by the receiving station; this can be used directly to estimate the calibration error.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Implementation Status

| Feature | Status |
|---|---|
| WSPR Type-1, 2 and 3 encoder (callsign + locator + power) | ✅ Complete |
| Si5351A oscillator driver (I2C, auto-detect) | ✅ Complete |
| AD9850 DDS oscillator driver (GPIO bit-bang) | ✅ Complete |
| Oscillator dummy mode (no hardware) | ✅ Complete |
| 3-bit GPIO LPF bank driver | ✅ Complete |
| NTP time synchronisation (SNTP) | ✅ Complete |
| GPS time synchronisation (NMEA UART) | ✅ Complete |
| Wi-Fi STA mode | ✅ Complete |
| Wi-Fi AP fallback (192.168.4.1) | ✅ Complete |
| Background Wi-Fi reconnect timer | ✅ Complete |
| NVS persistent configuration | ✅ Complete |
| Embedded SPA web interface | ✅ Complete |
| REST API (config, status, tx_toggle, reset, scan) | ✅ Complete |
| IARU Region selection for 60 m | ✅ Complete |
| 12-band support (2200 m – 10 m) | ✅ Complete |
| Frequency hopping | ✅ Complete |
| TX duty cycle | ✅ Complete |
| Crystal calibration (ppb) | ✅ Complete |
| English and Spanish WebUI | ✅ Complete |
| WSPRnet spot link | ✅ Complete |
| Reboot info (boot time, reset cause) in status | ✅ Complete |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Roadmap

Planned features and improvements for future releases:

- [ ] **OTA firmware update** — over-the-air firmware upgrade from the WebUI
- [ ] **GPS PPS input** — sub-millisecond time accuracy using PPS signal
- [ ] **Power amplifier enable GPIO** — switch a PA on/off around transmissions
- [ ] **Transmission schedule** — time-of-day or band/day-of-week scheduling rules
- [ ] **MQTT telemetry** — publish status to an MQTT broker for remote monitoring
- [ ] **Additional UI languages**
- [ ] **WSPRnet automatic upload** — direct HTTP upload of received spots (requires SDR RX)
- [ ] **6 m and 4 m band support** — extend frequency table for VHF WSPR
- [ ] **SPI display support** — optional OLED/TFT status display

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement". Don't forget to give the project a star! Thanks again!

1. Fork it (<https://github.com/hiperiondev/ESP32_WSPR/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

### Code style

- C99, ESP-IDF coding conventions.
- All new modules must have a corresponding `module.h` with Doxygen-style API documentation (see `gpio_filter.h`, `oscillator.h` for examples).
- No floating-point nor 64bits arithmetic.
- New Kconfig options must be documented in this README.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## License

Distributed under the **GNU General Public License v3.0**. See `LICENSE.txt` for more information.

```
Copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
```

The WSPR encoding algorithm is based on the original WSJT-X source code by Joe Taylor (K1JT) and the protocol description by Andy Talbot (G4JNT). All algorithms are used with respect for the original authors' contributions to amateur radio.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>

---

## Contact

*Emiliano Augusto Gonzalez — [egonzalez.hiperion@gmail.com](mailto:egonzalez.hiperion@gmail.com)*

Project Link: [https://github.com/hiperiondev/ESP32\_WSPR](https://github.com/hiperiondev/ESP32_WSPR)

---

## References

### WSPR Protocol

- **G4JNT (Andy Talbot)** — *"The WSPR Coding Process"* (2009): the definitive non-normative specification of the WSPR Type-1 encoding algorithm. [PDF](http://www.g4jnt.com/WSPR_Coding_Process.pdf)
- **K1JT (Joe Taylor)** — WSJT-X source code and documentation. [wsjt.sourceforge.io](https://wsjt.sourceforge.io/)
- **WSPRnet** — Global WSPR reception database and maps. [wsprnet.org](https://www.wsprnet.org)
- **Wikipedia** — WSPR (amateur radio software). [en.wikipedia.org/wiki/WSPR](https://en.wikipedia.org/wiki/WSPR_(amateur_radio_software))
- **Scott Harden (AJ4VD)** — WSPR protocol notes with Markdown rendering of the G4JNT paper. [swharden.com](https://swharden.com/software/FSKview/wspr/)
- **WSPR frequency list** — Official WSPRnet frequency coordination. [wsprnet.org/drupal/node/218](https://www.wsprnet.org/drupal/node/218)

### Oscillator hardware

- **Silicon Laboratories / Skyworks** — Si5351A/B/C-B Datasheet. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/Si5351-B.pdf)
- **Skyworks** — AN619: Manually Generating an Si5351 Register Map. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)
- **Skyworks** — AN1234: Manually Generating a Si5351 Register Map for 16QFN Devices. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/an1234-si5351-16qfn-register-map.pdf)
- **QRP Labs** — Si5351A demo code and synthesis theory. [qrp-labs.com](https://qrp-labs.com/synth/si5351ademo.html)
- **NT7S (Jason Milldrum)** — Si5351Arduino library: comprehensive Arduino Si5351 driver. [github.com/etherkit/Si5351Arduino](https://github.com/etherkit/Si5351Arduino)
- **Analog Devices** — AD9850 CMOS Complete DDS Synthesizer Datasheet. [analog.com](https://www.analog.com/media/en/technical-documentation/data-sheets/ad9850.pdf)

### Related ESP32 WSPR projects

- **danak6jq/ESP32-WSPR** — Complete stand-alone WSPR2 transmitter using ESP32 + Si5351 (ESP-IDF v3). [github.com](https://github.com/danak6jq/ESP32-WSPR)
- **mm5agm/WSPR-Multi-Band** — ESP32 multi-band WSPR beacon (Arduino). [github.com](https://github.com/mm5agm/WSPR-Multi-Band)
- **etherkit/JTEncode** — JT65/JT9/JT4/WSPR/FSQ encoder library for Arduino. [github.com](https://github.com/etherkit/JTEncode)

### ESP-IDF documentation

- **Espressif ESP-IDF Programming Guide**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- **ESP-IDF SNTP API**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html)
- **ESP-IDF HTTP Server**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html)
- **ESP-IDF Wi-Fi Driver**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- **ESP-IDF NVS Flash**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html)

---

<div align="center">

*73 de LU3VEA — Happy DXing!*

</div>

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="backtotop" width="30" height="30">
  </a>
</div>
