<a name="readme-top"></a>

<div align="center">

[![Logo](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/logo.jpg)](https://github.com/hiperiondev/ESP32_WSPR)

# Transmisor WSPR

**Transmisor WSPR (Weak Signal Propagation Reporter) para ESP32**

*Un beacon WSPR completo y autónomo construido sobre ESP-IDF — sin Arduino*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Oscillators](https://img.shields.io/badge/Oscillator-Si5351A%20%7C%20AD9850-red.svg)](#hardware-oscilador)

</div>

---

## Tabla de contenidos

1. [Acerca del proyecto](#acerca-del-proyecto)
2. [Descripción del protocolo WSPR](#descripción-del-protocolo-wspr)
3. [Características](#características)
4. [Requisitos de hardware](#requisitos-de-hardware)
5. [Arquitectura y código fuente](#arquitectura-y-código-fuente)
6. [Interfaz web (WebUI)](#interfaz-web-webui)
7. [Referencia de configuración](#referencia-de-configuración)
8. [Compilación y grabación](#compilación-y-grabación)
9. [Opciones de menuconfig (Kconfig)](#opciones-de-menuconfig-kconfig)
10. [Banco de filtros de paso bajo](#banco-de-filtros-de-paso-bajo)
11. [Hardware oscilador](#hardware-oscilador)
12. [Sincronización horaria](#sincronización-horaria)
13. [Wi-Fi y red](#wi-fi-y-red)
14. [Frecuencias de banda WSPR y regiones IARU](#frecuencias-de-banda-wspr-y-regiones-iaru)
15. [Modo de salto de frecuencia](#modo-de-salto-de-frecuencia)
16. [Ciclo de trabajo TX](#ciclo-de-trabajo-tx)
17. [Calibración de cristal](#calibración-de-cristal)
18. [Estado de implementación](#estado-de-implementación)
19. [Hoja de ruta](#hoja-de-ruta)
20. [Contribuciones](#contribuciones)
21. [Licencia](#licencia)
22. [Contacto](#contacto)
23. [Referencias](#referencias)

---

## Acerca del proyecto

**WSPR Transmitter** es un firmware completo y autónomo de beacon WSPR (Weak Signal Propagation Reporter) construido sobre el framework **ESP-IDF** de Espressif para la familia de microcontroladores ESP32. A diferencia de la mayoría de los proyectos WSPR para aficionados que dependen del ecosistema Arduino, este firmware está escrito en C puro sobre las APIs nativas de ESP-IDF, dándole acceso a la gestión de tareas FreeRTOS, el cliente SNTP nativo, la pila `esp_wifi`, almacenamiento persistente `nvs_flash` y el servidor web `esp_http_server` — todo sin la sobrecarga de la capa de abstracción Arduino.

El proyecto está diseñado para operación desatendida como beacon: codifica mensajes WSPR Tipo 1, 2 y 3 completamente en el chip, controla un oscilador RF (Si5351A o AD9850) con resolución sub-Hz por símbolo, selecciona automáticamente el filtro de paso bajo correcto mediante un bus GPIO de 3 bits, sincroniza la hora vía NTP o GPS (con PPS opcional), y expone una aplicación web de página única responsiva para configuración y monitoreo. Todos los ajustes del usuario persisten en la partición flash NVS del ESP32 y sobreviven a cortes de energía.

El modo WSPR ocupa aproximadamente 6 Hz de ancho de banda RF y puede decodificarse con relaciones señal/ruido tan bajas como −28 dB en un ancho de banda de referencia de 2,5 kHz. Una vez transmitidos, los reportes de recepción de estaciones WSPR automatizadas en todo el mundo se cargan automáticamente a [WSPRnet](https://www.wsprnet.org).

### Decisiones clave de diseño

- **Solo ESP-IDF** — sin Arduino, sin bibliotecas I2C de terceros. Todos los drivers del oscilador están escritos desde cero sobre las APIs `driver/i2c_master.h` y `driver/gpio.h` de IDF.
- **Soporte de oscilador dual** — el firmware autodetecta un Si5351A al arrancar mediante sondeo ACK por I2C, luego cae al AD9850 (serial bit-bang GPIO de solo escritura), y finalmente a un modo ficticio silencioso si ninguno está presente.
- **Aritmética solo entera de 32 bits** — el codificador WSPR completo, el cálculo de divisores PLL del Si5351 y el cómputo de la palabra de sintonización del AD9850 usan únicamente matemáticas enteras de 32 bits. Sin punto flotante, sin `double`.
- **SPA en un solo archivo embebido** — la interfaz web se compila en el firmware como un archivo de cabecera C; no se necesita componente de sistema de archivos ni partición SPIFFS/LittleFS.
- **WebUI multilingüe** — las tablas de cadenas en inglés y español se proveen como cabeceras separadas (`webui_en.h` / `webui_es.h`) seleccionadas en tiempo de compilación vía Kconfig.
- **Degradación elegante** — el hardware de oscilador faltante, la pérdida de Wi-Fi, NTP no disponible y los blobs NVS corruptos se manejan sin fallos.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Descripción del protocolo WSPR

WSPR (**W**eak **S**ignal **P**ropagation **R**eporter, pronunciado *"susurro"* en inglés) es un protocolo de radio digital para radioaficionados diseñado por Joe Taylor (K1JT), Premio Nobel de Física, y lanzado originalmente en 2008.

### Qué transmite WSPR

Un mensaje WSPR Tipo 1 codifica exactamente tres piezas de información:

| Campo | Descripción | Ancho de codificación |
|---|---|---|
| **Indicativo** | Identificador de estación (hasta 6 caracteres alfanuméricos) | 28 bits |
| **Localizador Maidenhead** | Cuadrícula de 4 caracteres (p. ej. `GF05`) | 15 bits |
| **Potencia TX** | Potencia de transmisión en dBm (uno de 19 niveles válidos) | 7 bits |

Carga útil total: **50 bits**.

### Tipos de mensaje

| Tipo | Formato de indicativo | Precisión del localizador | ¿Acompañante? |
|---|---|---|---|
| **1** | Simple (≤ 6 chars, sin `/`) | Cuadrícula de 4 chars (DDLL) | No |
| **2** | Compuesto (contiene `/`, p. ej. `PJ4/K1ABC`) | Ninguno en este trama | Sí (Tipo-3) |
| **3** | Hash de indicativo de 15 bits | Sub-cuadrícula de 6 chars (DDLLSS) | Acompañante de Tipo-1 o Tipo-2 |

Cuando se configura un indicativo compuesto, el planificador alterna entre Tipo-2 (paridad=0) y Tipo-3 (paridad=1) en slots sucesivos de minuto par. Cuando se configura un indicativo simple con localizador de 6 caracteres, alterna entre Tipo-1 (localizador de 4 chars, paridad=0) y Tipo-3 (localizador completo de 6 chars, paridad=1).

### Proceso de codificación (especificación G4JNT / K1JT)

El siguiente proceso está implementado en `wspr_encode.c`:

```
Entrada: indicativo + localizador + potencia_dBm
        │
        ▼
1. Empaquetar indicativo → entero de 28 bits (fórmula estándar G4JNT)
   - Rellenar a 6 chars; anteponer espacio si char[2] no es dígito
   - Chars 0-1: alfabeto de 37 símbolos (0-9=0..9, A-Z=10..35, espacio=36)
   - Char 2:    solo dígito (0-9)
   - Chars 3-5: alfabeto sufijo de 27 símbolos (A-Z=0..25, espacio=26)
        │
        ▼
2. Empaquetar localizador → entero de 15 bits
   - Fórmula: (179 - 10*(c0-'A') - d0) * 180 + (10*(c1-'A') + d1)
   - Para localizador de 6 chars, solo los primeros 4 chars se usan en el trama Tipo-1
        │
        ▼
3. Empaquetar potencia → entero de 7 bits  (potencia_dBm + 64)
   - Redondeado al nivel WSPR válido más cercano
        │
        ▼
4. Ensamblar mensaje de 50 bits en 7 bytes (justificado a la izquierda, 6 bits de relleno)
        │
        ▼
5. Codificación convolucional (K=32, tasa 1/2)
   - Polinomios: G1=0xF2D05351, G2=0xE4613C47
   - Entrada: 50 bits de datos + 31 bits de cola = 81 bits → 162 bits de salida
        │
        ▼
6. Intercalado por inversión de bits (permutación de inversión de bits de 256 puntos)
        │
        ▼
7. Combinar con el vector de sincronía de 162 bits:
   símbolo[i] = 2 * bit_intercalado[i] + sync[i]    → valores 0..3
        │
        ▼
Salida: 162 símbolos 4-FSK (valores 0, 1, 2, 3)
```

### Características RF

| Parámetro | Valor |
|---|---|
| Modulación | 4-FSK (4 tonos), fase continua |
| Designador de emisión | F1D |
| Espaciado de tonos | 12000 / 8192 Hz ≈ **1,4648 Hz** |
| Período de símbolo | 8192 / 12000 s ≈ **682,667 ms** |
| Duración total TX | 162 × 682,667 ms ≈ **110,6 segundos** |
| Ancho de banda ocupado | ~6 Hz (3 × espaciado de tonos) |
| Desplazamiento de audio | **+1500 Hz** sobre la frecuencia de marcación |
| SNR mínimo decodificable | −28 dB en 2,5 kHz de ancho de banda |

### Temporización

Las transmisiones WSPR **deben** comenzar en el segundo 1 de cada minuto UTC par (00:00:01, 00:02:01, …). El planificador pre-arma el oscilador y el relé LPF en el segundo 0 (`:00`), luego gira con temporización de resolución en microsegundos para alinear el primer símbolo dentro de 50 ms del segundo 1 (`:01`). El firmware se niega a transmitir hasta que la sincronización de tiempo (NTP o GPS) haya ocurrido.

### Ciclo de trabajo

El firmware implementa el ciclo de trabajo como un **acumulador determinista**: `acumulador += ciclo_pct` cada slot; se usa el slot cuando el acumulador alcanza o supera 100, momento en que se resta 100. Esto produce una distribución perfectamente uniforme sin aleatoriedad.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Características

- ✅ **Codificador WSPR Tipo 1, 2 y 3 completo** — empaquetado de indicativo, codificación convolucional (K=32, tasa 1/2), intercalado por inversión de bits, superposición del vector de sincronía; aritmética solo entera de 32 bits
- ✅ **Soporte de oscilador dual** — Si5351A (I2C, autodetectado mediante sondeo ACK) y AD9850 (bit-bang GPIO, asumido presente); modo ficticio elegante si ninguno es encontrado
- ✅ **12 bandas WSPR** — de 2200 m a 10 m (137 kHz a 28 MHz)
- ✅ **Selección de región IARU** — Región 1, 2 o 3 para la frecuencia de marcación correcta en 60 m
- ✅ **Selección automática de filtro de paso bajo** — bus GPIO binario de 3 bits, 8 posiciones de filtro, retardo de establecimiento de relé configurable
- ✅ **Sincronía de tiempo vía NTP** (SNTP, servidor seleccionable, cambio inmediato sin reinicio) o **GPS** (NMEA-0183 autodetectado al arranque via UART)
- ✅ **Soporte GPS PPS opcional** — ISR de flanco ascendente pone a cero el componente sub-segundo para temporización con precisión de µs
- ✅ **Modo STA Wi-Fi** con fallback a AP suave (192.168.4.1) y temporizador de reconexión en segundo plano
- ✅ **Salto de frecuencia** — rotación automática por las bandas habilitadas cada N segundos (mín. 120 s = 1 slot TX)
- ✅ **Ciclo de trabajo TX** — 0–100% configurable con selección de slot por acumulador determinista
- ✅ **Calibración de cristal** — corrección ±ppb almacenada en NVS, aplicada a todos los cálculos de frecuencia; diferida durante ventanas TX activas
- ✅ **Modo de tono de prueba** — portadora CW continua a frecuencia especificada por el usuario para calibración
- ✅ **Auto-calibración desde tono** — medir tono recibido vs nominal, calcular y aplicar corrección ppb en un clic
- ✅ **Botón GPS de localizador** — relleno con un clic del localizador Maidenhead desde coordenadas GPS en vivo
- ✅ **Aplicación web de página única embebida** — sin SPIFFS, sin archivos externos; completamente autocontenido
- ✅ **API REST** — endpoints JSON para lectura/escritura de configuración, sondeo de estado, escaneo Wi-Fi, alternancia TX, alternancia de tono, localizador GPS, reinicio del sistema
- ✅ **Autenticación HTTP Básica opcional** — protección por usuario/contraseña para todos los endpoints web
- ✅ **Configuración persistente NVS** — todos los ajustes sobreviven cortes de energía; verificación de versión de esquema (v6) con defaults automáticos ante discrepancia
- ✅ **UI multilingüe** — inglés y español (selección en tiempo de compilación vía Kconfig)
- ✅ **Integración WSPRnet** — enlace directo desde la WebUI al mapa de spots de la estación
- ✅ **Nativo ESP-IDF** — sin dependencia de Arduino

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Requisitos de hardware

### Mínimo

| Componente | Especificación |
|---|---|
| **Microcontrolador** | Cualquier módulo ESP32 (ESP32-WROOM-32, ESP32-DevKitC, ESP32-WROVER, etc.) |
| **Oscilador RF** | Placa breakout Si5351A (I2C) **o** módulo DDS AD9850 (serial bit-bang GPIO) |
| **Antena** | Antena de hilo apropiada para la(s) banda(s) de operación |

### Componentes adicionales recomendados

| Componente | Propósito |
|---|---|
| **Banco de filtros de paso bajo** | Supresión de armónicos (legalmente requerido en la mayoría de jurisdicciones) |
| **Decodificador BCD / driver de relé** | Controlado por 3 líneas GPIO (GPIO_A, GPIO_B, GPIO_C) |
| **Módulo GPS** (NMEA UART) | Para sincronía de tiempo por GPS (alternativa a NTP); funciona sin internet |
| **Amplificador de potencia** | Aumentar la salida más allá de los ~10 dBm del oscilador |
| **Fuente de alimentación 3,3 V** | Suministro estable para el ESP32 y el Si5351 |

### Conexionado del oscilador

#### Si5351A (preferido)

```
ESP32          Placa breakout Si5351A
GPIO_SDA  ──── SDA  (con pull-up de 4,7 kΩ a 3,3 V)
GPIO_SCL  ──── SCL  (con pull-up de 4,7 kΩ a 3,3 V)
3,3 V     ──── VCC
GND       ──── GND
CLK0      ──── Entrada LPF (a través de red de adaptación)
```

- Dirección I2C: `0x60` (fija en la mayoría de las placas breakout)
- Velocidad I2C: 400 kHz (Modo Rápido), usando la nueva API I2C master de ESP-IDF (`driver/i2c_master.h`)
- Cristal: 25 MHz o 27 MHz (configurable vía Kconfig)
- Corriente de salida: 2 mA / 4 mA / 6 mA / 8 mA (configurable vía Kconfig)
- Salida: onda cuadrada, niveles lógicos 3,3 V, ~10 dBm en 50 Ω

#### AD9850 DDS

```
ESP32                   Módulo AD9850
AD9850_CLK_GPIO    ──── W_CLK
AD9850_FQ_UD_GPIO  ──── FQ_UD
AD9850_DATA_GPIO   ──── D7 / DATA (modo serial)
AD9850_RESET_GPIO  ──── RESET
3,3 V / 5 V        ──── VCC  (verificar tensión del módulo)
GND                ──── GND
OUT1               ──── Entrada LPF (salida de onda sinusoidal)
```

- Reloj de referencia: 125 MHz (módulos AD9850 más comunes; configurable vía Kconfig)
- Interfaz: serial bit-bang (LSB primero, 32 bits de palabra de frecuencia + 8 bits de control); protegida por sección crítica FreeRTOS portMUX
- Palabra de sintonización: `FTW = freq_Hz × 2^32 / ref_clk_Hz`
- Salida: onda sinusoidal, ~1 V pico a pico en 50 Ω

### Conexionado del banco de filtros

```
ESP32                Placa decodificador BCD / driver de relé
GPIO_A (bit 0)  ──── Entrada A
GPIO_B (bit 1)  ──── Entrada B
GPIO_C (bit 2)  ──── Entrada C
3,3 V           ──── VCC lógica
GND             ──── GND
```

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Arquitectura y código fuente

### Mapa de módulos

```
main/
├── CMakeLists.txt          — Registro de componente IDF, lista de dependencias
├── Kconfig.projbuild       — Todas las opciones de menuconfig (pines, oscilador, Wi-Fi, etc.)
│
├── main.c                  — app_main(), scheduler_task(), status_task(), wspr_transmit()
│
├── config.c / config.h     — Struct de config persistente, carga/guardado/defaults NVS, tablas de bandas
├── oscillator.c / .h       — API unificada de oscilador; drivers Si5351A + AD9850; autodetección
├── gpio_filter.c / .h      — Driver de banco LPF GPIO de 3 bits
├── time_sync.c / .h        — Autodetección GPS + fallback NTP (tiempo de ejecución)
├── wifi_manager.c / .h     — Wi-Fi STA + fallback AP; reconexión en segundo plano; escaneo Wi-Fi
├── web_server.c / .h       — Servidor HTTP; API REST; caché de estado; mutex de configuración
├── wspr_encode.c / .h      — Codificador WSPR Tipo 1/2/3 completo (solo enteros de 32 bits)
│
├── webui_strings.h         — Cabecera de despacho: incluye webui_en.h o webui_es.h
├── webui_en.h              — Tabla de cadenas UI en inglés
└── webui_es.h              — Tabla de cadenas UI en español
```

### Estructura de tareas (FreeRTOS)

```
app_main()
  │
  ├─ [secuencia de inicialización]
  │    config_init() → config_load()
  │    gpio_filter_init()
  │    oscillator_init() → oscillator_set_cal()
  │    wifi_manager_start()
  │    time_sync_init()         ← puede lanzar gps_task (ver abajo)
  │    web_server_start()
  │
  ├─ xTaskCreate(status_task,    pila=6144, prioridad=3)
  │     — sondea tiempo/estado cada segundo,
  │       llama a web_server_update_status()
  │
  └─ xTaskCreate(scheduler_task, pila=8192, prioridad=5)
        — espera sincronía de tiempo, calcula el próximo slot TX,
          llama a wspr_transmit(), maneja lógica de salto/ciclo

  [si GPS es autodetectado al arrancar:]
  └─ xTaskCreate(gps_task, pila=6144, prioridad=5)
        — lee NMEA desde UART, parsea sentencias RMC/ZDA/GGA,
          actualiza el reloj del sistema via settimeofday(), maneja ISR PPS
```

### Secuencia de transmisión WSPR (`wspr_transmit()`)

```
1. Bloquear mutex de config; capturar indicativo, localizador, potencia, región, paridad
2. Determinar tipo de mensaje: wspr_encode_type()
3. Codificar 162 símbolos:
     - Tipo-1 o Tipo-2 (paridad=0): wspr_encode()
     - Acompañante Tipo-3 (paridad=1): wspr_encode_type3()
     - Paridad alternada SOLO en codificación exitosa
4. Si no está pre-armado (el pre-armado ocurre especulativamente en phase=0):
     a. oscillator_set_freq(base_hz)   ← base_hz = freq. de marcación + 1500 Hz
     b. gpio_filter_select(BAND_FILTER[banda])
     c. vTaskDelay(CONFIG_WSPR_LPF_SETTLE_MS)   ← establecimiento del relé
5. oscillator_tx_begin()              ← diferir cambios de cal durante TX
6. Aplicar offset de tono símbolo-0 ANTES de habilitar salida RF
7. oscillator_enable(true)
8. Para cada uno de los 162 símbolos (i > 0):
     a. oscillator_set_freq_mhz(base_hz, símbolo × 1464844 mHz)
     b. Espera activa hasta el deadline del símbolo (bucle temporizador µs)
9. oscillator_enable(false)
10. oscillator_tx_end()               ← aplicar cualquier actualización de cal diferida
```

> **Pre-armado:** El planificador pre-programa la frecuencia del oscilador y el relé LPF en `phase=0` (segundo `:00` del límite de minuto par) para que no se necesite retardo de establecimiento del relé cuando llega `phase=1`. El estado de pre-armado se rastrea en `g_pre_armed`.

### Estructura de configuración (`wspr_config_t`)

Todos los ajustes se almacenan en un único blob NVS con clave `"cfg"` en el espacio de nombres `"wspr"`. La versión actual del esquema es **6** (`CONFIG_SCHEMA_VERSION`).

```c
typedef struct {
    uint8_t  version;               // Versión del esquema (debe ser igual a CONFIG_SCHEMA_VERSION = 6)
    char     callsign[12];          // Indicativo amateur (simple o compuesto con '/')
    char     locator[7];            // Localizador Maidenhead: 4 o 6 caracteres
    uint8_t  power_dbm;             // Potencia TX en dBm (niveles válidos WSPR)
    char     wifi_ssid[33];         // SSID Wi-Fi
    char     wifi_pass[65];         // Contraseña Wi-Fi
    bool     band_enabled[12];      // Un flag por cada banda WSPR (indexado por wspr_band_t)
    bool     hop_enabled;           // Habilitar salto de frecuencia automático
    uint32_t hop_interval_sec;      // Intervalo de salto en segundos (mín. 120)
    char     ntp_server[64];        // Nombre de host o IP del servidor NTP
    bool     tx_enabled;            // Habilitación/deshabilitación maestra de TX
    uint8_t  tx_duty_pct;           // Ciclo de trabajo TX (0-100 %)
    int32_t  xtal_cal_ppb;          // Desplazamiento de calibración del cristal en ppb
    uint8_t  iaru_region;           // Región IARU: 1, 2 o 3
    // Campos solo de tiempo de ejecución — nunca guardados en NVS, siempre reseteados a 0/false en carga:
    bool     bands_changed;         // Establecido por el servidor web cuando cambia la selección de bandas
    uint8_t  tx_slot_parity;        // Contador de alternancia Tipo 2/3
    bool     tone_active;           // Flag de modo de tono de prueba activo
    float    tone_freq_khz;         // Frecuencia del tono de prueba en kHz
} wspr_config_t;
```

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Interfaz web (WebUI)

La interfaz web es una aplicación de página única (SPA) servida completamente desde la memoria flash como un literal de cadena C embebido en `web_server.c`. No se necesita ningún componente de sistema de archivos. La interfaz actualiza automáticamente los datos de estado cada 2 segundos vía el endpoint `/api/status`.

### Acceso a la WebUI

| Escenario | URL |
|---|---|
| Conectado a la red Wi-Fi doméstica (modo STA) | `http://<IP-asignada>` |
| Sin credenciales Wi-Fi, modo fallback AP | `http://192.168.4.1` |

La dirección IP también se imprime en la consola serie del ESP32 al arrancar.

> **Autenticación HTTP Básica:** Cuando `CONFIG_WSPR_HTTP_AUTH_ENABLE` está activo en menuconfig, todos los endpoints requieren una cabecera `Authorization: Basic <base64(usuario:contraseña)>`. El navegador mostrará su diálogo nativo de credenciales en el primer acceso. Las credenciales se configuran mediante `CONFIG_WSPR_HTTP_AUTH_USER` y `CONFIG_WSPR_HTTP_AUTH_PASS`.

### Tarjeta de Estación

![WebUI Estación](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_station.jpg)

La tarjeta **Estación** configura el contenido del mensaje WSPR:

- **Indicativo** — Tu indicativo de radioaficionado. Los indicativos simples (p. ej. `W1AW`, `LU3VEA`, `G4JNT`) usan hasta 6 caracteres alfanuméricos con un dígito en la posición 2. Los indicativos compuestos que contienen una barra (p. ej. `PJ4/K1ABC`, `K1ABC/P`) activan la alternancia automática de mensajes Tipo 2 + Tipo 3. El codificador maneja automáticamente la normalización G4JNT (antepone un espacio si el carácter 2 no es un dígito, p. ej. `G4JNT` → `[esp]G4JNT`).
- **Localizador Maidenhead** — Cuadrícula de 4 caracteres (p. ej. `GF05`) o sub-cuadrícula de 6 caracteres (p. ej. `GF05ab`). Un localizador de 6 caracteres activa la alternancia automática de mensajes Tipo 1 + Tipo 3, transmitiendo la precisión de sub-cuadrícula a las estaciones receptoras. El **botón GPS** (📌) aparece a la derecha del campo del localizador y se habilita cuando un módulo GPS es autodetectado al arrancar; al hacer clic rellena el localizador desde las coordenadas GPS actuales.
- **Potencia TX (dBm)** — Nivel de potencia de transmisión. Debe ser uno de los 19 niveles WSPR válidos: 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 dBm. Los valores fuera de este conjunto se redondean silenciosamente al nivel válido más cercano.
- **Calibración XTAL (ppb)** — Corrección de frecuencia del cristal en partes por mil millones. Un valor positivo compensa un cristal rápido (baja la frecuencia efectiva de salida). Se aplica inmediatamente a todos los cálculos de frecuencia del oscilador; diferida automáticamente si una TX está en progreso.
- **Tono de prueba recibido (kHz)** — Campo de entrada para la frecuencia del tono de prueba recibido (medida con un SDR o contador de frecuencia). Usado por el botón de **auto-calibración** para calcular y aplicar la corrección en ppb automáticamente.
- **Botón Tono de prueba / Detener tono** — Inicia o detiene una portadora CW continua a la frecuencia introducida en el campo de frecuencia de tono (en kHz). Útil para calibración de frecuencia y pruebas de ROE de antena.

---

### Tarjeta Wi-Fi

![WebUI Wi-Fi](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_wifi.jpg)

- **Botón de escaneo** — Lanza un escaneo de canales Wi-Fi bloqueante (≈2 s) y popula una lista desplegable de puntos de acceso descubiertos con SSID, RSSI y tipo de seguridad.
- **Contraseña** — Frase de acceso WPA/WPA2 (incluye interruptor mostrar/ocultar).
- **Servidor NTP** — Nombre de host o IP del servidor NTP (predeterminado: `pool.ntp.org`). Los cambios se aplican **inmediatamente** vía `time_sync_restart_ntp()` — no se requiere reinicio del dispositivo.
- **Indicación sin credenciales** — Si el campo SSID se deja en blanco, el ESP32 arranca en modo AP suave en `192.168.4.1`.

---

### Tarjeta de Bandas Activas

![WebUI Bandas](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_bands.jpg)

| Banda | Frecuencia de marcación (todas las regiones excepto 60 m) |
|---|---|
| 2200 m | 137,600 kHz |
| 630 m | 475,700 kHz |
| 160 m | 1.838,100 kHz |
| 80 m | 3.570,100 kHz |
| 60 m | Dependiente de la región (ver tarjeta Región IARU) |
| 40 m | 7.040,100 kHz |
| 30 m | 10.140,200 kHz |
| 20 m | 14.097,100 kHz |
| 17 m | 18.106,100 kHz |
| 15 m | 21.096,100 kHz |
| 12 m | 24.926,100 kHz |
| 10 m | 28.126,100 kHz |

Si no hay ninguna banda habilitada, el firmware cae automáticamente a 40 m.

---

### Tarjeta de Región IARU

![WebUI IARU](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_iaru.jpg)

| Región | Cobertura | Frecuencia de marcación WSPR en 60 m |
|---|---|---|
| **Región 1** | Europa, África, Oriente Medio | 5.288,600 kHz |
| **Región 2** | Américas (Norte, Sur, Caribe) | 5.346,500 kHz |
| **Región 3** | Asia, Pacífico, Oceanía | 5.367,000 kHz |

Todas las demás bandas usan frecuencias de marcación idénticas en todo el mundo.

---

### Tarjeta de Salto de Frecuencia

- **Interruptor de habilitación de salto** — Cuando está habilitado, el transmisor avanza a la siguiente banda habilitada después de que expire cada intervalo de salto.
- **Intervalo (segundos)** — Mínimo 120 segundos (un slot de transmisión WSPR completo). Los valores menores a 120 s almacenados en NVS se limitan a 120 s por `config_load()`.

---

### Tarjeta de Ciclo de Trabajo TX

Controla qué fracción de los slots WSPR de 2 minutos disponibles se usa realmente para transmisión. El firmware usa un **acumulador determinista** (no un generador de números aleatorios): `acumulador += tx_duty_pct` cada slot; el slot se usa cuando el acumulador ≥ 100, luego se resta 100.

- **0%** — Nunca transmitir
- **20%** — Estándar de la comunidad WSPR; transmite 1 de cada 5 slots
- **100%** — Transmitir en cada slot disponible

---

### Tarjeta de Control TX y panel de estado

El **panel de estado** en vivo (actualizado cada 2 segundos) muestra:

| Campo JSON | Descripción |
|---|---|
| `hw_name` | Oscilador detectado (`Si5351A`, `AD9850 (assumed)`, o `None (DUMMY)`) |
| `time` | Hora UTC o "Sin sincronizar" |
| `band` | Nombre de la banda WSPR activa |
| `freq` | Frecuencia de marcación exacta en MHz |
| `next_tx_sec` | Cuenta regresiva en segundos al próximo slot de transmisión |
| `tx_active` | Si una transmisión está actualmente en progreso |
| `symbol` | Índice del símbolo actual (0–161) durante la transmisión |
| `boot_time_str` | Marca de tiempo UTC del último reinicio (disponible tras primera sincronía) |
| `reboot_reason` | Razón hardware del último reinicio (Power-on, Software, Watchdog, etc.) |
| `gps_active` | `true` cuando la fuente de tiempo es GPS (autodetectado al arrancar) |
| `tone_active` | `true` cuando el modo de tono de prueba está activo |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

### API REST

| Método | URI | Descripción |
|---|---|---|
| `GET` | `/` | Devuelve el HTML completo de la SPA |
| `GET` | `/api/config` | Devuelve la `wspr_config_t` actual como JSON |
| `POST` | `/api/config` | Actualiza la configuración desde el cuerpo JSON; guarda en NVS |
| `POST` | `/api/tx_toggle` | Alterna `tx_enabled`; devuelve el nuevo estado como JSON |
| `GET` | `/api/status` | Devuelve una instantánea del estado en vivo como JSON |
| `GET` | `/api/wifi_scan` | Lanza escaneo Wi-Fi; devuelve array JSON de APs |
| `GET` | `/api/gps_loc` | Devuelve latitud/longitud GPS actual y localizador Maidenhead de 6 chars |
| `POST` | `/api/tone_toggle` | Inicia o detiene el tono de prueba; cuerpo: `{"active":true,"freq_khz":14097.1}` |
| `POST` | `/api/reset` | Programa `esp_restart()` tras un breve retardo |

#### Ejemplo: respuesta GET /api/status

```json
{
  "time_ok": true,
  "time": "14:22:05 UTC",
  "band": "20m",
  "freq": "14.0971 MHz",
  "next_tx_sec": 47,
  "tx_active": false,
  "tx_enabled": true,
  "symbol": 0,
  "hw_ok": true,
  "hw_name": "Si5351A",
  "ip": "192.168.1.42",
  "boot_time_str": "2026-03-01 08:00 UTC",
  "reboot_reason": "Power-on",
  "gps_active": false,
  "tone_active": false,
  "tone_freq_khz": 0.0
}
```

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Referencia de configuración

### Valores predeterminados

| Parámetro | Predeterminado | Notas |
|---|---|---|
| Indicativo | `N0CALL` | `CONFIG_WSPR_DEFAULT_CALLSIGN` en menuconfig |
| Localizador | `AA00` | `CONFIG_WSPR_DEFAULT_LOCATOR` en menuconfig |
| Potencia | 23 dBm | `CONFIG_WSPR_DEFAULT_POWER_DBM` en menuconfig |
| Servidor NTP | `pool.ntp.org` | — |
| Bandas habilitadas | 40 m, 20 m | — |
| TX habilitada | `false` | Debe habilitarse manualmente tras el primer arranque |
| Salto habilitado | `false` | — |
| Intervalo de salto | 120 s | — |
| Ciclo de trabajo TX | 20% | Estándar WSPR |
| Calibración XTAL | 0 ppb | Sin corrección |
| Región IARU | 1 (Europa) | — |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Compilación y grabación

### Requisitos previos

- **ESP-IDF v5.0** o posterior ([Guía de instalación](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- Python 3.8+, CMake 3.16+
- Puerto serie con el ESP32 conectado

### Clonar y configurar

```bash
git clone https://github.com/hiperiondev/ESP32_WSPR.git
cd ESP32_WSPR
idf.py set-target esp32
idf.py menuconfig
```

### Compilar y grabar

```bash
idf.py build
idf.py flash monitor
```

Usa `Ctrl+]` para salir del monitor.

### Tabla de particiones

La tabla de particiones predeterminada de ESP-IDF incluye una partición `nvs` suficientemente grande para el blob `wspr_config_t` (esquema v6, ~230 bytes). No se necesita tabla de particiones personalizada.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Opciones de menuconfig (Kconfig)

Todos los parámetros de tiempo de compilación están expuestos a través de `Kconfig.projbuild` bajo el menú **"WSPR Transmitter"**.

### Valores predeterminados de estación

| Opción | Símbolo | Predeterminado |
|---|---|---|
| Indicativo predeterminado | `CONFIG_WSPR_DEFAULT_CALLSIGN` | `N0CALL` |
| Localizador predeterminado | `CONFIG_WSPR_DEFAULT_LOCATOR` | `AA00` |
| Potencia predeterminada (dBm) | `CONFIG_WSPR_DEFAULT_POWER_DBM` | `23` |

### Oscilador — Si5351A

| Opción | Símbolo | Predeterminado |
|---|---|---|
| Puerto I2C | `CONFIG_SI5351_I2C_PORT` | 0 |
| GPIO SDA | `CONFIG_SI5351_SDA_GPIO` | 21 |
| GPIO SCL | `CONFIG_SI5351_SCL_GPIO` | 22 |
| Frecuencia de cristal (Hz) | `CONFIG_SI5351_XTAL_FREQ` | 25000000 |
| Corriente de salida | `CONFIG_SI5351_DRIVE_2MA` … `CONFIG_SI5351_DRIVE_8MA` | 8 mA |

### Oscilador — AD9850

| Opción | Símbolo | Predeterminado |
|---|---|---|
| GPIO CLK | `CONFIG_AD9850_CLK_GPIO` | 18 |
| GPIO FQ_UD | `CONFIG_AD9850_FQ_UD_GPIO` | 19 |
| GPIO DATA | `CONFIG_AD9850_DATA_GPIO` | 23 |
| GPIO RESET | `CONFIG_AD9850_RESET_GPIO` | 5 |
| Reloj de referencia (Hz) | `CONFIG_AD9850_REF_CLOCK` | 125000000 |

### Detección de oscilador

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Asumir AD9850 presente | `CONFIG_OSCILLATOR_ASSUME_AD9850` | `y` | Cuando está habilitado, trata al AD9850 como presente si el sondeo del Si5351 falla. Deshabilitar para entrar en modo DUMMY. |

### GPIO del banco de filtros de paso bajo

| Opción | Símbolo | Predeterminado |
|---|---|---|
| Bit de dirección 0 (LSB) | `CONFIG_FILTER_GPIO_A` | 25 |
| Bit de dirección 1 | `CONFIG_FILTER_GPIO_B` | 26 |
| Bit de dirección 2 (MSB) | `CONFIG_FILTER_GPIO_C` | 27 |

### Tiempo de establecimiento del relé LPF

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Tiempo de establecimiento (ms) | `CONFIG_WSPR_LPF_SETTLE_MS` | 10 | Retardo tras escritura GPIO antes de habilitar la salida RF. Rango: 5–100 ms. |

### Punto de acceso Wi-Fi (modo fallback)

| Opción | Símbolo | Predeterminado |
|---|---|---|
| SSID del AP | `CONFIG_WSPR_AP_SSID` | `WSPR-Config` |
| Contraseña del AP | `CONFIG_WSPR_AP_PASS` | (vacío = red abierta) |

### Sincronización horaria (autodetección en tiempo de ejecución)

La selección de la fuente de tiempo es **automática al arrancar** — no se necesita elección en tiempo de compilación. `time_sync_init()` sondea el UART GPS durante `CONFIG_GPS_DETECT_TIMEOUT_MS` milisegundos. Si se recibe una sentencia NMEA válida con checksum correcto, se activa el modo GPS; de lo contrario, el modo NTP arranca automáticamente.

#### Opciones GPS

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Puerto UART GPS | `CONFIG_GPS_UART_PORT` | 1 | Índice de periférico UART del ESP32 |
| GPIO RX | `CONFIG_GPS_RX_GPIO` | 16 | El ESP32 recibe NMEA desde el TX del GPS |
| GPIO TX | `CONFIG_GPS_TX_GPIO` | 17 | El ESP32 transmite al RX del GPS |
| Velocidad en baudios | `CONFIG_GPS_BAUD_RATE` | 9600 | La mayoría de los módulos GPS usan 9600 |
| GPIO PPS | `CONFIG_GPS_PPS_GPIO` | -1 | Poner a GPIO válido para soporte PPS; -1 para deshabilitar |
| Timeout de detección (ms) | `CONFIG_GPS_DETECT_TIMEOUT_MS` | 3000 | Ventana de sondeo GPS al arrancar; 1000–10000 ms |

### Autenticación HTTP Básica

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Habilitar autenticación | `CONFIG_WSPR_HTTP_AUTH_ENABLE` | `n` | Requerir usuario/contraseña para todos los endpoints HTTP |
| Nombre de usuario | `CONFIG_WSPR_HTTP_AUTH_USER` | `admin` | Solo cuando la autenticación está habilitada |
| Contraseña | `CONFIG_WSPR_HTTP_AUTH_PASS` | `wspr` | Solo cuando la autenticación está habilitada |

### Idioma de la interfaz web

| Opción | Símbolo |
|---|---|
| Inglés | `CONFIG_WEBUI_LANG_EN` |
| Español | `CONFIG_WEBUI_LANG_ES` |

### Opciones de depuración

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Watchdog de tarea | `CONFIG_WSPR_TASK_WDT_ENABLE` | `n` | Registrar la tarea del planificador con el watchdog de tarea IDF. Requiere timeout WDT ≥ 2 s en sdkconfig. |
| Log de sobrecargas de símbolo | `CONFIG_WSPR_SYMBOL_OVERRUN_LOG` | `y` | Emitir `ESP_LOGW` si un símbolo se programa >10 ms después de su deadline. |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Banco de filtros de paso bajo

Un filtro de paso bajo (LPF) es **legalmente requerido** en prácticamente todas las jurisdicciones para suprimir armónicos antes de conectar la salida del oscilador a una antena.

El firmware controla un bus de dirección binaria de 3 bits (GPIO_A = bit 0, GPIO_B = bit 1, GPIO_C = bit 2) que selecciona una de hasta ocho posiciones de filtro. El mapeo de banda a ID de filtro está definido en `BAND_FILTER[]` en `config.c`:

| ID Filtro | GPIO C | GPIO B | GPIO A | Bandas servidas |
|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 2200 m, 630 m |
| 1 | 0 | 0 | 1 | 160 m |
| 2 | 0 | 1 | 0 | 80 m, 60 m |
| 3 | 0 | 1 | 1 | 40 m |
| 4 | 1 | 0 | 0 | 30 m |
| 5 | 1 | 0 | 1 | 20 m, 17 m |
| 6 | 1 | 1 | 0 | 15 m, 12 m |
| 7 | 1 | 1 | 1 | 10 m |

Tras llamar a `gpio_filter_select()`, el firmware inserta un retardo `CONFIG_WSPR_LPF_SETTLE_MS` (predeterminado 10 ms) antes de habilitar la salida del oscilador. Los relés de estado sólido pueden necesitar tan solo 1–2 ms; los relés mecánicos típicamente necesitan 5–20 ms.

### Frecuencias de corte de LPF recomendadas

| Filtro | Bandas | Frecuencia de corte |
|---|---|---|
| LPF-0 | 2200 m / 630 m | 600 kHz |
| LPF-1 | 160 m | 2,5 MHz |
| LPF-2 | 80 m / 60 m | 6 MHz |
| LPF-3 | 40 m | 8 MHz |
| LPF-4 | 30 m | 12 MHz |
| LPF-5 | 20 m / 17 m | 20 MHz |
| LPF-6 | 15 m / 12 m | 26 MHz |
| LPF-7 | 10 m | 32 MHz |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Hardware oscilador

### Si5351A (preferido)

El **Si5351A** es un generador de reloj programable por I2C fabricado por Silicon Laboratories (ahora Skyworks). Es el oscilador preferido porque soporta **síntesis PLL fraccionaria-N**, es detectable por I2C (sondeo ACK en dirección `0x60`), y está disponible en placas breakout económicas.

**Arquitectura:** Cristal (25 o 27 MHz) → PLLA (VCO 600–900 MHz, multiplicador entero) → Divisor de salida MS0 (modo entero para bajo jitter) → R-divisor opcional (para bandas por debajo de 500 kHz) → salida CLK0. Solo se usa CLK0 bloqueado a PLLA.

**Actualizaciones símbolo a símbolo:** Solo se reescribe el numerador fraccional PLL (p2) por símbolo, evitando reseteos PLL entre tonos. Un **caché de banda** (`si5351_band_cache_t`) precomputa todos los coeficientes del divisor una vez por cambio de banda; dentro de una ventana de 162 símbolos solo se escriben 6 bytes de registros I2C por símbolo (~1 transacción cada 683 ms).

Se usa la nueva API I2C master de ESP-IDF (`driver/i2c_master.h`, `i2c_master_bus_handle_t`) a 400 kHz Modo Rápido.

### AD9850 DDS

El **AD9850** es un Sintetizador Digital Directo (DDS) de Analog Devices que opera con un reloj de referencia de 125 MHz (configurable). Usa una palabra de sintonización de frecuencia (FTW) de 32 bits. Debido a que su bus es de solo escritura (sin lectura de retorno), el firmware **siempre asume que está presente** tras el fallo del sondeo del Si5351 (cuando `CONFIG_OSCILLATOR_ASSUME_AD9850=y`). La salida es una onda sinusoidal reconstruida por DAC de 10 bits (~1 V pico a pico en 50 Ω). Se recomienda encarecidamente un amplificador de potencia externo.

### Diferimiento de calibración

Ambos drivers de oscilador soportan calibración diferida: cuando `oscillator_set_cal()` es llamado mientras un bucle de símbolos WSPR está activo (entre `oscillator_tx_begin()` y `oscillator_tx_end()`), el valor ppb se almacena internamente y se aplica atómicamente por `oscillator_tx_end()` después del último símbolo, previniendo transitorios de frecuencia a mitad de símbolo.

### Modo ficticio

Si no se encuentra ningún oscilador (sondeo I2C del Si5351 falla y `CONFIG_OSCILLATOR_ASSUME_AD9850=n`), el firmware entra en **modo ficticio**: todas las llamadas `oscillator_*` devuelven `ESP_OK` silenciosamente y la WebUI muestra `None (DUMMY)`.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Sincronización horaria

La hora UTC precisa es **esencial** para WSPR. Las transmisiones que comiencen con más de ±1–2 segundos del límite del minuto par no serán decodificadas.

### Autodetección (tiempo de ejecución)

La fuente de tiempo se **selecciona automáticamente al arrancar** — no se necesita ningún interruptor de configuración en tiempo de compilación. `time_sync_init()` sondea el UART GPS configurado durante `CONFIG_GPS_DETECT_TIMEOUT_MS` milisegundos (predeterminado 3000 ms):

- Si se recibe una sentencia NMEA válida con checksum XOR correcto → se activa el **modo GPS**.
- Si no se recibe ninguna sentencia válida → el **modo NTP** arranca automáticamente.

Tanto el código UART GPS como el código SNTP se compilan en el firmware incondicionalmente.

### Modo GPS

Una tarea en segundo plano `gps_task` (pila 6 kB, prioridad 5) lee sentencias NMEA-0183 del UART configurado. Tipos de sentencias soportados:

- `$GPRMC` / `$GNRMC` — Sentencias RMC (fecha + hora + flag de validez + posición)
- `$GPZDA` / `$GNZDA` — Sentencias ZDA (fecha + hora, año de 4 dígitos)
- `$GPGGA` / `$GNGGA` — Sentencias GGA (posición + calidad del fix, usadas para el botón de localizador GPS)

Cada sentencia se valida con un checksum XOR NMEA completo antes de parsear. La variable de entorno `TZ` se fuerza a `"UTC0"` antes de cualquier llamada a `mktime()`.

Precisión típica: ±1 s (limitada por la tasa de sentencias NMEA de 1 Hz y la latencia UART).

**Soporte PPS:** Cuando `CONFIG_GPS_PPS_GPIO` se establece a un GPIO válido (≥ 0), una ISR de flanco ascendente (`pps_isr`, en IRAM para mínima latencia) se activa en cada pulso PPS y pone a cero el componente sub-segundo del reloj del sistema. Esto reduce la incertidumbre de tiempo de ~10 ms (latencia UART) a unos pocos microsegundos. Establecer `CONFIG_GPS_PPS_GPIO = -1` (predeterminado) para deshabilitar.

El modo GPS es completamente independiente de Wi-Fi, adecuado para instalaciones portátiles o remotas sin internet.

### Modo NTP (fallback)

Usa el cliente SNTP de ESP-IDF (`esp_sntp`) para consultar periódicamente el servidor configurado. Precisión típica: 1–50 ms.

El nombre de host del servidor NTP es configurable desde la WebUI. Los cambios se aplican **inmediatamente** vía `time_sync_restart_ntp()` sin reinicio del dispositivo.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Wi-Fi y red

### Modo STA (estación)

Si se configura un SSID Wi-Fi, el firmware intenta asociarse. Se realizan hasta 5 reintentos consecutivos dentro de una ventana de 15 segundos. En caso de éxito la IP asignada por DHCP se registra y la interfaz web se vuelve accesible.

### Modo fallback AP

Si no se configura ningún SSID, o si todos los intentos STA fallan, el firmware arranca un punto de acceso suave:
- SSID: `CONFIG_WSPR_AP_SSID` (predeterminado `WSPR-Config`)
- Contraseña: `CONFIG_WSPR_AP_PASS` (si ≥ 8 caracteres, WPA2-PSK; de lo contrario red abierta)
- IP: `192.168.4.1`
- Hasta 4 clientes simultáneos

### Reconexión en segundo plano

Cuando está en modo solo AP con credenciales STA guardadas, un `esp_timer` periódico se activa cada **5 minutos** y reintenta la conexión STA (transiciona al modo APSTA, llama a `esp_wifi_connect()`). El AP suave permanece activo durante todo el proceso.

### Escaneo Wi-Fi

El endpoint `GET /api/wifi_scan` lanza un escaneo bloqueante (~2 s). En modo solo AP, el escaneo eleva temporalmente al modo APSTA, espera 300 ms para que la interfaz STA se inicialice, escanea, y luego vuelve al modo AP. Los resultados se limitan a 20 entradas. Las redes ocultas (SSID vacío) se filtran. Un mutex FreeRTOS previene llamadas de escaneo concurrentes.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Frecuencias de banda WSPR y regiones IARU

Todas las frecuencias son **frecuencias de centro RF** en Hz (= frecuencia de marcación SSB + 1500 Hz). El oscilador se programa directamente a estos valores; no se añade ningún desplazamiento adicional de 1500 Hz en tiempo de ejecución.

### Tabla de frecuencias completa

| Banda | Región 1 (EU/África/OM) | Región 2 (Américas) | Región 3 (Asia/Pacífico) |
|---|---|---|---|
| 2200 m | 137.600 Hz | 137.600 Hz | 137.600 Hz |
| 630 m | 475.700 Hz | 475.700 Hz | 475.700 Hz |
| 160 m | 1.838.100 Hz | 1.838.100 Hz | 1.838.100 Hz |
| 80 m | 3.570.100 Hz | 3.570.100 Hz | 3.570.100 Hz |
| **60 m** | **5.288.600 Hz** | **5.346.500 Hz** | **5.367.000 Hz** |
| 40 m | 7.040.100 Hz | 7.040.100 Hz | 7.040.100 Hz |
| 30 m | 10.140.200 Hz | 10.140.200 Hz | 10.140.200 Hz |
| 20 m | 14.097.100 Hz | 14.097.100 Hz | 14.097.100 Hz |
| 17 m | 18.106.100 Hz | 18.106.100 Hz | 18.106.100 Hz |
| 15 m | 21.096.100 Hz | 21.096.100 Hz | 21.096.100 Hz |
| 12 m | 24.926.100 Hz | 24.926.100 Hz | 24.926.100 Hz |
| 10 m | 28.126.100 Hz | 28.126.100 Hz | 28.126.100 Hz |

**Nota sobre los 60 m:** La Región 1 (Europa) usa 5.288,6 kHz. La Región 2 (Américas) usa 5.346,5 kHz según la coordinación FCC/ARRL. La Región 3 usa 5.367,0 kHz según la coordinación WIA/JARL. Siempre verifica que la operación en 60 m esté permitida bajo tu licencia nacional de radioafición.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Modo de salto de frecuencia

Cuando `hop_enabled = true`, el `scheduler_task` rota por la lista de bandas habilitadas después de cada `hop_interval_sec` segundos.

**Lógica de salto:**
1. En cada evaluación de slot TX, verificar si `(ahora - ultimo_salto) >= hop_interval_sec`.
2. Si sí, avanzar el índice de banda a la siguiente banda habilitada (con wraparound).
3. Llamar a `gpio_filter_select(BAND_FILTER[nueva_banda])` y `oscillator_set_freq(nueva_base_hz)`.
4. Si está pre-armado a la frecuencia anterior, invalidar el estado de pre-armado.

Si solo hay una banda habilitada, el salto queda efectivamente deshabilitado. Si no hay ninguna banda habilitada, el firmware cae automáticamente a 40 m. El intervalo mínimo de salto de 120 s es aplicado en `config_load()`.

---

## Ciclo de trabajo TX

El firmware implementa el ciclo de trabajo como un **acumulador determinista**:

- Antes de cada slot de 2 minutos: `acumulador += tx_duty_pct`
- Si `acumulador >= 100`: transmitir el slot, luego `acumulador -= 100`
- De lo contrario: omitir el slot

El acumulador se resetea cuando el porcentaje de ciclo de trabajo se cambia desde la WebUI.

- `tx_duty_pct = 0`: Nunca transmitir.
- `tx_duty_pct = 20`: Transmitir 1 de cada 5 slots — el estándar de la comunidad WSPR.
- `tx_duty_pct = 100`: Transmitir en cada slot.

---

## Calibración de cristal

El campo `xtal_cal_ppb` almacena el desplazamiento de calibración en **partes por mil millones (ppb)**.

### Cómo se aplica la calibración

**Si5351A:** La corrección en ppb se aplica a la frecuencia objetivo del VCO PLL y también ajusta directamente el numerador fraccional PLL (`pll_b_base`) para correcciones finas. Aplicar calibración invalida el caché de banda.

**AD9850:** La corrección en ppb escala las constantes de conversión frecuencia-a-FTW precomputadas (`_ad_ftw_per_mhz_cal`, `_ad_ftw_int_per_hz_cal`) proporcionalmente.

**Diferimiento:** Si una actualización de calibración llega durante un bucle de símbolos WSPR activo, se pone en cola y se aplica atómicamente por `oscillator_tx_end()` después del símbolo final.

### Cómo calibrar con el tono de prueba

1. Establece `xtal_cal_ppb = 0` en la WebUI.
2. Habilita el tono de prueba a una frecuencia conocida (p. ej. 14097,1 kHz para la marcación en 20 m).
3. Usa un receptor SDR calibrado para medir la frecuencia central real del tono.
4. Introduce la **frecuencia medida** en el campo "Tono de prueba recibido" en la tarjeta Estación.
5. Haz clic en el botón de **auto-calibración** (🔧) — calcula `cal_ppb = (medida - nominal) × 1e9 / nominal`, lo guarda y reinicia el tono con la frecuencia corregida.

Los reportes de recepción de WSPRnet también incluyen el desplazamiento de frecuencia en Hz medido por la estación receptora, que puede usarse para estimar el error de calibración tras una transmisión en vivo.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Estado de implementación

| Característica | Estado |
|---|---|
| Codificador WSPR Tipo 1 (indicativo simple + localizador 4 chars) | ✅ Completo |
| Codificador WSPR Tipo 2 (indicativo compuesto con `/`) | ✅ Completo |
| Codificador WSPR Tipo 3 (acompañante localizador 6 chars / hash indicativo) | ✅ Completo |
| Alternancia Tipo 1/Tipo 3 (indicativo simple + localizador 6 chars) | ✅ Completo |
| Alternancia Tipo 2/Tipo 3 (indicativo compuesto) | ✅ Completo |
| Driver Si5351A (I2C, autodetección via sondeo ACK) | ✅ Completo |
| Driver AD9850 DDS (bit-bang GPIO) | ✅ Completo |
| Modo ficticio del oscilador (sin hardware) | ✅ Completo |
| Diferimiento de calibración durante ventana TX activa | ✅ Completo |
| Driver de banco LPF GPIO de 3 bits | ✅ Completo |
| Autodetección GPS (sondeo NMEA UART al arrancar) | ✅ Completo |
| Sincronía de tiempo GPS ($GPRMC/$GNRMC/$GPZDA/$GNZDA) | ✅ Completo |
| Extracción de posición GPS para botón de localizador ($GPGGA/$GNGGA) | ✅ Completo |
| Precisión sub-segundo GPS PPS (ISR de flanco ascendente) | ✅ Completo |
| Sincronía de tiempo NTP (SNTP, cambio de servidor inmediato) | ✅ Completo |
| Modo Wi-Fi STA | ✅ Completo |
| Fallback Wi-Fi AP (192.168.4.1) | ✅ Completo |
| Temporizador de reconexión Wi-Fi en segundo plano (5 min) | ✅ Completo |
| Configuración persistente NVS (esquema v6) | ✅ Completo |
| Interfaz web SPA embebida | ✅ Completo |
| API REST completa (9 endpoints) | ✅ Completo |
| Autenticación HTTP Básica | ✅ Completo |
| Selección de región IARU para 60 m | ✅ Completo |
| Soporte de 12 bandas (2200 m – 10 m) | ✅ Completo |
| Salto de frecuencia | ✅ Completo |
| Ciclo de trabajo TX (acumulador determinista) | ✅ Completo |
| Calibración de cristal (ppb) con diferimiento | ✅ Completo |
| Modo de tono de prueba (portadora continua) | ✅ Completo |
| Auto-calibración desde frecuencia de tono medida | ✅ Completo |
| Pre-armado del oscilador en phase=0 para temporización precisa | ✅ Completo |
| WebUI en inglés y español | ✅ Completo |
| Enlace de spots WSPRnet / PSKReporter | ✅ Completo |
| Información de reinicio (hora de inicio, causa de reset) en estado | ✅ Completo |
| Integración con watchdog de tarea | ✅ Completo |
| Log de sobrecarga de temporización de símbolos | ✅ Completo |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Hoja de ruta

- [ ] **RTC DS3231** — cronometraje sin conexión cuando Wi-Fi y GPS no están disponibles
- [ ] **Actualización OTA de firmware** — actualización por aire desde la WebUI
- [ ] **GPIO de habilitación de amplificador de potencia** — encender/apagar un PA alrededor de las transmisiones
- [ ] **Programación de transmisiones** — reglas por hora del día o banda/día de la semana
- [ ] **Telemetría MQTT** — publicar estado a un broker MQTT para monitoreo remoto
- [ ] **Idiomas adicionales en la UI**
- [ ] **Carga automática a WSPRnet** — carga HTTP directa de spots recibidos (requiere RX SDR)
- [ ] **Soporte de bandas de 6 m y 4 m** — extensión para WSPR en VHF
- [ ] **Soporte de pantalla SPI** — pantalla de estado OLED/TFT opcional

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Contribuciones

Las contribuciones son muy apreciadas. Si tienes una sugerencia, haz un fork del repositorio y crea un pull request, o abre un issue con la etiqueta "enhancement".

1. Haz un fork (<https://github.com/hiperiondev/ESP32_WSPR/fork>)
2. Crea tu rama de característica (`git checkout -b feature/MiCaracteristica`)
3. Haz commit de tus cambios (`git commit -am 'Agregar MiCaracteristica'`)
4. Haz push a la rama (`git push origin feature/MiCaracteristica`)
5. Crea un nuevo Pull Request

### Estilo de código

- C99, convenciones de codificación ESP-IDF.
- Todos los módulos nuevos deben tener un `.h` correspondiente con documentación API estilo Doxygen.
- Sin aritmética de punto flotante ni de 64 bits en la ruta crítica.
- Las nuevas opciones de Kconfig deben documentarse en este README.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Licencia

Distribuido bajo la **Licencia Pública General GNU v3.0**. Ver `LICENSE.txt` para más información.

```
Copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)

Este programa es software libre: puedes redistribuirlo y/o modificarlo
bajo los términos de la Licencia Pública General GNU publicada por
la Free Software Foundation, ya sea la versión 3 de la Licencia, o
(a tu elección) cualquier versión posterior.
```

El algoritmo de codificación WSPR está basado en el código fuente original de WSJT-X de Joe Taylor (K1JT) y la descripción del protocolo de Andy Talbot (G4JNT).

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Contacto

*Emiliano Augusto Gonzalez — [egonzalez.hiperion@gmail.com](mailto:egonzalez.hiperion@gmail.com)*

Enlace del proyecto: [https://github.com/hiperiondev/ESP32\_WSPR](https://github.com/hiperiondev/ESP32_WSPR)

---

## Referencias

### Protocolo WSPR

- **G4JNT (Andy Talbot)** — *"The WSPR Coding Process"* (2009). [PDF](http://www.g4jnt.com/WSPR_Coding_Process.pdf)
- **K1JT (Joe Taylor)** — Código fuente y documentación de WSJT-X. [wsjt.sourceforge.io](https://wsjt.sourceforge.io/)
- **WSPRnet** — Base de datos global de recepción WSPR y mapas. [wsprnet.org](https://www.wsprnet.org)
- **Wikipedia** — WSPR (software de radioafición). [en.wikipedia.org/wiki/WSPR](https://en.wikipedia.org/wiki/WSPR_(amateur_radio_software))
- **Scott Harden (AJ4VD)** — Notas del protocolo WSPR. [swharden.com](https://swharden.com/software/FSKview/wspr/)
- **Lista de frecuencias WSPR** — [wsprnet.org/drupal/node/218](https://www.wsprnet.org/drupal/node/218)
- **Documentación Wsprry Pi** — Resumen de especificaciones WSPR. [wsprry-pi.readthedocs.io](https://wsprry-pi.readthedocs.io/en/latest/About_WSPR/)

### Hardware oscilador

- **Silicon Laboratories / Skyworks** — Datasheet Si5351A/B/C-B. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/Si5351-B.pdf)
- **Skyworks** — AN619: Generación manual de un mapa de registros Si5351. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)
- **Skyworks** — AN1234: Generación manual de un mapa de registros Si5351 para dispositivos 16QFN. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/an1234-si5351-16qfn-register-map.pdf)
- **QRP Labs** — Código demo Si5351A. [qrp-labs.com](https://qrp-labs.com/synth/si5351ademo.html)
- **NT7S (Jason Milldrum)** — Biblioteca Si5351Arduino. [github.com/etherkit/Si5351Arduino](https://github.com/etherkit/Si5351Arduino)
- **Analog Devices** — Datasheet AD9850. [analog.com](https://www.analog.com/media/en/technical-documentation/data-sheets/ad9850.pdf)

### Proyectos ESP32 WSPR relacionados

- **danak6jq/ESP32-WSPR** — Transmisor WSPR2 autónomo con ESP32 + Si5351 (ESP-IDF v3). [github.com](https://github.com/danak6jq/ESP32-WSPR)
- **mm5agm/WSPR-Multi-Band** — Beacon WSPR multibanda para ESP32 (Arduino). [github.com](https://github.com/mm5agm/WSPR-Multi-Band)
- **etherkit/JTEncode** — Biblioteca codificadora JT65/JT9/JT4/WSPR/FSQ para Arduino. [github.com](https://github.com/etherkit/JTEncode)

### Documentación ESP-IDF

- **Guía de programación ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- **API SNTP ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html)
- **Servidor HTTP ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html)
- **Driver Wi-Fi ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- **NVS Flash ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html)
- **Driver I2C Master ESP-IDF**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)

---

<div align="center">

*73 de LU3VEA — ¡Buen DX!*

</div>

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>
