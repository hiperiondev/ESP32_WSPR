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
14. [Frecuencias de banda WSPR y regiones UAIRO](#frecuencias-de-banda-wspr-y-regiones-uairo)
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

El proyecto está diseñado para ser apto para operación desatendida como beacon: codifica mensajes WSPR Tipo 1, 2 y 3 completamente en el chip, controla un oscilador RF (Si5351A o AD9850) con resolución sub-Hz por símbolo, selecciona automáticamente el filtro de paso bajo correcto mediante un bus GPIO de 3 bits, sincroniza la hora vía NTP o GPS (con PPS opcional), y expone una aplicación web de página única responsiva para configuración y monitoreo. Todos los ajustes del usuario persisten en la partición flash NVS (almacenamiento no volátil) del ESP32 y sobreviven a cortes de energía.

El modo WSPR ocupa aproximadamente 6 Hz de ancho de banda RF y puede decodificarse con relaciones señal/ruido tan bajas como −28 dB en un ancho de banda de referencia de 2,5 kHz, lo que lo hace extremadamente útil para estudios de propagación usando niveles de potencia muy bajos. Una vez transmitidos, los reportes de recepción de estaciones WSPR automatizadas en todo el mundo se cargan automáticamente a [WSPRnet](https://www.wsprnet.org), donde una interfaz de mapas permite ver exactamente hasta dónde llegó la señal.

### Decisiones clave de diseño

- **Solo ESP-IDF** — sin Arduino, sin bibliotecas I2C de terceros. Todos los drivers del oscilador están escritos desde cero sobre las APIs `driver/i2c_master.h` y `driver/gpio.h` de IDF.
- **Soporte de oscilador dual** — el firmware autodetecta un Si5351A al arrancar mediante sondeo ACK por I2C, luego cae al AD9850 (SPI serial bit-bang de solo escritura), y finalmente a un modo ficticio silencioso si ninguno está presente — así el sistema nunca falla ante hardware faltante.
- **Aritmética solo entera** — el codificador WSPR completo, el cálculo de divisores PLL del Si5351 y el cómputo de la palabra de sintonización del AD9850 usan únicamente matemáticas enteras de 32 bits. Sin punto flotante, sin `double`, lo que hace el código eficiente en el núcleo Xtensa LX6 sin necesitar la biblioteca soft-float.
- **SPA en un solo archivo embebido** — la interfaz web se compila en el firmware como un archivo de cabecera C; no se necesita componente de sistema de archivos ni partición SPIFFS/LittleFS.
- **WebUI multilingüe** — las tablas de cadenas en inglés y español se proveen como cabeceras separadas (`webui_en.h` / `webui_es.h`) seleccionadas en tiempo de compilación vía Kconfig.
- **Degradación elegante** — el hardware de oscilador faltante, la pérdida de Wi-Fi, NTP no disponible y los blobs NVS corruptos se manejan sin fallos, con mensajes de registro informativos e indicadores de estado en la WebUI.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Descripción del protocolo WSPR

WSPR (**W**eak **S**ignal **P**ropagation **R**eporter, pronunciado *"susurro"* en inglés) es un protocolo de radio digital para radioaficionados diseñado por Joe Taylor (K1JT), Premio Nobel de Física, y lanzado originalmente en 2008. Forma parte del conjunto WSJT-X y se ha convertido en uno de los modos de baliza de propagación más ampliamente utilizados en la radioafición.

### Qué transmite WSPR

Un mensaje WSPR Tipo 1 codifica exactamente tres piezas de información:

| Campo | Descripción | Ancho de codificación |
|---|---|---|
| **Indicativo** | Identificador de estación (hasta 6 caracteres alfanuméricos) | 28 bits |
| **Localizador Maidenhead** | Cuadrícula de 4 caracteres (p. ej. `GF05`) | 15 bits |
| **Potencia TX** | Potencia de transmisión en dBm (uno de 19 niveles válidos) | 7 bits |

Carga útil total: **50 bits**.

### Proceso de codificación (especificación G4JNT / K1JT)

El siguiente proceso está implementado en `wspr_encode.c`:

```
Entrada: indicativo + localizador + potencia_dBm
        │
        ▼
1. Empaquetar indicativo → entero de 28 bits (fórmula estándar G4JNT)
   - Rellenar a 6 chars; anteponer espacio si char[1] es una letra
   - Chars 0-1: alfabeto de 37 símbolos (0-9=0..9, A-Z=10..35, espacio=36)
   - Char 2:    solo dígito (0-9)
   - Chars 3-5: alfabeto sufijo de 27 símbolos (A-Z=0..25, espacio=26)
        │
        ▼
2. Empaquetar localizador → entero de 15 bits
   - Fórmula: (179 - 10*(c0-'A') - d0) * 180 + (10*(c1-'A') + d1)
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

Las transmisiones WSPR **deben** comenzar al inicio de un minuto UTC par (00:00, 00:02, 00:04, …). El planificador calcula el tiempo restante hasta el próximo límite de minuto par usando `ahora % 120` y entra en una espera de precisión en los últimos dos segundos para alinear el primer símbolo con pocos milisegundos del límite. El firmware se niega a transmitir hasta que la sincronización de tiempo (NTP o GPS) haya ocurrido, garantizando que el reloj del sistema sea válido.

### Ciclo de trabajo

El estándar de la comunidad WSPR recomienda transmitir en no más del 20% de los slots de 2 minutos disponibles, dejando el resto para recepción. Este firmware implementa un porcentaje de ciclo de trabajo configurable (0–100%) usando un acumulador determinista: `acumulador += ciclo_pct` cada slot; se usa un slot cuando el acumulador alcanza o supera 100, momento en que se resta 100. Esto produce una distribución perfectamente uniforme de slots de transmisión sin aleatoriedad.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Características

- ✅ **Codificador WSPR Tipo 1, 2 y 3 completo** — empaquetado de indicativo, codificación convolucional (K=32, tasa 1/2), intercalado por inversión de bits, superposición del vector de sincronía; aritmética solo entera
- ✅ **Soporte de oscilador dual** — Si5351A (I2C, autodetectado) y AD9850 (bit-bang GPIO, asumido presente); modo ficticio elegante si ninguno es encontrado
- ✅ **12 bandas WSPR** — de 2200 m a 10 m (137 kHz a 28 MHz)
- ✅ **Selección de región UAIRO/IARU** — Región 1, 2 o 3 para la frecuencia de marcación correcta en 60 m
- ✅ **Selección automática de filtro de paso bajo** — bus GPIO BCD de 3 bits, 8 posiciones de filtro, retardo de establecimiento de relé configurable
- ✅ **Sincronía de tiempo vía NTP** (SNTP, servidor seleccionable) o **GPS** (NMEA-0183 $GPRMC/$GNRMC/$GPZDA/$GNZDA por UART, con PPS opcional)
- ✅ **Modo STA Wi-Fi** con fallback a AP suave (192.168.4.1) y temporizador de reconexión en segundo plano
- ✅ **Salto de frecuencia** — rotación automática por las bandas habilitadas cada N segundos (mín. 120 s = 1 slot TX)
- ✅ **Ciclo de trabajo TX** — 0–100% configurable con selección de slot por acumulador determinista
- ✅ **Calibración de cristal** — corrección ±ppb almacenada en NVS, aplicada a todos los cálculos de frecuencia
- ✅ **Aplicación web de página única embebida** — sin SPIFFS, sin archivos externos; completamente autocontenido
- ✅ **API REST** — endpoints JSON para lectura/escritura de configuración, sondeo de estado, escaneo Wi-Fi, alternancia TX, reinicio del sistema
- ✅ **Configuración persistente NVS** — todos los ajustes sobreviven cortes de energía; verificación de versión de esquema con defaults automáticos ante discrepancia
- ✅ **UI multilingüe** — inglés y español (selección en tiempo de compilación vía Kconfig)
- ✅ **Autenticación HTTP Básica** — protección opcional por usuario/contraseña para la interfaz web
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
| **Oscilador RF** | Placa breakout Si5351A (I2C) **o** módulo DDS AD9850 (serial GPIO) |
| **Antena** | Antena de hilo apropiada para la(s) banda(s) de operación |

### Componentes adicionales recomendados

| Componente | Propósito |
|---|---|
| **Banco de filtros de paso bajo** | Supresión de armónicos (legalmente requerido en la mayoría de jurisdicciones) |
| **Decodificador BCD / driver de relé** | Controlado por 3 líneas GPIO (GPIO_A, GPIO_B, GPIO_C) |
| **Módulo GPS** (NMEA UART) | Para sincronía de tiempo por GPS (alternativa a NTP), especialmente en uso portátil/remoto |
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
- Velocidad I2C: 400 kHz (Modo Rápido)
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

- Reloj de referencia: 125 MHz (módulos AD9850 más comunes)
- Interfaz: serial bit-bang (MSB primero, palabra de frecuencia de 32 bits + control de 8 bits)
- Palabra de sintonización de frecuencia: `FTW = freq_Hz × 2^32 / ref_clk_Hz`
- Salida: onda sinusoidal, ~1 V pico a pico en 50 Ω (mucho menor que el Si5351)

### Conexionado del banco de filtros

```
ESP32                Placa decodificador BCD / driver de relé
GPIO_A (bit 0)  ──── Entrada A
GPIO_B (bit 1)  ──── Entrada B
GPIO_C (bit 2)  ──── Entrada C
3,3 V           ──── VCC lógica
GND             ──── GND
```

Ocho posiciones de filtro (0–7) son seleccionadas por el código binario de 3 bits en GPIO_A/B/C. El firmware selecciona el filtro correcto para cada banda automáticamente usando la tabla `BAND_FILTER[]` en `config.c`.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Arquitectura y código fuente

El firmware está organizado como un único componente ESP-IDF (`main/`). Todos los archivos fuente se compilan juntos vía `CMakeLists.txt` usando `GLOB_RECURSE`.

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
├── time_sync.c / .h        — Abstracción de sincronía NTP (SNTP) y GPS (NMEA UART + PPS opcional)
├── wifi_manager.c / .h     — Wi-Fi STA + fallback AP; reconexión en segundo plano; escaneo Wi-Fi
├── web_server.c / .h       — Servidor HTTP; API REST; caché de estado; mutex de configuración
├── wspr_encode.c / .h      — Codificador WSPR Tipo 1/2/3 completo (solo enteros)
│
├── webui_strings.h         — Cabecera de despacho: incluye webui_en.h o webui_es.h
├── webui_en.h              — Tabla de cadenas UI en inglés
└── webui_es.h              — Tabla de cadenas UI en español
```

### Estructura de tareas (FreeRTOS)

El firmware ejecuta dos tareas FreeRTOS de larga duración después de que `app_main()` completa su secuencia de inicialización:

```
app_main()
  │
  ├─ [secuencia de inicialización]
  │    config_init() → config_load()
  │    gpio_filter_init()
  │    oscillator_init() → oscillator_set_cal()
  │    wifi_manager_start()
  │    time_sync_init()
  │    web_server_start()
  │
  ├─ xTaskCreate(status_task,    pila=6144, prioridad=3)
  │     — sondea tiempo/estado cada segundo,
  │       llama a web_server_update_status()
  │
  └─ xTaskCreate(scheduler_task, pila=8192, prioridad=5)
        — espera sincronía de tiempo, calcula el próximo slot TX,
          llama a wspr_transmit(), maneja lógica de salto/ciclo
```

### Secuencia de transmisión WSPR (`wspr_transmit()`)

```
1. Bloquear mutex de config; capturar indicativo, localizador, potencia, región, paridad
2. Determinar tipo de mensaje (wspr_encode_type)
3. Codificar 162 símbolos:
     - Tipo 1 o Tipo 2 (paridad=0): wspr_encode()
     - Acompañante Tipo 3  (paridad=1): wspr_encode_type3()
4. Si no está pre-armado:
     a. Fijar frecuencia base del oscilador (frecuencia de marcación + 1500 Hz)
     b. gpio_filter_select(BAND_FILTER[banda])
     c. vTaskDelay(CONFIG_WSPR_LPF_SETTLE_MS)  — establecimiento del relé
5. oscillator_tx_begin()
6. oscillator_enable(true)
7. Para cada uno de los 162 símbolos:
     a. oscillator_set_freq_mhz(base_hz, símbolo × 1464844 mHz)
     b. Espera activa hasta el deadline del símbolo (temporizador con precisión de µs)
8. oscillator_enable(false)
9. oscillator_tx_end()
```

> **Nota:** Los pasos 4a–4c también se ejecutan especulativamente en `phase=0` (segundo 0 del límite de minuto par) para minimizar la latencia entre el límite y el primer símbolo. El estado de pre-armado se rastrea en `g_pre_armed`.

### Estructura de configuración (`wspr_config_t`)

Todos los ajustes se almacenan en un único blob NVS identificado por la clave `"cfg"` en el espacio de nombres `"wspr"`. La versión actual del esquema es **5** (`CONFIG_SCHEMA_VERSION`).

```c
typedef struct {
    uint8_t  version;               // Versión del esquema (debe ser igual a CONFIG_SCHEMA_VERSION = 5)
    char     callsign[12];          // Indicativo amateur (simple o compuesto con '/')
    char     locator[7];            // Localizador Maidenhead de 4 o 6 caracteres
    uint8_t  power_dbm;             // Potencia TX en dBm (niveles válidos WSPR)
    char     wifi_ssid[33];         // SSID Wi-Fi
    char     wifi_pass[65];         // Contraseña Wi-Fi
    char     ntp_server[64];        // Nombre de host o IP del servidor NTP
    bool     band_enabled[12];      // Un flag por cada banda WSPR
    bool     hop_enabled;           // Habilitar salto de frecuencia automático
    uint32_t hop_interval_sec;      // Intervalo de salto en segundos (mín. 120)
    bool     tx_enabled;            // Habilitación/deshabilitación maestra de TX
    uint8_t  tx_duty_pct;           // Ciclo de trabajo TX (0-100 %)
    int32_t  xtal_cal_ppb;          // Desplazamiento de calibración del cristal en ppb
    uint8_t  iaru_region;           // Región UAIRO: 1, 2 o 3
    bool     bands_changed;         // Flag en tiempo de ejecución: la lista de bandas necesita reconstrucción (no se guarda en NVS)
    uint8_t  tx_slot_parity;        // Contador de alternancia Tipo 2/3 (no se guarda en NVS)
} wspr_config_t;
```

> **Nota:** `bands_changed` y `tx_slot_parity` son campos solo de tiempo de ejecución. Siempre se resetean a `false`/`0` por `config_load()` y `config_defaults()`, y nunca se escriben en NVS.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Interfaz web (WebUI)

La interfaz web es una aplicación de página única (SPA) servida completamente desde la memoria flash como un literal de cadena C embebido en `web_server.c`. No se necesita ningún componente de sistema de archivos (SPIFFS/LittleFS). La interfaz actualiza automáticamente los datos de estado cada 2 segundos vía el endpoint `/api/status`.

### Acceso a la WebUI

| Escenario | URL |
|---|---|
| Conectado a la red Wi-Fi doméstica (modo STA) | `http://<IP-asignada>` |
| Sin credenciales Wi-Fi, modo fallback AP | `http://192.168.4.1` |
| Tras escaneo Wi-Fi | IP mostrada en `/api/status` → campo `ip` |

La dirección IP también se imprime en la consola serie del ESP32 al arrancar.

### Capturas de la WebUI

#### Vista general de la página principal

La página principal está dividida en varias tarjetas de configuración, un panel de estado en vivo y un botón de control TX. Todos los cambios requieren guardar con el botón **Guardar configuración** antes de que surtan efecto. La configuración se persiste inmediatamente en la flash NVS.

---

#### Tarjeta de Estación

![WebUI Estación](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_station.jpg)

La tarjeta **Estación** configura el contenido del mensaje WSPR:

- **Indicativo** — Tu indicativo de radioaficionado. Los indicativos simples (p. ej. `W1AW`, `LU3VEA`, `G4JNT`) usan hasta 6 caracteres alfanuméricos con un dígito en la posición 2. Los indicativos compuestos que contienen una barra (p. ej. `PJ4/K1ABC`, `K1ABC/P`) activan la alternancia automática de mensajes Tipo 2 + Tipo 3. El codificador maneja automáticamente la normalización G4JNT (antepone un espacio si el carácter 1 es una letra, p. ej. `G4JNT` → `[esp]G4JNT`).
- **Localizador Maidenhead** — Cuadrícula de 4 caracteres (p. ej. `GF05`) o sub-cuadrícula de 6 caracteres (p. ej. `GF05ab`). Un localizador de 6 caracteres activa la alternancia automática de mensajes Tipo 1 + Tipo 3, transmitiendo la precisión de sub-cuadrícula a las estaciones receptoras. Ambos formatos son totalmente compatibles.
- **Potencia TX (dBm)** — Nivel de potencia de transmisión. Debe ser uno de los 19 niveles WSPR válidos: 0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60 dBm. Los valores fuera de este conjunto se redondean silenciosamente al nivel válido más cercano por el codificador.
- **Calibración XTAL (ppb)** — Corrección de frecuencia del cristal en partes por mil millones. Un valor positivo significa que el cristal funciona rápido (el firmware baja la frecuencia efectiva para compensar). Este valor se aplica inmediatamente a todos los cálculos de frecuencia del oscilador.

---

#### Tarjeta Wi-Fi

![WebUI Wi-Fi](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_wifi.jpg)

La tarjeta **Red Wi-Fi** maneja la conectividad:

- **Botón de escaneo** — Lanza un escaneo de canales Wi-Fi bloqueante (≈2 s) y popula una lista desplegable de puntos de acceso descubiertos con su SSID, potencia de señal (RSSI) y tipo de seguridad. Al hacer clic en una entrada se rellena automáticamente el campo SSID.
- **Contraseña** — Frase de acceso WPA/WPA2 (incluye interruptor mostrar/ocultar).
- **Servidor NTP** — Nombre de host o IP del servidor NTP (predeterminado: `pool.ntp.org`). Usado solo en modo de sincronía NTP; los cambios se aplican inmediatamente vía `time_sync_restart_ntp()` sin necesidad de reiniciar.
- **Indicación sin credenciales** — Si el campo SSID se deja en blanco, el ESP32 arranca en modo AP suave en `192.168.4.1`. Esto es útil para la configuración inicial sin una red conocida.

Si la conexión STA falla, el firmware cae al modo AP y arranca un temporizador de reconexión en segundo plano (intervalo de 5 minutos) para que el dispositivo se reconecte automáticamente cuando la red doméstica vuelva a estar disponible.

---

#### Tarjeta de Bandas Activas

![WebUI Bandas](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_bands.jpg)

La tarjeta **Bandas activas** presenta una casilla de verificación para cada una de las 12 bandas WSPR compatibles:

| Banda | Frecuencia de marcación (todas las regiones excepto 60 m) |
|---|---|
| 2200 m | 137,600 kHz |
| 630 m | 475,700 kHz |
| 160 m | 1.838,100 kHz |
| 80 m | 3.570,100 kHz |
| 60 m | Dependiente de la región (ver tarjeta Región UAIRO/IARU) |
| 40 m | 7.040,100 kHz |
| 30 m | 10.140,200 kHz |
| 20 m | 14.097,100 kHz |
| 17 m | 18.106,100 kHz |
| 15 m | 21.096,100 kHz |
| 12 m | 24.926,100 kHz |
| 10 m | 28.126,100 kHz |

Se pueden habilitar múltiples bandas simultáneamente. Cuando el salto de frecuencia está activo, el firmware recorre todas las bandas habilitadas en orden. Cuando el salto está deshabilitado, el firmware transmite solo en la primera banda habilitada.

---

#### Tarjeta de Región UAIRO/IARU

![WebUI UAIRO](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_iaru.jpg)

La tarjeta **Región UAIRO/IARU y frecuencia en 60 m** selecciona la región administrativa ITU/UAIRO/IARU. Esto afecta únicamente la frecuencia de marcación en la banda de 60 m, que difiere entre regiones debido a las asignaciones nacionales de espectro:

| Región | Cobertura | Frecuencia de marcación WSPR en 60 m |
|---|---|---|
| **Región 1** | Europa, África, Oriente Medio | 5.288,600 kHz |
| **Región 2** | Américas (Norte, Sur, Caribe) | 5.346,500 kHz |
| **Región 3** | Asia, Pacífico, Oceanía | 5.367,000 kHz |

Todas las demás bandas usan frecuencias de marcación idénticas en todo el mundo. Siempre verifica que la operación en 60 m esté permitida por la autoridad nacional de licencias de radioafición antes de transmitir.

---

#### Tarjeta de Salto de Frecuencia

![WebUI Salto](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_hop.jpg)

La tarjeta **Salto de frecuencia** habilita la rotación automática de bandas:

- **Interruptor de habilitación de salto** — Cuando está habilitado, el transmisor avanza a la siguiente banda habilitada después de que expire cada intervalo de salto.
- **Intervalo (segundos)** — Mínimo 120 segundos (un slot de transmisión WSPR completo). Los valores menores a 120 s almacenados en NVS se limitan a 120 s en `config_load()`.

Ejemplo: con 40 m, 20 m y 15 m habilitados y un intervalo de salto de 240 segundos, la secuencia es: 40 m → 20 m → 15 m → 40 m → … , con una transmisión cada 240 s por banda.

El salto requiere al menos 2 bandas habilitadas para ser significativo. Si no hay ninguna banda habilitada, el firmware cae automáticamente a 40 m. La banda actual siempre es visible en el panel de estado.

---

#### Tarjeta de Ciclo de Trabajo TX

![WebUI Ciclo](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_duty.jpg)

La tarjeta **Ciclo de trabajo TX** controla qué fracción de los slots WSPR de 2 minutos disponibles se usa realmente para transmisión:

- **0%** — Nunca transmitir (desactiva efectivamente el transmisor sin desmarcar bandas)
- **20%** — Estándar de la comunidad WSPR; transmite aproximadamente 1 de cada 5 slots
- **100%** — Transmitir en cada slot disponible

El firmware usa un **acumulador determinista** (no un generador de números aleatorios) para decidir cada slot si transmitir. El promedio a largo plazo iguala precisamente el porcentaje configurado sin generar variabilidad aleatoria.

---

#### Tarjeta de Control TX y panel de estado

![WebUI Estado](https://github.com/hiperiondev/ESP32_WSPR/raw/main/images/webui_status.jpg)

La tarjeta **Control TX** contiene un único botón de alternancia (**Activar TX** / **Detener TX**) que habilita o deshabilita la transmisión inmediatamente. Esto equivale a cambiar `tx_enabled` en la configuración y se guarda en NVS.

El **panel de estado** en vivo (actualizado cada 2 segundos) muestra:

| Campo | Descripción |
|---|---|
| **Hardware RF** | Oscilador detectado (Si5351A, AD9850 (asumido), o Ninguno/DUMMY) |
| **Sincronización horaria** | Hora UTC o "Sin sincronizar" |
| **Banda actual** | Nombre de la banda WSPR activa |
| **Frecuencia** | Frecuencia de marcación exacta en kHz |
| **Próxima TX** | Cuenta regresiva en segundos al próximo slot de transmisión |
| **TX activa** | Si una transmisión está actualmente en progreso |
| **Símbolo** | Índice del símbolo actual (0–161) durante la transmisión |
| **Hora de inicio** | Marca de tiempo UTC del último reinicio |
| **Causa de reset** | Razón hardware del último reinicio (Encendido, Watchdog, etc.) |

Un enlace a **WSPRnet** al pie de la página abre directamente el mapa de spots de tu indicativo.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

### API REST

El servidor web expone los siguientes endpoints:

| Método | URI | Descripción |
|---|---|---|
| `GET` | `/` | Devuelve el HTML completo de la SPA (respuesta única de 30–40 kB) |
| `GET` | `/api/config` | Devuelve la `wspr_config_t` actual como JSON |
| `POST` | `/api/config` | Actualiza la configuración desde el cuerpo JSON; guarda en NVS |
| `POST` | `/api/tx_toggle` | Alterna `tx_enabled`; guarda en NVS |
| `GET` | `/api/status` | Devuelve una instantánea del estado en vivo como JSON |
| `GET` | `/api/wifi_scan` | Lanza escaneo Wi-Fi; devuelve array JSON de APs |
| `POST` | `/api/reset` | Programa `esp_restart()` |

#### Ejemplo: respuesta GET /api/status

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

#### Ejemplo: cuerpo POST /api/config

```json
{
  "callsign": "LU1ABC",
  "locator": "GF05",
  "power_dbm": 23,
  "wifi_ssid": "MiRed",
  "wifi_pass": "secreto",
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

Cuando `CONFIG_WSPR_HTTP_AUTH_ENABLE` está activo, todos los endpoints requieren una cabecera `Authorization: Basic <base64>`. El navegador mostrará su diálogo nativo de credenciales en el primer acceso.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Referencia de configuración

### Valores predeterminados (aplicados cuando no se encuentra configuración en NVS)

| Parámetro | Predeterminado | Notas |
|---|---|---|
| Indicativo | `CONFIG_WSPR_DEFAULT_CALLSIGN` | Configurado en menuconfig (predeterminado: `N0CALL`) |
| Localizador | `CONFIG_WSPR_DEFAULT_LOCATOR` | 4 o 6 chars, en menuconfig (predeterminado: `AA00`) |
| Potencia | `CONFIG_WSPR_DEFAULT_POWER_DBM` | En menuconfig (predeterminado: 23 dBm) |
| Servidor NTP | `pool.ntp.org` | — |
| Bandas habilitadas | 40 m, 20 m | — |
| TX habilitada | `false` | Debe habilitarse manualmente tras el primer arranque |
| Salto habilitado | `false` | — |
| Intervalo de salto | 120 s | — |
| Ciclo de trabajo TX | 20% | Estándar WSPR |
| Calibración XTAL | 0 ppb | Sin corrección |
| Región UAIRO | 1 (Europa) | — |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Compilación y grabación

### Requisitos previos

- **ESP-IDF v5.0** o posterior ([Guía de instalación](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- Python 3.8+
- CMake 3.16+
- Un puerto serie con el ESP32 conectado

### Clonar y configurar

```bash
git clone https://github.com/hiperiondev/ESP32_WSPR.git
cd ESP32_WSPR
idf.py set-target esp32
idf.py menuconfig
```

### Configurar (menuconfig)

Ejecuta `idf.py menuconfig` y navega a **WSPR Transmitter** para establecer todos los parámetros (ver [Opciones de menuconfig](#opciones-de-menuconfig-kconfig) abajo). Como mínimo, configura tu indicativo, localizador y asignaciones de pines GPIO para tu hardware.

### Compilar

```bash
idf.py build
```

### Grabar y monitorear

```bash
idf.py flash monitor
```

O grabar y monitorear por separado:

```bash
idf.py flash
idf.py monitor
```

Usa `Ctrl+]` para salir del monitor.

### Tabla de particiones

La tabla de particiones predeterminada de ESP-IDF incluye una partición `nvs` suficientemente grande para el blob `wspr_config_t`. No se necesita tabla de particiones personalizada.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Opciones de menuconfig (Kconfig)

Todos los parámetros de tiempo de compilación están expuestos a través de `Kconfig.projbuild` bajo el menú **"WSPR Transmitter"**.

### Valores predeterminados de estación

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Indicativo predeterminado | `CONFIG_WSPR_DEFAULT_CALLSIGN` | `N0CALL` | Predeterminado compilado; sobreescrito por NVS |
| Localizador predeterminado | `CONFIG_WSPR_DEFAULT_LOCATOR` | `AA00` | Maidenhead de 4 o 6 chars; sobreescrito por NVS |
| Potencia predeterminada (dBm) | `CONFIG_WSPR_DEFAULT_POWER_DBM` | `23` | Entero 0–60; sobreescrito por NVS |

### Oscilador — Si5351A

| Opción | Símbolo | Predeterminado |
|---|---|---|
| Número de puerto I2C | `CONFIG_SI5351_I2C_PORT` | 0 |
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
| Bit de dirección de filtro 0 (LSB) | `CONFIG_FILTER_GPIO_A` | 25 |
| Bit de dirección de filtro 1 | `CONFIG_FILTER_GPIO_B` | 26 |
| Bit de dirección de filtro 2 (MSB) | `CONFIG_FILTER_GPIO_C` | 27 |

Un `static_assert` en `gpio_filter.c` verifica que no haya dos GPIO de filtro que compartan el mismo número de pin.

### Tiempo de establecimiento del relé LPF

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Tiempo de establecimiento (ms) | `CONFIG_WSPR_LPF_SETTLE_MS` | 10 | Retardo tras escritura GPIO antes de habilitar la salida RF. Rango: 5–100 ms. |

### Punto de acceso Wi-Fi (modo fallback)

| Opción | Símbolo | Predeterminado |
|---|---|---|
| SSID del AP | `CONFIG_WSPR_AP_SSID` | `WSPR-Config` |
| Contraseña del AP | `CONFIG_WSPR_AP_PASS` | (vacío = red abierta) |

### Sincronización horaria

| Opción | Símbolo | Descripción |
|---|---|---|
| Modo NTP | `CONFIG_WSPR_TIME_NTP` | Usar SNTP (requiere Wi-Fi) |
| Modo GPS | `CONFIG_WSPR_TIME_GPS` | Usar receptor GPS NMEA por UART |

#### Opciones específicas de GPS (si se selecciona modo GPS)

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Puerto UART | `CONFIG_GPS_UART_PORT` | 1 | Índice de periférico UART del ESP32 |
| GPIO RX | `CONFIG_GPS_RX_GPIO` | 16 | GPIO que recibe NMEA desde TX del GPS |
| GPIO TX | `CONFIG_GPS_TX_GPIO` | 17 | GPIO que transmite al RX del GPS |
| Velocidad en baudios | `CONFIG_GPS_BAUD_RATE` | 9600 | La mayoría de los módulos GPS usan 9600 por defecto |
| GPIO PPS | `CONFIG_GPS_PPS_GPIO` | -1 | Poner a un GPIO válido para soporte PPS; -1 para deshabilitar |

### Autenticación HTTP Básica

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Habilitar autenticación | `CONFIG_WSPR_HTTP_AUTH_ENABLE` | `n` | Requerir usuario/contraseña para todos los endpoints HTTP |
| Nombre de usuario | `CONFIG_WSPR_HTTP_AUTH_USER` | `admin` | Usado solo cuando la autenticación está habilitada |
| Contraseña | `CONFIG_WSPR_HTTP_AUTH_PASS` | `wspr` | Usado solo cuando la autenticación está habilitada |

### Idioma de la interfaz web

| Opción | Símbolo |
|---|---|
| Inglés | `CONFIG_WEBUI_LANG_EN` |
| Español | `CONFIG_WEBUI_LANG_ES` |

### Opciones de depuración

| Opción | Símbolo | Predeterminado | Descripción |
|---|---|---|---|
| Watchdog de tarea | `CONFIG_WSPR_TASK_WDT_ENABLE` | `n` | Registrar la tarea del planificador con el watchdog de tarea IDF. Requiere timeout WDT ≥ 2 s en sdkconfig. |
| Log de sobrecargas de símbolo | `CONFIG_WSPR_SYMBOL_OVERRUN_LOG` | `y` | Emitir ESP_LOGW si un símbolo se programa >10 ms después de su deadline. |

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Banco de filtros de paso bajo

Un filtro de paso bajo (LPF) es **legalmente requerido** en prácticamente todas las jurisdicciones para suprimir armónicos antes de conectar la salida del oscilador a una antena. La salida del Si5351A es una onda cuadrada rica en armónicos impares; sin filtrado, el tercer armónico de una señal de 7 MHz aparecería en 21 MHz, y así sucesivamente.

El firmware controla un bus de dirección binaria de 3 bits (GPIO_A = bit 0, GPIO_B = bit 1, GPIO_C = bit 2) que selecciona una de hasta ocho posiciones de filtro a través de un decodificador BCD (p. ej. 74HC138) o una placa de driver de relés. El mapeo de banda a ID de filtro está definido en `BAND_FILTER[]` en `config.c`:

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

Tras llamar a `gpio_filter_select()`, el firmware inserta un retardo (`CONFIG_WSPR_LPF_SETTLE_MS`, predeterminado 10 ms) antes de habilitar la salida del oscilador para permitir que los contactos del relé se estabilicen físicamente. Esto previene clics de clave y emisiones fuera de banda durante la transición del relé. El tiempo de establecimiento es configurable en menuconfig.

Puedes modificar libremente la tabla `BAND_FILTER[]` para adaptarla al diseño físico de tu hardware LPF sin tocar ningún otro código.

### Diseño de LPF recomendado

Para cada filtro, diseña un **filtro de paso bajo en red T** (en lugar de una red Pi) para mejor eficiencia. QRP Labs, W3NQN y numerosas calculadoras LC en línea pueden generar valores de componentes para cada frecuencia de corte. Frecuencias de corte típicas:

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

El **Si5351A** es un generador de reloj programable por I2C muy versátil fabricado por Silicon Laboratories (ahora Skyworks). Es el oscilador preferido para beacons WSPR porque:

- Soporta **síntesis PLL fraccionaria-N**, permitiendo resolución de frecuencia sub-Hz en todo el espectro HF.
- Es detectable por I2C (sondeo ACK), por lo que el firmware puede confirmar su presencia al arrancar.
- Está disponible en placas breakout económicas de Adafruit, QRP Labs y numerosos proveedores.
- La potencia de salida es adecuada para uso directo con antena con un LPF apropiado (~10 dBm = 10 mW).

**Resumen de arquitectura (del datasheet del Si5351A):**

El Si5351A consiste en un oscilador de referencia (cristal de 25 o 27 MHz), dos PLLs (PLLA y PLLB) que multiplican el cristal a 600–900 MHz, y hasta 3 divisores de salida MultiSynth (CLK0–CLK2). Este firmware usa solo CLK0 bloqueado a PLLA.

El multiplicador PLL `a` se calcula como `875 MHz / f_xtal_MHz` (dando un VCO cercano a 875 MHz), y el divisor de salida se calcula como `VCO_Hz / f_out_Hz` con un par numerador/denominador fraccional (p2, p3) que proporciona la resolución de frecuencia en mili-Hz requerida para el espaciado de símbolos WSPR. Un divisor `R` post-divisor se aplica automáticamente para frecuencias de salida por debajo de 500 kHz (bandas de 2200 m y 630 m).

Un **caché de banda** (`si5351_band_cache_t`) precomputa todos los coeficientes de la cadena divisora una vez por cada cambio de frecuencia portadora. Dentro de una ventana WSPR (162 símbolos) solo se reescriben los seis registros del numerador PLL por símbolo (~1 transacción I2C cada 683 ms), minimizando el tráfico del bus y la latencia.

### AD9850 DDS

El **AD9850** es un circuito integrado Sintetizador Digital Directo (DDS) de Analog Devices, que opera con un reloj de referencia de 125 MHz. Usa una palabra de sintonización de frecuencia (FTW) de 32 bits para establecer la frecuencia de salida: `FTW = f_salida × 2^32 / f_ref`. El firmware lo calcula en aritmética entera de 32 bits usando constantes preescaladas para evitar desbordamiento de 64 bits en tiempo de ejecución.

Debido a que el bus del AD9850 es de solo escritura (sin lectura de retorno), el firmware no puede detectar su presencia y **siempre asume que está presente** tras el fallo del sondeo del Si5351 (cuando `CONFIG_OSCILLATOR_ASSUME_AD9850` está habilitado). La salida es una onda sinusoidal reconstruida por un DAC de 10 bits (~1 V p-p en 50 Ω). Se recomienda encarecidamente un amplificador de potencia externo con el AD9850.

La transferencia bit-bang del AD9850 está protegida por una sección crítica FreeRTOS `portMUX` para evitar interrupciones por el segundo núcleo del ESP32 en medio de una transferencia.

### Modo ficticio (Dummy)

Si no se encuentra ningún oscilador (el sondeo I2C del Si5351 falla y `CONFIG_OSCILLATOR_ASSUME_AD9850` está deshabilitado), el firmware entra en **modo ficticio**: todas las llamadas `oscillator_*` devuelven `ESP_OK` silenciosamente, y la WebUI muestra una advertencia. Esto permite que el resto del firmware (Wi-Fi, WebUI, sincronía de tiempo) siga funcionando para propósitos de depuración.

---

## Sincronización horaria

La hora UTC precisa es **esencial** para WSPR. Las transmisiones que comiencen con más de ±1–2 segundos del límite del minuto par no serán decodificadas. El firmware soporta dos fuentes de tiempo, seleccionadas en tiempo de compilación vía Kconfig.

### Modo NTP (`CONFIG_WSPR_TIME_NTP`)

Usa el cliente SNTP de ESP-IDF (`esp_sntp`) para consultar periódicamente un servidor NTP. El callback SNTP establece `_synced = true` y el `scheduler_task` del firmware se desbloquea. Precisión típica: 1–50 ms, más que suficiente para WSPR.

El nombre de host del servidor NTP es configurable por instancia desde la WebUI (predeterminado: `pool.ntp.org`). Los cambios se aplican inmediatamente vía `time_sync_restart_ntp()` sin reinicio del dispositivo.

### Modo GPS (`CONFIG_WSPR_TIME_GPS`)

Una tarea FreeRTOS en segundo plano (`gps_task`, pila 6 kB, prioridad 5) lee sentencias NMEA-0183 de un módulo GPS conectado por UART. El firmware acepta los cuatro tipos de sentencias:

- `$GPRMC` / `$GNRMC` — sentencias RMC de una y múltiples constelaciones (fecha + hora + flag de validez)
- `$GPZDA` / `$GNZDA` — sentencias ZDA de una y múltiples constelaciones (fecha + hora, año de 4 dígitos)

Cada sentencia se valida con un checksum XOR NMEA completo antes de parsear. La variable de entorno `TZ` se fuerza a `"UTC0"` antes de cualquier llamada a `mktime()` para evitar que se aplique el desplazamiento de zona horaria local a las marcas de tiempo UTC de NMEA.

Precisión típica: ±1 s (limitada por la tasa de sentencias NMEA de 1 Hz y la latencia UART).

#### Soporte PPS GPS

Cuando `CONFIG_GPS_PPS_GPIO` se establece a un número GPIO válido (≥ 0), una ISR de flanco ascendente (`pps_isr`) se activa en cada pulso PPS y pone a cero el componente sub-segundo del reloj del sistema. Esto reduce la incertidumbre de tiempo de ~10 ms (latencia UART) a unos pocos microsegundos. La sentencia NMEA sigue proporcionando el valor correcto del segundo UTC; PPS solo mejora la precisión sub-segundo. Establece `CONFIG_GPS_PPS_GPIO = -1` (el predeterminado) para deshabilitar PPS.

El modo GPS es completamente independiente de Wi-Fi, lo que lo hace adecuado para instalaciones de balizas portátiles o remotas sin acceso a internet.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Wi-Fi y red

### Modo STA (estación)

Si se configura un SSID Wi-Fi, el firmware intenta asociarse con el punto de acceso en modo STA. Se realizan hasta 5 reintentos de conexión dentro de una ventana de 15 segundos. En caso de éxito:
- El firmware procede con la sincronía NTP (si está configurado)
- La interfaz web se vuelve accesible en la IP asignada por DHCP
- La IP se muestra en el log de la consola serie y en `/api/status`

### Modo fallback AP

Si no se configura ningún SSID, o si todos los intentos de conexión STA fallan, el firmware arranca un punto de acceso suave:
- SSID: `CONFIG_WSPR_AP_SSID` (predeterminado `WSPR-Config`)
- Contraseña: `CONFIG_WSPR_AP_PASS` (si ≥ 8 caracteres, usa WPA2-PSK; de lo contrario red abierta)
- IP: `192.168.4.1`
- Hasta 4 clientes simultáneos

### Reconexión en segundo plano

Cuando está en modo solo AP con credenciales STA guardadas, un `esp_timer` periódico se activa cada **5 minutos** y reintenta la conexión STA (transiciona al modo APSTA, llama a `esp_wifi_connect()`). Esto permite que el beacon se recupere automáticamente cuando el router doméstico vuelve a estar en línea tras un corte de energía.

### Escaneo Wi-Fi

El endpoint `GET /api/wifi_scan` lanza un escaneo bloqueante (~2 s) que devuelve un array JSON de puntos de acceso cercanos con SSID, RSSI y tipo de seguridad. En modo solo AP, el escaneo eleva temporalmente al modo APSTA, espera **300 ms** para que la interfaz STA se inicialice, escanea, y luego vuelve al modo AP. Los resultados se limitan a 20 entradas para eficiencia del payload HTTP. Las redes ocultas (SSID vacío) se filtran. Un mutex FreeRTOS previene llamadas de escaneo concurrentes.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Frecuencias de banda WSPR y regiones UAIRO/IARU

El firmware almacena las frecuencias de marcación para las 12 bandas y las 3 regiones UAIRO en la tabla `BAND_FREQ_HZ[3][BAND_COUNT]` en `config.c`. La función en línea `config_band_freq_hz(region, banda)` selecciona la entrada correcta con acceso a array verificado por límites.

Todas las frecuencias son **frecuencias de marcación** en Hz. La frecuencia RF transmitida real es `freq_marcación + 1500 Hz + desplazamiento_símbolo` para cada símbolo.

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

**Nota sobre los 60 m:** La frecuencia WSPR en 60 m difiere entre regiones UAIRO debido a los diferentes planes nacionales de canalización. La Región 1 (Europa) usa 5.288,6 kHz. La Región 2 (Américas) usa 5.346,5 kHz según la coordinación FCC/ARRL. La Región 3 usa 5.367,0 kHz según la coordinación WIA/JARL. Siempre verifica que la operación en 60 m esté permitida bajo tu licencia nacional de radioafición.

### Selección de tu región UAIRO/IARU

Establece la región en la tarjeta **Región IARU** de la WebUI. La selección se guarda en NVS. El campo `iaru_region` en `wspr_config_t` almacena valores 1, 2 o 3; cualquier valor almacenado inválido es automáticamente corregido a 1 al arrancar por `config_load()`.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Modo de salto de frecuencia

Cuando `hop_enabled = true`, el `scheduler_task` rota por la lista de bandas habilitadas después de cada `hop_interval_sec` segundos. El estado de salto (índice de banda actual, contador de tiempo en banda) se mantiene en variables locales a la tarea y se resetea en cada arranque del firmware.

**Lógica de salto:**
1. Tras completar una transmisión, verificar si `tiempo_transcurrido >= hop_interval_sec`.
2. Si sí, avanzar el índice de banda a la siguiente banda habilitada (con wraparound).
3. Llamar a `gpio_filter_select(BAND_FILTER[nueva_banda])`.
4. Actualizar la caché de estado con el nuevo nombre de banda.
5. Si la banda cambia mientras está pre-armado, reprogramar inmediatamente el oscilador y el filtro para la nueva banda.

Si solo hay una banda habilitada, el salto queda efectivamente deshabilitado (la única banda siempre es "la siguiente"). Si no hay ninguna banda habilitada, el firmware cae automáticamente a 40 m. El intervalo mínimo de salto es 120 s según el protocolo; los valores menores almacenados en NVS se limitan en tiempo de carga.

---

## Ciclo de trabajo TX

El protocolo WSPR recomienda que las estaciones transmitan no más del 20% del tiempo, dejando el otro 80% para recepción. Esto es tanto una buena práctica operativa como una cortesía para otros usuarios que puedan estar escuchando en la misma frecuencia.

El firmware implementa el ciclo de trabajo como un **acumulador determinista**: antes de cada slot de 2 minutos, `acumulador += tx_duty_pct`. Cuando el acumulador alcanza o supera 100, se usa el slot para transmisión y se resta 100. Esto produce una distribución perfectamente uniforme sin aleatoriedad.

- `tx_duty_pct = 0`: Nunca transmitir.
- `tx_duty_pct = 20`: Transmitir 1 de cada 5 slots — el estándar de la comunidad WSPR.
- `tx_duty_pct = 100`: Transmitir en cada slot.

---

## Calibración de cristal

Incluso los cristales de alta calidad se desvían de su frecuencia nominal por decenas a cientos de partes por millón. Para WSPR, la frecuencia de transmisión debe estar dentro de ±200 Hz de la frecuencia de marcación (la ventana del receptor es de solo unos ±100 Hz), por lo que la calibración es importante.

El campo `xtal_cal_ppb` almacena el desplazamiento de calibración en **partes por mil millones (ppb)**.

### Cómo se aplica la calibración

**Si5351A:** La corrección en ppb se aplica a la frecuencia objetivo interna del VCO antes de calcular el divisor de salida MS0. Un `cal_ppb` positivo significa que el cristal es rápido (la frecuencia de salida sería demasiado alta), por lo que el VCO efectivo se reduce. Aplicar la calibración invalida la caché de banda, por lo que la siguiente llamada a `oscillator_set_freq()` recalcula la cadena divisora.

**AD9850:** La corrección en ppb escala las constantes de conversión frecuencia-a-palabra-de-sintonización precomputadas antes de calcular cada palabra de frecuencia.

En ambos casos, si una actualización de calibración llega durante una ventana de transmisión WSPR, se **difiere** y es aplicada por `oscillator_tx_end()` tras el símbolo final para evitar transitorios de frecuencia a mitad de transmisión.

### Cómo medir el desplazamiento de tu cristal

1. Establece `xtal_cal_ppb = 0` en la WebUI.
2. Transmite en una banda bien calibrada (p. ej. 40 m o 20 m).
3. Usa un receptor SDR calibrado para medir la frecuencia central real de tu señal.
4. Calcula: `cal_ppb = (medida_Hz - nominal_Hz) × 1e9 / nominal_Hz`
5. Si tu señal está 50 Hz alta en 14.097.100 Hz: `cal_ppb = 50 × 1e9 / 14097100 ≈ 3547 ppb`
6. Introduce un valor **negativo** para bajar la salida (el cristal funciona rápido).
7. Guarda y verifica en la siguiente transmisión.

Alternativamente, los reportes de recepción de WSPRnet incluyen el desplazamiento de frecuencia en Hz medido por la estación receptora; esto puede usarse directamente para estimar el error de calibración.

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Estado de implementación

| Característica | Estado |
|---|---|
| Codificador WSPR Tipo 1 (indicativo + localizador + potencia) | ✅ Completo |
| Codificador WSPR Tipo 2 (indicativo compuesto con '/') | ✅ Completo |
| Codificador WSPR Tipo 3 (acompañante localizador 6 chars) | ✅ Completo |
| Alternancia Tipo 1/Tipo 3 (localizador 6 chars) | ✅ Completo |
| Alternancia Tipo 2/Tipo 3 (indicativo compuesto) | ✅ Completo |
| Driver de oscilador Si5351A (I2C, autodetección) | ✅ Completo |
| Driver de oscilador AD9850 DDS (bit-bang GPIO) | ✅ Completo |
| Modo ficticio del oscilador (sin hardware) | ✅ Completo |
| Driver de banco LPF GPIO de 3 bits | ✅ Completo |
| Sincronía de tiempo NTP (SNTP) | ✅ Completo |
| Sincronía de tiempo GPS (NMEA UART, $GPRMC/$GNRMC/$GPZDA/$GNZDA) | ✅ Completo |
| Precisión sub-segundo GPS PPS (ISR de flanco ascendente) | ✅ Completo |
| Modo Wi-Fi STA | ✅ Completo |
| Fallback Wi-Fi AP (192.168.4.1) | ✅ Completo |
| Temporizador de reconexión Wi-Fi en segundo plano | ✅ Completo |
| Configuración persistente NVS (esquema v5) | ✅ Completo |
| Interfaz web SPA embebida | ✅ Completo |
| API REST (config, status, tx_toggle, reset, scan) | ✅ Completo |
| Autenticación HTTP Básica | ✅ Completo |
| Selección de región UAIRO para 60 m | ✅ Completo |
| Soporte de 12 bandas (2200 m – 10 m) | ✅ Completo |
| Salto de frecuencia | ✅ Completo |
| Ciclo de trabajo TX (acumulador determinista) | ✅ Completo |
| Calibración de cristal (ppb), diferida durante TX | ✅ Completo |
| Pre-armado del oscilador en phase=0 | ✅ Completo |
| WebUI en inglés y español | ✅ Completo |
| Enlace de spots WSPRnet | ✅ Completo |
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

Características y mejoras planeadas para futuras versiones:

- [ ] **RTC DS3231** — cronometraje sin conexión cuando Wi-Fi y GPS no están disponibles
- [ ] **Actualización OTA de firmware** — actualización de firmware por aire desde la WebUI
- [ ] **GPIO de habilitación de amplificador de potencia** — encender/apagar un PA alrededor de las transmisiones
- [ ] **Programación de transmisiones** — reglas de programación por hora del día o banda/día de la semana
- [ ] **Telemetría MQTT** — publicar estado a un broker MQTT para monitoreo remoto
- [ ] **Idiomas adicionales en la UI**
- [ ] **Carga automática a WSPRnet** — carga HTTP directa de spots recibidos (requiere RX SDR)
- [ ] **Soporte de bandas de 6 m y 4 m** — extensión de la tabla de frecuencias para WSPR en VHF
- [ ] **Soporte de pantalla SPI** — pantalla de estado OLED/TFT opcional

<div align="right">
  <a href="#readme-top">
    <img src="images/backtotop.png" alt="volver arriba" width="30" height="30">
  </a>
</div>

---

## Contribuciones

Las contribuciones son lo que hace que la comunidad de código abierto sea un lugar increíble para aprender, inspirar y crear. Cualquier contribución que hagas es **muy apreciada**.

Si tienes una sugerencia que mejoraría esto, por favor haz un fork del repositorio y crea un pull request. También puedes simplemente abrir un issue con la etiqueta "enhancement". ¡No olvides darle una estrella al proyecto! ¡Muchas gracias!

1. Haz un fork (<https://github.com/hiperiondev/ESP32_WSPR/fork>)
2. Crea tu rama de característica (`git checkout -b feature/MiCaracteristica`)
3. Haz commit de tus cambios (`git commit -am 'Agregar MiCaracteristica'`)
4. Haz push a la rama (`git push origin feature/MiCaracteristica`)
5. Crea un nuevo Pull Request

### Estilo de código

- C99, convenciones de codificación ESP-IDF.
- Todos los módulos nuevos deben tener un `modulo.h` correspondiente con documentación API estilo Doxygen (ver `gpio_filter.h`, `oscillator.h` como ejemplos).
- Sin aritmética de punto flotante ni de 64 bits.
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

El algoritmo de codificación WSPR está basado en el código fuente original de WSJT-X de Joe Taylor (K1JT) y la descripción del protocolo de Andy Talbot (G4JNT). Todos los algoritmos son usados con respeto por las contribuciones de los autores originales a la radioafición.

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

- **G4JNT (Andy Talbot)** — *"The WSPR Coding Process"* (2009): la especificación no normativa definitiva del algoritmo de codificación WSPR Tipo 1. [PDF](http://www.g4jnt.com/WSPR_Coding_Process.pdf)
- **K1JT (Joe Taylor)** — Código fuente y documentación de WSJT-X. [wsjt.sourceforge.io](https://wsjt.sourceforge.io/)
- **WSPRnet** — Base de datos global de recepción WSPR y mapas. [wsprnet.org](https://www.wsprnet.org)
- **Wikipedia** — WSPR (software de radioafición). [en.wikipedia.org/wiki/WSPR](https://en.wikipedia.org/wiki/WSPR_(amateur_radio_software))
- **Scott Harden (AJ4VD)** — Notas del protocolo WSPR con renderizado Markdown del paper G4JNT. [swharden.com](https://swharden.com/software/FSKview/wspr/)
- **Lista de frecuencias WSPR** — Coordinación oficial de frecuencias WSPRnet. [wsprnet.org/drupal/node/218](https://www.wsprnet.org/drupal/node/218)
- **Documentación Wsprry Pi** — Resumen conciso de especificaciones WSPR. [wsprry-pi.readthedocs.io](https://wsprry-pi.readthedocs.io/en/latest/About_WSPR/)

### Hardware oscilador

- **Silicon Laboratories / Skyworks** — Datasheet Si5351A/B/C-B. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/Si5351-B.pdf)
- **Skyworks** — AN619: Generación manual de un mapa de registros Si5351. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)
- **Skyworks** — AN1234: Generación manual de un mapa de registros Si5351 para dispositivos 16QFN. [skyworksinc.com](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/an1234-si5351-16qfn-register-map.pdf)
- **QRP Labs** — Código demo Si5351A y teoría de síntesis. [qrp-labs.com](https://qrp-labs.com/synth/si5351ademo.html)
- **NT7S (Jason Milldrum)** — Biblioteca Si5351Arduino: driver Si5351 completo para Arduino. [github.com/etherkit/Si5351Arduino](https://github.com/etherkit/Si5351Arduino)
- **Analog Devices** — Datasheet AD9850 CMOS Complete DDS Synthesizer. [analog.com](https://www.analog.com/media/en/technical-documentation/data-sheets/ad9850.pdf)

### Proyectos ESP32 WSPR relacionados

- **danak6jq/ESP32-WSPR** — Transmisor WSPR2 autónomo completo usando ESP32 + Si5351 (ESP-IDF v3). [github.com](https://github.com/danak6jq/ESP32-WSPR)
- **mm5agm/WSPR-Multi-Band** — Beacon WSPR multibanda para ESP32 (Arduino). [github.com](https://github.com/mm5agm/WSPR-Multi-Band)
- **etherkit/JTEncode** — Biblioteca codificadora JT65/JT9/JT4/WSPR/FSQ para Arduino. [github.com](https://github.com/etherkit/JTEncode)

### Documentación ESP-IDF

- **Guía de programación ESP-IDF de Espressif**. [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
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
