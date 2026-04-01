/**
 * @file webui_strings.h
 * @author Emiliano Augusto Gonzalez
 * @copyright Copyright (c) 2026 Emiliano Augusto Gonzalez
 * @brief ESP32 WSPR project
 * @see https://github.com/hiperiondev/ESP32_WSPR
 *
 * @license GNU General Public License v3.0
 */

#pragma once

#if defined(CONFIG_WEBUI_LANG_EN)
#include "webui_en.h"
#elif defined(CONFIG_WEBUI_LANG_ES)
#include "webui_es.h"
#else
// Fall back to English (the reference language)
#warning "Unknown WEBUI language config -- defaulting to English"
#include "webui_en.h"
#endif
