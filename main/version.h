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

#ifndef VERSION_H_
#define VERSION_H_

/**
 * @def VERSION_MAYOR
 * @brief Indicate a really big change that can cause a incompatibilities with previous versions.
 */
#define VERSION_MAYOR 1

/**
 * @def VERSION_MINOR
 * @brief Indicate some change on API or opcode or very important correction in functionality
 */
#define VERSION_MINOR 0

/**
 * @def VERSION_PATCH
 * @brief Indicate some minor change or correction
 */
#define VERSION_PATCH 0

////////////////////

#define STR_HELPER(x) #x
#define STR(x)        STR_HELPER(x)
#define FW_VERSION_STRING STR(VERSION_MAYOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH)

#endif /* VERSION_H_ */
