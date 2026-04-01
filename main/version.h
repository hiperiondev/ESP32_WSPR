/**
 * @file version.h
 * @brief Firmware version constants and version-string helper macros.
 * @copyright 2026 Emiliano Augusto Gonzalez (egonzalez.hiperion@gmail.com)
 * @see https://github.com/hiperiondev/ESP32_WSPR
 * @license GNU General Public License v3.0
 *
 * @details
 * This header defines the three-component semantic version number of the
 * ESP32 WSPR transmitter firmware and provides a pair of preprocessor macros
 * that convert the integer constants into a NUL-terminated string literal
 * suitable for use in log messages and HTTP responses.
 *
 * @par Semantic versioning
 * The version number follows the Semantic Versioning 2.0.0 scheme
 * (https://semver.org/):
 *  - @ref VERSION_MAYOR (MAJOR): incremented for incompatible API changes or
 *    fundamental architectural changes that break backwards compatibility
 *    with previously stored configurations or on-air behaviour.
 *  - @ref VERSION_MINOR (MINOR): incremented when new functionality is added
 *    in a backwards-compatible manner, or when a significant correction to
 *    operational behaviour is made.
 *  - @ref VERSION_PATCH (PATCH): incremented for minor backwards-compatible
 *    bug fixes and cosmetic changes.
 *
 * @par Usage
 * Include this header and use @ref FW_VERSION_STRING wherever a human-readable
 * version string is needed:
 * @code
 *   #include "version.h"
 *   ESP_LOGI(TAG, "Firmware version: %s", FW_VERSION_STRING);
 * @endcode
 * The macro expands at compile time to a string literal such as @c "1.0.0".
 * No runtime overhead is incurred.
 */

#ifndef VERSION_H_
#define VERSION_H_

/**
 * @defgroup version Firmware version
 * @{
 */

/**
 * @brief Major version number.
 *
 * Incremented when a change introduces incompatibilities with previous firmware
 * versions that would prevent a seamless upgrade.  Examples of major changes:
 * restructuring the NVS schema in a way that cannot be migrated automatically,
 * changing the WSPR encoding algorithm, or breaking the web API contract.
 *
 * When the major version is incremented, the minor and patch counters must
 * be reset to zero.
 *
 * Current value: @c 1.
 */
#define VERSION_MAYOR 1

/**
 * @brief Minor version number.
 *
 * Incremented when new features or significant corrections are added in a
 * backwards-compatible manner.  Examples: adding a new band, supporting a new
 * oscillator type, adding a new web API endpoint, or correcting a WSPR encoding
 * bug that changes on-air behaviour.
 *
 * When the minor version is incremented, the patch counter must be reset to zero.
 *
 * Current value: @c 0.
 */
#define VERSION_MINOR 0

/**
 * @brief Patch version number.
 *
 * Incremented for minor backwards-compatible bug fixes, code quality improvements,
 * documentation updates, or cosmetic changes that do not affect on-air behaviour
 * or user-visible functionality.
 *
 * Current value: @c 0.
 */
#define VERSION_PATCH 0

/**
 * @brief Stringify helper — converts a macro token to a string literal.
 *
 * This two-level macro expansion is required so that numeric macros such as
 * @ref VERSION_MAYOR are expanded to their values before stringification.
 * Using @c #x directly on an argument that is itself a macro would produce
 * the macro name rather than its value.
 *
 * @param x  Any preprocessor token.  The token is expanded before conversion.
 * @return   String literal containing the expanded form of @p x.
 */
#define STR_HELPER(x) #x

/**
 * @brief Expand and stringify a preprocessor token.
 *
 * Calls @ref STR_HELPER after one level of macro expansion so that numeric
 * macros are converted to their decimal string representation.
 *
 * @param x  Preprocessor token to expand and stringify.
 * @return   String literal of the expanded value of @p x.
 */
#define STR(x) STR_HELPER(x)

/**
 * @brief Complete firmware version as a NUL-terminated string literal.
 *
 * Constructed at compile time by concatenating the stringified major, minor,
 * and patch version numbers separated by dots.  Example: @c "1.0.0".
 *
 * The string is a compile-time constant with no runtime storage overhead.
 * It can be used directly as a @c printf / @c ESP_LOGI format argument:
 * @code
 *   ESP_LOGI("app", "WSPR fw v%s", FW_VERSION_STRING);
 * @endcode
 * or embedded in larger string literals via concatenation:
 * @code
 *   static const char banner[] = "ESP32-WSPR/" FW_VERSION_STRING;
 * @endcode
 */
#define FW_VERSION_STRING STR(VERSION_MAYOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH)

/** @} */

#endif /* VERSION_H_ */
