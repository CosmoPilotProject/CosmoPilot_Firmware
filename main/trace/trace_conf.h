/**
 * @file    trace.c
 * @brief   Trace and debugging output implementation.
 *
 * @copyright
 * Copyright (c) 2026 CosmoPilot Project
 *
 * @license
 * This project is licensed under the MIT License.
 * See the LICENSE file in the project root for the full license text.
 *
 * @website
 * https://cosmo-pilot.com
 *
 * @contact
 * info@cosmo-pilot.com
 */

/**
 * @note
 * This trace implementation is not thread-safe; concurrent access requires synchronization.
 */

#ifndef TRACE_CONF_H
#define TRACE_CONF_H

/* --- Includes ----------------------------------------------------------------------------------------------------- */

#include "trace_level_conf.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

#define TRACE_LEVEL_VER 0u
#define TRACE_LEVEL_DEB 1u
#define TRACE_LEVEL_INF 2u
#define TRACE_LEVEL_WAR 3u
#define TRACE_LEVEL_ERR 4u
#define TRACE_LEVEL_FAT 5u

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Trace severity levels.
 */
typedef enum
{
    TRACE_VER_ENUM = TRACE_LEVEL_VER,
    TRACE_DEB_ENUM = TRACE_LEVEL_DEB,
    TRACE_INF_ENUM = TRACE_LEVEL_INF,
    TRACE_WAR_ENUM = TRACE_LEVEL_WAR,
    TRACE_ERR_ENUM = TRACE_LEVEL_ERR,
    TRACE_FAT_ENUM = TRACE_LEVEL_FAT
} trace_level_t;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Output a formatted trace message.
 * @note    Invalid levels, NULL module/format strings and formatting errors discard the message.
 * @param   level Trace severity level.
 * @param   module_name Trace module name.
 * @param   fmt Format string.
 * @return  None.
 */
void trace_log(trace_level_t level, const char *module_name, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

/* Global Defines --------------------------------------------------------------------------------------------------- */

#ifndef TRACE_NAME
#define TRACE_NAME UNKNOWN
#endif

#ifndef TRACE_LEVEL
#define TRACE_LEVEL TRACE_LEVEL_DEB
#endif

#define TRACE_STR_HELPER(x) #x
#define TRACE_STR(x)        TRACE_STR_HELPER(x)

#if TRACE_ENABLE

#if TRACE_LEVEL <= TRACE_LEVEL_VER
#define TRACE_VER(...) trace_log(TRACE_VER_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_VER(...) ((void) 0)
#endif

#if TRACE_LEVEL <= TRACE_LEVEL_DEB
#define TRACE_DEB(...) trace_log(TRACE_DEB_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_DEB(...) ((void) 0)
#endif

#if TRACE_LEVEL <= TRACE_LEVEL_INF
#define TRACE_INF(...) trace_log(TRACE_INF_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_INF(...) ((void) 0)
#endif

#if TRACE_LEVEL <= TRACE_LEVEL_WAR
#define TRACE_WAR(...) trace_log(TRACE_WAR_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_WAR(...) ((void) 0)
#endif

#if TRACE_LEVEL <= TRACE_LEVEL_ERR
#define TRACE_ERR(...) trace_log(TRACE_ERR_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_ERR(...) ((void) 0)
#endif

#if TRACE_LEVEL <= TRACE_LEVEL_FAT
#define TRACE_FAT(...) trace_log(TRACE_FAT_ENUM, TRACE_STR(TRACE_NAME), __VA_ARGS__)
#else
#define TRACE_FAT(...) ((void) 0)
#endif

#else
#define TRACE_VER(...) ((void) 0)
#define TRACE_DEB(...) ((void) 0)
#define TRACE_INF(...) ((void) 0)
#define TRACE_WAR(...) ((void) 0)
#define TRACE_ERR(...) ((void) 0)
#define TRACE_FAT(...) ((void) 0)
#endif

#endif