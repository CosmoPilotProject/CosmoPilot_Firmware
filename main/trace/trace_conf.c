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

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "trace_conf.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Trace message buffer size.
 */
#define BUFFER_SIZE 128

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Transmit a trace message over UART.
 * @param   msg Pointer to message string.
 * @return  None.
 */
static void trace_write(const char *msg);

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace level name lookup table.
 */
static const char *level_str[] = {"VER", "DEB", "INF", "WAR", "ERR", "FAT"};

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void trace_log(trace_level_t level, const char *module_name, const char *fmt, ...)
{
    char buffer[BUFFER_SIZE];
    int len;
    va_list args;

    if ((unsigned) level >= sizeof(level_str) / sizeof(level_str[0]) || module_name == NULL || fmt == NULL)
    {
        return;
    }

    len = snprintf(buffer, sizeof(buffer), "[%s][%s] ", level_str[level], module_name);
    if (len < 0)
    {
        return;
    }

    /* A truncated prefix fills the buffer; its would-have-written length is not a valid offset. */
    if ((size_t) len < sizeof(buffer))
    {
        va_start(args, fmt);
        int message_len = vsnprintf(&buffer[len], sizeof(buffer) - (size_t) len, fmt, args);
        va_end(args);
        if (message_len < 0)
        {
            return;
        }
    }

    /* Use the actual terminated length, including when either formatter truncated its output. */
    strncat(buffer, "\r\n", sizeof(buffer) - strlen(buffer) - 1);

    trace_write(buffer);
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static void trace_write(const char *msg)
{
    fputs(msg, stdout);
}
