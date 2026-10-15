
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

#ifndef TRACE_LEVEL_CONF_H
#define TRACE_LEVEL_CONF_H

// --- Includes --------------------------------------------------------------------------------------------------------

/* Global Defines --------------------------------------------------------------------------------------------------- */

#define TRACE_ENABLE 1

#define MAIN_TRACE_LEVEL                  TRACE_LEVEL_INF
#define MAIN_HELPERS_TRACE_LEVEL          TRACE_LEVEL_INF
#define MOBILE_APP_DISPATCH_TRACE_LEVEL   TRACE_LEVEL_INF
#define MOBILE_APP_HANDLERS_TRACE_LEVEL   TRACE_LEVEL_INF
#define COMMON_FUNCTIONS_TRACE_LEVEL      TRACE_LEVEL_INF
#define BLUETOOTH_TRACE_LEVEL             TRACE_LEVEL_INF
#define JOYSTICK_TRACE_LEVEL              TRACE_LEVEL_INF
#define TRACKING_TRACE_LEVEL              TRACE_LEVEL_INF
#define CELESTIAL_OBJECTS_TRACE_LEVEL     TRACE_LEVEL_INF
#define TELESCOPE_COORDINATES_TRACE_LEVEL TRACE_LEVEL_INF
#define TELESCOPE_ALIGNMENT_TRACE_LEVEL   TRACE_LEVEL_INF
#define TELESCOPE_STEPPER_TRACE_LEVEL     TRACE_LEVEL_INF
#define FLASH_TRACE_LEVEL                 TRACE_LEVEL_INF

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif
