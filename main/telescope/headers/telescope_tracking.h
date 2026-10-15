/**
 * @file    telescope_tracking.h
 * @brief   Fixed right ascension and declination tracking interface.
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

#ifndef TELESCOPE_TRACKING_H
#define TELESCOPE_TRACKING_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Track a fixed RA/Dec target synchronously in the application task.
 *
 * Continues through BLE disconnect; pauses movement below the tracking horizon.
 * Any pending CMD interrupts between blocking moves and remains for the caller.
 * Refresh timing is sampled once. Invalid target/timing values are traced and
 * rejected before activation; exits after activation restore acceleration/status.
 * @param   radec Target right ascension in hours and declination in degrees.
 * @return  None.
 */
void RA_Dec_tracking(RADec radec);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_TRACKING_H
