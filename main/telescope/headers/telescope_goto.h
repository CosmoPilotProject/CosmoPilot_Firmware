/**
 * @file    telescope_goto.h
 * @brief   Celestial target GoTo positioning and visibility interface.
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

#ifndef TELESCOPE_GOTO_H
#define TELESCOPE_GOTO_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Move to equatorial coordinates using blocking motor control.
 * @param   ra Target right ascension in hours.
 * @param   dec Target declination in degrees.
 * @param   speed1_alt Altitude motor speed in degrees per second.
 * @param   speed2_az Azimuth motor speed in degrees per second.
 * @return  True if accepted, including fractional-step accumulation; false if movement was rejected.
 */
bool move_to_RA_Dec(double ra, double dec, float speed1_alt, float speed2_az);

/**
 * @brief   Check target altitude and report visibility when tracking is inactive.
 * @param   radec Target right ascension in hours and declination in degrees.
 * @param   visibility_min_altitude Minimum visible altitude in degrees.
 * @return  True if the target altitude meets or exceeds the threshold.
 */
bool is_object_visible(RADec radec, float visibility_min_altitude);

/**
 * @brief   Request a GoTo for visible application-provided equatorial coordinates.
 * @param   ra_hours Right ascension hour component.
 * @param   ra_minutes Right ascension minute component.
 * @param   ra_seconds Right ascension second component.
 * @param   dec_degrees Declination degree component.
 * @param   dec_minutes Declination arcminute component.
 * @param   dec_seconds Declination arcsecond component.
 * @return  True if the move is accepted, including interruption; false otherwise.
 */
bool goto_app_object(float ra_hours, float ra_minutes, float ra_seconds, float dec_degrees, float dec_minutes,
                     float dec_seconds);

/**
 * @brief   Request a GoTo for a supported solar-system object.
 * @param   object_name NUL-terminated object name, matched without case sensitivity.
 * @return  True if the visible target's move is accepted, including interruption; false otherwise.
 */
bool goto_planet(const char *object_name);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_GOTO_H
