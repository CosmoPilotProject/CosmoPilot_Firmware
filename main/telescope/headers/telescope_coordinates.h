/**
 * @file    telescope_coordinates.h
 * @brief   Astronomical coordinate types and angular conversion interface.
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

#ifndef TELESCOPE_COORDINATES_H
#define TELESCOPE_COORDINATES_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Right ascension and declination split into sexagesimal components.
 */
typedef struct
{
    int ra_hours;      /**< Whole right ascension hours. */
    int ra_minutes;    /**< Whole right ascension minutes. */
    double ra_seconds; /**< Right ascension seconds, including the fractional part. */

    int dec_degrees;       /**< Signed whole declination degrees. */
    int dec_arcminutes;    /**< Whole arcminutes; carries the sign when degrees are zero. */
    double dec_arcseconds; /**< Fractional arcseconds; carries the sign when degrees and minutes are zero. */
} RA_Dec_Split;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Convert degrees to radians.
 * @param   degrees Angle in degrees.
 * @return  Angle in radians.
 */
double deg_to_rad(double degrees);

/**
 * @brief   Convert radians to degrees.
 * @param   radians Angle in radians.
 * @return  Angle in degrees.
 */
double rad_to_deg(double radians);

/**
 * @brief   Convert equatorial coordinates to horizontal coordinates.
 * @param   radec Right ascension in hours and declination in degrees.
 * @param   latitude Observer latitude, in degrees.
 * @param   lst Local sidereal time, in hours.
 * @return  Altitude and azimuth in degrees, with azimuth normalized to [0, 360).
 *          Both fields are NaN on nonfinite input or calculation failure; callers must reject them.
 */
AltAz radec_to_altaz(RADec radec, double latitude, double lst);

/**
 * @brief   Convert horizontal coordinates to equatorial coordinates.
 * @param   altaz Altitude and azimuth, in degrees.
 * @param   latitude Observer latitude, in degrees.
 * @param   lst_in_hours Local sidereal time, in hours.
 * @return  Right ascension in [0, 24) hours and declination in degrees.
 *          Both fields are NaN on nonfinite input or calculation failure; callers must reject them.
 */
RADec altaz_to_radec(AltAz altaz, double latitude, double lst_in_hours);

/**
 * @brief   Convert right ascension hours to degrees.
 * @param   ra_in_hours Right ascension, in hours.
 * @return  Right ascension, in degrees.
 */
double hours_to_degrees(double ra_in_hours);

/**
 * @brief   Convert degrees to equivalent angular hours.
 * @param   degrees Angle in degrees.
 * @return  Angle in hours.
 */
double degrees_to_hours(double degrees);

/**
 * @brief   Combine hours, minutes and seconds into decimal hours.
 * @param   hours Whole hours.
 * @param   minutes Minute component.
 * @param   seconds Second component.
 * @return  Decimal hours.
 */
double hms_to_double(int hours, int minutes, int seconds);

/**
 * @brief   Combine DMS magnitudes using the sign of the first nonzero component.
 * @param   degrees Signed whole degrees.
 * @param   minutes Arcminutes, signed only when degrees are zero.
 * @param   seconds Arcseconds, signed only when degrees and minutes are zero.
 * @return  Decimal degrees.
 */
double dms_to_double(int degrees, int minutes, int seconds);

/**
 * @brief   Trace an angle as degrees, minutes and seconds at debug level.
 * @param   decimal_deg Angle in decimal degrees.
 * @return  None.
 */
void double_to_hms(double decimal_deg);

/**
 * @brief   Calculate the shortest signed difference between two angles.
 * @param   angle1 First angle, in degrees.
 * @param   angle2 Angle to subtract, in degrees.
 * @return  Angle1 minus angle2, wrapped to [-180, 180] degrees.
 */
double angle_difference(double angle1, double angle2);

/**
 * @brief   Split equatorial coordinates into hours and degree components.
 * @param   input Right ascension in hours and declination in degrees.
 * @return  Sexagesimal components with declination sign in the first nonzero DMS component.
 *          Invalid/unrepresentable input returns zero integer fields and NaN seconds; do not format those as integers.
 */
RA_Dec_Split split_radec(RADec input);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_COORDINATES_H
