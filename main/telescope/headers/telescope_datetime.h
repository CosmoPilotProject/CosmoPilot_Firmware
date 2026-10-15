/**
 * @file    telescope_datetime.h
 * @brief   Julian date, sidereal time, and astronomy time interface.
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

#ifndef TELESCOPE_DATETIME_H
#define TELESCOPE_DATETIME_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"
#include "astronomy_engine.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Calculate local sidereal time for an observer's longitude.
 * @param   gst Greenwich sidereal time in hours.
 * @param   longitude Observer longitude in degrees, positive east.
 * @return  Local sidereal time in hours.
 */
double calculate_lst(double gst, double longitude);

/**
 * @brief   Calculate Greenwich sidereal time from a Julian date.
 * @param   jd Julian date, including the fractional day.
 * @return  Greenwich sidereal time in hours.
 * @see     https://auass.com/wp-content/uploads/2021/01/Astronomical-Algorithms.pdf
 *          (chapter 12 Sidereal Time at Greenwich)
 */
double calculate_gst(double jd);

/**
 * @brief   Convert a Gregorian calendar date and Universal Time to a Julian date.
 * @param   year Calendar year.
 * @param   month Calendar month, from 1 to 12.
 * @param   day Day of the month.
 * @param   ut Universal Time in decimal hours.
 * @return  Julian day count including the fractional day.
 * @see     https://squarewidget.com/julian-day/
 */
double calculate_julian_date(int year, int month, int day, double ut);

/**
 * @brief   Refresh astronomy and whole-second UTC snapshots from the unchanged Julian-date epoch.
 * @param   now System-clock instant at which to evaluate the epoch.
 * @return  Astronomy Engine time for the same instant as global_jd.
 */
astro_time_t refresh_astronomy_time(time_t now);

/**
 * @brief   Rebase the epoch from global UTC date/time and refresh dependent snapshots.
 * @param   now System-clock reference for the supplied global date/time.
 * @return  None.
 */
void reset_astronomy_time(time_t now);

/**
 * @brief   Refresh Julian date, sidereal time and UTC snapshots from the elapsed system clock.
 * @return  None.
 */
void CALC_LST();

/**
 * @brief   Refresh UTC date/time from the advancing Julian-date epoch.
 * @param   ut Legacy argument, ignored; time is derived from the epoch.
 * @return  Current UTC time of day in decimal hours, from 0 inclusive to 24 exclusive.
 */
double current_time_in_hours(double ut);

/**
 * @brief   Refresh and return astronomy time from the advancing Julian-date epoch.
 * @return  Astronomy Engine time for the same absolute instant as the refreshed sidereal state.
 */
astro_time_t make_current_astronomy_time(void);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_DATETIME_H
