/**
 * @file    telescope_datetime.c
 * @brief   Julian date, sidereal time, and current astronomy time calculations.
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

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "driver/gpio.h"
#include "mobile_app/headers/mobile_app_commands.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h"
#include "config.h"
#include "headers/telescope_stepper.h"
#include "joystick/analog_joystick.h"
#include "headers/telescope_common.h"
#include <stdbool.h>
#include "flash.h"
#include "esp_log.h"
#include <string.h>
#include "bluetooth/bluetooth.h"
#include "mobile_app/headers/mobile_app_handlers.h"
#include "headers/telescope_datetime.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

double calculate_julian_date(int year, int month, int day, double ut)
{
    if (month <= 2)
    {
        year -= 1;
        month += 12;
    }
    int A = year / 100;
    int B = 2 - A + (A / 4);
    double JD = (int) (365.25 * (year + 4716)) + (int) (30.6001 * (month + 1)) + day + B - 1524.5 + ut / 24.0;
    return JD;
}

double calculate_gst(double jd)
{
    double T = (jd - 2451545.0) / 36525.0; // Centuries since J2000.0
    double GST = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * T * T - (T * T * T) / 38710000.0;
    GST = fmod(GST, 360.0); // Normalize to (0, 360)
    if (GST < 0)
        GST += 360.0;
    return GST / 15.0; // Convert to hours
}

double calculate_lst(double gst, double longitude)
{
    double lst = gst + longitude / 15.0; // Convert longitude to hours
    if (lst < 0)
        lst +=
            24.0; //If the LST is negative, it means the observer is west of Greenwich, adding 24 hours brings it into the valid range
    if (lst >= 24.0)
        lst -=
            24.0; //If the LST is 24 hours or more, it means the observer is east of Greenwich, subtracting 24 hours brings it into the valid range
    return lst;
}

astro_time_t refresh_astronomy_time(time_t now)
{
    double elapsed_seconds = difftime(now, global_start_clock);
    // The epoch is unchanged by snapshots: elapsed whole and fractional days are added once.
    global_jd = initial_jd + elapsed_seconds / 86400.0;
    global_gst = calculate_gst(global_jd);
    global_lst = calculate_lst(global_gst, global_longitude);

    astro_time_t current = Astronomy_TimeFromDays(global_jd - 2451545.0);
    // Commands and time(NULL) have whole-second resolution. Round only the civil
    // snapshot, including date carry, so JD roundoff cannot display the previous second.
    astro_time_t calendar_time = current;
    calendar_time.ut += 0.5 / 86400.0;
    astro_utc_t utc = Astronomy_UtcFromTime(calendar_time);
    global_year = utc.year;
    global_month = utc.month;
    global_day = utc.day;
    global_ut = (utc.hour * 3600 + utc.minute * 60 + (int) utc.second) / 3600.0;
    global_start_clock_ut = now;
    return current;
}

void reset_astronomy_time(time_t now)
{
    initial_jd = calculate_julian_date(global_year, global_month, global_day, global_ut);
    global_start_clock = now;
    (void) refresh_astronomy_time(now);
}

void CALC_LST(void)
{
    (void) refresh_astronomy_time(time(NULL));
}

double current_time_in_hours(double ut)
{
    (void) ut; // Retained for callers; the Julian-date epoch is the sole time source.
    (void) refresh_astronomy_time(time(NULL));
    return global_ut;
}

astro_time_t make_current_astronomy_time(void)
{
    return refresh_astronomy_time(time(NULL));
}

/* Private Functions ------------------------------------------------------------------------------------------------ */
