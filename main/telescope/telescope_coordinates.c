/**
 * @file    telescope_coordinates.c
 * @brief   Astronomical coordinate and angular conversion utilities.
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
#include <float.h>
#include <limits.h>
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

#include "headers/telescope_coordinates.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for telescope coordinate conversions.
 */
#define TRACE_NAME TELESCOPE_COORDINATES

/**
 * @brief   Trace verbosity for telescope coordinate conversions.
 */
#define TRACE_LEVEL TELESCOPE_COORDINATES_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

// Permit only rounding-sized excursions from the inverse-trigonometric domain.
static double clamp_trig_roundoff(double value);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

double hours_to_degrees(double ra_in_hours)
{
    return ra_in_hours * 15.0; // 1 hour = 15 degrees
}

double degrees_to_hours(double degrees)
{
    return degrees / 15.0; // 1 degree = 1/15 hours
}

RADec altaz_to_radec(AltAz altaz, double latitude, double lst_hours)
{
    RADec result;

    if (!isfinite(altaz.alt) || !isfinite(altaz.az) || !isfinite(latitude) || !isfinite(lst_hours))
    {
        return (RADec) {.ra = NAN, .dec = NAN};
    }

    // Convert all angles to radians
    double deg_to_rad = M_PI / 180.0;
    double alt_rad = altaz.alt * deg_to_rad;
    double az_rad = altaz.az * deg_to_rad;
    double lat_rad = latitude * deg_to_rad;

    // 1. Calculate Declination
    // Meeus, Ch.13
    // Reformulated using atan2 to resolve quadrant ambiguity (recommended in Meeus)
    result.dec = asin(clamp_trig_roundoff(sin(alt_rad) * sin(lat_rad) + cos(alt_rad) * cos(lat_rad) * cos(az_rad)));

    // 2. Calculate Hour Angle (HA)
    double dec_rad = result.dec;

    double ha_sin = (-sin(az_rad) * cos(alt_rad) / cos(dec_rad));
    double ha_cos = (sin(alt_rad) - sin(dec_rad) * sin(lat_rad)) /
                    (cos(dec_rad) * cos(lat_rad)); //Jean Meeus: Astronomical algorithms: chapter 13 (13.6)
    double ha_rad = atan2(ha_sin, ha_cos);         //atan2 to avoid quadrant issues

    // Normalize hour angle to (0, 2π)
    ha_rad = fmod(ha_rad, 2 * M_PI); //(-2π, 2π)
    if (ha_rad < 0)
    {
        ha_rad += 2 * M_PI; // Adjust negatives to (0, 2π)
    }

    // 3. Convert LST to radians (1 hour = π/12 radians)
    double lst_rad = lst_hours * (M_PI / 12.0);

    // 4. Calculate RA
    double ra_rad = lst_rad - ha_rad;

    ra_rad = fmod(ra_rad, 2 * M_PI); //(-2π, 2π)
    if (ra_rad < 0)
    {
        ra_rad += 2 * M_PI; //(0, 2π)
    }

    // Convert RA to hours and Dec to degrees
    result.ra = ra_rad * (12.0 / M_PI); // Radians -> hours
    result.dec = dec_rad * (180.0 / M_PI);

    if (!isfinite(result.ra) || !isfinite(result.dec))
    {
        return (RADec) {.ra = NAN, .dec = NAN};
    }

    return result;
}

AltAz radec_to_altaz(RADec radec, double latitude, double lst_in_hours)
{
    AltAz altaz;

    if (!isfinite(radec.ra) || !isfinite(radec.dec) || !isfinite(latitude) || !isfinite(lst_in_hours))
    {
        return (AltAz) {.alt = NAN, .az = NAN};
    }

    //Convert RA and LST to degrees
    double ra_deg = hours_to_degrees(radec.ra);
    double lst_deg = hours_to_degrees(lst_in_hours);

    //Convert to radians
    double ra_rad = ra_deg * M_PI / 180.0;
    double dec_rad = radec.dec * M_PI / 180.0;
    double lat_rad = latitude * M_PI / 180.0;
    double lst_rad = lst_deg * M_PI / 180.0;

    //Calculate Hour Angle (HA)
    double ha_rad = lst_rad - ra_rad;
    ha_rad = fmod(ha_rad, 2 * M_PI); // Normalize HA to (0, 2π)
    if (ha_rad < 0)
        ha_rad += 2 * M_PI;

    //Altitude calculation
    altaz.alt = asin(clamp_trig_roundoff(sin(dec_rad) * sin(lat_rad) + cos(dec_rad) * cos(lat_rad) * cos(ha_rad)));
    altaz.alt = altaz.alt * 180.0 / M_PI; // Convert to degrees

    // Azimuth calculation
    double alt_rad = altaz.alt * M_PI / 180.0; // Altitude in radians

    double az_sin = (-sin(ha_rad) * cos(dec_rad) / cos(alt_rad));
    double az_cos = (sin(dec_rad) - sin(alt_rad) * sin(lat_rad)) / (cos(alt_rad) * cos(lat_rad));

    altaz.az = atan2(az_sin, az_cos) * 180.0 / M_PI;

    // Normalize azimuth to (0, 360)
    if (altaz.az < 0)
        altaz.az += 360.0;

    if (!isfinite(altaz.alt) || !isfinite(altaz.az))
    {
        return (AltAz) {.alt = NAN, .az = NAN};
    }

    return altaz;
}

double hms_to_double(int hours, int minutes, int seconds)
{
    return hours + (minutes / 60.0) + (seconds / 3600.0);
}

double dms_to_double(int degrees, int minutes, int seconds)
{
    // The first nonzero component carries the sign; signed zero is never needed.
    bool negative = degrees < 0 || (degrees == 0 && minutes < 0) || (degrees == 0 && minutes == 0 && seconds < 0);
    double decimal_degrees = fabs((double) degrees) + fabs((double) minutes) / 60.0 + fabs((double) seconds) / 3600.0;
    return negative ? -decimal_degrees : decimal_degrees;
}

void double_to_hms(double decimal_deg)
{
    RA_Dec_Split components = split_radec((RADec) {.ra = 0.0, .dec = decimal_deg});
    if (!isfinite(components.ra_seconds) || !isfinite(components.dec_arcseconds))
    {
        return;
    }
    int degrees = components.dec_degrees;
    int minutes = components.dec_arcminutes;
    double seconds = components.dec_arcseconds;
    (void) degrees; // Used only when debug tracing is enabled.
    (void) minutes;
    (void) seconds;

    TRACE_DEB("%d° %d' %.2f\"", degrees, minutes, seconds);
}

double angle_difference(double angle1, double angle2)
{
    // Calculate the raw difference
    double diff = fmod((angle1 - angle2), 360.0);

    // Normalize the difference to [-180, 180] with direction preserved
    if (diff > 180.0)
    {
        diff -= 360.0;
    }
    else if (diff < -180.0)
    {
        diff += 360.0;
    }

    return diff; // Positive for counter-clockwise, negative for clockwise
}

double deg_to_rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

double rad_to_deg(double radians)
{
    return radians * 180.0 / M_PI;
}

RA_Dec_Split split_radec(RADec input)
{
    RA_Dec_Split result;

    // Invalid results must never reach a floating-point-to-integer conversion.
    if (!isfinite(input.ra) || !isfinite(input.dec) || fabs(input.ra) > INT_MAX || fabs(input.dec) > INT_MAX)
    {
        return (RA_Dec_Split) {.ra_seconds = NAN, .dec_arcseconds = NAN};
    }

    // RA (hours)
    double ra = input.ra; // e.g., 13.4789 hours
    result.ra_hours = (int) ra;
    double ra_minutes_full = (ra - result.ra_hours) * 60.0;
    result.ra_minutes = (int) ra_minutes_full;
    result.ra_seconds = (ra_minutes_full - result.ra_minutes) * 60.0;

    // Dec (degrees)
    double dec = input.dec;
    double abs_dec;
    int sign;

    if (dec >= 0)
    {
        sign = 1;
        abs_dec = dec;
    }
    else
    {
        sign = -1;
        abs_dec = -dec;
    }

    result.dec_degrees = sign * (int) abs_dec;
    double dec_minutes_full = (abs_dec - (int) abs_dec) * 60.0;
    result.dec_arcminutes = (int) dec_minutes_full;
    result.dec_arcseconds = (dec_minutes_full - result.dec_arcminutes) * 60.0;

    // Below one degree, move the sign to the first nonzero subcomponent.
    if (sign < 0 && result.dec_degrees == 0)
    {
        if (result.dec_arcminutes != 0)
        {
            result.dec_arcminutes = -result.dec_arcminutes;
        }
        else
        {
            result.dec_arcseconds = -result.dec_arcseconds;
        }
    }

    return result;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static double clamp_trig_roundoff(double value)
{
    // These arguments sum products of double-precision sines/cosines. Allow a
    // few rounding steps at +/-1, without accepting arbitrary invalid values.
    const double tolerance = 8.0 * DBL_EPSILON;
    if (!isfinite(value) || value < -1.0 - tolerance || value > 1.0 + tolerance)
    {
        return NAN;
    }
    if (value < -1.0)
    {
        return -1.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}
