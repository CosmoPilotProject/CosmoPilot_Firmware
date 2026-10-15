/**
 * @file    telescope_goto.c
 * @brief   GoTo positioning and visibility checks for celestial targets.
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
#include "headers/telescope_goto.h"
#include <strings.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier for GoTo operations.
 */
#define TRACE_NAME TELESCOPE_GOTO

/**
 * @brief   Configured trace level for GoTo operations.
 */
#define TRACE_LEVEL CELESTIAL_OBJECTS_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Observer elevation in meters for solar-system coordinate calculations.
 */
#define OBSERVER_HEIGHT_METERS 500.0

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Move toward equatorial coordinates and update the stored telescope position.
 * @param   ra Target right ascension in hours.
 * @param   dec Target declination in degrees.
 * @param   speed1_alt Altitude motor speed in degrees per second.
 * @param   speed2_az Azimuth motor speed in degrees per second.
 * @param   app_goto Whether to use the app GoTo path, enabling Stop GoTo when recoverable.
 * @return  False for a rejected motor request; true otherwise, including app GoTo interruption.
 */
static bool move_to_ra_dec_internal(double ra, double dec, float speed1_alt, float speed2_az, bool app_goto);

/**
 * @brief   Find a supported solar-system body by name without case sensitivity.
 * @param   name NUL-terminated body name, or NULL.
 * @return  Matching body identifier, or BODY_INVALID for a missing or unsupported name.
 */
static astro_body_t get_body_from_name(const char *name);

/**
 * @brief   Check equatorial components against their expected finite ranges.
 * @param   ra_hours Right ascension hour component.
 * @param   ra_minutes Right ascension minute component.
 * @param   ra_seconds Right ascension second component.
 * @param   dec_degrees Declination degree component.
 * @param   dec_minutes Declination arcminute component.
 * @param   dec_seconds Declination arcsecond component.
 * @return  True if every component is finite and within the expected ranges.
 */
static bool app_coordinates_are_valid(float ra_hours, float ra_minutes, float ra_seconds, float dec_degrees,
                                      float dec_minutes, float dec_seconds);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

bool move_to_RA_Dec(double ra, double dec, float speed1_alt, float speed2_az)
{
    // Tracking and two-star calibration keep their existing blocking behavior.
    return move_to_ra_dec_internal(ra, dec, speed1_alt, speed2_az, false);
}

bool is_object_visible(RADec radec, float visibility_min_altitude)
{

    // Step 1: Get current UT
    global_ut = current_time_in_hours(global_ut); // UT in hours

    // Step 2: Compute LST
    CALC_LST(); // Must update  LST global variable

    // Step 3: Convert RA/Dec to Alt/Az
    AltAz altaz = radec_to_altaz(radec, global_latitude, global_lst);

    // Step 4: Compare altitude with threshold
    bool isObjectVisible;
    if (altaz.alt >= visibility_min_altitude)
    {
        isObjectVisible = true;
    }
    else
        isObjectVisible = false;

    if (!global_status_is_tracking_active)
    {
        char buffer[64];
        for (int i = 0; i < 3; i++)
        {
            snprintf(buffer, sizeof(buffer), APP_TX_OBJECT_VISIBLE_FORMAT, isObjectVisible);
            ble_notify_app((uint8_t *) buffer, strlen(buffer));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    return isObjectVisible;
}

bool goto_app_object(float ra_hours, float ra_minutes, float ra_seconds, float dec_degrees, float dec_minutes,
                     float dec_seconds)
{
    if (!app_coordinates_are_valid(ra_hours, ra_minutes, ra_seconds, dec_degrees, dec_minutes, dec_seconds))
    {
        TRACE_WAR("Application coordinates are outside the expected range");
    }

    RADec object = {.ra = hms_to_double(ra_hours, ra_minutes, ra_seconds),
                    .dec = dms_to_double(dec_degrees, dec_minutes, dec_seconds)};

    if (!is_object_visible(object, global_alt_visib_limit))
    {
        TRACE_WAR("Object is outside the visible range (RA %.4f, Dec %.4f)", object.ra, object.dec);
        return false;
    }

    TRACE_INF("Moving to application object (RA %.4f, Dec %.4f)", object.ra, object.dec);
    return move_to_ra_dec_internal(object.ra, object.dec, global_max_speed, global_max_speed, true);
}

bool goto_planet(const char *object_name)
{
    if ((object_name == NULL) || (object_name[0] == '\0'))
    {
        TRACE_WAR("Cannot select a solar-system object without a name");
        return false;
    }

    // Reset cached astronomy state before every named-object calculation.
    Astronomy_Reset();

    astro_body_t body = get_body_from_name(object_name);
    if (body == BODY_INVALID)
    {
        TRACE_WAR("Unsupported solar-system object: %s", object_name);
        return false;
    }

    astro_time_t time = make_current_astronomy_time();
    astro_observer_t observer = Astronomy_MakeObserver(global_latitude, global_longitude, OBSERVER_HEIGHT_METERS);
    astro_equatorial_t equatorial = Astronomy_Equator(body, &time, observer, EQUATOR_OF_DATE, ABERRATION);

    if (equatorial.status != ASTRO_SUCCESS)
    {
        TRACE_ERR("Failed to calculate coordinates for %s: status %d", object_name, (int) equatorial.status);
        return false;
    }

    RADec object = {.ra = equatorial.ra, .dec = equatorial.dec};

    if (!is_object_visible(object, global_alt_visib_limit))
    {
        TRACE_WAR("%s is outside the visible range (RA %.4f, Dec %.4f)", object_name, object.ra, object.dec);
        return false;
    }

    TRACE_INF("Moving to %s (RA %.4f, Dec %.4f)", object_name, object.ra, object.dec);
    return move_to_ra_dec_internal(object.ra, object.dec, global_max_speed, global_max_speed, true);
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static bool move_to_ra_dec_internal(double ra, double dec, float speed1_alt, float speed2_az, bool app_goto)
{
    CALC_LST();

    // Target in real sky frame
    RADec radec_target = (RADec) {ra, dec};
    AltAz altaz_target_real = radec_to_altaz(radec_target, global_latitude, global_lst);

    // Current position must always be treated/stored as real frame
    AltAz altaz_current_real = global_possition;

    if (!isfinite(altaz_target_real.alt) || !isfinite(altaz_target_real.az) || !isfinite(altaz_current_real.alt) ||
        !isfinite(altaz_current_real.az))
    {
        TRACE_WAR("Positioning rejected: nonfinite coordinates");
        return false;
    }

    // Values actually used for motor command
    AltAz altaz_target_cmd = altaz_target_real;
    AltAz altaz_current_cmd = altaz_current_real;

    if (global_2_star_align)
    {
        // global_rotation_matrix maps measured -> real
        // for commanding motors we need real -> measured, so use inverse
        altaz_target_cmd = apply_two_star_local_goto(altaz_target_real, &global_two_star_local);
        altaz_current_cmd = apply_two_star_local_goto(altaz_current_real, &global_two_star_local);
    }

    double dAz = angle_difference(altaz_current_cmd.az, altaz_target_cmd.az);
    double dAlt = altaz_target_cmd.alt - altaz_current_cmd.alt;

    TRACE_DEB("current position(1 star): az=%.5f alt=%.5f", altaz_current_real.az, altaz_current_real.alt);
    TRACE_DEB("target position(1 star): az=%.5f alt=%.5f", altaz_target_real.az, altaz_target_real.alt);
    if (global_2_star_align)
    {
        TRACE_DEB("current position(2 star): az=%.5f alt=%.5f", altaz_current_cmd.az, altaz_current_cmd.alt);
        TRACE_DEB("target position(2 star): az=%.5f alt=%.5f", altaz_target_cmd.az, altaz_target_cmd.alt);
    }
    TRACE_DEB("Delta: dAz=%.5f dAlt=%.5f", dAz, dAlt);

    if (app_goto)
    {
        // An ambiguous correction disables only Stop GoTo, never the normal move.
        // The application task owns calibration for the full blocking movement.
        const bool position_recoverable = !global_2_star_align || two_star_local_can_invert(&global_two_star_local);
        const telescope_goto_move_result_t movement =
            move_steppers_goto(dAlt, speed1_alt, dAz, speed2_az, position_recoverable);
        if (movement.status == TELESCOPE_GOTO_MOVE_REJECTED)
        {
            // A rejected motor request did not move the telescope.
            TRACE_WAR("Positioning rejected: target not reached (RA %.4f, Dec %.4f)", ra, dec);
            return false;
        }
        if (movement.status == TELESCOPE_GOTO_MOVE_INTERRUPTED)
        {
            AltAz stopped = {.alt = altaz_current_cmd.alt + movement.alt_degrees,
                             // angle_difference(current, target) uses the opposite AZ sign.
                             .az = fmod(altaz_current_cmd.az - movement.az_degrees, 360.0)};
            if (stopped.az < 0.0)
            {
                stopped.az += 360.0;
            }
            if (global_2_star_align)
            {
                // The preflight bound guarantees a unique finite inverse. The
                // helper still supplies its best candidate if numeric verification
                // fails; report that uncertainty instead of keeping a stale target.
                if (!invert_two_star_local_goto(stopped, &global_two_star_local, &stopped))
                {
                    TRACE_ERR("Stopped GoTo position inverse exceeded numerical tolerance");
                }
            }
            global_possition = stopped;
            TRACE_INF("GoTo stopped at AZ %.5f ALT %.5f", stopped.az, stopped.alt);
            return true;
        }
    }
    else
    {
        if (!move_steppers(dAlt, speed1_alt, dAz, speed2_az))
        {
            TRACE_WAR("Positioning rejected: target not reached (RA %.4f, Dec %.4f)", ra, dec);
            return false;
        }
    }

    // Always store global position in real frame
    global_possition.az = altaz_target_real.az;
    global_possition.alt = altaz_target_real.alt;
    return true;
}

static astro_body_t get_body_from_name(const char *name)
{
    if (name == NULL)
    {
        return BODY_INVALID;
    }

    if (strcasecmp(name, APP_BODY_MERCURY) == 0)
    {
        return BODY_MERCURY;
    }
    if (strcasecmp(name, APP_BODY_VENUS) == 0)
    {
        return BODY_VENUS;
    }
    if (strcasecmp(name, APP_BODY_MARS) == 0)
    {
        return BODY_MARS;
    }
    if (strcasecmp(name, APP_BODY_JUPITER) == 0)
    {
        return BODY_JUPITER;
    }
    if (strcasecmp(name, APP_BODY_SATURN) == 0)
    {
        return BODY_SATURN;
    }
    if (strcasecmp(name, APP_BODY_URANUS) == 0)
    {
        return BODY_URANUS;
    }
    if (strcasecmp(name, APP_BODY_NEPTUNE) == 0)
    {
        return BODY_NEPTUNE;
    }
    if (strcasecmp(name, APP_BODY_MOON) == 0)
    {
        return BODY_MOON;
    }
    if (strcasecmp(name, APP_BODY_SUN) == 0)
    {
        return BODY_SUN;
    }

    // Pluto remains intentionally unsupported by the astronomy calculation error.
    return BODY_INVALID;
}

static bool app_coordinates_are_valid(float ra_hours, float ra_minutes, float ra_seconds, float dec_degrees,
                                      float dec_minutes, float dec_seconds)
{
    if (!isfinite(ra_hours) || !isfinite(ra_minutes) || !isfinite(ra_seconds) || !isfinite(dec_degrees) ||
        !isfinite(dec_minutes) || !isfinite(dec_seconds))
    {
        return false;
    }

    if ((ra_hours < 0.0f) || (ra_hours >= 24.0f) || (ra_minutes < 0.0f) || (ra_minutes >= 60.0f) ||
        (ra_seconds < 0.0f) || (ra_seconds >= 60.0f))
    {
        return false;
    }

    if ((dec_degrees < -90.0f) || (dec_degrees > 90.0f) || (fabsf(dec_minutes) >= 60.0f) ||
        (fabsf(dec_seconds) >= 60.0f) || (dec_minutes < 0.0f && dec_degrees != 0.0f) ||
        (dec_seconds < 0.0f && (dec_degrees != 0.0f || dec_minutes != 0.0f)))
    {
        return false;
    }

    if ((fabsf(dec_degrees) == 90.0f) && ((dec_minutes != 0.0f) || (dec_seconds != 0.0f)))
    {
        return false;
    }

    return true;
}
