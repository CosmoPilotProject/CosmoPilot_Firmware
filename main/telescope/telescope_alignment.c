/**
 * @file    telescope_alignment.c
 * @brief   Star alignment calibration and pointing correction implementation.
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
#include <ctype.h>
#include <stdlib.h>
#include "bluetooth/bluetooth.h"
#include "mobile_app/headers/mobile_app_handlers.h"

#include "headers/telescope_alignment.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for telescope alignment.
 */
#define TRACE_NAME TELESCOPE_ALIGNMENT

/**
 * @brief   Trace verbosity for telescope alignment.
 */
#define TRACE_LEVEL TELESCOPE_ALIGNMENT_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

// Keep the existing conversions rather than the astronomy header's macros.
#undef DEG2RAD
#undef RAD2DEG
/**
 * @brief   Conversion factor from degrees to radians.
 */
#define DEG2RAD (M_PI / 180.0)

/**
 * @brief   Conversion factor from radians to degrees.
 */
#define RAD2DEG (180.0 / M_PI)

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Limit a value to the supplied inclusive range.
 * @param   x Value to limit.
 * @param   lo Lower bound.
 * @param   hi Upper bound.
 * @return  Value clamped to [lo, hi].
 */
static double clampd(double x, double lo, double hi);

/**
 * @brief   Wrap an azimuth difference to [-180, 180] degrees.
 * @param   az_deg Azimuth difference, in degrees.
 * @return  Wrapped difference, in degrees.
 */
static double wrap_az_180(double az_deg);

/**
 * @brief   Wrap azimuth to [0, 360) degrees.
 * @param   az_deg Azimuth, in degrees.
 * @return  Wrapped azimuth, in degrees.
 */
static double wrap_az_360(double az_deg);

/**
 * @brief   Calculate the angular separation of two horizontal positions.
 * @param   a First position, in degrees.
 * @param   b Second position, in degrees.
 * @return  Angular separation from 0 to 180 degrees.
 */
static double angular_distance_deg(AltAz a, AltAz b);

/**
 * @brief   Check two-star components before integer coordinate conversion.
 * @param   ra_h Right ascension hours.
 * @param   ra_m Right ascension minutes.
 * @param   ra_s Right ascension seconds.
 * @param   dec_d Declination degrees.
 * @param   dec_m Declination arcminutes.
 * @param   dec_s Declination arcseconds.
 * @return  True for finite components within the accepted two-star ranges.
 */
static bool two_star_coordinates_valid(double ra_h, double ra_m, double ra_s, double dec_d, double dec_m, double dec_s);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Current local two-star pointing correction.
 */
TwoStarLocalCalib global_two_star_local = {0};

/* Public Functions ------------------------------------------------------------------------------------------------- */

void compute_two_star_local_calibration(AltAz real1, AltAz real2, AltAz meas2, TwoStarLocalCalib *cal)
{
    cal->real1 = real1;
    cal->real2 = real2;

    cal->delta_alt = meas2.alt - real2.alt;
    cal->delta_az = wrap_az_180(meas2.az - real2.az);

    cal->star_sep_deg = angular_distance_deg(real1, real2);
    cal->valid = (cal->star_sep_deg > 1.0);
}

AltAz apply_two_star_local_goto(AltAz real_target, const TwoStarLocalCalib *cal)
{
    AltAz cmd = real_target;

    if (!cal->valid)
    {
        return cmd;
    }

    double d1 = angular_distance_deg(real_target, cal->real1);
    double d12 = cal->star_sep_deg;

    double w = d1 / d12;
    w = clampd(w, 0.0, 1.0);

    cmd.alt = real_target.alt + w * cal->delta_alt;
    cmd.az = wrap_az_360(real_target.az + w * cal->delta_az);

    return cmd;
}

bool two_star_local_can_invert(const TwoStarLocalCalib *cal)
{
    if (cal == NULL)
    {
        return false;
    }
    if (!cal->valid)
    {
        return true;
    }

    if (!isfinite(cal->real1.alt) || !isfinite(cal->real1.az) || !isfinite(cal->real2.alt) ||
        !isfinite(cal->real2.az) || !isfinite(cal->delta_alt) || !isfinite(cal->delta_az) ||
        !isfinite(cal->star_sep_deg) || cal->star_sep_deg <= 1.0 || cal->star_sep_deg > 180.0)
    {
        return false;
    }

    // The angular-distance weight is Lipschitz with constant 1 / separation.
    // Keep a 10% margin: a nearly singular but mathematically unique inverse can
    // turn floating-point rounding into many degrees of sky-position error.
    // This bounds inverse error amplification by 10; the checked 0.00001-degree
    // per-axis residual stays below one microstep at maximum supported gearing.
    // This eligibility check affects only Stop GoTo, never normal calibration/GoTo.
    return hypot(cal->delta_alt, cal->delta_az) <= 0.9 * cal->star_sep_deg;
}

bool invert_two_star_local_goto(AltAz commanded, const TwoStarLocalCalib *cal, AltAz *real)
{
    if (real == NULL || !isfinite(commanded.alt) || !isfinite(commanded.az) || !two_star_local_can_invert(cal))
    {
        return false;
    }

    commanded.az = fmod(commanded.az, 360.0);
    if (commanded.az < 0.0)
    {
        commanded.az += 360.0;
    }
    if (!cal->valid)
    {
        *real = commanded;
        return true;
    }

    // For weight w, real = commanded - w * correction. The forward weight must
    // equal w. Its residual is monotone on [0, 1] under the invertibility bound.
    double low = 0.0;
    double high = 1.0;
    AltAz candidate = commanded;
    AltAz best_candidate = commanded;
    double best_residual = 2.0;
    for (int iteration = 0; iteration < 66; ++iteration)
    {
        // Check both endpoints explicitly: the existing acos calculation can
        // round tiny distances to zero, especially at the first reference star.
        double weight = (low + high) * 0.5;
        if (iteration == 0)
        {
            weight = 0.0;
        }
        else if (iteration == 1)
        {
            weight = 1.0;
        }
        candidate.alt = commanded.alt - weight * cal->delta_alt;
        candidate.az = wrap_az_360(commanded.az - weight * cal->delta_az);
        const double actual_weight = clampd(angular_distance_deg(candidate, cal->real1) / cal->star_sep_deg, 0.0, 1.0);
        const double residual = fabs(actual_weight - weight);
        if (residual < best_residual)
        {
            best_candidate = candidate;
            best_residual = residual;
        }

        if (actual_weight == weight || (iteration >= 2 && (weight == low || weight == high)))
        {
            break;
        }
        if (actual_weight > weight)
        {
            low = weight;
        }
        else
        {
            high = weight;
        }
    }

    const AltAz checked = apply_two_star_local_goto(best_candidate, cal);
    // acos in the existing forward transform loses precision near a reference
    // star. This tolerance is still below 1/14 microstep at the maximum gear ratio.
    const double residual_tolerance_deg = 0.00001;
    *real = best_candidate;
    if (!isfinite(checked.alt) || !isfinite(checked.az) || fabs(checked.alt - commanded.alt) > residual_tolerance_deg ||
        fabs(wrap_az_180(checked.az - commanded.az)) > residual_tolerance_deg)
    {
        return false;
    }

    return true;
}

Vec3 altaz_to_vector(AltAz a)
{
    double alt_rad = a.alt * DEG2RAD;
    double az_rad = a.az * DEG2RAD;

    Vec3 v;
    v.x = cos(alt_rad) * sin(az_rad);
    v.y = cos(alt_rad) * cos(az_rad);
    v.z = sin(alt_rad);
    return v;
}

double dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 normalize(Vec3 v)
{
    double len = sqrt(dot(v, v));
    return (Vec3) {v.x / len, v.y / len, v.z / len};
}

bool calibrate_to_radec(double hour, double minute, double second, double degree, double degree_minute,
                        double degree_second)
{

    // Preserve existing truncation, but reject unsafe implicit integer conversions.
    const double components[] = {hour, minute, second, degree, degree_minute, degree_second};
    for (size_t i = 0; i < sizeof(components) / sizeof(components[0]); ++i)
    {
        if (!isfinite(components[i]) || components[i] < INT_MIN || components[i] > INT_MAX)
        {
            TRACE_WAR("Calibration rejected: invalid coordinate component");
            return false;
        }
    }

    CALC_LST();

    // Use the requested reference star.
    RADec radec_reference = {hms_to_double(hour, minute, second), dms_to_double(degree, degree_minute, degree_second)};
    AltAz altaz_reference = radec_to_altaz(radec_reference, global_latitude, global_lst); // Convert RA/Dec to Alt/Az

    if (!isfinite(altaz_reference.alt) || !isfinite(altaz_reference.az))
    {
        TRACE_WAR("Calibration rejected: nonfinite position");
        return false;
    }

    // Publish the calibrated altitude and azimuth.
    global_possition.az = altaz_reference.az;
    global_possition.alt = altaz_reference.alt;

    return true;
}

bool parse_two_star_coordinates(const char *text, float coordinates[6])
{
    if (text == NULL || coordinates == NULL)
    {
        return false;
    }

    float parsed[6];
    for (size_t i = 0; i < 6; ++i)
    {
        char *end;
        parsed[i] = strtof(text, &end);
        if (end == text || (i < 5 && !isspace((unsigned char) *end)))
        {
            return false;
        }
        text = end;
    }
    while (isspace((unsigned char) *text))
    {
        ++text;
    }
    if (*text != '\0' || !two_star_coordinates_valid(parsed[0], parsed[1], parsed[2], parsed[3], parsed[4], parsed[5]))
    {
        return false;
    }
    memcpy(coordinates, parsed, sizeof(parsed));
    return true;
}

bool two_star_calibration(double s1_ra_h, double s1_ra_m, double s1_ra_s, double s1_dec_d, double s1_dec_m,
                          double s1_dec_s)
{
    if (!two_star_coordinates_valid(s1_ra_h, s1_ra_m, s1_ra_s, s1_dec_d, s1_dec_m, s1_dec_s))
    {
        TRACE_WAR("Invalid first-star coordinates");
        return false;
    }

    // STAR #1 (already centered when called)
    const uint64_t generation = mobile_app_calibration_begin();
    global_2_star_align = false;
    global_two_star_local.valid = false;
    if (mobile_app_calibration_cancelled(generation))
    {
        return false;
    }
    RADec s1 = (RADec) {hms_to_double(s1_ra_h, s1_ra_m, s1_ra_s), dms_to_double(s1_dec_d, s1_dec_m, s1_dec_s)};

    CALC_LST();
    if (!calibrate_to_radec(s1_ra_h, s1_ra_m, s1_ra_s, s1_dec_d, s1_dec_m, s1_dec_s))
    {
        return false;
    }
    AltAz real1 = radec_to_altaz(s1, global_latitude, global_lst); // truth

    if (!isfinite(real1.alt) || !isfinite(real1.az))
    {
        TRACE_WAR("Two-star calibration rejected: nonfinite first star");
        return false;
    }

    // Get STAR #2 RA/Dec from app
    RADec s2 = (RADec) {0};
    mobile_app_command_clear();
    send_ready_status(3);
    TRACE_INF("Waiting for NEXT cmd...");

    // User centering has no deadline; the connection/session controls cancellation.
    while (1)
    {
        mobile_app_command_refresh();
        if (mobile_app_calibration_cancelled(generation))
        {
            TRACE_WAR("Two-star calibration cancelled by disconnect or new start");
            return false;
        }
        if (strncmp(global_last_ble_command, APP_RX_NEXT_PREFIX, APP_RX_NEXT_PREFIX_MATCH_LENGTH) == 0)
        {
            float coordinates[6];
            if (!parse_two_star_coordinates(global_last_ble_command + APP_RAW_OFFSET_NEXT, coordinates))
            {
                TRACE_WAR("Invalid second-star coordinates; calibration aborted");
                mobile_app_command_clear();
                global_2_star_align = false;
                global_status_bar_screen_telescope = false;
                global_two_star_local.valid = false;
                return false;
            }
            s2.ra = hms_to_double(coordinates[0], coordinates[1], coordinates[2]);
            s2.dec = dms_to_double(coordinates[3], coordinates[4], coordinates[5]);
            break;
        }
        else if (strncmp(global_last_ble_command, APP_RX_NONE, APP_RX_NONE_MATCH_LENGTH) == 0)
        {
            global_2_star_align = false;
            global_status_bar_screen_telescope = false;
            global_two_star_local.valid = false;
            return false;
        }
        else if (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0)
        {
            global_2_star_align = false;
            global_status_bar_screen_telescope = false;
            global_two_star_local.valid = false;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    mobile_app_command_clear();

    // Slew near the second reference star before manual centering.
    if (mobile_app_calibration_cancelled(generation))
    {
        return false;
    }
    if (!move_to_RA_Dec(s2.ra, s2.dec, global_max_speed, global_max_speed))
    {
        TRACE_WAR("Two-star calibration aborted: second-star movement rejected");
        return false;
    }
    if (mobile_app_calibration_cancelled(generation))
    {
        return false;
    }

    // Let user fine tune until FINISH
    send_ready_status(3);
    TRACE_INF("Waiting for FINISH_2_STAR_CALIB...");

    // User centering has no deadline; the connection/session controls cancellation.
    int unexpected_cmd_count = 0; // Abort after two unexpected commands.
    while (1)
    {
        mobile_app_command_refresh();
        if (mobile_app_calibration_cancelled(generation))
        {
            TRACE_WAR("Two-star calibration cancelled by disconnect or new start");
            return false;
        }
        if (strncmp(global_last_ble_command, APP_RX_ANALOG_JOYSTICK, APP_RX_ANALOG_JOYSTICK_MATCH_LENGTH) == 0)
        {
            process_analog_joystick_command(global_last_ble_command);
        }
        else if (strncmp(global_last_ble_command, APP_RX_APP_JOYSTICK_START, APP_RX_APP_JOYSTICK_START_MATCH_LENGTH) ==
                 0)
        {
            process_app_joystick_command(APP_TRIMMED_APP_JOYSTICK_START);
        }
        else if (strncmp(global_last_ble_command, APP_RX_FINISH_2_STAR_CALIB,
                         APP_RX_FINISH_2_STAR_CALIB_MATCH_LENGTH) == 0)
        {
            break;
        }
        else if (strncmp(global_last_ble_command, APP_RX_NONE, APP_RX_NONE_MATCH_LENGTH) == 0)
        {
            global_2_star_align = false;
            global_status_bar_screen_telescope = false;
            global_two_star_local.valid = false;
            return false;
        }
        else if (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0)
        {

            unexpected_cmd_count++;
            TRACE_WAR("Unexpected CMD during 2-star calibration (%d/2): %.32s", unexpected_cmd_count,
                      global_last_ble_command);
            // consume it so we don't count the same stale cmd forever
            mobile_app_command_clear();
            for (int i = 0; i < 3; i++)
            {
                ble_notify_app((uint8_t *) APP_TX_READY, APP_TX_READY_LENGTH);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            if (unexpected_cmd_count >= 2)
            {
                global_2_star_align = false;
                global_status_bar_screen_telescope = false;
                global_two_star_local.valid = false;
                send_status_to_app(); // Publish the cleared alignment status before returning.
                return false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    mobile_app_command_clear();

    // STAR #2 capture
    CALC_LST();
    AltAz real2 = radec_to_altaz(s2, global_latitude, global_lst); // truth at press
    AltAz meas2 = global_possition;                                // mount at press

    if (!isfinite(real2.alt) || !isfinite(real2.az) || !isfinite(meas2.alt) || !isfinite(meas2.az))
    {
        TRACE_WAR("Two-star calibration rejected: nonfinite second star");
        return false;
    }

    // Build the local two-star pointing correction.
    compute_two_star_local_calibration(real1, real2, meas2, &global_two_star_local);
    global_two_star_local.valid = global_two_star_local.valid && isfinite(global_two_star_local.delta_alt) &&
                                  isfinite(global_two_star_local.delta_az);
    global_possition = real2;
    global_2_star_align = global_two_star_local.valid;

    if (!global_two_star_local.valid || !mobile_app_calibration_complete(generation))
    {
        global_2_star_align = false;
        global_two_star_local.valid = false;
        TRACE_WAR("Two-star calibration failed: invalid model");
        return false;
    }
    TRACE_INF("Two-star calibration complete");
    return true;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static bool two_star_coordinates_valid(double ra_h, double ra_m, double ra_s, double dec_d, double dec_m, double dec_s)
{
    return isfinite(ra_h) && isfinite(ra_m) && isfinite(ra_s) && isfinite(dec_d) && isfinite(dec_m) &&
           isfinite(dec_s) && ra_h >= 0.0 && ra_h < 24.0 && ra_m >= 0.0 && ra_m < 60.0 && ra_s >= 0.0 && ra_s < 60.0 &&
           dec_d >= -90.0 && dec_d <= 90.0 && fabs(dec_m) < 60.0 && fabs(dec_s) < 60.0 &&
           (dec_m >= 0.0 || dec_d == 0.0) && (dec_s >= 0.0 || (dec_d == 0.0 && dec_m == 0.0)) &&
           (fabs(dec_d) < 90.0 || (dec_m == 0.0 && dec_s == 0.0));
}

static double clampd(double x, double lo, double hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static double wrap_az_180(double az_deg)
{
    while (az_deg > 180.0)
        az_deg -= 360.0;
    while (az_deg < -180.0)
        az_deg += 360.0;
    return az_deg;
}

static double wrap_az_360(double az_deg)
{
    while (az_deg >= 360.0)
        az_deg -= 360.0;
    while (az_deg < 0.0)
        az_deg += 360.0;
    return az_deg;
}

static double angular_distance_deg(AltAz a, AltAz b)
{
    Vec3 va = normalize(altaz_to_vector(a));
    Vec3 vb = normalize(altaz_to_vector(b));

    double c = dot(va, vb);
    c = clampd(c, -1.0, 1.0);

    return acos(c) * RAD2DEG;
}
