/**
 * @file    telescope_tracking.c
 * @brief   Track a fixed right ascension and declination target.
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

#include "headers/telescope_tracking.h"
#include "headers/telescope_common.h"
#include "mobile_app/headers/mobile_app_commands.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for equatorial tracking.
 */
#define TRACE_NAME TRACKING

/**
 * @brief   Trace verbosity for equatorial tracking.
 */
#define TRACE_LEVEL TRACKING_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Interval between tracking speed recalculations, in microseconds.
 */
#define TRACKING_SPEED_UPDATE_PERIOD_US (30LL * 1000000LL)

/**
 * @brief   Requested task delay while awaiting a tracking update, in milliseconds.
 */
#define TRACKING_POLL_DELAY_MS 2

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/**
 * @brief   Altitude and azimuth motor speeds for tracking, in degrees per second.
 */
typedef struct
{
    float alt; /**< Altitude motor speed. */
    float az;  /**< Azimuth motor speed. */
} tracking_speeds_t;

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Clamp a motor speed to the supplied limits.
 * @param   speed Requested speed in degrees per second.
 * @param   minimum Lower speed limit in degrees per second.
 * @param   maximum Upper speed limit in degrees per second.
 * @return  Speed limited to the supplied range.
 */
static float clamp_speed(float speed, float minimum, float maximum);

/**
 * @brief   Refresh the command snapshot and check for a tracking-interrupting CMD prefix.
 * @return  True when an available command begins with the CMD prefix; otherwise false.
 */
static bool stop_requested(bool calibrating, uint64_t generation);

/**
 * @brief   Select tracking motor speeds for the altitude and refresh interval.
 * @param   altitude Target altitude in degrees.
 * @param   refresh_s Tracking update interval in seconds.
 * @return  Altitude and azimuth motor speeds in degrees per second.
 */
static tracking_speeds_t calculate_tracking_speeds(float altitude, float refresh_s);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void RA_Dec_tracking(RADec radec)
{
    const bool calibrating = mobile_app_calibration_pending();
    const uint64_t generation = mobile_app_calibration_generation();
    // Accept the existing coordinate conventions; reject only nonfinite inputs.
    if (!isfinite(radec.ra) || !isfinite(radec.dec))
    {
        TRACE_ERR("Cannot start tracking: nonfinite RA/Dec");
        return;
    }

    // Snapshot once per session, as before. Preserve float rounding before the cast.
    const float refresh_s = global_tracking_refreshment;
    const float interval_us = refresh_s * 1000000.0f;
    // 2^63 is the first out-of-range positive float for an int64_t conversion.
    if (!isfinite(interval_us) || interval_us < 1.0f || interval_us >= 0x1p63f)
    {
        TRACE_ERR("Cannot start tracking: invalid refresh interval %g s", (double) refresh_s);
        return;
    }
    const int64_t update_interval_us = (int64_t) interval_us;

    TRACE_INF("Tracking started: refresh=%g s", (double) refresh_s);

    const StatusMessage previous_status = global_status_message;
    global_status_message = STATUS_TRACKING_ON;
    update_display_screen();
    global_status_is_tracking_active = true;

    bool horizon_message_shown = false;
    int64_t last_position_update_us = esp_timer_get_time();
    const bool previous_acceleration = global_acceleration;
    global_acceleration = false;

    tracking_speeds_t speeds = {.alt = 0.04f, .az = 0.04f};
    // Keep the legacy initial speed-update timing relative to device uptime.
    int64_t last_speed_update_us = 0;
    const TickType_t poll_delay = pdMS_TO_TICKS(TRACKING_POLL_DELAY_MS);

    while (1)
    {
        if (stop_requested(calibrating, generation))
        {
            break;
        }

        const int64_t now = esp_timer_get_time();
        if ((now - last_position_update_us) <= update_interval_us)
        {
            vTaskDelay(poll_delay);
            continue;
        }
        last_position_update_us = now;

        // Visibility updates UT/LST and suppresses app replies while tracking is active.
        // Retain half of the normal altitude limit for tracking.
        if (!is_object_visible(radec, global_alt_visib_limit / 2.0f))
        {
            if (!horizon_message_shown)
            {
                TRACE_INF("Tracking paused: horizon limit reached");
                global_status_message = STATUS_HORIZ_LIMIT;
                update_display_screen();
                horizon_message_shown = true;
            }
            // Stay active and recheck visibility. Preserve the one-message-per-session
            // display behavior, including when motion later resumes.
            continue;
        }

        if ((now - last_speed_update_us) > TRACKING_SPEED_UPDATE_PERIOD_US)
        {
            last_speed_update_us = now;
            const AltAz altaz = radec_to_altaz(radec, global_latitude, global_lst);
            const float altitude = (float) altaz.alt;
            if (!isfinite(altitude))
            {
                TRACE_ERR("Tracking aborted: nonfinite altitude");
                break;
            }
            speeds = calculate_tracking_speeds(altitude, refresh_s);
        }

        // Check again after calculations. The existing move is blocking and cannot
        // be interrupted here; a command received during it is handled after return.
        if (stop_requested(calibrating, generation))
        {
            break;
        }
        move_to_RA_Dec(radec.ra, radec.dec, speeds.alt, speeds.az);
    }

    // Every exit after activation restores the original state in the original order.
    global_acceleration = previous_acceleration;
    TRACE_INF("Tracking ended");
    global_status_message = previous_status;
    update_display_screen();
    global_status_is_tracking_active = false;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static float clamp_speed(float speed, float minimum, float maximum)
{
    return (speed < minimum) ? minimum : (speed > maximum) ? maximum : speed;
}

static bool stop_requested(bool calibrating, uint64_t generation)
{
    mobile_app_command_refresh();
    // Any CMD interrupts tracking. Leave it intact for the joystick caller.
    // Ordinary tracking still survives disconnect. Nested calibration tracking
    // must return so its owning alignment can abort.
    return (calibrating && mobile_app_calibration_cancelled(generation)) ||
           (global_new_command_available &&
            (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0));
}

static tracking_speeds_t calculate_tracking_speeds(float altitude, float refresh_s)
{
    float base = 0.035f;
    if (refresh_s > 5.0f)
    {
        base = 0.060f;
    }

    float scale = 1.0f;
    if (altitude > 70.0f)
    {
        scale = 1.0f + (altitude - 70.0f) * 0.03f;
    }
    if (altitude > 80.0f)
    {
        scale = 1.3f + (altitude - 80.0f) * 0.05f;
    }

    float speed = clamp_speed(base * scale, 0.02f, 0.18f);
    float az_multiplier = 1.0f;
    if (altitude > 75.0f)
    {
        az_multiplier = 1.0f + (altitude - 75.0f) * 0.03f;
    }

    return (tracking_speeds_t) {.alt = speed, .az = clamp_speed(speed * az_multiplier, 0.02f, 0.25f)};
}
