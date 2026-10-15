/**
 * @file    app_joystick.c
 * @brief   Mobile-app joystick telescope motion control.
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
#include "esp_timer.h"
#include "config.h"
#include "telescope/headers/telescope_stepper.h"
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include "app_joystick.h"
#include "flash.h"
#include "bluetooth/bluetooth.h"
#include "telescope/headers/telescope_common.h"
#include "telescope/headers/telescope_tracking.h"
#include "mobile_app/headers/mobile_app_handlers.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier for joystick control.
 */
#define TRACE_NAME JOYSTICK

/**
 * @brief   Configured trace level for joystick control.
 */
#define TRACE_LEVEL JOYSTICK_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Wrap azimuth to [0, 360) degrees without discarding altitude travel.
 * @param   p Coordinates to normalize in place.
 * @return  None.
 */
static void normalize_altaz(AltAz *p);

/**
 * @brief   Compute the speed-dependent initial joystick interval.
 * @param   target_speed_dps Target speed in degrees per second.
 * @param   START_INTERVAL_BASE_MS Base and maximum initial interval in milliseconds.
 * @param   MIN_INTERVAL_MS_FAST Minimum initial interval in milliseconds.
 * @return  Initial interval in microseconds.
 */
static inline int compute_start_interval_us(float target_speed_dps, int START_INTERVAL_BASE_MS,
                                            int MIN_INTERVAL_MS_FAST);

/**
 * @brief   Apply cubic easing to clamped ramp progress.
 * @param   t Ramp progress, clamped to [0, 1].
 * @return  Cube of the clamped progress.
 */
static inline float ease_in_cubic(float t);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void move_app_joystick()
{
    const uint64_t generation = mobile_app_calibration_generation();
    if (!mobile_app_manual_control_begin())
    {
        TRACE_WAR("App joystick requires a connected mobile app");
        return;
    }

    mobile_app_joystick_watchdog_begin();

    // Local pacing controls the ramp; the caller manages global_acceleration.
    const int START_INTERVAL_BASE_MS = 200; // base initial pause at ~1 deg/s
    const int MIN_INTERVAL_MS_FAST = 20;    // clamp initial pause for high speeds
    const int RAMP_TIME_MS = 900;           // total time to reach full cadence/speed
    const float START_SPEED_DPS = 0.35f;    // gentle initial speed
    const double STEP_DEG = 0.1;            // default per-call nudge angle

    // Smaller nudges for the two lowest app speed settings.
    const double STEP_DEG_LVL1 = 0.002; // speed setting 1
    const double STEP_DEG_LVL2 = 0.005; // speed setting 2

    // Disable accel/ramp below this target speed (deg/s)
    const float NO_ACCEL_BELOW_DPS = 0.5f;

    // Preserve ramp state across calls. Direction STOP or watchdog expiry resets both axes.
    /**
     * @brief   Altitude and azimuth ramp start timestamps in microseconds.
     */
    static int64_t alt_ramp_start_us = 0, az_ramp_start_us = 0; // when ramp began
    /**
     * @brief   Last altitude and azimuth move start timestamps in microseconds.
     */
    static int64_t alt_last_call_us = 0, az_last_call_us = 0; // last executed call
    /**
     * @brief   Initial altitude ramp interval in microseconds.
     */
    static int alt_start_int_us = START_INTERVAL_BASE_MS * 1000;
    /**
     * @brief   Initial azimuth ramp interval in microseconds.
     */
    static int az_start_int_us = START_INTERVAL_BASE_MS * 1000;
    /**
     * @brief   Last active altitude direction sign, or zero after reset.
     */
    static int last_alt_sign = 0; // -1,0,+1
    /**
     * @brief   Last active azimuth direction sign, or zero after reset.
     */
    static int last_az_sign = 0;

    float target_speed_dps = global_max_speed / 2; // target speed (deg/s)

    int64_t last_position_notify_us = esp_timer_get_time();
    const float position_notify_hz = 4.0f; // Hz

    bool invert_alt = false;
    bool invert_az = false;

    // Keep the original app setting for nudge selection, alongside the converted motor speed.
    float app_speed_setting = 5.0f; // last received 1..10

    while (1)
    {
        if (mobile_app_manual_control_cancelled() || mobile_app_calibration_cancelled(generation))
        {
            TRACE_WAR("App joystick stopped after disconnect or new calibration");
            break;
        }
        mobile_app_command_refresh();
        if (strcmp(global_last_ble_command, APP_RX_NONE) == 0)
            break;

        // 1. Handle commands. Directions remain held; one-shot commands are cleared.
        if (strcmp(global_last_ble_command, APP_RX_APP_DIR_STOP) == 0)
        {
            // Reset ramps so next press starts gentle
            int64_t now = esp_timer_get_time();
            alt_ramp_start_us = az_ramp_start_us = now;
            alt_last_call_us = az_last_call_us = 0;
            alt_start_int_us = START_INTERVAL_BASE_MS * 1000;
            az_start_int_us = START_INTERVAL_BASE_MS * 1000;
            last_alt_sign = last_az_sign = 0;

            send_ready_status(1);
            mobile_app_command_clear();
            continue;
        }

        // Positive ALT means up; positive motor AZ means left.
        int alt_sign = 0, az_sign = 0; // Zero means this axis is idle.

        if (strcmp(global_last_ble_command, APP_RX_APP_DIR_UP) == 0)
        {
            alt_sign = 1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_DOWN) == 0)
        {
            alt_sign = -1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_LEFT) == 0)
        {
            az_sign = 1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_RIGHT) == 0)
        {
            az_sign = -1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_ULEFT) == 0)
        {
            alt_sign = 1;
            az_sign = 1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_URIGHT) == 0)
        {
            alt_sign = 1;
            az_sign = -1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_DLEFT) == 0)
        {
            alt_sign = -1;
            az_sign = 1;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_DIR_DRIGHT) == 0)
        {
            alt_sign = -1;
            az_sign = -1;
        }
        else if (strncmp(global_last_ble_command, APP_RX_APP_SPEED, APP_RX_APP_SPEED_MATCH_LENGTH) == 0)
        {
            // Preserve the accepted optional separators before the numeric value.
            const char *p = global_last_ble_command + APP_RAW_OFFSET_APP_SPEED; // right after "CMD APP SPEED"
            while (isspace((unsigned char) *p))
                p++; // skip whitespace

            if (*p == ':' || *p == '=' || *p == ',')
                p++;

            char *end = NULL;
            float new_speed = strtof(p, &end);
            bool valid_speed = end != p && isfinite(new_speed);
            while (isspace((unsigned char) *end))
                end++;
            if (valid_speed && *end == '\0')
            {
                if (new_speed < 1)
                    new_speed = 1;
                if (new_speed > 10)
                    new_speed = 10;

                // The app setting also selects the nudge size below.
                app_speed_setting = new_speed;

                target_speed_dps = 0.1f + ((new_speed - 1.0f) * (global_max_speed - 0.1f)) / 9.0f;
            }

            mobile_app_command_clear();
            continue;
        }
        else if (strcmp(global_last_ble_command, APP_RX_APP_START_TRACKING) == 0)
        {
            mobile_app_joystick_watchdog_end();
            mobile_app_command_clear();
            CALC_LST();
            RADec radec = altaz_to_radec(global_possition, global_latitude, global_lst);
            if (isfinite(radec.ra) && isfinite(radec.dec))
            {
                global_radec_position = radec;
                RA_Dec_tracking(global_radec_position);
            }
            else
            {
                TRACE_WAR("Tracking rejected: nonfinite position");
            }
            if (mobile_app_calibration_cancelled(generation))
                break;
            mobile_app_command_refresh();
            if (strcmp(global_last_ble_command, APP_RX_NONE) == 0)
                break;
            mobile_app_command_clear();
            mobile_app_joystick_watchdog_begin();
            continue;
        }
        else if (strncmp(global_last_ble_command, APP_RX_APP_TOGGLE, APP_RX_APP_TOGGLE_MATCH_LENGTH) == 0)
        {
            int ud = 0, lr = 0;
            if (sscanf(global_last_ble_command + APP_RAW_OFFSET_APP_TOGGLE, "%d %d", &ud, &lr) == 2)
            {
                invert_alt = ud;
                invert_az = lr;
                TRACE_DEB("Invert U/D: %s | R/L: %s", invert_alt ? "ON" : "OFF", invert_az ? "ON" : "OFF");
            }
            mobile_app_command_clear();
            continue;
        }
        else if (strncmp(global_last_ble_command, APP_RX_APP_SKY_BOX_ASSIST_STOP,
                         APP_RX_APP_SKY_BOX_ASSIST_STOP_MATCH_LENGTH) == 0)
        {
            mobile_app_command_clear();
            continue;
        }
        else if (strncmp(global_last_ble_command, APP_RX_APP_SKY_BOX_ASSIST_START,
                         APP_RX_APP_SKY_BOX_ASSIST_START_MATCH_LENGTH) == 0)
        {
            mobile_app_joystick_watchdog_end();
            process_sky_box_assist_command();
            if (mobile_app_calibration_cancelled(generation))
                break;
            mobile_app_command_refresh();
            if (strcmp(global_last_ble_command, APP_RX_NONE) == 0)
                break;
            mobile_app_command_clear();
            mobile_app_joystick_watchdog_begin();
            continue;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const mobile_app_joystick_hold_state_t hold = mobile_app_joystick_watchdog_poll();
        if (hold == MOBILE_APP_JOYSTICK_HOLD_SUPERSEDED)
            continue; // Refresh the newer command without disturbing the ramps.
        if (hold == MOBILE_APP_JOYSTICK_HOLD_EXPIRED)
        {
            // The pulse path already bounds any in-progress nudge or backlash.
            // Leave motor timing/position accounting untouched and never replay.
            int64_t now = esp_timer_get_time();
            alt_ramp_start_us = az_ramp_start_us = now;
            alt_last_call_us = az_last_call_us = 0;
            alt_start_int_us = START_INTERVAL_BASE_MS * 1000;
            az_start_int_us = START_INTERVAL_BASE_MS * 1000;
            last_alt_sign = last_az_sign = 0;
            mobile_app_command_clear();
            continue;
        }

        // Apply the two independent inversion settings only to movement commands.
        if (invert_alt)
            alt_sign = -alt_sign;
        if (invert_az)
            az_sign = -az_sign;

        // 2. Select the nudge size and update each axis's ramp independently.
        double step_deg = STEP_DEG;
        if (app_speed_setting <= 1.5f)
            step_deg = STEP_DEG_LVL1; // speed = 1
        else if (app_speed_setting <= 2.5f)
            step_deg = STEP_DEG_LVL2; // speed = 2

        int64_t now = esp_timer_get_time();

        // Restart only an axis whose requested direction has changed.
        if (alt_sign != 0 && alt_sign != last_alt_sign)
        {
            alt_ramp_start_us = now;
            alt_last_call_us = 0;
            alt_start_int_us =
                compute_start_interval_us(target_speed_dps, START_INTERVAL_BASE_MS, MIN_INTERVAL_MS_FAST);
            last_alt_sign = alt_sign;
        }
        if (az_sign != 0 && az_sign != last_az_sign)
        {
            az_ramp_start_us = now;
            az_last_call_us = 0;
            az_start_int_us = compute_start_interval_us(target_speed_dps, START_INTERVAL_BASE_MS, MIN_INTERVAL_MS_FAST);
            last_az_sign = az_sign;
        }

        // Clamp elapsed ramp progress to [0, 1].
        float alt_ramp_progress = 1.0f, az_ramp_progress = 1.0f; // default = ramp done
        if (alt_sign != 0 && alt_ramp_start_us > 0)
        {
            float dt_ms = (float) (now - alt_ramp_start_us) / 1000.0f;
            alt_ramp_progress = dt_ms / (float) RAMP_TIME_MS;
            if (alt_ramp_progress > 1.0f)
                alt_ramp_progress = 1.0f;
            if (alt_ramp_progress < 0.0f)
                alt_ramp_progress = 0.0f;
        }
        if (az_sign != 0 && az_ramp_start_us > 0)
        {
            float dt_ms = (float) (now - az_ramp_start_us) / 1000.0f;
            az_ramp_progress = dt_ms / (float) RAMP_TIME_MS;
            if (az_ramp_progress > 1.0f)
                az_ramp_progress = 1.0f;
            if (az_ramp_progress < 0.0f)
                az_ramp_progress = 0.0f;
        }

        // Very low speeds use constant cadence and speed, without a ramp.
        bool disable_ramp = (target_speed_dps <= NO_ACCEL_BELOW_DPS);
        if (disable_ramp)
        {
            alt_ramp_progress = 1.0f;
            az_ramp_progress = 1.0f;
        }

        // Ramp pacing starts slowly, then approaches the steady nudge interval.
        int alt_interval_us = (int) ((float) alt_start_int_us * (1.0f - alt_ramp_progress));
        int az_interval_us = (int) ((float) az_start_int_us * (1.0f - az_ramp_progress));

        // Keep a nonzero interval even after the ramp has finished.
        float cadence_speed_dps = target_speed_dps;
        if (cadence_speed_dps < 0.001f)
            cadence_speed_dps = 0.001f;

        int steady_interval_us = (int) ((step_deg / cadence_speed_dps) * 1e6f);
        if (steady_interval_us < 1000)
            steady_interval_us = 1000; // don't spam faster than 1ms

        if (alt_interval_us < steady_interval_us)
            alt_interval_us = steady_interval_us;
        if (az_interval_us < steady_interval_us)
            az_interval_us = steady_interval_us;

        // If ramp disabled: force constant cadence
        if (disable_ramp)
        {
            alt_interval_us = steady_interval_us;
            az_interval_us = steady_interval_us;
        }

        // Is each axis ready to nudge now?
        bool do_alt = (alt_sign != 0) && (alt_last_call_us == 0 || (now - alt_last_call_us) >= alt_interval_us);
        bool do_az = (az_sign != 0) && (az_last_call_us == 0 || (now - az_last_call_us) >= az_interval_us);

        // 3. Execute a single combined move, then account for the actual step counts.
        if (!do_alt && !do_az)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else
        {
            // Ease the commanded speed from START_SPEED_DPS to the target.
            float alt_speed_cmd =
                disable_ramp
                    ? target_speed_dps
                    : (START_SPEED_DPS + (target_speed_dps - START_SPEED_DPS) * ease_in_cubic(alt_ramp_progress));
            float az_speed_cmd =
                disable_ramp
                    ? target_speed_dps
                    : (START_SPEED_DPS + (target_speed_dps - START_SPEED_DPS) * ease_in_cubic(az_ramp_progress));

            // Keep a nonzero dummy speed for an axis that is not moving.
            double alt_deg = do_alt ? (step_deg * (alt_sign > 0 ? +1 : -1)) : 0.0;
            double az_deg = do_az ? (step_deg * (az_sign > 0 ? +1 : -1)) : 0.0;

            float alt_speed = do_alt ? alt_speed_cmd : 3.0f;
            float az_speed = do_az ? az_speed_cmd : 3.0f;

            const bool watchdog_timed_out = move_steppers_app_manual(alt_deg, alt_speed, az_deg, az_speed);

            // Timestamp the start of the blocking move, not its completion.
            if (do_alt)
                alt_last_call_us = now;
            if (do_az)
                az_last_call_us = now;

            // Motor AZ direction and sky AZ coordinates have opposite signs.
            if (do_alt)
            {
                global_possition.alt +=
                    (alt_deg > 0 ? 1 : -1) *
                    (global_count_steps_y / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0));
            }
            if (do_az)
            {
                if (az_deg > 0)
                {
                    global_possition.az +=
                        -1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                }
                else
                {
                    global_possition.az +=
                        1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                }
            }
            if (watchdog_timed_out)
            {
                // A newer explicit direction may already be pending. Reset the
                // expired ramp, but leave command ownership to the next refresh.
                alt_ramp_start_us = az_ramp_start_us = now;
                alt_last_call_us = az_last_call_us = 0;
                alt_start_int_us = START_INTERVAL_BASE_MS * 1000;
                az_start_int_us = START_INTERVAL_BASE_MS * 1000;
                last_alt_sign = last_az_sign = 0;
            }
        }

        normalize_altaz(&global_possition);

        // 4. Notify position only on the direction path, as before (also while pacing).
        if ((esp_timer_get_time() - last_position_notify_us) > (int64_t) ((1.0f / position_notify_hz) * 1e6f))
        {
            last_position_notify_us = esp_timer_get_time();

            CALC_LST();
            RADec radec = altaz_to_radec(global_possition, global_latitude, global_lst);
            if (isfinite(radec.ra) && isfinite(radec.dec))
            {
                RA_Dec_Split radec_split = split_radec(radec);
                if (isfinite(radec_split.ra_seconds) && isfinite(radec_split.dec_arcseconds))
                {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), APP_TX_RADEC_FORMAT, radec_split.ra_hours, radec_split.ra_minutes,
                             (int) radec_split.ra_seconds, radec_split.dec_degrees, radec_split.dec_arcminutes,
                             (int) radec_split.dec_arcseconds);
                    ble_notify_app((uint8_t *) buffer, strlen(buffer));
                }
            }
        }
    }
    mobile_app_joystick_watchdog_end();
    mobile_app_manual_control_end();
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static void normalize_altaz(AltAz *p)
{
    while (p->az >= 360.0)
        p->az -= 360.0;
    while (p->az < 0.0)
        p->az += 360.0;
}

static inline int compute_start_interval_us(float target_speed_dps, int START_INTERVAL_BASE_MS,
                                            int MIN_INTERVAL_MS_FAST)
{
    if (target_speed_dps < 0.1f)
        target_speed_dps = 0.1f;
    int start_ms = (int) ((float) START_INTERVAL_BASE_MS / target_speed_dps);
    if (start_ms < MIN_INTERVAL_MS_FAST)
        start_ms = MIN_INTERVAL_MS_FAST;
    if (start_ms > START_INTERVAL_BASE_MS)
        start_ms = START_INTERVAL_BASE_MS;
    return start_ms * 1000;
}

static inline float ease_in_cubic(float t)
{
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return t * t * t;
}
