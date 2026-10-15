/**
 * @file    telescope_other.c
 * @brief   Direction calibration, skybox assist, and telescope status utilities.
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
#include "headers/telescope_other.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier for telescope utility operations.
 */
#define TRACE_NAME COMMON_FUNCTIONS

/**
 * @brief   Configured trace level for telescope utility operations.
 */
#define TRACE_LEVEL COMMON_FUNCTIONS_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Observer elevation in meters for solar-system visibility calculations.
 */
#define OBSERVER_HEIGHT_METERS 500.0

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

/**
 * @brief   Direction-calibration command polling interval in milliseconds.
 */
#define DIRECTION_CALIBRATION_POLL_MS 50

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Refresh pending input and check whether a direction-calibration session must abort.
 * @param   generation Calibration generation captured before the first movement.
 * @return  True on disconnect, superseded calibration, or the existing CMD NONE exit.
 */
static bool direction_calibration_cancelled(uint64_t generation);

/**
 * @brief   Wait for an exact direction-calibration answer while its session remains active.
 * @param   generation Calibration generation captured before the first movement.
 * @param   command Expected bare command name.
 * @param   reverse Receives a valid 0/1 answer; NULL when waiting for bare NEXT.
 * @param   stage Handshake stage name for diagnostics.
 * @return  True for valid input; false on malformed matching input or cancellation.
 */
static bool wait_for_direction_command(uint64_t generation, const char *command, bool *reverse, const char *stage);

/**
 * @brief   Check whether session/manual-control cancellation or a command requests a skybox stop.
 * @param   calibrating Whether the scan belongs to an unfinished alignment.
 * @param   generation Calibration generation captured when the skybox scan began.
 * @return  1 if the skybox scan should stop; otherwise 0.
 */
static inline int skybox_should_stop(bool calibrating, uint64_t generation);

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   Supported solar-system bodies in visibility reply order.
 */
static const astro_body_t supported_bodies[] = {BODY_MERCURY, BODY_VENUS,   BODY_MARS, BODY_JUPITER, BODY_SATURN,
                                                BODY_URANUS,  BODY_NEPTUNE, BODY_MOON, BODY_SUN};

_Static_assert((sizeof(supported_bodies) / sizeof(supported_bodies[0])) == SOLAR_OBJECT_COUNT,
               "SOLAR_OBJECT_COUNT must match supported_bodies");

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Solar-system catalogue with placeholder coordinates.
 */
CelestialObject object_db_planets[] = {
    // These coordinates are placeholders because solar-system coordinates change over time.
    {"Mercury", 23, 12, 0.0, -5,  9,  0, "Planet",       -1.9 },
    {"Venus",   10, 24, 0.0, 12,  0,  0, "Planet",       -4.6 },
    {"Mars",    21, 30, 0.0, -18, 20, 0, "Planet",       -2.0 },
    {"Jupiter", 15, 0,  0.0, -15, 0,  0, "Planet",       -2.7 },
    {"Saturn",  19, 0,  0.0, -22, 30, 0, "Planet",       0.7  },
    {"Uranus",  2,  40, 0.0, 15,  0,  0, "Planet",       5.7  },
    {"Neptune", 23, 30, 0.0, -6,  30, 0, "Planet",       7.8  },
    {"Moon",    8,  0,  0.0, 20,  0,  0, "Moon",         -12.7},
    {"Pluto",   19, 48, 0.0, -22, 30, 0, "Dwarf Planet", 13.7 },
    {"Sun",     7,  45, 0.0, 20,  0,  0, "Star",         -26.7}
};

/**
 * @brief   Number of entries in the solar-system catalogue.
 */
int PLANETS_DATABASE_SIZE = (int) (sizeof(object_db_planets) / sizeof(object_db_planets[0]));

/* Public Functions ------------------------------------------------------------------------------------------------- */

void update_stepper_enable_state()
{
    if (STEPPER_1_ENABLED)
    {
        gpio_set_level(ENABLE_PIN_1, 0); // 0 = enable
    }
    else
    {
        gpio_set_level(ENABLE_PIN_1, 1); // 1 = disable
    }

    if (STEPPER_2_ENABLED)
    {
        gpio_set_level(ENABLE_PIN_2, 0);
    }
    else
    {
        gpio_set_level(ENABLE_PIN_2, 1);
    }
}

void calibrate_direction_flags(void)
{
    const uint64_t generation = mobile_app_calibration_generation();
    bool candidate_direction_1 = DIRECTION_1;
    bool candidate_direction_2 = DIRECTION_2;
    bool reverse = false;
    bool completed = false;

    // Drop the consumed invoking command before refreshing cancellation/input.
    // Any answer arriving during the move must survive until the response wait.
    mobile_app_command_clear();
    if (direction_calibration_cancelled(generation))
    {
        goto finish;
    }
    TRACE_INF("Direction ALT move requested: speed %.3f, ratio %.6f, enabled %d", global_max_speed, GEAR_RATIO_1,
              STEPPER_1_ENABLED);
    if (!move_steppers(10.0, global_max_speed, 0.0, global_max_speed))
    {
        TRACE_ERR("Direction calibration ALT move failed");
        goto finish;
    }
    TRACE_INF("Direction ALT move returned");
    if (direction_calibration_cancelled(generation))
    {
        goto finish;
    }

    send_ready_status(3);
    if (!wait_for_direction_command(generation, APP_ARG_DIR_CALIB_RESULT, &reverse, "ALT DIR_CALIB_RESULT"))
    {
        goto finish;
    }

    if (reverse)
    {
        candidate_direction_1 = !candidate_direction_1;
    }

    send_ready_status(3);
    mobile_app_command_clear();
    if (!wait_for_direction_command(generation, APP_RX_NEXT, NULL, "NEXT before AZ move"))
    {
        goto finish;
    }

    // NEXT has been consumed; retain any later answer through READY and the move.
    mobile_app_command_clear();
    send_ready_status(3);
    if (direction_calibration_cancelled(generation))
    {
        goto finish;
    }
    TRACE_INF("Direction AZ move requested: speed %.3f, ratio %.6f, enabled %d", global_max_speed, GEAR_RATIO_2,
              STEPPER_2_ENABLED);
    if (!move_steppers(0.0, global_max_speed, -10.0, global_max_speed))
    {
        TRACE_ERR("Direction calibration AZ move failed");
        goto finish;
    }
    TRACE_INF("Direction AZ move returned");
    if (direction_calibration_cancelled(generation))
    {
        goto finish;
    }

    send_ready_status(3);
    if (!wait_for_direction_command(generation, APP_ARG_DIR_CALIB_RESULT, &reverse, "AZ DIR_CALIB_RESULT"))
    {
        goto finish;
    }

    if (reverse)
    {
        candidate_direction_2 = !candidate_direction_2;
    }
    // The accepted final answer/session check starts the commit. A later disconnect
    // does not undo completed measurements or a successful save.
    if (mobile_app_calibration_cancelled(generation))
    {
        goto finish;
    }
    const esp_err_t err = save_directions_to_flash(candidate_direction_1, candidate_direction_2);
    if (err != ESP_OK)
    {
        TRACE_ERR("Could not save calibrated directions: %s", esp_err_to_name(err));
        goto finish;
    }
    DIRECTION_1 = candidate_direction_1;
    DIRECTION_2 = candidate_direction_2;
    completed = true;
    TRACE_INF("Direction calibration complete and saved");

finish:
    if (!completed)
    {
        mobile_app_command_clear();
        TRACE_ERR("Direction calibration failed; previous runtime directions preserved");
    }

    // Preserve both completion replies and their delays, after the save result is known.
    // Failure uses the existing response value; the void caller sends READY as before.
    char buffer[64];
    for (int i = 0; i < 2; ++i)
    {
        snprintf(buffer, sizeof(buffer), APP_TX_ESP_RESPONSE_FORMAT, completed ? 1 : 0);
        ble_notify_app((uint8_t *) buffer, strlen(buffer));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // This void routine owns its completion display state so cancellation cannot be
    // mistaken for success by the command handler.
    global_status_message = completed ? STATUS_DIRECTION_CALIB : STATUS_IDLE;
}

void sky_box_assists(int magnification, int speed_option)
{
    const bool calibrating = mobile_app_calibration_pending();
    const uint64_t generation = mobile_app_calibration_generation();
    bool temp_accel = global_acceleration;
    global_acceleration = true;

    float increment = 0.25f;
    if (magnification == 3)
        increment = 0.1f;
    else if (magnification == 2)
        increment = 0.25f;
    else
        increment = 0.5f;

    float speed = 0.4f;
    if (speed_option == 3)
        speed = 0.75f;
    else if (speed_option == 2)
        speed = 0.4f;
    else
        speed = 0.20f;

    TRACE_DEB("Skybox setup - increment: %f - speed: %f", increment, speed);

    mobile_app_command_clear();

    send_ready_status(3);
    if (calibrating && skybox_should_stop(calibrating, generation))
    {
        goto exit_loops;
    }

    // Spiral: L1, D1, R2, U2, L3, D3, ...
    // Each n adds a pair of legs, each containing n increments.
    const int max_n = 12; // The final pair ends at U12.

    for (int n = 1; n <= max_n; n++)
    {
        // Each "n" has two legs:
        // odd n: LEFT then DOWN
        // even n: RIGHT then UP
        // Each leg consists of n steps of size 'increment'
        if (n & 1)
        {
            // LEFT n
            for (int s = 0; s < n; s++)
            {
                // LEFT applies the cosine-scaled azimuth increment.
                float tilt = (float) global_possition.alt * DEG2RAD;
                float c = cosf(tilt);
                if (fabsf(c) < 0.1f)
                    c = (c < 0.0f) ? -0.1f : 0.1f;

                float step_left = increment / c;

                move_steppers(0.0, speed, step_left, speed);
                global_possition.az +=
                    -(global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                vTaskDelay(pdMS_TO_TICKS(30));

                if (skybox_should_stop(calibrating, generation))
                    goto exit_loops;
            }

            // DOWN n
            for (int s = 0; s < n; s++)
            {
                if (!move_steppers_skybox_altitude(-increment, speed) && global_status_bar_screen_telescope)
                {
                    TRACE_WAR("Skybox stopped: altitude increment rejected at %.6f deg", global_possition.alt);
                    goto exit_loops;
                }
                global_possition.alt +=
                    -(global_count_steps_y / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0));
                if (global_status_bar_screen_telescope && (global_possition.alt <= 0.0 || global_possition.alt >= 90.0))
                {
                    goto exit_loops;
                }
                vTaskDelay(pdMS_TO_TICKS(30));

                if (skybox_should_stop(calibrating, generation))
                    goto exit_loops;
            }
        }
        else
        {
            // RIGHT n
            for (int s = 0; s < n; s++)
            {
                float tilt = (float) global_possition.alt * DEG2RAD;
                float c = cosf(tilt);
                if (fabsf(c) < 0.1f)
                    c = (c < 0.0f) ? -0.1f : 0.1f;

                float step_right = increment / c;

                // RIGHT negates the cosine-scaled azimuth increment.
                move_steppers(0.0, speed, -step_right, speed);
                global_possition.az +=
                    (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                vTaskDelay(pdMS_TO_TICKS(30));

                if (skybox_should_stop(calibrating, generation))
                    goto exit_loops;
            }

            // UP n
            for (int s = 0; s < n; s++)
            {
                if (!move_steppers_skybox_altitude(+increment, speed) && global_status_bar_screen_telescope)
                {
                    TRACE_WAR("Skybox stopped: altitude increment rejected at %.6f deg", global_possition.alt);
                    goto exit_loops;
                }
                global_possition.alt +=
                    (global_count_steps_y / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0));
                if (global_status_bar_screen_telescope && (global_possition.alt <= 0.0 || global_possition.alt >= 90.0))
                {
                    goto exit_loops;
                }
                vTaskDelay(pdMS_TO_TICKS(30));

                if (skybox_should_stop(calibrating, generation))
                    goto exit_loops;
            }
        }

        TRACE_VER("Skybox spiral n: %d", n);
    }

exit_loops:
    global_acceleration = temp_accel;
}

void send_ready_status(int repeat)
{
    global_esp_busy = false;
    for (int i = 0; i < repeat; i++)
    {
        ble_notify_app((uint8_t *) APP_TX_READY, APP_TX_READY_LENGTH);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void fetch_solar_objects_visibility_status(bool *visible_objects_array)
{
    if (visible_objects_array == NULL)
    {
        TRACE_ERR("Cannot store solar-system visibility in a null array");
        return;
    }

    for (size_t i = 0; i < SOLAR_OBJECT_COUNT; ++i)
    {
        visible_objects_array[i] = false;
    }

    astro_time_t time = make_current_astronomy_time();
    astro_observer_t observer = Astronomy_MakeObserver(global_latitude, global_longitude, OBSERVER_HEIGHT_METERS);

    for (size_t i = 0; i < SOLAR_OBJECT_COUNT; ++i)
    {
        astro_body_t body = supported_bodies[i];

        // Preserve the reset before each body calculation.
        Astronomy_Reset();

        astro_equatorial_t equatorial = Astronomy_Equator(body, &time, observer, EQUATOR_OF_DATE, ABERRATION);

        if (equatorial.status != ASTRO_SUCCESS)
        {
            TRACE_ERR("Failed to calculate visibility for body %d: status %d", (int) body, (int) equatorial.status);
            continue;
        }

        RADec object = {.ra = equatorial.ra, .dec = equatorial.dec};

        visible_objects_array[i] = is_object_visible(object, global_alt_visib_limit);
        TRACE_DEB("Body %d is %s", (int) body, visible_objects_array[i] ? "visible" : "not visible");
    }
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static bool direction_calibration_cancelled(uint64_t generation)
{
    mobile_app_command_refresh();
    return mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0;
}

static bool wait_for_direction_command(uint64_t generation, const char *command, bool *reverse, const char *stage)
{
    const size_t command_length = strlen(command);
    bool unexpected_reported = false;
    TRACE_INF("Direction calibration waiting: %s", stage);

    while (1)
    {
        if (direction_calibration_cancelled(generation))
        {
            TRACE_WAR("Direction calibration cancelled while waiting for %s", stage);
            return false;
        }
        if (strncmp(global_last_ble_command, command, command_length) == 0)
        {
            if (reverse != NULL)
            {
                if (strcmp(global_last_ble_command, APP_RX_DIR_CALIB_RESULT_PREFIX "0") == 0)
                {
                    *reverse = false;
                    return true;
                }
                if (strcmp(global_last_ble_command, APP_RX_DIR_CALIB_RESULT_PREFIX "1") == 0)
                {
                    *reverse = true;
                    return true;
                }
            }
            else if (strcmp(global_last_ble_command, command) == 0)
            {
                return true;
            }
            TRACE_ERR("Invalid direction calibration input while waiting for %s: %s", stage, global_last_ble_command);
            return false;
        }
        if (global_last_ble_command[0] != '\0')
        {
            // Clear only this snapshot. Do not erase a newer pending arrival or
            // infer a missing direction answer from an out-of-order NEXT.
            if (!unexpected_reported)
            {
                TRACE_WAR("Direction calibration: unexpected command while waiting for %s", stage);
                unexpected_reported = true;
            }
            mobile_app_command_clear();
        }
        vTaskDelay(pdMS_TO_TICKS(DIRECTION_CALIBRATION_POLL_MS));
    }
}

static inline int skybox_should_stop(bool calibrating, uint64_t generation)
{
    mobile_app_command_refresh();
    if ((calibrating && mobile_app_calibration_cancelled(generation)) || mobile_app_manual_control_cancelled())
    {
        return 1;
    }
    // Commands must stay EXACTLY the same strings
    const char *CMD_APP_STOP = APP_RX_APP_SKY_BOX_ASSIST_STOP;
    const char *CMD_ANALOG_STOP = APP_RX_ANALOG_SKY_BOX_ASSIST_STOP;
    const char *CMD_NONE = APP_RX_NONE;

    if (strncmp(global_last_ble_command, CMD_APP_STOP, strlen(CMD_APP_STOP)) == 0)
        return 1;
    if (strncmp(global_last_ble_command, CMD_ANALOG_STOP, strlen(CMD_ANALOG_STOP)) == 0)
        return 1;
    if (strncmp(global_last_ble_command, CMD_NONE, strlen(CMD_NONE)) == 0)
        return 1;
    return 0;
}
