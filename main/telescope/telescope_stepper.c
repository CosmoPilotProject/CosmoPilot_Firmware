/**
 * @file    telescope_stepper.c
 * @brief   Stepper motor motion, GoTo stopping, and backlash calibration.
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
#include <math.h>
#include "esp_timer.h"
#include "config.h"
#include "headers/telescope_stepper.h"
#include "headers/telescope_common.h"
#include <string.h>
#include "bluetooth/bluetooth.h"
#include "flash.h"
#include <limits.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "mobile_app/headers/mobile_app_command_buffer.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier for stepper control.
 */
#define TRACE_NAME TELESCOPE_STEPPER

/**
 * @brief   Configured trace level for stepper control.
 */
#define TRACE_LEVEL TELESCOPE_STEPPER_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Minimum high time of a step pulse, in microseconds.
 */
#define STEP_PULSE_US 10

/**
 * @brief   Maximum altitude angle magnitude per move, in degrees.
 */
#define ALT_LIMIT_DEG 95.0 //safety check so telescope doesnt perform too big movement

/**
 * @brief   Maximum azimuth motor angle magnitude per move, in degrees.
 */
#define AZ_LIMIT_DEG 365.0

/**
 * @brief   Ramp acceleration in degrees per second squared.
 */
#define ACCEL_DPS2 5.0 // desired acceleration [deg/s²]

/**
 * @brief   Initial ramp speed in degrees per second.
 */
#define MIN_START_SPEED_DPS 1.0 // starting speed for ramp-up [deg/s]

/**
 * @brief   Minimum acceleration ramp duration in milliseconds.
 */
#define MIN_ACCEL_TIME_MS 150.0 // minimum ramp time (~0.15 s)

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/**
 * @brief   Published GoTo phase governing Stop GoTo acceptance.
 */
typedef enum
{
    GOTO_MOTION_INACTIVE,              /**< No active GoTo move. */
    GOTO_MOTION_ACCELERATING,          /**< Acceleration or initial backlash take-up is active. */
    GOTO_MOTION_CRUISING,              /**< At least one axis is cruising. */
    GOTO_MOTION_FINAL_DECELERATION,    /**< No axis remains accelerating or cruising. */
    GOTO_MOTION_STOPPING,              /**< A stop request is pending or being applied. */
    GOTO_MOTION_POSITION_UNRECOVERABLE /**< Position recovery is unavailable, so Stop GoTo is disabled. */
} goto_motion_phase_t;

/**
 * @brief   Application-task state for handling a GoTo stop.
 */
typedef struct
{
    goto_motion_phase_t published_phase; /**< Last phase published to the stop-request handler. */
    bool request_pending;                /**< A latched stop request remains active. */
    bool alt_handled;                    /**< The stop request has been considered for altitude. */
    bool az_handled;                     /**< The stop request has been considered for azimuth. */
    bool stop_started;                   /**< Stop deceleration has been requested for an axis. */
    bool ignored_final_deceleration;     /**< The request reached no cruising axis and was ignored. */
} goto_motion_state_t;

/**
 * @brief   Motion phase of one GoTo axis.
 */
typedef enum
{
    GOTO_AXIS_FINISHED,          /**< The axis step target has been reached. */
    GOTO_AXIS_ACCELERATING,      /**< The axis interval has not settled at cruising speed. */
    GOTO_AXIS_CRUISING,          /**< The axis is ready for cruising or stop deceleration. */
    GOTO_AXIS_FINAL_DECELERATION /**< The axis is in its final ramp segment. */
} goto_axis_phase_t;

/**
 * @brief   Actual take-up pulses and App-only deadline outcome.
 */
typedef struct
{
    int alt_pulses;
    int az_pulses;
    bool timed_out;
} backlash_move_result_t;

/**
 * @brief   Physical slack remaining after an interrupted App backlash move.
 *
 * Retained until main travel confirms contact. For known initial contact,
 * opposite motion traverses only slack already taken up; identical motion
 * completes the remainder. Unknown startup contact begins one full span from
 * either endpoint; bounded distances never claim an unproven contact.
 */
typedef struct
{
    bool active;
    bool target_level;
    int span;
    int to_target;
    int to_origin;
} interrupted_backlash_t;

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

static backlash_move_result_t move_steppers_backlash_internal(int steps1, float speed1, int steps2, float speed2,
                                                              bool app_watchdog);
static void move_steppers_manual_internal(double angle1, float speed1, double angle2, float speed2,
                                          bool *app_timed_out);
static void record_backlash_pulses(interrupted_backlash_t *state, bool direction_level, int pulses);

/**
 * @brief   Count whole altitude pulses available toward the aligned manual limit.
 * @param   angle Signed altitude angle; its sign selects the boundary.
 * @return  Nonnegative pulse budget, or zero for an invalid position/scale.
 */
static int manual_altitude_step_limit(double angle);

/**
 * @brief   Get the integer altitude step scale.
 * @return  Configured altitude motor steps per degree, truncated to an integer.
 */
static inline int steps_per_deg_axis1(void);

/**
 * @brief   Get the integer azimuth step scale.
 * @return  Configured azimuth motor steps per degree, truncated to an integer.
 */
static inline int steps_per_deg_axis2(void);

/**
 * @brief   Compute ramp steps for the configured acceleration and minimum ramp time.
 * @param   steps_per_deg Axis motor steps per degree.
 * @param   speed_dps Target angular speed in degrees per second.
 * @return  One step at speeds up to 0.1 degrees per second; otherwise at least two.
 */
static inline int accel_steps_needed(int steps_per_deg, float speed_dps);

/**
 * @brief   Compute the fractional step-interval change for a ramp.
 * @param   current_delay_us Current step interval in microseconds.
 * @param   target_delay_us Target step interval in microseconds.
 * @param   accel_steps Ramp step count, treated as at least one.
 * @return  Absolute interval change per ramp step, in microseconds.
 */
static inline double ramp_delta_us(double current_delay_us, double target_delay_us, int accel_steps);

/**
 * @brief   Classify an axis using the current GoTo ramp state.
 * @param   steps Total main-motion step target.
 * @param   completed Main-motion pulses already emitted.
 * @param   delay Current step interval in microseconds.
 * @param   target_delay Cruising step interval in microseconds.
 * @param   ramp_steps Acceleration and deceleration step count.
 * @return  Current axis motion phase.
 */
static goto_axis_phase_t goto_axis_phase(int steps, int completed, double delay, double target_delay, int ramp_steps);

/**
 * @brief   Publish a changed GoTo phase for Stop GoTo requests.
 * @param   state Motion state tracking the last published phase.
 * @param   phase New phase to publish.
 * @return  None.
 */
static void publish_goto_phase(goto_motion_state_t *state, goto_motion_phase_t phase);

/**
 * @brief   Prepare Stop GoTo deceleration without extending the axis target.
 * @param   steps Total step target, shortened in place.
 * @param   completed Main-motion pulses already emitted.
 * @param   delay Current step interval in microseconds, updated as needed.
 * @param   ramp_steps Deceleration step count, initialized if absent.
 * @param   increment Interval increase per step in microseconds, updated if needed.
 * @param   speed Angular speed in degrees per second.
 * @param   steps_per_degree Axis motor steps per degree.
 * @return  None.
 */
static void shorten_goto_axis(int *steps, int completed, double *delay, int *ramp_steps, double *increment, float speed,
                              int steps_per_degree);

/**
 * @brief   Check numeric inputs for Stop GoTo step and delay calculations.
 * @param   angle Signed motor angle in degrees.
 * @param   speed Angular speed in degrees per second.
 * @param   steps_per_degree Axis motor steps per degree.
 * @return  True if the numeric checks pass; otherwise false.
 */
static bool goto_axis_input_valid(double angle, float speed, double steps_per_degree);

/**
 * @brief   Check conversion and ramp arithmetic before either axis can move.
 * @param   angle Signed motor angle in degrees.
 * @param   speed Angular speed; ordinary conversion retains its magnitude convention.
 * @param   steps_per_degree Configured motor steps per degree.
 * @param   backlash_steps Configured backlash pulse count.
 * @return  True when this axis can be processed without invalid numeric conversions.
 */
static bool movement_axis_input_valid(double angle, float speed, double steps_per_degree, int backlash_steps);

/**
 * @brief   Compute the altitude step interval for backlash movement.
 * @param   speed_dps Angular speed in degrees per second, clamped to at least 0.01.
 * @return  Step interval in microseconds.
 */
static int delay_us_from_speed_axis1(float speed_dps);

/**
 * @brief   Compute the azimuth step interval for backlash movement.
 * @param   speed_dps Angular speed in degrees per second, clamped to at least 0.01.
 * @return  Step interval in microseconds.
 */
static int delay_us_from_speed_axis2(float speed_dps);

/**
 * @brief   Run both axis moves with backlash and optional GoTo stop handling.
 * @param   angle1 Signed altitude motor angle in degrees.
 * @param   speed1 Altitude speed in degrees per second.
 * @param   angle2 Signed azimuth motor angle in degrees.
 * @param   speed2 Azimuth speed in degrees per second.
 * @param   goto_result GoTo result output, or NULL for ordinary movement without Stop GoTo.
 * @param   max_alt_steps Maximum altitude main pulses, or -1 for no position limit.
 * @param   reject_alt_limit Reject the entire move instead of limiting altitude pulses.
 * @return  False if rejected or interrupted by the App deadline; true after completion.
 */
static bool move_steppers_internal(double angle1, float speed1, double angle2, float speed2,
                                   telescope_goto_move_result_t *goto_result, int max_alt_steps, bool reject_alt_limit,
                                   bool *app_timed_out);

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   Signed altitude conversion remainder carried between moves.
 */
static double remaining_steps_alt;

/**
 * @brief   Signed azimuth conversion remainder carried between moves.
 */
static double remaining_steps_az;

/**
 * @brief   Altitude direction setting used by the last emitted move.
 */
static bool last_direction_config_alt;

/**
 * @brief   Azimuth direction setting used by the last emitted move.
 */
static bool last_direction_config_az;

/**
 * @brief   Critical-section lock for GoTo phase and stop-request coordination.
 */
static portMUX_TYPE goto_stop_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief   Published GoTo phase used to accept or ignore stop requests.
 */
static goto_motion_phase_t goto_motion_phase;

/**
 * @brief   Stop GoTo request latch shared with the BLE task.
 */
static atomic_bool goto_stop_requested;

// Application-task owned; no receive callback or heartbeat touches motor state.
static interrupted_backlash_t interrupted_backlash_alt, interrupted_backlash_az;

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Last emitted altitude motor direction sign; zero means unknown.
 */
int current_direction_alt = 0; //0 is invalid direction, since at startup we dont know yet which was last direction

/**
 * @brief   Last emitted azimuth motor direction sign; zero means unknown.
 */
int current_direction_az = 0;

/* Public Functions ------------------------------------------------------------------------------------------------- */

void telescope_request_goto_stop(void)
{
    goto_motion_phase_t phase;
    bool already_pending;
    portENTER_CRITICAL(&goto_stop_lock);
    phase = goto_motion_phase;
    already_pending = atomic_load_explicit(&goto_stop_requested, memory_order_relaxed);
    if ((phase == GOTO_MOTION_ACCELERATING || phase == GOTO_MOTION_CRUISING) && !already_pending)
    {
        atomic_store_explicit(&goto_stop_requested, true, memory_order_relaxed);
    }
    portEXIT_CRITICAL(&goto_stop_lock);

    // Logging stays outside the critical section and the motor pulse loop.
    if (phase == GOTO_MOTION_POSITION_UNRECOVERABLE)
    {
        TRACE_WAR("Stop GoTo ignored: two-star calibration prevents reliable stopped-position recovery");
    }
    else if (phase == GOTO_MOTION_FINAL_DECELERATION)
    {
        TRACE_INF("Stop GoTo ignored: final deceleration already in progress");
    }
    else if (phase == GOTO_MOTION_INACTIVE)
    {
        TRACE_DEB("Stop GoTo ignored: no active GoTo");
    }
    else if (phase == GOTO_MOTION_STOPPING || already_pending)
    {
        TRACE_DEB("Stop GoTo ignored: stop already pending or in progress");
    }
    else
    {
        TRACE_INF("Stop GoTo requested%s", phase == GOTO_MOTION_ACCELERATING ? "; waiting for acceleration" : "");
    }
}

void motor_init(void)
{
    // Define GPIO configuration structure
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENABLE_PIN_2) | (1ULL << STEP_PIN_2) | (1ULL << DIR_PIN_2) | (1ULL << ENABLE_PIN_1) |
                        (1ULL << MS1_PIN_1) | (1ULL << MS2_PIN_1) | (1ULL << STEP_PIN_1) | (1ULL << DIR_PIN_1),
        .mode = GPIO_MODE_OUTPUT,              // All pins set as outputs
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up resistors
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistors
        .intr_type = GPIO_INTR_POSEDGE         // GPIO_INTR_DISABLE No interrupts
    };

    // Configure all motor control pins
    gpio_config(&io_conf);

    // Set initial states
    gpio_set_level(ENABLE_PIN_1, 0);        // Enable motor 1, y axis (set to 1 to disable)
    gpio_set_level(MS1_PIN_1, 0);           // 64 microsteps (we have 2 drivers for 2 steppers,
                                            // both are connected to same MS1 and MS2 pins)
    gpio_set_level(MS2_PIN_1, 1);           // 64 microsteps
    gpio_set_level(STEP_PIN_1, 0);          // STEP starts low
    gpio_set_level(DIR_PIN_1, DIRECTION_1); // Default direction

    gpio_set_level(ENABLE_PIN_2, 0);        // Enable motor 2, x axis (set to 1 to disable)
    gpio_set_level(STEP_PIN_2, 0);          // STEP starts low
    gpio_set_level(DIR_PIN_2, DIRECTION_2); // Default direction
}

/**
 * @brief   Emit blocking backlash pulses on both axes at constant speeds.
 * @param   steps1 Altitude pulse count; its magnitude is used.
 * @param   speed1 Altitude speed in degrees per second.
 * @param   steps2 Azimuth pulse count; its magnitude is used.
 * @param   speed2 Azimuth speed in degrees per second.
 * @return  None.
 */
void move_steppers_backlash(int steps1, float speed1, int steps2, float speed2)
{
    (void) move_steppers_backlash_internal(steps1, speed1, steps2, speed2, false);
}

static backlash_move_result_t move_steppers_backlash_internal(int steps1, float speed1, int steps2, float speed2,
                                                              bool app_watchdog)
{
    if (steps1 < 0)
        steps1 = -steps1;
    if (steps2 < 0)
        steps2 = -steps2;
    backlash_move_result_t result = {0};
    if (steps1 == 0 && steps2 == 0)
        return result;

    double d1 = (steps1 > 0) ? (double) delay_us_from_speed_axis1(speed1) : 0.0;
    double d2 = (steps2 > 0) ? (double) delay_us_from_speed_axis2(speed2) : 0.0;

    if (d1 > 0 && d1 < STEP_PULSE_US)
        d1 = STEP_PULSE_US;
    if (d2 > 0 && d2 < STEP_PULSE_US)
        d2 = STEP_PULSE_US;

    int c1 = 0, c2 = 0;
    int stop1 = (steps1 == 0);
    int stop2 = (steps2 == 0);

    int64_t t1 = esp_timer_get_time();
    int64_t t2 = t1;
    int64_t p1 = 0, p2 = 0;

    while (1)
    {
        int64_t now = esp_timer_get_time();
        if (app_watchdog && (result.timed_out || mobile_app_joystick_watchdog_due(now)))
        {
            result.timed_out = true;
            stop1 = stop2 = 1;
            goto finish_backlash_pulses;
        }

        if (!stop1 && p1 == 0 && (now - t1) >= (int) d1)
        {
            if (c1 < steps1)
            {
                gpio_set_level(STEP_PIN_1, 1);
                p1 = now;
                t1 = now;
                c1++;
            }
            else
                stop1 = 1;
        }

        if (!stop2 && p2 == 0 && (now - t2) >= (int) d2)
        {
            if (c2 < steps2)
            {
                gpio_set_level(STEP_PIN_2, 1);
                p2 = now;
                t2 = now;
                c2++;
            }
            else
                stop2 = 1;
        }

    finish_backlash_pulses:
        if (p1 != 0 && (now - p1) >= STEP_PULSE_US)
        {
            gpio_set_level(STEP_PIN_1, 0);
            p1 = 0;
        }
        if (p2 != 0 && (now - p2) >= STEP_PULSE_US)
        {
            gpio_set_level(STEP_PIN_2, 0);
            p2 = 0;
        }

        if (stop1 && stop2 && (!result.timed_out || (p1 == 0 && p2 == 0)))
            break;
    }
    result.alt_pulses = c1;
    result.az_pulses = c2;
    return result;
}

void angle_to_steps_1(double angle, float speed, int *step_delay_us, int *steps)
{

    // Only exact zero means an inactive axis; retain every real sub-step request.
    if (angle == 0.0)
    {
        *steps = 0;
        global_count_steps_y = 0;
        *step_delay_us = 0;
        return;
    }

    const int sign = angle > 0.0 ? 1 : -1;

    // Calculate the delay between steps based on speed (in degrees per second)
    double steps_per_degree = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
    double steps_per_second = fabs(speed * steps_per_degree);
    if (steps_per_second <= 0)
    {
        TRACE_ERR("ALT step conversion failed: speed=%g, steps/s=%g", (double) speed, steps_per_second);
        return;
    }

    *step_delay_us = (int) (1e6 / steps_per_second); //convert to full int number in microseconds
    //(some information is lost here but it shouldnt be a problem since this is one time action not a continous like active tracking)
    // Calculate the total steps for the given angle

    *steps = (int) round(fabs(angle) * steps_per_degree);

    remaining_steps_alt = remaining_steps_alt + (angle * steps_per_degree) - (sign * (*steps));

    // The debt is signed, but the pulse count is a magnitude in this move's direction.
    if (remaining_steps_alt >= 1)
    {
        *steps += sign;
        remaining_steps_alt--;
    }
    else if (remaining_steps_alt <= -1)
    {
        *steps -= sign;
        remaining_steps_alt++;
    }

    if (*steps > 0)
    {
        gpio_set_level(DIR_PIN_1, sign > 0 ? !DIRECTION_1 : DIRECTION_1);
    }
    global_count_steps_y = *steps;
}

void angle_to_steps_2(double angle, float speed, int *step_delay_us, int *steps)
{

    // Only exact zero means an inactive axis; retain every real sub-step request.
    if (angle == 0.0)
    {
        *steps = 0;
        global_count_steps_x = 0;
        *step_delay_us = 0;
        return;
    }

    const int sign = angle > 0.0 ? 1 : -1;
    // Calculate the delay between steps based on speed (in degrees per second)
    double steps_per_degree = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0;
    double steps_per_second = fabs(speed * steps_per_degree);
    if (steps_per_second <= 0)
    {
        TRACE_ERR("AZ step conversion failed: speed=%g, steps/s=%g", (double) speed, steps_per_second);
        return;
    }

    *step_delay_us = (int) (1e6 / steps_per_second);

    // Calculate the total steps for the given angle

    *steps = (int) round(fabs(angle) * steps_per_degree);

    remaining_steps_az = remaining_steps_az + (angle * steps_per_degree) -
                         (sign * (*steps)); //since we are rounding steps its important to track remaining

    // The debt is signed, but the pulse count is a magnitude in this move's direction.
    if (remaining_steps_az >= 1)
    {
        *steps += sign;
        remaining_steps_az--;
    }
    else if (remaining_steps_az <= -1)
    {
        *steps -= sign;
        remaining_steps_az++;
    }

    if (*steps > 0)
    {
        gpio_set_level(DIR_PIN_2, sign > 0 ? !DIRECTION_2 : DIRECTION_2);
    }
    global_count_steps_x = *steps;
}

bool move_steppers(double angle1, float speed1, double angle2, float speed2)
{
    return move_steppers_internal(angle1, speed1, angle2, speed2, NULL, -1, false, NULL);
}

void move_steppers_manual(double angle1, float speed1, double angle2, float speed2)
{
    move_steppers_manual_internal(angle1, speed1, angle2, speed2, NULL);
}

bool move_steppers_app_manual(double angle1, float speed1, double angle2, float speed2)
{
    global_count_steps_y = 0;
    global_count_steps_x = 0;
    if (!mobile_app_joystick_motion_begin())
        return false;
    bool timed_out = false;
    move_steppers_manual_internal(angle1, speed1, angle2, speed2, &timed_out);
    mobile_app_joystick_motion_end(timed_out);
    return timed_out;
}

static void move_steppers_manual_internal(double angle1, float speed1, double angle2, float speed2, bool *app_timed_out)
{
    int max_alt_steps = -1;
    global_count_steps_y = 0;
    global_count_steps_x = 0;
    if (global_status_bar_screen_telescope && angle1 != 0.0)
    {
        max_alt_steps = manual_altitude_step_limit(angle1);
        if (max_alt_steps == 0)
        {
            // A blocked nudge must not change DIR, take up backlash or accumulate future travel.
            remaining_steps_alt = 0.0;
            angle1 = 0.0;
            // Skip ramp arithmetic for this inactive axis, including an invalid step scale.
            speed1 = 0.1f;
        }
    }
    (void) move_steppers_internal(angle1, speed1, angle2, speed2, NULL, max_alt_steps, false, app_timed_out);
}

bool move_steppers_skybox_altitude(double angle, float speed)
{
    int max_alt_steps = -1;
    global_count_steps_y = 0;
    global_count_steps_x = 0;
    if (global_status_bar_screen_telescope)
    {
        const double altitude = global_possition.alt;
        const double endpoint = altitude + angle;
        // Reject the whole pattern increment; never shorten it and continue the scan.
        if (!isfinite(altitude) || !isfinite(endpoint) || altitude < 0.0 || altitude > 90.0 || endpoint < 0.0 ||
            endpoint > 90.0)
        {
            return false;
        }
        max_alt_steps = manual_altitude_step_limit(angle);
    }
    return move_steppers_internal(angle, speed, 0.0, speed, NULL, max_alt_steps, max_alt_steps >= 0, NULL);
}

telescope_goto_move_result_t move_steppers_goto(double angle1, float speed1, double angle2, float speed2,
                                                bool position_recoverable)
{
    telescope_goto_move_result_t result = {.status = TELESCOPE_GOTO_MOVE_REJECTED};
    global_count_steps_y = 0;
    global_count_steps_x = 0;
    if (position_recoverable)
    {
        const double steps_per_degree_alt = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
        const double steps_per_degree_az = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0;
        if (fabs(angle1) > ALT_LIMIT_DEG || fabs(angle2) > AZ_LIMIT_DEG ||
            !goto_axis_input_valid(angle1, speed1, steps_per_degree_alt) ||
            !goto_axis_input_valid(angle2, speed2, steps_per_degree_az))
        {
            TRACE_WAR("GoTo movement rejected: invalid angle, speed or gear ratio");
            return result;
        }
    }

    const double previous_remaining_alt = remaining_steps_alt;
    const double previous_remaining_az = remaining_steps_az;
    portENTER_CRITICAL(&goto_stop_lock);
    atomic_store_explicit(&goto_stop_requested, false, memory_order_relaxed);
    goto_motion_phase = position_recoverable ? GOTO_MOTION_ACCELERATING : GOTO_MOTION_POSITION_UNRECOVERABLE;
    portEXIT_CRITICAL(&goto_stop_lock);

    // The scope includes backlash take-up. A stop received there waits for the
    // normal acceleration; backlash pulses retain their existing exclusion from travel.
    if (position_recoverable)
    {
        (void) move_steppers_internal(angle1, speed1, angle2, speed2, &result, -1, false, NULL);
    }
    else
    {
        // Preserve the entire legacy path when calibration only rules out stopping.
        // In particular, do not apply stop-specific preflight or phase/ramp logic.
        if (move_steppers_internal(angle1, speed1, angle2, speed2, NULL, -1, false, NULL))
        {
            result.status = TELESCOPE_GOTO_MOVE_COMPLETED;
        }
    }

    portENTER_CRITICAL(&goto_stop_lock);
    goto_motion_phase = GOTO_MOTION_INACTIVE;
    atomic_store_explicit(&goto_stop_requested, false, memory_order_relaxed);
    portEXIT_CRITICAL(&goto_stop_lock);
    if (result.status == TELESCOPE_GOTO_MOVE_INTERRUPTED)
    {
        // Position is now advanced by emitted pulses, so the abandoned target
        // must not leave its fractional conversion debt in the next movement.
        remaining_steps_alt = previous_remaining_alt;
        remaining_steps_az = previous_remaining_az;
    }
    return result;
}

void backlash_calibration(float backlash_angle_az, float backlash_angle_alt)
{

    backlash_angle_az = backlash_angle_az / 3600; //arcseconds into degrees
    backlash_angle_alt = backlash_angle_alt / 3600;

    float steps_per_degree_az = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0;
    global_backlash_steps_az = (int) round(fabs(backlash_angle_az) * steps_per_degree_az);

    float steps_per_degree_alt = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
    global_backlash_steps_alt = (int) round(fabs(backlash_angle_alt) * steps_per_degree_alt);

    TRACE_DEB("backlash_steps_az correction: %d", global_backlash_steps_az);
    TRACE_DEB("backlash_steps_alt correction: %d", global_backlash_steps_alt);

    (void) save_backlash_calibration_to_flash(&global_backlash_steps_az,
                                              &global_backlash_steps_alt); // Flash module reports persistence failures.
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static int manual_altitude_step_limit(double angle)
{
    int max_alt_steps = 0;
    const double scale = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
    const double altitude = global_possition.alt;
    if (isfinite(altitude) && isfinite(scale) && scale > 0.0)
    {
        const double room = angle > 0.0 ? 90.0 - altitude : altitude;
        const double available = floor(fmax(room, 0.0) * scale);
        max_alt_steps = available >= INT_MAX ? INT_MAX : (int) available;
        // Keep rounding in the caller's pulse-to-position calculation inside the limit too.
        const double endpoint = altitude + (angle > 0.0 ? 1.0 : -1.0) * (max_alt_steps / scale);
        if (max_alt_steps > 0 && (angle > 0.0 ? endpoint > 90.0 : endpoint < 0.0))
        {
            --max_alt_steps;
        }
    }
    return max_alt_steps;
}

static inline int steps_per_deg_axis1(void)
{
    return (int) ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0);
}
static inline int steps_per_deg_axis2(void)
{
    return (int) ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0);
}

static inline int accel_steps_needed(int steps_per_deg, float speed_dps)
{
    if (speed_dps <= 0.1f)
        return 1;

    // Time to accelerate to speed (s = v/a)
    double t_accel = speed_dps / ACCEL_DPS2; // seconds
    if (t_accel * 1000.0 < MIN_ACCEL_TIME_MS)
        t_accel = MIN_ACCEL_TIME_MS / 1000.0; // min ramp time

    // Average speed during acceleration = v/2
    double steps_during_accel = (speed_dps / 2.0) * steps_per_deg * t_accel;
    int n = (int) ceil(steps_during_accel);
    return n < 2 ? 2 : n;
}

// Keep full double precision internally; avoid 1 µs quantization issues
static inline double ramp_delta_us(double current_delay_us, double target_delay_us, int accel_steps)
{
    if (accel_steps < 1)
        accel_steps = 1;
    double diff = fabs(current_delay_us - target_delay_us);
    double delta = diff / (double) accel_steps;

    // Don’t clamp to ≥1 µs; we now use fractional µs precision
    return delta;
}

static goto_axis_phase_t goto_axis_phase(int steps, int completed, double delay, double target_delay, int ramp_steps)
{
    if (completed >= steps)
    {
        return GOTO_AXIS_FINISHED;
    }
    // Match the existing ramp's strict comparison and deceleration-first ordering.
    if ((steps - completed) < ramp_steps)
    {
        return GOTO_AXIS_FINAL_DECELERATION;
    }
    // The legacy subtraction can undershoot by a fractional microsecond. Let
    // its following constant-speed branch clamp the interval before stopping.
    if (ramp_steps > 0 && delay != target_delay)
    {
        return GOTO_AXIS_ACCELERATING;
    }
    return GOTO_AXIS_CRUISING;
}

static void publish_goto_phase(goto_motion_state_t *state, goto_motion_phase_t phase)
{
    if (state->published_phase != phase)
    {
        portENTER_CRITICAL(&goto_stop_lock);
        goto_motion_phase = phase;
        portEXIT_CRITICAL(&goto_stop_lock);
        state->published_phase = phase;
    }
}

static void shorten_goto_axis(int *steps, int completed, double *delay, int *ramp_steps, double *increment, float speed,
                              int steps_per_degree)
{
    if (*ramp_steps == 0)
    {
        // Acceleration may be disabled. Add only the requested stop's deceleration,
        // using the existing speed/ramp tuning and never moving beyond the target.
        *ramp_steps = accel_steps_needed(steps_per_degree, speed);
        if (*ramp_steps > (*steps - completed))
        {
            *ramp_steps = *steps - completed;
        }
        // The legacy low-speed path has no ramp. Its Stop GoTo endpoint slows to
        // half the current speed; ordinary low-speed movement remains unchanged.
        const double end_delay =
            speed > MIN_START_SPEED_DPS ? 1e6 / (MIN_START_SPEED_DPS * steps_per_degree) : *delay * 2.0;
        *increment = ramp_delta_us(*delay, end_delay, *ramp_steps + 1);
        // Slow the very next interval, including a request with only one pulse
        // left. Include the final post-pulse interval in the ramp span.
        *delay += *increment;
    }

    if (*ramp_steps < (*steps - completed))
    {
        *steps = completed + *ramp_steps;
    }
}

static bool goto_axis_input_valid(double angle, float speed, double steps_per_degree)
{
    if (!isfinite(angle) || !isfinite(speed) || speed <= 0.0f || !isfinite(steps_per_degree) ||
        steps_per_degree < 1.0 || steps_per_degree >= INT_MAX)
    {
        return false;
    }
    const double count = round(fabs(angle) * steps_per_degree);
    const double delay = 1e6 / (speed * steps_per_degree);
    const double stop_delay = speed > MIN_START_SPEED_DPS ? delay : delay * 2.0;
    const double accel_time = fmax(speed / ACCEL_DPS2, MIN_ACCEL_TIME_MS / 1000.0);
    const double ramp_steps = ceil((speed / 2.0) * (int) steps_per_degree * accel_time);
    // Leave room for the existing fractional correction before converting to int.
    return isfinite(count) && count < INT_MAX - 1 && isfinite(stop_delay) && stop_delay < INT_MAX &&
           isfinite(ramp_steps) && ramp_steps < INT_MAX - 1;
}

static bool movement_axis_input_valid(double angle, float speed, double steps_per_degree, int backlash_steps)
{
    if (!isfinite(angle))
    {
        return false;
    }
    if (angle != 0.0)
    {
        if (!isfinite(speed) || !isfinite(steps_per_degree) || steps_per_degree <= 0.0)
        {
            return false;
        }
        const double count = round(fabs(angle) * steps_per_degree);
        const double delay = 1e6 / fabs(speed * steps_per_degree);
        // Reserve one pulse for fractional carry; exact-zero axes need no conversion.
        if (!isfinite(count) || count >= INT_MAX - 1 || !isfinite(delay) || delay >= INT_MAX)
        {
            return false;
        }
    }
    const bool ramp = global_acceleration && speed > 1.0f;
    if (ramp || (angle != 0.0 && backlash_steps != 0))
    {
        // These existing paths use an integer step scale, including idle-axis ramps.
        if (!isfinite(steps_per_degree) || steps_per_degree < 1.0 || steps_per_degree >= INT_MAX)
        {
            return false;
        }
    }
    if (ramp)
    {
        const double accel_time = fmax(speed / ACCEL_DPS2, MIN_ACCEL_TIME_MS / 1000.0);
        const double ramp_steps = ceil((speed / 2.0) * (int) steps_per_degree * accel_time);
        if (!isfinite(ramp_steps) || ramp_steps >= INT_MAX - 1)
        {
            return false;
        }
    }
    return true;
}

static int delay_us_from_speed_axis1(float speed_dps)
{
    const float s = fmaxf(speed_dps, 0.01f);
    return (int) (1e6f / (s * steps_per_deg_axis1()));
}

static int delay_us_from_speed_axis2(float speed_dps)
{
    const float s = fmaxf(speed_dps, 0.01f);
    return (int) (1e6f / (s * steps_per_deg_axis2()));
}

static bool move_steppers_internal(double angle1, float speed1, double angle2, float speed2,
                                   telescope_goto_move_result_t *goto_result, int max_alt_steps, bool reject_alt_limit,
                                   bool *app_timed_out)
{
    global_count_steps_y = 0;
    global_count_steps_x = 0;
    // Safety check so no erratic angles are executed
    if ((fabs(angle1) > ALT_LIMIT_DEG) || (fabs(angle2) > AZ_LIMIT_DEG))
    {
        TRACE_WAR("Movement rejected: angle exceeds safety limit | AZ: %.2f deg | ALT: %.2f deg", angle2, angle1);
        return false;
    }
    const double scale_alt = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
    const double scale_az = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0;
    if (!movement_axis_input_valid(angle1, speed1, scale_alt,
                                   interrupted_backlash_alt.active ? 1 : global_backlash_steps_alt))
    {
        TRACE_WAR("Movement rejected: invalid ALT conversion (angle %g, speed %g, steps/deg %g)", angle1,
                  (double) speed1, scale_alt);
        return false;
    }
    if (!movement_axis_input_valid(angle2, speed2, scale_az,
                                   interrupted_backlash_az.active ? 1 : global_backlash_steps_az))
    {
        TRACE_WAR("Movement rejected: invalid AZ conversion (angle %g, speed %g, steps/deg %g)", angle2,
                  (double) speed2, scale_az);
        return false;
    }
    // --- direction & backlash ---
    //perform backlash compensation if direction changed
    const bool direction_config_alt = DIRECTION_1;
    const bool direction_config_az = DIRECTION_2;
    const int direction_alt = angle1 > 0.0 ? 1 : -1;
    const int direction_az = angle2 > 0.0 ? 1 : -1;
    // Compare physical direction even if calibration changed the DIR mapping.
    const int last_direction_alt =
        last_direction_config_alt != direction_config_alt ? -current_direction_alt : current_direction_alt;
    const int last_direction_az =
        last_direction_config_az != direction_config_az ? -current_direction_az : current_direction_az;

    // --- convert angles to step counts and base delays ---
    int steps1 = 0, steps2 = 0;
    int step_delay_us1_int = 0, step_delay_us2_int = 0;
    const double previous_remaining_alt = remaining_steps_alt;
    const double previous_remaining_az = remaining_steps_az;
    angle_to_steps_1(angle1, speed1, &step_delay_us1_int, &steps1);
    if (reject_alt_limit && steps1 > max_alt_steps)
    {
        // Rounding/carry may cross the boundary even when the requested angle fits.
        // No backlash or main pulses have been emitted; discard this conversion.
        remaining_steps_alt = previous_remaining_alt;
        global_count_steps_y = 0;
        return false;
    }
    angle_to_steps_2(angle2, speed2, &step_delay_us2_int, &steps2);
    if (!reject_alt_limit && max_alt_steps >= 0 && steps1 >= max_alt_steps)
    {
        // Limit the rounded count including fractional carry, before any backlash/main pulses.
        steps1 = max_alt_steps;
        remaining_steps_alt = 0.0;
    }
    const int original_steps1 = steps1;
    const int original_steps2 = steps2;

    int b1 = 0, b2 = 0;
    if (last_direction_alt != direction_alt)
        b1 = global_backlash_steps_alt;
    if (last_direction_az != direction_az)
        b2 = global_backlash_steps_az;

    const bool direction_level_alt = direction_alt > 0 ? !direction_config_alt : direction_config_alt;
    const bool direction_level_az = direction_az > 0 ? !direction_config_az : direction_config_az;
    if (interrupted_backlash_alt.active)
        b1 = direction_level_alt == interrupted_backlash_alt.target_level ? interrupted_backlash_alt.to_target
                                                                          : interrupted_backlash_alt.to_origin;
    if (interrupted_backlash_az.active)
        b2 = direction_level_az == interrupted_backlash_az.target_level ? interrupted_backlash_az.to_target
                                                                        : interrupted_backlash_az.to_origin;
    if (steps1 == 0)
        b1 = 0;
    if (steps2 == 0)
        b2 = 0;

    // Only App motion can stop in take-up. Retain contact accounting through
    // completed backlash too, until a main pulse confirms the reached endpoint.
    if (app_timed_out != NULL && b1 != 0 && !interrupted_backlash_alt.active)
    {
        interrupted_backlash_alt = (interrupted_backlash_t) {.active = true,
                                                             .target_level = direction_level_alt,
                                                             .span = abs(b1),
                                                             .to_target = abs(b1),
                                                             .to_origin = last_direction_alt == 0 ? abs(b1) : 0};
    }
    if (app_timed_out != NULL && b2 != 0 && !interrupted_backlash_az.active)
    {
        interrupted_backlash_az = (interrupted_backlash_t) {.active = true,
                                                            .target_level = direction_level_az,
                                                            .span = abs(b2),
                                                            .to_target = abs(b2),
                                                            .to_origin = last_direction_az == 0 ? abs(b2) : 0};
    }
    if ((b1 != 0) || (b2 != 0))
    {
        const backlash_move_result_t backlash =
            move_steppers_backlash_internal(b1, 3.0f, b2, 3.0f, app_timed_out != NULL);
        record_backlash_pulses(&interrupted_backlash_alt, direction_level_alt, backlash.alt_pulses);
        record_backlash_pulses(&interrupted_backlash_az, direction_level_az, backlash.az_pulses);
        if (backlash.timed_out)
        {
            *app_timed_out = true;
            if (original_steps1 != 0)
                remaining_steps_alt = previous_remaining_alt;
            if (original_steps2 != 0)
                remaining_steps_az = previous_remaining_az;
            global_count_steps_y = 0;
            global_count_steps_x = 0;
            return false;
        }
    }

    int step_count1 = 0, step_count2 = 0;
    int stop1 = 0, stop2 = 0;

    int64_t start_time1 = esp_timer_get_time();
    int64_t start_time2 = esp_timer_get_time();
    int64_t start_time1_2 = 0;
    int64_t start_time2_2 = 0;

    // --- acceleration setup ---
    double step_delay_us1 = (double) step_delay_us1_int;
    double step_delay_us2 = (double) step_delay_us2_int;
    double step_delay_us1_acc = step_delay_us1;
    double step_delay_us2_acc = step_delay_us2;
    int acceleration_delay1 = 0;
    int acceleration_delay2 = 0;
    double inc1 = 0.0, inc2 = 0.0;

    if (global_acceleration == 1)
    {
        // ---- ALT axis ----
        if (speed1 > 1.0f)
        {
            const int spd_per_deg_1 = steps_per_deg_axis1();
            acceleration_delay1 = accel_steps_needed(spd_per_deg_1, speed1);

            // start delay derived from low start speed
            double start_delay_us_1 = 1e6 / (fmax(MIN_START_SPEED_DPS, 0.1) * spd_per_deg_1);
            step_delay_us1 = start_delay_us_1;

            // compute fractional ramp increment
            inc1 = ramp_delta_us(step_delay_us1, step_delay_us1_acc, acceleration_delay1);
        }

        // ---- AZ axis ----
        if (speed2 > 1.0f)
        {
            const int spd_per_deg_2 = steps_per_deg_axis2();
            acceleration_delay2 = accel_steps_needed(spd_per_deg_2, speed2);

            double start_delay_us_2 = 1e6 / (fmax(MIN_START_SPEED_DPS, 0.1) * spd_per_deg_2);
            step_delay_us2 = start_delay_us_2;

            inc2 = ramp_delta_us(step_delay_us2, step_delay_us2_acc, acceleration_delay2);
        }
    }

    goto_motion_state_t goto_state = {.published_phase = GOTO_MOTION_ACCELERATING};
    bool goto_update_needed = true;
    int64_t now = esp_timer_get_time();
    while (1)
    {

        // Only an explicit GoTo observes its dedicated latch. The App deadline
        // is checked separately below; other movement keeps the original loop.
        if (goto_result != NULL &&
            (goto_update_needed || atomic_load_explicit(&goto_stop_requested, memory_order_relaxed)))
        {
            goto_update_needed = false;
            if (atomic_load_explicit(&goto_stop_requested, memory_order_relaxed))
            {
                portENTER_CRITICAL(&goto_stop_lock);
                goto_state.request_pending = true;
                atomic_store_explicit(&goto_stop_requested, false, memory_order_relaxed);
                goto_motion_phase = GOTO_MOTION_STOPPING;
                portEXIT_CRITICAL(&goto_stop_lock);
                goto_state.published_phase = GOTO_MOTION_STOPPING;
            }

            const goto_axis_phase_t alt_phase =
                goto_axis_phase(steps1, step_count1, step_delay_us1, step_delay_us1_acc, acceleration_delay1);
            const goto_axis_phase_t az_phase =
                goto_axis_phase(steps2, step_count2, step_delay_us2, step_delay_us2_acc, acceleration_delay2);
            if (goto_state.request_pending)
            {
                // Axes can be in different phases. A short/finished axis never
                // prevents the other axis from stopping after its own acceleration.
                if (!goto_state.alt_handled && alt_phase != GOTO_AXIS_ACCELERATING)
                {
                    goto_state.alt_handled = true;
                    if (alt_phase == GOTO_AXIS_CRUISING)
                    {
                        shorten_goto_axis(&steps1, step_count1, &step_delay_us1, &acceleration_delay1, &inc1, speed1,
                                          steps_per_deg_axis1());
                        goto_state.stop_started = true;
                    }
                }
                if (!goto_state.az_handled && az_phase != GOTO_AXIS_ACCELERATING)
                {
                    goto_state.az_handled = true;
                    if (az_phase == GOTO_AXIS_CRUISING)
                    {
                        shorten_goto_axis(&steps2, step_count2, &step_delay_us2, &acceleration_delay2, &inc2, speed2,
                                          steps_per_deg_axis2());
                        goto_state.stop_started = true;
                    }
                }
                if (goto_state.alt_handled && goto_state.az_handled && !goto_state.stop_started)
                {
                    // Short ramps may enter final deceleration without cruising.
                    goto_state.request_pending = false;
                    goto_state.ignored_final_deceleration = true;
                }
            }

            goto_motion_phase_t phase = GOTO_MOTION_FINAL_DECELERATION;
            if (goto_state.request_pending)
            {
                phase = GOTO_MOTION_STOPPING;
            }
            else if (alt_phase == GOTO_AXIS_ACCELERATING || az_phase == GOTO_AXIS_ACCELERATING)
            {
                phase = GOTO_MOTION_ACCELERATING;
            }
            else if (alt_phase == GOTO_AXIS_CRUISING || az_phase == GOTO_AXIS_CRUISING)
            {
                phase = GOTO_MOTION_CRUISING;
            }
            publish_goto_phase(&goto_state, phase);
        }

        now = esp_timer_get_time();
        if (app_timed_out != NULL && (*app_timed_out || mobile_app_joystick_watchdog_due(now)))
        {
            *app_timed_out = true;
            stop1 = stop2 = 1;
            goto finish_main_pulses;
        }
        // ===== ALT axis =====
        if (start_time1_2 == 0 && (now - start_time1) >= (int) step_delay_us1)
        {
            if (step_count1 < steps1)
            {
                gpio_set_level(STEP_PIN_1, 1);
                start_time1_2 = now;
                start_time1 = now;
                step_count1++;
                goto_update_needed = true;

                if ((steps1 - step_count1) < acceleration_delay1)
                {
                    step_delay_us1 += inc1; // decel
                }
                else if (step_delay_us1 > step_delay_us1_acc)
                {
                    step_delay_us1 -= inc1; // accel
                }
                else
                {
                    step_delay_us1 = step_delay_us1_acc; // constant
                }
            }
            else
                stop1 = 1;
        }

        // ===== AZ axis =====
        if (start_time2_2 == 0 && (now - start_time2) >= (int) step_delay_us2)
        {
            if (step_count2 < steps2)
            {
                gpio_set_level(STEP_PIN_2, 1);
                start_time2_2 = now;
                start_time2 = now;
                step_count2++;
                goto_update_needed = true;

                if ((steps2 - step_count2) < acceleration_delay2)
                {
                    step_delay_us2 += inc2;
                }
                else if (step_delay_us2 > step_delay_us2_acc)
                {
                    step_delay_us2 -= inc2;
                }
                else
                {
                    step_delay_us2 = step_delay_us2_acc;
                }
            }
            else
                stop2 = 1;
        }

    finish_main_pulses:
        // --- finish pulses ---
        if (start_time1_2 != 0 && (now - start_time1_2) >= STEP_PULSE_US)
        {
            gpio_set_level(STEP_PIN_1, 0);
            start_time1_2 = 0;
        }
        if (start_time2_2 != 0 && (now - start_time2_2) >= STEP_PULSE_US)
        {
            gpio_set_level(STEP_PIN_2, 0);
            start_time2_2 = 0;
        }

        // --- exit condition ---
        if (stop1 && stop2 && (app_timed_out == NULL || !*app_timed_out || (start_time1_2 == 0 && start_time2_2 == 0)))
        {
            if (app_timed_out != NULL && *app_timed_out)
            {
                // Keep debt for completed axes; discard the unexecuted part of
                // an interrupted nudge without replaying it on a later press.
                if (step_count1 < original_steps1)
                    remaining_steps_alt = previous_remaining_alt;
                if (step_count2 < original_steps2)
                    remaining_steps_az = previous_remaining_az;
            }
            // Zero-pulse requests must not change the last physical direction.
            if (step_count1 > 0)
            {
                current_direction_alt = direction_alt;
                last_direction_config_alt = direction_config_alt;
                interrupted_backlash_alt.active = false;
            }
            if (step_count2 > 0)
            {
                current_direction_az = direction_az;
                last_direction_config_az = direction_config_az;
                interrupted_backlash_az.active = false;
            }
            global_count_steps_y = step_count1;
            global_count_steps_x = step_count2;
            if (goto_result != NULL)
            {
                goto_result->status = (step_count1 < original_steps1 || step_count2 < original_steps2)
                                          ? TELESCOPE_GOTO_MOVE_INTERRUPTED
                                          : TELESCOPE_GOTO_MOVE_COMPLETED;
                goto_result->alt_degrees =
                    copysign(step_count1 / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0), angle1);
                goto_result->az_degrees =
                    copysign(step_count2 / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0), angle2);
            }
            step_count1 = step_count2 = 0;
            stop1 = stop2 = 0;
            break;
        }
    }
    if (goto_state.ignored_final_deceleration)
    {
        TRACE_INF("Stop GoTo ignored: final deceleration reached before a constant-speed phase");
    }
    return app_timed_out == NULL || !*app_timed_out;
}

static void record_backlash_pulses(interrupted_backlash_t *state, bool direction_level, int pulses)
{
    if (!state->active || pulses == 0)
        return;
    if (direction_level == state->target_level)
    {
        state->to_target -= pulses;
        state->to_origin += pulses;
        if (state->to_origin > state->span)
            state->to_origin = state->span;
    }
    else
    {
        state->to_target += pulses;
        if (state->to_target > state->span)
            state->to_target = state->span;
        state->to_origin -= pulses;
    }
}
