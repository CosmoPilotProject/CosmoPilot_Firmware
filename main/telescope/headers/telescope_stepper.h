/**
 * @file    telescope_stepper.h
 * @brief   Stepper motion, GoTo stop results, and backlash calibration interface.
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

#ifndef TELESCOPE_CONTROL_H
#define TELESCOPE_CONTROL_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "flash/flash.h"
#include <stdbool.h>

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Reported outcome of a GoTo motor move.
 */
typedef enum
{
    TELESCOPE_GOTO_MOVE_COMPLETED,   /**< GoTo move reported completion. */
    TELESCOPE_GOTO_MOVE_INTERRUPTED, /**< Main travel ended before the original step target. */
    TELESCOPE_GOTO_MOVE_REJECTED     /**< GoTo movement was rejected. */
} telescope_goto_move_status_t;

/**
 * @brief   GoTo status and pulse-based travel, excluding backlash.
 */
typedef struct
{
    telescope_goto_move_status_t status; /**< Reported GoTo outcome. */
    double alt_degrees;                  /**< Signed altitude travel in degrees; zero without position recovery. */
    double az_degrees;                   /**< Signed azimuth motor travel in degrees; zero without position recovery. */
} telescope_goto_move_result_t;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Configure stepper GPIOs and their initial driver levels.
 * @return  None.
 */
void motor_init(void);

/**
 * @brief   Prepare altitude direction, step count, and timing for move_steppers.
 * @param   angle Signed altitude angle in degrees; exact zero keeps the axis inactive.
 * @param   speed Angular speed in degrees per second.
 * @param   step_delay_us Output step interval in microseconds.
 * @param   steps Output nonnegative pulse count, including accumulated fractional requests.
 * @return  None.
 */
void angle_to_steps_1(double angle, float speed, int *step_delay_us, int *steps);

/**
 * @brief   Prepare azimuth direction, step count, and timing for move_steppers.
 * @param   angle Signed azimuth motor angle in degrees; exact zero keeps the axis inactive.
 * @param   speed Angular speed in degrees per second.
 * @param   step_delay_us Output step interval in microseconds.
 * @param   steps Output nonnegative pulse count, including accumulated fractional requests.
 * @return  None.
 */
void angle_to_steps_2(double angle, float speed, int *step_delay_us, int *steps);

/**
 * @brief   Run a blocking two-axis move without Stop GoTo handling.
 * Accepted sub-step requests retain fractional carry even when no pulse is due.
 * @param   angle1 Signed altitude motor angle in degrees.
 * @param   speed1 Altitude speed in degrees per second.
 * @param   angle2 Signed azimuth motor angle in degrees.
 * @param   speed2 Azimuth speed in degrees per second.
 * @return  True if accepted; false if rejected before either axis moves.
 */
bool move_steppers(double angle1, float speed1, double angle2, float speed2);

/**
 * @brief   Run a manual move, limiting altitude pulses to [0, 90] degrees only while aligned.
 * @param   angle1 Signed altitude motor angle in degrees.
 * @param   speed1 Altitude speed in degrees per second.
 * @param   angle2 Signed azimuth motor angle in degrees.
 * @param   speed2 Azimuth speed in degrees per second.
 * @return  None.
 */
void move_steppers_manual(double angle1, float speed1, double angle2, float speed2);

/**
 * @brief   Run one App Joystick nudge with its owned keep-alive deadline.
 *
 * Preserves manual limits and nominal pulse timing. Expiry prevents new pulses
 * during main travel and backlash, finishes any high pulse, and records actual
 * travel plus unfinished backlash for the next explicit movement.
 * @return  True only when the watchdog interrupted this move; otherwise false.
 */
bool move_steppers_app_manual(double angle1, float speed1, double angle2, float speed2);

/**
 * @brief   Run one SkyBox altitude increment, rejecting aligned boundary crossings before pulses.
 * Uses the manual altitude pulse budget, including rounding and fractional carry.
 * Unaligned movement remains unrestricted. The caller accounts for emitted steps.
 * @param   angle Signed altitude motor angle in degrees.
 * @param   speed Altitude speed in degrees per second.
 * @return  True if accepted; false if rejected with zero emitted steps.
 */
bool move_steppers_skybox_altitude(double angle, float speed);

/**
 * @brief   Run a blocking GoTo move with Stop GoTo support when position is recoverable.
 * Only this entry point accepts Stop GoTo requests.
 * @param   angle1 Signed altitude motor angle in degrees.
 * @param   speed1 Altitude speed in degrees per second.
 * @param   angle2 Signed azimuth motor angle in degrees.
 * @param   speed2 Azimuth speed in degrees per second.
 * @param   position_recoverable Whether the caller can recover a stopped position.
 *                               False keeps legacy movement and traces ignored stops.
 * @return  Move status and signed travel in degrees, excluding backlash; travel is zero when recovery is disabled.
 */
telescope_goto_move_result_t move_steppers_goto(double angle1, float speed1, double angle2, float speed2,
                                                bool position_recoverable);

/**
 * @brief   Latch an eligible Stop GoTo request without motor or position changes.
 * Safe to call from the BLE task.
 * @return  None.
 */
void telescope_request_goto_stop(void);

/**
 * @brief   Convert backlash angles to motor steps and save the compensation.
 * @param   backlash_angle_az Azimuth backlash in arcseconds.
 * @param   backlash_angle_alt Altitude backlash in arcseconds.
 * @return  None.
 */
void backlash_calibration(float backlash_angle_az, float backlash_angle_alt);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif
