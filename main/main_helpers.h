/**
 * @file    main_helpers.h
 * @brief   Application initialization, main-loop state, and service interface.
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

#ifndef MAIN_HELPERS_H
#define MAIN_HELPERS_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Main application-loop delay configuration.
 */
enum
{
    MAIN_LOOP_DELAY_MS = 100 /**< Requested delay per loop iteration, in milliseconds. */
};

/**
 * @brief   Startup mode snapshot and timing state owned by the main application loop.
 */
typedef struct
{
    bool standalone_enabled_at_startup; /**< Startup snapshot of standalone mode. */
    int64_t display_reset_started_us;   /**< Post-command timer start in microseconds; zero means inactive. */
    int64_t last_ready_sent_us;         /**< Timer reference for periodic READY notifications, in microseconds. */
    int64_t last_status_sent_us;        /**< Timer reference for periodic status notifications, in microseconds. */
} main_loop_state_t;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Run application startup and its display animation in the main application task.
 *
 * Returned startup errors are logged without preventing the animation.
 * @return  None.
 */
void initialize_application(void);

/**
 * @brief   Capture the startup mode and initialize main-loop notification timing.
 * @return  Initial loop state with an inactive display timer and current notification timestamps.
 */
main_loop_state_t initialize_main_loop(void);

/**
 * @brief   Enter standalone control when enabled by the startup snapshot.
 * @param   standalone_enabled Whether to run standalone control.
 * @return  None.
 */
void run_startup_mode(bool standalone_enabled);

/**
 * @brief   Handle the pending command snapshot and update application status.
 * @param   state Initialized main-loop state; must not be NULL.
 * @return  None.
 */
void process_pending_command(main_loop_state_t *state);

/**
 * @brief   Refresh the display once its post-command delay has elapsed.
 * @param   state Initialized main-loop state; must not be NULL.
 * @return  None.
 */
void refresh_idle_display(main_loop_state_t *state);

/**
 * @brief   Send periodic READY and status notifications when due.
 * @param   state Initialized main-loop state; must not be NULL.
 * @return  None.
 */
void send_periodic_notifications(main_loop_state_t *state);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // MAIN_HELPERS_H
