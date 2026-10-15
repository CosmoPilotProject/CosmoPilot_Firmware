/**
 * @file    main.c
 * @brief   Firmware entry point and main application loop.
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

#include "main_helpers.h"
#include "display/display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for the firmware entry point and main loop.
 */
#define TRACE_NAME MAIN

/**
 * @brief   Trace verbosity for the firmware entry point and main loop.
 */
#define TRACE_LEVEL MAIN_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Main application loop.
 * @return  None.
 */
void app_main(void)
{
    TRACE_INF("Main application loop started!");

    initialize_application();

    main_loop_state_t loop_state = initialize_main_loop();
    run_startup_mode(loop_state.standalone_enabled_at_startup);

    while (1)
    {
        process_pending_command(&loop_state);
        refresh_idle_display(&loop_state);
        send_periodic_notifications(&loop_state);

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
    }

    TRACE_FAT("Main application loop ended unexpectedly");
    update_display_ERROR_screen(ERROR_UNEXPECTED_TERMINATION);
}

/* Private Functions ------------------------------------------------------------------------------------------------ */
