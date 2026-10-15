/**
 * @file    mobile_app_dispatch.c
 * @brief   Dispatch mobile-app commands to application handlers.
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

#include "headers/mobile_app_dispatch.h"
#include "headers/mobile_app_commands.h"
#include "headers/mobile_app_command_buffer.h"
#include "headers/mobile_app_handlers.h"
#include "telescope/headers/telescope_common.h"
#include "display/display.h"
#include <string.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for mobile-app command dispatch.
 */
#define TRACE_NAME MOBILE_APP_DISPATCH

/**
 * @brief   Trace verbosity for mobile-app command dispatch.
 */
#define TRACE_LEVEL MOBILE_APP_DISPATCH_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void mobile_app_dispatch_command(void)
{
    // Extract only the command part before the space (e.g., "SET_TIME")
    char command_name[MAX_COMMAND_LENGTH] = {0};
    // Empty/invalid keywords keep the existing unknown-command path.
    (void) mobile_app_command_keyword(global_last_ble_command, command_name, sizeof(command_name));

    // Now parse only the actual command keyword
    CommandID cmd = parse_command_id(command_name);
    TRACE_INF("Processing command: %s", command_name);

    process_command_id(cmd);
}

void process_command_id(CommandID cmd)
{
    TRACE_DEB("Dispatching command ID: %d", (int) cmd);

    // Nested movement/calibration loops may refresh the consumer view. Keep this
    // command's parameters stable for the entire handler call.
    char command_text[MAX_COMMAND_LENGTH];
    memcpy(command_text, global_last_ble_command, sizeof(command_text));

    switch (cmd)
    {
        case NONE:
            send_ready_status(3);
            break;

        // GOTO MENU
        case GOTO_ANY_OBJECT:
            process_goto_any_object_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case GOTO_SOLAR_SYSTEM:
            process_goto_solar_system_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        // CALIBRATION
        case CALIB_START:
            process_calib_start_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case STAR_1_CALIB:
            process_1_star_calib_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case STAR_2_CALIB:
            process_2_star_calib_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case STAR_3_CALIB:
            break;
        case BACKLASH_CALIB:
            process_backlash_calib_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        // PLAY GAME
        case PLAY_GAME:
            break;

        // FREE MODE
        case ANALOG_JOYSTICK:
            process_analog_joystick_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case STANDALONE_MODE:
            process_standalone_command();
            break;
        case APP_JOYSTICK:
            process_app_joystick_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case RELEASE_STEPPERS:
            process_release_steppers_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case SKY_BOX_ASSIST:
            process_sky_box_assist_command();
            break;

        // CONFIGURATION
        case BLUETOOTH:
            break;
        case SET_TIME:
            process_time_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case SET_DATE:
            process_date_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case SET_LOCATION:
            process_location_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case OTHER_SETTINGS:
            process_other_settings_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        // TELESCOPE SETUP
        case SET_GEAR_RATIO:
            process_set_gear_ratio_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case JOSTICK_CALIB:
            process_joystick_calib_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case DIRECTION_SETUP:
            process_direction_setup_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case SHOW_SETTINGS:
            process_show_settings_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;
        case FACTORY_RESET:
            process_factory_reset_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        case APP_J:
            process_app_joystick_command(APP_TRIMMED_APP_JOYSTICK_START);
            break;
        case ANALOG_J:
            process_analog_joystick_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        case SEND_SOLAR_OBJ_STATUS:
            process_solar_obj_status_command(&command_text[APP_COMMAND_PREFIX_LENGTH]);
            break;

        case VOID_COMMAND:
            mobile_app_command_clear();
            break;

        default:
            // Unknown command
            TRACE_ERR("Unknown command ID: %d", (int) cmd);
            update_display_ERROR_screen(ERROR_UNKNOWN_COMMAND_ID);
            break;
    }
}

CommandID parse_command_id(const char *str)
{
    CommandID id = mobile_app_parse_command_id(str);
    if (id != NONE)
    {
        return id;
    }

    TRACE_WAR("Invalid command: %s", str);
    // Invalid external input is recoverable; NONE retains the existing READY path.
    return NONE; // unknown or invalid command
}

/* Private Functions ------------------------------------------------------------------------------------------------ */
