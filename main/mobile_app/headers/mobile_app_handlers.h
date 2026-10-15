/**
 * @file    mobile_app_handlers.h
 * @brief   Mobile-app command handler declarations.
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

#ifndef MOBILE_APP_HANDLERS_H
#define MOBILE_APP_HANDLERS_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "mobile_app_parser.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Apply the requested time.
 * @param   cmd Time command without the "CMD " prefix.
 * @return  None.
 */
void process_time_command(const char *cmd);

/**
 * @brief   Apply the requested calendar date.
 * @param   cmd Date command without the "CMD " prefix.
 * @return  None.
 */
void process_date_command(const char *cmd);

/**
 * @brief   Apply and save the observer latitude and longitude.
 * @param   cmd Location command without the "CMD " prefix.
 * @return  None.
 */
void process_location_command(const char *cmd);

/**
 * @brief   Apply and save the selected telescope settings.
 * @param   cmd Settings command without the "CMD " prefix.
 * @return  None.
 */
void process_other_settings_command(const char *cmd);

/**
 * @brief   Apply and save valid axis gear ratios.
 * @param   cmd Gear-ratio command without the "CMD " prefix.
 * @return  None.
 */
void process_set_gear_ratio_command(const char *cmd);

/**
 * @brief   Apply backlash compensation from whole-arcsecond values.
 * @param   cmd Backlash command without the "CMD " prefix.
 * @return  None.
 */
void process_backlash_calib_command(const char *cmd);

/**
 * @brief   Invalidate alignment before calibration centering and send READY.
 * @param   cmd Exact calibration-start command without the "CMD " prefix.
 * @return  None.
 */
void process_calib_start_command(const char *cmd);

/**
 * @brief   Apply one-star calibration from equatorial coordinates.
 * @param   cmd Calibration command without the "CMD " prefix.
 * @return  None.
 */
void process_1_star_calib_command(const char *cmd);

/**
 * @brief   Run two-star calibration from the command coordinates.
 * @param   cmd Calibration command without the "CMD " prefix.
 * @return  None.
 */
void process_2_star_calib_command(const char *cmd);

/**
 * @brief   Run analog joystick calibration for an IDLE request.
 * @param   cmd Joystick calibration command without the "CMD " prefix.
 * @return  None.
 */
void process_joystick_calib_command(const char *cmd);

/**
 * @brief   Run motor direction calibration for a START request.
 * @param   cmd Direction setup command without the "CMD " prefix.
 * @return  None.
 */
void process_direction_setup_command(const char *cmd);

/**
 * @brief   Handle a GoTo request for equatorial coordinates.
 * @param   cmd GoTo command without the "CMD " prefix.
 * @return  None.
 */
void process_goto_any_object_command(const char *cmd);

/**
 * @brief   Handle a GoTo request for a solar-system object.
 * @param   cmd Solar-system GoTo command without the "CMD " prefix.
 * @return  None.
 */
void process_goto_solar_system_command(const char *cmd);

/**
 * @brief   Run analog joystick manual control.
 * @param   cmd Unused command argument.
 * @return  None.
 */
void process_analog_joystick_command(const char *cmd);

/**
 * @brief   Run mobile-app joystick control for a START request.
 * @param   cmd App joystick command without the "CMD " prefix.
 * @return  None.
 */
void process_app_joystick_command(const char *cmd);

/**
 * @brief   Run standalone joystick control.
 * @return  None.
 */
void process_standalone_command(void);

/**
 * @brief   Run skybox assistance with the configured settings.
 * @return  None.
 */
void process_sky_box_assist_command(void);

/**
 * @brief   Release both steppers until a command with the "CMD " prefix arrives.
 * @param   cmd Unused command argument.
 * @return  None.
 */
void process_release_steppers_command(const char *cmd);

/**
 * @brief   Send the current telescope settings to the mobile app.
 * @param   cmd Unused command argument.
 * @return  None.
 */
void process_show_settings_command(const char *cmd);
/**
 * @brief   Send solar-system object visibility to the mobile app.
 * @param   cmd Unused command argument.
 * @return  None.
 */
void process_solar_obj_status_command(const char *cmd);

/**
 * @brief   Erase stored settings and apply the runtime reset values.
 * @param   cmd Unused command argument.
 * @return  None.
 */
void process_factory_reset_command(const char *cmd);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // MOBILE_APP_HANDLERS_H
