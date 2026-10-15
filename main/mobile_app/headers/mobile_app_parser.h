/**
 * @file    mobile_app_parser.h
 * @brief   Internal command IDs and keyword parsing interface.
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

#ifndef MOBILE_APP_PARSER_H
#define MOBILE_APP_PARSER_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stddef.h>

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Internal identifiers used to dispatch mobile-app commands.
 */
typedef enum
{
    NONE, /**< Unrecognized or missing command keyword. */

    //GOTO MENU
    GOTO_ANY_OBJECT,   /**< General object GoTo command. */
    GOTO_SOLAR_SYSTEM, /**< Solar-system object GoTo command. */

    //CALIBRATION
    STAR_1_CALIB,   /**< One-star calibration command. */
    STAR_2_CALIB,   /**< Two-star calibration command. */
    STAR_3_CALIB,   /**< Three-star calibration command. */
    BACKLASH_CALIB, /**< Backlash calibration command. */

    //PLAY GAME
    PLAY_GAME, /**< Game command. */

    //FREE MODE
    ANALOG_JOYSTICK,  /**< Analog joystick command. */
    STANDALONE_MODE,  /**< Standalone control mode identifier. */
    RELEASE_STEPPERS, /**< Stepper release command. */
    APP_JOYSTICK,     /**< App joystick command. */

    //CONFIGURATION
    BLUETOOTH,      /**< Bluetooth configuration command. */
    SET_TIME,       /**< Time configuration command. */
    SET_DATE,       /**< Date configuration command. */
    SET_LOCATION,   /**< Observing location configuration command. */
    OTHER_SETTINGS, /**< Additional settings command. */

    //TELESCOPE SETUP
    SET_GEAR_RATIO,  /**< Gear ratio configuration command. */
    JOSTICK_CALIB,   /**< Joystick calibration command. */
    DIRECTION_SETUP, /**< Motor direction setup command. */
    SHOW_SETTINGS,   /**< Settings reporting command. */
    FACTORY_RESET,   /**< Factory reset command. */

    //SPECIALITIES
    SKY_BOX_ASSIST, /**< Sky-box assistance command. */

    APP_J, /**< App joystick entry alias. */       //to enter app joystick mode  -> bug fix
    ANALOG_J, /**< Analog joystick entry alias. */ //to enter analog joystick mode  -> bug fix

    SEND_SOLAR_OBJ_STATUS, /**< Solar-system object status command. */

    VOID_COMMAND, /**< Explicit NONE command. */
    CALIB_START   /**< Begin calibration before manual first-star centering. */

} CommandID;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Extract a keyword from a full CMD message without modifying the message.
 * @param   message Full command message to read.
 * @param   keyword Output buffer, cleared on failure when non-NULL and capacity is nonzero.
 * @param   capacity Output buffer size, including the terminating null byte.
 * @return  True when a keyword is extracted; otherwise false.
 */
bool mobile_app_command_keyword(const char *message, char *keyword, size_t capacity);

/**
 * @brief   Look up the internal identifier for a command keyword.
 * @param   keyword Case-sensitive keyword, or NULL.
 * @return  Matching command identifier, or NONE for an unknown or NULL keyword.
 */
CommandID mobile_app_parse_command_id(const char *keyword);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // MOBILE_APP_PARSER_H
