/**
 * @file    mobile_app_commands.h
 * @brief   Mobile-app command vocabulary and wire protocol constants.
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

#ifndef MOBILE_APP_COMMANDS_H
#define MOBILE_APP_COMMANDS_H

/* Includes --------------------------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

// Shared framing and command keywords. Keywords do NOT include "CMD " or a separator.
#define APP_COMMAND_PREFIX        "CMD "
#define APP_CMD_GOTO_ANY_OBJECT   "GOTO_ANY_OBJECT"
#define APP_CMD_GOTO_SOLAR_SYSTEM "GOTO_SOLAR_SYSTEM"
#define APP_CMD_STAR_1_CALIB      "STAR_1_CALIB"
#define APP_CMD_STAR_2_CALIB      "STAR_2_CALIB"
#define APP_CMD_STAR_3_CALIB      "STAR_3_CALIB"
#define APP_CMD_CALIB_START       "CALIB_START"
#define APP_CMD_BACKLASH_CALIB    "BACKLASH_CALIB"
#define APP_CMD_PLAY_GAME         "PLAY_GAME"
#define APP_CMD_ANALOG_JOYSTICK   "ANALOG_JOYSTICK"
#define APP_CMD_APP_JOYSTICK      "APP_JOYSTICK"
#define APP_CMD_SKY_BOX_ASSIST    "SKY_BOX_ASSIST"
#define APP_CMD_RELEASE_STEPPERS  "RELEASE_STEPPERS"
#define APP_CMD_BLUETOOTH         "BLUETOOTH"
#define APP_CMD_SET_TIME          "SET_TIME"
#define APP_CMD_SET_DATE          "SET_DATE"
#define APP_CMD_SET_LOCATION      "SET_LOCATION"
#define APP_CMD_OTHER_SETTINGS    "OTHER_SETTINGS"
#define APP_CMD_SET_GEAR_RATIO    "SET_GEAR_RATIO"
#define APP_CMD_JOYSTICK_CALIB    "JOYSTICK_CALIB"
#define APP_CMD_DIRECTION_SETUP   "DIRECTION_SETUP"
#define APP_CMD_SHOW_SETTINGS     "SHOW_SETTINGS"
#define APP_CMD_FACTORY_RESET     "FACTORY_RESET"
#define APP_CMD_APP               "APP"
#define APP_CMD_ANALOG            "ANALOG"
#define APP_CMD_SOLAR_OBJ_STATUS  "SOLAR_OBJ_STATUS"
#define APP_CMD_NONE              "NONE"

// Subcommands and symbolic arguments, not separate top-level dispatch entries.
#define APP_ARG_START               "START"
#define APP_ARG_STOP                "STOP"
#define APP_ARG_IDLE                "IDLE"
#define APP_ARG_MAX                 "MAX"
#define APP_ARG_NEXT                "NEXT"
#define APP_ARG_FINISH_2_STAR_CALIB "FINISH_2_STAR_CALIB"
#define APP_ARG_DIR_CALIB_RESULT    "DIR_CALIB_RESULT"
#define APP_ARG_DIR                 "DIR"
#define APP_ARG_UP                  "UP"
#define APP_ARG_DOWN                "DOWN"
#define APP_ARG_LEFT                "LEFT"
#define APP_ARG_RIGHT               "RIGHT"
#define APP_ARG_ULEFT               "ULEFT"
#define APP_ARG_URIGHT              "URIGHT"
#define APP_ARG_DLEFT               "DLEFT"
#define APP_ARG_DRIGHT              "DRIGHT"
#define APP_ARG_SPEED               "SPEED"
#define APP_ARG_TOGGLE              "TOGGLE"
#define APP_ARG_TRACKING            "TRACKING"

// Complete raw receive strings, before stripping "CMD ". Trailing spaces are significant.
#define APP_RX_STOP_GOTO                   APP_COMMAND_PREFIX "STOP GOTO"
#define APP_RX_APP_KEEPALIVE               APP_COMMAND_PREFIX APP_CMD_APP " KEEPALIVE"
#define APP_RX_CALIB_START                 APP_COMMAND_PREFIX APP_CMD_CALIB_START
#define APP_TRIMMED_APP_JOYSTICK_START     APP_CMD_APP_JOYSTICK " " APP_ARG_START
#define APP_RX_ANALOG_SPEED                APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_ARG_SPEED
#define APP_RX_ANALOG_START_TRACKING       APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_ARG_START " " APP_ARG_TRACKING
#define APP_RX_NONE                        APP_COMMAND_PREFIX APP_CMD_NONE
#define APP_RX_ANALOG_TOGGLE               APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_ARG_TOGGLE
#define APP_RX_ANALOG_SKY_BOX_ASSIST_STOP  APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_CMD_SKY_BOX_ASSIST " " APP_ARG_STOP
#define APP_RX_ANALOG_SKY_BOX_ASSIST_START APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_CMD_SKY_BOX_ASSIST " " APP_ARG_START
#define APP_RX_ANALOG_JOYSTICK             APP_COMMAND_PREFIX APP_CMD_ANALOG_JOYSTICK
#define APP_RX_ANALOG_STOP_TRACKING        APP_COMMAND_PREFIX APP_CMD_ANALOG " " APP_ARG_STOP " " APP_ARG_TRACKING
#define APP_RX_JOYSTICK_CALIB_MAX          APP_COMMAND_PREFIX APP_CMD_JOYSTICK_CALIB " " APP_ARG_MAX
#define APP_RX_APP_DIR_STOP                APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_STOP
#define APP_RX_APP_DIR_UP                  APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_UP
#define APP_RX_APP_DIR_DOWN                APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_DOWN
#define APP_RX_APP_DIR_LEFT                APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_LEFT
#define APP_RX_APP_DIR_RIGHT               APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_RIGHT
#define APP_RX_APP_DIR_ULEFT               APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_ULEFT
#define APP_RX_APP_DIR_URIGHT              APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_URIGHT
#define APP_RX_APP_DIR_DLEFT               APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_DLEFT
#define APP_RX_APP_DIR_DRIGHT              APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_DIR " " APP_ARG_DRIGHT
#define APP_RX_APP_SPEED                   APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_SPEED
#define APP_RX_APP_START_TRACKING          APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_START " " APP_ARG_TRACKING
#define APP_RX_APP_TOGGLE                  APP_COMMAND_PREFIX APP_CMD_APP " " APP_ARG_TOGGLE
#define APP_RX_APP_SKY_BOX_ASSIST_STOP     APP_COMMAND_PREFIX APP_CMD_APP " " APP_CMD_SKY_BOX_ASSIST " " APP_ARG_STOP
#define APP_RX_APP_SKY_BOX_ASSIST_START    APP_COMMAND_PREFIX APP_CMD_APP " " APP_CMD_SKY_BOX_ASSIST " " APP_ARG_START
#define APP_RX_NEXT_PREFIX                 APP_COMMAND_PREFIX APP_ARG_NEXT " "
#define APP_RX_APP_JOYSTICK_START          APP_COMMAND_PREFIX APP_CMD_APP_JOYSTICK " " APP_ARG_START
#define APP_RX_FINISH_2_STAR_CALIB         APP_COMMAND_PREFIX APP_ARG_FINISH_2_STAR_CALIB
#define APP_RX_DIR_CALIB_RESULT_PREFIX     APP_ARG_DIR_CALIB_RESULT " "
#define APP_RX_NEXT                        APP_ARG_NEXT

/* Global macros ---------------------------------------------------------------------------------------------------- */

#define APP_TEXT_LENGTH(text)                           ((int)(sizeof(text) - 1))

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

// Legacy comparison counts. Keep prefix-only and NUL-inclusive matches distinct.
// A +1 count includes the NUL and therefore rejects trailing input.
#define APP_COMMAND_PREFIX_MATCH_LENGTH                 APP_TEXT_LENGTH(APP_COMMAND_PREFIX)
#define APP_RX_ANALOG_SPEED_MATCH_LENGTH                APP_TEXT_LENGTH(APP_RX_ANALOG_SPEED)
#define APP_RX_ANALOG_TOGGLE_MATCH_LENGTH               APP_TEXT_LENGTH(APP_RX_ANALOG_TOGGLE)
#define APP_RX_ANALOG_SKY_BOX_ASSIST_STOP_MATCH_LENGTH  APP_TEXT_LENGTH(APP_RX_ANALOG_SKY_BOX_ASSIST_STOP)
#define APP_RX_ANALOG_SKY_BOX_ASSIST_START_MATCH_LENGTH APP_TEXT_LENGTH(APP_RX_ANALOG_SKY_BOX_ASSIST_START)
#define APP_RX_ANALOG_JOYSTICK_MATCH_LENGTH             (APP_TEXT_LENGTH(APP_RX_ANALOG_JOYSTICK) + 1)
#define APP_RX_ANALOG_STOP_TRACKING_MATCH_LENGTH        APP_TEXT_LENGTH(APP_RX_ANALOG_STOP_TRACKING)
#define APP_RX_JOYSTICK_CALIB_MAX_MATCH_LENGTH          APP_TEXT_LENGTH(APP_RX_JOYSTICK_CALIB_MAX)
#define APP_RX_APP_SPEED_MATCH_LENGTH                   APP_TEXT_LENGTH(APP_RX_APP_SPEED)
#define APP_RX_APP_TOGGLE_MATCH_LENGTH                  APP_TEXT_LENGTH(APP_RX_APP_TOGGLE)
#define APP_RX_APP_SKY_BOX_ASSIST_STOP_MATCH_LENGTH     APP_TEXT_LENGTH(APP_RX_APP_SKY_BOX_ASSIST_STOP)
#define APP_RX_APP_SKY_BOX_ASSIST_START_MATCH_LENGTH    APP_TEXT_LENGTH(APP_RX_APP_SKY_BOX_ASSIST_START)
#define APP_RX_NEXT_PREFIX_MATCH_LENGTH                 APP_TEXT_LENGTH(APP_RX_NEXT_PREFIX)
#define APP_RX_NONE_MATCH_LENGTH                        (APP_TEXT_LENGTH(APP_RX_NONE) + 1)
#define APP_RX_APP_JOYSTICK_START_MATCH_LENGTH          (APP_TEXT_LENGTH(APP_RX_APP_JOYSTICK_START) + 1)
#define APP_RX_FINISH_2_STAR_CALIB_MATCH_LENGTH         (APP_TEXT_LENGTH(APP_RX_FINISH_2_STAR_CALIB) + 1)
#define APP_RX_DIR_CALIB_RESULT_PREFIX_MATCH_LENGTH     APP_TEXT_LENGTH(APP_RX_DIR_CALIB_RESULT_PREFIX)
#define APP_RX_NEXT_MATCH_LENGTH                        APP_TEXT_LENGTH(APP_RX_NEXT)

// Offsets in the command AFTER stripping APP_COMMAND_PREFIX.
// Each skips the keyword and one separator, exactly as before.
// No whitespace normalization or parser fixes are performed here.
#define APP_OFFSET_SET_TIME          (APP_TEXT_LENGTH(APP_CMD_SET_TIME) + 1)
#define APP_OFFSET_SET_DATE          (APP_TEXT_LENGTH(APP_CMD_SET_DATE) + 1)
#define APP_OFFSET_SET_LOCATION      (APP_TEXT_LENGTH(APP_CMD_SET_LOCATION) + 1)
#define APP_OFFSET_OTHER_SETTINGS    (APP_TEXT_LENGTH(APP_CMD_OTHER_SETTINGS) + 1)
#define APP_OFFSET_STAR_1_CALIB      (APP_TEXT_LENGTH(APP_CMD_STAR_1_CALIB) + 1)
#define APP_OFFSET_STAR_2_CALIB      (APP_TEXT_LENGTH(APP_CMD_STAR_2_CALIB) + 1)
#define APP_OFFSET_BACKLASH_CALIB    (APP_TEXT_LENGTH(APP_CMD_BACKLASH_CALIB) + 1)
#define APP_OFFSET_GOTO_ANY_OBJECT   (APP_TEXT_LENGTH(APP_CMD_GOTO_ANY_OBJECT) + 1)
#define APP_OFFSET_APP_JOYSTICK      (APP_TEXT_LENGTH(APP_CMD_APP_JOYSTICK) + 1)
#define APP_OFFSET_SET_GEAR_RATIO    (APP_TEXT_LENGTH(APP_CMD_SET_GEAR_RATIO) + 1)
#define APP_OFFSET_GOTO_SOLAR_SYSTEM (APP_TEXT_LENGTH(APP_CMD_GOTO_SOLAR_SYSTEM) + 1)
#define APP_OFFSET_JOYSTICK_CALIB    (APP_TEXT_LENGTH(APP_CMD_JOYSTICK_CALIB) + 1)
#define APP_OFFSET_DIRECTION_SETUP   (APP_TEXT_LENGTH(APP_CMD_DIRECTION_SETUP) + 1)
#define APP_OFFSET_SOLAR_OBJ_STATUS  (APP_TEXT_LENGTH(APP_CMD_SOLAR_OBJ_STATUS) + 1)

// Offsets into the FULL raw receive buffer (different from the trimmed offsets above).
#define APP_COMMAND_PREFIX_LENGTH       APP_TEXT_LENGTH(APP_COMMAND_PREFIX)
#define APP_RAW_OFFSET_NEXT             APP_TEXT_LENGTH(APP_RX_NEXT_PREFIX)
#define APP_RAW_OFFSET_DIR_CALIB_RESULT APP_TEXT_LENGTH(APP_RX_DIR_CALIB_RESULT_PREFIX)
#define APP_RAW_OFFSET_ANALOG_SPEED     (APP_TEXT_LENGTH(APP_RX_ANALOG_SPEED) + 1)
#define APP_RAW_OFFSET_ANALOG_TOGGLE    (APP_TEXT_LENGTH(APP_RX_ANALOG_TOGGLE) + 1)
#define APP_RAW_OFFSET_APP_SPEED        APP_TEXT_LENGTH(APP_RX_APP_SPEED)
#define APP_RAW_OFFSET_APP_TOGGLE       (APP_TEXT_LENGTH(APP_RX_APP_TOGGLE) + 1)

// ESP -> app text messages. Formatting, field order and trailing spaces are wire ABI.
#define APP_TX_READY                                 "READY"
#define APP_TX_BUSY                                  "BUSY"
#define APP_TX_OBJECT_VISIBLE_FORMAT                 "OBJECT_VISIBLE %d"
#define APP_TX_ESP_RESPONSE_FORMAT                   "ESP_RESPONSE %d"
#define APP_TX_RADEC_FORMAT                          "RADEC %d %d %d %d %d %d "
#define APP_TX_SOLAR_OBJ_STATUS_FORMAT               APP_CMD_SOLAR_OBJ_STATUS " %d %d %d %d %d %d %d %d %d"
#define APP_TX_SHOW_SETTINGS_FIRMWARE_VERSION_FORMAT APP_CMD_SHOW_SETTINGS " FIRMWARE_VERSION %.1f"
#define APP_TX_SHOW_SETTINGS_GEAR_FORMAT             APP_CMD_SHOW_SETTINGS " GEAR %f %f"
#define APP_TX_SHOW_SETTINGS_IDLE_FORMAT             APP_CMD_SHOW_SETTINGS " IDLE %d %d %d %d %d %d"
#define APP_TX_SHOW_SETTINGS_INV_FORMAT              APP_CMD_SHOW_SETTINGS " INV %d %d"
#define APP_TX_SHOW_SETTINGS_DATE_FORMAT             APP_CMD_SHOW_SETTINGS " DATE %d %d %d"
#define APP_TX_SHOW_SETTINGS_TIME_FORMAT             APP_CMD_SHOW_SETTINGS " TIME %d %d %d"
#define APP_TX_SHOW_SETTINGS_LOC_FORMAT              APP_CMD_SHOW_SETTINGS " LOC %f %f"
#define APP_TX_SHOW_SETTINGS_BACKLASH_FORMAT         APP_CMD_SHOW_SETTINGS " BACKLASH %d %d"
#define APP_TX_SHOW_SETTINGS_MAX_SPEED_FORMAT        APP_CMD_SHOW_SETTINGS " MAX_SPEED %f"
#define APP_TX_SHOW_SETTINGS_TRAC_REFR_FORMAT        APP_CMD_SHOW_SETTINGS " TRAC_REFR %f"
#define APP_TX_READY_LENGTH                          APP_TEXT_LENGTH(APP_TX_READY)

// Binary status notification: tag byte followed by the unchanged packed status byte.
#define APP_TX_STATUS_TAG       'S'
#define APP_TX_STATUS_LENGTH    2
#define APP_STATUS_TRACKING_BIT 0
#define APP_STATUS_ALIGNED_BIT  1
#define APP_STATUS_SYNC_SHIFT   2
#define APP_STATUS_SYNC_MASK    0b00000111

// Symbolic GOTO_SOLAR_SYSTEM arguments. Matching remains case-insensitive.
#define APP_BODY_MERCURY "Mercury"
#define APP_BODY_VENUS   "Venus"
#define APP_BODY_MARS    "Mars"
#define APP_BODY_JUPITER "Jupiter"
#define APP_BODY_SATURN  "Saturn"
#define APP_BODY_URANUS  "Uranus"
#define APP_BODY_NEPTUNE "Neptune"
#define APP_BODY_MOON    "Moon"
#define APP_BODY_SUN     "Sun"

#endif // MOBILE_APP_COMMANDS_H
