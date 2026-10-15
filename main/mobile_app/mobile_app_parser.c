/**
 * @file    mobile_app_parser.c
 * @brief   Extract command keywords and map them to internal command IDs.
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

#include "headers/mobile_app_parser.h"
#include "headers/mobile_app_commands.h"
#include "headers/mobile_app_command_buffer.h"

#include <ctype.h>
#include <string.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   Command keyword and internal identifier lookup table.
 */
static const struct
{
    const char *keyword; /**< Case-sensitive command keyword. */
    CommandID id;        /**< Internal command identifier. */
} command_ids[] = {
    {APP_CMD_GOTO_ANY_OBJECT,   GOTO_ANY_OBJECT      },
    {APP_CMD_GOTO_SOLAR_SYSTEM, GOTO_SOLAR_SYSTEM    },
    {APP_CMD_STAR_1_CALIB,      STAR_1_CALIB         },
    {APP_CMD_STAR_2_CALIB,      STAR_2_CALIB         },
    {APP_CMD_STAR_3_CALIB,      STAR_3_CALIB         },
    {APP_CMD_CALIB_START,       CALIB_START          },
    {APP_CMD_BACKLASH_CALIB,    BACKLASH_CALIB       },
    {APP_CMD_PLAY_GAME,         PLAY_GAME            },
    {APP_CMD_ANALOG_JOYSTICK,   ANALOG_JOYSTICK      },
    {APP_CMD_APP_JOYSTICK,      APP_JOYSTICK         },
    {APP_CMD_SKY_BOX_ASSIST,    SKY_BOX_ASSIST       },
    {APP_CMD_RELEASE_STEPPERS,  RELEASE_STEPPERS     },
    {APP_CMD_BLUETOOTH,         BLUETOOTH            },
    {APP_CMD_SET_TIME,          SET_TIME             },
    {APP_CMD_SET_DATE,          SET_DATE             },
    {APP_CMD_SET_LOCATION,      SET_LOCATION         },
    {APP_CMD_OTHER_SETTINGS,    OTHER_SETTINGS       },
    {APP_CMD_SET_GEAR_RATIO,    SET_GEAR_RATIO       },
    {APP_CMD_JOYSTICK_CALIB,    JOSTICK_CALIB        },
    {APP_CMD_DIRECTION_SETUP,   DIRECTION_SETUP      },
    {APP_CMD_SHOW_SETTINGS,     SHOW_SETTINGS        },
    {APP_CMD_FACTORY_RESET,     FACTORY_RESET        },
    {APP_CMD_APP,               APP_J                },
    {APP_CMD_ANALOG,            ANALOG_J             },
    {APP_CMD_SOLAR_OBJ_STATUS,  SEND_SOLAR_OBJ_STATUS},
    {APP_CMD_NONE,              VOID_COMMAND         }
};

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

bool mobile_app_command_keyword(const char *message, char *keyword, size_t capacity)
{
    if (keyword == NULL || capacity == 0)
    {
        return false;
    }
    keyword[0] = '\0';
    if (message == NULL || strnlen(message, MAX_COMMAND_LENGTH) >= MAX_COMMAND_LENGTH ||
        strncmp(message, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) != 0)
    {
        return false;
    }

    const char *start = message + APP_COMMAND_PREFIX_LENGTH;
    while (isspace((unsigned char) *start))
    {
        start++;
    }
    size_t length = 0;
    while (start[length] != '\0' && !isspace((unsigned char) start[length]))
    {
        length++;
    }
    if (length == 0 || length >= capacity)
    {
        return false;
    }
    memcpy(keyword, start, length);
    keyword[length] = '\0';
    return true;
}

CommandID mobile_app_parse_command_id(const char *keyword)
{
    if (keyword == NULL)
    {
        return NONE;
    }
    for (size_t i = 0; i < sizeof(command_ids) / sizeof(command_ids[0]); i++)
    {
        if (strcmp(keyword, command_ids[i].keyword) == 0)
        {
            return command_ids[i].id;
        }
    }
    return NONE;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */
