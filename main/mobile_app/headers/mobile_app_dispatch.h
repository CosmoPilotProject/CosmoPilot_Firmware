/**
 * @file    mobile_app_dispatch.h
 * @brief   Mobile-app command dispatch interface.
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

#ifndef MOBILE_APP_DISPATCH_H
#define MOBILE_APP_DISPATCH_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "mobile_app_parser.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Dispatch the current CMD-prefixed command snapshot in the application task.
 *
 * The caller owns refresh/clear, busy/status transitions and periodic notifications.
 * Handlers may block and poll additional commands; their arguments remain local.
 * @return  None.
 */
void mobile_app_dispatch_command(void);

/**
 * @brief   Dispatch a command ID using a local copy of the current command text.
 * @param   cmd Command ID to handle, including the internal STANDALONE_MODE ID.
 * @return  None.
 */
void process_command_id(CommandID cmd);

/**
 * @brief   Look up a command keyword and trace an invalid keyword without blocking dispatch.
 * @param   str Command keyword to look up.
 * @return  Matching command ID, or NONE for an unknown or invalid keyword.
 */
CommandID parse_command_id(const char *str);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // MOBILE_APP_DISPATCH_H
