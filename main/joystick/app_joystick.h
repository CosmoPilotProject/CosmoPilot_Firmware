/**
 * @file    app_joystick.h
 * @brief   Mobile-app joystick control interface.
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

#ifndef APP_JOYSTICK_H
#define APP_JOYSTICK_H

/* Includes --------------------------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Process mobile-app joystick commands and update telescope motion.
 * @return  None.
 */
void move_app_joystick(void);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // APP_JOYSTICK_H
