/**
 * @file    telescope_common.h
 * @brief   Shared includes for telescope module interfaces.
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

#ifndef TELESCOPE_COMMON_H
#define TELESCOPE_COMMON_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"
#include <stdbool.h>
#include <stdio.h>
#include "flash/flash.h"

// Shared header: declarations belong to the corresponding telescope modules.
#include "telescope_coordinates.h"
#include "telescope_datetime.h"
#include "telescope_alignment.h"
#include "telescope_goto.h"
#include "telescope_other.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_COMMON_H
