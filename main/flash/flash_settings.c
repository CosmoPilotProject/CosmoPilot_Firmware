/**
 * @file    flash_settings.c
 * @brief   Load persisted settings into application state.
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

#include "flash.h"

#include "config.h"
#include "nvs.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

esp_err_t load_values_from_flash(void)
{
    esp_err_t first_error = ESP_OK;
    esp_err_t err = load_location_from_flash(&global_longitude, &global_latitude);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND))
    {
        first_error = err;
    }

    err = load_directions_from_flash(&DIRECTION_1, &DIRECTION_2);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND) && (first_error == ESP_OK))
    {
        first_error = err;
    }

    err = load_backlash_calibration_from_flash(&global_backlash_steps_az, &global_backlash_steps_alt);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND) && (first_error == ESP_OK))
    {
        first_error = err;
    }

    err = load_gear_ratio_from_flash(&GEAR_RATIO_1, &GEAR_RATIO_2);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND) && (first_error == ESP_OK))
    {
        first_error = err;
    }

    err = load_other_settings_from_flash(&global_display_enable, &global_skybox_magnification, &global_skybox_speed,
                                         &global_standalone_mode, &global_max_speed, &global_tracking_refreshment);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND) && (first_error == ESP_OK))
    {
        first_error = err;
    }
    
    return first_error;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */
