/**
 * @file    flash.h
 * @brief   Persistent settings storage and initialization interface.
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

#ifndef FLASH_H
#define FLASH_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Joystick center and endpoint calibration data.
 */
typedef struct JoystickCalibration JoystickCalibration;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Initialize NVS with the existing erase-and-retry recovery policy.
 * @return  ESP_OK on success, otherwise an error code.
 * @note    Recovery applies to ESP_ERR_NVS_NO_FREE_PAGES and ESP_ERR_NVS_NEW_VERSION_FOUND.
 * @note    A failed recovery erase aborts through ESP_ERROR_CHECK.
 */
esp_err_t flash_init(void);

/**
 * @brief   Erase the entire default NVS partition, including BLE data, then reinitialize it.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t flash_erase_all(void);

/**
 * @brief   Load application globals in startup order, attempting every settings group.
 * @return  ESP_OK if groups load or have no saved values, otherwise the first genuine error.
 */
esp_err_t load_values_from_flash(void);

/**
 * @brief   Save the motor direction flags.
 * @param   direction_1 Altitude motor direction flag.
 * @param   direction_2 Azimuth motor direction flag.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t save_directions_to_flash(bool direction_1, bool direction_2);
/**
 * @brief   Load the motor direction flags.
 * @param   direction_1 Output altitude motor direction flag.
 * @param   direction_2 Output azimuth motor direction flag.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t load_directions_from_flash(bool *direction_1, bool *direction_2);

/**
 * @brief   Save the observer location.
 * @param   longitude Longitude in degrees, positive east.
 * @param   latitude Latitude in degrees.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t save_location_to_flash(double longitude, double latitude);
/**
 * @brief   Load the observer location.
 * @param   longitude Output longitude in degrees, positive east.
 * @param   latitude Output latitude in degrees.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t load_location_from_flash(double *longitude, double *latitude);

/**
 * @brief   Save the motor gear ratios.
 * @param   gear_ratio1 Altitude motor gear ratio.
 * @param   gear_ratio2 Azimuth motor gear ratio.
 * @return  ESP_OK on success, otherwise an error code.
 * @note    Nonfinite or out-of-range ratios are rejected before any NVS write.
 */
esp_err_t save_gear_ratio_to_flash(double gear_ratio1, double gear_ratio2);
/**
 * @brief   Load the motor gear ratios.
 * @param   gear_ratio1 Output altitude motor gear ratio.
 * @param   gear_ratio2 Output azimuth motor gear ratio.
 * @return  ESP_OK on success, otherwise an error code.
 * @note    Invalid or unreadable groups leave both output values unchanged.
 */
esp_err_t load_gear_ratio_from_flash(double *gear_ratio1, double *gear_ratio2);

/**
 * @brief   Save the display, Skybox, standalone, and motion settings.
 * @param   display_setting Display enable flag.
 * @param   skybox_magnification Skybox magnification selection code.
 * @param   skybox_speed Skybox speed selection code.
 * @param   standalone_mode Standalone mode flag.
 * @param   max_speed Maximum telescope speed in degrees per second.
 * @param   track_refr Tracking refresh interval in seconds.
 * @return  ESP_OK on success, otherwise an error code.
 * @note    Invalid numeric settings are rejected before any NVS write.
 */
esp_err_t save_other_settings_to_flash(bool display_setting, int skybox_magnification, int skybox_speed, bool standalone_mode, float max_speed, float track_refr);
/**
 * @brief   Load the display, Skybox, standalone, and motion settings.
 * @param   display_setting Output display enable flag.
 * @param   skybox_magnification Output Skybox magnification selection code.
 * @param   skybox_speed Output Skybox speed selection code.
 * @param   standalone_mode Output standalone mode flag.
 * @param   max_speed Output maximum telescope speed in degrees per second.
 * @param   track_refr Output tracking refresh interval in seconds.
 * @return  ESP_OK on success, otherwise an error code.
 * @note    Missing legacy max_speed/track_refr keys retain the original 4.0f/3.0f defaults.
 * @note    Invalid or unreadable groups leave all output values unchanged.
 */
esp_err_t load_other_settings_from_flash(bool *display_setting, int *skybox_magnification, int *skybox_speed, bool *standalone_mode, float *max_speed, float *track_refr);

/**
 * @brief   Save the joystick center and endpoint calibration.
 * @param   calib Calibration values to save.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t save_joystick_calibration(const JoystickCalibration *calib);
/**
 * @brief   Load the joystick center and endpoint calibration.
 * @param   calib Output calibration values.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t load_joystick_calibration(JoystickCalibration *calib);

/**
 * @brief   Save the motor backlash calibration.
 * @param   backlash_steps_az Pointer to azimuth backlash in motor microsteps.
 * @param   backlash_steps_alt Pointer to altitude backlash in motor microsteps.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t save_backlash_calibration_to_flash(int16_t *backlash_steps_az, int16_t *backlash_steps_alt);
/**
 * @brief   Load the motor backlash calibration.
 * @param   out_az Output azimuth backlash in motor microsteps.
 * @param   out_alt Output altitude backlash in motor microsteps.
 * @return  ESP_OK on success, otherwise an error code.
 */
esp_err_t load_backlash_calibration_from_flash(int16_t *out_az, int16_t *out_alt);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // FLASH_H
