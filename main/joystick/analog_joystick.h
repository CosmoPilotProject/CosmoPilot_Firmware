/**
 * @file    analog_joystick.h
 * @brief   Analog joystick calibration data and control interface.
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

#ifndef ANALOG_JOYSTICK_H
#define ANALOG_JOYSTICK_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "flash/flash.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Calibrated ADC center and range values for both joystick axes.
 */
struct JoystickCalibration
{
    int16_t x_idle; /**< X-axis center reading. */
    int16_t y_idle; /**< Y-axis center reading. */
    int16_t x_min;  /**< Minimum X-axis reading. */
    int16_t x_max;  /**< Maximum X-axis reading. */
    int16_t y_min;  /**< Minimum Y-axis reading. */
    int16_t y_max;  /**< Maximum Y-axis reading. */
};

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Configure the ESP32-S3 joystick button and ADC inputs.
 * @return  None.
 */
void init_joystick_button(void);

/**
 * @brief   Run analog joystick control with app settings and position notifications.
 * @return  None.
 */
void move_analog_joystick(void);

/**
 * @brief   Load saved calibration or measure, save and apply a requested recalibration.
 * @param   adc_handle ADC unit used to sample the joystick.
 * @return  False if requested recalibration was cancelled or acquisition/save failed; true otherwise.
 */
bool manage_joystick_calibration(adc_oneshot_unit_handle_t adc_handle);

/**
 * @brief   Measure joystick centers and extremes using the app calibration handshake.
 * @param   adc_handle ADC unit used to sample the joystick.
 * @param   result Measured values; unchanged on cancellation, ADC error or out-of-range data.
 * @return  True after all in-range samples complete in the same connected calibration session.
 */
bool calibrate_joystick(adc_oneshot_unit_handle_t adc_handle, JoystickCalibration *result);

/**
 * @brief   Normalize an ADC reading relative to its calibrated center and range.
 * @param   raw Raw ADC reading.
 * @param   idle Calibrated center reading.
 * @param   min Calibrated minimum reading.
 * @param   max Calibrated maximum reading.
 * @return  Signed fraction of the calibrated half-range, or zero when that range is zero.
 */
float normalize_axis(int raw, int idle, int min, int max);

/**
 * @brief   Run joystick control with button-selected speeds until standalone mode ends.
 * @return  None.
 */
void move_joystick_standalone_mode(void);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // ANALOG_JOYSTICK_H
