/**
 * @file    hw_config.h
 * @brief   Board selection, pin assignments, and hardware configuration.
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

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/**
 * @brief   Supported hardware configuration identifiers.
 */
#define HW_CONFIG_PCB_V1_2 1
#define HW_CONFIG_DIY      2

/**
 * @brief   Select the PCB V1.2 or DIY hardware configuration.
 */
#define HW_CONFIG HW_CONFIG_PCB_V1_2

/**
 * @brief   I2C data GPIO for the display.
 */
#define DISPLAY_SDA GPIO_NUM_18

/**
 * @brief   I2C clock GPIO for the display.
 */
#define DISPLAY_SCL GPIO_NUM_17

#if HW_CONFIG == HW_CONFIG_PCB_V1_2

/**
 * @brief   Enable GPIO for stepper driver 2.
 */
#define ENABLE_PIN_2 GPIO_NUM_1

/**
 * @brief   Shared microstep selection MS1 GPIO.
 */
#define MS1_PIN_1    GPIO_NUM_40

/**
 * @brief   Shared microstep selection MS2 GPIO.
 */
#define MS2_PIN_1    GPIO_NUM_39

/**
 * @brief   Step pulse GPIO for stepper driver 2.
 */
#define STEP_PIN_2   GPIO_NUM_42

/**
 * @brief   Direction GPIO for stepper driver 2.
 */
#define DIR_PIN_2    GPIO_NUM_2

/**
 * @brief   Enable GPIO for stepper driver 1.
 */
#define ENABLE_PIN_1 GPIO_NUM_41

/**
 * @brief   Step pulse GPIO for stepper driver 1.
 */
#define STEP_PIN_1   GPIO_NUM_38

/**
 * @brief   Direction GPIO for stepper driver 1.
 */
#define DIR_PIN_1    GPIO_NUM_37


/**
 * @brief   Enable GPIO assigned to stepper driver 3.
 */
#define ENABLE_PIN_3 GPIO_NUM_47

/**
 * @brief   Step pulse GPIO assigned to stepper driver 3.
 */
#define STEP_PIN_3   GPIO_NUM_36

/**
 * @brief   Direction GPIO assigned to stepper driver 3.
 */
#define DIR_PIN_3    GPIO_NUM_35

/**
 * @brief   Joystick push-button GPIO.
 */
#define JOYSTICK_SW GPIO_NUM_6

/**
 * @brief   ADC channel for the joystick X axis.
 */
#define JOYSTICK_ADC_CHANNEL_X ADC_CHANNEL_3  // GPIO4

/**
 * @brief   ADC channel for the joystick Y axis.
 */
#define JOYSTICK_ADC_CHANNEL_Y ADC_CHANNEL_4  // GPIO5

#elif HW_CONFIG == HW_CONFIG_DIY

/**
 * @brief   Enable GPIO for stepper driver 1.
 */
#define ENABLE_PIN_1 GPIO_NUM_1

/**
 * @brief   Shared microstep selection MS1 GPIO.
 */
#define MS1_PIN_1    GPIO_NUM_40

/**
 * @brief   Shared microstep selection MS2 GPIO.
 */
#define MS2_PIN_1    GPIO_NUM_39

/**
 * @brief   Step pulse GPIO for stepper driver 1.
 */
#define STEP_PIN_1   GPIO_NUM_2

/**
 * @brief   Direction GPIO for stepper driver 1.
 */
#define DIR_PIN_1    GPIO_NUM_42

/**
 * @brief   Enable GPIO for stepper driver 2.
 */
#define ENABLE_PIN_2 GPIO_NUM_37

/**
 * @brief   Step pulse GPIO for stepper driver 2.
 */
#define STEP_PIN_2   GPIO_NUM_36

/**
 * @brief   Direction GPIO for stepper driver 2.
 */
#define DIR_PIN_2    GPIO_NUM_35

/**
 * @brief   Joystick push-button GPIO.
 */
#define JOYSTICK_SW GPIO_NUM_6

/**
 * @brief   ADC channel for the joystick X axis.
 */
#define JOYSTICK_ADC_CHANNEL_X ADC_CHANNEL_4  // GPIO5

/**
 * @brief   ADC channel for the joystick Y axis.
 */
#define JOYSTICK_ADC_CHANNEL_Y ADC_CHANNEL_3  // GPIO4

#else
#error "Unsupported HW_CONFIG: select HW_CONFIG_PCB_V1_2 or HW_CONFIG_DIY"
#endif

/**
 * @brief   Microsteps per full motor step for stepper 1.
 */
#define MICROSTEPS_1 64

/**
 * @brief   Full motor steps per revolution for stepper 1.
 */
#define STEPS_PER_REVOLUTION_1 200

/**
 * @brief   Microsteps per full motor step for stepper 2.
 */
#define MICROSTEPS_2 64

/**
 * @brief   Full motor steps per revolution for stepper 2.
 */
#define STEPS_PER_REVOLUTION_2 200


/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // HW_CONFIG_H
