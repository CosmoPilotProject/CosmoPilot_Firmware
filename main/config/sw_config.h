/**
 * @file    sw_config.h
 * @brief   Software configuration constants, types, and shared state declarations.
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

#ifndef SW_CONFIG_H
#define SW_CONFIG_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include "esp_adc/adc_oneshot.h"
#include "joystick/analog_joystick.h"
#include "display/display.h"
#include <math.h>

#include "hw_config.h"

#include "mobile_app/headers/mobile_app_command_buffer.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

#if HW_CONFIG == HW_CONFIG_PCB_V1_2

/**
 * @brief   Firmware version identifier for PCB V1.2.
 */
#define FIRMWARE_VERSION 21 //21 == 2.1

#elif HW_CONFIG == HW_CONFIG_DIY
/**
 * @brief   Firmware version identifier for DIY.
 */
#define FIRMWARE_VERSION 20 //20 == 2.0

#endif

/**
 * @brief   Minimum accepted motor gear reduction ratio.
 */
#define GEAR_RATIO_MIN 1.0

/**
 * @brief   Maximum accepted motor gear reduction ratio.
 */
#define GEAR_RATIO_MAX 200.0

/**
 * @brief   Minimum accepted backlash setting in arcseconds.
 */
#define BACKLASH_MIN_ARCSEC 0

/**
 * @brief   Maximum accepted backlash setting in arcseconds.
 */
#define BACKLASH_MAX_ARCSEC 5000

/**
 * @brief   Unsynchronized UTC year used at boot and factory reset.
 */
#define DEFAULT_UTC_YEAR 2026

/**
 * @brief   Unsynchronized UTC month used at boot and factory reset.
 */
#define DEFAULT_UTC_MONTH 10

/**
 * @brief   Unsynchronized UTC day used at boot and factory reset.
 */
#define DEFAULT_UTC_DAY 30

/**
 * @brief   Unsynchronized UTC hours used at boot and factory reset.
 */
#define DEFAULT_UTC_HOURS 20.0

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Right ascension and declination coordinates.
 */
typedef struct
{
    double ra;  /**< Right ascension in hours. */
    double dec; // Declination in degrees
} RADec;

/**
 * @brief   Altitude and azimuth coordinates in degrees.
 */
typedef struct
{
    double alt; // Altitude in degrees
    double az;  // Azimuth in degrees
} AltAz;

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   System-clock reference for advancing the Julian date.
 */
extern time_t global_start_clock; //for julian date calculations

/**
 * @brief   Legacy clock reference recording the most recent UTC snapshot.
 */
extern time_t global_start_clock_ut; //legacy UTC snapshot timestamp

/**
 * @brief   Current telescope altitude and azimuth in degrees.
 */
extern AltAz global_possition;

/**
 * @brief   Equatorial coordinates captured when joystick control starts tracking.
 */
extern RADec global_radec_position;

/**
 * @brief   Azimuth motion microstep count.
 */
extern int global_count_steps_x;

/**
 * @brief   Altitude motion microstep count.
 */
extern int global_count_steps_y;

/**
 * @brief   ADC unit handle used by the analog joystick.
 */
extern adc_oneshot_unit_handle_t adc_handle;

/**
 * @brief   Enable acceleration and deceleration during stepper movement.
 */
extern bool global_acceleration;

/**
 * @brief   Configured maximum telescope speed in degrees per second.
 */
extern float global_max_speed;

/**
 * @brief   Tracking update interval in seconds.
 */
extern float global_tracking_refreshment;

/**
 * @brief   UTC calendar date from the latest astronomical time snapshot.
 */
extern int global_year, global_month, global_day;

/**
 * @brief   UTC time of day from the latest astronomical snapshot, in decimal hours.
 */
extern double global_ut;

/**
 * @brief   Observer longitude in degrees, positive east.
 */
extern double global_longitude;

/**
 * @brief   Observer latitude in degrees.
 */
extern double global_latitude;

/**
 * @brief   Julian date in days and Greenwich/local sidereal times in hours.
 */
extern double global_jd, global_gst, global_lst;

/**
 * @brief   Julian date at the system-clock reference.
 */
extern double initial_jd;

/**
 * @brief   Object visibility altitude threshold in degrees.
 */
extern float global_alt_visib_limit;

/**
 * @brief   Request flag for analog joystick recalibration.
 */
extern bool global_recalibrate_joystick;

/**
 * @brief   Azimuth and altitude position corrections in degrees.
 */
extern double global_az_correction, global_alt_correction;

/**
 * @brief   Enable use of two-star pointing correction.
 */
extern bool global_2_star_align;

/**
 * @brief   Analog joystick center and endpoint calibration values.
 */
extern JoystickCalibration global_joystick_calib;

/**
 * @brief   Configured altitude motor direction polarity.
 */
extern bool DIRECTION_1;

/**
 * @brief   Configured azimuth motor direction polarity.
 */
extern bool DIRECTION_2;

/**
 * @brief   Requested altitude stepper enable state.
 */
extern bool STEPPER_1_ENABLED;

/**
 * @brief   Requested azimuth stepper enable state.
 */
extern bool STEPPER_2_ENABLED;

/**
 * @brief   Altitude backlash compensation in motor microsteps.
 */
extern int16_t global_backlash_steps_alt;

/**
 * @brief   Azimuth backlash compensation in motor microsteps.
 */
extern int16_t global_backlash_steps_az;

/**
 * @brief   Bluetooth connection indicator displayed in the status bar.
 */
extern bool global_status_bar_screen_bluetooth;

/**
 * @brief   Synchronization flags: bit 0 time, bit 1 date, bit 2 location.
 */
extern uint8_t global_status_bar_screen_loc_date;

/**
 * @brief   Telescope alignment validity, also cleared from the BLE receive task.
 */
extern atomic_bool global_status_bar_screen_telescope;

/**
 * @brief   Current display status message.
 */
extern StatusMessage global_status_message;

/**
 * @brief   Indicates that telescope tracking is active.
 */
extern bool global_status_is_tracking_active;

/**
 * @brief   Application command-processing busy flag.
 */
extern bool global_esp_busy;

/**
 * @brief   Packed tracking, alignment, and synchronization status sent over BLE.
 */
extern uint8_t global_transmit_status;

/**
 * @brief   Current BLE connection state.
 */
extern bool global_ble_is_connected;

/**
 * @brief   BLE connection start timestamp in microseconds since boot.
 */
extern int64_t global_ble_connected_start_time;

/**
 * @brief   Enable joystick-only mode at application startup.
 */
extern bool global_standalone_mode;

/**
 * @brief   Indicates that the display is available.
 */
extern bool global_display_present;

/**
 * @brief   Enable the normal status display.
 */
extern bool global_display_enable;

/**
 * @brief   Skybox magnification selection controlling movement increments.
 */
extern int global_skybox_magnification;

/**
 * @brief   Skybox movement speed selection.
 */
extern int global_skybox_speed;

/**
 * @brief   Gear reduction ratio for the altitude axis.
 */
extern double GEAR_RATIO_1;

/**
 * @brief   Gear reduction ratio for the azimuth axis.
 */
extern double GEAR_RATIO_2;

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // SW_CONFIG_H
