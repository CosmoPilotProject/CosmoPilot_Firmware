/**
 * @file    sw_config.c
 * @brief   Software configuration defaults and shared runtime state.
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

#include "sw_config.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

//default gear ratios
double GEAR_RATIO_1 = 18.5;
double GEAR_RATIO_2 = 23.86;

AltAz global_possition = {0, 0};
RADec global_radec_position = {0, 0};
int global_count_steps_x = 0;
int global_count_steps_y = 0;

adc_oneshot_unit_handle_t adc_handle;
bool global_acceleration = 1;
float global_max_speed = 4.0;
float global_tracking_refreshment = 3.0;

time_t global_start_clock;
time_t global_start_clock_ut;
double initial_jd = 0;

float global_alt_visib_limit = 10.0; // Tracking uses half this visibility limit.

// Unsynchronized UTC defaults; replaced by date/time commands.
int global_year = DEFAULT_UTC_YEAR, global_month = DEFAULT_UTC_MONTH, global_day = DEFAULT_UTC_DAY;
double global_ut = DEFAULT_UTC_HOURS;

// Observer's default position
double global_longitude = 20.13456;
double global_latitude = 46.123456;
double global_jd = 0, global_gst = 0, global_lst = 0;

bool global_recalibrate_joystick = false; //acts as a flag to trigger joystick recalibration

double global_az_correction = 0, global_alt_correction = 0;

bool global_2_star_align = false;

JoystickCalibration global_joystick_calib = {2048, 2048, 0, 4000, 0, 4000};

// Motor direction polarity for each axis.
bool DIRECTION_1 = false;
bool DIRECTION_2 = false;

//disable/enable steppers
bool STEPPER_1_ENABLED = true; // true = enabled, false = disabled
bool STEPPER_2_ENABLED = true;

int16_t global_backlash_steps_alt = 0;
int16_t global_backlash_steps_az = 0;

bool global_status_bar_screen_bluetooth = false;
uint8_t global_status_bar_screen_loc_date = 0b000;
atomic_bool global_status_bar_screen_telescope = false;
StatusMessage global_status_message = STATUS_BT_NOT_CONNECTED;
bool global_status_is_tracking_active = false;
bool global_standalone_mode = false;

bool global_esp_busy = false;
uint8_t global_transmit_status = 0;
bool global_ble_is_connected = false;
int64_t global_ble_connected_start_time = 0;

//other settings
bool global_display_present = true;
bool global_display_enable = true;
int global_skybox_magnification = 0;
int global_skybox_speed = 0;

/* Public Functions ------------------------------------------------------------------------------------------------- */

/* Private Functions ------------------------------------------------------------------------------------------------ */
