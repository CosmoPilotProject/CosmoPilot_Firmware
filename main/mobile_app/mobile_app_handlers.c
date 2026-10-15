/**
 * @file    mobile_app_handlers.c
 * @brief   Handle mobile-app settings, calibration, and telescope commands.
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

#include "headers/mobile_app_handlers.h"
#include "headers/mobile_app_commands.h"
#include "bluetooth/bluetooth.h"
#include <string.h>
#include "telescope/headers/telescope_common.h"
#include "joystick/analog_joystick.h"
#include "joystick/app_joystick.h"
#include "telescope/headers/telescope_tracking.h"
#include "config.h"
#include "display/display.h"
#include "telescope/headers/telescope_stepper.h"
#include "flash.h"

#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier for mobile-app handlers.
 */
#define TRACE_NAME MOBILE_APP_HANDLERS

/**
 * @brief   Configured trace level for mobile-app handlers.
 */
#define TRACE_LEVEL MOBILE_APP_HANDLERS_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Locate the arguments at a fixed command offset.
 * @param   cmd NUL-terminated command, or NULL.
 * @param   offset Argument offset in bytes.
 * @return  Pointer to the arguments, or an empty string for a missing or short command.
 */
static const char *command_arguments(const char *cmd, size_t offset);

/**
 * @brief   Parse a requested number of decimal integers.
 * @param   text Text containing the numeric fields.
 * @param   values Output array with room for count integers.
 * @param   count Number of integers to parse.
 * @return  True if every requested field fits in an int; otherwise false.
 */
static bool parse_integers(const char *text, int *values, size_t count);

/**
 * @brief   Parse finite double values with optional trailing-input rejection.
 * @param   text Text containing the numeric fields.
 * @param   values Output array with room for count values.
 * @param   count Number of values to parse.
 * @param   require_end Whether to reject trailing non-whitespace input.
 * @return  True if parsing and the requested end check succeed; otherwise false.
 */
static bool parse_doubles(const char *text, double *values, size_t count, bool require_end);

/**
 * @brief   Check a gear ratio against the supported finite range.
 * @param   ratio Gear ratio to check.
 * @return  True if the ratio is finite and within the configured limits.
 */
static bool gear_ratio_is_valid(double ratio);

/**
 * @brief   Parse six equatorial coordinate components.
 * @param   text Text containing RA hours, minutes, seconds and declination degrees, minutes, seconds.
 * @param   coordinates Output array for the six components.
 * @return  True if all six components are parsed; otherwise false.
 */
static bool parse_coordinates(const char *text, float coordinates[6]);

/**
 * @brief   Check coordinate components for safe integer conversion and valid DMS sign placement.
 * @param   coordinates RA hours, minutes, seconds and declination degrees, minutes, seconds.
 * @return  True if integer casts are safe and only the first nonzero DMS component is signed.
 */
static bool coordinates_are_convertible(const float coordinates[6]);

/**
 * @brief   Trace a failed settings write to flash.
 * @param   setting Setting name for the diagnostic.
 * @param   err Flash operation result.
 * @return  None.
 */
static void report_flash_result(const char *setting, esp_err_t err);

/**
 * @brief   Format and send one settings reply with a 100 ms pacing delay.
 * @param   format Printf-style reply format.
 * @param   ... Values for the format conversions.
 * @return  None.
 */
static void send_settings_reply(const char *format, ...) __attribute__((format(printf, 1, 2)));

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void process_time_command(const char *cmd)
{
    int fields[3];
    if (!parse_integers(command_arguments(cmd, APP_OFFSET_SET_TIME), fields, 3) || fields[0] < 0 || fields[0] > 23 ||
        fields[1] < 0 || fields[1] > 59 || fields[2] < 0 || fields[2] > 59)
    {
        TRACE_ERR("Invalid time: expected hour 0..23, minute/second 0..59");
        send_ready_status(3);
        return;
    }

    const time_t now = time(NULL);
    (void) refresh_astronomy_time(now);
    global_ut = hms_to_double(fields[0], fields[1], fields[2]);
    reset_astronomy_time(now);
    send_ready_status(3);
    global_status_bar_screen_loc_date |= 0b001;
    global_status_message = STATUS_TIME_SET;
    TRACE_INF("Time synchronized");
}

void process_date_command(const char *cmd)
{
    int fields[3];
    if (!parse_integers(command_arguments(cmd, APP_OFFSET_SET_DATE), fields, 3))
    {
        TRACE_ERR("Invalid date format");
        send_ready_status(3);
        return;
    }

    int year = fields[0], month = fields[1], day = fields[2];
    /**
     * @brief   Month lengths for a non-leap year.
     */
    static const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool leap_year = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    // Bound year before the downstream Julian-date integer arithmetic.
    if (year < 1 || year > 9999 || month < 1 || month > 12 || day < 1 ||
        day > days_per_month[month - 1] + (month == 2 && leap_year))
    {
        TRACE_ERR("Invalid calendar date (supported years 1..9999)");
        send_ready_status(3);
        return;
    }

    const time_t now = time(NULL);
    (void) refresh_astronomy_time(now);
    global_year = year;
    global_month = month;
    global_day = day;
    reset_astronomy_time(now);
    send_ready_status(3);
    global_status_bar_screen_loc_date |= 0b010;
    global_status_message = STATUS_DATE_SET;
    TRACE_INF("Date synchronized");
}

void process_location_command(const char *cmd)
{
    double fields[2];
    if (!parse_doubles(command_arguments(cmd, APP_OFFSET_SET_LOCATION), fields, 2, false) || fields[0] < -90.0 ||
        fields[0] > 90.0 || fields[1] < -180.0 || fields[1] > 180.0)
    {
        TRACE_ERR("Invalid location: expected latitude -90..90, longitude -180..180");
        send_ready_status(3);
        return;
    }

    global_latitude = fields[0];
    global_longitude = fields[1];
    report_flash_result("location", save_location_to_flash(global_longitude, global_latitude));
    send_ready_status(3);
    global_status_bar_screen_loc_date |= 0b100;
    global_status_message = STATUS_LOCATION_SET;
    TRACE_INF("Location synchronized");
}

void process_other_settings_command(const char *cmd)
{
    // Six update flags, followed by twelve values. Preserve unused fields,
    // nonzero-as-true semantics, selection precedence and the shared skybox flag.
    /**
     * @brief   Indexes of the OTHER_SETTINGS command fields.
     */
    enum
    {
        UPDATE_DISPLAY,     /**< Display update flag. */
        UPDATE_SKYBOX,      /**< Skybox magnification and speed update flag. */
        UPDATE_STANDALONE,  /**< Standalone mode update flag. */
        UPDATE_MAX_SPEED,   /**< Maximum speed update flag. */
        UPDATE_REFRESH,     /**< Tracking refresh update flag. */
        UNUSED_FLAG,        /**< Unused update flag. */
        DISPLAY_ON,         /**< Display enabled value. */
        DISPLAY_OFF,        /**< Unused display disabled value. */
        MAG_LOW,            /**< Low skybox magnification selection. */
        MAG_MEDIUM,         /**< Medium skybox magnification selection. */
        MAG_HIGH,           /**< High skybox magnification selection. */
        SPEED_SLOW,         /**< Slow skybox speed selection. */
        SPEED_NORMAL,       /**< Normal skybox speed selection. */
        SPEED_FAST,         /**< Fast skybox speed selection. */
        STANDALONE_ON,      /**< Standalone enabled value. */
        STANDALONE_OFF,     /**< Unused standalone disabled value. */
        MAX_SPEED_TENTHS,   /**< Maximum speed in tenths of a degree per second. */
        REFRESH_HUNDREDTHS, /**< Tracking refresh interval in hundredths of a second. */
        FIELD_COUNT         /**< Number of command fields. */
    };
    int fields[FIELD_COUNT];
    if (!parse_integers(command_arguments(cmd, APP_OFFSET_OTHER_SETTINGS), fields, FIELD_COUNT))
    {
        TRACE_ERR("Invalid OTHER_SETTINGS: expected 18 integers");
        send_ready_status(3);
        return;
    }

    if (fields[UPDATE_DISPLAY])
    {
        global_display_enable = fields[DISPLAY_ON] != 0;
    }
    if (fields[UPDATE_SKYBOX])
    {
        global_skybox_magnification = fields[MAG_LOW] ? 1 : (fields[MAG_MEDIUM] ? 2 : (fields[MAG_HIGH] ? 3 : 0));
        global_skybox_speed = fields[SPEED_SLOW] ? 1 : (fields[SPEED_NORMAL] ? 2 : (fields[SPEED_FAST] ? 3 : 0));
    }
    if (fields[UPDATE_STANDALONE])
    {
        global_standalone_mode = fields[STANDALONE_ON] != 0;
    }
    if (fields[UPDATE_MAX_SPEED])
    {
        global_max_speed = (float) fields[MAX_SPEED_TENTHS] / 10.0f;
    }
    if (fields[UPDATE_REFRESH])
    {
        global_tracking_refreshment = (float) fields[REFRESH_HUNDREDTHS] / 100.0f;
    }

    // Existing fallbacks apply even when the corresponding update flag is unset.
    if (!isfinite(global_max_speed) || global_max_speed < 1.9 || global_max_speed > 8)
    {
        TRACE_WAR("Invalid maximum speed; using existing fallback 4");
        global_max_speed = 4.0f;
    }
    if (!isfinite(global_tracking_refreshment) || global_tracking_refreshment < 0.04f ||
        global_tracking_refreshment > 30.0f)
    {
        TRACE_WAR("Invalid tracking refresh; using existing fallback 3");
        global_tracking_refreshment = 3.0f;
    }
    report_flash_result("other settings", save_other_settings_to_flash(
                                              global_display_enable, global_skybox_magnification, global_skybox_speed,
                                              global_standalone_mode, global_max_speed, global_tracking_refreshment));
    send_ready_status(3);
    global_status_message = STATUS_OTHER_SETTINGS;
    TRACE_INF("Other settings applied");
}

void process_set_gear_ratio_command(const char *cmd)
{
    double ratios[2]; // Wire order: azimuth (X), altitude (Y).
    if (!parse_doubles(command_arguments(cmd, APP_OFFSET_SET_GEAR_RATIO), ratios, 2, true) ||
        !gear_ratio_is_valid(ratios[0]) || !gear_ratio_is_valid(ratios[1]))
    {
        TRACE_ERR("Invalid gear ratios: expected %.1f..%.1f on both axes", GEAR_RATIO_MIN, GEAR_RATIO_MAX);
        send_ready_status(3);
        return;
    }

    // Validate both axes before changing either. Keep double precision and NVS order.
    GEAR_RATIO_2 = ratios[0];
    GEAR_RATIO_1 = ratios[1];
    report_flash_result("gear ratios", save_gear_ratio_to_flash(GEAR_RATIO_1, GEAR_RATIO_2));
    global_status_bar_screen_telescope = false;
    global_2_star_align = false;
    send_ready_status(3);
    global_status_message = STATUS_SET_GEAR_RATIO;
    TRACE_INF("Gear ratios applied");
}

void process_backlash_calib_command(const char *cmd)
{
    double arcseconds[2]; // Wire order: azimuth, altitude; storage remains motor steps.
    if (!parse_doubles(command_arguments(cmd, APP_OFFSET_BACKLASH_CALIB), arcseconds, 2, true) ||
        arcseconds[0] < BACKLASH_MIN_ARCSEC || arcseconds[0] > BACKLASH_MAX_ARCSEC ||
        arcseconds[1] < BACKLASH_MIN_ARCSEC || arcseconds[1] > BACKLASH_MAX_ARCSEC ||
        trunc(arcseconds[0]) != arcseconds[0] || trunc(arcseconds[1]) != arcseconds[1])
    {
        TRACE_ERR("Invalid backlash: expected whole arcseconds %d..%d", BACKLASH_MIN_ARCSEC, BACKLASH_MAX_ARCSEC);
        send_ready_status(3);
        return;
    }

    // Old devices may contain invalid ratios. Check the actual conversion too,
    // so future config changes cannot overflow the int16_t compensation fields.
    if (!gear_ratio_is_valid(GEAR_RATIO_1) || !gear_ratio_is_valid(GEAR_RATIO_2))
    {
        TRACE_ERR("Cannot apply backlash: invalid gear ratios");
        send_ready_status(3);
        return;
    }

    float steps_per_degree_az = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0;
    float steps_per_degree_alt = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0;
    double steps_az = round(((float) arcseconds[0] / 3600) * (double) steps_per_degree_az);
    double steps_alt = round(((float) arcseconds[1] / 3600) * (double) steps_per_degree_alt);
    if (!isfinite(steps_az) || !isfinite(steps_alt) || steps_az > INT16_MAX || steps_alt > INT16_MAX)
    {
        TRACE_ERR("Cannot apply backlash: step count overflow");
        send_ready_status(3);
        return;
    }

    backlash_calibration((float) arcseconds[0], (float) arcseconds[1]);
    // The existing stepper helper reports persistence errors through the flash module.
    send_ready_status(3);
    global_status_message = STATUS_BACKLASH;
    TRACE_INF("Backlash applied");
}

void process_calib_start_command(const char *cmd)
{
    if (cmd != NULL && strcmp(cmd, APP_CMD_CALIB_START) == 0)
    {
        global_status_bar_screen_telescope = false;
        global_2_star_align = false;
        global_two_star_local.valid = false;
        TRACE_INF("Calibration started; alignment invalidated");
    }
    else
    {
        TRACE_WAR("Invalid calibration-start command");
    }
    send_ready_status(3);
}

void process_1_star_calib_command(const char *cmd)
{
    const char *calib_str = command_arguments(cmd, APP_OFFSET_STAR_1_CALIB);
    float coordinates[6];
    if (parse_coordinates(calib_str, coordinates) && coordinates_are_convertible(coordinates))
    {
        const uint64_t generation = mobile_app_calibration_begin();
        global_2_star_align = false;
        global_two_star_local.valid = false;
        if (mobile_app_calibration_cancelled(generation))
        {
            mobile_app_calibration_end(generation);
            global_status_message = STATUS_IDLE;
            send_ready_status(3);
            return;
        }
        if (!calibrate_to_radec(coordinates[0], coordinates[1], coordinates[2], coordinates[3], coordinates[4],
                                coordinates[5]) ||
            !mobile_app_calibration_complete(generation))
        {
            mobile_app_calibration_end(generation);
            global_status_message = STATUS_IDLE;
            send_ready_status(3);
            return;
        }
        mobile_app_calibration_end(generation);
        TRACE_INF("Calibration complete.");

        global_2_star_align = false;
    }
    else
    {
        TRACE_ERR("Invalid one-star coordinates");
        send_ready_status(3);
        return;
    }

    send_ready_status(3);

    global_status_message = STATUS_ALIGN_1_STAR;
}

void process_2_star_calib_command(const char *cmd)
{
    const char *calib_str = command_arguments(cmd, APP_OFFSET_STAR_2_CALIB);
    float coordinates[6];
    if (!parse_two_star_coordinates(calib_str, coordinates))
    {
        TRACE_WAR("Invalid first-star coordinates");
        send_ready_status(3);
        return;
    }

    global_2_star_align = false;
    const uint64_t generation = mobile_app_calibration_generation();
    const bool completed = two_star_calibration(coordinates[0], coordinates[1], coordinates[2], coordinates[3],
                                                coordinates[4], coordinates[5]);
    mobile_app_calibration_end(generation);
    if (!completed)
    {
        global_status_message = STATUS_IDLE;
    }
    send_ready_status(3);
    if (completed)
    {
        TRACE_INF("2 star calibration complete.");
        global_status_message = STATUS_ALIGN_2_STAR;
    }
}

void process_joystick_calib_command(const char *cmd)
{
    const char *str = command_arguments(cmd, APP_OFFSET_JOYSTICK_CALIB);
    // Preserve the existing invalid-command display policy; a running abort is idle.
    bool completed = true;

    if (strcmp(str, APP_ARG_IDLE) == 0)
    {
        mobile_app_command_clear();

        global_recalibrate_joystick = true;
        completed = manage_joystick_calibration(adc_handle);
    }
    else
    {
        TRACE_ERR("Invalid calibrate joystick command.");
    }

    send_ready_status(3);

    TRACE_DEB("Joystick calibration handler returned");

    global_status_message = completed ? STATUS_JOYSTICK_CALIB : STATUS_IDLE;
}

void process_direction_setup_command(const char *cmd)
{
    const char *str = command_arguments(cmd, APP_OFFSET_DIRECTION_SETUP);

    if (strcmp(str, APP_ARG_START) == 0)
    {
        mobile_app_command_clear();

        calibrate_direction_flags();

        global_status_bar_screen_telescope = false; // align calibration is no longer valid
        global_2_star_align = false;
    }
    else
    {
        TRACE_ERR("Invalid calibrate direction command.");
        global_status_message = STATUS_DIRECTION_CALIB; // Preserve the legacy invalid-command status.
    }

    send_ready_status(3);

    TRACE_DEB("Direction setup handler returned");

    // calibrate_direction_flags() owns the result status, including idle on cancellation.
}

void process_goto_any_object_command(const char *cmd)
{
    const char *calib_str = command_arguments(cmd, APP_OFFSET_GOTO_ANY_OBJECT);
    float coordinates[6];
    if (parse_coordinates(calib_str, coordinates))
    {
        if (!coordinates_are_convertible(coordinates))
        {
            TRACE_ERR("Object coordinates cannot be converted safely");
            // Keep the existing visibility notifications/clock update, including
            // tracking's suppression of replies, without unsafe float-to-int casts.
            (void) is_object_visible((RADec) {.ra = NAN, .dec = NAN}, global_alt_visib_limit);
        }
        else if (!goto_app_object(coordinates[0], coordinates[1], coordinates[2], coordinates[3], coordinates[4],
                                  coordinates[5]))
        {
            TRACE_WAR("Requested object move was not performed");
        }
    }
    else
    {
        TRACE_ERR("Invalid GOTO_ANY_OBJECT coordinates");
    }

    send_ready_status(3);

    global_status_message = STATUS_GOTO_OBJECT;
}

void process_goto_solar_system_command(const char *cmd)
{
    const char *str = command_arguments(cmd, APP_OFFSET_GOTO_SOLAR_SYSTEM);

    if (!goto_planet(str))
    {
        TRACE_WAR("Solar-system object move was not performed");
    }

    send_ready_status(3);

    TRACE_DEB("Solar-system object handler returned");
}

void process_analog_joystick_command(const char *cmd)
{
    (void) cmd;

    global_status_message = STATUS_ANALOG_JOYSTICK;
    update_display_screen();

    TRACE_INF("Free mode joystick.");
    mobile_app_command_clear();

    send_ready_status(3);

    global_acceleration = 0;
    move_analog_joystick();
    global_acceleration = 1;

    mobile_app_command_clear();

    send_ready_status(3);

    TRACE_INF("Analog joystick command complete");
}

void process_app_joystick_command(const char *cmd)
{
    global_status_message = STATUS_APP_JOYSTICK;
    update_display_screen();

    TRACE_INF("Free mode app joystick.");

    const char *str = command_arguments(cmd, APP_OFFSET_APP_JOYSTICK);

    if (strcmp(str, APP_ARG_START) == 0)
    {
        mobile_app_command_clear();

        send_ready_status(3);

        global_acceleration = 0;
        move_app_joystick();
        global_acceleration = 1;
    }
    else
    {
        TRACE_ERR("Could not start app joystick function, missing START command.");
    }

    mobile_app_command_clear();

    send_ready_status(3);

    TRACE_DEB("App joystick handler returned");
}

void process_standalone_command(void)
{
    global_status_message = STATUS_STANDALONE_MODE;
    update_display_screen();

    TRACE_INF("Standalone mode start.");

    global_acceleration = 0;
    move_joystick_standalone_mode();
    global_acceleration = 1;

    global_esp_busy = false;
    TRACE_INF("Standalone mode complete");
}

void process_sky_box_assist_command(void)
{
    StatusMessage last_status_message = global_status_message;
    global_status_message = STATUS_SKY_BOX_ASSIST;
    update_display_screen();

    TRACE_INF("SkyBox assist started.");

    sky_box_assists(global_skybox_magnification, global_skybox_speed);

    send_ready_status(3);

    global_status_message = last_status_message;
    update_display_screen();
    TRACE_INF("SkyBox assist command complete.");
}

void process_release_steppers_command(const char *cmd)
{
    (void) cmd;

    global_status_message = STATUS_RELEASE_STEPPERS;
    update_display_screen();

    STEPPER_1_ENABLED = 0;
    STEPPER_2_ENABLED = 0;
    update_stepper_enable_state();

    global_status_bar_screen_telescope = false; // telescope needs to be recalibrated again (star align)
    global_2_star_align = false;

    send_ready_status(3);

    mobile_app_command_clear();
    while (1)
    {
        mobile_app_command_refresh();
        if (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0)
        {
            // Preserve the existing exit handshake: this command is consumed below.
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    mobile_app_command_clear();

    STEPPER_1_ENABLED = 1;
    STEPPER_2_ENABLED = 1;
    update_stepper_enable_state();

    send_ready_status(3);
}

void process_show_settings_command(const char *cmd)
{
    (void) cmd;
    send_settings_reply(APP_TX_SHOW_SETTINGS_FIRMWARE_VERSION_FORMAT, (float) FIRMWARE_VERSION / 10.0f);

    send_settings_reply(APP_TX_SHOW_SETTINGS_GEAR_FORMAT, GEAR_RATIO_2, GEAR_RATIO_1);

    send_settings_reply(APP_TX_SHOW_SETTINGS_IDLE_FORMAT, global_joystick_calib.x_idle, global_joystick_calib.y_idle,
                        global_joystick_calib.x_min, global_joystick_calib.x_max, global_joystick_calib.y_min,
                        global_joystick_calib.y_max);

    send_settings_reply(APP_TX_SHOW_SETTINGS_INV_FORMAT, DIRECTION_1, DIRECTION_2);

    // DATE and TIME use one snapshot even if the paced replies cross midnight.
    const double snapshot_ut = current_time_in_hours(global_ut);
    send_settings_reply(APP_TX_SHOW_SETTINGS_DATE_FORMAT, global_year, global_month, global_day);

    if (isfinite(snapshot_ut) && snapshot_ut >= 0.0 && snapshot_ut < 24.0)
    {
        int seconds = (int) round(snapshot_ut * 3600.0);
        int hour = seconds / 3600;
        int minute = (seconds / 60) % 60;
        int second = seconds % 60;
        send_settings_reply(APP_TX_SHOW_SETTINGS_TIME_FORMAT, hour, minute, second);
    }
    else
    {
        TRACE_ERR("Cannot report time: invalid runtime value");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    send_settings_reply(APP_TX_SHOW_SETTINGS_LOC_FORMAT, global_latitude, global_longitude);

    // Preserve float intermediates and truncation when converting steps to arcseconds.
    double factor_az = (STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0f;
    double factor_alt = (STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0f;
    bool backlash_reported = false;
    if (isfinite(factor_az) && isfinite(factor_alt) && fabs(factor_az) >= FLT_MIN && fabs(factor_az) <= FLT_MAX &&
        fabs(factor_alt) >= FLT_MIN && fabs(factor_alt) <= FLT_MAX)
    {
        float steps_per_degree_az = (float) factor_az;
        float steps_per_degree_alt = (float) factor_alt;
        float backlash_deg_az = global_backlash_steps_az / steps_per_degree_az;
        float backlash_deg_alt = global_backlash_steps_alt / steps_per_degree_alt;
        float backlash_arcsec_az = backlash_deg_az * 3600.0f;
        float backlash_arcsec_alt = backlash_deg_alt * 3600.0f;
        if (isfinite(backlash_arcsec_az) && isfinite(backlash_arcsec_alt) && (double) backlash_arcsec_az >= INT_MIN &&
            (double) backlash_arcsec_az <= INT_MAX && (double) backlash_arcsec_alt >= INT_MIN &&
            (double) backlash_arcsec_alt <= INT_MAX)
        {
            send_settings_reply(APP_TX_SHOW_SETTINGS_BACKLASH_FORMAT, (int) backlash_arcsec_az,
                                (int) backlash_arcsec_alt);
            backlash_reported = true;
        }
    }
    if (!backlash_reported)
    {
        TRACE_ERR("Cannot report backlash: invalid runtime conversion");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    send_settings_reply(APP_TX_SHOW_SETTINGS_MAX_SPEED_FORMAT, global_max_speed);

    send_settings_reply(APP_TX_SHOW_SETTINGS_TRAC_REFR_FORMAT, global_tracking_refreshment);

    send_ready_status(3);

    TRACE_INF("Show settings command complete");
}

void process_solar_obj_status_command(const char *cmd)
{
    (void) cmd;

    // Compute visibility on ESP
    bool visible[9] = {0};
    fetch_solar_objects_visibility_status(visible);

    // Send to app (single packet)
    char buffer[128];
    int n = snprintf(buffer, sizeof(buffer), APP_TX_SOLAR_OBJ_STATUS_FORMAT,
                     visible[0] ? 1 : 0, // Mercury
                     visible[1] ? 1 : 0, // Venus
                     visible[2] ? 1 : 0, // Mars
                     visible[3] ? 1 : 0, // Jupiter
                     visible[4] ? 1 : 0, // Saturn
                     visible[5] ? 1 : 0, // Uranus
                     visible[6] ? 1 : 0, // Neptune
                     visible[7] ? 1 : 0, // Moon
                     visible[8] ? 1 : 0  // Sun
    );

    if (n > 0 && (size_t) n < sizeof(buffer))
    {
        ble_notify_app((uint8_t *) buffer, (size_t) n);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    else
    {
        TRACE_ERR("Solar-object status reply formatting failed");
    }

    // Finish like the other commands
    send_ready_status(3);

    TRACE_INF("Solar object status command complete.");
    global_status_message = STATUS_SOLAR_OBJ;
}

void process_factory_reset_command(const char *cmd)
{
    (void) cmd;

    TRACE_INF("Factory reset requested!");

    esp_err_t err = flash_erase_all();
    if (err != ESP_OK)
    {
        TRACE_ERR("Factory reset failed: %s", esp_err_to_name(err));
        return;
    }

    global_status_bar_screen_telescope = false; // align calibration is no longer valid
    global_2_star_align = false;

    GEAR_RATIO_1 = 20.123;
    GEAR_RATIO_2 = 25.123;

    global_year = DEFAULT_UTC_YEAR;
    global_month = DEFAULT_UTC_MONTH;
    global_day = DEFAULT_UTC_DAY;
    global_ut = DEFAULT_UTC_HOURS;

    // Observer's longitude (positive east, negative west)
    global_longitude = 20.13456;
    global_latitude = 46.123456;
    reset_astronomy_time(time(NULL));
    global_status_bar_screen_loc_date = 0;

    // Manual assignment
    global_joystick_calib.x_idle = 2048;
    global_joystick_calib.y_idle = 2048;
    global_joystick_calib.x_min = 0;
    global_joystick_calib.x_max = 4000;
    global_joystick_calib.y_min = 0;
    global_joystick_calib.y_max = 4000;

    // Restore the default motor direction polarity.
    DIRECTION_1 = false;
    DIRECTION_2 = false;

    global_backlash_steps_alt = 0;
    global_backlash_steps_az = 0;

    send_ready_status(3);

    TRACE_INF("Factory reset command complete");

    global_status_message = STATUS_FACTORY_RESET;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static const char *command_arguments(const char *cmd, size_t offset)
{
    if (cmd == NULL || strlen(cmd) < offset)
    {
        TRACE_ERR("Missing command arguments");
        return "";
    }
    return cmd + offset;
}

static bool parse_integers(const char *text, int *values, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        char *end;
        errno = 0;
        long value = strtol(text, &end, 10);
        if (end == text || errno == ERANGE || value < INT_MIN || value > INT_MAX)
        {
            return false;
        }
        values[i] = (int) value;
        text = end;
    }
    return true;
}

static bool parse_doubles(const char *text, double *values, size_t count, bool require_end)
{
    for (size_t i = 0; i < count; ++i)
    {
        char *end;
        errno = 0;
        values[i] = strtod(text, &end);
        if (end == text || errno == ERANGE || !isfinite(values[i]))
        {
            return false;
        }
        text = end;
    }
    while (isspace((unsigned char) *text))
    {
        ++text;
    }
    return !require_end || *text == '\0';
}

static bool gear_ratio_is_valid(double ratio)
{
    return isfinite(ratio) && ratio >= GEAR_RATIO_MIN && ratio <= GEAR_RATIO_MAX;
}

static bool parse_coordinates(const char *text, float coordinates[6])
{
    for (size_t i = 0; i < 6; ++i)
    {
        char *end;
        coordinates[i] = strtof(text, &end);
        if (end == text)
        {
            return false;
        }
        text = end;
    }
    return true;
}

static bool coordinates_are_convertible(const float coordinates[6])
{
    // The existing hms/dms helpers take integers. Preserve their truncation and
    // noncanonical magnitudes, but reject misplaced DMS signs and unsafe casts.
    for (size_t i = 0; i < 6; ++i)
    {
        double value = coordinates[i];
        if (!isfinite(value) || value < INT_MIN || value > INT_MAX)
        {
            return false;
        }
    }
    return (double) coordinates[3] > INT_MIN && (coordinates[4] >= 0.0f || coordinates[3] == 0.0f) &&
           (coordinates[5] >= 0.0f || (coordinates[3] == 0.0f && coordinates[4] == 0.0f));
}

static void report_flash_result(const char *setting, esp_err_t err)
{
    if (err != ESP_OK)
    {
        // Preserve accepted RAM settings on persistence failure, as before.
        TRACE_ERR("Could not save %s: %s (runtime values retained)", setting, esp_err_to_name(err));
    }
}

// Keep each settings packet and its delay separate, in the existing wire order.
static void send_settings_reply(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0 || (size_t) length >= sizeof(buffer))
    {
        TRACE_ERR("Settings reply formatting failed or exceeded buffer");
    }
    else
    {
        ble_notify_app((uint8_t *) buffer, (size_t) length);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
