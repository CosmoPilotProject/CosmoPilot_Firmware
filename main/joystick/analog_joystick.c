/**
 * @file    analog_joystick.c
 * @brief   Analog joystick calibration and telescope motion control.
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

#include "driver/gpio.h"
#include "mobile_app/headers/mobile_app_commands.h"
#include "esp_timer.h"
#include "config.h"
#include "telescope/headers/telescope_stepper.h"
#include <math.h>
#include <stdlib.h>
#include "analog_joystick.h"
#include "flash.h"
#include "bluetooth/bluetooth.h"
#include "telescope/headers/telescope_common.h"
#include "telescope/headers/telescope_tracking.h"
#include "mobile_app/headers/mobile_app_handlers.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for analog joystick control.
 */
#define TRACE_NAME ANALOG_JOYSTICK

/**
 * @brief   Trace verbosity for analog joystick control.
 */
#define TRACE_LEVEL JOYSTICK_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

#define JOYSTICK_ADC_BITWIDTH ADC_BITWIDTH_12

/**
 * @brief   Normalized deflection required to engage or reverse an axis.
 */
#define JOYSTICK_ENGAGE_THRESHOLD  0.20f

/**
 * @brief   Normalized deflection magnitude at or below which a held axis is released.
 */
#define JOYSTICK_RELEASE_THRESHOLD 0.16f

/**
 * @brief   Initial motor speed for the joystick ramp, in degrees per second.
 */
#define JOYSTICK_START_SPEED_DPS   0.35f

/**
 * @brief   Duration of the joystick speed and cadence ramp, in milliseconds.
 */
#define JOYSTICK_RAMP_TIME_MS      900

/**
 * @brief   Base and maximum initial nudge interval, in milliseconds.
 */
#define JOYSTICK_START_INTERVAL_MS 200

/**
 * @brief   Minimum initial nudge interval, in milliseconds.
 */
#define JOYSTICK_MIN_INTERVAL_MS   20

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/**
 * @brief   Persistent ramp and movement timing for one joystick axis.
 */
typedef struct
{
    int64_t ramp_start_us; /**< Ramp start timestamp, in microseconds. */
    int64_t last_move_us;  /**< Last move start timestamp, or zero before a move, in microseconds. */
    int start_interval_us; /**< Initial nudge interval, in microseconds. */
    int last_direction;    /**< Previous requested direction: -1, 0, or 1. */
} joystick_axis_ramp_t;

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Wrap azimuth to [0, 360) degrees without discarding altitude travel.
 * @param   p Position to normalize in place.
 * @return  None.
 */
static void normalize_altaz(AltAz *p);

/**
 * @brief   Compute the speed-scaled initial nudge interval within the supplied limits.
 * @param   target_speed_dps Target motor speed, in degrees per second.
 * @param   START_INTERVAL_BASE_MS Base and maximum interval, in milliseconds.
 * @param   MIN_INTERVAL_MS_FAST Minimum interval, in milliseconds.
 * @return  Initial nudge interval, in microseconds.
 */
static inline int compute_start_interval_us(float target_speed_dps, int START_INTERVAL_BASE_MS,
                                            int MIN_INTERVAL_MS_FAST);

/**
 * @brief   Apply cubic ease-in to speed-ramp progress.
 * @param   t Ramp progress, clamped to [0, 1].
 * @return  Cubed progress after clamping.
 */
static inline float ease_in_cubic(float t);

/**
 * @brief   Read a latched physical axis direction before app inversion; reversals require the engage threshold.
 * @param   channel ADC channel to sample.
 * @param   idle Calibrated center reading.
 * @param   min Calibrated minimum reading.
 * @param   max Calibrated maximum reading.
 * @param   previous_direction Previously latched physical direction.
 * @return  -1 above center, 1 below center, or 0 when released or the ADC read fails.
 */
static int read_axis_direction(adc_channel_t channel, int idle, int min, int max, int previous_direction);

/**
 * @brief   Update axis ramp state for the requested direction and compute progress.
 * @param   ramp Axis ramp state to update.
 * @param   direction Requested direction: -1, 0, or 1.
 * @param   target_speed_dps Target motor speed, in degrees per second.
 * @param   now Current timestamp, in microseconds.
 * @return  Ramp progress clamped to [0, 1], or 1 for an idle axis.
 */
static float update_axis_ramp(joystick_axis_ramp_t *ramp, int direction, float target_speed_dps, int64_t now);

/**
 * @brief   Check whether a moving axis is due for its next nudge.
 * @param   ramp Axis ramp and movement timing state.
 * @param   direction Requested direction: -1, 0, or 1.
 * @param   progress Ramp progress from 0 to 1.
 * @param   now Current timestamp, in microseconds.
 * @return  True when a nonzero direction is due to move; otherwise false.
 */
static bool axis_move_due(const joystick_axis_ramp_t *ramp, int direction, float progress, int64_t now);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

void init_joystick_button(void)
{
    // Configure the selected joystick button GPIO as an input with pull-up.
    gpio_config_t button_config = {.pin_bit_mask = (1ULL << JOYSTICK_SW), // Configured joystick button pin.
                                   .mode = GPIO_MODE_INPUT,
                                   .pull_up_en = GPIO_PULLUP_ENABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&button_config);

    // Configure ADC1 for the selected joystick X and Y channels.
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // Joystick ADC unit.
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,          // 0–3.3V range
        .bitwidth = JOYSTICK_ADC_BITWIDTH, // 12-bit resolution
    };

    // Configure ADC1_CHANNEL_4 (GPIO5) for Y
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_ADC_CHANNEL_Y, &channel_config));
    // Configure ADC1_CHANNEL_3 (GPIO4) for X
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_ADC_CHANNEL_X, &channel_config));
}

void move_analog_joystick(void)
{
    const uint64_t generation = mobile_app_calibration_generation();
    const bool calibrating = mobile_app_calibration_pending();
    // App speed settings 1 and 2 use smaller nudges.
    const double STEP_DEG = 0.1; // per-call nudge angle (default)

    // Preserve the existing low-speed step sizes.
    const double STEP_DEG_LVL1 = 0.002; // speed setting 1
    const double STEP_DEG_LVL2 = 0.005; // speed setting 2

    // Each mode keeps its own persistent ramp history, as before.
    /**
     * @brief   Persistent altitude ramp state for app-configured analog control.
     */
    static joystick_axis_ramp_t alt_ramp = {.start_interval_us = JOYSTICK_START_INTERVAL_MS * 1000};
    /**
     * @brief   Persistent azimuth ramp state for app-configured analog control.
     */
    static joystick_axis_ramp_t az_ramp = {.start_interval_us = JOYSTICK_START_INTERVAL_MS * 1000};

    // Start each session with no held input; latches are independent of ramp history.
    int x_direction = 0, y_direction = 0;

    float target_speed_dps = global_max_speed / 2; // target speed (deg/s), controlled via BLE

    // Retain the app setting separately from the converted motor speed.
    float analog_speed_setting = 5.0f; // last received 1..10

    int64_t last_position_notify_us = esp_timer_get_time();
    const float position_notify_hz = 4.0f; // Hz

    bool invert_alt = false;
    bool invert_az = false;

    while (1)
    {
        if (calibrating && mobile_app_calibration_cancelled(generation))
        {
            break;
        }

        // 1. Sample both axes before processing settings, preserving the original order.
        x_direction = read_axis_direction(JOYSTICK_ADC_CHANNEL_X, global_joystick_calib.x_idle,
                                          global_joystick_calib.x_min, global_joystick_calib.x_max, x_direction);
        y_direction = read_axis_direction(JOYSTICK_ADC_CHANNEL_Y, global_joystick_calib.y_idle,
                                          global_joystick_calib.y_min, global_joystick_calib.y_max, y_direction);
        // Reverse the default vertical mapping; the app toggle reverses this default.
        int alt_direction = invert_alt ? x_direction : -x_direction;
        int az_direction = invert_az ? -y_direction : y_direction;

        // 2. Process app settings and nested modes. Inversion changes affect the next sample.
        mobile_app_command_refresh();
        if (strncmp(global_last_ble_command, APP_RX_ANALOG_SPEED, APP_RX_ANALOG_SPEED_MATCH_LENGTH) == 0)
        {
            const char *str = global_last_ble_command + APP_RAW_OFFSET_ANALOG_SPEED;
            char *end = NULL;
            float new_speed = strtof(str, &end);
            if (end != str && isfinite(new_speed))
            {
                if (new_speed < 1)
                    new_speed = 1;
                if (new_speed > 10)
                    new_speed = 10;

                // Keep the app setting for nudge-size selection.
                analog_speed_setting = new_speed;

                target_speed_dps = 0.1f + ((new_speed - 1.0f) * (global_max_speed - 0.1f)) / 9.0f;
            }
            mobile_app_command_clear();
        }
        else if (strcmp(global_last_ble_command, APP_RX_ANALOG_START_TRACKING) == 0)
        {
            mobile_app_command_clear();
            CALC_LST();
            RADec radec = altaz_to_radec(global_possition, global_latitude, global_lst);
            if (isfinite(radec.ra) && isfinite(radec.dec))
            {
                global_radec_position = radec;
                RA_Dec_tracking(global_radec_position);
            }
            else
            {
                TRACE_WAR("Tracking rejected: nonfinite position");
            }
            if (calibrating && mobile_app_calibration_cancelled(generation))
            {
                break;
            }
            mobile_app_command_refresh();
            if (strcmp(global_last_ble_command, APP_RX_NONE) == 0)
                break;
            mobile_app_command_clear();
        }
        else if (strncmp(global_last_ble_command, APP_RX_ANALOG_TOGGLE, APP_RX_ANALOG_TOGGLE_MATCH_LENGTH) == 0)
        {
            int ud = 0, lr = 0;
            if (sscanf(global_last_ble_command + APP_RAW_OFFSET_ANALOG_TOGGLE, "%d %d", &ud, &lr) == 2)
            {
                invert_alt = ud;
                invert_az = lr;
                TRACE_DEB("Invert U/D: %s | L/R: %s", invert_alt ? "ON" : "OFF", invert_az ? "ON" : "OFF");
            }
            mobile_app_command_clear();
        }
        else if (strncmp(global_last_ble_command, APP_RX_ANALOG_SKY_BOX_ASSIST_STOP,
                         APP_RX_ANALOG_SKY_BOX_ASSIST_STOP_MATCH_LENGTH) == 0)
        {
            mobile_app_command_clear();
        }
        else if (strncmp(global_last_ble_command, APP_RX_ANALOG_SKY_BOX_ASSIST_START,
                         APP_RX_ANALOG_SKY_BOX_ASSIST_START_MATCH_LENGTH) == 0)
        {
            process_sky_box_assist_command();
            if (calibrating && mobile_app_calibration_cancelled(generation))
            {
                break;
            }
            mobile_app_command_refresh();
            if (strcmp(global_last_ble_command, APP_RX_NONE) == 0)
                break;
            mobile_app_command_clear();
        }

        if (calibrating && mobile_app_calibration_cancelled(generation))
        {
            break;
        }

        // Select nudge size without changing the speed mapping.
        double step_deg = STEP_DEG;
        if (analog_speed_setting <= 1.5f)
            step_deg = STEP_DEG_LVL1; // speed = 1
        else if (analog_speed_setting <= 2.5f)
            step_deg = STEP_DEG_LVL2; // speed = 2

        // 3. Update the two ramps independently, then issue one combined motor move.
        int64_t now = esp_timer_get_time();
        float alt_ramp_progress = update_axis_ramp(&alt_ramp, alt_direction, target_speed_dps, now);
        float az_ramp_progress = update_axis_ramp(&az_ramp, az_direction, target_speed_dps, now);
        bool do_alt = axis_move_due(&alt_ramp, alt_direction, alt_ramp_progress, now);
        bool do_az = axis_move_due(&az_ramp, az_direction, az_ramp_progress, now);

        if (do_alt || do_az)
        {
            // Ease the motor speed while the cadence ramp builds up.
            float alt_speed_cmd = JOYSTICK_START_SPEED_DPS +
                                  (target_speed_dps - JOYSTICK_START_SPEED_DPS) * ease_in_cubic(alt_ramp_progress);
            float az_speed_cmd = JOYSTICK_START_SPEED_DPS +
                                 (target_speed_dps - JOYSTICK_START_SPEED_DPS) * ease_in_cubic(az_ramp_progress);

            // Use a nonzero dummy speed for an idle axis.
            double alt_deg = do_alt ? (step_deg * (alt_direction > 0 ? +1 : -1)) : 0.0;
            double az_deg = do_az ? (step_deg * (az_direction > 0 ? +1 : -1)) : 0.0;

            float alt_speed = do_alt ? alt_speed_cmd : 3.0f; // dummy on idle axis
            float az_speed = do_az ? az_speed_cmd : 3.0f;

            move_steppers_manual(alt_deg, alt_speed, az_deg, az_speed);

            // Record the start of the blocking move, not its completion.
            if (do_alt)
                alt_ramp.last_move_us = now;
            if (do_az)
                az_ramp.last_move_us = now;

            // Motor AZ direction and sky AZ coordinates have opposite signs.
            if (do_alt)
            {
                global_possition.alt +=
                    (alt_deg > 0 ? 1 : -1) *
                    (global_count_steps_y / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0));
            }
            if (do_az)
            {
                if (az_deg > 0)
                {
                    global_possition.az +=
                        -1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                }
                else
                {
                    global_possition.az +=
                        1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0));
                }
            }

            normalize_altaz(&global_possition);
        }

        // 4. Retain the existing notification cadence and command-exit check.
        if ((esp_timer_get_time() - last_position_notify_us) > (int64_t) ((1.0f / position_notify_hz) * 1e6f))
        {

            last_position_notify_us = esp_timer_get_time();

            CALC_LST();
            RADec radec = altaz_to_radec(global_possition, global_latitude, global_lst);
            if (isfinite(radec.ra) && isfinite(radec.dec))
            {
                RA_Dec_Split radec_split = split_radec(radec);
                if (isfinite(radec_split.ra_seconds) && isfinite(radec_split.dec_arcseconds))
                {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), APP_TX_RADEC_FORMAT, radec_split.ra_hours, radec_split.ra_minutes,
                             (int) radec_split.ra_seconds, radec_split.dec_degrees, radec_split.dec_arcminutes,
                             (int) radec_split.dec_arcseconds);
                    ble_notify_app((uint8_t *) buffer, strlen(buffer));
                }
            }

            // Foreign commands exit here, after the position notification, as before.
            mobile_app_command_refresh();
            if (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0 &&
                strncmp(global_last_ble_command, APP_RX_ANALOG_JOYSTICK, APP_RX_ANALOG_JOYSTICK_MATCH_LENGTH) !=
                    0 && // if we recive same command from app twice it should not exit analog joystick
                strncmp(global_last_ble_command, APP_RX_ANALOG_TOGGLE, APP_RX_ANALOG_TOGGLE_MATCH_LENGTH) != 0 &&
                strncmp(global_last_ble_command, APP_RX_ANALOG_STOP_TRACKING,
                        APP_RX_ANALOG_STOP_TRACKING_MATCH_LENGTH) != 0 &&
                strcmp(global_last_ble_command, APP_RX_ANALOG_START_TRACKING) != 0 &&
                strncmp(global_last_ble_command, APP_RX_ANALOG_SPEED, APP_RX_ANALOG_SPEED_MATCH_LENGTH) != 0 &&
                strncmp(global_last_ble_command, APP_RX_ANALOG_SKY_BOX_ASSIST_STOP,
                        APP_RX_ANALOG_SKY_BOX_ASSIST_STOP_MATCH_LENGTH) != 0 &&
                strncmp(global_last_ble_command, APP_RX_ANALOG_SKY_BOX_ASSIST_START,
                        APP_RX_ANALOG_SKY_BOX_ASSIST_START_MATCH_LENGTH) != 0)
            {
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool manage_joystick_calibration(adc_oneshot_unit_handle_t adc_handle)
{
    if (global_recalibrate_joystick)
    {
        const uint64_t generation = mobile_app_calibration_generation();
        JoystickCalibration calib;
        TRACE_INF("Recalibrating joystick...");
        const bool completed = calibrate_joystick(adc_handle, &calib);
        global_recalibrate_joystick = false;
        mobile_app_command_refresh();
        if (!completed || mobile_app_calibration_cancelled(generation) ||
            strcmp(global_last_ble_command, APP_RX_NONE) == 0)
        {
            TRACE_WAR("Joystick calibration did not complete; keeping previous values");
            return false;
        }

        // Keep runtime calibration unchanged until the completed candidate is saved.
        const esp_err_t err = save_joystick_calibration(&calib);
        if (err != ESP_OK)
        {
            TRACE_ERR("Joystick calibration save failed: %s; keeping runtime values", esp_err_to_name(err));
            return false;
        }
        global_joystick_calib = calib;

        TRACE_INF("Joystick calibration complete.");
        TRACE_DEB("X idle: %d, min: %d, max: %d", global_joystick_calib.x_idle, global_joystick_calib.x_min,
                  global_joystick_calib.x_max);
        TRACE_DEB("Y idle: %d, min: %d, max: %d", global_joystick_calib.y_idle, global_joystick_calib.y_min,
                  global_joystick_calib.y_max);

        char buffer[64];
        for (int i = 0; i < 3; i++)
        {
            mobile_app_command_refresh();
            if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
            {
                break;
            }
            snprintf(buffer, sizeof(buffer), APP_TX_ESP_RESPONSE_FORMAT, 1);
            ble_notify_app((uint8_t *) buffer, strlen(buffer));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    else
    {
        if (load_joystick_calibration(&global_joystick_calib) != ESP_OK)
        {
            TRACE_WAR("Could not load joystick calibration; keeping current values");
            TRACE_DEB("Automatic joystick calibration skipped");
        }
    }
    return true;
}

/**
 * @brief   Read one calibration sample, rejecting ADC errors and values outside the configured width.
 * @param   handle ADC unit used to sample the joystick.
 * @param   channel Joystick axis channel.
 * @param   raw Storage for a reading; use only when this function returns true.
 * @return  True for a successful in-range conversion; false after tracing the failure.
 */
static bool read_calibration_axis(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int *raw)
{
    const esp_err_t err = adc_oneshot_read(handle, channel, raw);
    if (err != ESP_OK)
    {
        TRACE_ERR("Joystick calibration ADC channel %d failed: %s", channel, esp_err_to_name(err));
        return false;
    }
    const int max_raw = (1 << JOYSTICK_ADC_BITWIDTH) - 1;
    if (*raw < 0 || *raw > max_raw)
    {
        TRACE_ERR("Joystick calibration ADC channel %d out of range: %d (0..%d)", channel, *raw, max_raw);
        return false;
    }
    return true;
}

bool calibrate_joystick(adc_oneshot_unit_handle_t adc_handle, JoystickCalibration *result)
{
    const uint64_t generation = mobile_app_calibration_generation();
    mobile_app_command_refresh();
    if (result == NULL || mobile_app_calibration_cancelled(generation) ||
        strcmp(global_last_ble_command, APP_RX_NONE) == 0)
    {
        return false;
    }

    JoystickCalibration calib = {0};
    int x_val = 0, y_val = 0;

    TRACE_INF("Center the joystick and set idle position...");
    if (!read_calibration_axis(adc_handle, JOYSTICK_ADC_CHANNEL_X, &x_val))
    {
        return false;
    }
    calib.x_idle = x_val;
    if (!read_calibration_axis(adc_handle, JOYSTICK_ADC_CHANNEL_Y, &y_val))
    {
        return false;
    }
    calib.y_idle = y_val;
    TRACE_DEB("Idle position: X = %d, Y = %d", calib.x_idle, calib.y_idle);

    // Initialize min/max with idle values
    calib.x_min = calib.x_max = calib.x_idle;
    calib.y_min = calib.y_max = calib.y_idle;

    mobile_app_command_refresh();
    if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
    {
        return false;
    }
    send_ready_status(3);

    while (1)
    {
        mobile_app_command_refresh();
        if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
        {
            return false;
        }
        if (strncmp(global_last_ble_command, APP_RX_JOYSTICK_CALIB_MAX, APP_RX_JOYSTICK_CALIB_MAX_MATCH_LENGTH) == 0)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
    {
        return false;
    }
    send_ready_status(3);

    TRACE_INF("Move the joystick to all extremes (full range) for 10 seconds...");

    for (int i = 0; i < 1000; ++i)
    {
        mobile_app_command_refresh();
        if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
        {
            return false;
        }
        if (!read_calibration_axis(adc_handle, JOYSTICK_ADC_CHANNEL_X, &x_val))
        {
            return false;
        }
        if (x_val < calib.x_min)
        {
            calib.x_min = x_val;
        }
        if (x_val > calib.x_max)
        {
            calib.x_max = x_val;
        }

        if (!read_calibration_axis(adc_handle, JOYSTICK_ADC_CHANNEL_Y, &y_val))
        {
            return false;
        }
        if (y_val < calib.y_min)
        {
            calib.y_min = y_val;
        }
        if (y_val > calib.y_max)
        {
            calib.y_max = y_val;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    mobile_app_command_refresh();
    if (mobile_app_calibration_cancelled(generation) || strcmp(global_last_ble_command, APP_RX_NONE) == 0)
    {
        return false;
    }
    *result = calib;
    return true;
}

float normalize_axis(int raw, int idle, int min, int max)
{
    if (raw > idle)
    {
        int denom = max - idle;
        if (denom == 0)
            return 0.0f; // avoid division by zero
        return (float) (raw - idle) / (float) denom;
    }
    else
    {
        int denom = idle - min;
        if (denom == 0)
            return 0.0f; // avoid division by zero
        return (float) (raw - idle) / (float) denom;
    }
}

void move_joystick_standalone_mode(void)
{
    // Standalone keeps its fixed nudge size and physical-button speed control.
    const double STEP_DEG = 0.1; // per-call nudge angle

    // Each mode keeps its own persistent ramp history, as before.
    /**
     * @brief   Persistent altitude ramp state for standalone control.
     */
    static joystick_axis_ramp_t alt_ramp = {.start_interval_us = JOYSTICK_START_INTERVAL_MS * 1000};
    /**
     * @brief   Persistent azimuth ramp state for standalone control.
     */
    static joystick_axis_ramp_t az_ramp = {.start_interval_us = JOYSTICK_START_INTERVAL_MS * 1000};

    // Start each session with no held input; latches are independent of ramp history.
    int x_direction = 0, y_direction = 0;

    // Start at medium speed, as before.
    float target_speed_dps = 2.0f; // deg/s (your original default)

    int button_state = 0;
    int last_button_state = 1;
    int64_t last_press_time = 0;
    const int64_t debounce_delay = 100000; // 100 ms

    global_status_message = STATUS_STANDALONE_MEDIUM;
    update_display_screen();

    while (1)
    {
        // 1. Sample both axes before processing settings, preserving the original order.
        x_direction = read_axis_direction(JOYSTICK_ADC_CHANNEL_X, global_joystick_calib.x_idle,
                                          global_joystick_calib.x_min, global_joystick_calib.x_max, x_direction);
        y_direction = read_axis_direction(JOYSTICK_ADC_CHANNEL_Y, global_joystick_calib.y_idle,
                                          global_joystick_calib.y_min, global_joystick_calib.y_max, y_direction);
        // Match the default vertical mapping used by app-controlled analog mode.
        int alt_direction = -x_direction;
        int az_direction = y_direction;

        // 2. Cycle speed on a debounced button release.
        int64_t now = esp_timer_get_time();
        button_state = gpio_get_level(JOYSTICK_SW);

        // The pulled-up button reads high when released.
        if (button_state == 1 && last_button_state == 0 && (now - last_press_time) > debounce_delay)
        {
            if (target_speed_dps == 0.3f)
            {
                target_speed_dps = 2.0f;
                global_status_message = STATUS_STANDALONE_MEDIUM;
            }
            else if (target_speed_dps == 2.0f)
            {
                target_speed_dps = global_max_speed;
                global_status_message = STATUS_STANDALONE_HIGH;
            }
            else
            {
                target_speed_dps = 0.3f;
                global_status_message = STATUS_STANDALONE_LOW;
            }
            update_display_screen();

            TRACE_DEB("Speed changed to: %.1f", target_speed_dps);
            last_press_time = now;
        }
        last_button_state = button_state;

        // 3. Update the two ramps independently, then issue one combined motor move.
        float alt_ramp_progress = update_axis_ramp(&alt_ramp, alt_direction, target_speed_dps, now);
        float az_ramp_progress = update_axis_ramp(&az_ramp, az_direction, target_speed_dps, now);
        bool do_alt = axis_move_due(&alt_ramp, alt_direction, alt_ramp_progress, now);
        bool do_az = axis_move_due(&az_ramp, az_direction, az_ramp_progress, now);

        // A stopped axis still receives the existing nonzero dummy speed.
        if (do_alt || do_az)
        {
            // Speed ramp: START_SPEED_DPS -> target speed, eased
            float alt_speed_cmd = JOYSTICK_START_SPEED_DPS +
                                  (target_speed_dps - JOYSTICK_START_SPEED_DPS) * ease_in_cubic(alt_ramp_progress);
            float az_speed_cmd = JOYSTICK_START_SPEED_DPS +
                                 (target_speed_dps - JOYSTICK_START_SPEED_DPS) * ease_in_cubic(az_ramp_progress);

            // Degrees for this nudge
            double alt_deg = do_alt ? (STEP_DEG * (alt_direction > 0 ? +1 : -1)) : 0.0;
            double az_deg = do_az ? (STEP_DEG * (az_direction > 0 ? +1 : -1)) : 0.0;

            // Dummy speed values for idle axis (keep non-zero)
            float alt_speed = do_alt ? alt_speed_cmd : 3.0f;
            float az_speed = do_az ? az_speed_cmd : 3.0f;

            move_steppers_manual(alt_deg, alt_speed, az_deg, az_speed);

            if (do_alt)
                alt_ramp.last_move_us = now;
            if (do_az)
                az_ramp.last_move_us = now;

            // Standalone retains its per-nudge corrections and does not normalize position.
            if (do_alt)
            {
                global_possition.alt +=
                    (alt_deg > 0 ? 1 : -1) *
                        (global_count_steps_y / ((STEPS_PER_REVOLUTION_1 * MICROSTEPS_1 * GEAR_RATIO_1) / 360.0)) +
                    global_alt_correction;
            }

            if (do_az)
            {
                if (az_deg > 0)
                {
                    global_possition.az +=
                        -1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0)) +
                        global_az_correction;
                }
                else
                {
                    global_possition.az +=
                        1 * (global_count_steps_x / ((STEPS_PER_REVOLUTION_2 * MICROSTEPS_2 * GEAR_RATIO_2) / 360.0)) +
                        global_az_correction;
                }
            }
        }

        // Check after the movement, preserving the existing BLE-connect exit ordering.
        if (false == global_standalone_mode)
            break;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static void normalize_altaz(AltAz *p)
{
    while (p->az >= 360.0)
        p->az -= 360.0;
    while (p->az < 0.0)
        p->az += 360.0;
}

static inline int compute_start_interval_us(float target_speed_dps, int START_INTERVAL_BASE_MS,
                                            int MIN_INTERVAL_MS_FAST)
{
    if (target_speed_dps < 0.1f)
        target_speed_dps = 0.1f;
    int start_ms = (int) ((float) START_INTERVAL_BASE_MS / target_speed_dps);
    if (start_ms < MIN_INTERVAL_MS_FAST)
        start_ms = MIN_INTERVAL_MS_FAST;
    if (start_ms > START_INTERVAL_BASE_MS)
        start_ms = START_INTERVAL_BASE_MS;
    return start_ms * 1000;
}

static inline float ease_in_cubic(float t)
{
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return t * t * t;
}

static int read_axis_direction(adc_channel_t channel, int idle, int min, int max, int previous_direction)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, channel, &raw) != ESP_OK)
    {
        return 0;
    }

    float normalized = normalize_axis(raw, idle, min, max);
    if (normalized > JOYSTICK_ENGAGE_THRESHOLD)
    {
        return -1;
    }
    if (normalized < -JOYSTICK_ENGAGE_THRESHOLD)
    {
        return 1;
    }
    if (previous_direction == -1 && normalized > JOYSTICK_RELEASE_THRESHOLD)
    {
        return -1;
    }
    if (previous_direction == 1 && normalized < -JOYSTICK_RELEASE_THRESHOLD)
    {
        return 1;
    }
    return 0;
}

static float update_axis_ramp(joystick_axis_ramp_t *ramp, int direction, float target_speed_dps, int64_t now)
{
    // Centering resets the ramp, so the next push starts gently.
    if (direction == 0 || direction != ramp->last_direction)
    {
        ramp->ramp_start_us = now;
        ramp->last_move_us = 0;
        ramp->start_interval_us =
            compute_start_interval_us(target_speed_dps, JOYSTICK_START_INTERVAL_MS, JOYSTICK_MIN_INTERVAL_MS);
        ramp->last_direction = direction;
    }

    float progress = 1.0f;
    if (direction != 0)
    {
        float elapsed_ms = (float) (now - ramp->ramp_start_us) / 1000.0f;
        progress = elapsed_ms / (float) JOYSTICK_RAMP_TIME_MS;
        if (progress > 1.0f)
            progress = 1.0f;
        if (progress < 0.0f)
            progress = 0.0f;
    }
    return progress;
}

static bool axis_move_due(const joystick_axis_ramp_t *ramp, int direction, float progress, int64_t now)
{
    if (direction == 0)
    {
        return false;
    }

    // Preserve the existing pacing: after ramp-up, the blocking motor call sets cadence.
    int interval_us = (int) ((float) ramp->start_interval_us * (1.0f - progress));
    return ramp->last_move_us == 0 || (now - ramp->last_move_us) >= interval_us;
}
