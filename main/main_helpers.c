/**
 * @file    main_helpers.c
 * @brief   Application initialization and main-loop service implementation.
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

#include "main_helpers.h"
#include "config.h"
#include "bluetooth/bluetooth.h"
#include "telescope/headers/telescope_common.h"
#include "display/display.h"
#include "flash.h"
#include "joystick/analog_joystick.h"
#include "telescope/headers/telescope_stepper.h"
#include "mobile_app/headers/mobile_app_commands.h"
#include "mobile_app/headers/mobile_app_dispatch.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <string.h>
#include <time.h>

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace tag for application initialization and main-loop services.
 */
#define TRACE_NAME MAIN_HELPERS

/**
 * @brief   Trace verbosity for application initialization and main-loop services.
 */
#define TRACE_LEVEL MAIN_HELPERS_TRACE_LEVEL
#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/**
 * @brief   Main-loop display and notification intervals, in microseconds.
 */
enum
{
    DISPLAY_RESET_DELAY_US = 2000000, /**< Delay before the post-command display refresh. */
    READY_INTERVAL_US = 1000000,      /**< Interval between periodic READY notifications. */
    STATUS_INTERVAL_US = 2500000      /**< Interval between periodic status notifications. */
};

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Initialize the BLE controller, stack, callbacks, and GATT application.
 * @return  ESP_OK or the first returned setup error; MTU configuration errors are only logged.
 */
static esp_err_t bluetooth_init(void);

/**
 * @brief   Load joystick calibration or perform a requested recalibration.
 * @return  None.
 */
static void joytick_init(void);

/**
 * @brief   Initialize storage, Bluetooth, controls, settings, clock references, and display.
 * @return  ESP_OK or the first initialization error; later startup stages still run.
 */
static esp_err_t STARTUP_INIT(void);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

main_loop_state_t initialize_main_loop(void)
{
    // Snapshot once: Bluetooth may change global_standalone_mode later.
    bool standalone_enabled = global_standalone_mode;
    main_loop_state_t state = {.standalone_enabled_at_startup = standalone_enabled,
                               .display_reset_started_us = 0,
                               .last_ready_sent_us = esp_timer_get_time()};
    state.last_status_sent_us = state.last_ready_sent_us;
    return state;
}

void process_pending_command(main_loop_state_t *state)
{
    mobile_app_command_refresh();
    if (global_new_command_available)
    {
        if (strncmp(global_last_ble_command, APP_COMMAND_PREFIX, APP_COMMAND_PREFIX_MATCH_LENGTH) == 0)
        {
            if (global_esp_busy)
            {
                const char *msg = APP_TX_BUSY;
                ble_notify_app((uint8_t *) msg, strlen(msg));
                TRACE_WAR("Command rejected because the controller is busy");
            }
            else
            {
                global_esp_busy = true;
                global_status_message = STATUS_PROCESSING;
                update_display_screen();

                mobile_app_dispatch_command();

                // Show the handler's result before selecting idle for the delayed refresh.
                update_display_screen();
                global_status_message = STATUS_IDLE;
                state->display_reset_started_us = esp_timer_get_time();

                send_ready_status(5); // Also clears global_esp_busy, as before.
            }
        }

        // Consume only the observed snapshot, including busy/non-CMD input.
        mobile_app_command_clear();
    }
}

void refresh_idle_display(main_loop_state_t *state)
{
    if (((esp_timer_get_time() - state->display_reset_started_us) > DISPLAY_RESET_DELAY_US) &&
        (state->display_reset_started_us > 0))
    {
        update_display_screen();
        state->display_reset_started_us = 0;
    }
}

void send_periodic_notifications(main_loop_state_t *state)
{
    // Keep separate clock reads: handlers and READY notifications can block.
    // READY also recovers an app that missed an earlier completion notification.
    if ((esp_timer_get_time() - state->last_ready_sent_us) > READY_INTERVAL_US)
    {
        send_ready_status(1);
        state->last_ready_sent_us = esp_timer_get_time();
    }

    if ((esp_timer_get_time() - state->last_status_sent_us) > STATUS_INTERVAL_US)
    {
        send_status_to_app();
        state->last_status_sent_us = esp_timer_get_time();
    }
}

void run_startup_mode(bool standalone_enabled)
{
    if (standalone_enabled)
    {
        process_command_id(STANDALONE_MODE);
    }
}

void initialize_application(void)
{
    TRACE_INF("Application starting");
    esp_err_t ret;
    ret = STARTUP_INIT();

    if (ret == ESP_OK)
    {
        TRACE_INF("Startup initialization complete");
    }
    else
    {
        TRACE_ERR("Startup initialization completed with error: %s", esp_err_to_name(ret));
    }

    startup_animation();
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static esp_err_t STARTUP_INIT(void)
{
    esp_err_t first_error = flash_init(); // Initialize Non-Volatile Storage (used for BLE pairing info, settings, etc.)
    esp_err_t ret = bluetooth_init();
    if (ret != ESP_OK && first_error == ESP_OK)
    {
        first_error = ret;
    }
    motor_init(); // Initialize motor control pins and joystick
    init_joystick_button();
    joytick_init(); // Load joystick calibration or recalibrate when requested.
    ret = load_values_from_flash();
    if (ret != ESP_OK && first_error == ESP_OK)
    {
        first_error = ret;
    }

    reset_astronomy_time(time(NULL)); // Unsynchronized defaults are replaced by later date/time commands.

    ret = init_display();
    if (ret != ESP_OK)
    {
        TRACE_WAR("Display initialization failed (%s); continuing without OLED", esp_err_to_name(ret));
        global_display_present = false;
        if (first_error == ESP_OK)
        {
            first_error = ret;
        }
    }
    return first_error;
}

static esp_err_t bluetooth_init(void)
{

    esp_err_t ret;
    // Classic Bluetooth is not used, release memory to free RAM
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize Bluetooth Controller (handles radio layer, BLE packets)
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        TRACE_ERR("Bluetooth controller initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable the Bluetooth controller in BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        TRACE_ERR("Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid stack (handles upper BLE layers: GAP, GATT)
    ret = esp_bluedroid_init();
    if (ret)
    {
        TRACE_ERR("Bluedroid initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret)
    {
        TRACE_ERR("Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GATT event handler (for server-side BLE events like read/write/connect)
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        TRACE_ERR("GATT callback registration failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Register GAP event handler (for advertising, scan, connect params)
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        TRACE_ERR("GAP callback registration failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Register application profiles — app ID identifies your service/logic
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID); // Main telescope control?
    if (ret)
    {
        TRACE_ERR("GATT application registration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set MTU size (max packet payload size); useful for sending longer messages
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        TRACE_WAR("Failed to set local Bluetooth MTU: %s", esp_err_to_name(local_mtu_ret));
    }

    TRACE_INF("Bluetooth and GATT server initialized");

    return ret;
}

static void joytick_init(void)
{
    manage_joystick_calibration(adc_handle);
}
