/**
 * @file    mobile_app_command_buffer.c
 * @brief   BLE command handoff and manual-control connection lifecycle.
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

#include "headers/mobile_app_command_buffer.h"
#include "headers/mobile_app_commands.h"
#include "telescope/headers/telescope_stepper.h"

#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

// Shared configuration flag; atomic because BLE invalidates it before task dispatch.
extern atomic_bool global_status_bar_screen_telescope;

/* Trace ------------------------------------------------------------------------------------------------------------ */

/* Private Defines -------------------------------------------------------------------------------------------------- */

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

static bool command_matches(const uint8_t *data, size_t length, const char *text);
static bool app_direction_command(const uint8_t *data, size_t length);

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   Lock protecting the receive mailbox and connection lifecycle state.
 */
static portMUX_TYPE command_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief   Latest received command awaiting transfer to the application task.
 */
static char received_command[MAX_COMMAND_LENGTH];

/**
 * @brief   Whether the receive mailbox has a pending command or disconnect clear.
 */
static bool received_pending;

/**
 * @brief   Bluetooth connection state used by manual control.
 */
static bool connected;

/**
 * @brief   Disconnect counter used to distinguish manual-control connection lifetimes.
 */
static uint64_t connection_generation;

/**
 * @brief   Calibration lifetime, advanced by a new start or disconnect.
 */
static uint64_t calibration_generation;

/**
 * @brief   Calibration lifetime attached to the pending and current command snapshots.
 */
static uint64_t received_calibration_generation, command_calibration_generation;

// Alignment intent survives mailbox coalescing and covers first-star centering.
// Both fields are protected by command_lock.
static bool alignment_pending;
static uint64_t alignment_pending_generation;

/**
 * @brief   Whether the application task has begun an active manual-control scope.
 */
static bool manual_control_active;

/**
 * @brief   Disconnect counter captured when manual control begins.
 */
static uint64_t manual_connection_generation;

// App hold state and command identities are protected by command_lock. The
// identity distinguishes a new identical press from the retained direction text.
static uint64_t received_command_generation, command_generation;
static bool received_app_direction;
static bool app_joystick_watchdog_enabled, app_joystick_hold_active;
static int64_t app_joystick_last_keepalive_us;
// A finite move owns a separate lease: a pending STOP/new direction cannot
// disable or renew the old move while it is inside a long backlash operation.
static bool app_joystick_motion_active;
static uint64_t app_joystick_motion_generation;
static atomic_uint app_joystick_motion_deadline_us;
_Static_assert(sizeof(unsigned int) == sizeof(uint32_t) && ATOMIC_INT_LOCK_FREE == 2,
               "App joystick pulse deadline requires lock-free 32-bit loads");

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Application-task snapshot of the latest received command.
 */
char global_last_ble_command[MAX_COMMAND_LENGTH];
/**
 * @brief   Whether the application-task command snapshot is nonempty.
 */
bool global_new_command_available;

/* Public Functions ------------------------------------------------------------------------------------------------- */

esp_err_t mobile_app_command_receive(const uint8_t *data, size_t length)
{
    if (data == NULL && length != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    // Keep the legacy receive contract: one bounded message, then a local NUL.
    // Clamp before reading data, including when length itself is very large.
    if (length >= MAX_COMMAND_LENGTH)
    {
        length = MAX_COMMAND_LENGTH - 1;
    }

    const size_t stop_length = sizeof(APP_RX_STOP_GOTO) - 1;
    if (length >= stop_length && memcmp(data, APP_RX_STOP_GOTO, stop_length) == 0 &&
        (length == stop_length || data[stop_length] == '\0'))
    {
        // Stop GoTo has its own latch while the application is blocked.
        // Preserve first-NUL text semantics; never expose an idle stop to other modes.
        telescope_request_goto_stop();
        return ESP_OK;
    }

    if (command_matches(data, length, APP_RX_APP_KEEPALIVE))
    {
        portENTER_CRITICAL(&command_lock);
        if (app_joystick_watchdog_enabled && app_joystick_hold_active && connected)
        {
            const int64_t now = esp_timer_get_time();
            // Check before renewal: a late packet cannot rescue a timed-out hold
            // while the application task is still finishing a finite nudge.
            if (now - app_joystick_last_keepalive_us < APP_JOYSTICK_KEEPALIVE_TIMEOUT_US)
            {
                app_joystick_last_keepalive_us = now;
                if (app_joystick_motion_active && app_joystick_motion_generation == received_command_generation)
                    atomic_store_explicit(&app_joystick_motion_deadline_us,
                                          (uint32_t) (now + APP_JOYSTICK_KEEPALIVE_TIMEOUT_US), memory_order_relaxed);
            }
            else
                app_joystick_hold_active = false;
        }
        portEXIT_CRITICAL(&command_lock);
        return ESP_OK;
    }

    const bool direction = app_direction_command(data, length);
    portENTER_CRITICAL(&command_lock);
    ++received_command_generation;
    received_app_direction = direction && connected;
    app_joystick_hold_active = received_app_direction && app_joystick_watchdog_enabled;
    if (received_app_direction)
        app_joystick_last_keepalive_us = esp_timer_get_time();
    const size_t start_length = sizeof(APP_RX_CALIB_START) - 1;
    if (length >= start_length && memcmp(data, APP_RX_CALIB_START, start_length) == 0 &&
        (length == start_length || data[start_length] == '\0'))
    {
        ++calibration_generation;
        global_status_bar_screen_telescope = false;
        alignment_pending = true;
        alignment_pending_generation = calibration_generation;
    }
    memset(received_command, 0, sizeof(received_command));
    if (length != 0)
    {
        memcpy(received_command, data, length);
    }
    received_pending = true;
    received_calibration_generation = calibration_generation;
    portEXIT_CRITICAL(&command_lock);
    return ESP_OK;
}

void mobile_app_command_refresh(void)
{
    portENTER_CRITICAL(&command_lock);
    if (received_pending)
    {
        memcpy(global_last_ble_command, received_command, sizeof(global_last_ble_command));
        global_new_command_available = global_last_ble_command[0] != '\0';
        command_calibration_generation = received_calibration_generation;
        command_generation = received_command_generation;
        received_pending = false;
    }
    portEXIT_CRITICAL(&command_lock);
}

void mobile_app_command_clear(void)
{
    // A callback may publish during this clear; its independent pending copy survives.
    memset(global_last_ble_command, 0, sizeof(global_last_ble_command));
    global_new_command_available = false;
}

void mobile_app_command_connected(void)
{
    portENTER_CRITICAL(&command_lock);
    connected = true;
    portEXIT_CRITICAL(&command_lock);
}

void mobile_app_command_disconnected(void)
{
    portENTER_CRITICAL(&command_lock);
    connected = false;
    connection_generation++;
    ++received_command_generation;
    received_app_direction = false;
    app_joystick_watchdog_enabled = false;
    app_joystick_hold_active = false;
    calibration_generation++;
    memset(received_command, 0, sizeof(received_command));
    received_pending = true;
    received_calibration_generation = calibration_generation;
    portEXIT_CRITICAL(&command_lock);
}

uint64_t mobile_app_calibration_begin(void)
{
    portENTER_CRITICAL(&command_lock);
    global_status_bar_screen_telescope = false;
    const uint64_t generation = command_calibration_generation;
    if (connected && generation == calibration_generation)
    {
        alignment_pending = true;
        alignment_pending_generation = generation;
    }
    portEXIT_CRITICAL(&command_lock);
    return generation;
}

uint64_t mobile_app_calibration_generation(void)
{
    return command_calibration_generation; // Application-task snapshot, preserved by clear().
}

void mobile_app_calibration_end(uint64_t generation)
{
    portENTER_CRITICAL(&command_lock);
    if (alignment_pending_generation == generation)
    {
        alignment_pending = false;
    }
    portEXIT_CRITICAL(&command_lock);
}

bool mobile_app_calibration_pending(void)
{
    portENTER_CRITICAL(&command_lock);
    // A new start may replace the pending generation just before an older
    // centering command enters tracking. Keep that stale entry cancellable;
    // refreshed commands on a later connection do not inherit the old session.
    const bool pending = alignment_pending && (alignment_pending_generation == command_calibration_generation ||
                                               command_calibration_generation != calibration_generation);
    portEXIT_CRITICAL(&command_lock);
    return pending;
}

bool mobile_app_calibration_cancelled(uint64_t generation)
{
    portENTER_CRITICAL(&command_lock);
    const bool cancelled = !connected || generation != calibration_generation;
    portEXIT_CRITICAL(&command_lock);
    return cancelled;
}

bool mobile_app_calibration_complete(uint64_t generation)
{
    portENTER_CRITICAL(&command_lock);
    const bool current = connected && generation == calibration_generation;
    if (current)
    {
        global_status_bar_screen_telescope = true;
    }
    portEXIT_CRITICAL(&command_lock);
    return current;
}

bool mobile_app_manual_control_begin(void)
{
    portENTER_CRITICAL(&command_lock);
    manual_control_active = connected;
    manual_connection_generation = connection_generation;
    portEXIT_CRITICAL(&command_lock);
    return manual_control_active;
}

void mobile_app_manual_control_end(void)
{
    manual_control_active = false;
}

bool mobile_app_manual_control_cancelled(void)
{
    portENTER_CRITICAL(&command_lock);
    bool cancelled = manual_control_active && (!connected || manual_connection_generation != connection_generation);
    portEXIT_CRITICAL(&command_lock);
    return cancelled;
}

void mobile_app_joystick_watchdog_begin(void)
{
    portENTER_CRITICAL(&command_lock);
    app_joystick_watchdog_enabled =
        connected && manual_control_active && manual_connection_generation == connection_generation;
    // The handler reports READY before entering its manual loop. Preserve a
    // direction received during that handoff, with its original deadline; never
    // revive retained snapshot text or renew it with out-of-scope keep-alives.
    app_joystick_hold_active =
        app_joystick_watchdog_enabled && received_pending && received_app_direction &&
        esp_timer_get_time() - app_joystick_last_keepalive_us < APP_JOYSTICK_KEEPALIVE_TIMEOUT_US;
    portEXIT_CRITICAL(&command_lock);
}

void mobile_app_joystick_watchdog_end(void)
{
    portENTER_CRITICAL(&command_lock);
    app_joystick_watchdog_enabled = false;
    app_joystick_hold_active = false;
    portEXIT_CRITICAL(&command_lock);
}

mobile_app_joystick_hold_state_t mobile_app_joystick_watchdog_poll(void)
{
    portENTER_CRITICAL(&command_lock);
    mobile_app_joystick_hold_state_t state = MOBILE_APP_JOYSTICK_HOLD_EXPIRED;
    if (command_generation != received_command_generation)
    {
        state = MOBILE_APP_JOYSTICK_HOLD_SUPERSEDED;
    }
    else if (app_joystick_watchdog_enabled && app_joystick_hold_active && connected)
    {
        if (esp_timer_get_time() - app_joystick_last_keepalive_us < APP_JOYSTICK_KEEPALIVE_TIMEOUT_US)
            state = MOBILE_APP_JOYSTICK_HOLD_ACTIVE;
        else
            app_joystick_hold_active = false;
    }
    portEXIT_CRITICAL(&command_lock);
    return state;
}

bool mobile_app_joystick_motion_begin(void)
{
    portENTER_CRITICAL(&command_lock);
    const bool active = connected && app_joystick_watchdog_enabled && app_joystick_hold_active &&
                        command_generation == received_command_generation &&
                        esp_timer_get_time() - app_joystick_last_keepalive_us < APP_JOYSTICK_KEEPALIVE_TIMEOUT_US;
    if (active)
    {
        app_joystick_motion_active = true;
        app_joystick_motion_generation = command_generation;
        atomic_store_explicit(&app_joystick_motion_deadline_us,
                              (uint32_t) (app_joystick_last_keepalive_us + APP_JOYSTICK_KEEPALIVE_TIMEOUT_US),
                              memory_order_relaxed);
    }
    portEXIT_CRITICAL(&command_lock);
    return active;
}

void mobile_app_joystick_motion_end(bool timed_out)
{
    portENTER_CRITICAL(&command_lock);
    if (timed_out && app_joystick_motion_active && app_joystick_motion_generation == received_command_generation)
        app_joystick_hold_active = false;
    app_joystick_motion_active = false;
    portEXIT_CRITICAL(&command_lock);
}

bool mobile_app_joystick_watchdog_due(int64_t now_us)
{
    // The active pulse loop observes this continuously. Unsigned subtraction
    // handles the 32-bit microsecond wrap; its 35-minute half range is much
    // longer than a hold deadline. Idle/session checks retain 64-bit timestamps.
    const uint32_t deadline = atomic_load_explicit(&app_joystick_motion_deadline_us, memory_order_relaxed);
    return (uint32_t) ((uint32_t) now_us - deadline) < UINT32_C(0x80000000);
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static bool command_matches(const uint8_t *data, size_t length, const char *text)
{
    const size_t text_length = strlen(text);
    return length >= text_length && memcmp(data, text, text_length) == 0 &&
           (length == text_length || data[text_length] == '\0');
}

static bool app_direction_command(const uint8_t *data, size_t length)
{
    return command_matches(data, length, APP_RX_APP_DIR_UP) || command_matches(data, length, APP_RX_APP_DIR_DOWN) ||
           command_matches(data, length, APP_RX_APP_DIR_LEFT) || command_matches(data, length, APP_RX_APP_DIR_RIGHT) ||
           command_matches(data, length, APP_RX_APP_DIR_ULEFT) ||
           command_matches(data, length, APP_RX_APP_DIR_URIGHT) ||
           command_matches(data, length, APP_RX_APP_DIR_DLEFT) || command_matches(data, length, APP_RX_APP_DIR_DRIGHT);
}
