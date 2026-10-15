/**
 * @file    mobile_app_command_buffer.h
 * @brief   Command mailbox and manual-control lifecycle interface.
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

#ifndef MOBILE_APP_COMMAND_BUFFER_H
#define MOBILE_APP_COMMAND_BUFFER_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/**
 * @brief   Command buffer capacity in bytes, including the terminating NUL.
 */
#define MAX_COMMAND_LENGTH 128

/**
 * @brief   Maximum age of an active App Joystick direction or keep-alive.
 */
#define APP_JOYSTICK_KEEPALIVE_TIMEOUT_US INT64_C(3500000)

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

typedef enum
{
    MOBILE_APP_JOYSTICK_HOLD_ACTIVE,
    MOBILE_APP_JOYSTICK_HOLD_EXPIRED,
    MOBILE_APP_JOYSTICK_HOLD_SUPERSEDED,
} mobile_app_joystick_hold_state_t;

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Application-task snapshot of the latest received command.
 */
extern char global_last_ble_command[MAX_COMMAND_LENGTH];
/**
 * @brief   Whether the application-task command snapshot is nonempty.
 */
extern bool global_new_command_available;

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Submit a Bluetooth message, request Stop GoTo, or refresh an active app hold.
 *
 * Exact CMD STOP GOTO and CMD APP KEEPALIVE text bypass the mailbox. Keep-alive only
 * refreshes an unexpired App Joystick hold; it never starts motion or changes READY.
 * CMD CALIB_START immediately invalidates alignment and enters the ordinary mailbox
 * for its READY response.
 * Copies are bounded and NUL-terminated, preserving transmitted/embedded NUL semantics.
 * Runs in the Bluetooth task without allocation or waiting on the application task.
 * @param   data Message bytes; may be NULL only when length is zero.
 * @param   length Number of supplied bytes, truncated to the command buffer capacity minus one.
 * @return  ESP_OK when accepted, or ESP_ERR_INVALID_ARG for NULL data with nonzero length.
 */
esp_err_t mobile_app_command_receive(const uint8_t *data, size_t length);

/**
 * @brief   Refresh the command snapshot from the application task when an arrival is pending.
 * @return  None.
 */
void mobile_app_command_refresh(void);

/**
 * @brief   Clear the application-task snapshot without consuming newer pending arrivals.
 *
 * Application task only.
 * @return  None.
 */
void mobile_app_command_clear(void);

/**
 * @brief   Record a Bluetooth connection from the Bluetooth task.
 * @return  None.
 */
void mobile_app_command_connected(void);

/**
 * @brief   Record a Bluetooth-task disconnect, discarding pending commands and cancelling manual control.
 *
 * Cancellation remains detectable even if reconnection occurs before the next poll.
 * @return  None.
 */
void mobile_app_command_disconnected(void);

/**
 * @brief   Invalidate alignment and capture the current command's calibration generation.
 * @return  Generation to check before movement or publishing calibration success.
 */
uint64_t mobile_app_calibration_begin(void);

/**
 * @brief   Capture the application command's session without invalidating alignment.
 * @return  Generation for joystick/direction calibration and interactive control.
 */
uint64_t mobile_app_calibration_generation(void);

/**
 * @brief   End this alignment without clearing a newer calibration start.
 * @param   generation Generation captured by the alignment handler.
 */
void mobile_app_calibration_end(uint64_t generation);

/**
 * @brief   Check whether this command belongs to an unfinished star alignment.
 * @return  True during first-star centering and the one-/two-star handler.
 */
bool mobile_app_calibration_pending(void);

/**
 * @brief   Check whether a new calibration start or disconnect superseded a calibration.
 * @param   generation Generation captured when calibration began.
 * @return  True when the captured calibration can no longer complete.
 */
bool mobile_app_calibration_cancelled(uint64_t generation);

/**
 * @brief   Publish successful alignment only for the current calibration generation.
 * @param   generation Generation captured when calibration began.
 * @return  True if alignment was published; false if the calibration was superseded.
 */
bool mobile_app_calibration_complete(uint64_t generation);

/**
 * @brief   Begin app-joystick and skybox manual control in the application task.
 *
 * Tracking does not consult this scope's cancellation state.
 * @return  True if Bluetooth is connected and the scope is active; otherwise false.
 */
bool mobile_app_manual_control_begin(void);

/**
 * @brief   End the manual-control scope in the application task.
 * @return  None.
 */
void mobile_app_manual_control_end(void);

/**
 * @brief   Check manual-control cancellation from the application task.
 * @return  True if active manual control lost its original connection; otherwise false.
 */
bool mobile_app_manual_control_cancelled(void);

/**
 * @brief   Enable the App Joystick watchdog, retaining only a fresh pending direction.
 *
 * Application task only, inside an active manual-control connection scope.
 * Suspend with end() around tracking and SkyBox; only explicit direction ingress arms it.
 * A direction pending during the handler READY handoff keeps its original deadline.
 */
void mobile_app_joystick_watchdog_begin(void);

/**
 * @brief   Disable the App Joystick watchdog and discard any active hold.
 */
void mobile_app_joystick_watchdog_end(void);

/**
 * @brief   Check the current directional snapshot before starting its next nudge.
 *
 * An expired hold stays stopped until a fresh explicit direction arrives, even if
 * the same direction text remains in the snapshot. Superseded snapshots must be
 * refreshed without clearing pending commands or resetting direction-change ramps.
 * Active App pulses also enforce the deadline without disturbing their normal timing.
 */
mobile_app_joystick_hold_state_t mobile_app_joystick_watchdog_poll(void);

/**
 * @brief   Capture the current direction's deadline for one App-only motor move.
 * @return  True if the current snapshot still owns an unexpired direction.
 */
bool mobile_app_joystick_motion_begin(void);

/**
 * @brief   Release a finite move's lease, without clearing a newer pending direction.
 * @param   timed_out Whether motor generation stopped at its deadline.
 */
void mobile_app_joystick_motion_end(bool timed_out);

/**
 * @brief   Read an active move's deadline without locking or reading the clock.
 * @param   now_us Existing motor-loop clock sample.
 * @return  True once the captured movement deadline expires.
 */
bool mobile_app_joystick_watchdog_due(int64_t now_us);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // MOBILE_APP_COMMAND_BUFFER_H
