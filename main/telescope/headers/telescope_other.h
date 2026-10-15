/**
 * @file    telescope_other.h
 * @brief   Direction calibration, skybox assist, and status utility interface.
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

#ifndef TELESCOPE_OTHER_H
#define TELESCOPE_OTHER_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"
#include <stddef.h>

/* Global Defines --------------------------------------------------------------------------------------------------- */

/**
 * @brief   Number of supported solar-system visibility results.
 */
#define SOLAR_OBJECT_COUNT 9u

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Celestial catalogue entry with equatorial coordinates, type, and magnitude.
 */
typedef struct
{
    const char *name;  /**< Object name. */
    float ra_hours;    /**< Right ascension hour component. */
    float ra_minutes;  /**< Right ascension minute component. */
    float ra_seconds;  /**< Right ascension second component. */
    float dec_degrees; /**< Declination degree component. */
    float dec_minutes; /**< Declination arcminute component. */
    float dec_seconds; /**< Declination arcsecond component. */
    char type[25];     /**< Object category text. */
    float magnitude;   /**< Apparent magnitude. */
} CelestialObject;

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Solar-system catalogue with placeholder coordinates.
 */
extern CelestialObject object_db_planets[];

/**
 * @brief   Number of entries in the solar-system catalogue.
 */
extern int PLANETS_DATABASE_SIZE;

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Calibrate motor directions through synchronous app handshakes.
 * Waits have no time limit. Cancellation, invalid answers or failed moves discard local
 * candidates without saving. Save failure preserves runtime flags and reports failure.
 * Only two valid answers and a successful save publish flags and calibrated status.
 * An in-progress finite motor move returns before cancellation.
 * @return  None.
 */
void calibrate_direction_flags(void);

/**
 * @brief   Apply STEPPER_1_ENABLED and STEPPER_2_ENABLED to the motor enable pins.
 * @return  None.
 */
void update_stepper_enable_state();

/**
 * @brief   Run a spiral sky scan until it finishes or is stopped.
 * Aligned scans stop before a boundary-crossing altitude increment, or upon reaching 0/90 degrees.
 * Unaligned scans retain unrestricted altitude movement.
 * @param   magnification Magnification option: 2 medium, 3 high, otherwise low.
 * @param   speed Speed option: 2 normal, 3 fast, otherwise slow.
 * @return  None.
 */
void sky_box_assists(int magnification, int speed);

/**
 * @brief   Clear the busy flag and send repeated READY notifications.
 * @param   repeat Number of notifications to send, with a 20 ms delay after each.
 * @return  None.
 */
void send_ready_status(int repeat);

/**
 * @brief   Store visibility for Mercury, Venus, Mars, Jupiter, Saturn, Uranus, Neptune, Moon, and Sun.
 * @param   visible_objects_array Output array with at least SOLAR_OBJECT_COUNT elements;
 *          failed calculations remain false.
 * @return  None.
 */
void fetch_solar_objects_visibility_status(bool *visible_objects_array);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_OTHER_H
