/**
 * @file    telescope_alignment.h
 * @brief   Star alignment calibration and pointing correction interface.
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

#ifndef TELESCOPE_ALIGNMENT_H
#define TELESCOPE_ALIGNMENT_H

/* Includes --------------------------------------------------------------------------------------------------------- */

#include "config.h"

/* Global Defines --------------------------------------------------------------------------------------------------- */

/* Global macros ---------------------------------------------------------------------------------------------------- */

/* Global Types ----------------------------------------------------------------------------------------------------- */

/**
 * @brief   Three-dimensional Cartesian vector.
 */
typedef struct
{
    double x, y, z; /**< Cartesian vector components. */
} Vec3;

/**
 * @brief   Local two-star correction and reference positions.
 */
typedef struct
{
    AltAz real1;         /**< First reference star's true position, in degrees. */
    AltAz real2;         /**< Second reference star's true position, in degrees. */
    double delta_alt;    /**< Measured minus true altitude at the second star, in degrees. */
    double delta_az;     /**< Measured minus true azimuth, wrapped to [-180, 180] degrees. */
    double star_sep_deg; /**< Angular separation of the reference stars, in degrees. */
    bool valid;          /**< Whether the reference stars are separated by more than one degree. */
} TwoStarLocalCalib;

/* Public variables ------------------------------------------------------------------------------------------------- */

/**
 * @brief   Current local two-star pointing correction.
 */
extern TwoStarLocalCalib global_two_star_local;

/* Public Function Prototypes --------------------------------------------------------------------------------------- */

/**
 * @brief   Scale a nonzero vector to unit length.
 * @param   v Nonzero vector to normalize.
 * @return  Unit vector in the same direction.
 */
Vec3 normalize(Vec3 v);

/**
 * @brief   Calculate the dot product of two vectors.
 * @param   a First vector.
 * @param   b Second vector.
 * @return  Scalar dot product.
 */
double dot(Vec3 a, Vec3 b);

/**
 * @brief   Convert horizontal coordinates to an east/north/up unit vector.
 * @param   a Altitude and azimuth, in degrees.
 * @return  Unit vector with east, north and up components.
 */
Vec3 altaz_to_vector(AltAz a);

/**
 * @brief   Build the local correction from two reference stars and the second measured position.
 * @param   real1 First star's true horizontal coordinates, in degrees.
 * @param   real2 Second star's true horizontal coordinates, in degrees.
 * @param   meas2 Second star's measured horizontal coordinates, in degrees.
 * @param   cal Calibration data to fill; valid when star separation exceeds one degree.
 * @return  None.
 */
void compute_two_star_local_calibration(AltAz real1, AltAz real2, AltAz meas2, TwoStarLocalCalib *cal);

/**
 * @brief   Apply the distance-weighted local two-star pointing correction.
 * @param   real_target True target horizontal coordinates, in degrees.
 * @param   cal Local two-star calibration to apply.
 * @return  Corrected coordinates in degrees, or the unchanged target when calibration is invalid.
 */
AltAz apply_two_star_local_goto(AltAz real_target, const TwoStarLocalCalib *cal);

/**
 * @brief   Check for a one-to-one correction safely away from singularity for Stop GoTo recovery.
 * @param   cal Local calibration to check, or NULL.
 * @return  True for an inactive correction or a model satisfying the inverse safety bound.
 */
bool two_star_local_can_invert(const TwoStarLocalCalib *cal);

/**
 * @brief   Recover true horizontal coordinates from a commanded position.
 *
 * Eligible inputs produce the best estimate even if residual verification fails.
 * The residual limit is less than 1/14 microstep at the supported maximum gear ratio.
 * @param   commanded Commanded horizontal coordinates, in degrees.
 * @param   cal Local calibration satisfying the inverse safety check.
 * @param   real Output estimate; unchanged for invalid arguments or an ineligible model.
 * @return  True if the forward residual is at most 0.00001 degrees per axis; otherwise false.
 */
bool invert_two_star_local_goto(AltAz commanded, const TwoStarLocalCalib *cal, AltAz *real);

/**
 * @brief   Refresh sidereal time and set the telescope position from reference RA/Dec.
 * @param   hour Reference right ascension hours.
 * @param   minute Reference right ascension minutes.
 * @param   second Reference right ascension seconds.
 * @param   degree Reference declination degrees, signed when nonzero.
 * @param   degree_minute Reference arcminutes, signed only when degrees are zero.
 * @param   degree_second Reference arcseconds, signed only when degrees and minutes are zero.
 * @return  True after publishing finite coordinates; false leaves the position unchanged.
 */
bool calibrate_to_radec(double hour, double minute, double second, double degree, double degree_minute,
                        double degree_second);

/**
 * @brief   Parse six bounded two-star RA/Dec components separated by whitespace.
 * @param   text Coordinate payload with no trailing non-whitespace input.
 * @param   coordinates Output components; unchanged when parsing fails.
 * @return  True for finite components within the accepted two-star ranges.
 */
bool parse_two_star_coordinates(const char *text, float coordinates[6]);

/**
 * @brief   Run the app-driven two-star calibration from an already centered first star.
 * @param   first_star_ra_h First star's right ascension hours.
 * @param   first_star_ra_m First star's right ascension minutes.
 * @param   first_star_ra_s First star's right ascension seconds.
 * @param   first_star_dec_d First star's declination degrees, signed when nonzero.
 * @param   first_star_dec_m First star's arcminutes, signed only when degrees are zero.
 * @param   first_star_dec_s First star's arcseconds, signed only when degrees and minutes are zero.
 * @return  True only when calibration finishes with a valid model; otherwise false.
 */
bool two_star_calibration(double first_star_ra_h, double first_star_ra_m, double first_star_ra_s,
                          double first_star_dec_d, double first_star_dec_m, double first_star_dec_s);

/* Global Defines --------------------------------------------------------------------------------------------------- */

#endif // TELESCOPE_ALIGNMENT_H
