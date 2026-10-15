/**
 * @file    flash.c
 * @brief   NVS initialization and persistent settings storage implementation.
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

#include "flash.h"

#include <math.h>
#include <stddef.h>

#include "config.h"
#include "joystick/analog_joystick.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "trace_level_conf.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier.
 */
#define TRACE_NAME  FLASH
/**
 * @brief   Trace verbosity configured for the flash module.
 */
#define TRACE_LEVEL FLASH_TRACE_LEVEL

#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   NVS namespace for motor directions and observer location.
 */
#define CALIB_NAMESPACE    "calib_ns"

/**
 * @brief   NVS namespace for motor gear ratios.
 */
#define GEAR_NAMESPACE     "gear_r_ns"

/**
 * @brief   NVS namespace for application settings.
 */
#define SETTINGS_NAMESPACE "other_ns"

/**
 * @brief   NVS namespace for joystick calibration.
 */
#define JOYSTICK_NAMESPACE "joystick_ns"

/**
 * @brief   NVS namespace for motor backlash calibration.
 */
#define BACKLASH_NAMESPACE "backlash_ns"

/* Private macros --------------------------------------------------------------------------------------------------- */

/**
 * @brief   Number of entries in a field descriptor array.
 */
#define FIELD_COUNT(fields) (sizeof(fields) / sizeof((fields)[0]))

/* Private Types ---------------------------------------------------------------------------------------------------- */

/**
 * @brief   Storage type of an NVS field.
 */
typedef enum
{
    FLASH_I8,         /**< Signed 8-bit NVS integer. */
    FLASH_U8,         /**< Unsigned 8-bit NVS integer. */
    FLASH_I16,        /**< Signed 16-bit NVS integer. */
    FLASH_FLOAT_BLOB, /**< Float stored as an NVS blob. */
    FLASH_DOUBLE_BLOB /**< Double stored as an NVS blob. */
} flash_value_type_t;

/**
 * @brief   Scalar value read from or written to NVS.
 */
typedef union
{
    int8_t i8;           /**< Signed 8-bit value. */
    uint8_t u8;          /**< Unsigned 8-bit value. */
    int16_t i16;         /**< Signed 16-bit value. */
    float float_value;   /**< Float value. */
    double double_value; /**< Double value. */
} flash_value_t;

/**
 * @brief   NVS key, storage type, value, and optional-key policy.
 */
typedef struct
{
    const char *key;         /**< NVS key name. */
    flash_value_type_t type; /**< NVS storage type. */
    flash_value_t value;     /**< Stored value or supplied default. */
    bool optional;           // A missing optional key leaves its explicitly supplied default intact.
} flash_field_t;

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Write one field using its declared NVS storage type.
 * @param   handle Open NVS handle for writing.
 * @param   field Field key, type, and value to write.
 * @return  ESP_OK on success, otherwise an error code.
 */
static esp_err_t write_field(nvs_handle_t handle, const flash_field_t *field);

/**
 * @brief   Read an NVS blob and require the expected byte count.
 * @param   handle Open NVS handle for reading.
 * @param   key NVS key name.
 * @param   value Destination buffer for the blob.
 * @param   expected_size Required blob size and buffer capacity in bytes.
 * @return  ESP_OK on success, otherwise an error code.
 */
static esp_err_t read_blob(nvs_handle_t handle, const char *key, void *value, size_t expected_size);

/**
 * @brief   Read one field using its declared NVS storage type.
 * @param   handle Open NVS handle for reading.
 * @param   field Field descriptor receiving the stored value.
 * @return  ESP_OK on success, otherwise an error code.
 */
static esp_err_t read_field(nvs_handle_t handle, flash_field_t *field);

/**
 * @brief   Write fields to an NVS namespace and commit after all writes succeed.
 * @param   namespace_name NVS namespace to open.
 * @param   fields Field descriptors and values to save.
 * @param   count Number of field descriptors.
 * @return  ESP_OK on success, otherwise an error code.
 */
static esp_err_t save_fields(const char *namespace_name, const flash_field_t *fields, size_t count);

/**
 * @brief   Read fields from NVS, retaining supplied defaults for missing optional keys.
 * @param   namespace_name NVS namespace to open.
 * @param   fields Field descriptors to update; earlier reads remain applied on failure.
 * @param   count Number of field descriptors.
 * @return  ESP_OK on success, otherwise an error code.
 */
static esp_err_t load_fields(const char *namespace_name, flash_field_t *fields, size_t count);

/**
 * @brief   Trace an invalid argument error for the named function.
 * @param   function_name Name of the function reporting the error.
 * @return  ESP_ERR_INVALID_ARG.
 */
static esp_err_t invalid_argument(const char *function_name);

/**
 * @brief   Validate both gear ratios using the existing application limits.
 * @param   gear_ratio1 Altitude gear ratio.
 * @param   gear_ratio2 Azimuth gear ratio.
 * @return  ESP_OK for valid ratios, otherwise ESP_ERR_INVALID_ARG with a trace.
 */
static esp_err_t validate_gear_ratios(double gear_ratio1, double gear_ratio2);

/**
 * @brief   Validate numeric settings before saving or applying a loaded group.
 * @param   magnification Skybox magnification code, including the zero default.
 * @param   speed Skybox speed code, including the zero default.
 * @param   max_speed Maximum telescope speed in degrees per second.
 * @param   track_refr Tracking refresh interval in seconds.
 * @return  ESP_OK for valid settings, otherwise ESP_ERR_INVALID_ARG with a trace.
 */
static esp_err_t validate_other_settings(int magnification, int speed, float max_speed, float track_refr);

/* Private variables ------------------------------------------------------------------------------------------------ */

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

esp_err_t flash_init(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        TRACE_WAR("NVS initialization failed (%s); erasing partition for recovery", esp_err_to_name(err));
        // Preserve the original fatal policy if this recovery erase fails.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        TRACE_ERR("Cannot initialize NVS: %s", esp_err_to_name(err));
    }
    else
    {
        TRACE_INF("NVS initialized");
    }
    return err;
}

esp_err_t flash_erase_all(void)
{
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK)
    {
        TRACE_ERR("Cannot erase NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Factory reset previously reinitialized once, without the startup recovery retry.
    err = nvs_flash_init();
    if (err != ESP_OK)
    {
        TRACE_ERR("Cannot reinitialize NVS after erase: %s", esp_err_to_name(err));
    }
    else
    {
        TRACE_INF("NVS erased and reinitialized");
    }
    return err;
}

esp_err_t save_directions_to_flash(bool direction_1, bool direction_2)
{
    const flash_field_t fields[] = {
        {.key = "DIRECTION_1", .type = FLASH_I8, .value.i8 = direction_1 ? 1 : 0},
        {.key = "DIRECTION_2", .type = FLASH_I8, .value.i8 = direction_2 ? 1 : 0}
    };
    return save_fields(CALIB_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_directions_from_flash(bool *direction_1, bool *direction_2)
{
    if ((direction_1 == NULL) || (direction_2 == NULL))
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "DIRECTION_1", .type = FLASH_I8},
        {.key = "DIRECTION_2", .type = FLASH_I8}
    };
    esp_err_t err = load_fields(CALIB_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        // Preserve the original == 1 conversion (not a generic nonzero test).
        *direction_1 = fields[0].value.i8 == 1;
        *direction_2 = fields[1].value.i8 == 1;
    }
    return err;
}

esp_err_t save_location_to_flash(double longitude, double latitude)
{
    const flash_field_t fields[] = {
        {.key = "longitude", .type = FLASH_DOUBLE_BLOB, .value.double_value = longitude},
        {.key = "latitude",  .type = FLASH_DOUBLE_BLOB, .value.double_value = latitude }
    };
    return save_fields(CALIB_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_location_from_flash(double *longitude, double *latitude)
{
    if ((longitude == NULL) || (latitude == NULL))
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "longitude", .type = FLASH_DOUBLE_BLOB},
        {.key = "latitude",  .type = FLASH_DOUBLE_BLOB}
    };
    esp_err_t err = load_fields(CALIB_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        *longitude = fields[0].value.double_value;
        *latitude = fields[1].value.double_value;
    }
    return err;
}

esp_err_t save_gear_ratio_to_flash(double gear_ratio1, double gear_ratio2)
{
    esp_err_t err = validate_gear_ratios(gear_ratio1, gear_ratio2);
    if (err != ESP_OK)
    {
        return err;
    }
    const flash_field_t fields[] = {
        {.key = "gear_ratio1", .type = FLASH_DOUBLE_BLOB, .value.double_value = gear_ratio1},
        {.key = "gear_ratio2", .type = FLASH_DOUBLE_BLOB, .value.double_value = gear_ratio2}
    };
    return save_fields(GEAR_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_gear_ratio_from_flash(double *gear_ratio1, double *gear_ratio2)
{
    if ((gear_ratio1 == NULL) || (gear_ratio2 == NULL))
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "gear_ratio1", .type = FLASH_DOUBLE_BLOB},
        {.key = "gear_ratio2", .type = FLASH_DOUBLE_BLOB}
    };
    esp_err_t err = load_fields(GEAR_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        err = validate_gear_ratios(fields[0].value.double_value, fields[1].value.double_value);
    }
    if (err == ESP_OK)
    {
        *gear_ratio1 = fields[0].value.double_value;
        *gear_ratio2 = fields[1].value.double_value;
    }
    return err;
}

esp_err_t save_other_settings_to_flash(bool display_setting, int skybox_magnification, int skybox_speed, bool standalone_mode, float max_speed, float track_refr)
{
    esp_err_t err = validate_other_settings(skybox_magnification, skybox_speed, max_speed, track_refr);
    if (err != ESP_OK)
    {
        return err;
    }
    const flash_field_t fields[] = {
        {.key = "display",    .type = FLASH_U8,         .value.u8 = (uint8_t)display_setting     },
        {.key = "magnif",     .type = FLASH_U8,         .value.u8 = (uint8_t)skybox_magnification},
        {.key = "speed",      .type = FLASH_U8,         .value.u8 = (uint8_t)skybox_speed        },
        {.key = "standalone", .type = FLASH_U8,         .value.u8 = (uint8_t)standalone_mode     },
        {.key = "max_speed",  .type = FLASH_FLOAT_BLOB, .value.float_value = max_speed           },
        {.key = "track_refr", .type = FLASH_FLOAT_BLOB, .value.float_value = track_refr          }
    };
    return save_fields(SETTINGS_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_other_settings_from_flash(bool *display_setting, int *skybox_magnification, int *skybox_speed, bool *standalone_mode, float *max_speed, float *track_refr)
{
    if ((display_setting == NULL) || (skybox_magnification == NULL) || (skybox_speed == NULL) || (standalone_mode == NULL) || (max_speed == NULL) || (track_refr == NULL))
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "display", .type = FLASH_U8},
        {.key = "magnif", .type = FLASH_U8},
        {.key = "speed", .type = FLASH_U8},
        {.key = "standalone", .type = FLASH_U8},
        {.key = "max_speed", .type = FLASH_FLOAT_BLOB, .value.float_value = 4.0f, .optional = true},
        {.key = "track_refr", .type = FLASH_FLOAT_BLOB, .value.float_value = 3.0f, .optional = true}
    };
    esp_err_t err = load_fields(SETTINGS_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        err = validate_other_settings(fields[1].value.u8, fields[2].value.u8, fields[4].value.float_value,
                                      fields[5].value.float_value);
    }
    if (err == ESP_OK)
    {
        *display_setting = fields[0].value.u8 != 0;
        *skybox_magnification = fields[1].value.u8;
        *skybox_speed = fields[2].value.u8;
        *standalone_mode = fields[3].value.u8 != 0;
        *max_speed = fields[4].value.float_value;
        *track_refr = fields[5].value.float_value;
    }
    return err;
}

esp_err_t save_joystick_calibration(const JoystickCalibration *calib)
{
    if (calib == NULL)
    {
        return invalid_argument(__func__);
    }
    const flash_field_t fields[] = {
        {.key = "x_idle", .type = FLASH_I16, .value.i16 = calib->x_idle},
        {.key = "x_min",  .type = FLASH_I16, .value.i16 = calib->x_min },
        {.key = "x_max",  .type = FLASH_I16, .value.i16 = calib->x_max },
        {.key = "y_idle", .type = FLASH_I16, .value.i16 = calib->y_idle},
        {.key = "y_min",  .type = FLASH_I16, .value.i16 = calib->y_min },
        {.key = "y_max",  .type = FLASH_I16, .value.i16 = calib->y_max }
    };
    return save_fields(JOYSTICK_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_joystick_calibration(JoystickCalibration *calib)
{
    if (calib == NULL)
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "x_idle", .type = FLASH_I16},
        {.key = "x_min",  .type = FLASH_I16},
        {.key = "x_max",  .type = FLASH_I16},
        {.key = "y_idle", .type = FLASH_I16},
        {.key = "y_min",  .type = FLASH_I16},
        {.key = "y_max",  .type = FLASH_I16}
    };
    esp_err_t err = load_fields(JOYSTICK_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        calib->x_idle = fields[0].value.i16;
        calib->x_min = fields[1].value.i16;
        calib->x_max = fields[2].value.i16;
        calib->y_idle = fields[3].value.i16;
        calib->y_min = fields[4].value.i16;
        calib->y_max = fields[5].value.i16;
    }
    return err;
}

esp_err_t save_backlash_calibration_to_flash(int16_t *backlash_steps_az, int16_t *backlash_steps_alt)
{
    if ((backlash_steps_az == NULL) || (backlash_steps_alt == NULL))
    {
        return invalid_argument(__func__);
    }
    const flash_field_t fields[] = {
        {.key = "backlash_az",  .type = FLASH_I16, .value.i16 = *backlash_steps_az },
        {.key = "backlash_alt", .type = FLASH_I16, .value.i16 = *backlash_steps_alt}
    };
    return save_fields(BACKLASH_NAMESPACE, fields, FIELD_COUNT(fields));
}

esp_err_t load_backlash_calibration_from_flash(int16_t *out_az, int16_t *out_alt)
{
    if ((out_az == NULL) || (out_alt == NULL))
    {
        return invalid_argument(__func__);
    }
    flash_field_t fields[] = {
        {.key = "backlash_az",  .type = FLASH_I16},
        {.key = "backlash_alt", .type = FLASH_I16}
    };
    esp_err_t err = load_fields(BACKLASH_NAMESPACE, fields, FIELD_COUNT(fields));
    if (err == ESP_OK)
    {
        *out_az = fields[0].value.i16;
        *out_alt = fields[1].value.i16;
    }
    return err;
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static esp_err_t validate_gear_ratios(double gear_ratio1, double gear_ratio2)
{
    if (!isfinite(gear_ratio1) || gear_ratio1 < GEAR_RATIO_MIN || gear_ratio1 > GEAR_RATIO_MAX ||
        !isfinite(gear_ratio2) || gear_ratio2 < GEAR_RATIO_MIN || gear_ratio2 > GEAR_RATIO_MAX)
    {
        TRACE_ERR("Rejected invalid gear settings: ALT=%g AZ=%g; expected %g..%g", gear_ratio1, gear_ratio2,
                  GEAR_RATIO_MIN, GEAR_RATIO_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t validate_other_settings(int magnification, int speed, float max_speed, float track_refr)
{
    // Zero is the existing unselected/default code; explicit choices are 1, 2 and 3.
    if (magnification < 0 || magnification > 3 || speed < 0 || speed > 3)
    {
        TRACE_ERR("Rejected invalid Skybox settings: magnification=%d speed=%d; expected 0..3", magnification, speed);
        return ESP_ERR_INVALID_ARG;
    }
    // Match the existing handler exactly, including its double-precision 1.9 bound.
    if (!isfinite(max_speed) || max_speed < 1.9 || max_speed > 8)
    {
        TRACE_ERR("Rejected invalid maximum speed setting: %g", (double) max_speed);
        return ESP_ERR_INVALID_ARG;
    }
    if (!isfinite(track_refr) || track_refr < 0.04f || track_refr > 30.0f)
    {
        TRACE_ERR("Rejected invalid tracking refresh setting: %g", (double) track_refr);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t write_field(nvs_handle_t handle, const flash_field_t *field)
{
    switch (field->type)
    {
        case FLASH_I8:
            return nvs_set_i8(handle, field->key, field->value.i8);
        case FLASH_U8:
            return nvs_set_u8(handle, field->key, field->value.u8);
        case FLASH_I16:
            return nvs_set_i16(handle, field->key, field->value.i16);
        case FLASH_FLOAT_BLOB:
            return nvs_set_blob(handle, field->key, &field->value.float_value, sizeof(field->value.float_value));
        case FLASH_DOUBLE_BLOB:
            return nvs_set_blob(handle, field->key, &field->value.double_value, sizeof(field->value.double_value));
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t read_blob(nvs_handle_t handle, const char *key, void *value, size_t expected_size)
{
    size_t size = expected_size;
    esp_err_t err = nvs_get_blob(handle, key, value, &size);
    // NVS also returns ESP_OK for a shorter blob; that is not a complete scalar.
    if ((err == ESP_OK) && (size != expected_size))
    {
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    return err;
}

static esp_err_t read_field(nvs_handle_t handle, flash_field_t *field)
{
    switch (field->type)
    {
        case FLASH_I8:
            return nvs_get_i8(handle, field->key, &field->value.i8);
        case FLASH_U8:
            return nvs_get_u8(handle, field->key, &field->value.u8);
        case FLASH_I16:
            return nvs_get_i16(handle, field->key, &field->value.i16);
        case FLASH_FLOAT_BLOB:
            return read_blob(handle, field->key, &field->value.float_value, sizeof(field->value.float_value));
        case FLASH_DOUBLE_BLOB:
            return read_blob(handle, field->key, &field->value.double_value, sizeof(field->value.double_value));
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t save_fields(const char *namespace_name, const flash_field_t *fields, size_t count)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(namespace_name, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        TRACE_ERR("Cannot open %s for writing: %s", namespace_name, esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < count; ++i)
    {
        err = write_field(handle, &fields[i]);
        if (err != ESP_OK)
        {
            TRACE_ERR("Cannot write %s/%s: %s", namespace_name, fields[i].key, esp_err_to_name(err));
            break;
        }
    }

    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
        if (err != ESP_OK)
        {
            TRACE_ERR("Cannot commit %s: %s", namespace_name, esp_err_to_name(err));
        }
    }

    nvs_close(handle);
    if (err == ESP_OK)
    {
        TRACE_DEB("Saved %s", namespace_name);
    }
    return err;
}

static esp_err_t load_fields(const char *namespace_name, flash_field_t *fields, size_t count)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(namespace_name, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            TRACE_WAR("No saved settings in %s; keeping current values", namespace_name);
        }
        else
        {
            TRACE_ERR("Cannot open %s for reading: %s", namespace_name, esp_err_to_name(err));
        }
        return err;
    }

    for (size_t i = 0; i < count; ++i)
    {
        err = read_field(handle, &fields[i]);
        if ((err == ESP_ERR_NVS_NOT_FOUND) && fields[i].optional)
        {
            TRACE_DEB("Missing %s/%s; using legacy default", namespace_name, fields[i].key);
            err = ESP_OK;
            continue;
        }
        if (err != ESP_OK)
        {
            if (err == ESP_ERR_NVS_NOT_FOUND)
            {
                TRACE_WAR("Missing %s/%s; keeping current values", namespace_name, fields[i].key);
            }
            else
            {
                TRACE_ERR("Cannot read %s/%s: %s", namespace_name, fields[i].key, esp_err_to_name(err));
            }
            break;
        }
    }

    nvs_close(handle);
    return err;
}

static esp_err_t invalid_argument(const char *function_name)
{
    TRACE_ERR("Invalid argument to %s", function_name);
    return ESP_ERR_INVALID_ARG;
}
