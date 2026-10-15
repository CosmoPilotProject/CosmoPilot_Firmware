/**
 * @file    display.c
 * @brief   OLED initialization, rendering, and status display implementation.
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

#include <string.h>

/**
 * @brief   Include display bitmap assets in this translation unit.
 */
#define DISPLAY_INCLUDE_ASSETS
#include "display.h"
#undef DISPLAY_INCLUDE_ASSETS

#include "config.h"
#include "ssd1306.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "trace_level_conf.h"

/* Trace ------------------------------------------------------------------------------------------------------------ */

/**
 * @brief   Trace module identifier.
 */
#define TRACE_NAME  DISPLAY

/**
 * @brief   Trace verbosity configured for the display module.
 */
#define TRACE_LEVEL DISP_MAIN_TRACE_LEVEL

#include "trace_conf.h"

/* Private Defines -------------------------------------------------------------------------------------------------- */

/**
 * @brief   Reset pin selection; -1 disables the dedicated reset pin.
 */
#define DISPLAY_RESET_PIN       (-1)

/**
 * @brief   OLED display width in pixels.
 */
#define DISPLAY_WIDTH_PIXELS    128

/**
 * @brief   OLED display height in pixels.
 */
#define DISPLAY_HEIGHT_PIXELS   64

/**
 * @brief   Number of eight-pixel-high display pages.
 */
#define DISPLAY_PAGE_COUNT      8

/**
 * @brief   Number of bytes in a full display bitmap.
 */
#define DISPLAY_BITMAP_SIZE     1024

/**
 * @brief   Combined location, date, and time readiness flags.
 */
#define STATUS_FLAGS_ALL_SET    0x07u

/* Private macros --------------------------------------------------------------------------------------------------- */

/* Private Types ---------------------------------------------------------------------------------------------------- */

/* Private Functions Prototypes ------------------------------------------------------------------------------------- */

/**
 * @brief   Release OLED I2C resources and reset its pins once the handles are released.
 * @return  ESP_OK on success or if already released, otherwise the first cleanup error.
 */
static esp_err_t release_display_resources(void);

/**
 * @brief   Check display presence and attempt resource cleanup when absent.
 * @return  True if the display-present flag is set, otherwise false.
 */
static bool display_is_available(void);

/**
 * @brief   Draw text for the selected application status.
 * @param   message Application status message to display.
 * @return  None.
 */
static void display_status_message(StatusMessage message);

/**
 * @brief   Draw text for the selected application error.
 * @param   error Application error code to display.
 * @return  None.
 */
static void display_error_message(ErrorCode error);

/* Private variables ------------------------------------------------------------------------------------------------ */

/**
 * @brief   OLED device state and I2C resource handles.
 */
static SSD1306_t display_device;

/**
 * @brief   Whether both OLED I2C handles have been released.
 */
static bool display_resources_released = true;

/* Public variables ------------------------------------------------------------------------------------------------- */

/* Public Functions ------------------------------------------------------------------------------------------------- */

esp_err_t init_display(void)
{
    if (!display_resources_released) {
        TRACE_ERR("OLED initialization requested while resources are still active");
        return ESP_ERR_INVALID_STATE;
    }

    TRACE_INF("Initializing OLED on SDA %d, SCL %d", DISPLAY_SDA, DISPLAY_SCL);
    global_display_present = true;

    esp_err_t err = i2c_master_init(
        &display_device,
        DISPLAY_SDA,
        DISPLAY_SCL,
        DISPLAY_RESET_PIN);
    if (err != ESP_OK) {
        TRACE_ERR("OLED I2C initialization failed: %s", esp_err_to_name(err));
        global_display_present = false;
        display_resources_released = false;
        (void)release_display_resources();
        return err;
    }

    display_resources_released = false;
    ssd1306_init(&display_device, DISPLAY_WIDTH_PIXELS, DISPLAY_HEIGHT_PIXELS);

    if (!global_display_present) {
        TRACE_ERR("OLED controller initialization failed");
        (void)release_display_resources();
        return ESP_FAIL;
    }

    ssd1306_clear_screen(&display_device, false);
    if (!global_display_present)
    {
        TRACE_ERR("OLED initial clear failed");
        (void) release_display_resources();
        return ESP_FAIL;
    }

    TRACE_INF("OLED initialized (%dx%d)", DISPLAY_WIDTH_PIXELS, DISPLAY_HEIGHT_PIXELS);

    return ESP_OK;
}

void display_text(const char *str, int page)
{
    if (!display_is_available()) {
        return;
    }

    if (str == NULL) {
        TRACE_ERR("Cannot display a null text string");
        return;
    }

    if ((page < 0) || (page >= DISPLAY_PAGE_COUNT)) {
        TRACE_ERR("Display page %d is outside the valid range", page);
        return;
    }

    size_t text_length = strlen(str);
    if (text_length > 16u) {
        TRACE_WAR("Display text is longer than 16 characters and will be truncated");
        text_length = 16u;
    }

    ssd1306_clear_line(&display_device, page, false);
    ssd1306_display_text(&display_device, page, str, (int)text_length, false);
}

void display_bitmap(const uint8_t *bmp)
{
    if (!display_is_available()) {
        return;
    }

    if (bmp == NULL) {
        TRACE_ERR("Cannot display a null bitmap");
        return;
    }

    ssd1306_set_buffer(&display_device, bmp);
    ssd1306_show_buffer(&display_device);
}

void draw_status_bar(bool show_bluetooth, uint8_t show_loc_date, bool show_telescope)
{
    if (!display_is_available()) {
        return;
    }

    uint8_t merged_bitmap[DISPLAY_BITMAP_SIZE] = {0};

    if (show_telescope) {
        for (size_t i = 0; i < DISPLAY_BITMAP_SIZE; ++i) {
            merged_bitmap[i] |= status_indicator_telescope[i];
        }
    }

    if (show_loc_date == STATUS_FLAGS_ALL_SET) {
        for (size_t i = 0; i < DISPLAY_BITMAP_SIZE; ++i) {
            merged_bitmap[i] |= status_indicator_loc_date[i];
        }
    }

    if (show_bluetooth) {
        for (size_t i = 0; i < DISPLAY_BITMAP_SIZE; ++i) {
            merged_bitmap[i] |= status_indicator_bluetooth[i];
        }
    }

    if (show_bluetooth || show_loc_date || show_telescope) {
        display_bitmap(merged_bitmap);
    } else {
        display_bitmap(status_indicator_line);
    }
}

void startup_animation(void)
{
    if (!display_is_available()) {
        return;
    }

    display_bitmap(logo_bitmap);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ssd1306_clear_screen(&display_device, false);

    draw_status_bar(true, 0x00u, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    draw_status_bar(true, STATUS_FLAGS_ALL_SET, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    draw_status_bar(true, STATUS_FLAGS_ALL_SET, true);
    vTaskDelay(pdMS_TO_TICKS(400));

    ssd1306_clear_screen(&display_device, false);
    draw_status_bar(false, 0x00u, false);

    update_display_screen();
}

void update_display_screen(void)
{
    if (!display_is_available()) {
        return;
    }

    if (global_display_enable) {
        draw_status_bar(
            global_status_bar_screen_bluetooth,
            global_status_bar_screen_loc_date,
            global_status_bar_screen_telescope);
        display_status_message(global_status_message);
    } else {
        ssd1306_clear_screen(&display_device, false);
    }
}

void update_display_ERROR_screen(ErrorCode error)
{
    if (!display_is_available()) {
        while(1) vTaskDelay(pdMS_TO_TICKS(5000));
        return;
    }

    TRACE_ERR("Displaying application error code %d", (int)error);
    draw_status_bar(
        global_status_bar_screen_bluetooth,
        global_status_bar_screen_loc_date,
        global_status_bar_screen_telescope);
    display_error_message(error);

    while(1) vTaskDelay(pdMS_TO_TICKS(5000));
}

/* Private Functions ------------------------------------------------------------------------------------------------ */

static esp_err_t release_display_resources(void)
{
    if (display_resources_released) {
        return ESP_OK;
    }

    esp_err_t first_error = ESP_OK;

    if (display_device._i2c_dev_handle != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(display_device._i2c_dev_handle);
        if (err != ESP_OK) {
            TRACE_WAR("Failed to remove OLED I2C device: %s", esp_err_to_name(err));
            first_error = err;
        } else {
            display_device._i2c_dev_handle = NULL;
        }
    }

    if ((display_device._i2c_dev_handle == NULL) &&
        (display_device._i2c_bus_handle != NULL)) {
        esp_err_t err = i2c_del_master_bus(display_device._i2c_bus_handle);
        if (err != ESP_OK) {
            TRACE_WAR("Failed to delete OLED I2C bus: %s", esp_err_to_name(err));
            if (first_error == ESP_OK) {
                first_error = err;
            }
        } else {
            display_device._i2c_bus_handle = NULL;
        }
    }

    display_resources_released =
        (display_device._i2c_dev_handle == NULL) &&
        (display_device._i2c_bus_handle == NULL);

    if (display_resources_released) {
        esp_err_t err = gpio_reset_pin(DISPLAY_SDA);
        if (err != ESP_OK) {
            TRACE_WAR("Failed to reset OLED SDA pin: %s", esp_err_to_name(err));
            if (first_error == ESP_OK) {
                first_error = err;
            }
        }

        err = gpio_reset_pin(DISPLAY_SCL);
        if (err != ESP_OK) {
            TRACE_WAR("Failed to reset OLED SCL pin: %s", esp_err_to_name(err));
            if (first_error == ESP_OK) {
                first_error = err;
            }
        }
    }

    return first_error;
}

static bool display_is_available(void)
{
    if (global_display_present) {
        return true;
    }

    (void)release_display_resources();
    return false;
}

static void display_status_message(StatusMessage message)
{
    switch (message) {
        case STATUS_TIME_SET:
            display_text("      Time      ", 5);
            display_text("  Synchronized  ", 7);
            break;

        case STATUS_LOCATION_SET:
            display_text("    Location    ", 5);
            display_text("  Synchronized  ", 7);
            break;
        
        case STATUS_OTHER_SETTINGS:
            display_text(" Other Settings ", 5);
            display_text("    Received    ", 7);
            break;

        case STATUS_DATE_SET:
            display_text("      Date      ", 5);
            display_text("  Synchronized  ", 7);
            break;

        case STATUS_AUTO_SYNC:
            display_text("    Auto-sync   ", 5);
            display_text("    Completed   ", 7);
            break;

        case STATUS_ALIGN_1_STAR:
            display_text("  1-star align  ", 5);
            display_text("   Completed    ", 7);
            break;

        case STATUS_ALIGN_2_STAR:
            display_text("  2-star align  ", 5);
            display_text("   Completed    ", 7);
            break;

        case STATUS_GOTO_OBJECT:
            display_text("      GOTO      ", 5);
            display_text("     Object     ", 7);
            break;

        case STATUS_APP_JOYSTICK:
            display_text("      App       ", 5);
            display_text("    Joystick    ", 7);
            break;

        case STATUS_ANALOG_JOYSTICK:
            display_text("     Analog     ", 5);
            display_text("    Joystick    ", 7);
            break;

        case STATUS_SKY_BOX_ASSIST:
            display_text("     SkyBox     ", 5);
            display_text("     Assist     ", 7);
            break;

        case STATUS_STANDALONE_MODE:
            display_text("   Standalone   ", 5);
            display_text("      Mode      ", 7);
            break;

        case STATUS_RELEASE_STEPPERS:
            display_text("    Steppers    ", 5);
            display_text("    Released    ", 7);
            break;

        case STATUS_BT_CONNECTED:
            display_text("    Bluetooth   ", 5);
            display_text("    connected   ", 7);
            break;

        case STATUS_BT_NOT_CONNECTED:
            display_text("    Bluetooth   ", 5);
            display_text("  NOT connected ", 7);
            break;

        case STATUS_SET_GEAR_RATIO:
            display_text("   Gear Ratio   ", 5);
            display_text("     Saved      ", 7);
            break;

        case STATUS_JOYSTICK_CALIB:
            display_text("    Joystick    ", 5);
            display_text("   Calibrated   ", 7);
            break;

        case STATUS_DIRECTION_CALIB:
            display_text("    Direction   ", 5);
            display_text("   Calibrated   ", 7);
            break;

        case STATUS_FACTORY_RESET:
            display_text("     FACTORY    ", 5);
            display_text("      RESET    ", 7);
            break;

        case STATUS_TRACKING_ON:
            display_text("    Tracking    ", 5);
            display_text("     Active     ", 7);
            break;

        case STATUS_IDLE:
            display_text("    Telescope   ", 5);
            display_text("      READY     ", 7);
            break;

        case STATUS_PROCESSING:
            display_text("   Processing   ", 5);
            display_text("      ....      ", 7);
            break;

        case STATUS_BACKLASH:
            display_text(" Backlash Calib ", 5);
            display_text("    Complete    ", 7);
            break;

        case STATUS_HORIZ_LIMIT:
            display_text("Tracking stopped", 5);
            display_text("  Horiz. limit  ", 7);
            break;

        case STATUS_SOLAR_OBJ:
            display_text("  Solar objects ", 5);
            display_text("   Visib. sent  ", 7);
            break;

        case STATUS_STANDALONE_LOW:
            display_text("      LOW       ", 1);
            display_text("   Standalone   ", 5);
            display_text("      Mode      ", 7);
            break;

        case STATUS_STANDALONE_MEDIUM:
            display_text("     MEDIUM     ", 1);
            display_text("   Standalone   ", 5);
            display_text("      Mode      ", 7);
            break;

        case STATUS_STANDALONE_HIGH:
            display_text("      HIGH      ", 1);
            display_text("   Standalone   ", 5);
            display_text("      Mode      ", 7);
            break;

        default:
            display_text("     Unknown    ", 5);
            display_text("     Message    ", 7);
            break;
    }
}

static void display_error_message(ErrorCode error)
{
    switch (error) {
        case ERROR_UNKNOWN_COMMAND_ID:
            display_text("Internal command", 5);
            display_text("     error      ", 7);
            break;

        case ERROR_UNEXPECTED_TERMINATION:
            display_text("  Control loop  ", 5);
            display_text("    stopped     ", 7);
            break;

        case ERROR_INVALID_COMMAND:
            display_text("    Invalid     ", 5);
            display_text("    command     ", 7);
            break;

        case ERROR_NONE:
            display_text("    No error    ", 5);
            display_text("    reported    ", 7);
            break;

        default:
            display_text("    Unknown     ", 5);
            display_text("   error code   ", 7);
            break;
    }
}
