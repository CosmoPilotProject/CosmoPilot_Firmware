# CosmoPilot Firmware

CosmoPilot is a motorized GoTo and tracking system for Dobsonian telescopes,
developed by the **CosmoPilot Project**. This repository contains the open-source
ESP32-S3 firmware, released under the [MIT License](LICENSE.md).

The firmware controls the altitude and azimuth motors, performs coordinate and
alignment calculations, and communicates with the separate CosmoPilot mobile app
over Bluetooth Low Energy (BLE). The app provides object selection, setup,
calibration, and telescope controls; its catalogs and user interface are separate
from this firmware.

## Firmware capabilities

- GoTo positioning from equatorial coordinates and supported solar-system targets.
- Tracking through altitude/azimuth motor movements.
- One-star and two-star alignment.
- Manual control through the app or an analog joystick, including a configurable
  standalone joystick mode.
- SkyBox Assist for searching around the current pointing position.
- Persistent telescope settings in NVS and an optional OLED status display.

## Supported hardware

- An **ESP32-S3** controller using the **PCB V1.2** or **DIY** wiring configuration.
- Two **TMC2209** stepper drivers in standalone STEP/DIR mode and two NEMA 17
  motors, with mechanical reduction for the telescope axes. The firmware is
  configured for **200 full steps per revolution and 64 microsteps** on each axis.
- An analog joystick with two axes and a push button for physical manual control.
- An optional **SSD1306 128x64 I2C OLED**, address `0x3C`, in non-flipped orientation.
  SPI, 128x32 panels, and flipped operation are not supported by the application.
- Suitable motor power, controller power, and mechanical mounting for the build.

Motor channel 1 is **altitude (ALT)**; channel 2 is **azimuth (AZ)**. Match wiring
to the selected profile in [hw_config.h](main/config/hw_config.h), which defines
all motor and joystick pins. Both profiles use OLED SDA **GPIO18** and SCL
**GPIO17**, with no display reset GPIO. Firmware startup can continue without
an OLED.

## Toolchain and project configuration

Build with **ESP-IDF v5.3.1** for **ESP32-S3**. This is the version used by the
checked-in configuration and documented firmware builds. Follow Espressif's
[ESP32-S3 setup guide for v5.3.1](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/get-started/index.html)
to install ESP-IDF and its compiler, Python environment, CMake, and Ninja tools.

### Select the hardware profile

Edit the single selector near the top of
[main/config/hw_config.h](main/config/hw_config.h):

```c
#define HW_CONFIG_PCB_V1_2 1
#define HW_CONFIG_DIY      2

#define HW_CONFIG HW_CONFIG_PCB_V1_2
```

PCB V1.2 is the default. For DIY wiring, change only the selector line to
`#define HW_CONFIG HW_CONFIG_DIY`, then rebuild and flash. Unsupported selector
values produce a compile-time error. Board selection is made in this header,
not in `menuconfig`.

### Keep the supplied sdkconfig

Use the tracked [sdkconfig](sdkconfig) as the starting configuration. It selects:

| Setting | Checked-in configuration |
| --- | --- |
| Target | ESP32-S3 |
| Flash | 2 MB, DIO, 80 MHz |
| Partition table | ESP-IDF single application, no OTA |
| Bluetooth | Bluedroid with BLE enabled |
| OLED | I2C, 128x64, non-flipped, new I2C driver |
| FreeRTOS tick rate | 100 Hz |
| Serial console | UART0, 115200 baud |

The target is already selected. Keep `sdkconfig` when preparing the build;
[`idf.py set-target`](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-guides/tools/idf-py.html#select-the-target-chip-set-target)
regenerates it from scratch. 

For deliberate configuration changes, run `idf.py menuconfig`. The OLED wrapper
uses the pins from `hw_config.h` and disables the reset pin; the generic
`CONFIG_SDA_GPIO`, `CONFIG_SCL_GPIO`, and `CONFIG_RESET_GPIO` entries in
`sdkconfig` do not define the application's OLED wiring.

## Build

Confirm the active ESP-IDF version, then build:

```sh
idf.py --version
idf.py build
```

The version should report `v5.3.1`. Build output is written to `build/`.


## First use and mobile app

1. Confirm that the wiring matches the selected hardware profile and that the
   motor/driver setup matches the configured steps and microsteps.
2. Connect from the CosmoPilot app to **`CosmoPilot Telescope`**. The firmware uses
   BLE service `0x00FF` and characteristic `0xFF01` for writes and notifications.
3. Set and verify the actual **ALT and AZ gear ratios** in the app. The firmware
   accepts **1 to 200** on each axis; compiled defaults do not calibrate your mount.
4. Complete motor-direction setup and, if using the physical joystick, its
   calibration. Check small manual movements before attempting a GoTo. Adjust
   backlash settings for the mechanism as needed.
5. Synchronize the observing **location, UTC date, and time**, then complete
   one-star or two-star alignment before using GoTo and tracking. Repeat time
   synchronization and alignment after a restart; boot defaults do not establish
   the current time or telescope orientation.

Settings such as gear ratios, directions, and joystick calibration are stored in
NVS and loaded at startup. Alignment is runtime state. Manual altitude limits
depend on successful alignment and are not active before or during calibration.


## Project, contact, and license

- **Project:** CosmoPilot Project
- **Website:** [https://cosmo-pilot.com](https://cosmo-pilot.com)
- **Contact:** [info@cosmo-pilot.com](mailto:info@cosmo-pilot.com)
- **License:** [MIT License](LICENSE.md)

Third-party source files retain their original copyright and license notices.
The SSD1306 driver's MIT license, Apache-2.0 font-port license, and public-domain
font attribution are documented in the [component notices](main/display/ssd1306/NOTICE.md).
