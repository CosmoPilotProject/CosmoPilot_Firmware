# SSD1306 third-party notices

## Driver and component configuration

The driver and component configuration originate from
[nopnop2002/esp-idf-ssd1306](https://github.com/nopnop2002/esp-idf-ssd1306/tree/master/components/ssd1306).

Copyright (c) 2022 nopnop2002.

They are distributed under the MIT license. The complete upstream text is in
[LICENSE](LICENSE). CosmoPilot adaptations do not replace the upstream notices.

## Font attribution

`font8x8_basic.h` was included through the same upstream component and retains
its existing author, creation-date, source and conversion-code comments.

- The original [font8x8 data](https://github.com/dhepper/font8x8/blob/master/font8x8_basic.h)
  is declared public domain. It credits Daniel Hepper and Marcel Sondaar and
  identifies IBM VGA fonts as its basis.
- The inherited SSD1306 font port and conversion material come from
  [yanbe/ssd1306-esp-idf-i2c](https://github.com/yanbe/ssd1306-esp-idf-i2c/blob/master/main/font8x8_basic.h),
  distributed under Apache License 2.0. Its complete license is preserved in
  [LICENSE-APACHE-2.0](LICENSE-APACHE-2.0). That upstream project has no separate
  NOTICE file.
- The nopnop2002 version adds the up/down arrow glyphs and static storage
  declaration. Its additions retain the MIT notice above.

CosmoPilot added licensing/provenance comments to the font header; the font data
and executable declarations were not changed by that update.

These third-party terms apply to their respective material; the project-level
CosmoPilot MIT notice does not replace them. Retain these license files and
attributions when redistributing the component, including with binary releases.
