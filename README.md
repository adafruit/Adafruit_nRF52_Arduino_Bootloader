# Adafruit nRF52832 Arduino Bootloader

This repository contains the bootloader for Adafruit nRF52832 boards (512KB flash, 64KB SRAM).

It is based on nRF52 SDK 11.0.0 using the classic serial and OTA update options, and adds the following additional features:

- Reset into serial boot mode: When the bootloader powers up, it starts in serial bootloader mode for about 1.5s waiting to see if any serial update requests come in. If this delay times out without a valid serial request, it then passes execution to the user binary.
- Adds a special bootloader blinky pattern on the status LED
- Checks the factory reset pin status at startup and clears the device if the pin is GND'ed
- Adds a Device Information Service (DIS) in bootloader mode with Adafruit Industries as the manufacturer, plus some meta data like the SoftDevice family and version so that we can distinguish nRF51 from nRF52 in the Bluefruit LE Connect apps. DIS will report the versions in the following format: `S132 2.0.1, 0.5.0` (SoftDevice Family and Version, Bootloader Version)

Note: The bootloader .hex file gets merged with the SoftDevice .hex file since they are dependent on each other due to the OTA DFU support, and to avoid having to flash multiple binaries or any version conflicts between the two.

# Files

- `feather52_bootloader_v050_s132_v201.hex` - Intel hex file containing the Bluefruit nRF52 Feather bootloader ans S132 (v2.0.1) SD, for use when flashing with Adalink or a JLink
- `metro52_bootloader_v050_s132_v201.hex` - Intel hex file containing the Metro nRF52 bootloader ans S132 (v2.0.1) SD, for use when flashing with Adalink or a JLink

> `.zip` files are used for OTA updates and are provided for testing purposes, but shouldn't be made available to customers since OTA bootloader and SD updates are a high risk operation.
