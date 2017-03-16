# Adafruit nRF52 Arduino Bootloader

This repository contains the bootloader for the Adafruit nRF52 Feather boards.

It is based on nRF52 SDK 11.0.0 using the classic serial and OTA update options, and adds the following additional features:

- Checks the factory reset pin status at startup and clears the device if the pin is GND'ed
- Adds a Device Information Service (DIS) in bootloader mode with Adafruit Industries as the manufacturer, plus some meta data like the SoftDevice family and version so that we can distinguish nRF51 from nRF52 in the Bluefruit LE Connect apps.
