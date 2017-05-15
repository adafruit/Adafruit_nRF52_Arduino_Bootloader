/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"
#include "metro52.h"

// Make sure we have at least two buttons (DFU + FRESET since DFU+FRST=OTA)
#if BUTTONS_NUMBER < 2
#error "At least two buttons required in the BSP (see 'BUTTONS_NUMBER')"
#endif

#define LED_STATUS_PIN        LED_1
#if LEDS_NUMBER > 1
#define LED_CONNECTION_PIN    LED_2
#endif

#define led_on(pin)           nrf_gpio_pin_write(pin, LED_STATE_ON)
#define led_off(pin)          nrf_gpio_pin_write(pin, 1-LED_STATE_ON)

#endif
