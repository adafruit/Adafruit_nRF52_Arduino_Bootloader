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
#include "feather52.h"


#define LED_STATUS_PIN        LED_1
#define LED_CONNECTION_PIN    LED_2

#define led_on(pin)           nrf_gpio_pin_write(pin, LED_STATE_ON)
#define led_off(pin)          nrf_gpio_pin_write(pin, 1-LED_STATE_ON)

#endif
