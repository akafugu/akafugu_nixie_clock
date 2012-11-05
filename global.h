/*
 * The Akafugu Nixie Clock
 * (C) 2011-12 Akafugu Corporation
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

// GLOBAL include file: Should be the first file to include in all source files

#include <Arduino.h>

#include <stdbool.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "direct_pin_read.h"

struct PinMap
{
    // Rotary encoder
    static const uint8_t button = 13;
    // Colon dots
    static const uint8_t dot1 = 5;
    static const uint8_t dot2 = 6;
    // Alarm on/off switch
    static const uint8_t alarm_switch = 10;
    // RTC SQW pin
    static const uint8_t sqw = 3;
    // Piezo
    static const uint8_t piezo = 9;
    // Nixie anodes (digits)
    static const uint8_t digit0 = 2;
    static const uint8_t digit1 = 4;
    static const uint8_t digit2 = 7;
    static const uint8_t digit3 = 8;
    static const uint8_t digit4 = 11;
    static const uint8_t digit5 = 12;
    // K155ID1 nixie driver (must be PC0~PC3)
    static const uint8_t nixie_driver0 = A0;
    static const uint8_t nixie_driver1 = A1;
    static const uint8_t nixie_driver2 = A2;
    static const uint8_t nixie_driver3 = A3;
    
    // HV518 / HV5812
    static const uint8_t data = A0;
    static const uint8_t clock = A2;
    static const uint8_t latch = A1;
    static const uint8_t blank = 9;
};
