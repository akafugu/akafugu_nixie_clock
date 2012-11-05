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
    
    // HV518 / HV5812
    static const uint8_t data = A0;
    static const uint8_t clock = A2;
    static const uint8_t latch = A1;
    static const uint8_t blank = 9;
};
