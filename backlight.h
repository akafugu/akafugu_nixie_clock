/*
 * The Akafugu Nixie Clock
 * (C) 2012-13 Akafugu Corporation
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

#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_

#include "global.h"

#if defined(HAVE_RGB_BACKLIGHT)
#define BACKLIGHT_MODES 11
#elif defined(HAVE_LED_BACKLIGHT)
#define BACKLIGHT_MODES 5
#endif

void init_backlight();

void set_backlight_mode(uint8_t mode);
void increment_backlight_mode();

void push_backlight_mode();
void pop_backlight_mode();

void set_backlight_hh();
void set_backlight_mm();
void set_backlight_ss();

void backlight_tick();

#endif // BACKLIGHT_H_


