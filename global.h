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

// GLOBAL include file: Should be the first file to include in all source files
#ifndef GLOBAL_H_
#define GLOBAL_H_

// define the type of board we are compiling for
// The Akafugu Nixie Clock (http://www.akafugu.jp/posts/products/nixie/)
//#define BOARD_STANDARD
// Diet Nixie board (http://www.akafugu.jp/posts/products/diet_nixie/)
//#define BOARD_DIET
// The Akafugu Nixie Clock mk2 (http://www.akafugu.jp/posts/products/nixie/)
//#define BOARD_MK2
// Nixie Modular Clock
#define BOARD_MODULAR
#define MODULAR_4D
//#define MODULAR_6D

#if defined(BOARD_STANDARD) && defined(BOARD_DIET)
#error Only one board type can be defined (either BOARD_STANDARD, BOARD_DIET or BOARD_MK2)
#endif

#if defined(BOARD_STANDARD) && defined(BOARD_MK2)
#error Only one board type can be defined (either BOARD_STANDARD, BOARD_DIET or BOARD_MK2)
#endif

#if defined(BOARD_DIET) && defined(BOARD_MK2)
#error Only one board type can be defined (either BOARD_STANDARD, BOARD_DIET or BOARD_MK2)
#endif

#if !defined(BOARD_STANDARD) && !defined(BOARD_DIET) && !defined(BOARD_MK2) && !defined(BOARD_MODULAR)
#error No board type defined (must defined either BOARD_STANDARD or BOARD_DIET)
#endif

#include <Arduino.h>
#include "features.h"

#include <stdbool.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "direct_pin_read.h"

// display state
typedef enum {
  // basic states
  STATE_CLOCK = 0,    // shows hh:mm(:ss) / :ss depending on g_display_mode
  STATE_SHOW_ALARM,   // shows current alarm for a brief time
  STATE_ALARMING,     // alarm is going off
  STATE_SET_ALARM_HH, // set alarm, hours
  STATE_SET_ALARM_MM, // set alarm, minutes
  STATE_SET_CLOCK_HH, // set clock, hours 
  STATE_SET_CLOCK_MM, // set clock, minutes 
  STATE_SET_CLOCK_SS, // set clock, seconds
  STATE_TEST_MODE,    // test mode for testing that all digits work
  STATE_ANTIPOISON,   // cathode anti-poisoning mode

  // menu
  STATE_MENU_24H,           // 1 - 12/24
  STATE_MENU_DOTS,          // 2 - 0(off), 1(on), 2(blink)
  STATE_MENU_LEADING_ZEROS, // 3 - 0(off), 1(on)
  STATE_MENU_WAKE_SOUND,    // 4 - 0(off), 1(on)
  STATE_MENU_SCREEN_SAVER,  // 5 - 0(off), 1(on)
  STATE_MENU_ANTI_POISON,   // 6 - 0(off), 1(on)
  
#ifdef HAVE_GPS
  STATE_MENU_GPS,           // 7  - 0(off), 1(on)
  STATE_MENU_GPS_TZH,       // 8  - -13 to 14
  STATE_MENU_GPS_TZM,       // 9  - 0, 15, 30, 45
  STATE_MENU_DST_OFFSET,    // 10 - 0, 1
#endif
  
  STATE_MENU_LAST,
} state_t;

// indicator led/signal definitions
#define INDICATOR_OFF  0
#define INDICATOR_LO   1
#define INDICATOR_MID  2
#define INDICATOR_HIGH 3


#ifdef BOARD_STANDARD

struct PinMap
{
    // Rotary encoder button
    static const int8_t button = 13;
    // Alarm button
    static const int8_t button_alarm = -1;
    // Colon dots
    static const int8_t dot1 = 5;
    static const int8_t dot2 = 6;
    // Alarm on/off switch
    static const int8_t alarm_switch = 10;
    // RTC SQW pin
    static const int8_t sqw = 3;
    // Piezo
    static const int8_t piezo = 9;
    // Nixie anodes (digits)
    static const int8_t digit0 = 2;
    static const int8_t digit1 = 4;
    static const int8_t digit2 = 7;
    static const int8_t digit3 = 8;
    static const int8_t digit4 = 11;
    static const int8_t digit5 = 12;
    // K155ID1 nixie driver (must be PC0~PC3)
    static const int8_t nixie_driver0 = A0;
    static const int8_t nixie_driver1 = A1;
    static const int8_t nixie_driver2 = A2;
    static const int8_t nixie_driver3 = A3;
    
    // HV518 / HV5812
    static const int8_t data = -1;
    static const int8_t clock = -1;
    static const int8_t latch = -1;
    static const int8_t blank = -1;
};

#elif defined(BOARD_DIET)

// diet nixie pinmap
struct PinMap
{
    // Rotary encoder button
    static const int8_t button = 13;
    // Alarm button
    static const int8_t button_alarm = 12;
    // Colon dot
    static const int8_t dot = 7;
    // Alarm dot
    static const int8_t alarm_dot = 8;
    // Backlight
    static const int8_t backlight_1 = 5;
    static const int8_t backlight_2 = 6;
    static const int8_t backlight_3 = -1;
    // RTC SQW pin
    static const int8_t sqw = 3;
    // Piezo
    static const int8_t piezo = 9;
    // Nixie anodes (digits)
    static const int8_t digit0 = 2;
    static const int8_t digit1 = 4;    
    // HV518 / HV5812
    static const int8_t data = A0;
    static const int8_t clock = A1;
    static const int8_t latch = A2;
    static const int8_t blank = 10;
};

#elif defined(BOARD_MODULAR)

struct PinMap
{
    // Rotary encoder button
    static const int8_t button = -1;
    // Alarm button
    static const int8_t button_alarm = 12;
    // Colon dot
    static const int8_t dot1 =  7;
    static const int8_t dot2 = A3;
    // Alarm dot
    static const int8_t alarm_dot = 8;
    // Backlight
    static const int8_t backlight_1 = 5;
    static const int8_t backlight_2 = 6;
    static const int8_t backlight_3 = 10;
    // RTC SQW pin
    static const int8_t sqw = 3;
    // Piezo
    static const int8_t piezo = 9;
    // Nixie anodes (digits)
    static const int8_t digit0 = 2;
    static const int8_t digit1 = 4;    
    static const int8_t digit2 = 11;    
    // HV518 / HV5812
    static const int8_t data = A0;
    static const int8_t clock = A1;
    static const int8_t latch = A2;
    static const int8_t blank = 13;
};


#elif defined(BOARD_MK2)

// The Akafugu Nixie Clock mk2 pinmap
struct PinMap
{
    // Rotary encoder
    static const int8_t button = 13;
    // Alarm button
    static const int8_t button_alarm = -1;
    // Colon dots (LED)
    static const int8_t dots = 8;
    // Alarm on/off button
    static const int8_t alarm_switch = 10;
    // RTC SQW pin
    static const int8_t sqw = 3;
    // Piezo
    static const int8_t piezo = 9;
    // Nixie anodes (digits)
    static const int8_t digit0 = 2;
    static const int8_t digit1 = 4;    
    // HV518 / HV5812
    static const int8_t data = A0;
    static const int8_t clock = A1;
    static const int8_t latch = A2;
    static const int8_t blank = 11;
};

#endif // board type

#endif //　GLOBAL_H_

