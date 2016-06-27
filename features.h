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

#ifndef FEATURES_H_
#define FEATURES_H_

#define YES 2
#define NO  1

#ifdef BOARD_STANDARD
#  define FEATURE_HV5812 NO
#  define FEATURE_RGB_BACKLIGHT YES
#  define FEATURE_WS2812_BACKLIGHT NO
#  define FEATURE_LED_BACKLIGHT NO
#  define FEATURE_3_BUTTON_CONTROL NO
#  define FEATURE_ROTARY YES
#  define FEATURE_ALARM_SLIDE_SWITCH YES
#  define FEATURE_MENU_AUDIO_INDICATOR NO
#elif defined(BOARD_MK2)
#  define FEATURE_HV5812 YES
#  define FEATURE_RGB_BACKLIGHT YES
#  define FEATURE_WS2812_BACKLIGHT NO
#  define FEATURE_LED_BACKLIGHT NO
#  define FEATURE_3_BUTTON_CONTROL NO
#  define FEATURE_ROTARY YES
#  define FEATURE_ALARM_SLIDE_SWITCH YES
#  define FEATURE_MENU_AUDIO_INDICATOR NO
#elif defined(BOARD_DIET)
#  define FEATURE_HV5812 YES
#  define FEATURE_RGB_BACKLIGHT NO
#  define FEATURE_WS2812_BACKLIGHT NO
#  define FEATURE_LED_BACKLIGHT YES
#  define FEATURE_3_BUTTON_CONTROL NO
#  define FEATURE_ROTARY YES
#  define FEATURE_ALARM_SLIDE_SWITCH NO
#  define FEATURE_MENU_AUDIO_INDICATOR YES
#elif defined(BOARD_MODULAR)
#  define FEATURE_HV5812 YES
#  define FEATURE_RGB_BACKLIGHT NO
#  define FEATURE_WS2812_BACKLIGHT NO
#  define FEATURE_LED_BACKLIGHT YES
#  define FEATURE_3_BUTTON_CONTROL YES
#  define FEATURE_ROTARY NO
#  define FEATURE_ALARM_SLIDE_SWITCH NO
#  define FEATURE_MENU_AUDIO_INDICATOR YES
#elif defined(BOARD_MK4)
#  define FEATURE_HV5812 YES
#  define FEATURE_RGB_BACKLIGHT NO
#  define FEATURE_WS2812_BACKLIGHT YES
#  define FEATURE_LED_BACKLIGHT NO
#  define FEATURE_3_BUTTON_CONTROL NO
#  define FEATURE_ROTARY YES
#  define FEATURE_ALARM_SLIDE_SWITCH NO
#  define FEATURE_MENU_AUDIO_INDICATOR YES
#endif

#define FEATURE_GPS YES

///////////////////////////////////////////

#if !(defined FEATURE_HV5812) || FEATURE_HV5812 < NO || FEATURE_HV5812 > YES
#  error Must define FEATURE_HV5812 to be YES or NO
#endif

#if FEATURE_HV5812 == YES
#  define HAVE_HV5812
#endif

///////////////////////////////////////////

#if !(defined FEATURE_RGB_BACKLIGHT) || FEATURE_RGB_BACKLIGHT < NO || FEATURE_RGB_BACKLIGHT > YES
#  error Must define FEATURE_RGB_BACKLIGHT to be YES or NO
#endif

#if FEATURE_RGB_BACKLIGHT == YES
#  define HAVE_RGB_BACKLIGHT
#endif

///////////////////////////////////////////

#if !(defined FEATURE_WS2812_BACKLIGHT) || FEATURE_WS2812_BACKLIGHT < NO || FEATURE_WS2812_BACKLIGHT > YES
#  error Must define FEATURE_WS2812_BACKLIGHT to be YES or NO
#endif

#if FEATURE_WS2812_BACKLIGHT == YES
#  define HAVE_WS2812_BACKLIGHT
#endif

///////////////////////////////////////////

#if !(defined FEATURE_MENU_AUDIO_INDICATOR) || FEATURE_MENU_AUDIO_INDICATOR < NO || FEATURE_MENU_AUDIO_INDICATOR > YES
#  error Must define FEATURE_MENU_AUDIO_INDICATOR to be YES or NO
#endif

#if FEATURE_MENU_AUDIO_INDICATOR == YES
#  define HAVE_MENU_AUDIO_INDICATOR
#endif


///////////////////////////////////////////

#if !(defined FEATURE_RGB_BACKLIGHT) || FEATURE_RGB_BACKLIGHT < NO || FEATURE_RGB_BACKLIGHT > YES
#  error Must define FEATURE_RGB_BACKLIGHT to be YES or NO
#endif

#if FEATURE_RGB_BACKLIGHT == YES
#  define HAVE_RGB_BACKLIGHT
#endif

///////////////////////////////////////////

#if !(defined FEATURE_LED_BACKLIGHT) || FEATURE_LED_BACKLIGHT < NO || FEATURE_LED_BACKLIGHT > YES
#  error Must define FEATURE_LED_BACKLIGHT to be YES or NO
#endif

#if FEATURE_LED_BACKLIGHT == YES
#  define HAVE_LED_BACKLIGHT
#endif

#if FEATURE_LED_BACKLIGHT == YES && FEATURE_RGB_BACKLIGHT == YES
#  error Can only have either FEATURE_LED_BACKLIGHT or FEATURE_RGB_BACKLIGHT
#endif

///////////////////////////////////////////

#if !(defined FEATURE_3_BUTTON_CONTROL) || FEATURE_3_BUTTON_CONTROL < NO || FEATURE_3_BUTTON_CONTROL > YES
#  error Must define FEATURE_3_BUTTON_CONTROL to be YES or NO
#endif

#if FEATURE_3_BUTTON_CONTROL == YES
#  define HAVE_3_BUTTON_CONTROL
#endif

///////////////////////////////////////////

#if !(defined FEATURE_ROTARY) || FEATURE_ROTARY < NO || FEATURE_ROTARY > YES
#  error Must define FEATURE_ROTARY to be YES or NO
#endif

#if FEATURE_ROTARY == YES
#  define HAVE_ROTARY
#endif

///////////////////////////////////////////

#if !(defined FEATURE_ALARM_SLIDE_SWITCH) || FEATURE_ALARM_SLIDE_SWITCH < NO || FEATURE_ALARM_SLIDE_SWITCH > YES
#  error Must define FEATURE_ALARM_SLIDE_SWITCH to be YES or NO
#endif

#if FEATURE_ALARM_SLIDE_SWITCH == YES
#  define HAVE_ALARM_SLIDE_SWITCH
#endif

///////////////////////////////////////////

#if !(defined FEATURE_GPS) || FEATURE_GPS < NO || FEATURE_GPS > YES
#  error Must define FEATURE_GPS to be YES or NO
#endif

#if FEATURE_GPS == YES
#  define HAVE_GPS
#endif

#endif // FEATURES_H_

