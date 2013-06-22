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

#include "backlight.h"

static uint8_t saved_mode;
extern volatile uint8_t g_digits;

#if defined(HAVE_RGB_BACKLIGHT)

#include "rgbled.h"

void fade_in_backlight(int ch)
{
  for (uint16_t i = 0; i <= 4000; i+=10) {
    pca9685_set_channel(ch, i);
    delay(1);
  }
}

void test_backlight()
{
  fade_in_backlight(0);
  delay(1000);
  
  for (uint8_t i = 1; i < 16; i++)
  {
    pca9685_set_channel(i-1, 0);
    fade_in_backlight(i);
    delay(1000);
  }
}

void init_backlight()
{
    pca9685_wake();
    
    // Turn off all LEDs
    for (uint8_t i = 0; i < 16; i++)
        pca9685_set_channel(i, 0);
}

void set_backlight_mode(uint8_t mode)
{
    saved_mode = mode;
    set_rgb_mode(mode);
}

void set_backlight_hh()
{
    set_rgb_mode(0);
    set_rgb_ch(0, 1000, 1000, 1000);
    set_rgb_ch(1, 1000, 1000, 1000);
}

void set_backlight_mm()
{
    set_rgb_mode(0);
    set_rgb_ch(2, 1000, 1000, 1000);
    set_rgb_ch(3, 1000, 1000, 1000);
}

void set_backlight_ss()
{
    set_rgb_mode(0);
    set_rgb_ch(2, 0, 500, 1000);
    set_rgb_ch(3, 0, 500, 1000);
}

void backlight_tick()
{
    rgb_tick();
}

#elif defined(HAVE_LED_BACKLIGHT)

void set_backlight(uint8_t val)
{
   analogWrite(PinMap::backlight_1, val);
   analogWrite(PinMap::backlight_2, val);
   if (PinMap::backlight_3 != -1)
     analogWrite(PinMap::backlight_3, val);
}

void set_backlight(uint8_t b1, uint8_t b2, uint8_t b3)
{
   analogWrite(PinMap::backlight_1, b1);
   analogWrite(PinMap::backlight_2, b2);
   if (PinMap::backlight_3 != -1)
     analogWrite(PinMap::backlight_3, b3);
}

void init_backlight()
{
   // backlight
   pinMode(PinMap::backlight_1, OUTPUT);
   pinMode(PinMap::backlight_2, OUTPUT);

   if (PinMap::backlight_3 != -1) {
     pinMode(PinMap::backlight_3, OUTPUT);
   }

   set_backlight(255);
   //analogWrite(PinMap::backlight_3, 255);
}

extern volatile int8_t g_pulse_direction;
extern volatile uint16_t g_pulse_value;
volatile bool g_pulse;

void set_backlight_mode(uint8_t mode)
{
    Serial.print("set_backlight_mode ");
    Serial.println(mode);
  
    saved_mode = mode;
    g_pulse = false;
  
    if (mode == 0) {
       set_backlight(255);
    }
    else if (mode == 1) {
       set_backlight(0);
    }
    else if (mode == 2) {
       set_backlight(100);
    }
    else if (mode == 3) {
       set_backlight(200);
    }
    else if (mode == 4) {
       set_backlight(0);
       g_pulse = true; 
    }
}

void set_backlight_hh()
{
  set_backlight(100, 255, 255);
}

void set_backlight_mm()
{
  set_backlight(255, 100, 255);
}

void set_backlight_ss()
{
  if (g_digits == 6)
    set_backlight(255, 255, 100);
  else
    set_backlight(255, 100, 255);
}

void backlight_tick()
{
    g_pulse_value += g_pulse_direction;

    if (g_pulse) {
      set_backlight(255-g_pulse_value);
      //set_backlight(255-g_pulse_value, 255-g_pulse_value, 255-g_pulse_value);
    }
}

#endif // HAVE_RGB_BACKLIGHT / HAVE_LED_BACKLIGHT

void increment_backlight_mode()
{
    saved_mode++;
    if (saved_mode == BACKLIGHT_MODES)
        saved_mode = 0;
    
    set_backlight_mode(saved_mode);    
}

void push_backlight_mode()
{
  uint8_t temp = saved_mode;
  set_backlight_mode(0);
  saved_mode = temp;  
}

void pop_backlight_mode()
{
  set_backlight_mode(saved_mode);
}

