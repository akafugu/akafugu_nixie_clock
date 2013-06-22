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

#include "global.h"

#ifdef BOARD_STANDARD

#include "rgbled.h"

extern uint8_t display_on;
extern uint8_t digits;
extern volatile uint8_t data[6];

extern volatile uint8_t g_blank;
extern volatile bool g_blink_on;

extern state_t g_clock_state;
extern volatile bool g_24h;
extern volatile bool g_is_am;

// anodes (digits)
pin_direct_t digit0_pin;
pin_direct_t digit1_pin;
pin_direct_t digit2_pin;
pin_direct_t digit3_pin;
pin_direct_t digit4_pin;
pin_direct_t digit5_pin;

// K155ID1 nixie driver (A~D = PC0~PC3)
// PC0~PC3 = 14~17 (A0~A3)
#define DRIVER_DDR DDRC
#define DRIVER_PORT PORTC

uint8_t multiplex_counter = 0;

void clear_display();
void write_nixie(uint8_t digit, uint8_t value);
void set_number(uint8_t num);
uint32_t rnd(void);
void set_indicator(uint8_t intensity, bool override_state = false);

// fixme: hide alarm switch behind abstraction
extern pin_direct_t switch_pin;
extern volatile uint8_t g_alarm_switch;

void board_init()
{
  // alarm switch
  pinMode(PinMap::alarm_switch, INPUT); // switch as input
  digitalWrite(PinMap::alarm_switch, HIGH); // enable pullup
  
  switch_pin.pin = PinMap::alarm_switch;
  switch_pin.reg = PIN_TO_INPUT_REG(PinMap::alarm_switch);
  switch_pin.bitmask = PIN_TO_BITMASK(PinMap::alarm_switch);
  
  if ( (*switch_pin.reg & switch_pin.bitmask) == 0)
    g_alarm_switch = true;
  else
    g_alarm_switch = false;

  // dots
  pinMode(PinMap::dot1, OUTPUT);
  pinMode(PinMap::dot2, OUTPUT);

  // driver
  pinMode(PinMap::nixie_driver0, OUTPUT);
  pinMode(PinMap::nixie_driver1, OUTPUT);
  pinMode(PinMap::nixie_driver2, OUTPUT);
  pinMode(PinMap::nixie_driver3, OUTPUT);

  // digits
  pinMode(PinMap::digit0, OUTPUT);
  pinMode(PinMap::digit1, OUTPUT);
  pinMode(PinMap::digit2, OUTPUT);
  pinMode(PinMap::digit3, OUTPUT);
  pinMode(PinMap::digit4, OUTPUT);
  pinMode(PinMap::digit5, OUTPUT);

  digit0_pin.pin = PinMap::digit0;
  digit0_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit0);
  digit0_pin.bitmask = PIN_TO_BITMASK(PinMap::digit0);
  digit1_pin.pin = PinMap::digit1;
  digit1_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit1);
  digit1_pin.bitmask = PIN_TO_BITMASK(PinMap::digit1);  
  digit2_pin.pin = PinMap::digit2;
  digit2_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit2);
  digit2_pin.bitmask = PIN_TO_BITMASK(PinMap::digit2);
  digit3_pin.pin = PinMap::digit3;
  digit3_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit3);
  digit3_pin.bitmask = PIN_TO_BITMASK(PinMap::digit3);
  digit4_pin.pin = PinMap::digit4;
  digit4_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit4);
  digit4_pin.bitmask = PIN_TO_BITMASK(PinMap::digit4);
  digit5_pin.pin = PinMap::digit5;
  digit5_pin.reg = PIN_TO_OUTPUT_REG(PinMap::digit5);
  digit5_pin.bitmask = PIN_TO_BITMASK(PinMap::digit5);
}

void set_dots(bool dot1, bool dot2)
{
  if (dot1) digitalWrite(PinMap::dot1, HIGH);
  else      digitalWrite(PinMap::dot1, LOW);

  if (dot1) digitalWrite(PinMap::dot2, HIGH);
  else      digitalWrite(PinMap::dot2, LOW);
}

// Display multiplex routine optimized for 4 digits
void display_multiplex(void)
{
  if (multiplex_counter >= 0 && multiplex_counter <= 4)
    clear_display();
  else if (multiplex_counter >= 5 && multiplex_counter <= 14)
    display_on ? write_nixie(0, data[5]) : clear_display();
  else if (multiplex_counter >= 15 && multiplex_counter <= 19)
    clear_display();
  else if (multiplex_counter >= 20 && multiplex_counter <= 29)
    display_on ? write_nixie(1, data[4]) : clear_display();
  else if (multiplex_counter >= 30  && multiplex_counter <= 34)
    clear_display();
  else if (multiplex_counter >= 35 && multiplex_counter <= 44)
    display_on ? write_nixie(2, data[3]) : clear_display();
  else if (multiplex_counter >= 45  && multiplex_counter <= 49)
    clear_display();
  else if (multiplex_counter >= 50 && multiplex_counter <= 59)
    display_on ? write_nixie(3, data[2]) : clear_display();

  multiplex_counter++;

  if (multiplex_counter == 60) multiplex_counter = 0;
}

// Display multiplex routine for up to 6 digits
void display_multiplex_6(void)
{
  if (multiplex_counter == 0)
    clear_display();
  else if (multiplex_counter >= 1 && multiplex_counter <= 10)
    display_on ? write_nixie(0, data[5]) : clear_display();
  else if (multiplex_counter == 11)
    clear_display();
  else if (multiplex_counter >= 12 && multiplex_counter <= 21)
    display_on ? write_nixie(1, data[4]) : clear_display();
  else if (multiplex_counter == 22)
    clear_display();
  else if (multiplex_counter >= 23 && multiplex_counter <= 32)
    display_on ? write_nixie(2, data[3]) : clear_display();
  else if (multiplex_counter == 33)
    clear_display();
  else if (multiplex_counter >= 34 && multiplex_counter <= 43)
    display_on ? write_nixie(3, data[2]) : clear_display();
  else if (multiplex_counter == 44)
    clear_display();
  else if (multiplex_counter >= 45 && multiplex_counter <= 54)
    display_on ? write_nixie(4, data[1]) : clear_display();

  else if (multiplex_counter == 55)
    clear_display();
  else if (multiplex_counter >= 56 && multiplex_counter <= 65)
    display_on ? write_nixie(4, data[1]) : clear_display();

  multiplex_counter++;

  if (multiplex_counter == 66) multiplex_counter = 0;
}

void write_nixie(uint8_t digit, uint8_t value)
{
  //clear_display();

  if (g_blink_on) {
    if (g_blank == 4) { clear_display(); return; }
    else if (g_blank == 1 && (digit == 0 || digit == 1)) { clear_display(); return; }
    else if (g_blank == 2 && (digit == 2 || digit == 3)) { clear_display(); return; }
    else if (g_blank == 3 && (digit == 4 || digit == 5)) { clear_display(); return; }
  }

  if (value == 10) return;

  switch (digit) {
    case 0:
      DIRECT_PIN_LOW(digit0_pin.reg, digit0_pin.bitmask);
      break; 
    case 1:
      DIRECT_PIN_LOW(digit1_pin.reg, digit1_pin.bitmask);
      break; 
    case 2:
      DIRECT_PIN_LOW(digit2_pin.reg, digit2_pin.bitmask);
      break; 
    case 3:
      DIRECT_PIN_LOW(digit3_pin.reg, digit3_pin.bitmask);
      break; 
    case 4:
      DIRECT_PIN_LOW(digit4_pin.reg, digit4_pin.bitmask);
      break; 
    case 5:
      DIRECT_PIN_LOW(digit5_pin.reg, digit5_pin.bitmask);
      break; 
  }

  set_number(value);
}

void clear_display(void)
{
  DIRECT_PIN_HIGH(digit0_pin.reg, digit0_pin.bitmask);
  DIRECT_PIN_HIGH(digit1_pin.reg, digit1_pin.bitmask);
  DIRECT_PIN_HIGH(digit2_pin.reg, digit2_pin.bitmask);
  DIRECT_PIN_HIGH(digit3_pin.reg, digit3_pin.bitmask);
  DIRECT_PIN_HIGH(digit4_pin.reg, digit4_pin.bitmask);
  DIRECT_PIN_HIGH(digit5_pin.reg, digit5_pin.bitmask);
}

void set_number(uint8_t num)
{
  DRIVER_PORT = (DRIVER_PORT & 0xF0) | num;
}

void set_indicator(uint8_t intensity, bool override_state)
{
  switch (intensity)
  {
  case INDICATOR_OFF:
      //analogWrite(PinMap::piezo, 0);
  
    if (g_clock_state == STATE_CLOCK && !override_state) {
      if (g_24h)    
        pca9685_set_channel(13, 0);
      else
        pca9685_set_channel(13, g_is_am ? 0 : 100);

      pca9685_set_channel(14, 0);
      pca9685_set_channel(15, g_alarm_switch ? 100 : 0);  
    }
    else {
      pca9685_set_channel(13, 0);
      pca9685_set_channel(14, 0);
      pca9685_set_channel(15, 0);
    }
    break;
  case INDICATOR_HIGH:
      //analogWrite(PinMap::piezo, 150);
    pca9685_set_channel(13, 3000);
  case INDICATOR_MID:
      //analogWrite(PinMap::piezo, 100);
    pca9685_set_channel(14, 3000);
  case INDICATOR_LO:
      //analogWrite(PinMap::piezo, 50);
    pca9685_set_channel(15, 3000);
  }
}

#endif // BOARD_STANDARD

