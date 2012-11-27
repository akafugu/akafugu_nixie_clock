/*
 * The Akafugu Nixie Clock
 * (C) 2012 Akafugu Corporation
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

#include <EEPROM.h>
#include <Wire.h>
#include <WireRtcLib.h>

WireRtcLib rtc;

#include "rotary.h"
#include "button.h"
#include "rgbled.h"
#include "pitches.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

uint8_t digits = 6;
volatile uint8_t data[6]; // Digit data
uint8_t multiplex_counter = 0;
uint8_t display_on = false;

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

// rotary encoder
extern Rotary rotary;

// alarm switch
pin_direct_t switch_pin;

// SQW pin from RTC
pin_direct_t sqw_pin;

uint8_t g_volume = 0;
WireRtcLib::tm* t = NULL; // for holding RTC values

struct BUTTON_STATE button;

// Used for setting time and alarm
uint8_t set_hh, set_mm, set_ss;

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

  // menu
  STATE_MENU_24H,
  STATE_MENU_DOTS,
  STATE_MENU_LEADING_ZEROS,
  STATE_MENU_WAKE_SOUND,
  STATE_MENU_SCREEN_SAVER,
  STATE_MENU_ANTI_POISON,
  STATE_MENU_LAST,
} state_t;

state_t g_clock_state = STATE_CLOCK;
uint8_t g_display_mode = 0;
volatile bool g_is_am = false;

// toggled once every second to update the RTC
volatile bool g_update_rtc = false;
// Toggled whenever the rotary encoder moves to update RGB colors
volatile bool g_update_rgb = false;
// Toggled between 1 and -1 every 0.5 seconds to control backlight pulsing
volatile int8_t g_pulse_direction;
volatile uint16_t g_pulse_value = 1000;
// current state of alarm switch
volatile uint8_t g_alarm_switch;
// countdown timer for how long to show current alarm
volatile uint8_t g_show_alarm_counter;

// blanking:
// 0 - not blanked
// 1 - blank hh
// 2 - blank mm
// 3 - blank ss
// 4 - blank everything
volatile uint8_t g_blank = 0;
volatile bool g_blink_on = true;

// numbers for time display will be randomized when this flag is true
// (for anti-poisoning feature)
volatile bool g_randomize_on = false;

volatile uint16_t g_rotary_moved_timer;

// Display sleep mode
#define SLEEP_MODE_START_TIME 2 // sleep mode starts at 2:00 at night
#define SLEEP_MODE_END_TIME 7   // sleep mode ends at 7:00 in the morning

///////////////////////
// Settings (saved to EEPROM)
// 24 hour display mode
volatile bool g_24h = true;
// Dots setting (0 - off, 1 - on, 2 - blink)
volatile uint8_t g_dots_setting = 2;
// Leading zeros in hour display
volatile bool g_leading_zeros = true;
// Sound on wakeup
volatile bool g_wakeup_sound = true;
// Screen saver (display turns off at night)
volatile bool g_screensaver = false;
// Cathode anti-poisoning
volatile bool g_antipoison = true;

void display_init(void);
void set_dots(bool dot1, bool dot2);
void display_multiplex(void);
void write_nixie(uint8_t digit, uint8_t value);
void clear_display(void);
void set_number(uint8_t num);
void count(void);
void read_rtc(void);
void rtc_sqw_interrupt();

void display_init(void)
{
  // set INPUTS and OUTPUTS

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

  // dots
  pinMode(PinMap::dot1, OUTPUT);
  pinMode(PinMap::dot2, OUTPUT);

  // driver
  pinMode(PinMap::nixie_driver0, OUTPUT);
  pinMode(PinMap::nixie_driver1, OUTPUT);
  pinMode(PinMap::nixie_driver2, OUTPUT);
  pinMode(PinMap::nixie_driver3, OUTPUT);

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
  
  // SQW output from RTC
  pinMode(PinMap::sqw, INPUT);
  digitalWrite(PinMap::sqw, HIGH); // enable pullup
  
  sqw_pin.pin = PinMap::sqw;
  sqw_pin.reg = PIN_TO_INPUT_REG(PinMap::sqw);
  sqw_pin.bitmask = PIN_TO_BITMASK(PinMap::sqw);

  // Piezo  
  pinMode(PinMap::piezo, OUTPUT);
  digitalWrite(PinMap::piezo, LOW);

  clear_display();

  //EEPROM.write(0, 0x0); uncomment to force reset EEPROM
  read_eeprom();
  
  if (g_wakeup_sound) {
    tone(PinMap::piezo, NOTE_A4, 500);
    delay(100);
    tone(PinMap::piezo, NOTE_C3, 750);
    delay(100);
    tone(PinMap::piezo, NOTE_A4, 500);
    delay(500);
  }
  else {
    delay(1000);
  }

  // Inititalize timer for multiplexing
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  ASSR &= ~(1<<AS2);
  TIMSK2 &= ~(1<<OCIE2A);

  TCCR2B = (1<<CS21); // Set Prescaler to clk/8 : 1 click = 1us.
  TIMSK2 |= (1<<TOIE2); // Enable Overflow Interrupt Enable
  TCNT2 = 0; // Initialize counter

  // set up interrupt for SQW output from RTC
  // fixme: hardcoded for pin 3
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT19);

  // Initialize button
  initialize_button(PinMap::button, -1);

  // Initialize rotary encoder
  rotary.begin();
  rotary.setRange(0, 12); // should match the number of RGB led settings

  sei();	// Enable interrupts
  Wire.begin();

  pca9685_wake();
  
  // Turn off all LEDs
  for (uint8_t i = 0; i < 16; i++)
    pca9685_set_channel(i, 0);

  rtc.begin();

  // enable SQW interrupt from RTC at 1Hz
  rtc.SQWSetFreq(WireRtcLib::FREQ_1);
  rtc.SQWEnable(true);

  read_rtc();
  display_on = true;
}

void read_eeprom()
{
  // Check if there are existing settings
  if (EEPROM.read(0) == 0x42) {
    //Serial.println("Reading EEPROM");
    g_24h = EEPROM.read(1);
    g_dots_setting = EEPROM.read(2);
    g_leading_zeros = EEPROM.read(3);
    g_wakeup_sound = EEPROM.read(4);
    g_screensaver = EEPROM.read(5);
    g_antipoison = EEPROM.read(6);
    
    //Serial.print("g_24h = ");
    //Serial.println(g_24h);
    //Serial.print("g_dots_setting = ");
    //Serial.println(g_dots_setting);
    //Serial.print("g_leading_zeros = ");
    //Serial.println(g_leading_zeros);
    //Serial.print("g_wakeup_sound = ");
    //Serial.println(g_wakeup_sound);
    //Serial.print("g_screensaver = ");
    //Serial.println(g_screensaver);
    //Serial.print("g_antipoison = ");
    //Serial.println(g_antipoison);
  }
  // No existing settings, wipe with defaults
  else {
    //Serial.println("Initializing EEPROM with defaults");
    write_eeprom();
  }
}

void write_eeprom()
{
  //Serial.println("Writing EEPROM");
  EEPROM.write(0, 0x42); // control byte
  EEPROM.write(1, g_24h);
  EEPROM.write(2, g_dots_setting);
  EEPROM.write(3, g_leading_zeros);
  EEPROM.write(4, g_wakeup_sound);
  EEPROM.write(5, g_screensaver);
  EEPROM.write(6, g_antipoison);
  
  //Serial.print("g_24h = ");
  //Serial.println(g_24h);
  //Serial.print("g_dots_setting = ");
  //Serial.println(g_dots_setting);
  //Serial.print("g_leading_zeros = ");
  //Serial.println(g_leading_zeros);
  //Serial.print("g_wakeup_sound = ");
  //Serial.println(g_wakeup_sound);
  //Serial.print("g_screensaver = ");
  //Serial.println(g_screensaver);
  //Serial.print("g_antipoison = ");
  //Serial.println(g_antipoison);
}

void set_dots(bool dot1, bool dot2)
{
  if (dot1) digitalWrite(PinMap::dot1, HIGH);
  else      digitalWrite(PinMap::dot1, LOW);

  if (dot1) digitalWrite(PinMap::dot2, HIGH);
  else      digitalWrite(PinMap::dot2, LOW);
}

#define INDICATOR_OFF  0
#define INDICATOR_LO   1
#define INDICATOR_MID  2
#define INDICATOR_HIGH 3

void set_indicator(uint8_t intensity, bool override_state = false)
{
  switch (intensity)
  {
  case INDICATOR_OFF:
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
    pca9685_set_channel(13, 3000);
  case INDICATOR_MID:
    pca9685_set_channel(14, 3000);
  case INDICATOR_LO:
    pca9685_set_channel(15, 3000);
  }
}

// Display multiplex routine optimized for 4 digits
void display_multiplex_4(void)
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

  multiplex_counter++;

  if (multiplex_counter == 44) multiplex_counter = 0;
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

uint8_t interrupt_counter = 0;
uint16_t button_counter = 0;

// 1 click = 1us. 
ISR(TIMER2_OVF_vect)
{ 
  display_multiplex_4();
  interrupt_counter = 0;

  // button polling
  if (++button_counter == 150) {
    button_timer();
    button_counter = 0;
  }
}

uint32_t rnd(void);

void write_nixie(uint8_t digit, uint8_t value)
{
  clear_display();

  if (g_blink_on) {
    if (g_blank == 4) { clear_display(); return; }
    else if (g_blank == 1 && (digit == 0 || digit == 1)) { clear_display(); return; }
    else if (g_blank == 2 && (digit == 2 || digit == 3)) { clear_display(); return; }
    else if (g_blank == 3 && (digit == 4 || digit == 5)) { clear_display(); return; }
  }

  if (g_antipoison && g_randomize_on) {
    value = rnd() % 10;
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

// RTC SQW output interrupt and alarm switch change interrupt
ISR( PCINT2_vect )
{
  if ( (*sqw_pin.reg & sqw_pin.bitmask) == 0) {
    g_update_rtc = true;
    
    if (g_pulse_direction == 1) {
      g_pulse_direction = -1;
      //Serial.print("Pulse -1 at ");
      //Serial.println(g_pulse_value);
      g_pulse_value = 1000;
    }
    else {
      g_pulse_direction = 1;
      //Serial.print("Pulse +1 at ");
      //Serial.println(g_pulse_value);
    }
    
    g_blink_on = false;
  }
  else  if (g_rotary_moved_timer == 0) {
    g_blink_on = true;
  }
  
  if (g_alarm_switch == false && (*switch_pin.reg & switch_pin.bitmask) == 0) {
    g_alarm_switch = true;
    if (g_clock_state == STATE_CLOCK) {
      g_clock_state = STATE_SHOW_ALARM;
      g_show_alarm_counter = 150;
    }
  }
  else if ( (*switch_pin.reg & switch_pin.bitmask) == 0)
    g_alarm_switch = true;
  else
    g_alarm_switch = false;
}

// Displays a 2 digit number in position pos (0, 1, 2)
void disp2(uint8_t pos, uint8_t value)
{
  if (pos == 0) {
    data[4] = value % 10;
    value /= 10;
    data[5] = value % 10;
  }
  else if (pos == 1) {
    data[2] = value % 10;
    value /= 10;
    data[3] = value % 10;
  }
  else if (pos == 2) {
    data[0] = value % 10;
    value /= 10;
    data[1] = value % 10;
  }
}

void read_rtc(void)
{ 
  t = rtc.getTime();
  if (t == NULL) {
    t = rtc.getTime();

    if (t == NULL) {
        return;
    }
  }

  // check for display sleep mode
  if (g_screensaver && t->hour >= SLEEP_MODE_START_TIME && t->hour < SLEEP_MODE_END_TIME) {
    data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 10;
    set_dots(false, false);
    return;
  }
  
  // check if it is time for anti-poisoning routine
  if (t->sec < 1 && t->min % 5 == 0)
    g_randomize_on = true;
  else
    g_randomize_on = false ;

  // CONTROL DOTS
  if (g_dots_setting == 0) { // always off
    set_dots(false, false);
  }
  else if (g_dots_setting == 1) { // always on
    set_dots(true, true);
  }
  else if (t->sec % 2 == 0) {
    set_dots(true, true);
  }
  else {
    set_dots(false, false);
  }

  g_is_am = t->am;

  if (g_display_mode == 0) {
    uint8_t hour = g_24h ? t->hour : t->twelveHour;
    
    if (!g_leading_zeros && hour < 10) { // no leading zeros on hour
      data[5] = 10;
      data[4] = hour;
    }
    else {
      disp2(0, hour);
    }
    
    disp2(1, t->min);
    disp2(2, t->sec);
  }
  else {
    data[0] = data[1] = 10; // blank seconds
    disp2(1, t->sec);
    data[4] = data[5] = 10; // blank hours
  }  
}

void start_alarm()
{
  // blank display
  data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 10;
  set_number(10);
  set_dots(false, false);
  
  // cancel any randomization event
  g_randomize_on = false;
  
  // cancel multiplexing timer
  TIMSK2 = 0;
  
  // cancel SQW interrupt
  PCICR &= ~(1 << PCIE2);
  
  // save rotary state
  rotary.save();
  
  g_clock_state = STATE_ALARMING;
}

void stop_alarm()
{
  // Re-initialze timer for multiplexing
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  ASSR &= ~(1<<AS2);
  TIMSK2 &= ~(1<<OCIE2A);

  TCCR2B = (1<<CS21); // Set Prescaler to clk/8 : 1 click = 1us. CS01=1
  TIMSK2 |= (1<<TOIE2); // Enable Overflow Interrupt Enable
  TCNT2 = 0; // Initialize counter

  // restart SQW interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT19);

  // restore rotary state
  rotary.restore();

  g_clock_state = STATE_CLOCK;
}

void sound_alarm()
{
  data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 10;  
  
  tone(PinMap::piezo, NOTE_A4, 500);
  delay(100);
  tone(PinMap::piezo, NOTE_C3, 750);
  delay(100);
  tone(PinMap::piezo, NOTE_A4, 500);
  delay(500);
}

#define DELAY 250
uint8_t int_counter = 0;
int16_t button_held_counter = 0;

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

void setup() {
  //Serial.begin(9600);
  //Serial.println("Nixie Clock");  
}

// Counter values for holding the button to enter settings/menu
#define CHARGE_LOW 150
#define CHARGE_MID 300
#define CHARGE_HIGH 450

void loop() {
  display_init();
  
  // test code
  //rtc.setTime_s(23, 8, 0);
  //rtc.runClock(true);
  //test_backlight();

  while(1) {
    _delay_us(DELAY);
      
    if (int_counter++ == 15) {
      int_counter = 0;
      
      get_button_state(&button);

      switch(g_clock_state) {
        case STATE_CLOCK:
        {          
          if (g_update_rtc) {
            read_rtc();
            g_update_rtc = false;
          }

          if (g_update_rgb) {
            set_rgb_mode(rotary.getPosition());
            g_update_rgb = false;
          }
          
          // Testing for held button. Used to enter menu states
          if (button.b1_keydown) {
            button_held_counter++;

            if (button_held_counter < CHARGE_LOW) {
              set_indicator(INDICATOR_OFF, true); // turn off all indicators 
            }
            if (button_held_counter == CHARGE_LOW) {
              set_indicator(INDICATOR_LO);
              //Serial.println("Charged 200");
            }
            else if (button_held_counter == CHARGE_MID) {
              set_indicator(INDICATOR_MID);
              //Serial.println("Charged 400");
            }
            else if (button_held_counter == CHARGE_HIGH) {
              set_indicator(INDICATOR_HIGH);
              //Serial.println("Charged 600");
            }
          }
          else {
            // button held for 5 seconds or more: CUSTOM SETTINGS
            if (button_held_counter >= CHARGE_HIGH) {
              //Serial.println("Entering custom settings");
              
              rotary.save();
              rotary.setDivider(10);
              rotary.setRange(0, 1);
              rotary.setPosition(g_24h);

              g_blank = 0;
              g_clock_state = STATE_MENU_24H;
              set_rgb_mode(0);
            }
            // button held for 3-4 seconds: SET TIME
            else if (button_held_counter >= CHARGE_MID) {
              //Serial.println("Entering set time");
              t = rtc.getTime();
              set_hh = t->hour;
              set_mm = t->min;
              set_ss = 0;

              rotary.save();
              rotary.setRange(0, 23);
              rotary.setPosition(set_hh);
              
              disp2(0, set_hh);
              disp2(1, set_mm);
              set_dots(false, false);
              
              g_blank = 1;
              g_clock_state = STATE_SET_CLOCK_HH;
              
              set_rgb_mode(0);
              set_rgb_ch(0, 1000, 1000, 1000);
              set_rgb_ch(1, 1000, 1000, 1000);              
            }
            // button held for 1-2 seconds: SET ALARM
            else if (button_held_counter >= CHARGE_LOW) {
              //Serial.println("Entering set alarm");
              t = rtc.getAlarm();
              set_hh = t->hour;
              set_mm = t->min;
              set_ss = 0;

              rotary.save();
              rotary.setRange(0, 23);
              rotary.setPosition(set_hh);
              
              disp2(0, set_hh);
              disp2(1, set_mm);
              set_dots(false, false);
              
              g_blank = 1;
              g_clock_state = STATE_SET_ALARM_HH;
              set_rgb_mode(0);
              set_rgb_ch(0, 1000, 1000, 1000);
              set_rgb_ch(1, 1000, 1000, 1000);              
            }
            else {
              set_indicator(INDICATOR_OFF);
            }
            
            button_held_counter = 0;
          }

          if (button.b1_keyup) {
            g_display_mode = !g_display_mode;
            button.b1_keyup = 0;
            g_update_rtc = true;
          }
        }
          break;
        case STATE_SHOW_ALARM:
        {
          t = rtc.getAlarm();
          
          if (t) {
            disp2(0, t->hour);
            disp2(1, t->min);
          }
          
          if (--g_show_alarm_counter == 0)
            g_clock_state = STATE_CLOCK;
        }
          break;
        case STATE_ALARMING:
        {
          if ( (*switch_pin.reg & switch_pin.bitmask) != 0) {
            stop_alarm();
          }
          else {
            sound_alarm();
          }
        }
          break;
        case STATE_SET_ALARM_HH:
        {
          // move to setting minutes when button is pressed
          if (button.b1_keyup) {
            g_blank = 2;
            
            rotary.setRange(0, 59);
            rotary.setPosition(set_mm);
            
            g_clock_state = STATE_SET_ALARM_MM;
            button.b1_keyup = 0;
            
            set_rgb_mode(0);
            set_rgb_ch(2, 1000, 1000, 1000);
            set_rgb_ch(3, 1000, 1000, 1000);            
          }
          else {
            set_hh = (rotary.getPosition()) % 24;
            disp2(0, set_hh);
          }
        }
          break;
        case STATE_SET_ALARM_MM:
        {
          // go back to clock mode when button is pressed
          if (button.b1_keyup) {
            g_blank = 0;
            
            rtc.setAlarm_s(set_hh, set_mm, set_ss);
            
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            g_update_rgb = true;
            rotary.restore();
            g_clock_state = STATE_CLOCK;            
            button.b1_keyup = 0;
          }
          else {
            set_mm = (rotary.getPosition()) % 60;
            disp2(1, set_mm);
          }
        }
          break;
        case STATE_SET_CLOCK_HH:
        {
          // move to setting minutes when button is pressed
          if (button.b1_keyup) {
            g_blank = 2;
            
            rotary.setRange(0, 59);
            rotary.setPosition(set_mm);
            
            g_clock_state = STATE_SET_CLOCK_MM;
            button.b1_keyup = 0;
            
            set_rgb_mode(0);
            set_rgb_ch(2, 1000, 1000, 1000);
            set_rgb_ch(3, 1000, 1000, 1000);            
          }
          else {
            set_hh = (rotary.getPosition()) % 24;
            disp2(0, set_hh);
          }
        }
          break;
        case STATE_SET_CLOCK_MM:
        {
          // move to setting seconds when button is pressed
          if (button.b1_keyup) {
            g_blank = 2;
            
            rotary.setRange(0, 59);
            rotary.setPosition(set_ss);
            
            g_clock_state = STATE_SET_CLOCK_SS;
            button.b1_keyup = 0;
            
            set_rgb_mode(0);
            set_rgb_ch(2, 0, 500, 1000);
            set_rgb_ch(3, 0, 500, 1000);
          }
          else {
            set_mm = (rotary.getPosition()) % 60;
            disp2(1, set_mm);
          }
        }
        break;
        case STATE_SET_CLOCK_SS:
        {
          // move to setting seconds when button is pressed
          if (button.b1_keyup) {
            g_blank = 0;
            
            rtc.setTime_s(set_hh, set_mm, set_ss);
            
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            g_update_rgb = true;
            rotary.restore();
            g_clock_state = STATE_CLOCK;            
            button.b1_keyup = 0;
          }
          else {
            set_ss = (rotary.getPosition()) % 60;
            data[4] = data[5] = 10;
            disp2(1, set_ss);
          }
        }
        break;
        case STATE_MENU_24H:
        {
          if (button.b1_keyup) { // go to STATE_MENU_DOTS
            g_24h = rotary.getPosition();
            
            rotary.setRange(0, 2);
            rotary.setPosition(g_dots_setting);
            g_clock_state = STATE_MENU_DOTS;
          }
          else {
            disp2(0, 1);
            disp2(1, rotary.getPosition() % 2 == 0 ? 12 : 24);
          }
        }
        break;
        case STATE_MENU_DOTS:
        {
          if (button.b1_keyup) { // go to STATE_MENU_LEADING_ZEROS
            g_dots_setting = rotary.getPosition();
          
            rotary.setRange(0, 1);
            rotary.setPosition(g_leading_zeros);
            g_clock_state = STATE_MENU_LEADING_ZEROS;
          }
          else {
            disp2(0, 2);
            disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_LEADING_ZEROS:
        {
          if (button.b1_keyup) { // go to STATE_MENU_WAKE_SOUND
            g_leading_zeros = rotary.getPosition();
          
            rotary.setRange(0, 1);
            rotary.setPosition(g_wakeup_sound);
            g_clock_state = STATE_MENU_WAKE_SOUND;
          }
          else {
            disp2(0, 3);
            disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_WAKE_SOUND:
        {
          if (button.b1_keyup) {
            g_wakeup_sound = rotary.getPosition();
          
            rotary.setRange(0, 1);
            rotary.setPosition(g_screensaver);
            g_clock_state = STATE_MENU_SCREEN_SAVER;
          }
          else {
            disp2(0, 4);
            disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_SCREEN_SAVER:
        {
          if (button.b1_keyup) { // go to STATE_MENU_ANTI_POISON
            g_screensaver = rotary.getPosition();
          
            rotary.setRange(0, 1);
            rotary.setPosition(g_antipoison);
            g_clock_state = STATE_MENU_ANTI_POISON;
          }
          else {
            disp2(0, 5);
            disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_ANTI_POISON:
        {
          if (button.b1_keyup) { // go to STATE_CLOCK
            g_antipoison = rotary.getPosition();
          
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            rotary.setDivider(4);
            rotary.restore();
            g_clock_state = STATE_CLOCK;            
            button.b1_keyup = 0;
          
            // write all settings to EEPROM
            write_eeprom();  
          }
          else {
            disp2(0, 6);
            disp2(1, rotary.getPosition());
          }
        }
        break;
      }
      
      rgb_tick();
      
      if (g_clock_state == STATE_CLOCK && g_alarm_switch && rtc.checkAlarm())
        start_alarm();
      
      // cooldown for rotary encoder moved
      if (g_rotary_moved_timer > 0)
        g_rotary_moved_timer--;
    }
  }
}

