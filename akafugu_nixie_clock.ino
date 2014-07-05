/*
 * The Akafugu Nixie Clock
 * (C) 2012-1 Akafugu Corporation
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

#include "gps.h"
#include "rotary.h"
#include "button.h"
#include "backlight.h"
#include "pitches.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

uint8_t digits = 6;
volatile uint8_t data[6]; // Digit data
uint8_t display_on = false;

void set_indicator(uint8_t intensity, bool override_state = false);

// rotary encoder
extern Rotary rotary;

// alarm switch
pin_direct_t switch_pin;

// buttons
extern pin_direct_t button1;
extern pin_direct_t button2;

// SQW pin from RTC
pin_direct_t sqw_pin;

uint8_t g_volume = 0;
WireRtcLib::tm* t = NULL; // for holding RTC values

struct BUTTON_STATE button;

// Used for setting time and alarm
uint8_t set_hh, set_mm, set_ss;

state_t g_clock_state = STATE_CLOCK;
uint8_t g_display_mode = 0;
volatile bool g_is_am = false;

// toggled once every second to update the RTC
volatile bool g_update_rtc = false;
// Toggled whenever the rotary encoder moves to update the backlight
volatile bool g_update_backlight = false;
// Toggled between 1 and -1 every 0.5 seconds to control backlight pulsing
volatile int8_t g_pulse_direction;
volatile uint16_t g_pulse_value = 1000;
// current state of alarm switch
volatile uint8_t g_alarm_switch;
// countdown timer for how long to show current alarm
volatile uint8_t g_show_alarm_counter;

// test mode counter
volatile uint8_t g_test_counter;

// blanking:
// 0 - not blanked
// 1 - blank hh
// 2 - blank mm
// 3 - blank ss
// 4 - blank everything
volatile uint8_t g_blank;
volatile bool g_blink_on = true;

volatile uint16_t g_rotary_moved_timer;

#ifdef HAVE_GPS
volatile bool g_gps_enabled = false;
volatile int8_t g_TZ_hour;
volatile int8_t g_TZ_minute;
volatile uint8_t g_dst_offset;
#endif

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
// Number of digits
#ifdef MODULAR_6D
volatile uint8_t g_digits = 6;
#else
volatile uint8_t g_digits = 4;
#endif

void display_init();
void display_multiplex(void);
void clear_display(void);

void board_init(); // performs special initialization based on board type

void set_dots(bool dot1, bool dot2);
void set_alarm_led(bool on);

#ifdef BOARD_STANDARD
void set_number(uint8_t num);
#else
void set_number(uint8_t value1, uint8_t value2);
#endif // board type

void count(void);
void read_rtc(void);
void rtc_sqw_interrupt();

void display_init()
{
  clear_display();
  board_init();
  
  // SQW output from RTC
  pinMode(PinMap::sqw, INPUT);
  digitalWrite(PinMap::sqw, HIGH); // enable pullup
  
  sqw_pin.pin = PinMap::sqw;
  sqw_pin.reg = PIN_TO_INPUT_REG(PinMap::sqw);
  sqw_pin.bitmask = PIN_TO_BITMASK(PinMap::sqw);

  // Piezo  
  pinMode(PinMap::piezo, OUTPUT);
  digitalWrite(PinMap::piezo, LOW);

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
  
  TIMSK2 &= ~(1<<OCIE2A);

  //TCCR2B = (1<<CS20); // Set Prescaler to clk/8 : 1 click = 1us.
  TCCR2B = (1<<CS21); // Set Prescaler to clk/8 : 1 click = 1us.
  TIMSK2 |= (1<<TOIE2); // Enable Overflow Interrupt Enable
  TCNT2 = 0; // Initialize counter

  // set up interrupt for SQW output from RTC
  // fixme: hardcoded for pin 3
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT19);

  // Initialize button
  initialize_button(PinMap::button, PinMap::button_alarm);

  // Initialize rotary encoder
#ifdef HAVE_ROTARY
  rotary.begin();
#endif

  rotary.setRange(0, BACKLIGHT_MODES); // should match the number of RGB led settings

  sei();	// Enable interrupts
  Wire.begin();

  init_backlight();

  rtc.begin();

  // enable SQW interrupt from RTC at 1Hz
  rtc.SQWSetFreq(WireRtcLib::FREQ_1);
  rtc.SQWEnable(true);

  read_rtc();
  display_on = true;

#ifdef HAVE_GPS
    gps_init(g_gps_enabled ? 48 : 0); // fixme: support both modes
#endif // HAVE_GPS
  
  Serial.begin(9600);
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
    
#ifdef HAVE_GPS
    g_gps_enabled = EEPROM.read(7);
    g_TZ_hour     = EEPROM.read(8);
    g_TZ_minute   = EEPROM.read(9);
    g_dst_offset  = EEPROM.read(10);
#endif
    
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
  
#ifdef HAVE_GPS
  EEPROM.write(7, g_gps_enabled);
  EEPROM.write(8, g_TZ_hour);
  EEPROM.write(9, g_TZ_minute);
  EEPROM.write(10, g_dst_offset);
#else
  EEPROM.write(7, 0);
  EEPROM.write(8, 0);
  EEPROM.write(9, 0);
  EEPROM.write(10, 0);
#endif

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

void tick()
{
  analogWrite(PinMap::piezo, 50);              
  delay(5);
  analogWrite(PinMap::piezo, 0);
}

uint8_t interrupt_counter = 0;
uint16_t button_counter = 0;

#ifdef HAVE_GPS
uint8_t gps_counter = 0;
#endif

// 1 click = 1us. 
ISR(TIMER2_OVF_vect)
{

#ifdef HAVE_HV5812
  if (interrupt_counter++ == 2) {
      display_multiplex();
      interrupt_counter = 0;
  }
#else
  display_multiplex();
#endif // board type

  // button polling
  if (++button_counter == 150) {
    button_timer();
    button_counter = 0;
  }
  
#ifdef FEATURE_GPS
  if (++gps_counter == 4) {  // every 0.001024 seconds
    GPSread();  // check for data on the serial port
    gps_counter = 0;
  }
#endif // HAVE_GPS
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
    if (!button.b3_keydown)
      g_blink_on = true;
  }
  
#ifdef HAVE_ALARM_SLIDE_SWITCH
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
#endif

  g_test_counter++;
  if (g_test_counter == 10) g_test_counter = 0;
}

// Displays a 2 digit number in position pos (0, 1, 2)
void disp2(uint8_t pos, int8_t value)
{
  if (value == -1)
      value = 10;
    
  // hour position, digits 1 and 2
  if (pos == 0) {
    data[4] = value % 10;
    value /= 10;
    data[5] = value % 10;
  }
  // minute position, digits 3 and 4
  else if (pos == 1) {
    data[2] = value % 10;
    value /= 10;
    data[3] = value % 10;
  }
  // second position, digits 5 and 6
  else if (pos == 2) {
    data[0] = value % 10;
    value /= 10;
    data[1] = value % 10;
  }
}

void enter_anti_poison_mode()
{
  rotary.save();
  g_blank = 0;
  g_test_counter = 0;
  g_clock_state = STATE_ANTIPOISON;
  push_backlight_mode();
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
  if (g_antipoison && t->sec < 1 && t->min % 5 == 0)
    enter_anti_poison_mode();

  // CONTROL DOTS
  if (g_dots_setting == 0) { // always off
    set_dots(false, false);
  }
  else if (g_display_mode != 0) {
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
  uint8_t hour = g_24h ? t->hour : t->twelveHour;

  if (g_display_mode == 0) { // primary display mode
    
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
  else { // alternate display mode
    if (g_digits == 4) {
      data[0] = data[1] = 10; // blank seconds
      disp2(1, t->sec);
      data[4] = data[5] = 10; // blank hours
    }
    else if (g_digits == 6) {
      data[5] = 10;
      
      uint8_t value = hour;
      data[3] = value % 10;
      value /= 10;
      data[4] = value % 10;
      
      value = t->min;
      data[1] = value % 10;
      value /= 10;
      data[2] = value % 10;
      
      data[0] = 10;
    }
  }  
}

void start_alarm()
{
  // blank display
  data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 10;
  
#ifdef HAVE_HV5812
  set_number(10, 10);
#else
  set_number(10);
#endif // HAVE_HV5812

  set_dots(false, false);
  
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

void exit_menu()
{
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

#define DELAY 250
uint8_t int_counter;
int16_t button_held_counter;
int16_t alarm_button_held_counter;

void setup() {
  //Serial.begin(9600);
  //Serial.println("Nixie Clock");  
}

// Counter values for holding the button to enter settings/menu
// fixme: use better timing method
#if defined (BOARD_STANDARD) || defined (BOARD_MK2)
#  define CHARGE_LOW 150
#  define CHARGE_MID 300
#  define CHARGE_HIGH 450
#  define CHARGE_TEST 800
#elif defined(BOARD_DIET) || defined(BOARD_MODULAR)
#  define CHARGE_LOW 10
#  define CHARGE_MID 20
#  define CHARGE_HIGH 30
#endif

// Button 3 handling (for clocks that have two control buttons (plus alarm button)
// instead of a rotary encoder
void handle_button_3()
{
    switch (g_clock_state) {
    case STATE_CLOCK:
        // change backlight
        if (button.b3_keyup) {
            button.b3_keyup = 0;
            increment_backlight_mode();
            tick();
        }
        break;
    case STATE_SHOW_ALARM:
    case STATE_ALARMING:
    case STATE_TEST_MODE:
    case STATE_ANTIPOISON:
        // no effect
        if (button.b3_keyup) {
            button.b3_keyup = 0;
        }
        break;
    case STATE_SET_ALARM_HH:
    case STATE_SET_ALARM_MM:
    case STATE_SET_CLOCK_HH:
    case STATE_SET_CLOCK_MM:
    case STATE_SET_CLOCK_SS:
    case STATE_MENU_24H:
    case STATE_MENU_DOTS:
    case STATE_MENU_LEADING_ZEROS:
    case STATE_MENU_WAKE_SOUND:
    case STATE_MENU_SCREEN_SAVER:
    case STATE_MENU_ANTI_POISON:
#ifdef HAVE_GPS
    case STATE_MENU_GPS:
    case STATE_MENU_GPS_TZH:
    case STATE_MENU_GPS_TZM:
    case STATE_MENU_DST_OFFSET:
#endif // HAVE_GPS
        // increment
        if (button.b3_keyup) {
            rotary.incrementPosition();
            button.b3_keyup = 0;
            button.b3_repeat = 0;
        }
        
        if (button.b3_repeat) {
            rotary.incrementPosition();
            button.b3_repeat = 0;
        }
        
        break;        
    }
}

void loop() {
  display_init();
  
  // test code
  //rtc.setTime_s(17, 25, 0);
  //test_backlight();

  rtc.runClock(true);
  
  while(1) {
#ifdef HAVE_GPS
    if (g_gps_enabled && g_clock_state == STATE_CLOCK) {
      if (gpsDataReady()) {
        parseGPSdata(gpsNMEA());  // get the GPS serial stream and possibly update the clock 
      }
      else {
        _delay_us(DELAY);
      }
    }
    else
      _delay_us(DELAY);
#else
    _delay_us(DELAY);
#endif
      
    if (int_counter++ == 15) {
      int_counter = 0;
      
      get_button_state(&button);

      handle_button_3();

      switch(g_clock_state) {
        case STATE_CLOCK:
        {          
          if (g_update_rtc) {
            read_rtc();
            g_update_rtc = false;
          }

          if (g_update_backlight) {
#ifdef HAVE_ROTARY
            set_backlight_mode(rotary.getPosition());
#else
            pop_backlight_mode();
#endif
            g_update_backlight = false;
          }
          
#ifndef HAVE_ALARM_SLIDE_SWITCH
          // check state of the alarm on/off button
          if (button.b2_keyup) {
              button.b2_keyup = 0;
              g_alarm_switch = !g_alarm_switch;
            
              if (g_alarm_switch) {
                  g_clock_state = STATE_SHOW_ALARM;
                  g_show_alarm_counter = 150;
              }
              
              tick();
              set_alarm_led(g_alarm_switch);
          }
          
          // check if alarm button is held
          if (button.b2_keydown) {
              alarm_button_held_counter++;
            
              if (alarm_button_held_counter >= 300) {
                  analogWrite(PinMap::piezo, 150);
                  delay(200);
                  analogWrite(PinMap::piezo, 0);
                  
                  rotary.save();
                  g_blank = 0;
                  g_test_counter = 0;
                  g_clock_state = STATE_TEST_MODE;
                  push_backlight_mode();
              }
          }
          else {
              alarm_button_held_counter = 0;
          }
          
#endif  // HAVE_ALARM_SLIDE_SWITCH        

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
            
#ifdef HAVE_ALARM_SLIDE_SWITCH
            // button held for more than 10 seconds: TEST MODE
            else if (button_held_counter == CHARGE_TEST) {
                set_indicator(INDICATOR_OFF, true); // turn off all indicators
                button_held_counter = 0;
              
                analogWrite(PinMap::piezo, 150);
                delay(200);
                analogWrite(PinMap::piezo, 0);
                  
                rotary.save();
                g_blank = 0;
                g_test_counter = 0;
                g_clock_state = STATE_TEST_MODE;
                push_backlight_mode();
            }
#endif // HAVE_ALARM_SLIDE_SWITCH

#ifdef HAVE_MENU_AUDIO_INDICATOR
            delay(100);
            analogWrite(PinMap::piezo, 0);
#endif // HAVE_MENU_AUDIO_INDICATOR
          }
          else {
            // button held for 5 seconds or more: CUSTOM SETTINGS
            if (button_held_counter >= CHARGE_HIGH) {
              //Serial.println("Entering custom settings");
              
              rotary.save();
              rotary.setDivider(10);
              rotary.setRange(0, 1);
              rotary.setPosition(g_24h);

              set_dots(false, false);
              g_blank = 0;
              g_clock_state = STATE_MENU_24H;
              push_backlight_mode();
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
              disp2(2, set_ss);
              set_dots(false, false);
              
              g_blank = 1;
              g_clock_state = STATE_SET_CLOCK_HH;
              
              set_backlight_hh();
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
              disp2(2, 0);
              set_dots(false, false);
              
              g_blank = 1;
              g_clock_state = STATE_SET_ALARM_HH;
              
              set_backlight_hh();
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
            tick();
          }
        }
          break;
        case STATE_SHOW_ALARM:
        {
          t = rtc.getAlarm();
          set_dots(0, 0);
          
          if (t && g_digits == 4) {
            disp2(0, t->hour);
            disp2(1, t->min);
            disp2(2, t->sec);
          }
          else if (t && g_digits == 6) {
            data[5] = 10;
      
            uint8_t value = t->hour;
            data[3] = value % 10;
            value /= 10;
            data[4] = value % 10;
      
            value = t->min;
            data[1] = value % 10;
            value /= 10;
            data[2] = value % 10;
      
            data[0] = 10;            
          }
          
          if (--g_show_alarm_counter == 0)
            g_clock_state = STATE_CLOCK;
        }
          break;
        case STATE_ALARMING:
        {
          // fixme: define for all button/slide switch combination
#if defined(BOARD_STANDARD) || defined(BOARD_MK2)
          if ( (*switch_pin.reg & switch_pin.bitmask) != 0) {
#elif defined(BOARD_DIET) || defined(BOARD_MODULAR) || defined(BOARD_MK2)
          if ( (*button1.reg & button1.bitmask) == 0 || (*button2.reg & button2.bitmask) == 0) {
#endif
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
            
            set_backlight_mm();
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
            g_update_backlight = true;
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
            
            set_backlight_mm();
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
            if (g_digits == 6)
                g_blank = 3;
            else
                g_blank = 2;
            
            rotary.setRange(0, 59);
            rotary.setPosition(set_ss);
            
            g_clock_state = STATE_SET_CLOCK_SS;
            button.b1_keyup = 0;
            
            set_backlight_ss();
          }
          else {
            set_mm = (rotary.getPosition()) % 60;
            disp2(1, set_mm);
          }
        }
        break;
        case STATE_SET_CLOCK_SS:
        {
          // move back to clock mode when button is pressed
          if (button.b1_keyup) {
            g_blank = 0;
            
            rtc.setTime_s(set_hh, set_mm, set_ss);
            
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            g_update_backlight = true;
            rotary.restore();
            g_clock_state = STATE_CLOCK;            
            button.b1_keyup = 0;
          }
          else {
            set_ss = (rotary.getPosition()) % 60;
            
            if (g_digits == 6) {
                disp2(2, set_ss);
            }
            else {
                data[0] = data[1] = 10;
                data[4] = data[5] = 10;
                disp2(1, set_ss);
            }
          }
        }
        break;
        case STATE_TEST_MODE:
        {
          data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = g_test_counter;
          
#if defined(BOARD_STANDARD) || defined(BOARD_MK2)
          if (button.b1_keyup) {
#elif defined(BOARD_DIET) || defined(BOARD_MODULAR)
            if (button.b2_keyup) {
#endif // board type
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            g_update_backlight = true;
            rotary.restore();            
            
            g_clock_state = STATE_CLOCK;            
            button.b1_keyup = 0;
          }
        }
        break;
        case STATE_ANTIPOISON:
        {
          static uint16_t antipoison_counter = 0;

          set_dots(0, 0);

          if (antipoison_counter % 4 == 0) {
            data[0] = rnd() % 10;
            data[2] = rnd() % 10;
            data[4] = rnd() % 10;
          }
          else if (antipoison_counter % 4 == 1) {
            data[1] = rnd() % 10;
            data[3] = rnd() % 10;
            data[5] = rnd() % 10;            
          }
          
          if (antipoison_counter++ == 0x01ff) {
            set_indicator(INDICATOR_OFF);
            g_display_mode = 0;
            g_update_rtc = true;
            g_update_backlight = true;
            rotary.restore();
            
            g_clock_state = STATE_CLOCK;
            antipoison_counter = 0;
          }
        }
        break;
        case STATE_MENU_24H: // menu item 1 (12/24)
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
            data[0] = data[1] = 10;
          }
        }
        break;
        case STATE_MENU_DOTS: // menu item 2 (0, 1, 2)
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
        case STATE_MENU_LEADING_ZEROS: // menu item 3 (0, 1)
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
        case STATE_MENU_WAKE_SOUND: // menu item 4 (0, 1)
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
        case STATE_MENU_SCREEN_SAVER: // menu item 5 (0, 1)
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
        case STATE_MENU_ANTI_POISON: // menu item 6 (0, 1)
        {
#ifdef HAVE_GPS
          if (button.b1_keyup) { // go to STATE_MENU_GPS
            g_antipoison = rotary.getPosition();
            
            rotary.setRange(0, 1);
            rotary.setPosition(g_gps_enabled);
            g_clock_state = STATE_MENU_GPS;
          }
#else          
          if (button.b1_keyup) { // EXIT MENU
            g_antipoison = rotary.getPosition();
          
            exit_menu();
          }
#endif // HAVE_GPS    
          else {
            disp2(0, 6);
            disp2(1, rotary.getPosition());
          }
        }
        break;
#ifdef HAVE_GPS
        case STATE_MENU_GPS: // menu item 7 
        {
          if (button.b1_keyup) { // Go to STATE_MENU_GPS_TZH
            g_gps_enabled = rotary.getPosition();
            
            gps_init(g_gps_enabled ? 48 : 0); // fixme: support both modes
          
            rotary.setDivider(4);
            rotary.setRange(0, 27);
            rotary.setPosition(g_TZ_hour+13);
            g_clock_state = STATE_MENU_GPS_TZH;            
          }
          else {
            disp2(0, 7);
            disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_GPS_TZH: // menu item 8 (-13 to 14)
        {
          if (button.b1_keyup) { // Go to STATE_MENU_GPS_TZM
            g_TZ_hour = rotary.getPosition() - 13;
          
            rotary.setRange(0, 3); // 0, 15, 30, 45 min
            rotary.setPosition(g_TZ_minute);
            g_clock_state = STATE_MENU_GPS_TZM;            
          }
          else {
            int8_t tzh = rotary.getPosition() - 13;
            if (tzh < 0) {
              disp2(1, -tzh);
              set_dots(true, true);
            }
            else {
              disp2(1, tzh);
              set_dots(false, false);
            }
            disp2(0, 8);
            ///disp2(1, rotary.getPosition());
          }
        }
        break;
        case STATE_MENU_GPS_TZM: // menu item 9 (0, 15, 30, 45)
        {
          if (button.b1_keyup) {
            g_TZ_minute = rotary.getPosition();
            
            rotary.setDivider(10);
            rotary.setRange(0, 1);
            rotary.setPosition(g_dst_offset);
            g_clock_state = STATE_MENU_DST_OFFSET;            
          }
          else {
            disp2(0, 9);
            disp2(1, rotary.getPosition()*15);
          }
        }
        break;
        case STATE_MENU_DST_OFFSET: // menu item 10 (0, 1)
        {
          if (button.b1_keyup) {
            g_dst_offset = rotary.getPosition();
            
            exit_menu();
          }
          else {
            disp2(0, 10);
            disp2(1, rotary.getPosition());
          }
        }
        break;

#endif // HAVE_GPS
        case STATE_MENU_LAST: // unused: present to supress compiler warning
        break;
      }
      
      backlight_tick();
      
      if ((g_clock_state == STATE_CLOCK || g_clock_state == STATE_ANTIPOISON) &&
          g_alarm_switch && rtc.checkAlarm())
        start_alarm();
      
      // cooldown for rotary encoder moved
      if (g_rotary_moved_timer > 0)
        g_rotary_moved_timer--;
    }
  }
}

// random number seed
volatile uint32_t lfsr = 0xbeefcacc;

void seed_random(uint32_t seed)
{
    lfsr = seed;
}

uint32_t rnd(void)
{
    // http://en.wikipedia.org/wiki/Linear_feedback_shift_register
    // Galois LFSR: taps: 32 31 29 1; characteristic polynomial: x^32 + x^31 + x^29 + x + 1 */
    lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xD0000001u);
    return lfsr;
}


