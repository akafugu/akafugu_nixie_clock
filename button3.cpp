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
#include "button.h"

#ifdef HAVE_3_BUTTON_CONTROL

pin_direct_t button1;
pin_direct_t button2;
pin_direct_t button3;

uint8_t button_count = 3;

uint8_t saved_keystatus = 0x00;
uint8_t keydown_keys = 0x00;
uint8_t keyup_keys = 0x00;
uint8_t keyrepeat_keys = 0x00;

uint16_t keyboard_counter[3] = {0, 0, 0};
uint8_t button_bit[3];

#define REPEAT_SPEED	5
volatile uint8_t repeat_speed = REPEAT_SPEED;

// 3 button control:
//   button 1: toggle display mode / enter menu button (LEFT) - PB6
//   button 2: alarm on/off button (RIGHT) - parameter
//   button 3: toggle backlight / change menu selection (MIDDLE) - PB7
//
// PB6 and PB7 do not have Arduino pin numbers assigned, so these are accessed directly
void initialize_button(int8_t pin1, int8_t pin2)
{
    // set as input
    DDRB &= ~(_BV(PORTB6));
    DDRB &= ~(_BV(PORTB7));
    
    // enable pullups
    PORTB |= _BV(PORTB6);
    PORTB |= _BV(PORTB7);
    
    button1.pin = -1;
    button1.reg = &PINB;
    button1.bitmask = _BV(PORTB6);

    button3.pin = -1;
    button3.reg = &PINB;
    button3.bitmask = _BV(PORTB7);
    
    pinMode(pin2, INPUT);
    digitalWrite(pin2, HIGH); // enable pullup

    button2.pin = pin2;
    button2.reg = PIN_TO_INPUT_REG(pin2);
    button2.bitmask = PIN_TO_BITMASK(pin2);

    button_count = 3;
    
    button_bit[0] = button1.bitmask;
    button_bit[1] = button2.bitmask;
    button_bit[2] = button3.bitmask;
}

void button_timer(void)
{
    uint8_t keystatus;

    keystatus = (~(*button1.reg)&(button1.bitmask)) | (~(*button2.reg)&(button2.bitmask)) | (~(*button3.reg)&(button3.bitmask));

    keydown_keys |= (uint8_t)(keystatus & ~(saved_keystatus));
    keyup_keys   |= (uint8_t)(~(keystatus) & saved_keystatus);
    saved_keystatus = keystatus;


    for(uint8_t i = 0; i < button_count; i++) {
        if(~(keydown_keys)&button_bit[i])
            ; // Do nothing, no keyrepeat is needed
        else if(keyup_keys&button_bit[i])
            keyboard_counter[i] = 0;
        else {
            if(keyboard_counter[i] >= repeat_speed) {
                keyrepeat_keys |= button_bit[i];
                keyboard_counter[i] = 0;
            }
            
            keyboard_counter[i]++;
        }
    }
}

void get_button_state(struct BUTTON_STATE* buttons)
{
    buttons->b1_keydown = keydown_keys&button1.bitmask;
    buttons->b1_keyup = keyup_keys&button1.bitmask;
    buttons->b1_repeat = keyrepeat_keys&button1.bitmask;

    if (keyrepeat_keys&button1.bitmask)
      keyrepeat_keys &= ~(button1.bitmask);

    // Reset if we got keyup
    if(keyup_keys&button1.bitmask) {
        keydown_keys   &= ~(button1.bitmask);
        keyup_keys     &= ~(button1.bitmask);
        keyrepeat_keys &= ~(button1.bitmask);
        keyboard_counter[0] = 0;
    }

    buttons->b2_keydown = keydown_keys&button2.bitmask;
    buttons->b2_keyup = keyup_keys&button2.bitmask;
    buttons->b2_repeat = keyrepeat_keys&button2.bitmask;

    if (keyrepeat_keys&button2.bitmask)
      keyrepeat_keys &= ~(button2.bitmask);
	
    // Reset if we got keyup
    if(keyup_keys&button2.bitmask) {
        keydown_keys   &= ~(button2.bitmask);
        keyup_keys     &= ~(button2.bitmask);
        keyrepeat_keys &= ~(button2.bitmask);
        keyboard_counter[0] = 0;
    }

    buttons->b3_keydown = keydown_keys&button3.bitmask;
    buttons->b3_keyup = keyup_keys&button3.bitmask;
    buttons->b3_repeat = keyrepeat_keys&button3.bitmask;
    
    if (keyrepeat_keys&button3.bitmask) {
      keyrepeat_keys &= ~(button3.bitmask);
      if (repeat_speed > 1) repeat_speed--;
    }
	
    // Reset if we got keyup
    if(keyup_keys&button3.bitmask) {
        keydown_keys   &= ~(button3.bitmask);
        keyup_keys     &= ~(button3.bitmask);
        keyrepeat_keys &= ~(button3.bitmask);
        keyboard_counter[0] = 0;
        repeat_speed = REPEAT_SPEED;
    }


    buttons->both_held = (keydown_keys&button1.bitmask) && (keydown_keys&button3.bitmask);
    buttons->none_held = ~(saved_keystatus)&(button1.bitmask) && ~(saved_keystatus)&(button3.bitmask);
}

#endif

