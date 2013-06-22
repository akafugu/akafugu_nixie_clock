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

#ifndef HAVE_3_BUTTON_CONTROL

pin_direct_t button1;
pin_direct_t button2;

uint8_t button_count = 1;

uint8_t saved_keystatus = 0x00;
uint8_t keydown_keys = 0x00;
uint8_t keyup_keys = 0x00;
uint8_t keyrepeat_keys = 0x00;

uint16_t keyboard_counter[2] = {0, 0};
uint8_t button_bit[2];

#define REPEAT_SPEED	5

void initialize_button(int8_t pin1, int8_t pin2)
{
    pinMode(pin1, INPUT);
    digitalWrite(pin1, HIGH); // enable pullup
    
    button1.pin = pin1;
    button1.reg = PIN_TO_INPUT_REG(pin1);
    button1.bitmask =   PIN_TO_BITMASK(pin1);
    
    if (pin2 != -1) {
        pinMode(pin2, INPUT);
        digitalWrite(pin2, HIGH); // enable pullup

        button2.pin = pin2;
        button2.reg = PIN_TO_INPUT_REG(pin2);
        button2.bitmask = PIN_TO_BITMASK(pin2);
        
        button_count = 2;
    }
    
    button_bit[0] = button1.bitmask;
    button_bit[1] = button2.bitmask;
}

void button_timer(void)
{
    uint8_t keystatus;

    if (button_count == 2)
        keystatus = (~(*button1.reg)&(button1.bitmask)) | (~(*button2.reg)&(button2.bitmask));
    else
        keystatus = ~(*button1.reg)&(button1.bitmask);

    keydown_keys |= (uint8_t)(keystatus & ~(saved_keystatus));
    keyup_keys   |= (uint8_t)(~(keystatus) & saved_keystatus);
    saved_keystatus = keystatus;
	
    for(uint8_t i = 0; i < button_count; i++) {
        if(~(keydown_keys)&button_bit[i])
            ; // Do nothing, no keyrepeat is needed
        else if(keyup_keys&button_bit[i])
            keyboard_counter[i] = 0;
        else {
            if(keyboard_counter[i] >= REPEAT_SPEED) {
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

    if (button_count == 2) {
        buttons->b2_keydown = keydown_keys&button2.bitmask;
        buttons->b2_keyup = keyup_keys&button2.bitmask;
        buttons->b2_repeat = keyrepeat_keys&button2.bitmask;
	
        if (keyrepeat_keys&button1.bitmask)
          keyrepeat_keys &= ~(button1.bitmask);

        // Reset if we got keyup
        if(keyup_keys&button2.bitmask) {
            keydown_keys   &= ~(button2.bitmask);
            keyup_keys     &= ~(button2.bitmask);
            keyrepeat_keys &= ~(button2.bitmask);
            keyboard_counter[0] = 0;
        }

        buttons->both_held = (keydown_keys&button1.bitmask) && (keydown_keys&button2.bitmask);
        buttons->none_held = ~(saved_keystatus)&(button1.bitmask) && ~(saved_keystatus)&(button2.bitmask);
    }
    else {
        buttons->both_held = (keydown_keys&button1.bitmask);
        buttons->none_held = ~(saved_keystatus)&(button1.bitmask);      
    }
}

#endif // HAVE_3_BUTTON_CONTROL
