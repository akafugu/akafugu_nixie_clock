/*
 * VFD Modular Clock
 * (C) 2011 Akafugu Corporation
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

pin_direct_t button1;
pin_direct_t button2;

void initialize_button(uint8_t pin1, uint8_t pin2)
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
    }
}

bool is_button_pressed(void)
{
    if (*button1.reg & button1.bitmask)
        return true;
    return false;
}

uint8_t saved_keystatus = 0x00;
uint8_t keydown_keys = 0x00;
uint8_t keyup_keys = 0x00;
uint8_t keyrepeat_keys = 0x00;

uint16_t keyboard_counter[1] = {0};
uint8_t button_bit[1] = { button1.bitmask };

//#define REPEAT_SPEED	2000
#define REPEAT_SPEED	20

void button_timer(void)
{
    	uint8_t keystatus = ~(*button1.reg)&(button1.bitmask);
	keydown_keys |= (uint8_t)(keystatus & ~(saved_keystatus));
	keyup_keys   |= (uint8_t)(~(keystatus) & saved_keystatus);
	saved_keystatus = keystatus;
	
	if(~(keydown_keys)&button_bit[0])
		; // Do nothing, no keyrepeat is needed
	else if(keyup_keys&button_bit[0])
		keyboard_counter[0] = 0;
	else {
		if(keyboard_counter[0] >= REPEAT_SPEED) {
			keyrepeat_keys |= button_bit[0];
			keyboard_counter[0] = 0;
		}
		keyboard_counter[0]++;
	}
}

void get_button_state(struct BUTTON_STATE* buttons)
{
	buttons->b1_keydown = keydown_keys&button1.bitmask;
	buttons->b1_keyup = keyup_keys&button1.bitmask;
	buttons->b1_repeat = keyrepeat_keys&button1.bitmask;
	
	// Reset if we got keyup
	if(keyup_keys&button1.bitmask)
	{
		keydown_keys   &= ~(button1.bitmask);
		keyup_keys     &= ~(button1.bitmask);
		keyrepeat_keys &= ~(button1.bitmask);
		keyboard_counter[0] = 0;
	}

    buttons->none_held = ~(saved_keystatus)&(button1.bitmask);
}

