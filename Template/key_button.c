/*!
    \file    key_button.c
    \brief   key button driver for motor control (state machine implementation)

    \version 2026-4-19, V1.0.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce in the documentation and/or
       other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
    IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "key_button.h"
#include "systick.h"
#include "motor_control.h"
#include <stdio.h>


typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED,
    KEY_STATE_LONG_PRESSED,
    KEY_STATE_RELEASE
} key_state_t;

typedef struct {
    key_state_t state;
    uint32_t press_start_time;
    uint8_t long_press_reported;
    uint8_t debounce_cnt;
    uint8_t last_raw;
} key_info_t;

static key_info_t forward_key = {KEY_STATE_IDLE, 0, 0, 0, 0};
static key_info_t reverse_key = {KEY_STATE_IDLE, 0, 0, 0, 0};

void key_button_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(KEY_FORWARD_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, KEY_FORWARD_PIN);
    gpio_init(KEY_REVERSE_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, KEY_REVERSE_PIN);

    led_init();
}

static void key_update(key_info_t *key, uint8_t is_pressed, uint8_t direction)
{
    uint32_t current_time = get_system_tick();

    if (is_pressed == key->last_raw) {
        if (key->debounce_cnt < 4) key->debounce_cnt++;
    } else {
        key->debounce_cnt = 0;
        key->last_raw = is_pressed;
        return;
    }
    if (key->debounce_cnt < 4) return;
    uint8_t stable = is_pressed;

    switch (key->state) 
    {
        case KEY_STATE_IDLE:
            if (stable) 
            {
                key->state = KEY_STATE_PRESSED;
                key->press_start_time = current_time;
                key->long_press_reported = 0;
            }
            break;

        case KEY_STATE_PRESSED:
            if (stable) 
            {
                uint32_t press_duration = current_time - key->press_start_time;
                if (press_duration >= KEY_LONG_PRESS_TIME) 
                {
                    key->state = KEY_STATE_LONG_PRESSED;
                    key->long_press_reported = 1;
                    motor_set_speed(MOTOR_CONTINUOUS_SPEED, direction);
                    if (direction == 0) 
                    {
                        led_on(LED_FORWARD);
                    } 
                    else 
                    {
                        led_on(LED_REVERSE);
                    }
                    
                }
            } 
            else 
            {
                uint32_t press_duration = current_time - key->press_start_time;
                if (press_duration < KEY_LONG_PRESS_TIME) 
                {
                    motor_set_speed(MOTOR_CONTINUOUS_SPEED, direction);
                    delay_1ms(MOTOR_STEP_PULSE_TIME);
                    motor_stop();  
                    
                }
                key->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_LONG_PRESSED:
            if (!stable) 
            {
                motor_stop();
                if (direction == 0) 
                {
                    led_off(LED_FORWARD);
                } 
                else 
                {
                    led_off(LED_REVERSE);
                }
                key->state = KEY_STATE_IDLE;
                
            }
            break;

        default:
            key->state = KEY_STATE_IDLE;
            break;
    }
}

void key_scan(void)
{
    uint8_t forward_key_state = !gpio_input_bit_get(KEY_FORWARD_PORT, KEY_FORWARD_PIN);
    uint8_t reverse_key_state = !gpio_input_bit_get(KEY_REVERSE_PORT, KEY_REVERSE_PIN);

    key_update(&forward_key, forward_key_state, 0);
    key_update(&reverse_key, reverse_key_state, 1);
}

uint8_t key_forward_get_state(void)
{
    return forward_key.state;
}

uint8_t key_reverse_get_state(void)
{
    return reverse_key.state;
}