/*!
    \file    key_button.c
    \brief   key button driver for motor control

    \version 2026-4-18, V1.0.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
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
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "key_button.h"
#include "systick.h"
#include "motor_control.h"

volatile uint8_t forward_key_pressed = 0;
volatile uint8_t reverse_key_pressed = 0;
volatile uint16_t forward_key_time = 0;
volatile uint16_t reverse_key_time = 0;
volatile uint8_t forward_key_long_detected = 0;
volatile uint8_t reverse_key_long_detected = 0;

void key_button_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(KEY_FORWARD_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, KEY_FORWARD_PIN);
    gpio_init(KEY_REVERSE_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, KEY_REVERSE_PIN);

    led_init();
}

void key_scan(void)
{
    uint8_t forward_key_state = !gpio_input_bit_get(KEY_FORWARD_PORT, KEY_FORWARD_PIN);
    uint8_t reverse_key_state = !gpio_input_bit_get(KEY_REVERSE_PORT, KEY_REVERSE_PIN);

    if (forward_key_state) {
        if (!forward_key_pressed) {
            forward_key_pressed = 1;
            forward_key_time = 0;
            forward_key_long_detected = 0;
        } else {
            forward_key_time += KEY_SCAN_INTERVAL;

            if (forward_key_time >= KEY_LONG_PRESS_TIME && !forward_key_long_detected) {
                forward_key_long_detected = 1;
                motor_set_speed(MOTOR_CONTINUOUS_SPEED, 0);
                led_on(LED_FORWARD);
            }
        }
    } else {
        if (forward_key_pressed && !forward_key_long_detected) {
            motor_set_speed(MOTOR_CONTINUOUS_SPEED, 0);
            delay_1ms(MOTOR_STEP_PULSE_TIME);
            motor_stop();
        }
        forward_key_pressed = 0;
        forward_key_time = 0;
        forward_key_long_detected = 0;
        if (!reverse_key_long_detected) {
            motor_stop();
            led_off(LED_FORWARD);
        }
    }

    if (reverse_key_state) {
        if (!reverse_key_pressed) {
            reverse_key_pressed = 1;
            reverse_key_time = 0;
            reverse_key_long_detected = 0;
        } else {
            reverse_key_time += KEY_SCAN_INTERVAL;

            if (reverse_key_time >= KEY_LONG_PRESS_TIME && !reverse_key_long_detected) {
                reverse_key_long_detected = 1;
                motor_set_speed(MOTOR_CONTINUOUS_SPEED, 1);
                led_on(LED_REVERSE);
            }
        }
    } else {
        if (reverse_key_pressed && !reverse_key_long_detected) {
            motor_set_speed(MOTOR_CONTINUOUS_SPEED, 1);
            delay_1ms(MOTOR_STEP_PULSE_TIME);
            motor_stop();
        }
        reverse_key_pressed = 0;
        reverse_key_time = 0;
        reverse_key_long_detected = 0;
        if (!forward_key_long_detected) {
            motor_stop();
            led_off(LED_REVERSE);
        }
    }
}