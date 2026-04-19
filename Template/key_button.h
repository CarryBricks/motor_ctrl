/*!
    \file    key_button.h
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

#ifndef __KEY_BUTTON_H__
#define __KEY_BUTTON_H__

#include "gd32f30x.h"
#include "led_control.h"

#define KEY_FORWARD_PORT      GPIOB
#define KEY_FORWARD_PIN       GPIO_PIN_9
#define KEY_REVERSE_PORT      GPIOC
#define KEY_REVERSE_PIN       GPIO_PIN_14

#define KEY_SHORT_PRESS_TIME      50
#define KEY_LONG_PRESS_TIME       500
#define KEY_SCAN_INTERVAL         10
#define MOTOR_STEP_PULSE_TIME     50
#define MOTOR_CONTINUOUS_SPEED    200

extern volatile uint8_t forward_key_pressed;
extern volatile uint8_t reverse_key_pressed;
extern volatile uint16_t forward_key_time;
extern volatile uint16_t reverse_key_time;
extern volatile uint8_t forward_key_long_detected;
extern volatile uint8_t reverse_key_long_detected;

void key_button_init(void);
void key_scan(void);

#endif