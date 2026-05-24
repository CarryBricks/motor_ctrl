/*!
    \file    led_control.c
    \brief   LED control driver

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

#include "led_control.h"

void led_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOA);

    /* 配置 PA15 为普通 GPIO（禁用 JTAG/SWD 功能） */
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    gpio_init(LED_FORWARD_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_FORWARD_PIN);
    gpio_init(LED_REVERSE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_REVERSE_PIN);

    led_off(LED_FORWARD);
    led_off(LED_REVERSE);
}

void led_on(led_id_enum led)
{
    switch(led) {
        case LED_FORWARD:
            gpio_bit_reset(LED_FORWARD_PORT, LED_FORWARD_PIN);
            break;
        case LED_REVERSE:
            gpio_bit_reset(LED_REVERSE_PORT, LED_REVERSE_PIN);
            break;
        case LED_BOTH:
            gpio_bit_reset(LED_FORWARD_PORT, LED_FORWARD_PIN);
            gpio_bit_reset(LED_REVERSE_PORT, LED_REVERSE_PIN);
            break;
    }
}

void led_off(led_id_enum led)
{
    switch(led) {
        case LED_FORWARD:
            gpio_bit_set(LED_FORWARD_PORT, LED_FORWARD_PIN);
            break;
        case LED_REVERSE:
            gpio_bit_set(LED_REVERSE_PORT, LED_REVERSE_PIN);
            break;
        case LED_BOTH:
            gpio_bit_set(LED_FORWARD_PORT, LED_FORWARD_PIN);
            gpio_bit_set(LED_REVERSE_PORT, LED_REVERSE_PIN);
            break;
    }
}

void led_toggle(led_id_enum led)
{
    switch(led) {
        case LED_FORWARD:
            gpio_bit_write(LED_FORWARD_PORT, LED_FORWARD_PIN,
                          (bit_status)(1 - gpio_input_bit_get(LED_FORWARD_PORT, LED_FORWARD_PIN)));
            break;
        case LED_REVERSE:
            gpio_bit_write(LED_REVERSE_PORT, LED_REVERSE_PIN,
                          (bit_status)(1 - gpio_input_bit_get(LED_REVERSE_PORT, LED_REVERSE_PIN)));
            break;
        case LED_BOTH:
            gpio_bit_write(LED_FORWARD_PORT, LED_FORWARD_PIN,
                          (bit_status)(1 - gpio_input_bit_get(LED_FORWARD_PORT, LED_FORWARD_PIN)));
            gpio_bit_write(LED_REVERSE_PORT, LED_REVERSE_PIN,
                          (bit_status)(1 - gpio_input_bit_get(LED_REVERSE_PORT, LED_REVERSE_PIN)));
            break;
    }
}