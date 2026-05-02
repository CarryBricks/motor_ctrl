/*!
    \file    hall_sensor.c
    \brief   hall sensor module implementation

    \version 2026-4-10, V1.0.0, firmware for GD32F30x
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
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "hall_sensor.h"
#include "systick.h"
#include "motor_control.h"
#include "gd32f30x_exti.h"

volatile uint32_t hall_count = 0;
volatile uint16_t actual_speed = 0;

static uint8_t last_hall_state = 0;
static uint32_t last_hall_time = 0;
static uint32_t speed_calculation_interval = 100;
static uint32_t last_speed_calculation_time = 0;
static uint32_t hall_count_in_interval = 0;

extern void delay_decrement(void);

void EXTI0_IRQHandler(void)
{
    if(exti_flag_get(EXTI_0)){
        hall_sensor_update();
        exti_flag_clear(EXTI_0);
    }
}

void EXTI1_IRQHandler(void)
{
    if(exti_flag_get(EXTI_1)){
        hall_sensor_update();
        exti_flag_clear(EXTI_1);
    }
}

void EXTI10_15_IRQHandler(void)
{
    if(exti_flag_get(EXTI_6)){
        hall_sensor_update();
        exti_flag_clear(EXTI_6);
    }

    if(exti_flag_get(EXTI_7)){
        hall_sensor_update();
        exti_flag_clear(EXTI_7);
    }
}

void hall_sensor_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_init(HALL_EA_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, HALL_EA_PIN);
    gpio_init(HALL_EB_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, HALL_EB_PIN);

    rcu_periph_clock_enable(RCU_AF);

    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_6);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);

    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_flag_clear(EXTI_7);

    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);
}

void hall_sensor_process(void)
{
    uint32_t current_time = get_system_tick();

    if (current_time - last_speed_calculation_time >= speed_calculation_interval) {
        actual_speed = (hall_count_in_interval * 60 * 1000) / (2 * speed_calculation_interval);

        hall_count_in_interval = 0;
        last_speed_calculation_time = current_time;
    }
}

void hall_sensor_update(void)
{
    uint8_t hall_a = gpio_input_bit_get(HALL_EA_PORT, HALL_EA_PIN);
    uint8_t hall_b = gpio_input_bit_get(HALL_EB_PORT, HALL_EB_PIN);
    uint8_t current_state = (hall_a << 1) | hall_b;

    if (current_state != last_hall_state) {
        hall_count++;
        hall_count_in_interval++;
        last_hall_state = current_state;
    }
}