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
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "hall_sensor.h"
#include "motor_control.h"
#include "gd32f30x_exti.h"

// 外部变量声明
extern volatile uint32_t system_tick;

// 全局变量
volatile uint32_t hall_count = 0;
volatile uint16_t actual_speed = 0;

// 局部变量
static uint8_t last_hall_state = 0;
static uint32_t last_hall_time = 0;
static uint32_t speed_calculation_interval = 100; // 100ms
static uint32_t last_speed_calculation_time = 0;
static uint32_t hall_count_in_interval = 0;

// 外部函数声明
extern void delay_decrement(void);

// EXTI0 中断处理（Hall sensor A 通道）
void EXTI0_IRQHandler(void)
{
    if(exti_flag_get(EXTI_0)){
        /* update hall sensor state */
        hall_sensor_update();
        /* clear the EXTI0 pending bit */
        exti_flag_clear(EXTI_0);
    }
}

// EXTI1 中断处理（Hall sensor B 通道）
void EXTI1_IRQHandler(void)
{
    if(exti_flag_get(EXTI_1)){
        /* update hall sensor state */
        hall_sensor_update();
        /* clear the EXTI1 pending bit */
        exti_flag_clear(EXTI_1);
    }
}

// EXTI10_15 中断处理（Hall sensor PB6/PB7）
void EXTI10_15_IRQHandler(void)
{
    if(exti_flag_get(EXTI_6)){
        /* update hall sensor state */
        hall_sensor_update();
        /* clear the EXTI6 pending bit */
        exti_flag_clear(EXTI_6);
    }
    
    if(exti_flag_get(EXTI_7)){
        /* update hall sensor state */
        hall_sensor_update();
        /* clear the EXTI7 pending bit */
        exti_flag_clear(EXTI_7);
    }
}

/*!
    \brief      Hall sensor initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hall_sensor_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /* configure Hall sensor pins */
    gpio_init(HALL_EA_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, HALL_EA_PIN);
    gpio_init(HALL_EB_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, HALL_EB_PIN);
    
    /* configure EXTI for Hall sensors */
    rcu_periph_clock_enable(RCU_AF);
    
    /* connect EXTI line to Hall sensor pins */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_6);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
    
    /* configure EXTI line */
    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_flag_clear(EXTI_7);
    
    /* enable and set EXTI interrupt to the lowest priority */
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);
}

/*!
    \brief      process Hall sensor data
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hall_sensor_process(void)
{
    /* read current time */
    uint32_t current_time = system_tick;
    
    /* calculate speed every 100ms */
    if (current_time - last_speed_calculation_time >= speed_calculation_interval) {
        /* calculate speed (RPM) */
        /* assuming 4 poles motor, 2 Hall edges per revolution */
        actual_speed = (hall_count_in_interval * 60 * 1000) / (2 * speed_calculation_interval);
        
        /* reset counter */
        hall_count_in_interval = 0;
        last_speed_calculation_time = current_time;
    }
}

/*!
    \brief      update Hall sensor state and count
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hall_sensor_update(void)
{
    /* read current Hall state */
    uint8_t hall_a = gpio_input_bit_get(HALL_EA_PORT, HALL_EA_PIN);
    uint8_t hall_b = gpio_input_bit_get(HALL_EB_PORT, HALL_EB_PIN);
    uint8_t current_state = (hall_a << 1) | hall_b;
    
    /* update hall count based on Hall state change */
    if (current_state != last_hall_state) {
        hall_count++;
        hall_count_in_interval++;
        last_hall_state = current_state;
    }
}