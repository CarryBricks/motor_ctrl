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
#include <stdio.h>



volatile uint32_t hall_count = 0;
volatile uint16_t actual_speed = 0;
volatile uint16_t motor_rpm = 0;

static uint8_t last_hall_state = 0;
static uint32_t last_hall_time = 0;
static uint32_t speed_calculation_interval = 100;
static uint32_t last_speed_calculation_time = 0;
static uint32_t hall_count_in_interval = 0;
static uint32_t last_full_rotation_time = 0;
static uint8_t last_hall_ab_state = 0;
static uint16_t rpm_filtered = 0;
static uint32_t hall_11_count = 0;
static uint32_t hall_11_count_last_time = 0;

#define RPM_MIN_TIME_DIFF_MS   5
#define RPM_MAX_VALUE          4700
#define RPM_SMOOTH_FACTOR      80

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

// PB6中断计数（全局变量，供主循环读取）
static uint32_t pb6_int_count = 0;
static uint32_t pb6_int_count_last_time = 0;
extern uint32_t pb6_int_count_per_sec;  // 每秒中断次数（对外暴露）
uint32_t pb6_int_count_per_sec = 0;

// EXTI5~9 共用中断入口
void EXTI5_9_IRQHandler(void)
{
    uint8_t hall_a, hall_b, current_state;
    static uint8_t last_hall_state = 0;

  

    #if  1
    // 读取实时霍尔电平
    hall_a = gpio_input_bit_get(GPIOB, GPIO_PIN_6) ? 1 : 0;
    hall_b = gpio_input_bit_get(GPIOB, GPIO_PIN_7) ? 1 : 0;
    current_state = (hall_a << 1) | hall_b;

    // 仅状态切换才计数，重复状态忽略
    if(current_state != last_hall_state)
    {
        last_hall_state = current_state;
        hall_count++;

        uint32_t current_time = get_system_tick();
        static uint32_t last_pulse_count = 0;
        if (current_time - last_speed_calculation_time >= speed_calculation_interval)
        {
            last_speed_calculation_time = current_time;

            uint32_t pulse_diff = hall_count - last_pulse_count;
            last_pulse_count = hall_count;
            motor_rpm = (pulse_diff * 600) / 4;
           // printf("RPM=%d\n", motor_rpm);
        }
    }
#endif
    // 清除中断标志
    if(exti_interrupt_flag_get(EXTI_6))
        exti_interrupt_flag_clear(EXTI_6);
    if(exti_interrupt_flag_get(EXTI_7))
        exti_interrupt_flag_clear(EXTI_7);
    
    #if 0
    // PB6 中断
    if(exti_interrupt_flag_get(EXTI_6) == SET)
    {
        exti_interrupt_flag_clear(EXTI_6);
        // ======================
        // PB6 下降沿要执行的代码
        // ======================
        hall_sensor_update();
    }

    // PB7 中断
    if(exti_interrupt_flag_get(EXTI_7) == SET)
    {
        exti_interrupt_flag_clear(EXTI_7);

        // ======================
        // PB7 下降沿要执行的代码
        // ======================
        hall_sensor_update();
    }
        #endif
}





void hall_sensor_init(void)
{
    // 开启时钟
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    // 配置 PB6、PB7 为上拉输入
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    // PB6 → EXTI6 下降沿触发
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_6);
    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_6);

    // PB7 → EXTI7 下降沿触发
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_7);

    // 配置中断优先级
    nvic_irq_enable(EXTI5_9_IRQn, 2, 0);

    last_full_rotation_time = get_system_tick();
}

void hall_sensor_process(void)
{
    #if 0
    uint32_t current_time = get_system_tick();
    static uint32_t last_pulse_count = 0;


    if (current_time - last_speed_calculation_time >= speed_calculation_interval) 
    {
        //actual_speed = (hall_count_in_interval * 60 * 1000) / (2 * speed_calculation_interval);

        //hall_count_in_interval = 0;
        last_speed_calculation_time = current_time;

        uint32_t pulse_diff = hall_count - last_pulse_count;
        last_pulse_count = hall_count;
        motor_rpm = (pulse_diff * 600) / 4;
        printf("RPM=%d\n", motor_rpm);
    }
    #endif
}

void hall_sensor_update(void)
{
    uint8_t hall_a = gpio_input_bit_get(HALL_EA_PORT, HALL_EA_PIN);
    uint8_t hall_b = gpio_input_bit_get(HALL_EB_PORT, HALL_EB_PIN);
    uint8_t current_state = (hall_a << 1) | hall_b;
    uint8_t current_hall_ab_state = (hall_a << 1) | hall_b;

    printf("hall_a=%d, hall_b=%d\n", hall_a, hall_b);

    if (hall_a == 1 && hall_b == 1)
    {
        hall_11_count++;
        uint32_t current_time = get_system_tick();
        if (current_time - hall_11_count_last_time >= 1000)
        {
            printf("Hall_11_count_per_second=%d\n", hall_11_count);
            hall_11_count = 0;
            hall_11_count_last_time = current_time;
        }

        if (last_hall_ab_state != 0x03)
        {
            uint32_t time_diff = current_time - last_full_rotation_time;

           // printf("time_diff=%d\n", time_diff);

            if (time_diff > RPM_MIN_TIME_DIFF_MS && time_diff < 60000)
            {
                uint16_t raw_rpm = (uint16_t)((60 * 1000) / time_diff);

                if (raw_rpm > RPM_MAX_VALUE)
                {
                    raw_rpm = RPM_MAX_VALUE;
                }

                if (rpm_filtered == 0)
                {
                    rpm_filtered = raw_rpm;
                }
                else
                {
                    rpm_filtered = (rpm_filtered * (100 - RPM_SMOOTH_FACTOR) + raw_rpm * RPM_SMOOTH_FACTOR) / 100;
                }

                motor_rpm = rpm_filtered;
                //printf("RPM=%d\n", motor_rpm);
            }
            else if (time_diff >= 60000)
            {
                motor_rpm = 0;
                rpm_filtered = 0;
            }

            last_full_rotation_time = current_time;
        }
    }

    last_hall_ab_state = current_hall_ab_state;

    if (current_state != last_hall_state) {
        hall_count++;
        hall_count_in_interval++;
        last_hall_state = current_state;
    }
}




// void test_pb6_low_cont_in_1_second(void)
// {
//     #if 0
//     static uint32_t last_time = 0;
//     uint32_t current_time = get_system_tick();
    
//     // 每隔10ms读取一次PB6电平
//     if (current_time - last_time >= 1)
//     {
//         last_time = current_time;
//         uint8_t pb6_level = gpio_input_bit_get(GPIOB, GPIO_PIN_6);
//         printf("PB6=%d\n", pb6_level);
//     }
//     #endif
// }

// PB6中断统计函数（放在主循环中调用）
void test_pb6_low_cont_in_1_second(void)
{
    #if 0
    uint32_t current_time = get_system_tick();
    if (current_time - pb6_int_count_last_time >= 1000)
    {
        pb6_int_count_per_sec = pb6_int_count;
        printf("PB6_INT_PER_SEC=%d\n", pb6_int_count_per_sec);
        pb6_int_count = 0;
        pb6_int_count_last_time = current_time;
    }
    #endif
}