/*!
    \file    current_sensor.h
    \brief   current sensor module header file

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

#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "gd32f30x.h"

// 硬件定义 - 电流检测使用PA5和PA6两路ADC
#define CURRENT_ADC1_CHANNEL  ADC_CHANNEL_5
#define CURRENT_ADC1_PORT     GPIOA
#define CURRENT_ADC1_PIN      GPIO_PIN_5

#define CURRENT_ADC2_CHANNEL  ADC_CHANNEL_6
#define CURRENT_ADC2_PORT     GPIOA
#define CURRENT_ADC2_PIN      GPIO_PIN_6

// 函数声明
void current_sensor_init(void);
uint16_t read_motor_current(void);
void check_stall_current(void);

// 外部变量声明
extern volatile uint16_t current_value;
extern volatile uint16_t current_adc1_value;
extern volatile uint16_t current_adc2_value;

// 电流阈值定义
#define OVERLOAD_CURRENT_THRESHOLD    3000    // 过载电流阈值 (mA)
#define STALL_CURRENT_THRESHOLD       4000    // 堵转电流阈值 (mA)

#endif /* CURRENT_SENSOR_H */
