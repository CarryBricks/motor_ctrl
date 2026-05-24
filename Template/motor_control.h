/*!
    \file    motor_control.h
    \brief   motor control module header file

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

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "gd32f30x.h"
#include <stdbool.h>
// 硬件定义好
//#define MOTOR_ENABLE_PIN     GPIO_PIN_0
//#define MOTOR_ENABLE_PORT    GPIOC
#define HIN1_PIN             GPIO_PIN_14
#define HIN1_PORT            GPIOB
#define LIN1_PIN             GPIO_PIN_9
#define LIN1_PORT            GPIOA
#define HIN2_PIN             GPIO_PIN_13
#define HIN2_PORT            GPIOB
#define LIN2_PIN             GPIO_PIN_8
#define LIN2_PORT            GPIOA

// 电机状态定义
typedef enum {
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_REVERSE,
    MOTOR_STATE_BRAKE
} motor_state_t;

// 控制模式定义
typedef enum {
    MODE_OPEN_LOOP = 0,
    MODE_CLOSED_LOOP
} control_mode_t;

// 电机参数定义
#define MAX_SPEED            4700         //1000    // 最大速度 (RPM)
#define MAX_CURRENT          3000    // 最大电流 (mA)
#define MAX_STROKE           10000   // 最大行程 (脉冲数)

// Modbus命令定义
#define CMD_STOP             0x00    // 停止
#define CMD_FORWARD          0x01    // 正向运行
#define CMD_REVERSE          0x02    // 反向运行
#define CMD_BRAKE            0x03    // 刹车

// 函数声明
void motor_control_init(void);
void motor_set_speed(uint16_t speed, bool direction);
void motor_brake(void);
void motor_stop(void);

// 外部变量声明
extern volatile motor_state_t motor_state;
extern volatile control_mode_t control_mode;
extern volatile uint16_t target_speed;
extern volatile uint16_t actual_speed;
void motor_control_test(uint8_t cmd);
void motor_over_current_process(void);





#endif /* MOTOR_CONTROL_H */
