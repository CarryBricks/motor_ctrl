/*!
    \file    modbus.h
    \brief   Modbus RTU communication module header file

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

#ifndef MODBUS_H
#define MODBUS_H

#include "gd32f30x.h"

// Modbus配置
#define MODBUS_SLAVE_ADDR     0x01
#define MODBUS_BUFFER_SIZE    256
#define MODBUS_TIMEOUT        1000    // 1000ms

// Modbus功能码
#define MODBUS_FC_READ_HOLDING_REGISTERS   0x03
#define MODBUS_FC_WRITE_SINGLE_REGISTER    0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10

// Modbus错误码
#define MODBUS_ERR_ILLEGAL_FUNCTION       0x01
#define MODBUS_ERR_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_ERR_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_ERR_SLAVE_DEVICE_FAILURE   0x04

// Modbus寄存器定义
#define REG_MOTOR_STATE      0x0000  // 电机状态
#define REG_CONTROL_MODE     0x0001  // 控制模式
#define REG_TARGET_SPEED     0x0002  // 目标速度
#define REG_ACTUAL_SPEED     0x0003  // 实际速度
#define REG_STROKE_COUNT     0x0004  // 行程计数
#define REG_MOTOR_CURRENT    0x0005  // 电机电流
#define REG_TEMPERATURE      0x0006  // 温度
#define REG_POSITION_DETECT  0x0007  // 到位检测
#define REG_COMMAND          0x0008  // 控制命令

// 函数声明
void modbus_init(void);
void modbus_process(void);
void modbus_send_response(uint8_t *buffer, uint16_t length);

// 外部变量声明
extern uint8_t modbus_rx_buffer[];
extern uint8_t modbus_tx_buffer[];
extern uint16_t modbus_rx_length;
extern uint16_t modbus_tx_length;
extern uint32_t modbus_rx_timeout;

#endif /* MODBUS_H */
