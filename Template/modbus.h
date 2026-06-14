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
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MODBUS_H
#define MODBUS_H

#include "gd32f30x.h"

// 协议帧边界
#define CUSTOM_SOF          0xA5
#define CUSTOM_EOF          0x5A

// 设备配置
#define SLAVE_ADDR           0x01
#define CUSTOM_BAUDRATE      115200
#define CUSTOM_TIMEOUT       1000

// 功能码
#define FC_CONTROL          0x03
#define FC_RESPONSE         0x81
#define FC_EVENT            0x82

// 控制命令
#define CMD_FORWARD         0x0001
#define CMD_REVERSE         0x0002
#define CMD_STOP            0x0003

// 事件类型
#define EVENT_STALL         0x0001

// 函数声明
void modbus_init(void);
void modbus_process(void);
void modbus_upload_status(void);
void modbus_send_event(uint8_t event_type, uint16_t current);

#endif