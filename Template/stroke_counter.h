/*!
    \file    stroke_counter.h
    \brief   stroke counter module header file

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

#ifndef STROKE_COUNTER_H
#define STROKE_COUNTER_H

#include "gd32f30x.h"

// 到位检测引脚定义
#define TOP_POSITION_DETECT_PIN    GPIO_PIN_5
#define TOP_POSITION_DETECT_PORT   GPIOB
#define BOTTOM_POSITION_DETECT_PIN    GPIO_PIN_4
#define BOTTOM_POSITION_DETECT_PORT   GPIOB

// 函数声明
void stroke_counter_init(void);
void stroke_counter_update(void);
void stroke_counter_reset(void);
void stroke_counter_process(void);
uint32_t get_stroke_count(void);    
void position_detect_process(void);


// 外部变量声明
extern volatile uint8_t position_detected;

#endif /* STROKE_COUNTER_H */
