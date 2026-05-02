/*!
    \file    stroke_counter.c
    \brief   stroke counter module implementation

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
#include "gd32f30x.h"
#include "stroke_counter.h"
#include "motor_control.h"
// 全局变量
static uint32_t stroke_count = 0;
volatile uint8_t position_detected = 0;
volatile uint8_t top_position_detected = 0;
volatile uint8_t bottom_position_detected = 0;

/*!
    \brief      stroke counter initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void stroke_counter_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /* configure top position detect pin */
    gpio_init(TOP_POSITION_DETECT_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, TOP_POSITION_DETECT_PIN);
    
    /* configure bottom position detect pin */
    gpio_init(BOTTOM_POSITION_DETECT_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BOTTOM_POSITION_DETECT_PIN);
}

/*!
    \brief      update stroke counter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void stroke_counter_update(void)
{
    /* increment stroke count */
    stroke_count++;
    
    /* check top position detection */
    top_position_detected = gpio_input_bit_get(TOP_POSITION_DETECT_PORT, TOP_POSITION_DETECT_PIN);
    
    /* check bottom position detection */
    bottom_position_detected = gpio_input_bit_get(BOTTOM_POSITION_DETECT_PORT, BOTTOM_POSITION_DETECT_PIN);
    
    /* set position_detected if either top or bottom is detected */
    position_detected = top_position_detected || bottom_position_detected;
}

/*!
    \brief      reset stroke counter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void stroke_counter_reset(void)
{
    stroke_count = 0;
}


uint32_t get_stroke_count(void)
{
    return stroke_count;
}


void position_detect_process(void)
{
    /* check position detection */
    top_position_detected = gpio_input_bit_get(TOP_POSITION_DETECT_PORT, TOP_POSITION_DETECT_PIN);
    bottom_position_detected = gpio_input_bit_get(BOTTOM_POSITION_DETECT_PORT, BOTTOM_POSITION_DETECT_PIN);
    position_detected = top_position_detected || bottom_position_detected;
    
    if (position_detected) 
    {
        motor_stop();
        if (top_position_detected) 
        {
            //printf("\r\nTop position detected");
        } 
        else if (bottom_position_detected) 
        {
            //printf("\r\nBottom position detected");
        }
        
        /* increment stroke count on position detection */
        stroke_counter_update();
        
        /* simple debounce delay */
        //for(i=0;i<60000;i++);
    }
}