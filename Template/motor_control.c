/*!
    \file    motor_control.c
    \brief   motor control module implementation

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

#include "motor_control.h"
#include "systick.h"

// 全局变量
volatile motor_state_t motor_state = MOTOR_STATE_STOP;
volatile control_mode_t control_mode = MODE_OPEN_LOOP;
volatile uint16_t target_speed = 0;

// 外部变量声明
extern volatile uint16_t actual_speed;

/*!
    \brief      motor control initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_control_init(void)
{
    /* enable GPIO clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    
    /* enable TIMER clock for PWM */
    rcu_periph_clock_enable(RCU_TIMER1);
    
    /* configure motor enable pin */
    gpio_init(MOTOR_ENABLE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, MOTOR_ENABLE_PIN);
    gpio_bit_reset(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
    
    /* configure H-bridge control pins */
    gpio_init(HIN1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, HIN1_PIN);
    gpio_init(LIN1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LIN1_PIN);
    gpio_init(HIN2_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, HIN2_PIN);
    gpio_init(LIN2_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LIN2_PIN);
    
    /* reset all control pins */
    gpio_bit_reset(HIN1_PORT, HIN1_PIN);
    gpio_bit_reset(LIN1_PORT, LIN1_PIN);
    gpio_bit_reset(HIN2_PORT, HIN2_PIN);
    gpio_bit_reset(LIN2_PORT, LIN2_PIN);
    
    /* configure TIMER1 for PWM */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    
    timer_deinit(TIMER1);
    
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 71;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);
    
    /* CH0, CH1 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocinitpara);
    
    /* CH0, CH1 configuration in PWM mode0 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    
    /* start TIMER1 */
    timer_enable(TIMER1);
}

/*!
    \brief      set motor speed and direction
    \param[in]  speed: target speed (0-1000 RPM)
    \param[in]  direction: 0 for forward, 1 for reverse
    \param[out] none
    \retval     none
*/
void motor_set_speed(uint16_t speed, uint8_t direction)
{
    extern volatile uint32_t stroke_count;
    
    if (stroke_count >= MAX_STROKE) {
        motor_stop();
        return;
    }
    
    if (speed == 0) {
        motor_stop();
        return;
    }
    
    /* calculate PWM duty cycle (0-999) */
    uint16_t duty = (speed * 999) / MAX_SPEED;
    
    /* enable motor */
    gpio_bit_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
    
    if (direction == 0) {
        // Forward
        motor_state = MOTOR_STATE_FORWARD;
        
        /* set H-bridge for forward */
        gpio_bit_set(HIN1_PORT, HIN1_PIN);
        gpio_bit_reset(LIN1_PORT, LIN1_PIN);
        gpio_bit_set(HIN2_PORT, HIN2_PIN);
        gpio_bit_reset(LIN2_PORT, LIN2_PIN);
        
        /* set PWM */
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, duty);
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, duty);
    } else {
        // Reverse
        motor_state = MOTOR_STATE_REVERSE;
        
        /* set H-bridge for reverse */
        gpio_bit_reset(HIN1_PORT, HIN1_PIN);
        gpio_bit_set(LIN1_PORT, LIN1_PIN);
        gpio_bit_reset(HIN2_PORT, HIN2_PIN);
        gpio_bit_set(LIN2_PORT, LIN2_PIN);
        
        /* set PWM */
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, duty);
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, duty);
    }
}

/*!
    \brief      motor brake
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_brake(void)
{
    motor_state = MOTOR_STATE_BRAKE;
    
    /* set H-bridge for brake */
    gpio_bit_reset(HIN1_PORT, HIN1_PIN);
    gpio_bit_set(LIN1_PORT, LIN1_PIN);
    gpio_bit_reset(HIN2_PORT, HIN2_PIN);
    gpio_bit_set(LIN2_PORT, LIN2_PIN);
    
    /* set PWM to 0 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 0);
    
    /* keep motor enabled for brake */
    gpio_bit_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
    
    /* brake for 500ms */
    delay_1ms(500U);
    
    /* stop motor */
    motor_stop();
}

/*!
    \brief      motor stop
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_stop(void)
{
    motor_state = MOTOR_STATE_STOP;
    
    /* reset all control pins */
    gpio_bit_reset(HIN1_PORT, HIN1_PIN);
    gpio_bit_reset(LIN1_PORT, LIN1_PIN);
    gpio_bit_reset(HIN2_PORT, HIN2_PIN);
    gpio_bit_reset(LIN2_PORT, LIN2_PIN);
    
    /* set PWM to 0 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 0);
    
    /* disable motor */
    gpio_bit_reset(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
}
