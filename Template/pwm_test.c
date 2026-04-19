/*!
    \file    pwm_test.c
    \brief   PWM output test module implementation

    \version 2026-4-18, V1.0.0, firmware for GD32F30x
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

#include "pwm_test.h"
#include "systick.h"
#include "stdio.h"

/*!
    \brief      initialize PWM test
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_test_init(void)
{
    /* enable GPIO clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER1);
    
    /* configure all PWM pins as alternate function */
    // HIN1 (PB14) - TIMER1_CH3
    gpio_init(HIN1_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, HIN1_PIN);
    // LIN1 (PA9) - TIMER1_CH2
    gpio_init(LIN1_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, LIN1_PIN);
    // HIN2 (PB13) - TIMER1_CH1N
    gpio_init(HIN2_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, HIN2_PIN);
    // LIN2 (PA8) - TIMER1_CH1
    gpio_init(LIN2_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, LIN2_PIN);
    
    /* configure TIMER1 for PWM */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    
    timer_deinit(TIMER1);
    
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 71;  // 72MHz / (71+1) = 1MHz
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 999;  // 1MHz / 1000 = 1kHz
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);
    
    /* CH0 configuration (LIN2 - PA8) - no complementary output */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);
    
    /* CH1 configuration (HIN2 - PB13) - complementary output only */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocinitpara);
    
    /* CH2 configuration (LIN1 - PA9) - no complementary output */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocinitpara);
    
    /* CH3 configuration (HIN1 - PB14) - no complementary output */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocinitpara);
    
    /* Set initial duty cycle to 0 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);  // LIN2 (PA8)
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 0);  // HIN2 (PB13) - complementary
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 0);  // LIN1 (PA9)
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, 0);  // HIN1 (PB14)
    
    /* Set PWM mode */
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    
    /* Enable shadow registers */
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    
    /* start TIMER1 */
    timer_enable(TIMER1);
    
    printf("PWM test initialized with hardware PWM\r\n");
}

/*!
    \brief      test all PWM output pins
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_test_all_pins(void)
{
    printf("\r\n--- Testing all PWM pins ---\r\n");
    
    /* Set all channels to 50% duty cycle */
    uint16_t pulse = (50 * 999) / 100;  // 50% duty cycle
    
    printf("Setting all pins to 50%% duty cycle PWM\r\n");
    
    /* Set all channels to 50% duty cycle */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, pulse);  // LIN2 (PA8)
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, pulse);  // HIN2 (PB13) - complementary
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, pulse);  // LIN1 (PA9)
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, pulse);  // HIN1 (PB14)
    
    printf("All pins are now outputting 50%% duty cycle PWM\r\n");
    printf("Press reset to stop the test\r\n");
    
    /* Loop forever to keep PWM output active */
    while(1) {
        // Keep the program running
    }
}

/*!
    \brief      test hardware PWM on specific channel
    \param[in]  pin_name: pin name as string
    \param[in]  timer_periph: timer peripheral
    \param[in]  channel: timer channel
    \param[in]  duty_cycle: duty cycle (0-100)
    \param[out] none
    \retval     none
*/
void pwm_test_hardware_pwm(const char *pin_name, uint32_t timer_periph, uint32_t channel, uint8_t duty_cycle)
{
    /* Calculate pulse value (0-999) */
    uint16_t pulse = (duty_cycle * 999) / 100;
    
    printf("Testing %s with %d%% duty cycle\r\n", pin_name, duty_cycle);
    
    /* Set duty cycle */
    timer_channel_output_pulse_value_config(timer_periph, channel, pulse);
    
    /* Wait for 2 seconds */
    delay_1ms(2000U);
    
    /* Set duty cycle to 0 */
    timer_channel_output_pulse_value_config(timer_periph, channel, 0);
    
    /* Wait for 500ms between tests */
    delay_1ms(500U);
}
