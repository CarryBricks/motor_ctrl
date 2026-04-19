/*!
    \file    pwm_test_register.c
    \brief   PWM test using direct register operations

    \version 2026-4-18, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x.h"
#include <stdio.h>

void pwm_test_register(void)
{
    printf("\r\n=== Register Level PWM Test ===\r\n");
    
    /* Enable clocks */
    rcu_periph_clock_enable(RCU_TIMER1);  // Enable TIMER1 clock
    rcu_periph_clock_enable(RCU_GPIOA);   // Enable GPIOA clock
    rcu_periph_clock_enable(RCU_GPIOB);   // Enable GPIOB clock
    rcu_periph_clock_enable(RCU_AF);      // Enable AF clock
    
    printf("Clocks enabled\r\n");
    
    /* Configure GPIO pins */
    // PA8 - TIMER1_CH1 (AF_PP)
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    
    // PA9 - TIMER1_CH2 (AF_PP)
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    
    // PB13 - TIMER1_CH1N (AF_PP)
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    
    // PB14 - TIMER1_CH3 (AF_PP)
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    
    printf("GPIO configured\r\n");
    
    /* Configure TIMER1 */
    timer_deinit(TIMER1);
    
    // Set prescaler: 72MHz / (71+1) = 1MHz
    timer_prescaler_config(TIMER1, 71, TIMER_PSC_RELOAD_NOW);
    
    // Set period: 1MHz / 1000 = 1kHz
    timer_autoreload_value_config(TIMER1, 999);
    
    /* Configure CH1 (PA8) */
    timer_oc_parameter_struct timer_ocintpara;
    
    // CH1 configuration
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 499);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    
    // CH1N configuration
    timer_channel_complementary_output_state_config(TIMER1, TIMER_CH_1, TIMER_CCXN_ENABLE);
    
    // CH2 configuration
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 499);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    
    // CH3 configuration
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, 499);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* Enable auto-reload */
    timer_auto_reload_shadow_enable(TIMER1);
    
    /* Enable main output */
    timer_primary_output_config(TIMER1, ENABLE);
    
    /* Start timer */
    timer_enable(TIMER1);
    
    printf("Timer started - PWM should be running now\r\n");
    printf("Testing pins: PA8, PA9, PB13, PB14\r\n");
    
    /* Loop forever */
    while(1) {
    }
}


