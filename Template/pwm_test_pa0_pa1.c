/*!
    \file    pwm_test_pa0_pa1.c
    \brief   PWM test for PA0 (TIMER1_CH0), PA1 (TIMER1_CH1), PA8 (TIMER0_CH0), PA9 (TIMER0_CH1)

    \version 2026-4-19, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>

#define PWM_PERIOD    999U  // 1kHz 周期

void pwm_pa0_pa1_init(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER1);

    /* 2. GPIO 配置为复用推挽 */
    /* PA0 - TIMER1_CH0, PA1 - TIMER1_CH1 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);

    /* 3. TIMER1 基础配置 */
    timer_deinit(TIMER1);
    timerpara.prescaler         = 71;        /* 72MHz / (71+1) = 1MHz */
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = PWM_PERIOD;       /* 1MHz / 1000 = 1kHz */
    timerpara.clockdivision     = TIMER_CKDIV_DIV1;
    timerpara.repetitioncounter = 0;
    timer_init(TIMER1, &timerpara);

    /* 4. TIMER1 PWM 通道配置 */
    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 TIMER1 CH0 (PA0) - 50% 占空比 */
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 500);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* 配置 TIMER1 CH1 (PA1) - 30% 占空比 */
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 300);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 5. 影子寄存器使能 */
    timer_auto_reload_shadow_enable(TIMER1);

    /* 6. 高级定时器主输出使能 */
    timer_primary_output_config(TIMER1, ENABLE);

    /* 7. 启动定时器 */
    timer_enable(TIMER1);
}

void pwm_pa8_pa9_init(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. GPIO 配置为复用推挽 */
    /* PA8 - TIMER0_CH0, PA9 - TIMER0_CH1 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);

    /* 3. TIMER0 基础配置 */
    timer_deinit(TIMER0);
    timerpara.prescaler         = 71;        /* 72MHz / (71+1) = 1MHz */
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = PWM_PERIOD;       /* 1MHz / 1000 = 1kHz */
    timerpara.clockdivision     = TIMER_CKDIV_DIV1;
    timerpara.repetitioncounter = 0;
    timer_init(TIMER0, &timerpara);

    /* 4. TIMER0 PWM 通道配置 */
    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 TIMER0 CH0 (PA8) - 50% 占空比 */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 500);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* 配置 TIMER0 CH1 (PA9) - 30% 占空比 */
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 300);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 5. 影子寄存器使能 */
    timer_auto_reload_shadow_enable(TIMER0);

    /* 6. 高级定时器主输出使能 */
    timer_primary_output_config(TIMER0, ENABLE);

    /* 7. 启动定时器 */
    timer_enable(TIMER0);
}

/*!
    \brief      设置 PA0 (TIMER1_CH0) 的占空比
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void set_pwm_pa0_duty(uint8_t duty)
{
    uint16_t pulse;
    
    /* 限制占空比范围 */
    if(duty > 100) {
        duty = 100;
    }
    
    /* 计算脉冲值 */
    pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
    /* 设置脉冲值 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, pulse);
}

/*!
    \brief      设置 PA1 (TIMER1_CH1) 的占空比
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void set_pwm_pa1_duty(uint8_t duty)
{
    uint16_t pulse;
    
    /* 限制占空比范围 */
    if(duty > 100) {
        duty = 100;
    }
    
    /* 计算脉冲值 */
    pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
    /* 设置脉冲值 */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, pulse);
}

/*!
    \brief      设置 PA8 (TIMER0_CH0) 的占空比
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void set_pwm_pa8_duty(uint8_t duty)
{
    uint16_t pulse;
    
    /* 限制占空比范围 */
    if(duty > 100) {
        duty = 100;
    }
    
    /* 计算脉冲值 */
    pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
    /* 设置脉冲值 */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse);
}

/*!
    \brief      设置 PA9 (TIMER0_CH1) 的占空比
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void set_pwm_pa9_duty(uint8_t duty)
{
    uint16_t pulse;
    
    /* 限制占空比范围 */
    if(duty > 100) {
        duty = 100;
    }
    
    /* 计算脉冲值 */
    pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
    /* 设置脉冲值 */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse);
}

void pwm_test_all(void)
{
    printf("\r\n=== PWM Test (PA0/PA1/PA8/PA9) ===\r\n");
    printf("PA0 (TIMER1_CH0): 1kHz, 50%% duty cycle\r\n");
    printf("PA1 (TIMER1_CH1): 1kHz, 30%% duty cycle\r\n");
    printf("PA8 (TIMER0_CH0): 1kHz, 50%% duty cycle\r\n");
    printf("PA9 (TIMER0_CH1): 1kHz, 30%% duty cycle\r\n");

    pwm_pa0_pa1_init();
    pwm_pa8_pa9_init();

    printf("PWM started - check all pins with oscilloscope\r\n");

    while(1);
}

void pwm_test_duty_adjust(void)
{
    uint8_t duty = 0;
    
    printf("\r\n=== PWM Duty Cycle Adjust Test ===\r\n");
    
    pwm_pa0_pa1_init();
    pwm_pa8_pa9_init();
    
    printf("Testing duty cycle adjustment...\r\n");
    
    while(1) {
        /* 递增占空比 */
        for(duty = 0; duty <= 100; duty += 10) {
            set_pwm_pa0_duty(duty);
            set_pwm_pa1_duty(duty);
            set_pwm_pa8_duty(duty);
            set_pwm_pa9_duty(duty);
            
            printf("Duty: %d%%\r\n", duty);
            delay_1ms(1000U);
        }
        
        /* 递减占空比 */
        for(duty = 90; duty > 0; duty -= 10) {
            set_pwm_pa0_duty(duty);
            set_pwm_pa1_duty(duty);
            set_pwm_pa8_duty(duty);
            set_pwm_pa9_duty(duty);
            
            printf("Duty: %d%%\r\n", duty);
            delay_1ms(1000U);
        }
    }
}