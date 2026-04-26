/*!
    \file    pwm_test_pb13_pb14.c
    \brief   PWM test for PB13 (TIMER0_CH0N) and PB14 (TIMER0_CH1N)

    \version 2026-4-19, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>

#define PWM_PERIOD    999U  // 1kHz 周期
#if 1
void pwm_test_pb14_only(uint8_t duty);
void pwm_test_pb13_only(uint8_t duty);

void pwm_pb13_pb14_init(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;
    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. GPIO 配置为复用推挽 */
    /* PB13 - TIMER0_CH0N, PB14 - TIMER0_CH1N */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

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
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;  // 启用互补输出
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 TIMER0 CH0 (PB13) - 50% 占空比 */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 500);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* 配置 TIMER0 CH1 (PB14) - 30% 占空比 */
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
    \brief      设置 PB13 (TIMER0_CH0N) 的占空比
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
// void set_pwm_pb13_duty(uint8_t duty)
// {
//     uint16_t pulse;
    
//     /* 限制占空比范围 */
//     if(duty > 100) {
//         duty = 100;
//     }
    
//     /* 计算脉冲值 */
//     pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
//     /* 设置脉冲值 */
//     timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse);
// }

// /*!
//     \brief      设置 PB14 (TIMER0_CH1N) 的占空比
//     \param[in]  duty: 占空比 (0-100)
//     \param[out] none
//     \retval     none
// */
// void set_pwm_pb14_duty(uint8_t duty)
// {
//     uint16_t pulse;是
    
//     /* 限制占空比范围 */
//     if(duty > 100) {
//         duty = 100;
//     }
    
//     /* 计算脉冲值 */
//     pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    
//     /* 设置脉冲值 */
//     timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse);
// }


void pwm_test_pb13_pb14(void)
{
    printf("\r\n=== PB13/PB14 PWM Test ===\r\n");
    printf("PB13 (TIMER0_CH0N): 1kHz, 50%% duty cycle\r\n");
    printf("PB14 (TIMER0_CH1N): 1kHz, 30%% duty cycle\r\n");

    pwm_pb13_pb14_init();

    printf("PWM started - check PB13 and PB14 with oscilloscope\r\n");

    while(1);
}
static uint8_t duty_test = 20;
static uint8_t duty_test1 = 10;
void pwm_test_pb13_pb14_duty_adjust(void)
{
  

	
	 pwm_pb13_pb14_init();

     //set_pwm_pb13_duty(10);
    //set_pwm_pb14_duty(50);


    #if 0
    uint8_t duty = 0;
    
    printf("\r\n=== PB13/PB14 Duty Cycle Adjust Test ===\r\n");
    
    pwm_pb13_pb14_init();
    
    printf("Testing duty cycle adjustment...\r\n");对
    
    while(1) {
        /* 递增占空比 */
        for(duty = 0; duty <= 100; duty += 10) {
            set_pwm_pb13_duty(duty);
            set_pwm_pb14_duty(duty);
            
            printf("Duty: %d%%\r\n", duty);
            delay_1ms(1000U);
        }
        
        /* 递减占空比 */
        for(duty = 90; duty > 0; duty -= 10) {
            set_pwm_pb13_duty(duty);
            set_pwm_pb14_duty(duty);
            
            printf("Duty: %d%%\r\n", duty);
            delay_1ms(1000U);
        }
    }
    #endif
}

/*!
    \brief      单独测试 PB13 (TIMER0_CH0N) 输出 PWM 波形
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void pwm_test_pb13_only(uint8_t duty)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. GPIO 配置为复用推挽 */
    /* PB13 - TIMER0_CH0N */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

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
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;  // 启用互补输出
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 TIMER0 CH0 (PB13) */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    uint16_t pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* 5. 影子寄存器使能 */
    timer_auto_reload_shadow_enable(TIMER0);

    /* 6. 高级定时器主输出使能 */
    timer_primary_output_config(TIMER0, ENABLE);

    /* 7. 启动定时器 */
    timer_enable(TIMER0);

    printf("\r\n=== PB13 Only PWM Test ===\r\n");
    printf("PB13 (TIMER0_CH0N): 1kHz, %d%% duty cycle\r\n", duty);
    printf("PWM started - check PB13 with oscilloscope\r\n");
}



/*!
    \brief      单独测试 PB14 (TIMER0_CH1N) 输出 PWM 波形
    \param[in]  duty: 占空比 (0-100)
    \param[out] none
    \retval     none
*/
void pwm_test_pb14_only(uint8_t duty)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. GPIO 配置为复用推挽 */
    /* PB14 - TIMER0_CH1N */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);

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
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;  // 启用互补输出
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 TIMER0 CH1 (PB14) */
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
    uint16_t pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100U);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 5. 影子寄存器使能 */
    timer_auto_reload_shadow_enable(TIMER0);

    /* 6. 高级定时器主输出使能 */
    timer_primary_output_config(TIMER0, ENABLE);

    /* 7. 启动定时器 */
    timer_enable(TIMER0);

    printf("\r\n=== PB14 Only PWM Test ===\r\n");
    printf("PB14 (TIMER0_CH1N): 1kHz, %d%% duty cycle\r\n", duty);
    printf("PWM started - check PB14 with oscilloscope\r\n");
}

/*!
    \brief      enable PB13 (TIMER0_CH0N) PWM output
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_pb13_enable(void)
{
    /* enable TIMER0 */
    timer_enable(TIMER0);
    
    /* enable TIMER0 CH0 output */
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    
    printf("PB13 PWM enabled\r\n");
}

/*!
    \brief      disable PB13 (TIMER0_CH0N) PWM output
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_pb13_disable(void)
{
    /* disable TIMER0 CH0 output */
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    
    printf("PB13 PWM disabled\r\n");
}

/*!
    \brief      enable PB14 (TIMER0_CH1N) PWM output
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_pb14_enable(void)
{
    /* enable TIMER0 */
    timer_enable(TIMER0);
    
    /* enable TIMER0 CH1 output */
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    
    printf("PB14 PWM enabled\r\n");
}

/*!
    \brief      disable PB14 (TIMER0_CH1N) PWM output
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_pb14_disable(void)
{
    /* disable TIMER0 CH1 output */
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    
    printf("PB14 PWM disabled\r\n");
}
#endif
