/*!
    \file    pwm_test_standalone.c
    \brief   PWM test using TIMER0 and TIMER1

    \version 2026-4-18, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>

void uart2_init_for_test(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_AF);

    AFIO_PCF0 &= ~(BITS(4,5));
    AFIO_PCF0 |= BIT(5);

    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
}

void pwm_test123_pwm(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);

    /* 2. GPIO 配置为复用推挽 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

    /* 3. TIMER0 基础配置 */
    timer_deinit(TIMER0);
    timerpara.prescaler         = 71;
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = 999;
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

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 500);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 500);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 5. TIMER1 基础配置 */
    timer_deinit(TIMER1);
    timerpara.prescaler         = 71;
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = 999;
    timerpara.clockdivision     = TIMER_CKDIV_DIV1;
    timerpara.repetitioncounter = 0;
    timer_init(TIMER1, &timerpara);

    /* 6. TIMER1 PWM 通道配置 */
    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 500);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 500);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* 7. 影子寄存器使能 */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_auto_reload_shadow_enable(TIMER1);

    /* 8. 高级定时器主输出使能 */
    timer_primary_output_config(TIMER0, ENABLE);
    timer_primary_output_config(TIMER1, ENABLE);

    /* 9. 启动定时器 */
    timer_enable(TIMER0);
    timer_enable(TIMER1);
}

// PWM 频率 10KHz  72M时钟
#define PWM_PERIOD    (999U)   // 周期值
#define PWM_DUTY_50   (500U)   // PB13 50%
#define PWM_DUTY_75   (750U)   // PB14 75%

void pwm_pa0_pa1_init0(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER1);

    /* 2. GPIO 配置0为复用推挽 */
    /* PA0 - TIMER1_CH0, PA1 - TIMER1_CH1 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);

    /* 3. TIMER1 基础配置 */
    timer_deinit(TIMER1);
    timerpara.prescaler         = 71;        /* 72MHz / (71+1) = 1MHz */
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = 999;       /* 1MHz / 1000 = 1kHz */
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



void pwm_init(void)
{
 timer_oc_parameter_struct timer_oc_struct;
    timer_parameter_struct timer_struct;

    // 时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_TIMER2);  

    // PA0 / PA1  复用推挽（默认就是 TIMER2 通道，不用重映射！）
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);

    // 定时器 10kHz
    timer_deinit(TIMER2);
    timer_struct.prescaler = 71;
    timer_struct.counterdirection = TIMER_COUNTER_UP;
    timer_struct.period = 999;
    timer_struct.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_struct);

    // PWM 配置
    timer_oc_struct.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_oc_struct.outputstate = TIMER_CCX_ENABLE;

    // PA0 = 50%
    timer_channel_output_config(TIMER2, TIMER_CH_0, &timer_oc_struct);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 500);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM1);

    // PA1 = 75%
    timer_channel_output_config(TIMER2, TIMER_CH_1, &timer_oc_struct);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 750);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM1);

    timer_auto_reload_shadow_enable(TIMER2);
    timer_enable(TIMER2);
		
}




void pwm_test_default_pins(void)
{
    timer_oc_parameter_struct timer_oc_init_struct;
    timer_parameter_struct timer_init_struct;

    /* 1. 开时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_TIMER1);

    /* 2. 默认引脚 PA8=TIM1_CH1, PA9=TIM1_CH2 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);

    /* 3. TIM1 时基 */
    timer_deinit(TIMER1);
    timer_init_struct.prescaler         = 71;
    timer_init_struct.counterdirection  = TIMER_COUNTER_UP;
    timer_init_struct.period            = 999;
    timer_init_struct.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init_struct.repetitioncounter = 0;
    timer_init(TIMER1, &timer_init_struct);

    /* 4. PWM配置 */
    timer_oc_init_struct.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_oc_init_struct.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_oc_init_struct.outputstate  = TIMER_CCX_ENABLE;

    /* CH1 PA8 50% */
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_oc_init_struct);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 500);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM1);

    /* CH2 PA9 75% */
    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_oc_init_struct);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 750);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM1);

    timer_auto_reload_shadow_enable(TIMER1);
    timer_enable(TIMER1);
}




void gpio_test_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    
    // PB13 PB14 配置为普通推挽输出
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);
	 while(1)
    {
        // 快速翻转：PB13和PB14同时输出方波
        gpio_bit_set(GPIOB, GPIO_PIN_13);
        gpio_bit_set(GPIOB, GPIO_PIN_14);
        
        for(volatile uint32_t i=0; i<100000; i++);
        
        gpio_bit_reset(GPIOB, GPIO_PIN_13);
        gpio_bit_reset(GPIOB, GPIO_PIN_14);
        
        for(volatile uint32_t i=0; i<100000; i++);
    }
}




void pwm_test_standalone(void)
{
    //uart2_init_for_test();
    printf("\r\n=== PWM Test ===\r\n");
	//pwm_init();
	//gpio_test_init();
	//pwm_test_default_pins();
	pwm_init();
    //pwm_test123_pwm();
    printf("PWM Test: TIMER0 (PA8,PA9) + TIMER1 (PB13,PB14)\r\n");
    printf("All channels should output 1kHz, 50%% duty cycle\r\n");
    while(1);
}