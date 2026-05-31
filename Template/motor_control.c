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
#include "current_sensor.h"
#include "temperature_sensor.h"
#include "hall_sensor.h"
#include "systick.h"
#include "gd32f30x.h"
#include <stdio.h>
#include "stroke_counter.h"

// LIN1 = PA9 普通GPIO
#define LIN1_HIGH()    gpio_bit_set(GPIOA, GPIO_PIN_9)
#define LIN1_LOW()     gpio_bit_reset(GPIOA, GPIO_PIN_9)

// LIN2 = PA8 普通GPIO
#define LIN2_HIGH()    gpio_bit_set(GPIOA, GPIO_PIN_8)
#define LIN2_LOW()     gpio_bit_reset(GPIOA, GPIO_PIN_8)


void motor_gpio_init(void)
{
    // 1. 使能时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    // ========================
    // LIN1(PA9) + LIN2(PA8) = 普通GPIO
    // ========================
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    LIN1_LOW();
    LIN2_LOW();

    // ========================
    // HIN1(PB14) + HIN2(PB13) = PWM 输出
    // ========================
    //gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);
}












#define PWM_PERIOD    82U   // 20kHz 周期 (120MHz/72/83≈20kHz)


// 全局变量
volatile motor_state_t motor_state = MOTOR_STATE_STOP;
volatile control_mode_t control_mode = MODE_OPEN_LOOP;
volatile uint16_t target_speed = 0;

// 外部变量声明
extern volatile uint16_t actual_speed;


//函数声明
void PWM_LIN2_Enable(bool enable);
void PWM_HIN2_Enable(bool enable);      
void PWM_LIN1_Enable(bool enable);
void PWM_HIN1_Enable(bool enable);
void PWM_set_LIN2_Duty(uint8_t duty);
void PWM_set_HIN2_Duty(uint8_t duty);
void PWM_set_LIN1_Duty(uint8_t duty);
void PWM_set_HIN1_Duty(uint8_t duty);

#if 0
//void PWM_PA8_PA9_PB13_PB14_Init(void)
void motor_control_init(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 开全部时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. 配置全部4个引脚 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

    /* 3. TIMER0 基础配置 只初始化一次！*/
    timer_deinit(TIMER0);
    timerpara.prescaler         = 71;
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = PWM_PERIOD;
    timerpara.clockdivision     = TIMER_CKDIV_DIV1;
    timerpara.repetitioncounter = 0;
    timer_init(TIMER0, &timerpara);

    /* ==================== 配置通道0（PA8 + PB13） ==================== */
    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;   // 主通道 PA8 使能
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;  // 互补通道 PB13 使能
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OC_POLARITY_LOW;//TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);  // 50%
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

    /* ==================== 配置通道1（PA9 + PB14） ==================== */
    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;   // 主通道 PA9 使能
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;  // 互补通道 PB14 使能

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);  // 30%
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);

    /* 4. 统一使能 */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_primary_output_config(TIMER0, ENABLE);
    timer_enable(TIMER0);
}
#endif

void motor_control_init(void)
{
    timer_oc_parameter_struct timer_ocpara;
    timer_parameter_struct timerpara;

    /* 1. 开全部时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER0);

    /* 2. 引脚配置（关键修改）
        PA8、PA9 = 普通推挽输出 GPIO
        PB13、PB14 = 复用功能 PWM
    */
    // PA8、PA9 → 普通GPIO
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_bit_reset(GPIOA, GPIO_PIN_8 | GPIO_PIN_9); // 默认低电平

    // PB13、PB14 → 复用PWM
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

    /* 3. TIMER0 基础配置（完全保留你的参数） */
    timer_deinit(TIMER0);
    timerpara.prescaler         = 71;
    timerpara.alignedmode       = TIMER_COUNTER_EDGE;
    timerpara.counterdirection  = TIMER_COUNTER_UP;
    timerpara.period            = PWM_PERIOD;
    timerpara.clockdivision     = TIMER_CKDIV_DIV1;
    timerpara.repetitioncounter = 0;
    timer_init(TIMER0, &timerpara);

    /* ==================== 配置通道0（ PA8关闭 + PB13 开 ） ==================== */
    // 主通道（PA8）= 关闭
    // 互补通道（PB13）= 使能（PWM）
    timer_ocpara.outputstate  = TIMER_CCX_DISABLE;   // 主通道 PA8 关闭
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;   // 互补通道 PB13 使能
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

    /* ==================== 配置通道1（ PA9关闭 + PB14 开 ） ==================== */
    // 主通道（PA9）= 关闭
    // 互补通道（PB14）= 使能（PWM）
    timer_ocpara.outputstate  = TIMER_CCX_DISABLE;   // 主通道 PA9 关闭
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;   // 互补通道 PB14 使能

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);

    /* 4. 统一使能（保留你原来的） */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_primary_output_config(TIMER0, ENABLE);
    timer_automatic_output_enable(TIMER0);
    timer_enable(TIMER0);


}



// ==============================
// 四路独立设置占空比（0~100）
// ==============================

// 设置 PA8 占空比 0~100
//void PWM_Set_PA8_Duty(uint8_t duty)
void PWM_set_LIN2_Duty(uint8_t duty)
{
    // if(duty > 100) duty = 100;
    // uint32_t pulse = (duty * (PWM_PERIOD+1)) / 100;
    // timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse);
}

// 设置 PB13 占空比 0~100
//void PWM_Set_PB13_Duty(uint8_t duty)
void PWM_set_HIN2_Duty(uint8_t duty)
{
    if(duty > 100) duty = 100;
    duty = 100 - duty; // HIN占空比取反
    //PWM_HIN2_Enable(1); //确保互补通道使能
    uint32_t pulse = ((100-duty) * (PWM_PERIOD+1)) / 100;
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse);
}

// 设置 PA9 占空比 0~100
void PWM_set_LIN1_Duty(uint8_t duty)
{
    // if(duty > 100) duty = 100;
    // uint32_t pulse = (duty * (PWM_PERIOD+1)) / 100;
    // timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse);
}

// 设置 PB14 占空比 0~100
void PWM_set_HIN1_Duty(uint8_t duty)
{
    if(duty > 100) duty = 100;
    duty = 100 - duty; // HIN占空比取反
    uint32_t pulse = ((100-duty) * (PWM_PERIOD+1)) / 100;
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse);
}




// 开关 PA8（TIMER0_CH0）
//void PWM_PA8_Enable(bool enable)
void PWM_LIN2_Enable(bool enable)
{
    timer_oc_parameter_struct timer_ocpara;

    // 先读取当前配置，再修改主通道状态
    timer_ocpara.outputstate  = enable ? TIMER_CCX_ENABLE : TIMER_CCX_DISABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE; // 保持互补通道开启
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
}

// 开关 PB13（TIMER0_CH0N）
//void PWM_PB13_Enable(bool enable)
void PWM_HIN2_Enable(bool enable)
{
    timer_oc_parameter_struct timer_ocpara;

    timer_ocpara.outputstate  = TIMER_CCX_ENABLE; // 保持主通道开启
    timer_ocpara.outputnstate = enable ? TIMER_CCXN_ENABLE : TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocpara);
}

// 开关 PA9（TIMER0_CH1）
//void PWM_PA9_Enable(bool enable)
void PWM_LIN1_Enable(bool enable)
{
    timer_oc_parameter_struct timer_ocpara;

    timer_ocpara.outputstate  = enable ? TIMER_CCX_ENABLE : TIMER_CCX_DISABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
}

// 开关 PB14（TIMER0_CH1N）
//void PWM_PB14_Enable(bool enable)
void PWM_HIN1_Enable(bool enable)
{
    timer_oc_parameter_struct timer_ocpara;

    timer_ocpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocpara.outputnstate = enable ? TIMER_CCXN_ENABLE : TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocpara);
}



/**
 * @brief  设置电机速度和方向，并包含行程保护
 * @param  speed: 目标速度值 (0 ~ MAX_SPEED)
 * @param  direction: 旋转方向
 *         - 0: 正转
 *         - 1: 反转
 * @retval None
 * @note   该函数会在行程达到最大值时自动停止电机。
 */
void motor_set_speed(uint16_t speed, bool direction)
{
   // 1. 行程保护：检查是否达到最大行程，若达到则停止电机
    uint32_t stroke_count = get_stroke_count();
    if (stroke_count >= MAX_STROKE) 
    {
        motor_stop();
        return;
    }
    if (speed == 0) 
    {
        motor_stop();
        return;
    }
    //参数校验：确保速度不超过最大值
    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }

    
    /* calculate PWM duty cycle (0-999) */
    //uint16_t duty = (speed * 999) / MAX_SPEED;
    uint8_t duty = (uint8_t)((speed * 100) / MAX_SPEED); // 0-100



    //根据方向设置电机状态和PWM输出
    static uint8_t duty_test = 83;
    if (direction == 0) //正转
    {
        motor_state = MOTOR_STATE_FORWARD;// 更新电机状态
       // PWM_set_HIN1_Duty(83); // HIN1(PB14) 设置为目标速度
       PWM_set_HIN1_Duty(duty_test);
        //delay_1ms(500);
        LIN1_HIGH(); // LIN1(PA9) 设置为常高 (100%) 
        LIN2_LOW(); // LIN2(PA8) 设置为常低 (0%)
        delay_1ms(100);
        PWM_set_HIN2_Duty(0); // HIN2(PB13) 设置为目标速度

    }
    else //反转
    {
        motor_state = MOTOR_STATE_REVERSE;// 更新电机状态
        PWM_set_HIN1_Duty(0); // HIN1(PB14) 设置为目标速度
        LIN2_HIGH(); // LIN2(PA8) 设置为常高 (100%)
        LIN1_LOW(); // LIN1(PA9) 设置为常低 (0%)
        PWM_set_HIN2_Duty(duty_test);
        delay_1ms(100);
    }

}

/**
 * @brief  电机刹车（短路制动）
 * @note   通过同时开启两个下管（LIN1, LIN2），使电机线圈短路，实现快速停止。
 *         必须确保所有上管（HIN1, HIN2）处于关闭状态。
 */
void motor_brake(void)
{
    motor_state = MOTOR_STATE_BRAKE;
    
    /* set H-bridge for brake */

    //安全第一：关闭所有上管 (HIN1, HIN2)
    // 先设置占空比为0，再关闭使能
    PWM_set_HIN1_Duty(0); // 0% 占空比
    PWM_HIN1_Enable(0);
    PWM_set_HIN2_Duty(0); // 0% 占空比
    PWM_HIN2_Enable(0);


    // 2. 执行刹车：开启两个下管 (LIN1, LIN2)
    // 先设置占空比为100%，再开启使能
    PWM_set_LIN1_Duty(100); // 100% 占空比
    PWM_LIN1_Enable(1);
    PWM_set_LIN2_Duty(100); // 100% 占空比
    PWM_LIN2_Enable(1);
 

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
    #if 0

    // 强制将所有通道的占空比设置为 0
    // 使用库函数直接设置比较寄存器值，避免使用带取反逻辑的HIN设置函数
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0); // LIN2 & HIN2
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0); // LIN1 & HIN1

    // 关闭所有通道的输出使能
    PWM_LIN2_Enable(0);
    PWM_HIN2_Enable(0);
    PWM_LIN1_Enable(0);
    PWM_HIN1_Enable(0);
    #endif



    PWM_set_HIN1_Duty(0); // HIN1(PB14) 设置为目标速度
    PWM_set_HIN2_Duty(0); // HIN2(PB13) 设置为目标速度
    delay_1ms(1);
    LIN1_LOW(); // LIN1(PA9) 设置为常高 (100%)
    LIN2_LOW(); // LIN2(PA8) 设置为常低 (0%)


}




/*
 * @brief  处理电机过流情况
 * @param  none
 * @retval none
 */
void motor_over_current_process(void)
{

    /* check for overcurrent */
    uint16_t current = read_motor_current();
    if (current > MAX_CURRENT) 
    {
        motor_brake();
        printf("\r\nOvercurrent detected: %d mA", current);
    }
    
}





void motor_control_test(uint8_t cmd)
{
    switch(cmd)
    {
        case 0x01: // Forward at 50% speed
            motor_set_speed(MAX_SPEED / 2, 0);
            break;
        case 0x02: // Reverse at 50% speed
            motor_set_speed(MAX_SPEED / 2, 1);
            break;
        case 0x03: // Brake
            motor_brake();
            break;
        case 0x00: // Stop
        default:
            motor_stop();
            break;
    }
}


void test_pwm_init(uint8_t data)
{
     if (data >=0 && data <= 30) 
        {
        PWM_set_LIN2_Duty(data);
        }
        else if (data > 30&& data <= 50) 
        {
            PWM_set_LIN1_Duty(data);
        }
    else if (data > 50 && data <= 80) 
    {
        PWM_set_HIN2_Duty(data);
    }
    else if (data > 80 && data <= 100) 
    {
        PWM_set_HIN1_Duty(data);
    }
    else if (data == 0xaa) 
    {
        PWM_set_LIN2_Duty(0); 
    }
    else if (data == 0xab) 
    {
        PWM_set_LIN1_Duty(0); 
    }
    else if (data == 0xbb) 
    {
        PWM_set_HIN2_Duty(0); 
    }
    else if (data == 0xcc) 
    {
        PWM_set_HIN1_Duty(0); 
    }
    else if (data == 0xdd) 
    {
        PWM_set_LIN2_Duty(100); 
    }
    else if (data == 0xee) 
    {
        PWM_set_LIN1_Duty(100); 
    }
    else if (data == 0xff) 
    {
        PWM_set_HIN2_Duty(100); 
    }
    else if (data == 0x65) 
    {
        PWM_set_HIN1_Duty(100); 
    }
    else if (data == 0x66) 
    {
        PWM_LIN2_Enable(1); 
    }
    else if (data == 0x67) 
    {
        PWM_LIN1_Enable(1); 
    }
    else if (data == 0x68) 
    {
        PWM_HIN2_Enable(1); 
    }
    else if (data == 0x69) 
    {
        PWM_HIN1_Enable(1); 
    }
    else if (data == 0x6a) 
    {
        PWM_LIN2_Enable(0); 
    }
    else if (data == 0x6b) 
    {
        PWM_LIN1_Enable(0); 
    }
    else if (data == 0x6c) 
    {
        PWM_HIN2_Enable(0); 
    }
    else if (data == 0x6d) 
    {
        PWM_HIN1_Enable(0); 
    }
}


