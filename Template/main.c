/*!
    \file    main.c
    \brief   Motor control system with Hall sensor, current protection and Modbus RTU

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
#include "systick.h"
#include <stdio.h>
#include <string.h>

#include "main.h"

// 模块头文件包含
#include "motor_control.h"
#include "current_sensor.h"
#include "temperature_sensor.h"
#include "hall_sensor.h"
#include "stroke_counter.h"
#include "modbus.h"
#include "led_control.h"
#include "key_button.h"
#include "hub_reset.h"
#include "test_pa5_pa6_adc.h"


// 函数声明
void system_init(void);






void timer_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz

    TIMER1 channel0 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel1 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel2 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER0);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 15999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

    /* CH0,CH1 and CH2 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);

    /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,3999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,7999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,11999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    /* auto-reload preload enable */
    timer_enable(TIMER0);
}





/*!
    \brief      redirect C library printf function to UART2
    \param[in]  ch: character to send
    \param[in]  *f: file pointer (unused)
    \param[out] none
    \retval     character sent
*/
/*!
    \brief      system initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void system_init(void)
{
    /* configure systick */
    systick_config();
    
    /* enable GPIO clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    
    /* enable TIMER clock for PWM */
    rcu_periph_clock_enable(RCU_TIMER1);
    
    /* enable USART clock for printf */
    rcu_periph_clock_enable(RCU_USART2);
    
    /* key button initialization (includes LED and key GPIO) */
    key_button_init();
    
    /* HUB reset pin initialization */
    hub_reset_init();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

void uart2_simple_init(void)
{
	  // 开启时钟
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
    
    // 配置 TX PB10
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    
    // 配置 RX PB11
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    
    // 串口参数配置
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    
    // 使能串口
    usart_enable(USART2);
}




int main(void)
{
    /* system initialization */
    system_init();
	
	 // 使用自定义函数初始化串口 USART2 (PB10/PB11)
    uart2_simple_init();

    /* motor control initialization */
    motor_control_init();
    
    /* current sensor initialization */
    current_sensor_init();
    
    /* temperature sensor initialization */
    temperature_sensor_init();
    
    /* Hall sensor initialization */
    hall_sensor_init();
    
    /* stroke counter initialization */
    stroke_counter_init();
    
    /* Modbus initialization */
    modbus_init();
		
		
	
  static uint8_t direction = 0;
  static uint16_t target_speed = 2000;
   motor_set_speed(target_speed, direction); //测试：反转100%速度
	 //test_pa5_pa6_adc();
    while (1 )
	{


        /* scan key buttons for motor control */
      key_scan();
        /* process motor overcurrent protection */
        motor_over_current_process();
        /* process Hall sensor data */
      hall_sensor_process();
        /* handle temperature overheat condition */
       //temperature_overheat_process();
        /* check position detection */
      position_detect_process();
      /* speed closed loop control */
      speed_closed_loop_control();


      //current_sensor_init_origin();
     // printf_adc_val0_and_val1_per_second();
      /* process Modbus messages */
        #if 0
        modbus_process();
        /* increment Modbus timeout counter */
        modbus_rx_timeout += 10;
        printf("\r\nBottom position detected");
        /* delay for 10ms */
        delay_1ms(10U);
        #endif

    }
}
