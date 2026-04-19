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

// 覆盖EVAL_COM0定义，使其指向USART2
#define EVAL_COM0 USART2

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
#include "pwm_test.h"

// 函数声明
void system_init(void);
void uart2_init(void);
void pwm_test_standalone(void);
void pwm_test_register(void);
void uart2_test_standalone(void);
void pwm_test_all(void);
void pwm_test_duty_adjust(void);
void pwm_test_pb13_pb14_duty_adjust(void);
/*!
    \brief      UART1 initialization for printf
    \param[in]  none
    \param[out] none
    \retval     none
*/

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
	
	  rcu_periph_clock_enable(RCU_GPIOB);

    /*Configure PA0 PA1 PA2(TIMER1 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	
	
	
	  gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
}



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




void uart2_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);
    
    /* enable AFIO clock */
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure USART2 remapping: try different remap configurations */
    /* First try direct register access */
    AFIO_PCF0 &= ~(BITS(4,5));  /* Clear USART2 remap bits */
    AFIO_PCF0 |= BIT(5);        /* Set for PB10/PB11 mapping */
    
    /* configure USART2 GPIO - PB10 (TX), PB11 (RX) */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);  // TX
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);  // RX
    
    /* configure USART2 */
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    
    /* enable USART2 */
    usart_enable(USART2);
    
    /* wait for USART2 to be ready */
    delay_1ms(10U);
    
    /* test direct UART transmission */
    usart_data_transmit(USART2, 'T');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, 'E');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, 'S');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, 'T');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, '\r');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, '\n');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
}

/*!
    \brief      redirect C library printf function to UART2
    \param[in]  ch: character to send
    \param[in]  *f: file pointer (unused)
    \param[out] none
    \retval     character sent
*/
#if defined(__GNUC__) && !defined(__clang__)
int __io_putchar(int ch)
{
    usart_data_transmit(USART2, (uint8_t)ch);
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    return ch;
}
#endif

#ifdef __CC_ARM
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART2, (uint8_t)ch);
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    return ch;
}
#endif

/* For Keil MDK compiler, fputc is already defined in gd32f307c_eval.c */

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
	#if 0
    // 只启用必要的时钟
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
   // rcu_periph_clock_enable(RCU_AF);
    
    // 配置 PB10/PB11 为 UART2
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);  // TX
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);  // RX
    
    // 配置 UART2
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
	#endif
	
	
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

void uart2_send_5_bytes(void)
{
    uint8_t data = 0x55;
	  uint32_t i = 0;
    //for(int i = 0; i < 5; i++) 
		while(1)
	  {
        while(!usart_flag_get(USART2, USART_FLAG_TBE));
        usart_data_transmit(USART2, data);
        while(!usart_flag_get(USART2, USART_FLAG_TC));
				for(i=0;i<60000;i++);
    }
}


int main(void)
{
    /* system initialization */
    system_init();
	
	
	 // 简单初始化 UART2
    uart2_simple_init();
    
    // 发送 5 个 0x0A
   //uart2_send_5_bytes();
	//while(1);
    
    /* test direct UART before any other initialization */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_AF);
    
    /* Try default PA2/PA3 first */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
    
    /* send test data */
    usart_data_transmit(USART2, 'H');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, 'I');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, '\r');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, '\n');
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    
    /* UART2 initialization for printf */
    uart2_init();

    //uart2_test_standalone();
    
    /* simple printf test */
    printf("OK\r\n");

    
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
    
    /* print system information */
    printf("\r\nMotor Control System Initialized");
    printf("\r\nCK_SYS: %d Hz", rcu_clock_freq_get(CK_SYS));
    
    /* test mode: run basic tests */
    printf("\r\n--- Test Mode ---\r\n");
    
    // Test 5: PWM output test (register level) - Run first to avoid interference
    printf("Test 5: PWM output test\r\n");
    //pwm_test_register();
    
    // Test 0: LED test
    printf("Test 0: LED test\r\n");
		#if 0
    for(int i = 0; i < 3; i++) {
        led_on(LED_FORWARD);
        delay_1ms(300U);
        led_off(LED_FORWARD);
        led_on(LED_REVERSE);
        delay_1ms(300U);
        led_off(LED_REVERSE);
        delay_1ms(300U);
    }
#endif
    // Test 1: Motor forward
    printf("Test 1: Motor forward at 500 RPM\r\n");
    led_on(LED_FORWARD);
    motor_set_speed(500, 0);
    //delay_1ms(3000U);
    motor_stop();
    led_off(LED_FORWARD);
   // delay_1ms(1000U);

    // Test 2: Motor reverse
    printf("Test 2: Motor reverse at 300 RPM\r\n");
    led_on(LED_REVERSE);
    motor_set_speed(300, 1);
    //delay_1ms(3000U);
    motor_stop();
    led_off(LED_REVERSE);
    //delay_1ms(1000U);

    // Test 3: Motor brake
    printf("Test 3: Motor brake test\r\n");
    
    // Test 4: HUB reset test
    printf("Test 4: HUB reset test\r\n");
    printf("Executing HUB reset...\r\n");
    //hub_reset();
    printf("HUB reset completed\r\n");
    
    // Test 4: Current and temperature reading
    printf("Test 4: Current and temperature reading\r\n");
    current_value = read_motor_current();
    temperature_value = read_temperature();
    printf("Current: %d mA, Temperature: %d.%d °C\r\n", 
           current_value, temperature_value / 10, temperature_value % 10);
    
    // Test 5: Stroke count
    printf("Test 5: Stroke count: %u\r\n", stroke_count);
    
    printf("\r\n--- Test Mode Complete ---\r\n");
    printf("\r\nEntering normal operation mode...\r\n");
  
	
	 // gpio_config();
  //  timer_config();
	//pwm_test_standalone();
	//pwm_test_all();
	//pwm_test_duty_adjust();
    //pwm_test_pb13_pb14_duty_adjust();
		//  while(1);
    while (1) {
        /* scan key buttons for motor control */
        key_scan();
        
        /* read motor current */
        current_value = read_motor_current();
        
        /* read temperature */
        temperature_value = read_temperature();
        
        /* process Hall sensor data */
        hall_sensor_process();
        
        /* check for overcurrent */
        if (current_value > MAX_CURRENT) {
            motor_brake();
            printf("\r\nOvercurrent detected: %d mA", current_value);
        }
        
        /* check position detection */
        top_position_detected = gpio_input_bit_get(TOP_POSITION_DETECT_PORT, TOP_POSITION_DETECT_PIN);
        bottom_position_detected = gpio_input_bit_get(BOTTOM_POSITION_DETECT_PORT, BOTTOM_POSITION_DETECT_PIN);
        position_detected = top_position_detected || bottom_position_detected;
        
        if (position_detected) {
            motor_stop();
            if (top_position_detected) {
                printf("\r\nTop position detected");
            } else if (bottom_position_detected) {
                printf("\r\nBottom position detected");
            }
        }
        
        /* process Modbus messages */
        modbus_process();
        
        /* increment Modbus timeout counter */
        modbus_rx_timeout += 10;
        
        /* delay for 10ms */
        delay_1ms(10U);
    }
}
