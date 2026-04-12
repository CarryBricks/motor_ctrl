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

// 函数声明
void system_init(void);

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
    
    /* enable USART clock for Modbus */
    rcu_periph_clock_enable(RCU_USART0);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* system initialization */
    system_init();
    
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
    printf("\r\n\r\n--- Test Mode ---\r\n");
    
    // Test 1: Motor forward
    printf("Test 1: Motor forward at 500 RPM\r\n");
    motor_set_speed(500, 0);
    delay_1ms(3000U); // Run for 3 seconds
    motor_stop();
    delay_1ms(1000U);
    
    // Test 2: Motor reverse
    printf("Test 2: Motor reverse at 300 RPM\r\n");
    motor_set_speed(300, 1);
    delay_1ms(3000U); // Run for 3 seconds
    motor_stop();
    delay_1ms(1000U);
    
    // Test 3: Motor brake
    printf("Test 3: Motor brake test\r\n");
    motor_set_speed(200, 0);
    delay_1ms(1000U);
    motor_brake();
    delay_1ms(1000U);
    
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
    
    while (1) {
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
        position_detected = gpio_input_bit_get(STROKE_POSITION_DETECT_PORT, STROKE_POSITION_DETECT_PIN);
        if (position_detected) {
            motor_stop();
            printf("\r\nPosition detected");
        }
        
        /* process Modbus messages */
        modbus_process();
        
        /* increment Modbus timeout counter */
        modbus_rx_timeout += 10;
        
        /* delay for 10ms */
        delay_1ms(10U);
    }
}
