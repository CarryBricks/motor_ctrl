/*!
    \file    current_sensor.c
    \brief   current sensor module implementation

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

#include "current_sensor.h"
#include "systick.h"
#include "motor_control.h"

// 全局变量
volatile uint16_t current_value = 0;
volatile uint16_t current_adc1_value = 0;
volatile uint16_t current_adc2_value = 0;

/*!
    \brief      current sensor initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void current_sensor_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    
    /* configure ADC GPIO - PA5 and PA6 */
    gpio_init(CURRENT_ADC1_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, CURRENT_ADC1_PIN);
    gpio_init(CURRENT_ADC2_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, CURRENT_ADC2_PIN);
    
    /* ADC configuration */
    adc_deinit(ADC0);
    
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    
    /* ADC special function config */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    
    /* ADC channel length config - 2 channels */
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 2);
    
    /* ADC channel config - channel 5 (PA5) as first channel */
    adc_routine_channel_config(ADC0, 0, CURRENT_ADC1_CHANNEL, ADC_SAMPLETIME_55POINT5);
    
    /* ADC channel config - channel 6 (PA6) as second channel */
    adc_routine_channel_config(ADC0, 1, CURRENT_ADC2_CHANNEL, ADC_SAMPLETIME_55POINT5);
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_EXTTRIG_ROUTINE_T1_CH1);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1U);
    
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    delay_1ms(1U);
}

/*!
    \brief      read motor current
    \param[in]  none
    \param[out] none
    \retval     current value in mA
*/
uint16_t read_motor_current(void)
{
    /* start ADC conversion */
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    
    /* wait for conversion complete */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    
    /* read ADC values - both channels */
    uint16_t adc1_value = adc_routine_data_read(ADC0);
    uint16_t adc2_value = adc_routine_data_read(ADC0);
    
    /* store individual ADC values */
    current_adc1_value = adc1_value;
    current_adc2_value = adc2_value;
    
    /* use the average of two channels for current calculation */
    uint16_t adc_value = (adc1_value + adc2_value) / 2;
    
    /* convert ADC value to current (mA) */
    /* assuming 10mΩ shunt resistor and 30x gain */
    uint16_t current = (adc_value * 3300) / 4096; // mV
    current = (current * 1000) / (30 * 10); // mA
    
    current_value = current;
    
    /* check for stall current */
    check_stall_current();
    
    return current;
}

/*!
    \brief      check for stall current and stop motor if necessary
    \param[in]  none
    \param[out] none
    \retval     none
*/
void check_stall_current(void)
{
    /* get motor state */
    motor_state_t state = motor_state;
    
    /* only check if motor is running */
    if (state == MOTOR_STATE_FORWARD || state == MOTOR_STATE_REVERSE) 
    {
        /* check both ADC channels for stall current */
        uint16_t adc1_current = (current_adc1_value * 3300) / 4096; // mV
        adc1_current = (adc1_current * 1000) / (30 * 10); // mA
        
        uint16_t adc2_current = (current_adc2_value * 3300) / 4096; // mV
        adc2_current = (adc2_current * 1000) / (30 * 10); // mA
        
        /* check if either channel exceeds stall threshold */
        if (adc1_current > STALL_CURRENT_THRESHOLD || adc2_current > STALL_CURRENT_THRESHOLD) 
        {
            /* stop motor immediately */
            motor_stop();
            
            /* set motor state to stopped */
            motor_state = MOTOR_STATE_STOP;
        }
    }
}


