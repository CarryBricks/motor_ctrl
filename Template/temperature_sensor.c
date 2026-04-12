/*!
    \file    temperature_sensor.c
    \brief   temperature sensor module implementation

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

#include "temperature_sensor.h"

// 全局变量
volatile uint16_t temperature_value = 0;

/*!
    \brief      temperature sensor initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void temperature_sensor_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /* configure temperature sensor GPIO */
    gpio_init(TEMPERATURE_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, TEMPERATURE_ADC_PIN);
}

/*!
    \brief      read temperature
    \param[in]  none
    \param[out] none
    \retval     temperature value in 0.1°C
*/
uint16_t read_temperature(void)
{
    /* start ADC conversion */
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    
    /* wait for conversion complete */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    
    /* read temperature value */
    uint16_t adc_value = adc_routine_data_read(ADC0);
    
    /* convert ADC value to temperature */
    /* assuming NTC thermistor */
    // Simplified temperature calculation
    uint16_t temperature = (3300 - (adc_value * 3300 / 4096)) / 10;
    
    temperature_value = temperature;
    return temperature;
}
