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
#include "stdio.h"

// 全局变量
volatile uint16_t current_value = 0;
volatile uint16_t current_adc1_value = 0;
volatile uint16_t current_adc2_value = 0;

// 滑动窗口滤波参数
#define FILTER_WINDOW_SIZE  10
static uint16_t adc_filter_buffer[FILTER_WINDOW_SIZE] = {0};  // 滤波缓冲区
static uint8_t adc_filter_index = 0;                           // 当前缓冲区索引
static uint32_t adc_filter_sum = 0;                            // 当前缓冲区总和
static uint8_t adc_filter_count = 0;                           // 当前有效数据个数

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
    //adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_EXTTRIG_ROUTINE_T1_CH1);
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE);
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


    //printf("ADC1=%d, ADC2=%d\n", adc1_value, adc2_value);

    

    /* store individual ADC values */
    current_adc1_value = adc1_value;
    current_adc2_value = adc2_value;
    
    /* use the average of two channels for current calculation */
    uint16_t raw_adc_value = (adc1_value + adc2_value) / 2;
    
    /* 滑动窗口滤波：当adc1_value或adc2_value为0时不计入 */
    uint16_t adc_value = 0;
    if (adc1_value != 0 && adc2_value != 0)
    {
        // 从总和中减去即将被替换的旧值
        adc_filter_sum -= adc_filter_buffer[adc_filter_index];
        
        // 存储新值到缓冲区
        adc_filter_buffer[adc_filter_index] = raw_adc_value;
        
        // 将新值加入总和
        adc_filter_sum += raw_adc_value;
        
        // 更新索引
        adc_filter_index = (adc_filter_index + 1) % FILTER_WINDOW_SIZE;
        
        // 更新有效数据个数（达到窗口大小后保持不变）
        if (adc_filter_count < FILTER_WINDOW_SIZE)
        {
            adc_filter_count++;
        }
        
        // 计算平均值
        adc_value = (uint16_t)(adc_filter_sum / adc_filter_count);
    }
    else
    {
        // 如果任一通道为0，使用上一次的滤波值（保持稳定）
        // 如果是第一次且值为0，使用原始值
        if (adc_filter_count > 0)
        {
            adc_value = (uint16_t)(adc_filter_sum / adc_filter_count);
        }
        else
        {
            adc_value = raw_adc_value;
        }
    }
    
    /* convert ADC value to current (mA) */
    /* assuming 10mΩ shunt resistor and 30x gain */
    double  current = ((double)(adc_value) * 3300) / 4096; // mV
    current = (current * 1000) / (30 * 10); // mA
    
    current_value = current;

    printf("motor Current=%f mA\n", current);
    
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
        /* check both ADC cchannels for stall current */
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





#if  0
void adc_config(void)
{
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    // 两路通道
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 2);

    // ===================== 关键修改 =====================
    // 通道0：PA5 = ADC_CHANNEL_5
    adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);
    // 通道1：PA6 = ADC_CHANNEL_6
    adc_routine_channel_config(ADC0, 1, ADC_CHANNEL_6, ADC_SAMPLETIME_55POINT5);
    // ====================================================

    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, ENABLE);

    // 关闭内部传感器（我们用外部引脚）
    // adc_tempsensor_vrefint_enable(); 

    adc_dma_mode_enable(ADC0);
    adc_enable(ADC0);
    delay_1ms(1);
    adc_calibration_enable(ADC0);

    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
}

void dma_config(void)
{
    dma_parameter_struct dma_data_parameter;

    dma_deinit(DMA0, DMA_CH0);

    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 2;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);
    dma_channel_enable(DMA0, DMA_CH0);
}


// 新增：配置 PA5 PA6 为模拟输入
void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_6);
}

void rcu_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
}
#endif