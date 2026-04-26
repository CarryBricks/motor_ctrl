/*!
    \file    modbus.c
    \brief   Modbus RTU communication module implementation

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

#include "modbus.h"
#include "motor_control.h"
#include "current_sensor.h"
#include "temperature_sensor.h"
#include "stroke_counter.h"




// 全局变量
uint8_t modbus_rx_buffer[MODBUS_BUFFER_SIZE];
uint8_t modbus_tx_buffer[MODBUS_BUFFER_SIZE];
uint16_t modbus_rx_length = 0;
uint16_t modbus_tx_length = 0;
uint32_t modbus_rx_timeout = 0;

// 局部函数声明
static uint16_t modbus_crc16(uint8_t *buffer, uint16_t length);
static void modbus_handle_read_holding_registers(uint8_t *rx_buffer, uint16_t rx_length);
static void modbus_handle_write_single_register(uint8_t *rx_buffer, uint16_t rx_length);

/*!
    \brief      This function handles USART1 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler(void)
{
    if(usart_flag_get(USART1, USART_FLAG_RBNE)){
        /* read data from USART1 */
        uint8_t data = usart_data_receive(USART1);

        motor_control_test(data);

        // if(data == 0x00) 
        // {
        //     // 这里可以添加一些特殊处理，例如重置接收缓冲区等
        //    // pwm_pb13_enable();
        // }
        // else if(data == 0x01)
        // {
        //    // pwm_pb13_disable();
        // }
        // else if (data == 0x02) 
        // {
        //    // pwm_pb14_enable();
        // }
        // else if (data == 0x03) 
        // {
        //    // pwm_pb14_disable();
        // }
        // else 
        {
            
        }
        


        
        // 处理Modbus数据接收
        if (modbus_rx_length < MODBUS_BUFFER_SIZE) 
        {
            modbus_rx_buffer[modbus_rx_length++] = data;
            modbus_rx_timeout = 0; // 重置超时计数器
        }
    }
    
    if(usart_flag_get(USART1, USART_FLAG_TBE)){
        // 发送数据处理
    }
}

/*!
    \brief      Modbus initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void modbus_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);
    
    /* configure USART GPIO - PA2 (TX), PA3 (RX) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    
    /* configure USART */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    
    /* enable USART */
    usart_enable(USART1);
    
    /* enable USART receive interrupt */
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    nvic_irq_enable(USART1_IRQn, 1U, 0U);
}

/*!
    \brief      process Modbus messages
    \param[in]  none
    \param[out] none
    \retval     none
*/
void modbus_process(void)
{
    /* check for timeout */
    if (modbus_rx_length > 0 && modbus_rx_timeout > MODBUS_TIMEOUT) {
        modbus_rx_length = 0;
    }
    
    /* check if we have a complete message */
    if (modbus_rx_length >= 8) {
        /* check slave address */
        if (modbus_rx_buffer[0] != MODBUS_SLAVE_ADDR) {
            modbus_rx_length = 0;
            return;
        }
        
        /* check CRC */
        uint16_t received_crc = (modbus_rx_buffer[modbus_rx_length - 1] << 8) | modbus_rx_buffer[modbus_rx_length - 2];
        uint16_t calculated_crc = modbus_crc16(modbus_rx_buffer, modbus_rx_length - 2);
        
        if (received_crc != calculated_crc) {
            modbus_rx_length = 0;
            return;
        }
        
        /* process function code */
        uint8_t function_code = modbus_rx_buffer[1];
        
        switch (function_code) {
            case MODBUS_FC_READ_HOLDING_REGISTERS:
                modbus_handle_read_holding_registers(modbus_rx_buffer, modbus_rx_length);
                break;
            case MODBUS_FC_WRITE_SINGLE_REGISTER:
                modbus_handle_write_single_register(modbus_rx_buffer, modbus_rx_length);
                break;
            default:
                /* unsupported function code */
                modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
                modbus_tx_buffer[1] = function_code | 0x80;
                modbus_tx_buffer[2] = MODBUS_ERR_ILLEGAL_FUNCTION;
                modbus_tx_length = 3;
                
                /* add CRC */
                uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
                modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
                modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
                
                /* send response */
                modbus_send_response(modbus_tx_buffer, modbus_tx_length);
                break;
        }
        
        /* reset receive buffer */
        modbus_rx_length = 0;
    }
}

/*!
    \brief      send Modbus response
    \param[in]  buffer: response buffer
    \param[in]  length: response length
    \param[out] none
    \retval     none
*/
void modbus_send_response(uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        while(!usart_flag_get(USART1, USART_FLAG_TBE));
        usart_data_transmit(USART1, buffer[i]);
    }
    
    /* wait for transmission complete */
    while(!usart_flag_get(USART1, USART_FLAG_TC));
}

/*!
    \brief      calculate Modbus CRC16
    \param[in]  buffer: data buffer
    \param[in]  length: data length
    \param[out] none
    \retval     CRC16 value
*/
static uint16_t modbus_crc16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/*!
    \brief      handle read holding registers request
    \param[in]  rx_buffer: received buffer
    \param[in]  rx_length: received length
    \param[out] none
    \retval     none
*/
static void modbus_handle_read_holding_registers(uint8_t *rx_buffer, uint16_t rx_length)
{
    /* extract starting address and quantity */
    uint16_t start_address = (rx_buffer[2] << 8) | rx_buffer[3];
    uint16_t quantity = (rx_buffer[4] << 8) | rx_buffer[5];
    uint32_t stroke_count = 0;
    /* check quantity */
    if (quantity > 125) {
        modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
        modbus_tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS | 0x80;
        modbus_tx_buffer[2] = MODBUS_ERR_ILLEGAL_DATA_VALUE;
        modbus_tx_length = 3;
        
        /* add CRC */
        uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
        modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
        modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
        
        /* send response */
        modbus_send_response(modbus_tx_buffer, modbus_tx_length);
        return;
    }
    
    /* prepare response */
    modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
    modbus_tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    modbus_tx_buffer[2] = quantity * 2;
    modbus_tx_length = 3;
    
    /* read registers */
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t address = start_address + i;
        uint16_t value = 0;
        
        switch (address) {
            case REG_MOTOR_STATE:
                value = (uint16_t)motor_state;
                break;
            case REG_CONTROL_MODE:
                value = (uint16_t)control_mode;
                break;
            case REG_TARGET_SPEED:
                value = target_speed;
                break;
            case REG_ACTUAL_SPEED:
                value = actual_speed;
                break;
            case REG_STROKE_COUNT:
                stroke_count = get_stroke_count();
                value = (uint16_t)(stroke_count & 0xFFFF);
                break;
            case REG_MOTOR_CURRENT:
                value = current_value;
                break;
            case REG_TEMPERATURE:
                value = temperature_value;
                break;
            case REG_POSITION_DETECT:
                value = position_detected;
                break;
            default:
                /* illegal data address */
                modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
                modbus_tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS | 0x80;
                modbus_tx_buffer[2] = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
                modbus_tx_length = 3;
                
                /* add CRC */
                uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
                modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
                modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
                
                /* send response */
                modbus_send_response(modbus_tx_buffer, modbus_tx_length);
                return;
        }
        
        /* add value to response */
        modbus_tx_buffer[modbus_tx_length++] = (value >> 8) & 0xFF;
        modbus_tx_buffer[modbus_tx_length++] = value & 0xFF;
    }
    
    /* add CRC */
    uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
    modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
    modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
    
    /* send response */
    modbus_send_response(modbus_tx_buffer, modbus_tx_length);
}

/*!
    \brief      handle write single register request
    \param[in]  rx_buffer: received buffer
    \param[in]  rx_length: received length
    \param[out] none
    \retval     none
*/
static void modbus_handle_write_single_register(uint8_t *rx_buffer, uint16_t rx_length)
{
    /* extract register address and value */
    uint16_t address = (rx_buffer[2] << 8) | rx_buffer[3];
    uint16_t value = (rx_buffer[4] << 8) | rx_buffer[5];
    
    /* write register */
    switch (address) {
        case REG_MOTOR_STATE:
            if (value < 4) {
                motor_state = (motor_state_t)value;
            }
            break;
        case REG_CONTROL_MODE:
            if (value < 2) {
                control_mode = (control_mode_t)value;
            }
            break;
        case REG_TARGET_SPEED:
            target_speed = value;
            break;
        case REG_COMMAND:
            switch (value) {
                case 0x00:
                    motor_stop();
                    break;
                case 0x01:
                    motor_set_speed(target_speed, 0);
                    break;
                case 0x02:
                    motor_set_speed(target_speed, 1);
                    break;
                case 0x03:
                    motor_brake();
                    break;
            }
            break;
        default:
            /* illegal data address */
            modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
            modbus_tx_buffer[1] = MODBUS_FC_WRITE_SINGLE_REGISTER | 0x80;
            modbus_tx_buffer[2] = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
            modbus_tx_length = 3;
            
            /* add CRC */
            uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
            modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
            modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
            
            /* send response */
            modbus_send_response(modbus_tx_buffer, modbus_tx_length);
            return;
    }
    
    /* prepare response */
    modbus_tx_buffer[0] = MODBUS_SLAVE_ADDR;
    modbus_tx_buffer[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
    modbus_tx_buffer[2] = (address >> 8) & 0xFF;
    modbus_tx_buffer[3] = address & 0xFF;
    modbus_tx_buffer[4] = (value >> 8) & 0xFF;
    modbus_tx_buffer[5] = value & 0xFF;
    modbus_tx_length = 6;
    
    /* add CRC */
    uint16_t crc = modbus_crc16(modbus_tx_buffer, modbus_tx_length);
    modbus_tx_buffer[modbus_tx_length++] = crc & 0xFF;
    modbus_tx_buffer[modbus_tx_length++] = (crc >> 8) & 0xFF;
    
    /* send response */
    modbus_send_response(modbus_tx_buffer, modbus_tx_length);
}
