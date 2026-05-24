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
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "modbus.h"
#include "motor_control.h"
#include "current_sensor.h"
#include "stroke_counter.h"
#include "systick.h"

// 全局变量
static uint8_t modbus_rx_buffer[256];
static uint16_t modbus_rx_len = 0;
static uint32_t modbus_last_time = 0;

void USART1_IRQHandler(void)
{
    if(usart_flag_get(USART1, USART_FLAG_RBNE))
    {
        uint8_t data = (uint8_t)usart_data_receive(USART1);

        if(modbus_rx_len < 256) {
            modbus_rx_buffer[modbus_rx_len++] = data;
            modbus_last_time = get_system_tick();
        }

        usart_flag_clear(USART1, USART_FLAG_RBNE);
    }
}

void modbus_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    usart_deinit(USART1);
    usart_baudrate_set(USART1, MODBUS_BAUDRATE);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    usart_enable(USART1);

    nvic_irq_enable(USART1_IRQn, 2U, 0U);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
}

static uint16_t modbus_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for(uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_send_response(uint8_t *buffer, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++) {
        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
        usart_data_transmit(USART1, buffer[i]);
    }
    while(RESET == usart_flag_get(USART1, USART_FLAG_TC));
}

void modbus_process(void)
{
    if(modbus_rx_len == 0) {
        return;
    }

    if(get_system_tick() - modbus_last_time > MODBUS_TIMEOUT) {
        modbus_rx_len = 0;
        return;
    }

    if(modbus_rx_len < 8) {
        return;
    }

    if(modbus_rx_buffer[0] != MODBUS_SLAVE_ID) {
        modbus_rx_len = 0;
        return;
    }

    uint16_t crc_received = (modbus_rx_buffer[modbus_rx_len-1] << 8) | modbus_rx_buffer[modbus_rx_len-2];
    uint16_t crc_calculated = modbus_crc(modbus_rx_buffer, modbus_rx_len-2);

    if(crc_received != crc_calculated) {
        modbus_rx_len = 0;
        return;
    }

    uint8_t function_code = modbus_rx_buffer[1];
    uint16_t start_address = (modbus_rx_buffer[2] << 8) | modbus_rx_buffer[3];
    uint16_t quantity = (modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];

    uint8_t response[256];
    uint16_t response_len = 0;

    switch(function_code) {
        case MODBUS_FC_READ_HOLDING_REGS:
            if(quantity > 125) {
                response[0] = MODBUS_SLAVE_ID;
                response[1] = function_code | 0x80;
                response[2] = MODBUS_ERR_ILLEGAL_DATA_VALUE;
                response_len = 3;
            } else {
                response[0] = MODBUS_SLAVE_ID;
                response[1] = function_code;
                response[2] = quantity * 2;
                response_len = 3;

                for(uint16_t i = 0; i < quantity; i++) {
                    uint16_t address = start_address + i;
                    uint16_t value = 0;

                    switch(address) {
                        case REG_MOTOR_MODE:
                            value = (uint16_t)motor_state;
                            break;
                        case REG_MOTOR_SPEED:
                            value = target_speed;
                            break;
                        case REG_MOTOR_STATE:
                            value = (uint16_t)motor_state;
                            break;
                        case REG_MOTOR_CURRENT:
                            value = current_value;
                            break;
                        case REG_MOTOR_STROKE:
                            value = (uint16_t)(get_stroke_count() & 0xFFFF);
                            break;
                        default:
                            break;
                    }

                    response[3 + i*2] = (value >> 8) & 0xFF;
                    response[4 + i*2] = value & 0xFF;
                }

                response_len = 3 + quantity * 2;
            }
            break;

        case MODBUS_FC_WRITE_SINGLE_REG:
            if(start_address == REG_MOTOR_CONTROL) {
                uint16_t value = (modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];

                switch(value) {
                    case 0x0001:
                        motor_set_speed(target_speed, 0);
                        break;
                    case 0x0002:
                        motor_set_speed(target_speed, 1);
                        break;
                    case 0x0003:
                        motor_stop();
                        break;
                    case 0x0004:
                        motor_brake();
                        break;
                    default:
                        break;
                }
            } else if(start_address == REG_MOTOR_SPEED) {
                target_speed = (modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];
            }

            response[0] = MODBUS_SLAVE_ID;
            response[1] = function_code;
            response[2] = modbus_rx_buffer[2];
            response[3] = modbus_rx_buffer[3];
            response[4] = modbus_rx_buffer[4];
            response[5] = modbus_rx_buffer[5];
            response_len = 6;
            break;

        default:
            response[0] = MODBUS_SLAVE_ID;
            response[1] = function_code | 0x80;
            response[2] = MODBUS_ERR_ILLEGAL_FUNCTION;
            response_len = 3;
            break;
    }

    uint16_t crc = modbus_crc(response, response_len);
    response[response_len++] = crc & 0xFF;
    response[response_len++] = (crc >> 8) & 0xFF;

    modbus_send_response(response, response_len);
    modbus_rx_len = 0;
}