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
#include <stdio.h>

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
        }
        modbus_last_time = get_system_tick();

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
    usart_baudrate_set(USART1, CUSTOM_BAUDRATE);
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

static void send_response_frame(uint8_t func, uint8_t *data, uint8_t data_len)
{
    uint8_t frame[64];
    uint16_t crc;

    frame[0] = CUSTOM_SOF;
    frame[1] = SLAVE_ADDR;
    frame[2] = func;
    frame[3] = data_len;

    for(uint8_t i = 0; i < data_len; i++) {
        frame[4 + i] = data[i];
    }

    crc = modbus_crc(&frame[2], 2 + data_len);
    frame[4 + data_len] = crc & 0xFF;
    frame[5 + data_len] = (crc >> 8) & 0xFF;
    frame[6 + data_len] = CUSTOM_EOF;

    for(uint16_t i = 0; i < 7 + data_len; i++) {
        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
        usart_data_transmit(USART1, frame[i]);
    }
    while(RESET == usart_flag_get(USART1, USART_FLAG_TC));
}

void modbus_send_event(uint8_t event_type, uint16_t current)
{
    uint8_t data[4];
    data[0] = (event_type >> 8) & 0xFF;
    data[1] = event_type & 0xFF;
    data[2] = (current >> 8) & 0xFF;
    data[3] = current & 0xFF;
    send_response_frame(FC_EVENT, data, 4);
}

void modbus_upload_status(void)
{
    uint8_t data[12];
    uint32_t stroke = get_stroke_count();

    data[0] = (motor_state >> 8) & 0xFF;
    data[1] = motor_state & 0xFF;
    data[2] = (current_value >> 8) & 0xFF;
    data[3] = current_value & 0xFF;
    data[4] = (actual_speed >> 8) & 0xFF;
    data[5] = actual_speed & 0xFF;
    data[6] = (stroke >> 8) & 0xFF;
    data[7] = stroke & 0xFF;
    data[8] = 0;
    data[9] = 0;
    data[10] = 0;
    data[11] = (position_detected != 0) ? 1 : 0;

#if 0
    printf("\r\n[U] upload: state=%d cur=%d spd=%d strk=%d\n",
           motor_state, current_value, actual_speed, (int)stroke);
#endif
    send_response_frame(FC_RESPONSE, data, 12);
}

void modbus_process(void)
{
    uint16_t i, frame_len, crc_calc, crc_recv;
    uint8_t func, data_len;
    uint16_t cmd, speed;

    if(modbus_rx_len == 0) {
        return;
    }

    if(get_system_tick() - modbus_last_time > CUSTOM_TIMEOUT) {
        modbus_rx_len = 0;
        return;
    }

    i = 0;
    while(i < modbus_rx_len && modbus_rx_buffer[i] != CUSTOM_SOF) {
        i++;
    }
    if(i > 0) {
        for(uint16_t j = 0; j < modbus_rx_len - i; j++) {
            modbus_rx_buffer[j] = modbus_rx_buffer[i + j];
        }
        modbus_rx_len -= i;
    }

    if(modbus_rx_len < 7) {
        return;
    }

    data_len = modbus_rx_buffer[3];
    frame_len = 7 + data_len;
    if(modbus_rx_len < frame_len) {
        return;
    }

    if(modbus_rx_buffer[1] != SLAVE_ADDR) {
        modbus_rx_len = 0;
        return;
    }

    if(modbus_rx_buffer[frame_len - 1] != CUSTOM_EOF) {
        for(i = 0; i < modbus_rx_len - 1; i++) {
            modbus_rx_buffer[i] = modbus_rx_buffer[i + 1];
        }
        modbus_rx_len--;
        return;
    }

    crc_recv = (modbus_rx_buffer[frame_len - 2] << 8) | modbus_rx_buffer[frame_len - 3];
    crc_calc = modbus_crc(&modbus_rx_buffer[2], 2 + data_len);
    if(crc_recv != crc_calc) {
        for(i = 0; i < modbus_rx_len - frame_len; i++) {
            modbus_rx_buffer[i] = modbus_rx_buffer[frame_len + i];
        }
        modbus_rx_len -= frame_len;
        return;
    }

    func = modbus_rx_buffer[2];

    if(func == FC_CONTROL && data_len >= 4) {
        cmd = (modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];
        speed = (modbus_rx_buffer[6] << 8) | modbus_rx_buffer[7];

        if(cmd == CMD_FORWARD) {
            motor_set_speed(speed, 0);
        } else if(cmd == CMD_REVERSE) {
            motor_set_speed(speed, 1);
        } else if(cmd == CMD_STOP) {
            motor_stop_ramp();
        }
        modbus_upload_status();
    }

    for(i = 0; i < modbus_rx_len - frame_len; i++) {
        modbus_rx_buffer[i] = modbus_rx_buffer[frame_len + i];
    }
    modbus_rx_len -= frame_len;
}