/*!
    \file    uart2_test.c
    \brief   UART2 direct byte output test

    \version 2026-4-19, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x.h"
#include "systick.h"

void uart2_init_test(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);
    
    /* enable AF clock */
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure USART2 remapping for PB10/PB11 */
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
}

void uart2_send_bytes(void)
{
    uint8_t data = 0x0A;  // 换行符
    uint8_t count = 5;
    
    /* initialize UART2 */
    uart2_init_test();
    
    /* send 5 bytes of 0x0A */
    for(uint8_t i = 0; i < count; i++) {
        /* wait for transmit buffer to be empty */
        while(!usart_flag_get(USART2, USART_FLAG_TBE));
        
        /* send data */
        usart_data_transmit(USART2, data);
        
        /* wait for transmission complete */
        while(!usart_flag_get(USART2, USART_FLAG_TC));
        
        /* small delay */
        delay_1ms(100U);
    }
    
    /* send a final carriage return */
    while(!usart_flag_get(USART2, USART_FLAG_TBE));
    usart_data_transmit(USART2, 0x0D);  // 回车符
    while(!usart_flag_get(USART2, USART_FLAG_TC));
}

void uart2_test_standalone(void)
{
    uart2_send_bytes();
    while(1);
}