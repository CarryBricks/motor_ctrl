/*!
    \file    pwm_test.h
    \brief   PWM output test module

    \version 2026-4-18, V1.0.0, firmware for GD32F30x
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

#ifndef PWM_TEST_H
#define PWM_TEST_H

#include "gd32f30x.h"
#include "motor_control.h"

/*!
    \brief      initialize PWM test
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_test_init(void);

/*!
    \brief      test all PWM output pins
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_test_all_pins(void);

/*!
    \brief      test specific PWM output pin
    \param[in]  pin_name: pin name as string
    \param[in]  port: GPIO port
    \param[in]  pin: GPIO pin
    \param[in]  duty_cycle: duty cycle (0-100)
    \param[out] none
    \retval     none
*/
void pwm_test_single_pin(const char *pin_name, uint32_t port, uint32_t pin, uint8_t duty_cycle);

/*!
    \brief      test hardware PWM on specific channel
    \param[in]  pin_name: pin name as string
    \param[in]  timer_periph: timer peripheral
    \param[in]  channel: timer channel
    \param[in]  duty_cycle: duty cycle (0-100)
    \param[out] none
    \retval     none
*/
void pwm_test_hardware_pwm(const char *pin_name, uint32_t timer_periph, uint32_t channel, uint8_t duty_cycle);

#endif /* PWM_TEST_H */
