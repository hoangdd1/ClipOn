/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "system_info.h"
#include "hitless_data.h"  
#include "gpio.h"

#define SAMPLES_IN_BUFFER 1                         // 1 samples for every channle
//#define BAT_PIN NRF_SAADC_INPUT_AIN6                // Analog input 6 (AIN6)
//#define USB_PIN NRF_SAADC_INPUT_AIN7                // Analog input 7 (AIN7)
#define VDD_PIN NRF_SAADC_INPUT_VDD                 // VDD as input
//#define BAT_CHANNEL 0                               // SAADC channel num         
//#define USB_CHANNEL 1                               // SAADC channel num
#define VDD_CHANNEL 0                               // SAADC channel num
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(4);
static nrf_saadc_value_t     adc_buf[SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
extern void wrap_NVIC_SystemReset();


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
;
}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


#define NEW_FACTOR_ADC 1.968 //(1/25.4)*50

#define OLD_FACTOR_ADC 0.3515625 //(3.6/1024)*100

#define NEW_FACTOR_vbat 0.38671875 //( 3.6/1024 )*11*100/10           //divide in 10- the vbat for upload data of range 0-1000 to uint8_t variable.

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {        
//       hitless_ptr()->vbat = (int16_t)(p_event->data.done.p_buffer[BAT_CHANNEL]*NEW_FACTOR_vbat); 
//        hitless_ptr()->v_usb = (int16_t)(p_event->data.done.p_buffer[USB_CHANNEL]*OLD_FACTOR_ADC);
        hitless_ptr()->v_vdd = (int16_t)(p_event->data.done.p_buffer[VDD_CHANNEL]*OLD_FACTOR_ADC);
        m_adc_evt_counter++;
        hitless_ptr()->ADC = 16000;
        wrap_NVIC_SystemReset();
            
    } else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
        // don't do anything - we call calibrate every now and then and this comes back
    } else if (p_event->type ==  NRFX_SAADC_EVT_LIMIT) {
        // not setting any limits with nrfx_saadc_limits_set so should never fire
    }    
  
}


void saadc_init(void)
{
    ret_code_t err_code;
//    nrf_saadc_channel_config_t channel_config_bat =
//    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BAT_PIN);//AIN6
//    
//    nrf_saadc_channel_config_t channel_config_usb =
//    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(USB_PIN); //AIN7

    nrf_saadc_channel_config_t channel_config_vdd =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VDD_PIN); //VDD
     
    //channel_config_vdd.reference = NRF_SAADC_REFERENCE_INTERNAL;  
    
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    // do a calibrate    
    while (nrfx_saadc_calibrate_offset() != NRFX_SUCCESS)
        nrf_delay_ms(10);
    while (nrfx_saadc_is_busy())
        nrf_delay_ms(10);
        
//    err_code = nrf_drv_saadc_channel_init(BAT_CHANNEL, &channel_config_bat); //AIN6
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = nrf_drv_saadc_channel_init(USB_CHANNEL, &channel_config_usb); //AIN7
//    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_channel_init(VDD_CHANNEL, &channel_config_vdd); //VDD
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(adc_buf, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);  
      
}

void init_ADC()
{  
    saadc_gpio_init(); 
    saadc_init();
    /* I/O setting */
    saadc_sampling_event_init();
    saadc_sampling_event_enable(); 
}