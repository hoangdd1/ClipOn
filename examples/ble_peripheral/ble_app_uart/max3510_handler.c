/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "gpio.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "max3510x.h"
#include "transducer.h"
#include "message_handler.h"
#include "general.h"   
#include "uart_commands.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */

extern void push_event(uint8_t *str_event , int16_t str_event_len);

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    //NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        //NRF_LOG_INFO(" Received:");
        //NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}
void init_board();

#define CUBIC_METERS_TO_GALLONS	264.172
#define LITER_PER_HOUR_CONVERSION 3000000 

static max3510x_tof_results_t 	s_tof_results;
static volatile uint32_t        s_results_count;
static volatile uint32_t	s_timeout_count;
max3510x_t g_max35103;
bool max5310x_int_isr_flag = 0;
void max3510x_int_isr()
{
   
    if ( max5310x_int_isr_flag )
    {
        // handle MAX35103 interrupts
        uint16_t status = max3510x_interrupt_status( &g_max35103 );
        
        if( (status & (MAX3510X_REG_INTERRUPT_STATUS_TOF|MAX3510X_REG_INTERRUPT_STATUS_TO)) == MAX3510X_REG_INTERRUPT_STATUS_TOF )
        {
                max3510x_read_tof_results( &g_max35103, &s_tof_results );
                hitless_ptr()->device_bit = SPI_READ_ISR_STATUS_SUCCESS; 
                s_results_count++;
        }
        else if( status )
        {
                s_timeout_count++;
                hitless_ptr()->device_bit = SPI_ERROR_READ_ISR_STATUS;   
        }
        max5310x_int_isr_flag = 0;
    }    
}

extern int ble_device_during_session;  
volatile uint32_t time_max3510 = 0;

/* SysTick interrupt Handler. */
void SysTick_Handler2(void)  {
    time_max3510++;
    //int32_t status = 0;
	// systick provides the sampling timebase
     if(!s_results_count )
     {
#ifndef DEBUG_COMMU       
	max3510x_tof_diff(&g_max35103); 
#endif        
     }
     //if (ble_device_during_session == 1)       //rd 1
     //  bsp_board_led_invert(BSP_BOARD_LED_0);  //green  
     //else
     //  bsp_board_led_off(BSP_BOARD_LED_0);    //green
}

char print_msg[128];
bool led_state = false;
double_t last_flow = 0.0, sum_flow, flow, second_data=0.0, first_data=0.0, gap ;
uint32_t one_second = 0;
int32_t shutoff = 100.0 /** board_read_bcd_switches()*/;

#define DETECT_BUFF_SIZE 2                //every 10 sec the buffer's avg will be checked.
float detection_buffer[DETECT_BUFF_SIZE];  
int detection_buffer_index = 0;
bool unusage_alert_flag = false, pizuz_alert_flag = false, leakage_alert_flag = false, leakage_alert_flag_off = false, unusage_alert_flag_off = false;

void alerts_detection() //alerts_detection colects every DETECT_BUFF_SIZE flow data and check if their avg is over the limit 
{
  
  float average_detection = 0,sum = 0;
  detection_buffer[ detection_buffer_index ] = last_flow;
  detection_buffer_index++;
 
  if ( detection_buffer_index >= DETECT_BUFF_SIZE ){                //buffer is full
    detection_buffer_index = 0;                                    
    for( int i = 0; i < DETECT_BUFF_SIZE; i++ )                     //average calculation
       sum += detection_buffer[i]; 
                                                 
    average_detection = sum / DETECT_BUFF_SIZE;
    last_flow = average_detection;                                  //save the received average(from DETECT_BUFF_SIZE data) for uploading.
    
    if ( hitless_ptr()->bubbles_error_counter < DETECT_BUFF_SIZE ){ //when the bubbles error is not accured.
       /* Pitzuz */
      if ( average_detection >= hitless_ptr()->detection_limit ) //checking the average compared to the pizuz limit. 
        hitless_ptr()->limit_count_result[1]++; 
      else
        hitless_ptr()->limit_count_result[1] = 0;
      
        if ( hitless_ptr()->limit_count_result[1] >= hitless_ptr()->limit_count_result[0] )//if go over the limit more then "limit_count_result[0]" times in a row its pizuz.
        {
             hitless_ptr()->limit_count_result[1] = 0;
             pizuz_alert_flag = true;
             return;
        }
        /* Unusage */
        if ( ( average_detection < hitless_ptr()->unusage_limit ) && ( average_detection > (-1)*hitless_ptr()->unusage_limit ) )//checking the average compared to the unusage_limit.
          hitless_ptr()->unusage_limit_counter[1]++; 
        else {
               hitless_ptr()->unusage_limit_counter[1] = 0;
               if ( hitless_ptr()->unusage_alert_status ) {//there was alert of unusage
                  hitless_ptr()->unusage_alert_status = 0; //alert off
                  unusage_alert_flag_off = true;
                  return;
               }   
        }
      
        if ( hitless_ptr()->unusage_limit_counter[1] >= hitless_ptr()->unusage_limit_counter[0] )//if go over the limit more then "unusage_limit_counter[0]" times in a row its unusage alert.
        {
             hitless_ptr()->unusage_limit_counter[1] = 0;
             hitless_ptr()->unusage_alert_status = 1; //alert on 
             unusage_alert_flag = true;
             return;
        }
       /* Leakage */
       if ( average_detection >= hitless_ptr()->drip_test_limit ){//checking the average compared to the leak limit.
            hitless_ptr()->leak_counter[1]++;//n
            /* Moving average formula : avg = ((avg*(n-1))+flow)/n */
            hitless_ptr()->leak_avg = ((( hitless_ptr()->leak_avg * (hitless_ptr()->leak_counter[1]-1)) + last_flow) / hitless_ptr()->leak_counter[1]);
       }
       else {
            hitless_ptr()->leak_counter[1] = 0;
            hitless_ptr()->leak_avg = 0;        //reset the leak avg calculation
            if ( hitless_ptr()->leak_status ) { //there was alert of leak
              hitless_ptr()->leak_status = 0;   //no leakage
              leakage_alert_flag_off = true;
              return;
            }
       }
      
      if ( hitless_ptr()->leak_counter[1] >= hitless_ptr()->leak_counter[0] )//if go over the limit more then "leak_counter[0]" times in a row its leakage.
      {
           hitless_ptr()->leak_counter[1] = 0;
           leakage_alert_flag = true;
           hitless_ptr()->leak_status = (int)hitless_ptr()->leak_avg; //save the leak status on device.
           return;
      }
    }
  }
}
        
int init_water_sense(void)
{
   
        bsp_board_init(BSP_INIT_LEDS);
        APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
        NRF_LOG_DEFAULT_BACKENDS_INIT();

        nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
        spi_config.ss_pin   = SPI_SS_PIN;
        spi_config.miso_pin = SPI_MISO_PIN;
        spi_config.mosi_pin = SPI_MOSI_PIN;
        spi_config.sck_pin  = SPI_SCK_PIN;
        APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

        //NRF_LOG_INFO("Read Sensor started.");
#ifndef DEBUG_COMMU      
        init_board();
#endif       
        
         
        s_results_count = 0;
	gpio_init();
        //SysTick_Config(SystemCoreClock / 12800); // 50Hz sampling rate
        //NVIC_EnableIRQ(SysTick_IRQn);	
        //while(1);
        return 1;

} 
extern void change_water_flow(char param);
int32_t counter_down = 3000000;     
extern uint32_t nus_send_data_buff(uint8_t *send_buff, int send_buff_length);
extern void wrap_NVIC_SystemReset();

void update_water_sense()
{

        
        nrf_delay_ms(20);
      
        SysTick_Handler2();
#ifndef DEBUG_COMMU          
        counter_down = 3000000;
        while(max5310x_int_isr_flag == 0&&counter_down!=0) {
          nrf_delay_us(1);
          counter_down--;
        }
       
        if(counter_down==0)
        {
           hitless_ptr()->device_bit = SPI_WAIT_FOR_ISR;
           wrap_NVIC_SystemReset(); 
        }        
        max3510x_int_isr(); 

        flow = 0;
        
        if( s_results_count )
        {
                // these checks can help flush out SPI issues.  Provided here as a refernce for new board designs.
                if( max3510x_validate_measurement( &s_tof_results.up, transducer_hit_count() ) &&
                        max3510x_validate_measurement( &s_tof_results.down, transducer_hit_count() ) )
                {
                        float_t t2_ideal = max3510x_ratio_to_float(s_tof_results.up.t2_ideal);
                        float_t t1_t2 = max3510x_ratio_to_float(s_tof_results.up.t1_t2);
                        if( t2_ideal >= 0.95f && t1_t2 > 0.50f )
                        {
                                // Use the ratios to validate measuremnets.  If air bubbles are in the flow body, attenuation
                                // can cause the t1 wave to be missed.  Production applicaiotns can use more sophisticatd recovery
                                // logic, but here we just duplicate the last valid sample.  This works for itermittant attenuation
                                // due to the occastional bubble, but is insuffecient for applications that have large amounts
                                // of undissolved gases present in the flow body.
                                
                                // For more information about attenuation due to undissolved gases, see Maxim application note 6357 "Dealing with Bubbles"
                                
                                double_t up = max3510x_fixed_to_double( &s_tof_results.up.average );
                                double_t down = max3510x_fixed_to_double( &s_tof_results.down.average );
                                flow = last_flow = LITER_PER_HOUR_CONVERSION*(hitless_ptr()->change_flow_sign)*transducer_flow( up, down );//duplicate at (-1/1)"CS"
                                hitless_ptr()->last_flow = last_flow; //save the last valide flow 
                                hitless_ptr()->bubbles_error_counter = 0;
                        }
                }
                // This patch helps us deal with the problem of bubbles in the piping and abnormal flow readings.
                // this is a temporary solution to the problem.
                if( !flow )
                {
                      flow = last_flow = hitless_ptr()->last_flow;
                      hitless_ptr()->bubbles_error_counter++; //count the number of bubbles error until the valid sensor reading 
                }
                
                sum_flow += (flow * CUBIC_METERS_TO_GALLONS/50.0);	// scale flow to gallons based on 100Hz sampling rate
                s_results_count = 0;			
        }
      
#endif      
        
   /*     if(  shutoff <= sum_flow )
          {
                //board_relay(false);
                //board_printf("RELAY OFF\r\n");
                //while(1);	// done for this cycle
          }
   */
        
        if( (time_max3510 - one_second) > 10 )   
        {
                //board_led(led_state);
                //led_state = !led_state;
                union_measure_data_t data_measure;
                char data_in[EVENT_SIZE];
                int event_temp_size;
                memset(print_msg,0x0,128); 
                memset(data_in,0x0,EVENT_SIZE); 
               
                data_measure.data.A = last_flow;
                data_measure.data.B = (last_flow * CUBIC_METERS_TO_GALLONS * 60.0);
                data_measure.data.C = (hitless_ptr()->v_usb);//temporary parameter for save the USB status- shir 
#ifdef DEBUG_COMMU  
                data_measure.data.A = 0.12345;
                data_measure.data.B = 1.98765;                
                data_measure.data.C = 2.12345;
                data_measure.data.D = 2.76543;
#endif                
                 //drip test - CT command.
                 switch(hitless_ptr()->drip_test_stages){ // The system checks if tiny drip is exist .
                  case -1:                               //when the flag is down
                        break;
                  case 0:                                //when the flag is down
                        break;       
                  case 1: 
                        change_water_flow('1');         //stop flowing 
                        hitless_ptr()->drip_test_stages++;
                        break;
                 case 90:
                        change_water_flow('0');         //start flowing  
                        hitless_ptr()->drip_test_stages++;
                        break;
                 case 91:
                        first_data=last_flow;          //save the first data
                        hitless_ptr()->drip_test_stages++;
                        break;
                 case 92:
                        second_data=last_flow;         //save the second data + checking
                        gap = first_data - second_data;
                        if ( gap > hitless_ptr()->drip_test_limit )
                        {
                          sprintf(hitless_ptr()->critic_msg,"[3]Warning water leak: The gap is:[%f]",gap);
                          hitless_ptr()->leak_status = (int)gap;
                        }
                        else if( gap == 0 )
                          sprintf(hitless_ptr()->critic_msg,"[!]CT went wrong: The gap is:[%f]",gap); //the error of bubbles accured, so the gap is 0.
                        else
                        {
                          sprintf(hitless_ptr()->critic_msg,"[4]Everything is fine: The gap is:[%f]",gap); 
                          hitless_ptr()->leak_status = 0; //no leakage
                        }
                        hitless_ptr()->send_critical_msg = 1;         //Need to send a Critical message-via modem.
                        hitless_ptr()->drip_test_stages = -1;         //Reset drip_test_status
                        hitless_ptr()->check_message_coutdown = 10;   //KP_0
                        hitless_ptr()->device_state = SLEEP_STATE;    //MO_0
                        wrap_NVIC_SystemReset ();  
                        break;
                 default:
                        hitless_ptr()->drip_test_stages++;
                        break; 
                 }
               
                 //Detection of PIZUZ, every 30 second check 3 data average and save the average on buff.
                 if(hitless_ptr()->detect_pizuz_flags[1] > 0)
                 {
                    hitless_ptr()->detect_pizuz_flags[1]--;
                    alerts_detection(); //Finally "last_flow" will contain the received average.
                    
                    if (hitless_ptr()->detect_pizuz_flags[1]==0){
                       //Update the received average as data.
                       data_measure.data.A = last_flow;  
                       data_measure.data.B = (last_flow * CUBIC_METERS_TO_GALLONS * 60.0);
                        if( hitless_ptr()->vbat || ( hitless_ptr()->event_buff.event_buffer_index > 2 ) ){//Check if the adc accrued for battery voltage
                         //Save the data on the buffer.
                         event_encapsulation(1,data_measure.buff,sizeof(data_measure.buff),(uint8_t *)print_msg,&event_temp_size); 
                         push_event((uint8_t *)print_msg,EVENT_SIZE-1);
                        }
                       //checking for alerts
                       if ( unusage_alert_flag || pizuz_alert_flag || leakage_alert_flag || leakage_alert_flag_off )   //The system detects a water critical use (unusage or an explosion or leakage).
                      {
                         if (unusage_alert_flag)  
                            sprintf( hitless_ptr()->critic_msg,"[5]Water critical unusage\n[%.1f]l/h [%d] times" , hitless_ptr()->unusage_limit, hitless_ptr()->unusage_limit_counter[0]); 
                         else if (pizuz_alert_flag) 
                            sprintf( hitless_ptr()->critic_msg,"[2]Crossed:[%.1f]l/h->[%.0f] times!\nsend SD command" , hitless_ptr()->detection_limit, hitless_ptr()->limit_count_result[0]);
                         else if (leakage_alert_flag) 
                           sprintf(hitless_ptr()->critic_msg,"[6]Warning water leak: crossed:[%d]l/h [%d]times", hitless_ptr()->leak_status, hitless_ptr()->leak_counter[0]);
                         else if (leakage_alert_flag_off) 
                           sprintf(hitless_ptr()->critic_msg,"[7]Water leak stopped: leak status:[%d]", hitless_ptr()->leak_status);
                         else if (unusage_alert_flag_off)
                           sprintf(hitless_ptr()->critic_msg,"[8]Unusage alert stopped: alert status:[%d]", hitless_ptr()->unusage_alert_status);
                         
                         hitless_ptr()->send_critical_msg = 1;         //There is a Critical message 
                         hitless_ptr()->upload_data[0] = (leakage_alert_flag_off || unusage_alert_flag_off) ? 600 : 1;  //Upload the data from the detection period, after the message will be sent.
                         hitless_ptr()->check_message_coutdown = 10;   //KP_0
                      }
                    
                      hitless_ptr()->detect_pizuz_flags[0] = 30;   
                      hitless_ptr()->device_state = SLEEP_STATE;    //MO_0
                      wrap_NVIC_SystemReset();   
                    }
                    
                 }
                 
                         
                if ( ble_device_during_session == 1)//RD_1 
                {
                  event_encapsulation(1,data_measure.buff,sizeof(data_measure.buff),(uint8_t *)print_msg,&event_temp_size); 
                  if( hitless_ptr()->vbat != 0 )//Save just a valid battery values in the buffer.
                      push_event((uint8_t *)print_msg,EVENT_SIZE-1); 
//#ifndef DEBUG_COMMU                                 
                  sprintf(print_msg,"[%d][%f][%f]", hitless_ptr()->event_buff.events_stack.size,last_flow,(last_flow * CUBIC_METERS_TO_GALLONS * 60.0));//print data via UART
                  nus_send_data_buff((uint8_t *)print_msg,strlen(print_msg));
//#endif                  
                  nrf_delay_ms(50);   
                }
                one_second = time_max3510;
        }        
}


#include "max3510x.h"
#include "transducer.h"

void max3510x_spi_xfer( max3510x_t p, void *pv_in, const void *pv_out, uint8_t count )
{    
        uint8_t             tx_data[256];   /**< Pointer to a buffer to transmit data from. */
        uint8_t             rx_data[256];
       
        uint8_t             len;        /**< Number of bytes to send from the \p tx_data buffer. */ 
        
        memcpy(tx_data , pv_out , count); 
	memset(rx_data , 0x0,count );             
	spi_xfer_done = false;       
	len = count;
            
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, len, rx_data, len));

        while (!spi_xfer_done)
        {
            __WFE();
        }
        
        if(rx_data != NULL)
        {
            memcpy(pv_in , rx_data , len); 
        }
        
        NRF_LOG_FLUSH();

        //bsp_board_led_invert(BSP_BOARD_LED_0);  //green  
}


max3510x_t g_max35103;

#define BOARD_SIGNATURE			0x8C01
void init_board()
{
        const uint16_t board_rev_addr = (MAX3510X_FLASH_BLOCK_SIZE_WORDS*MAX3510X_FLASH_BLOCK_COUNT)-2;
        uint16_t sig = 0;
	transducer_init();

        /* MAX35103 reset */
        bsp_board_led_on(BSP_RST_N_5302);  /* '0' */
        nrf_delay_ms(50); 
        bsp_board_led_off(BSP_RST_N_5302);/* '1' */	
	max3510x_wait_for_reset_complete( &g_max35103 );
              
	max3510x_init(&g_max35103, transducer_config() );	// configure the MAX3510x according per transducer requirements
     
	nrf_delay_ms(2);   			// give the MAX35103 some time to init

	max3510x_ldo(&g_max35103,max3510x_ldo_mode_on);   

        nrf_delay_ms(50);   			// give the MAX35103 some time to init  
	      
	max3510x_poll_interrupt_status( &g_max35103 );
        
	//board_max3510x_clear_interrupt();
               
	sig = max3510x_read_flash( &g_max35103, board_rev_addr );
        
        max3510x_ldo(&g_max35103,max3510x_ldo_mode_off); 
        
	if( sig == BOARD_SIGNATURE )
	{
          nrf_delay_ms(3);   			// give the MAX35103 some time to init     
          NRF_LOG_INFO("MAX35103 found\r\n");
         
		// get the trim value from the max35103 flash
		uint16_t trim = max3510x_read_flash( &g_max35103, board_rev_addr-2 );
#if 0                  
		MXC_PWRSEQ->reg6 = (MXC_PWRSEQ->reg6 & ~MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF) |
						   ((trim << MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS) & MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF);

		SystemCoreClockUpdate();
#endif                
	}
	
	
        // init ISR
	//GPIO_RegisterCallback(&s_board_max35103_int, p_max3510x_isr, NULL );        
}