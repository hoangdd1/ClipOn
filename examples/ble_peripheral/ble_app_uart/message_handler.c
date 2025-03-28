/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "general.h"
#include <string.h>
#include <stdlib.h> 
#include "datastruct_stack.h"
#include "message_handler.h"
#include "boards.h"
#include "hitless_data.h"
#include "gpio.h"




/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Initialize the ADC16
 */
   
extern int section_timed();


extern int16_t UART_tx(uint8_t * tx_Buffer , int16_t num_of_bytes);

extern int8_t virtual_send_buff(uint8_t * send_buffer, uint32_t size);
char str_out2[100];
#define PRINTUSB(str_out2)  virtual_send_buff((uint8_t *)str_out2, strlen(str_out2))


extern void radio_restart();
extern void radio_standby();
extern void samples_RPW_task_init();
extern void samples_RPW_task_standby();
//extern void Rpw_finalize_push_and_update();
extern int Rpw_finalize(int *SleepSectionsNum, int *high_detect_flag);
extern int Rpw_finalize_init();
extern int sizeOf_pwEventInfoStt_t;
extern int sizeOf_RpwSectionStatisticsStt_t;
extern int sizeOf_RpwSummeryPacket;
extern void UART_init(void);
extern void UART_deinit(void);
extern void init_ADC();
extern void saadc_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

int16_t detection_flag = 0;
uint8_t finalize_string[EVENT_SIZE]; 



/*******************************************************************************
 * Code
 ******************************************************************************/

int16_t counter_20min_flag;   
void events_stack_init(RESET_REASON reset_type)
{        

    if ( reset_type == POWER_ON_RESET )
    {
        memset((uint8_t *)&hitless_ptr()->event_buff,0x0,sizeof(EVENT_STACK_DATA));
        hitless_ptr()->event_buff.event_buffer_index = 0;
        hitless_ptr()->event_buff.events_stack.capacity = NUMBER_OF_EVENTS;
        hitless_ptr()->event_buff.events_stack.size = 0;
        hitless_ptr()->event_buff.events_stack.elements = (int *)hitless_ptr()->event_buff.events_stack_elements;        
    }
     else
     {
        if(hitless_ptr()->event_buff.events_stack.size > NUMBER_OF_EVENTS  ||
           hitless_ptr()->event_buff.event_buffer_index >  NUMBER_OF_EVENTS)
        {
        memset((uint8_t *)&hitless_ptr()->event_buff,0x0,sizeof(EVENT_STACK_DATA));
        hitless_ptr()->event_buff.event_buffer_index = 0;
        hitless_ptr()->event_buff.events_stack.capacity = NUMBER_OF_EVENTS;
        hitless_ptr()->event_buff.events_stack.size = 0;
        hitless_ptr()->event_buff.events_stack.elements = (int *)hitless_ptr()->event_buff.events_stack_elements;
        }
     }
}




extern unsigned int b64_encode(const unsigned char* in, unsigned int in_len, unsigned char* out);

void event_encapsulation(int packet_type,uint8_t * data_in, int data_in_len,uint8_t *push_message , int *push_message_len)
{
  char myb64[EVENT_SIZE+1];
  int len_base64 = 0;
  union_messagePacket_t temp;
  uint8_t checksum;
  memset( myb64,0x0,EVENT_SIZE+1); 
  temp.packet.preamble[0] = 0x55;
  temp.packet.preamble[1] = 0xaa;
  temp.packet.preamble[2] = 0x55;
  temp.packet.preamble[3] = 0xaa;
  
  temp.packet.device_UUID[0] = sys_info.idCodeShort.buff[7];
  temp.packet.device_UUID[1] = sys_info.idCodeShort.buff[6];
  temp.packet.device_UUID[2] = sys_info.idCodeShort.buff[5];
  temp.packet.device_UUID[3] = sys_info.idCodeShort.buff[4];
  temp.packet.device_UUID[4] = sys_info.idCodeShort.buff[3];
  temp.packet.device_UUID[5] = sys_info.idCodeShort.buff[2];
  temp.packet.device_UUID[6] = sys_info.idCodeShort.buff[1];
  temp.packet.device_UUID[7] = sys_info.idCodeShort.buff[0];
  
  temp.packet.UnixTimeStamp[0] = ((uint8_t *)&hitless_ptr()->rtc_var.m_time)[3];
  temp.packet.UnixTimeStamp[1] = ((uint8_t *)&hitless_ptr()->rtc_var.m_time)[2];
  temp.packet.UnixTimeStamp[2] = ((uint8_t *)&hitless_ptr()->rtc_var.m_time)[1];
  temp.packet.UnixTimeStamp[3] = ((uint8_t *)&hitless_ptr()->rtc_var.m_time)[0];
 
  temp.packet.packet_type = 1;
  temp.packet.data_length = 19;
  
  temp.packet.data_battery_level = (uint8_t)(hitless_ptr()->vbat);
  sys_info.counter_self_msg++;
  //temp.packet.data_transmit_counter=sys_info.counter_self_msg;
  temp.packet.data_transmit_counter = (uint8_t)hitless_ptr()->event_buff.event_buffer_index;//shir changed because sys_info var initialized every uploading data.
  
#ifdef MAX3510_SENSOR
  temp.packet.data_sensor_status_byte = 0x5a; //Max3510 sensor status. 
#elif TI_SENSOR
  temp.packet.data_sensor_status_byte = hitless_ptr()->ti_sensor_status; //Ti sensor status.
#endif 
 
  memcpy(temp.packet.data_measure,data_in,data_in_len);   
 
  checksum = 0x00;
  for ( int i = 0 ; i < MESSAGE_PACKET_SIZE -1 ; i ++ )
  { 
    checksum = checksum +((uint8_t*)&temp.packet)[i];  
  }
   temp.packet.packet_checksum = checksum;
   //encryptDecrypt(&temp);
   len_base64 = b64_encode((const unsigned char *)&temp.packet, MESSAGE_PACKET_SIZE , (uint8_t *)myb64);

  *push_message_len = len_base64;
  
   memcpy(push_message,myb64,len_base64);
}


void push_event(uint8_t *str_event , int16_t str_event_len)
{            
      if ( str_event_len <  EVENT_SIZE && hitless_ptr()->event_buff.event_buffer_index < NUMBER_OF_EVENTS ) 
      {
        if (push_fifo((Stack_utill *)&hitless_ptr()->event_buff.events_stack,hitless_ptr()->event_buff.event_buffer_index))
        {

            for ( int i = 0 ; i < EVENT_SIZE ; i++)
              hitless_ptr()->event_buff.buffer_of_events[hitless_ptr()->event_buff.event_buffer_index].buffer.uint8_data[i] = 0;

             
              memcpy((char *)hitless_ptr()->event_buff.buffer_of_events[(int)hitless_ptr()->event_buff.event_buffer_index].buffer.uint8_data,str_event,str_event_len);
      
              if(hitless_ptr()->event_buff.event_buffer_index >= NUMBER_OF_EVENTS)//not relevant?
                hitless_ptr()->event_buff.event_buffer_index = 0;
              hitless_ptr()->event_buff.event_buffer_index++;

         }
      }      
}  
 
int event_buffer_upload() //True : if we want to upload data,  False: O.W
{
   bool ret_val = false;
   
   if ( hitless_ptr()->detect_pizuz_flags[1] > 0 )//the program is reading data from ti sense. update_water_sense();
     return ret_val;

   if ( hitless_ptr()->send_critical_msg ) //critical_msg or Battery saving   //  || (hitless_ptr()->vbat && (hitless_ptr()->vbat < CRITICAL_BATTERY_VOLTAGE*50))
    return ret_val;
   
  int buffer_top_index;  
  buffer_top_index = top((Stack_utill *)&hitless_ptr()->event_buff.events_stack); 
  
  if (  hitless_ptr()->event_buff.events_stack.size > 0 &&
        (  
           (hitless_ptr()->upload_data[0] == 1 && buffer_top_index >= 0) //duplicate condition?
           ||       
           (hitless_ptr()->event_buff.event_buffer_index >= NUMBER_OF_EVENTS-100)  
        )   
     )
  {    
      hitless_ptr()->modem_event = MODEM_CLOUD_EVENT;
      ret_val = true;      
  }
  if( hitless_ptr()->upload_data[0]>0)  hitless_ptr()->upload_data[0]--; 
  else hitless_ptr()->upload_data[0]= hitless_ptr()->upload_data[1];
  
  return ret_val;
}

int event_buffer_not_empty()  //True : There is data in the buffer,  False: O.W
{
  bool ret_val = false;
  int buffer_top_index;  
 
  buffer_top_index = top((Stack_utill *)&hitless_ptr()->event_buff.events_stack);  
  if ( buffer_top_index >= 0 && hitless_ptr()->event_buff.events_stack.size > 0) //duplicate condition?
  {    
      hitless_ptr()->modem_event = MODEM_CLOUD_EVENT;
      ret_val = true;      
  }
  return ret_val;
}


/*
//extern void nrfx_saadc_irq_handler(void);
int event_check_message()
{ 
  if (hitless_ptr()->check_message_coutdown > -10 && 
      hitless_ptr()->check_message_coutdown <= hitless_ptr()->message_coutdown_interval)
  {
    hitless_ptr()->check_message_coutdown--;

    if ( hitless_ptr()->check_message_coutdown == 2  )
    {
      init_ADC();      
    }
    return 0;
  }
 
  else //hitless_ptr()->check_message_coutdown == -10
  {   
    //nrfx_saadc_irq_handler();
    if (hitless_ptr()->message_coutdown_interval < 60 || hitless_ptr()->message_coutdown_interval > 40000)
           hitless_ptr()->message_coutdown_interval = 300;      
    hitless_ptr()->check_message_coutdown = hitless_ptr()->message_coutdown_interval;
    hitless_ptr()->modem_event = MODEM_SMS_EVENT;
    return 1;
  }
}
*/

int event_check_message() //True : if we want to send sms msg,  False: O.W
{ 
   bool ret_val = false;
   
   if ( hitless_ptr()->detect_pizuz_flags[1] > 0 )//the program is reading data from ti sense. update_water_sense();
     return ret_val;
   
//   if ( ( hitless_ptr()->vbat != 0 ) && (hitless_ptr()->vbat < CRITICAL_BATTERY_VOLTAGE*50) && (!hitless_ptr()->send_critical_msg) && (hitless_ptr()->messages_counter != 0 ) ){ //Battery saving  
//       return ret_val;
//   }
   
   /* Deal with skip modem bug - shir */
   switch( hitless_ptr()->skip_modem_flag ){//the number of retry modem_sms state
      case 1:
      case 2:
      case 3:
          hitless_ptr()->check_message_coutdown = 0;//retry
          break;
      case 4:
          hitless_ptr()->skip_modem_flag = 0;//flag off
          break;
      default:
          break;
  }
  
  if ( hitless_ptr()->check_message_coutdown > 0 && hitless_ptr()->check_message_coutdown <= hitless_ptr()->message_coutdown_interval )
  {
    if( hitless_ptr()->ADC >= 0 ){  //check if the adc() function is enabled by the external command AD.
        hitless_ptr()->check_message_coutdown--;
    }       
    else if (hitless_ptr()->ADC == -1){
     
            if( hitless_ptr()->check_message_coutdown > 2 )
            {
              hitless_ptr()->check_message_coutdown--;
            }
            else if( hitless_ptr()->check_message_coutdown == 2 )
            {
              init_ADC(); 
              hitless_ptr()->check_message_coutdown--;
            }
            else if( hitless_ptr()->check_message_coutdown == 1 )
            {
              ;//wating for ADC result to finish 
            }
    }
    
    return ret_val;
  }
  else
  {   
    if (hitless_ptr()->message_coutdown_interval < 60 || hitless_ptr()->message_coutdown_interval > 40000){
        hitless_ptr()->message_coutdown_interval = 300;
    }
    
    hitless_ptr()->check_message_coutdown = hitless_ptr()->message_coutdown_interval;
  
    hitless_ptr()->skip_modem_flag++;
    hitless_ptr()->modem_event = MODEM_SMS_EVENT;
    ret_val= true ;
    
    return ret_val;
  }
}

int get_buffer_top_index()
{
  int ret_val = top((Stack_utill *)&hitless_ptr()->event_buff.events_stack);  
  return ret_val;
}

  
void pop_event()
{
  pop((Stack_utill *)&hitless_ptr()->event_buff.events_stack);
}

void pop_events(int max_msg)
{
  while(max_msg>0){
    pop((Stack_utill *)&hitless_ptr()->event_buff.events_stack);
    max_msg--;
  }
}

void top_event(uint8_t * pop_data , int * length_of_pop_data)
{
  int msg_index;
  
  *length_of_pop_data = 0;
  
  msg_index = top((Stack_utill *)&hitless_ptr()->event_buff.events_stack);  
  if ( msg_index >= 0 && hitless_ptr()->event_buff.events_stack.size > 0)
  {	  
      memcpy(pop_data,(char *)hitless_ptr()->event_buff.buffer_of_events[msg_index].buffer.uint8_data,EVENT_SIZE);  
      *length_of_pop_data = EVENT_SIZE;
  }
}

void top_events(uint8_t * pop_data , int * length_of_pop_data , int max_msg)
{
  int msg_index;
  //int msg_chain_index = 0;
  *length_of_pop_data = 0;
  
  msg_index = top((Stack_utill *)&hitless_ptr()->event_buff.events_stack);  
  if ( msg_index >= 0 && hitless_ptr()->event_buff.events_stack.size > max_msg)
  {
    while(max_msg > 0){      
      memcpy((void*)pop_data[*length_of_pop_data],(void *)hitless_ptr()->event_buff.buffer_of_events[msg_index%1400].buffer.uint8_data,EVENT_SIZE);  
      msg_index++;
      max_msg--;
      *length_of_pop_data += EVENT_SIZE;
    }
  }
  else if ( (msg_index >= 0) && (hitless_ptr()->event_buff.events_stack.size > 0))
  {	  
      memcpy(pop_data,(char *)hitless_ptr()->event_buff.buffer_of_events[msg_index].buffer.uint8_data,EVENT_SIZE);  
      *length_of_pop_data = EVENT_SIZE;
  }  
}


int event_stack_fulled(){
  int num_of_events = 0;
  int ret_val = 0;
  
  num_of_events =  hitless_ptr()->event_buff.events_stack.size ;
  if(num_of_events > NUMBER_OF_EVENTS-1)
    ret_val = 1;  
  return ret_val;
}
