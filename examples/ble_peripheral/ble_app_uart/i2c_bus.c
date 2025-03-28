/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>

#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
//#include "24f02.h"
#include "i2c_bus.h"
#include "hitless_data.h"
#include "message_handler.h"
#include "ti_version.h"
#include "FR6047_USSSWLib_AQSMRT_V2.0.0.h"

#if 1 
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     1



/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample[16];


struct EEPROM_STRUCTURE
{  
  char manuf_date[8]; // "DD/MM/YY"
  char padding[8];
  char device_name_string[11]; // 123.456.789
  char padding2[5];
  char device_mode[1];  // 'R' / 'H' / 'S'
  char serial_number[16];  // 'R' / 'H' / 'S'
  char activate_key_H[16];  
  char activate_key_L[16];
  char _4G_apn_h[16];  
  char _4G_apn_l[16];
  char _ip_address[16];  
  char _udp_port[16];
  char tree_size[1];
  char _4G_cid[3];
  char _4G_auth[4];
  char _4G_pin_number[8];
  char device_rx_color[1];
  char device_tx_color[1];
  char gateway_modem_type[1];
  char _4G_usrename[16];
  char _4G_password[16];
} ee_buf;


EEPROM_DB ee_db[NUM_OF_EE_VARS] = {
  {MANUF_DATE   ,       0       ,       10       ,ee_buf.manuf_date             ,       "01/05/2016"      },
  {DEV_STR_NAME ,       16      ,       11       ,ee_buf.device_name_string     ,       "255.255.255"     },
  {SERIAL_NUMBER,       48      ,       16       ,ee_buf.serial_number          ,       "1234567890ABCDE\0"},
  {ACTIVATE_KEY_H ,     64      ,       16       ,ee_buf.activate_key_H         ,       "1234567890ABCDE\0"},
  {ACTIVATE_KEY_L ,     80      ,       16       ,ee_buf.activate_key_L         ,       "1234567890ABCDE\0"},
  {_4G_APN_H ,          96      ,       16       ,ee_buf._4G_apn_h              ,       "uinternet"},
  {_4G_APN_L ,          112     ,       16       ,ee_buf._4G_apn_l              ,       ""},
  {_IP_ADDRESS ,        128     ,       16       ,ee_buf._ip_address            ,       "34.68.215.44"},
  {_UDP_PORT ,          144     ,       4        ,ee_buf._udp_port              ,       "20001"},
  {_4G_CID,             168     ,       3        ,ee_buf._4G_cid                ,       "1\0"},
  {_4G_AUTH,            176     ,       4        ,ee_buf._4G_auth               ,       "PAP\0"},
  {_4G_PINMUN,          184     ,       8        ,ee_buf._4G_pin_number         ,       "9876543\0"},
  {_4G_USR,             200     ,       16       ,ee_buf._4G_usrename           ,       ""},
  {_4G_PASS,            216     ,       16       ,ee_buf._4G_password           ,       ""},
};
  



/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */

int AT24CXX_Write(uint8_t WriteAddr,uint8_t *pBuffer,int NumToWrite)
{
  uint8_t reg[9]; 
  int i;
  reg[0] = WriteAddr;
  
  i = 0 ;
  while ( (NumToWrite/8)> 0 )
  {
    m_xfer_done = false;
    memcpy(&reg[1],&pBuffer[i],8);
    m_xfer_done = nrf_drv_twi_tx(&m_twi, EE_24F02_ADDR, reg, 9, false); 
    while (m_xfer_done == false);
    nrf_delay_ms(10);
    i+=8;
    reg[0]+=8;
    NumToWrite -= 8;
  }
  
  if ( NumToWrite < 8 && NumToWrite > 0)
  {
    m_xfer_done = false;
    memcpy(&reg[1],&pBuffer[i],NumToWrite);
    m_xfer_done = nrf_drv_twi_tx(&m_twi, EE_24F02_ADDR, reg, NumToWrite+1, false); 
    while (m_xfer_done == false);
    nrf_delay_ms(10);
  }
  
  return (m_xfer_done == true)? 1:0;
}

int AT24CXX_Read(uint8_t ReadAddr,uint8_t *pBuffer,int NumToRead)
{
    nrf_drv_twi_tx(&m_twi, EE_24F02_ADDR, &ReadAddr, 1, true); 
    nrf_delay_us(500);
    m_xfer_done = false;
    nrf_drv_twi_rx(&m_twi, EE_24F02_ADDR, pBuffer, NumToRead);
    while (m_xfer_done == false);
     nrf_delay_ms(10);
     return (m_xfer_done == true)? 1:0;
}

uint8_t AT24CXX_ReadOneByte(uint8_t ReadAddr)
{
    uint8_t ret_val = 0; 
    /* Read 1 byte from the current address */
    
    nrf_drv_twi_tx(&m_twi, EE_24F02_ADDR, &ReadAddr, 1, true); 
    nrf_delay_ms(50);
    m_xfer_done = false;
    //nrf_drv_twi_rx(&m_twi, EE_24F02_ADDR, &ret_val, 1);
    nrf_delay_ms(50);  
    return ret_val;
}

void AT24CXX_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
    uint8_t reg[2] = {0,0};
    reg[0] = WriteAddr;
    reg[1] = DataToWrite;
    
   
    nrf_drv_twi_tx(&m_twi, EE_24F02_ADDR, reg, sizeof(reg), false); 
    nrf_delay_ms(10);   
}


 
int AT24XX_modem_params_exist()
{
    char temp[10];
    int ret_val = 0;
    int udp_port;
    InitI2CBUS();
    AT24CXX_Read(ee_db[_UDP_PORT].ee_addr,(uint8_t *)temp,ee_db[_UDP_PORT].ee_len);
    DenitI2CBUS();
    
    udp_port = atoi(temp);
    
    if (udp_port >= 8000 && udp_port != 8888 && udp_port <= 9999)
      ret_val = udp_port;
    return ret_val;
}

int AT24CXX_Check1(void)
{
    int ret_val = 0;
    uint8_t reg[] = {0x00, 0x01 , 0x02 , 0x03, 0x04, 0x05 , 0x06 , 0x07 , 0x08, 0x09 , 0x0a , 0x0b , 0x0c, 0x0d , 0x0e , 0x0f};
    //uint8_t reg[] = {0x00, 0x00 , 0x00 , 0x00, 0x00, 0x00 , 0x00 , 0x00 , 0x00, 0x00 , 0x00 , 0x00 , 0x00, 0x00 , 0x00 , 0x00};
    

    reg[0] = 0; // address to write
    /* Read 1 byte from the current address */
    AT24CXX_Write(0,&reg[0],12);
    memset(m_sample,0,sizeof(m_sample));
    AT24CXX_Read(0,m_sample,16);

    /***************************************/
    if (m_sample[1] == 0x01)
      ret_val = 1;
    
    return ret_val;
}


/*********************************************************************************************************/
/**********                 sensor  **********************************************************************/

#define COMMAND_HANDLER_GAP_PLS_ADC_START_ID                0x80
#define COMMAND_HANDLER_NUM_PLS_ID                          0x81


#define COMMAND_HANDLER_TIME_OUT                        900000
#define COMMAND_HANDLER_UPS_DNS_CAP_PADDING_SIZE            4
#define COMMAND_HANDLER_UPS_DNS_CAP_MAX_CAP_SIZE            27

#define COMMAND_HANDLER_CMD_BYTE_POS                        1
#define COMMAND_HANDLER_R_W_BYTE_POS                        2
#define COMMAND_HANDLER_WRITE_CMD                           true
#define COMMAND_HANDLER_READ_CMD                            false





//! \def Packet synchronization byte in serial stream
#define PACKET_SYNC (0x55)

//! \def Packet blank byte in serial stream
#define PACKET_BLANK (0xAA)

//! \def Packet payload start offset
#define PACKET_PAYLOAD_START (0x03)

//! \def Max packet payload size 62 byte ( 60 byte + 2 checksum bytes)
#define PACKET_PAYLOAD_MAX_SZ (0x3E)

//! \def Max packet payload size
#define PACKET_CHECKSUM_SZ (0x02)

//! \typedef Packet object definition
typedef struct
{
	uint8_t payload[PACKET_PAYLOAD_MAX_SZ];
	uint8_t length;
} Packet_t;



volatile uint32_t gRXUpdateFlag;

uint8_t Test = false;

void Comm_writePacket(const Packet_t *packet)
{
      uint16_t i = 1000;
      ret_code_t err_code;
      if(m_xfer_done == false && Test == true)nrf_delay_ms(10);
      m_xfer_done = false;
      Test = true;
      
      /* Simply call the write buffer function, passing the packet parameters */
      err_code = nrf_drv_twi_tx(&m_twi, SENSOR_ADDR,packet->payload, packet->length, false);
      APP_ERROR_CHECK(err_code);
      
      while (m_xfer_done == false && i--);	
}

void Comm_listenForCmd(Packet_t *packet)
{
    uint16_t i = 1000;   
    if(m_xfer_done == false && Test == true)nrf_delay_ms(10);
    m_xfer_done = false;
    Test = true;
    
     /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, SENSOR_ADDR, packet->payload, packet->length);
    APP_ERROR_CHECK(err_code);
    
    while (m_xfer_done == false && i--);
}

void CommandHandler_transmitPacket(Packet_t *packet)
{
    Comm_writePacket(packet);   
}

#define COMMAND_HANDLER_GAP_PLS_ADC_START_RX_FLAG               0x00000001
#define COMMAND_HANDLER_NUM_PLS_RX_FLAG                         0x00000002

uint32_t resolveRXFlag(uint8_t command)
{
    uint32_t flag;
    switch (command) {
    case COMMAND_HANDLER_GAP_PLS_ADC_START_ID:
        flag = COMMAND_HANDLER_GAP_PLS_ADC_START_RX_FLAG;
        break;
    case COMMAND_HANDLER_NUM_PLS_ID:
        flag = COMMAND_HANDLER_NUM_PLS_RX_FLAG;
        break;
    }
    return flag;
}


void CommandHandler_rxUpdate(uint8_t command, int32_t* ptimeOut , Packet_t * packet)
{
    uint32_t flag = resolveRXFlag(command);


    Comm_listenForCmd(packet);
}

void CommandHandler_prepareForParamUpdate(uint8_t command)
{
    uint32_t flag = resolveRXFlag(command);

    gRXUpdateFlag |= flag;
}

 /* Global Variables for read water sensor ti */

extern void nrf_delay_ms_wdt(uint32_t x_msec);
extern uint32_t nus_send_data_buff(uint8_t *send_buff, int send_buff_length);
extern void push_event(uint8_t *str_event , int16_t str_event_len);
extern void wrap_NVIC_SystemReset();

#define COMMAND_HANDLER_USS_GUI_ID_BYTE 0x04
#define CUBIC_METERS_TO_GALLONS	264.172
#define LITER_PER_HOUR_CONVERSION 1 
#define UPDATE_BUFF_SIZE 5  

double last_update_data = 0.0;
int update_buffer_index = 0;
float update_buffer[UPDATE_BUFF_SIZE]; 
char print_msg_ti[128];
bool unusage_alert_flag_ti = false, pizuz_alert_flag_ti = false, leakage_alert_flag_ti = false, leakage_alert_flag_off_ti = false, unusage_alert_flag_off_ti = false;

/* alerts_detection colects every UPDATE_BUFF_SIZE flow data and check if their avg is over the limit */
void alerts_detection_ti() 
{
  
  float average_detection = 0,sum = 0;
  update_buffer[ update_buffer_index ] = last_update_data;
  update_buffer_index++;
 
  if ( update_buffer_index >= UPDATE_BUFF_SIZE ){                //buffer is full
    update_buffer_index = 0;                                    
    for( int i = 0; i < UPDATE_BUFF_SIZE; i++ )                  //average calculation
       sum += update_buffer[i]; 
                                                 
    average_detection = sum / UPDATE_BUFF_SIZE;
    last_update_data = average_detection;                        //save the received average(from UPDATE_BUFF_SIZE data) for uploading.
    
    if ( hitless_ptr()->bubbles_error_counter < UPDATE_BUFF_SIZE ){ //when the bubbles error is not accured.
       /* Pitzuz */
      if ( average_detection >= hitless_ptr()->detection_limit ) //checking the average compared to the pizuz limit. 
        hitless_ptr()->limit_count_result[1]++; 
      else
        hitless_ptr()->limit_count_result[1] = 0;
      
        if ( hitless_ptr()->limit_count_result[1] >= hitless_ptr()->limit_count_result[0] )//if go over the limit more then "limit_count_result[0]" times in a row its pizuz.
        {
             hitless_ptr()->limit_count_result[1] = 0;
             pizuz_alert_flag_ti = true;
             return;
        }
        /* Unusage */
        if ( ( average_detection < hitless_ptr()->unusage_limit ) && ( average_detection > (-1)*hitless_ptr()->unusage_limit ) )//checking the average compared to the unusage_limit.
          hitless_ptr()->unusage_limit_counter[1]++; 
        else {
               hitless_ptr()->unusage_limit_counter[1] = 0;
               if ( hitless_ptr()->unusage_alert_status ) {//there was alert of unusage
                  hitless_ptr()->unusage_alert_status = 0; //alert off
                  unusage_alert_flag_off_ti = true;
                  return;
               }   
        }
      
        if ( hitless_ptr()->unusage_limit_counter[1] >= hitless_ptr()->unusage_limit_counter[0] )//if go over the limit more then "unusage_limit_counter[0]" times in a row its unusage alert.
        {
             hitless_ptr()->unusage_limit_counter[1] = 0;
             hitless_ptr()->unusage_alert_status = 1; //alert on 
             unusage_alert_flag_ti = true;
             return;
        }
       /* Leakage */
       if ( average_detection >= hitless_ptr()->drip_test_limit ){//checking the average compared to the leak limit.
            hitless_ptr()->leak_counter[1]++;//n
            /* Moving average formula : avg = ((avg*(n-1))+flow)/n */
            hitless_ptr()->leak_avg = ((( hitless_ptr()->leak_avg * (hitless_ptr()->leak_counter[1]-1)) + last_update_data) / hitless_ptr()->leak_counter[1]);
       }
       else {
            hitless_ptr()->leak_counter[1] = 0;
            hitless_ptr()->leak_avg = 0;        //reset the leak avg calculation
            if ( hitless_ptr()->leak_status ) { //there was alert of leak
              hitless_ptr()->leak_status = 0;   //no leakage
              leakage_alert_flag_off_ti = true;
              return;
            }
       }
      
      if ( hitless_ptr()->leak_counter[1] >= hitless_ptr()->leak_counter[0] )//if go over the limit more then "leak_counter[0]" times in a row its leakage.
      {
           hitless_ptr()->leak_counter[1] = 0;
           leakage_alert_flag_ti = true;
           hitless_ptr()->leak_status = (int)hitless_ptr()->leak_avg; //save the leak status on device.
           return;
      }
    }
  }
}

/* Calibrate for connection with TI component */
void Ti_calibrate(void){
 
    /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    txPacket.length = 1;//BYTE
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* TI component initialization */
    CommandHandler_prepareForParamUpdate(updateParams[0]); 
    
    /* USS BEGIN INITIALIZATION */
    txPacket.payload[0] = 0x7E;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER sends SLAVE 0x7E. The USS initialization procedure begins.

    /* FINISH INITIALIZATION */
}

/* Read from TI one float for Command SR  */
void TiReadData_byI2c_command(void)
{
    /* Variables */
    memset(print_msg_ti,0x0,128); 
    //union_measure_data_t data_measure;
    //int event_temp_size;
    float result;
    uint8_t *result_ptr = (uint8_t*)&result;
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    Packet_t  rxPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    
    /* USS DATA EXCHANGE */
    rxPacket.length = sizeof(float)+2;
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket);   //Read 4 + 2 bytes.
    nrf_delay_ms(10);
    
    int i;
    /* Save all the bytes to float result */
    for( i = 0; i < sizeof(float); i++ ) { 
       result_ptr[i] = rxPacket.payload[i];
    }
    
    /* Save the 5-th byte of the sensor status */
    hitless_ptr()->ti_sensor_status = rxPacket.payload[i]; 
    
    /* Save the Read Data */
//    data_measure.data.A = hitless_ptr()->ti_sensor_status != 0xff ? result : 0.0 ;  //! = 255 status sensor ERROR
//    data_measure.data.B = hitless_ptr()->ti_sensor_status != 0xff ? (result * CUBIC_METERS_TO_GALLONS * 60.0) : 0.0 ;
//    data_measure.data.C = hitless_ptr()->v_vdd;
    
    /* Save on Buffer */
//    event_encapsulation(1,data_measure.buff,sizeof(data_measure.buff),(uint8_t *)print_msg_ti,&event_temp_size); 
//    push_event((uint8_t *)print_msg_ti,EVENT_SIZE-1);

    /* Print data via UART */
    sprintf(print_msg_ti,"[%d][%f][%d][%d]", hitless_ptr()->event_buff.events_stack.size, result , hitless_ptr()->ti_sensor_status , rxPacket.payload[++i]);
    nus_send_data_buff((uint8_t *)print_msg_ti,strlen(print_msg_ti));
    
    /* FINISH READ DATA */
}
/* Read from TI one float */
float TiReadData_byI2c(void)
{
    /* Variables */
    float result;
    uint8_t *result_ptr = (uint8_t*)&result;
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    Packet_t  rxPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    
    /* USS DATA EXCHANGE */
    rxPacket.length = sizeof(float)+2; // 4 bytes of the data and 2 byte of sensor status .
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket);    //Read 4 + 2 bytes.
    nrf_delay_ms(10);
    
    int i;
    /* Save all the bytes to float result */
    for( i = 0; i < sizeof(float); i++ ) {
       result_ptr[i] = rxPacket.payload[i];
    }
    /* Save the 5-th byte of the sensor status */
    hitless_ptr()->ti_sensor_status = rxPacket.payload[i]; 
    /* There is 6-th byte of another sensor status */
    
    return (hitless_ptr()->ti_sensor_status != 0xff) ? result : 0.0;
    /* FINISH READ DATA*/
}
/* Func - to start read from TI - USS ENABLE DATA EXCHANGE */
void StartTiRead(void){
   
    /* INIT I2C */
    InitI2CBUS( );
    
   /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* USS ENABLE DATA EXCHANGE */
   
    txPacket.payload[0] = 0x9E;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER tells the SLAVE to begin reading measurements
    
    /* FINISH ENABLE DATA */
}
/* Func - to stop read from TI - USS DISABLE DATA EXCHANGE */
void StopTiRead(void){
  
    /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* USS DISABLE DATA EXCHANGE */
   
    txPacket.payload[0] = 0x8E;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER tells the SLAVE to stop reading measurements
    
     DenitI2CBUS( );
    /* FINISH DISABLE DATA*/
}

#if 0
/* manager of updating water flow status */ 
void update_water_sense_ti(void){
  
    /* Variables */
    memset(print_msg_ti,0x0,128); 
    union_measure_data_t data_measure;
    int event_temp_size;
    
    if( hitless_ptr()->initTIDone == 0 ){
      
        hitless_ptr()->initTIDone = 1;
       /* ENABLE TI */
        StartTiRead();
        return;
    }

     /* TI READ DATA */
     last_update_data = hitless_ptr()->change_flow_sign * TiReadData_byI2c();
     nrf_delay_ms(125);
     
      /* CHECK DATA */
     if ( last_update_data < -1000 || last_update_data > 2500 ){//This range of values indicates a critical state of the sensor reading.
       last_update_data = hitless_ptr()->last_flow;
       hitless_ptr()->bubbles_error_counter++; //count the number of ti sensor error until the valid sensor reading.
     }
     else if ( hitless_ptr()->ti_sensor_status == 0xff ){  //hitless_ptr()->ti_sensor_status == 0xff == 255.
        hitless_ptr()->bubbles_error_counter++; //count the number of ti sensor error until the valid sensor reading.
     }
     else {
        hitless_ptr()->last_flow = last_update_data; //save the last valide flow 
        hitless_ptr()->bubbles_error_counter = 0;
     }
     
     //Detection of PIZUZ, every 30 second check 2 data average and save the average on buff.
     if( hitless_ptr()->detect_pizuz_flags[1] > 0)
     {
        hitless_ptr()->detect_pizuz_flags[1]--;
        alerts_detection_ti(); //Finally "last_flow" will contain the received average.
        
        if ( hitless_ptr()->detect_pizuz_flags[1] == 0 ){
           
            //Update the received average as data.
            data_measure.data.A = last_update_data;  
            data_measure.data.B = (last_update_data * CUBIC_METERS_TO_GALLONS * 60.0);
            data_measure.data.C = hitless_ptr()->v_vdd;//temporary parameter for save the USB status- shir 
//            if( ( hitless_ptr()->vbat || ( hitless_ptr()->event_buff.event_buffer_index > 2 )) ){//Check if the adc accrued for battery voltage
               event_encapsulation(1,data_measure.buff,sizeof(data_measure.buff),(uint8_t *)print_msg_ti,&event_temp_size); 
               push_event((uint8_t *)print_msg_ti,EVENT_SIZE-1);
//            }
           //checking for alerts
           if ( unusage_alert_flag_ti || pizuz_alert_flag_ti || leakage_alert_flag_ti || leakage_alert_flag_off_ti )   //The system detects a water critical use (unusage or an explosion or leakage).
          {
             if (unusage_alert_flag_ti)  
                sprintf( hitless_ptr()->critic_msg,"[5]Water critical unusage\n[%.1f]l/h [%d] times" , hitless_ptr()->unusage_limit, hitless_ptr()->unusage_limit_counter[0]); 
             else if (pizuz_alert_flag_ti) 
                sprintf( hitless_ptr()->critic_msg,"[2]Crossed:[%.1f]l/h->[%.0f] times!\nsend SD command" , hitless_ptr()->detection_limit, hitless_ptr()->limit_count_result[0]);
             else if (leakage_alert_flag_ti) 
               sprintf(hitless_ptr()->critic_msg,"[6]Warning water leak: crossed:[%d]l/h [%d]times", hitless_ptr()->leak_status, hitless_ptr()->leak_counter[0]);
             else if (leakage_alert_flag_off_ti) 
               sprintf(hitless_ptr()->critic_msg,"[7]Water leak stopped: leak status:[%d]", hitless_ptr()->leak_status);
             else if (unusage_alert_flag_off_ti)
               sprintf(hitless_ptr()->critic_msg,"[8]Unusage alert stopped: alert status:[%d]", hitless_ptr()->unusage_alert_status);
             
             hitless_ptr()->send_critical_msg = 1;         //There is a Critical message 
             hitless_ptr()->upload_data[0] = (leakage_alert_flag_off_ti || unusage_alert_flag_off_ti) ? 600 : 1;  //Upload the data from the detection period, after the message will be sent.
             hitless_ptr()->check_message_coutdown = 10;   //KP_0
          }
          /* DISABLE */
          StopTiRead();
          hitless_ptr()->initTIDone = 0;
          hitless_ptr()->detect_pizuz_flags[0] = 30;   
          hitless_ptr()->device_state = SLEEP_STATE;    //MO_0
          wrap_NVIC_SystemReset();   
        }
     }       
    /* FINISH UPDATE WATER FLOW STATUS*/
}
#else //

#define MAX_TIME_AFTER_RESET        (1000)
#define MAX_FAIL_COUNTER            (5)
#define MAX_TIME_DELAY              (30)

static ret_code_t   Sensor_GetSamples   ( void  );
extern void         TI_reset            ( void  );

extern volatile HITLESS_STRUCTURE hitless_struct;

static ret_code_t Comm_IsBussy                  ( void                                                      );
static ret_code_t Comm_writePacketResult        ( const Packet_t *packet                                    );
static ret_code_t StartTiReadResult             ( void                                                      );
static ret_code_t Comm_listenForCmdResult       ( Packet_t *packet                                          );
static ret_code_t CommandHandler_rxUpdateResult ( uint8_t command, int32_t* ptimeOut , Packet_t * packet    );
static ret_code_t TiReadData_byI2cResult        ( float *pData                                              );
static ret_code_t StopTiReadResult              ( void                                                      );

/**********************************************************************
*  Function Name: Comm_writePacketResult                              *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t Comm_writePacketResult( const Packet_t *packet ){
  if( m_xfer_done == false )return( NRF_ERROR_BUSY ) ;
  m_xfer_done = false                                ;
      
  if( nrf_drv_twi_tx( &m_twi, SENSOR_ADDR,packet->payload, packet->length, false ) != NRF_SUCCESS )return( NRF_ERROR_INTERNAL );
  
  uint16_t i = 1000                                     ;
  while( m_xfer_done == false && i-- )nrf_delay_ms( 1 ) ;	
  
  return( NRF_SUCCESS );
}
/**********************************************************************
*  Function Name: Comm_IsBussy                                        *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t Comm_IsBussy( void ){
  if( m_xfer_done == false )return( true ) ;
 
  return( false );
}
/**********************************************************************
*  Function Name: Comm_listenForCmdResult                             *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t Comm_listenForCmdResult( Packet_t *packet ){
  if(m_xfer_done == false )return( NRF_ERROR_BUSY ) ;
  m_xfer_done = false                               ;

  if( nrf_drv_twi_rx( &m_twi, SENSOR_ADDR, packet->payload, packet->length ) != NRF_SUCCESS )return( NRF_ERROR_INTERNAL );
  
  uint16_t i = 1000                                     ;
  while( m_xfer_done == false && i-- )nrf_delay_ms( 1 ) ; 	
  
  return( NRF_SUCCESS );
}
/**********************************************************************
*  Function Name: Sensor_Init                                         *
*  Description  :                                                     *
**********************************************************************/
void Sensor_Init( void ){
  m_xfer_done = true;
  
  RESET_REASON rr = check_reset_reason( );
  if( rr != POWER_ON_RESET )return;
  
  memset( &hitless_ptr( )->Sensor, 0, sizeof(Sensor_t) ) ;
  hitless_ptr( )->Sensor.State = STATE_SENSOR_RESET      ;
}
/**********************************************************************
*  Function Name: CommandHandler_rxUpdateResult                       *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t CommandHandler_rxUpdateResult( uint8_t command, int32_t* ptimeOut , Packet_t * packet ){
  return( Comm_listenForCmdResult( packet ) );
}
/**********************************************************************
*  Function Name: StopTiReadResult                                    *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t StopTiReadResult( void ){
  /* Variables */
  Packet_t  txPacket = {0};
  const uint8_t updateParams[] = {
    COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
    COMMAND_HANDLER_NUM_PLS_ID,        
  };

  txPacket.length = 1;//BYTE
  txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
  txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
  txPacket.payload[1] = updateParams[0];
    
  /* USS DISABLE DATA EXCHANGE */
   
  txPacket.payload[0] = 0x8E;
    
  return( Comm_writePacketResult( &txPacket ) );   //Write - The MASTER tells the SLAVE to stop reading measurements
}
/**********************************************************************
*  Function Name: StartTiReadResult                                   *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t StartTiReadResult(void){
   
   
   /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* USS ENABLE DATA EXCHANGE */
   
    txPacket.payload[0] = 0x9E;
        
    return( Comm_writePacketResult( &txPacket ) ); 
    /* FINISH ENABLE DATA */
}
/**********************************************************************
*  Function Name: TiReadData_byI2cResult                              *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t TiReadData_byI2cResult( float *pData ){
    /* Variables */
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    Packet_t  rxPacket = {0};
    
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    
    /* USS DATA EXCHANGE */
    rxPacket.length = sizeof(float)+2; // 4 bytes of the data and 2 byte of sensor status .
   
    
    if( CommandHandler_rxUpdateResult( updateParams[0], &timeout , &rxPacket ) != NRF_SUCCESS )return( NRF_ERROR_BUSY );
    
    memcpy( pData, rxPacket.payload, sizeof(float) );
    
   
    /* Save the 5-th byte of the sensor status */
    hitless_ptr()->ti_sensor_status = rxPacket.payload[sizeof(float)]; 
    /* There is 6-th byte of another sensor status */
    
    return( (hitless_ptr()->ti_sensor_status != 0xff) ? NRF_SUCCESS : NRF_ERROR_BUSY );
}
/**********************************************************************
*  Function Name: Sensor_GetSamples                                   *
*  Description  :                                                     *
**********************************************************************/
static ret_code_t Sensor_GetSamples( void ){
  union_measure_data_t  data_measure    ;
  int                   event_temp_size ;

  memset( print_msg_ti, 0, 128 ); 
 
   /* TI READ DATA */
  float Sample = 0;
  
  hitless_ptr( )->Sensor.State  = STATE_SENSOR_GET_SAMPLES_DELAY;
  
  if( TiReadData_byI2cResult( &Sample ) != NRF_SUCCESS )return( NRF_ERROR_INTERNAL );
    
  hitless_ptr( )->Sensor.Counter = MAX_FAIL_COUNTER             ;
   last_update_data = hitless_ptr( )->change_flow_sign * Sample;
     
   /* CHECK DATA */
   if ( last_update_data < -1000 || last_update_data > 2500 ){//This range of values indicates a critical state of the sensor reading.
     last_update_data = hitless_ptr( )->last_flow;
     hitless_ptr( )->bubbles_error_counter++; //count the number of ti sensor error until the valid sensor reading.
   }
   else if ( hitless_ptr( )->ti_sensor_status == 0xff ){  //hitless_ptr()->ti_sensor_status == 0xff == 255.
     hitless_ptr( )->bubbles_error_counter++; //count the number of ti sensor error until the valid sensor reading.
   }
   else {
     hitless_ptr( )->last_flow = last_update_data; //save the last valide flow 
     hitless_ptr( )->bubbles_error_counter = 0;
   }
     
   //Detection of PIZUZ, every 30 second check 2 data average and save the average on buff.
   if( hitless_ptr( )->detect_pizuz_flags[1] > 0 ){
     hitless_ptr( )->detect_pizuz_flags[1]-- ;
     alerts_detection_ti( )                  ; //Finally "last_flow" will contain the received average.
        
     if( hitless_ptr( )->detect_pizuz_flags[1] == 0 ){
       //Update the received average as data.
       data_measure.data.A = last_update_data                                   ;  
       data_measure.data.B = (last_update_data * CUBIC_METERS_TO_GALLONS * 60.0);
       data_measure.data.C = hitless_ptr( )->v_vdd                              ;//temporary parameter for save the USB status- shir 
       event_encapsulation( 1                           , 
                            data_measure.buff           ,
                            sizeof(data_measure.buff)   , 
                            (uint8_t *)print_msg_ti     , 
                            &event_temp_size            )                       ; 
       push_event         ( (uint8_t *)print_msg_ti     ,
                            EVENT_SIZE - 1              )                       ;
       //checking for alerts
       if( unusage_alert_flag_ti || pizuz_alert_flag_ti || leakage_alert_flag_ti || leakage_alert_flag_off_ti ){   //The system detects a water critical use (unusage or an explosion or leakage).
              if( unusage_alert_flag_ti     )sprintf( hitless_ptr( )->critic_msg, "[5]Water critical unusage\n[%.1f]l/h [%d] times"      , hitless_ptr( )->unusage_limit, hitless_ptr()->unusage_limit_counter[0] ); 
         else if( pizuz_alert_flag_ti       )sprintf( hitless_ptr( )->critic_msg, "[2]Crossed:[%.1f]l/h->[%.0f] times!\nsend SD command" , hitless_ptr( )->detection_limit, hitless_ptr()->limit_count_result[0]  );
         else if( leakage_alert_flag_ti     )sprintf( hitless_ptr( )->critic_msg, "[6]Warning water leak: crossed:[%d]l/h [%d]times"     , hitless_ptr( )->leak_status, hitless_ptr()->leak_counter[0]            );
         else if( leakage_alert_flag_off_ti )sprintf( hitless_ptr( )->critic_msg, "[7]Water leak stopped: leak status:[%d]"              , hitless_ptr( )->leak_status                                            );
         else if( unusage_alert_flag_off_ti )sprintf( hitless_ptr( )->critic_msg, "[8]Unusage alert stopped: alert status:[%d]"          , hitless_ptr( )->unusage_alert_status                                   );
             
         hitless_ptr( )->send_critical_msg       = 1                                                                 ;   //There is a Critical message 
         hitless_ptr( )->upload_data[0]          = (leakage_alert_flag_off_ti || unusage_alert_flag_off_ti) ? 600 : 1;   //Upload the data from the detection period, after the message will be sent.
         hitless_ptr( )->check_message_coutdown  = 10                                                                ;   //KP_0
       }
        
       hitless_ptr( )->detect_pizuz_flags[1] = 1                             ;
       hitless_ptr( )->Sensor.State          = STATE_SENSOR_SEND_STOP_COMMAND;
     }
   }  
   
   return( NRF_SUCCESS );
}
/**********************************************************************
*  Function Name: update_water_sense_ti                               *
*  Description  :                                                     *
**********************************************************************/  
void update_water_sense_ti(void){
  switch( hitless_ptr( )->Sensor.State ){  
    case STATE_SENSOR_RESET:
      TI_reset( )                                                      ;
      hitless_ptr( )->Sensor.Dly    = MAX_TIME_AFTER_RESET             ;
      hitless_ptr( )->Sensor.State  = STATE_SENSOR_DELAY_AFTER_RESET   ;
      break;
        
    case STATE_SENSOR_DELAY_AFTER_RESET:
      if( --(hitless_ptr( )->Sensor.Dly) ){
        nrf_delay_ms( 1 );
        break;
      }
      
      hitless_ptr( )->Sensor.State = STATE_SENSOR_INIT_I2C;
        
    case STATE_SENSOR_INIT_I2C:
      InitI2CBUS( );
        
      hitless_ptr( )->Sensor.Counter  = MAX_FAIL_COUNTER                  ;
      hitless_ptr( )->Sensor.State    = STATE_SENSOR_SEND_START_COMMAND   ;
      break;
        
    case STATE_SENSOR_SEND_START_COMMAND:
      if( StartTiReadResult( ) != NRF_SUCCESS ){
        if( --(hitless_ptr( )->Sensor.Counter) )break;
          
        hitless_ptr( )->Sensor.State = STATE_SENSOR_RESET;
        break;
      }
        
      hitless_ptr( )->Sensor.State    = STATE_SENSOR_GET_SAMPLES  ;
      hitless_ptr( )->Sensor.Counter  = MAX_FAIL_COUNTER          ;
      break;
        
    case STATE_SENSOR_GET_SAMPLES:
      if( Sensor_GetSamples( ) != NRF_SUCCESS ){
        if( !(--(hitless_ptr( )->Sensor.Counter)) ){
          hitless_ptr( )->Sensor.State = STATE_SENSOR_RESET;
          break;
        }
      }
        
      hitless_ptr( )->Sensor.Dly = MAX_TIME_DELAY;
      break;
      
    case STATE_SENSOR_GET_SAMPLES_DELAY:
      if( --(hitless_ptr( )->Sensor.Dly) ){
        nrf_delay_ms( 1 );
        break;
      }
            
      hitless_ptr( )->Sensor.State = STATE_SENSOR_GET_SAMPLES;
      break;
      
    case STATE_SENSOR_SEND_STOP_COMMAND:
      if( StopTiReadResult( ) != NRF_SUCCESS ){
        if( --(hitless_ptr( )->Sensor.Counter) )break;
          
        hitless_ptr( )->Sensor.State = STATE_SENSOR_RESET;
        break;
      }
        
      hitless_ptr( )->Sensor.Counter = MAX_FAIL_COUNTER;
      
    case STATE_SENSOR_FINISH:
      if( Comm_IsBussy( ) == true ){
        if( --(hitless_ptr( )->Sensor.Counter) )break;
          
        hitless_ptr( )->Sensor.State = STATE_SENSOR_RESET;
        break;
      }
        
      DenitI2CBUS( )                                                    ;
      hitless_ptr( )->Sensor.State          = STATE_SENSOR_INIT_I2C     ;
      hitless_ptr( )->initTIDone            = 0                         ;
      hitless_ptr( )->detect_pizuz_flags[0] = 30                        ;
      hitless_ptr( )->detect_pizuz_flags[1] = 0                         ;  
      hitless_ptr( )->device_state          = SLEEP_STATE               ;     //MO_0
      wrap_NVIC_SystemReset( )                                          ;
      break;
  }
}
#endif 
#define I2C_HEADER    0x80
#define CMD_WRITE_DATA 0x10
#define CMD_EXECUTING_FIRMWARE 0x17
#define CMD_READ_BSL_VERSION 0x19
#define CMD_SEND_BSL_PASSWORD 0x11
#define CMD_MASS_ERASE 0x15
/* Func - to read from TI - Firmware Version && Software Entry Point
   to do - save the current status of the running program in 2 diff vars of version && entry point */
void Firmware_Version(void){//זה פקודה ביני לבין אפרים ובינתיים לא צריכים אותה 
  
    /* Variables */
    Packet_t  txPacket = {0};
    Packet_t  rxPacket = {0};
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* WRITE 0x5A - COMMAND */
    
    txPacket.payload[0] = 0x5A;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER send the SLAVE command to read the firmware version.
  
    /* READ 6 BYTES - DATA 
       Byte 1: FIRMWARE_MAJOR=1.
       Byte 2: FIRMWARE_MINOR=5.
       Byte 3: FIRMWARE_PATCH=2
       Byte 4: Software Entry Point low-byte= address of enter program.
       Byte 5: Software Entry Point  middle-byte=address of enter program.
       Byte 6: Software Entry Point high-byte=address of enter program.
    */
    rxPacket.length = 6; // 6 BYTES 
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket);  //Read 3 + 3 bytes.
    nrf_delay_ms(10);
    
    //compare version.h -the Define firmware version macros to the 3 first bytes.
    //to do after the update software
    //to do include to ti_version.h and get the variable VERSION_MAJOR.VERSION_MINOR.VERSION_PATCH
    
   /* FINISH READ FIRRMWARE VERSION AND SOFTWARE ENTRY POINT */
}
/* enter the TI to bsl mode by the pin test TI_BSL(1,8) && MSP430_RES */
void enter_BSL_Mode_by_HW(void)
{
    nrf_gpio_cfg_output(MSP430_RES);
    nrf_gpio_cfg_output(TI_BSL);
    nrf_gpio_pin_clear(MSP430_RES);
    nrf_delay_ms(5);
    nrf_gpio_pin_set(TI_BSL); //(1,8) pulse up
    nrf_delay_ms(5);
    nrf_gpio_pin_clear(TI_BSL);
    nrf_delay_ms(2);          //break
    nrf_gpio_pin_set(TI_BSL); //(1,8) pulse up
    nrf_delay_ms(2);
    nrf_gpio_pin_set(MSP430_RES);//MSP430_RES 
    nrf_delay_ms(2);
    nrf_gpio_pin_clear(TI_BSL);//bootloader starts
    nrf_delay_ms(10);
}
/* Func - Sending Command to Put the TI into BSL (Bootloader) Mode */
void Bootloader_Mode(void){//זה פקודה שהיא ביני לבין אפרים והוא מכניס את התוכנה לבוט 
  
    /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* WRITE 0x5B - COMMAND */
    
    txPacket.payload[0] = 0x5B;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER send the SLAVE command to Put the TI into BSL.
    
    nrf_delay_ms(10);// wait time TBD!!
    
   /* FINISH Put the TI into BSL */
}
/* Func - Sending Password to Open the BSL */ 
void Password_to_BSL_1(void){

    /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
  
    /* WRITE 4 bytes + 32 bytes BSL_PW + 2 bytes checksum */
    
    txPacket.length = 36;  //4+32 BYTES.
    int i = 0;
    txPacket.payload[i++] = updateParams[0];
    txPacket.payload[i++] = 0x21;
    txPacket.payload[i++] = 0x00;
    txPacket.payload[i++] = CMD_SEND_BSL_PASSWORD;
   
    for(; i<txPacket.length; i++)
       txPacket.payload[i] = 0xFF;

    txPacket.payload[i++] = 0x9E;       // Checksum, CKL
    txPacket.payload[i++] = 0xE6;       // Checksum, CKH
    txPacket.length += 2;               // Increment length to include checksum

    CommandHandler_transmitPacket(&txPacket);  // Write - The MASTER send the SLAVE password.
 
    /* FINISH Sending Password to Open the BSL */ 
}
void Password_to_BSL_1_original(void){

    /* Variables */
    Packet_t  txPacket = {0};
    unsigned char bsl_pw[32] = BSL_PW;
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
  
    /* WRITE 4 bytes + 32 bytes BSL_PW + 2 bytes checksum */
    
    txPacket.length = 36;  //4+32 BYTES.
    int i = 0;
    txPacket.payload[i++] = updateParams[0];
    txPacket.payload[i++] = 0x21;
    txPacket.payload[i++] = 0x00;
    txPacket.payload[i++] = 0x11;
   
    for(; i<36; i++) //sizeof(bsl_pw)
       txPacket.payload[i] = bsl_pw[i-4];

    // Calculate checksum
    uint16_t sum = 0;
    for (int j = 0; j < txPacket.length; j++){
        sum += txPacket.payload[j];
    }
    uint16_t checksum = (uint16_t)(0x10000 - sum); // Two's complement
    uint8_t checksum_low = checksum & 0xFF;
    uint8_t checksum_high = (checksum >> 8) & 0xFF;
    // Append checksum to the payload
    if (txPacket.length + 2 <= PACKET_PAYLOAD_MAX_SZ){
        txPacket.payload[txPacket.length++] = checksum_low;   // Checksum, CKL
        txPacket.payload[txPacket.length++] = checksum_high;  // Checksum, CKH
    } else{
        // Handle error: Payload buffer too small to append checksum
    }
    CommandHandler_transmitPacket(&txPacket);  // Write - The MASTER send the SLAVE password.
 
    /* FINISH Sending Password to Open the BSL */ 
}
/* Func - Sending Password to Open the BSL */ 
int Password_to_BSL_2(void){//read response for sending password
  
    Packet_t  rxPacket = {0};
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
 
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    
    /* READ STATUS 0/1 */
    rxPacket.length = 8; // BYTE 
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket); //Read status boolean = 0 for success, else error.
    int response = rxPacket.payload[4];//0x3B - BSL response for a successful password
    nrf_delay_ms(10);
    
    return response;
}

void Mass_Erase_1(void){
    /* Variables */
    Packet_t  txPacket = {0};
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
    /* WRITE 6 bytes */
    txPacket.length = 6; 
    txPacket.payload[0] = updateParams[0];//0x80 0x01 0x15 0x94
    txPacket.payload[1] =  0x01;
    txPacket.payload[2] =  0x00;
    txPacket.payload[3] =  CMD_MASS_ERASE;
    txPacket.payload[4] =  0x64;
    txPacket.payload[5] =  0xA3;
    CommandHandler_transmitPacket(&txPacket);  // Write - The MASTER send the SLAVE command to erase the flash.
}
/* Func - Send Command to BSL to Remove the Mass(flash) */ 
int Mass_Erase_2(void){
  
    /* Variables */
    Packet_t  rxPacket = {0};
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };
  
    /* READ 8 bytes */
    rxPacket.length = 1;//8
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket); //Read status boolean = 0 for success, else error.
    int response = rxPacket.payload[0];  //0x3B (59 decimal)rxPacket.payload[4]
    nrf_delay_ms(10);
    
    return response;
    /* FINISH Sending Command to erase mass to BSL */ 
}
/* Reading the BSL Version */ 
void Read_BSL_Version(void){// אזה פקודה ביני לבין אפרים ובינתיים לא צריכים אותה - אפשרי אחרי השליחה של הסיסמא - וצריכה לסדר אותה לפי המסמך 

    /* Variables */
    Packet_t  txPacket = {0};
    Packet_t  rxPacket = {0};
    int32_t timeout = COMMAND_HANDLER_TIME_OUT;
    const uint8_t updateParams[] =
    {
        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
        COMMAND_HANDLER_NUM_PLS_ID,        
    };

    txPacket.length = 1;//BYTE
    txPacket.payload[0] = COMMAND_HANDLER_USS_GUI_ID_BYTE;
    txPacket.payload[2] = COMMAND_HANDLER_READ_CMD;
    txPacket.payload[1] = updateParams[0];
    
    /* WRITE - COMMAND */
    
    txPacket.payload[0] = CMD_READ_BSL_VERSION;
    CommandHandler_transmitPacket(&txPacket);   //Write - The MASTER send the SLAVE command 0x19 to read the BSL version.
    
   /*  READ 10 BYTES - DATA as follows:
       bsl_version_response[0] = ACK byte (0x00)
       bsl_version_response[1] = Header byte (0x80)
       bsl_version_response[2] = Length byte low (0x05)
       bsl_version_response[3] = Length byte high (0x00)
       bsl_version_response[4] = Command echo byte (0x3A)
       bsl_version_response[5] = Vendor ID (e.g., 0x00 for TI BSL)
       bsl_version_response[6] = Command Interpreter Version
       bsl_version_response[7] = API Version
       bsl_version_response[8] = Peripheral Interface Version
       bsl_version_response[9] = Checksum byte 1 (CKL)
   */
    rxPacket.length = 10; // BYTES 
    CommandHandler_rxUpdate(updateParams[0], &timeout , &rxPacket);// Read 10 BYTES
    nrf_delay_ms(10);
    
   /* FINISH Reading the BSL Version */ 
}
// Helped Function to calculate the checksum
uint8_t calculate_checksum(uint8_t* data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    uint8_t checksum = (uint8_t)(0x100 - (sum & 0xFF));
    return checksum;
}

/* Executing New/Current Firmware - when TI in the BSL */
void Executing_Firmware(void){
  
    /* Variables */
    uint8_t address[3] = {0x1C,0x79,0x00}; 
    /*  2. Software Entry Point (changes for every version, defined in version.h):
       AL = 0xLL
       AM = 0xMM
       AH = 0xHH
    */
    Packet_t  txPacket = {0};
//    const uint8_t updateParams[] =
//    {
//        COMMAND_HANDLER_GAP_PLS_ADC_START_ID,
//        COMMAND_HANDLER_NUM_PLS_ID,        
//    };
  
    /* WRITE command 0x17(1 byte) + address (3 bytes) + checksum (2 bytes) = 6 bytes */
    
    txPacket.length = 6;  //BYTES
    txPacket.payload[0] = CMD_EXECUTING_FIRMWARE; //command  0x17
    int i=1;
    for(; i<4; i++)
        txPacket.payload[i] = address[i - 1];  // Address
    
    // Calculate the checksum for the first 4 bytes
    txPacket.payload[i] = calculate_checksum(txPacket.payload, 4);  // Checksum, CKL
    txPacket.payload[++i] = 0x00;  // Checksum, CKH
    
//    txPacket.payload[++i] = updateParams[0];
//    txPacket.payload[++i] = COMMAND_HANDLER_READ_CMD;
    CommandHandler_transmitPacket(&txPacket);  //Write - The MASTER send the SLAVE command to Executing New/Current Firmware.
 
    /* FINISH Executing New/Current Firmware */
}

/* Function for write the program code to bsl-slave and reload a new version of program */
void write_DataStruct(void){
  
    // Iterate through each TI_HEX_data_struct
    for (int i = 0; i < sizeof(TI_HEX_data_array) / sizeof(TI_HEX_data_array[0]); i++) {
        TI_HEX_DataStruct dataStruct = TI_HEX_data_array[i];
        
        // Calculate number of bytes to send
        int bytesToSend = dataStruct.data_size > 40 ? 40 : dataStruct.data_size;

        // Send data in chunks of 40 bytes
        for (int j = 0; j < dataStruct.data_size; j += bytesToSend) {
            // Prepare data packet
            Packet_t txPacket = {0};
            txPacket.payload[0] = I2C_HEADER;
            txPacket.payload[1] = ( bytesToSend + 4 ) & 0xFF;             // LSB - Length of packet without including header and checksums.
            txPacket.payload[2] = (( bytesToSend + 4 ) >> 8 ) & 0xFF;     // MSB
            txPacket.payload[3] = CMD_WRITE_DATA;                         // Command WRITE DATA
            txPacket.payload[4] = (dataStruct.base_address >> 8) & 0xFF;  // Address MSB
            txPacket.payload[5] = dataStruct.base_address & 0xFF;         // Address LSB
            txPacket.payload[6] = 0x00;                                   //AH ADDRESS = 0  cause the base address type is just two bytes on the file.h
            for (int k = 0; k < bytesToSend; k++) {
                txPacket.payload[7 + k] = dataStruct.data[j + k];
            }
            txPacket.payload[7 + bytesToSend] = calculate_checksum(txPacket.payload, bytesToSend + 5);      // Checksum Low
            txPacket.payload[8 + bytesToSend] = calculate_checksum(txPacket.payload + 5, bytesToSend + 1);  // Checksum High
            txPacket.length = bytesToSend + 9;

            // Transmit packet
            CommandHandler_transmitPacket(&txPacket); // Write - The MASTER send the SLAVE command to WRITE DATA.
            nrf_delay_ms(100);
            // Move base address forward by 40 bytes
            dataStruct.base_address += 40;
            bytesToSend = (dataStruct.data_size - j - bytesToSend) > 40 ? 40 : (dataStruct.data_size - j - bytesToSend);
        }
    }
}
/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
   
uint8_t rcv_i2c_buffer[128];
int rcv_i2c_index = 0;
__STATIC_INLINE void data_handler(uint8_t temp)
{
    rcv_i2c_buffer[rcv_i2c_index] = temp;//NRF_LOG_INFO("Read Byte.\r\n", temp);
    rcv_i2c_index++;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    m_xfer_done = true;
    Test = false;
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample[0]);
            }
            else
            {
              rcv_i2c_index = 0;
            }
            break;
        default:
            m_xfer_done = true;
            break;
    }
}


void twi_Deinit (void)
{   
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);
}


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bus_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_bus_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief Function for main application entry.
 */
void DenitI2CBUS(void)
{
     twi_Deinit();
     //ACCEL_SW_DIS;
}
void InitI2CBUS(void)
{
     //ACCEL_SW_EN;
     twi_init();
}

int AT24CXX_Init(void)
{
    EEPROM_VARS index;
    int ret_val = 0;
    
    /* read EEPROM */
    for ( index = MANUF_DATE ; index < NUM_OF_EE_VARS ; index++)
    {
      ret_val = AT24CXX_Read(ee_db[index].ee_addr,(uint8_t *)ee_db[index].ee_buff,ee_db[index].ee_len);
      if (ret_val == 0)
        break;
    }
    
    /* check data and correct from defualt */
    if (ee_db[MANUF_DATE].ee_buff[2]!= '/' || ee_db[MANUF_DATE].ee_buff[5]!= '/')
    {
      AT24CXX_Write(ee_db[MANUF_DATE].ee_addr,(uint8_t *)ee_db[MANUF_DATE].ee_resotre_buff,ee_db[MANUF_DATE].ee_len);
    }
    else
      ret_val++;

    
    /* verify eeprom correction*/
    if (ret_val == 0)
    {
          for ( index = MANUF_DATE ; index < NUM_OF_EE_VARS ; index++)
          {
            AT24CXX_Read(ee_db[index].ee_addr,(uint8_t *)ee_db[index].ee_buff,ee_db[index].ee_len);
          }

          if (ee_db[MANUF_DATE].ee_buff[2]== '/' && ee_db[MANUF_DATE].ee_buff[5]== '/')
          {
           ret_val = 1;
          }
    }
    
    return ret_val;
}

void AT24CXX_restore_default(void)
{
    EEPROM_VARS index;

    InitI2CBUS();
    for ( index = MANUF_DATE ; index < NUM_OF_EE_VARS ; index++)
    {
       AT24CXX_Write(ee_db[index].ee_addr,(uint8_t *)ee_db[index].ee_resotre_buff,ee_db[index].ee_len);
    }
   DenitI2CBUS();
}


#endif
/** @} */
