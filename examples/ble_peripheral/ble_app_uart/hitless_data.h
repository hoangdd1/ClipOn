#ifndef HITLESS_DATA_H
#define HITLESS_DATA_H
#define HITLESS_STATIC_ADDRESS
#include "general.h"
#include "system_info.h"
#include "datastruct_stack.h"
#include "time.h"
/*
RAM: (da07)
START ADDRESS :0x200025f8
END ADDRESS   :0x2000ffff
*/

//#define HITLESS_START_ADDRESS 0x200025f8 /* 2KB */
#define HITLESS_START_ADDRESS 0x20003324 /* 2KB */

/* The arrangment of the items are significance.
If there is a new item, add it to the end.
And do not delete old items. */



typedef union _EVENT_MSG{
  uint16_t uint16_data[EVENT_SIZE/2];
  uint8_t uint8_data[EVENT_SIZE];
}ONE_EVENT_MSG;

typedef enum _SAMPLE_STATUS
{
  EMPTY,
  FIILED
}EVENT_STATUS;



typedef struct _ONE_EVENT_MSG_PACKET
{
   EVENT_STATUS status;
   ONE_EVENT_MSG buffer;
}ONE_EVENT_MSG_PACKET;

typedef struct _EVENT_STACK_DATA
{ 
  Stack_utill events_stack;
  int events_stack_elements[NUMBER_OF_EVENTS+1];
  int event_buffer_index;
  ONE_EVENT_MSG_PACKET buffer_of_events[NUMBER_OF_EVENTS];
}EVENT_STACK_DATA;

typedef union _NUMERIC_ORDER{
  uint32_t uint32_data;
  uint8_t uint8_data[4];
}NUMERIC_ORDER;


typedef enum
{
    POWER_ON_RESET,
    WATCHDOG_RESET,
    EXTERNAL_RESET,
    SOFTWARE_RESET,
    WAKE_UP_RESET   
} RESET_REASON;

typedef enum //shir - Secondary to variable for future use
{
  TEMP_1,
  TEMP_2,
  TEMP_3,
  TEMP_4,
}TEMP;
  
typedef enum
{   
    SLEEP_STATE,
    INIT_WORKING_DEVICE_STATE,
    INIT_SLEEP_STATE,    
    WORKING_DEVICE_STATE,
    PRE_SENDING_VIA_CELLULAR_MODEM,
    SENDING_VIA_CELLULAR_MODEM,
    NUM_OF_DEVICE_STATE,
}DEVICE_STATES;

typedef enum  //shir
{
    MONOGOTO,
    GOLAN_TELECOM,
    BOUYGUES_TELECOM,
    ORANGE,
    SFR,
    FREE_MOBILE,
    PARTNER,
//    PELEPHONE,
//    HOT_MOBILE,
//    CELLCOM,
//    RAMI_LEVY,
//    _012_MOBILE,
   // CELLCOM_INTERNET,
    NUM_OF_NETWORKS,
}CELLULAR_NET;

typedef struct
{
  time_t m_time; 
  time_t m_last_calibrate_time;
  float m_calibrate_factor;
  uint32_t m_rtc_increment;
}RTC_VARS;

typedef enum BIT_STATES_
{
  SPI_WAIT_FOR_ISR,
  SPI_READ_ISR_STATUS_SUCCESS,
  SPI_ERROR_READ_ISR_STATUS,
  NUM_OF_BIT_STATES
}BIT_STATES;
#pragma pack(1)
typedef enum{
  STATE_SENSOR_RESET                ,
  STATE_SENSOR_INIT_I2C             ,
  STATE_SENSOR_DELAY_AFTER_RESET    ,
  STATE_SENSOR_SEND_START_COMMAND   ,
  STATE_SENSOR_GET_SAMPLES          ,
  STATE_SENSOR_GET_SAMPLES_DELAY    ,
  STATE_SENSOR_SEND_STOP_COMMAND    ,
  STATE_SENSOR_FINISH               ,
}Sensor_State_e;

typedef struct{
  Sensor_State_e    State   ;
  uint8_t           Counter ;
  uint16_t          Dly     ;
}Sensor_t;

typedef struct 
{  
  int16_t DFU_flag;    
  RESET_REASON reset_reason;
  DEVICE_STATES device_state;
  EVENT_STACK_DATA event_buff;
  RTC_VARS rtc_var;
  BIT_STATES device_bit;
  int messages_counter;                  //The number of messages that were sent
  int messages_counter_rx;               //The number of messages that have been answered
  int check_message_coutdown;            //Timer of modem section 
  int message_coutdown_interval;         //frequency of modem section
  MODEM_CLUDE_EVENTS modem_event;
  char phone_number_of_administrator[16];//phone num of admin/server.
  int detect_pizuz_flags[4];             //flags for the pizuz detection [0]: countdown 5 minuts  [1]: 30 steps of the detect  [2]: stopwatch from user. [3]:countdown to close the flow.
  float detection_limit;                 //(large flow)-threshold of explosion
  float limit_count_result[2];           //(large flow)-reading counter (0-50) [0]:the const counter  [1]:the current counter. 
  int drip_test_limit;                   //(tiny flow) -threshold of leakage
  int drip_test_stages;                  //(tiny flow) Follow the states of the drip checking
  float unusage_limit;                   // Threshold of lack of water use 
  int unusage_limit_counter[2];          //unusage threshold counter (12 h) [0]:the const counter  [1]:the current counter. 
  int send_critical_msg;                 //Do I have to send a Critical message?
  char critic_msg[50];                   //The critical message to sending
  int flow_controller;                   //enable/disable water flow
  int change_flow_sign;                  //to change the sign of data flow. 
  int upload_data[2];                    //command to Uploading data to the cloud (timer) : [0]:current timer [1]: const value
  int wdt;                               //the number of wdt reset
  int ADC;                               //enable/disable adc(); method
  TEMP for_future_use;                   //Secondary to variable for future use.
  int16_t vbat;                          //save the adc() result of battery voltage
  int16_t v_usb;                         //save the adc() result of usb voltage
  int16_t v_vdd;                         //save the adc() result of usb voltage
  CELLULAR_NET cellular_net;             //save the sim's cellular network
  int16_t BLE_main_task_countdown;       //save the BLE coutdown after power on reset
  int number_of_software_reset;  
  float last_flow;                       //The last valide sensor measurement.
  int bubbles_error_counter;             //The number of occurrences of "Dealing with Bubbles".
  int leak_counter[2];                   //leak counter (4.5 h) [0]:the const counter  [1]:the current counter.
  int modem_signal_quality;              //Save the modem signal quality.
  int leak_status;                       //Save the current leak status - values 0: no leakage, 6: any leak, value 24: medium leak, value 60: severe leak
  int unusage_alert_status;              //Save the current status of unusage - values 1: unusage alert on, 0: unusage alert off.
  float leak_avg;                        //Save the average of the leak.
  int skip_modem_flag;                   //For follow modem skipping bug.
  char network_operator[16];             //The name of the operator of cellolar network.
  int padding[2];                        //For Future Used
  int initTIDone;                        //True if Init TI is Done
  uint8_t ti_sensor_status;              //save the status of the ti sensor. 
  Sensor_t Sensor;
  
} HITLESS_STRUCTURE;
#pragma pack(4)  
RESET_REASON  check_reset_reason(void);
void reset_hitless_data();

HITLESS_STRUCTURE * hitless_ptr();

#endif
