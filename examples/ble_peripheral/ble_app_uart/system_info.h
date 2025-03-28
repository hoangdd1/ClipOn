#ifndef _SYSTEM_INFO_H_
#define _SYSTEM_INFO_H_

typedef enum
{
    MODEM_POWERUP,
    MODEM_SETUP,
    MODEM_SETUP_SMS,
    MODEM_TRANSMMISION_BUFFER,
    MODEM_WAIT_FOR_COMMAND,
    MODEM_POWER_OFF,
    MODEM_FINISH,
    MODEM_ERROR_POWERUP,
    MODEM_SETUP_ERROR,
    MODEM_STATE_NUM   
} MODEM_STATE;

typedef union _UNION_idCodeShort{
  uint8_t buff[8];
  uint32_t uid[2];
}UNION_idCodeShort;


typedef enum _MODEM_CLUDE_EVENTS_
{
  NO_EVENT,
  MODEM_CLOUD_EVENT,
  MODEM_SMS_EVENT,
  MODEM_NUM_OF_EVENTS
}MODEM_CLUDE_EVENTS;



typedef struct{  
  uint8_t date;
  uint8_t time_hour; 
  uint32_t transmmision_number;
  uint32_t sysuptime;     // second 
  uint32_t keepalive_timestamp;
  int32_t  countdown_sleep_seconds;
  int16_t  countdown_transmission_suspend;
  int16_t  transmission_faild_countdown;
  int16_t  rpw_20Min_working_bank;
  uint8_t counter_self_msg;
  uint16_t counter_relay_msg;
  UNION_idCodeShort idCodeShort;       
  uint32_t noise_start_sysuptime; //itai[+]
  uint8_t  noise_section_counter; //itai[+]
  uint8_t count_less_then_1H_sleep_section;
  int8_t modem_params_exist;
  int8_t standalone_name;
  int16_t msg_relay_budget;
  int8_t keep_alive_event;
  int8_t ble_tx_power;
  int8_t non_filter_rpw;
  int8_t system_error;
  MODEM_STATE modem_state;
  int finish_sms_session;
}SYSTEM_INFO;


typedef struct {
  uint8_t packet_type;
  uint8_t sysuptime[4];  
  uint8_t time_hour;
  uint8_t time_min;
  uint8_t battary_level;
  uint8_t operation_mode;  
  uint8_t software_ver[2]; 
} system_info_Packet_t;



void system_info_init();
extern SYSTEM_INFO sys_info;


#endif /* _SYSTEM_INFO_H_ */


  
