#ifndef _MESSAGE_HANDLLER_H_
#define _MESSAGE_HANDLLER_H_
#include "general.h"
#include "hitless_data.h"
typedef struct {
        uint8_t UUID[6];
        uint8_t sensor_number[3];
        uint8_t packet_type;
        uint8_t sysuptime[4];
        uint8_t counter_relay_msg;
        uint8_t counter_self_msg;
        uint8_t vbat; /* 3.2v = 320 - 200 = 120 */
} messagePacket2_t;

#define MESSAGE_PACKET_1_LENGTH 36
typedef struct __PACKET_
{
    uint8_t preamble [4];
    uint8_t device_UUID[8];
    uint8_t UnixTimeStamp[4];
    int8_t  packet_type;
    int8_t  data_length;
    uint8_t data_transmit_counter;
    uint8_t data_battery_level;
    uint8_t data_sensor_status_byte;
    uint8_t data_measure[16];
    uint8_t packet_checksum;
}messagePacket_t; 

#define MESSAGE_PACKET_SIZE 38 

typedef union{
  messagePacket_t packet;
  uint8_t buff[MESSAGE_PACKET_1_LENGTH];
}union_messagePacket_t;

typedef struct _MESURE_DATA_
{
  float A;
  float B;
  float C;
  float D;
}MEASURE_DATA;

typedef union{
  uint32_t UTC;
  uint8_t UnixTimeStamp[4];
}UnixTimeStamp_t;

  
typedef union{
  MEASURE_DATA data;
  uint8_t buff[16];
}union_measure_data_t;    
    
typedef struct
{
  /*point to the App data buffer*/
  uint8_t* Buff;
  /*LoRa App data buffer size*/
  uint8_t BuffSize;

  
} pop_AppData_t;

void top_event(uint8_t * pop_data , int * length_of_pop_data);
void pop_event();
int event_buffer_upload();
int event_buffer_not_empty();
void event_encapsulation(int packet_type,uint8_t * payload_event,int payload_len,uint8_t *push_message , int *push_message_len);
int check_activeation_key();
void events_stack_init(RESET_REASON reset_type);
int event_check_message();
#endif /* _MESSAGE_HANDLLER_H_ */