#ifndef _GENERAL_H_
#define _GENERAL_H_



/*************************************/
/* Samples And Sending Packet Length */
/*************************************/


#define EVENT_SIZE 54
#define NUMBER_OF_EVENTS 1800
#define CRITICAL_BATTERY_VOLTAGE 3.6
//#define DEBUG_COMMU 1
//#define DEBUG_BYPASS_MODEM 1
//#define SMS_DEBUG 1
//#define MODE_WORK_MO_1 1
//#define DBG_BUFFER_NOT_EMPTY 1
//#define MAX3510_SENSOR 1   
#define TI_SENSOR 1
//#define SIMCOM7080G 0 /* !! this define also included in pca10056.h !!*/
#define SIMCOM7670 1 /* !! this define also included in pca10056.h !!*/
#define TCP_COMMU 1

#define IMPROVING_EDITING_VER 1

#ifdef DEBUG_COMMU
  #define VERSION "4.00DBG"
#else
  #define VERSION "4.00"
#endif

#define HW_VERSION "2.0"
//#define VALVE_ASSEMBLED 1 

//#define DEBUG_4G 1
#endif

