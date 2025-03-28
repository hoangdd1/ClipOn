#ifndef _GENERAL_H_
#define _GENERAL_H_



/*************************************/
/* Samples And Sending Packet Length */
/*************************************/


#define EVENT_SIZE 54
#define NUMBER_OF_EVENTS 1800
#define CRITICAL_BATTERY_VOLTAGE 3.66
//#define DEBUG_COMMU 1
//#define DEBUG_BYPASS_MODEM 1
//#define SMS_DEBUG 1
//#define MODE_WORK_MO_1 1
//#define DBG_BUFFER_NOT_EMPTY 1
#define IMPROVING_EDITING_VER 1

#ifdef DEBUG_COMMU
  #define VERSION "3.33DBG"
#else
  #define VERSION "3.33"
#endif

#endif

