/*
 * sms_message.c
 *
 * Created: 10/9/2015 12:15:54 AM
 *  Author: ofer
 */ 

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include "common.h"
#include "sms_message.h"
#include "rf_message.h"
#include "status.h"
#include "cleaning_wagon_control.h"
//#include "mirf.h"
#include "timing.h"
#include "power_supply.h"
#include "sim5360e.h"
#include "rtc.h"
#include "water_flow_meter.h"
#include "water_pump.h"
#include "battery.h"
#include "smart_battery.h"
#include "stm32f4xx_hal.h"

/**************************************************************/
/*                       DEFINITIONS                          */
/**************************************************************/
#define SMS_SEND_MAX_RETRY        20

typedef struct  
{
	char *sms_string;
	char *sms_parameters;
	char *sms_help;
} SMS_MESSAGE_DETAILS;

typedef enum
{
    READ_COMMAND,
    WRITE_COMMAND
} COMMAND_TYPE;

/**************************************************************/
/*                        VARIABLES                           */
/**************************************************************/
#if 1

SMS_MESSAGE_DETAILS sms_message_details[NUMBER_OF_SMS_MESSAGE] =
{
/*  0 - HP_SMS_MESSAGE */ { "HP" , "NO PARAMETERS"  , "HELP"				 } ,
/*  1 - OK_SMS_MESSAGE */ { "OK" , "NO PARAMETERS"  , "STATUS REPORTING"		 } ,
/*  2 - ST_SMS_MESSAGE */ { "ST" , "[HH:MM]"        , "SET (ACTIVATION) TIME"		 } ,				
/*  3 - SF_SMS_MESSAGE */ { "SF" , "[DAYS]"         , "SET (ACTIVATION) FREQUENCY"	 } ,				
/*  4 - ER_SMS_MESSAGE */ { "ER" , "NO PARAMETERS"  , "ERROR EVENT"			 } ,
/*  5 - CC_SMS_MESSAGE */ { "CC" , "NO PARAMETERS"  , "CLEANING COMPLETED"		 } ,				
/*  6 - SC_SMS_MESSAGE */ { "SC" , "NO PARAMETERS"  , "START CLEANING"                   } ,
/*  7 - CT_SMS_MESSAGE */ { "CT" , "[VOLTAGE]"      , "BATTERY THRESHOLD CLEANING WAGON" } ,
/*  8 - TT_SMS_MESSAGE */ { "TT" , "[VOLTAGE]"      , "BATTERY THRESHOLD TRANSFER WAGON" } ,
/*  9 - AD_SMS_MESSAGE */ { "AD" , "NO PARAMETERS"  , "ADC DETAILS"			 } ,
/* 10 - LC_SMS_MESSAGE */ { "LC" , "NO PARAMETERS"  , "LOCATION DETAILS"		 } ,				
/* 11 - WF_SMS_MESSAGE */ { "WF" , "[Hz]"           , "WATER FLOW DETAILS"		 } ,
/* 12 - PM_SMS_MESSAGE */ { "PM" , "[%]"            , "PUMP MOTOR PRECENTAGES"  	 } ,
/* 13 - GF_SMS_MESSAGE */ { "GF" , "NO PARAMETERS"  , "GO FORWARD"			 } ,
/* 14 - GB_SMS_MESSAGE */ { "GB" , "NO PARAMETERS"  , "GO BACKWARD"			 } ,				
/* 15 - SP_SMS_MESSAGE */ { "SP" , "NO PARAMETERS"  , "STOP CLEANING PROCESS"		 } ,
/* 16 - RS_SMS_MESSAGE */ { "RS" , "NO PARAMETERS"  , "RESET SYSTEM"			 } ,
/* 17 - PA_SMS_MESSAGE */ { "PA" , "NO PARAMETERS"  , "PAUSE SYSTEM"			 } ,
/* 18 - CO_SMS_MESSAGE */ { "CO" , "NO PARAMETERS"  , "CONTINUE SYSTEM"			 } ,
/* 19 - DB_SMS_MESSAGE */ { "DB" , "NO PARAMETERS"  , "DEBUGING"			 } ,
/* 20 - AN_SMS_MESSAGE */ { "AN" , "[PHONE NUMBER]" , "ADD (PHONE) NUMBER"		 } ,
/* 21 - RN_SMS_MESSAGE */ { "RN" , "[PHONE NUMBER]" , "REMOVE (PHONE) NUMBER"		 } ,		
/* 22 - LN_SMS_MESSAGE */ { "LN" , "NO PARAMETERS"  , "LIST (OF PHONE) NUMBERS"		 }
};

#else

char *sms_message_details[NUMBER_OF_SMS_MESSAGE] =
{
/*  0 - HP_SMS_MESSAGE */ "HP" ,
/*  1 - OK_SMS_MESSAGE */ "OK" ,
/*  2 - ST_SMS_MESSAGE */ "ST" ,				
/*  3 - SF_SMS_MESSAGE */ "SF" ,				
/*  4 - ER_SMS_MESSAGE */ "ER" ,
/*  5 - CC_SMS_MESSAGE */ "CC" ,				
/*  6 - SC_SMS_MESSAGE */ "SC" ,
/*  7 - CT_SMS_MESSAGE */ "CT" ,
/*  8 - TT_SMS_MESSAGE */ "TT" ,
/*  9 - AD_SMS_MESSAGE */ "AD" ,
/* 10 - LC_SMS_MESSAGE */ "LC" ,				
/* 11 - WF_SMS_MESSAGE */ "WF" ,
/* 12 - PM_SMS_MESSAGE */ "PM" ,
/* 13 - GF_SMS_MESSAGE */ "GF" ,
/* 14 - GB_SMS_MESSAGE */ "GB" ,				
/* 15 - SP_SMS_MESSAGE */ "SP" ,
/* 16 - RS_SMS_MESSAGE */ "RS" ,
/* 17 - PA_SMS_MESSAGE */ "PA" ,
/* 18 - CO_SMS_MESSAGE */ "CO" ,
/* 19 - DB_SMS_MESSAGE */ "DB" ,
/* 20 - AN_SMS_MESSAGE */ "AN" ,
/* 21 - RN_SMS_MESSAGE */ "RN" ,		
/* 22 - LN_SMS_MESSAGE */ "LN"
};

#endif

static char phone_number_of_administrator[11] = "0544522121";
//static char phone_number_of_administrator[11] = "0523969145";
static char phone_number_of_sender[11]        = "0544522121";
//static char phone_number_of_sender[11]        = "0523969145";
uint8_t number_of_waiting_messages = 0;

/**************************************************************/
/*                        FUNCTIONS                           */
/**************************************************************/
#if 0
char * send_help_sms_message()
{
    SMS_MESSAGE sms_message_index;
    static char sms_message[500];
    char        sms_message_record[20];
    
    //for( sms_message_index = 0 ; sms_message_index < NUMBER_OF_SMS_MESSAGE ; sms_message_index++ )
    for( sms_message_index = (SMS_MESSAGE)0 ; sms_message_index < AN_SMS_MESSAGE ; sms_message_index++ )
    {
        sprintf( sms_message_record , "%s %s\r\n" , sms_message_details[sms_message_index].sms_string , sms_message_details[sms_message_index].sms_parameters );
        strcat( sms_message , sms_message_record );
    }
    return sms_message;
}
#endif

char *get_help_string()
{
    SMS_MESSAGE sms_command_index;
    static char help_string[300] = "COMMANDS:\r\n";
    char current_string[40];
    
    for( sms_command_index = HP_SMS_MESSAGE ; sms_command_index < 20/*NUMBER_OF_SMS_MESSAGE*/ ; sms_command_index++ )
    {
        sprintf( current_string , "%s\r\n" , sms_message_details[sms_command_index].sms_string );
        //sprintf( current_string , "%s\r\n" , sms_message_details[sms_command_index] );
        strcat( help_string , current_string );
    }
    return help_string;
}

void send_sms_to_administrator( char* sms_message )
{
    sim5360e_send_sms_message( phone_number_of_administrator , sms_message );
}

void send_sms_to_sender( char* sms_message )
{
    sim5360e_send_sms_message( phone_number_of_sender , sms_message );
}

void send_sms_message_to_sender( SMS_MESSAGE sms_message )
{
    //static char sms_message_text[500];
    //get_sms_message( sms_message , sms_message_text );
    //sim5360e_send_sms_message( phone_number_of_sender , sms_message_text );
    sim5360e_send_sms_message( phone_number_of_sender , get_sms_message( sms_message ) );
}

void set_phone_number_of_sender( char *international_phone_number_of_sender )
{
	memcpy( phone_number_of_sender , &international_phone_number_of_sender[3] , strlen(international_phone_number_of_sender) - 3 );
    phone_number_of_sender[0] = '0';  // First digit of a phone number in local calling
}

COMMAND_TYPE received_sms_message_parsing( char * sms_message , char * command , char * command_parameter )
{
    uint16_t index = 0;
    char ok_string[30];
    char phone_number[256];
    char date[30];
    char time[30];
    char sms_text[256];
    char parameter[30];
    char temporary_string[256];
    char *start_of_relavent_information;
    char *end_of_relavent_information;
    
    start_of_relavent_information = (char*)memchr( sms_message     , '+' , 50 );
    start_of_relavent_information = (char*)memchr( start_of_relavent_information + 1 , '+' , 50 );
    strcpy( temporary_string , start_of_relavent_information );
    end_of_relavent_information = (char*)memchr( start_of_relavent_information , '\0' , strlen(start_of_relavent_information) + 1 );
    while( &start_of_relavent_information[index] != end_of_relavent_information )
    {
        if( start_of_relavent_information[index] == ','
         || start_of_relavent_information[index] == '"' 
         || start_of_relavent_information[index] == '\r' 
         || start_of_relavent_information[index] == '\n' )
            temporary_string[index] = ' ';
        else
            temporary_string[index] = start_of_relavent_information[index];
        index++;
    }
    sscanf( temporary_string , "%s%s%s%s%s%s" , phone_number , date , time , sms_text , parameter , ok_string );      
    set_date( &date[0] );
    set_time( &time[0] );
    set_phone_number_of_sender( phone_number );
    strcpy( command , sms_text );
    strcpy( command_parameter , parameter );
    return ( ( strncmp( parameter , "OK" , 2 ) == 0 ) ? READ_COMMAND : WRITE_COMMAND );
}

void string_to_upper( char * string )
{
    while( string != NULL && *string != CARRIAGE_RETURN && *string != LINE_FEED )
    {
        *string = toupper( *string );
        string++;
    }
}

void sms_message_loop()
{
    char sms_text[257];
    char command[4];
    char parameter[10];
    SMS_MESSAGE  sms_message;
    COMMAND_TYPE command_type;	
    uint8_t      message[2] = { IRRELEVANT , IRRELEVANT };    
    char error_string[30];

    if( number_of_waiting_messages > 0 )                        //New Message has arrived
    {
        number_of_waiting_messages--;
        sim5360e_reset_ri_pin_of_serial_port();
        sim5360e_read_received_message(sms_text);
        if( strstr( (const char *)sms_text , "ERROR" ) == NULL )
        //if( 1 )
        {
            command_type = received_sms_message_parsing( sms_text , command , parameter );
            string_to_upper(command);
            for( sms_message = (SMS_MESSAGE)0 ; sms_message < NUMBER_OF_SMS_MESSAGE ; sms_message++ )
            {
                if( strncmp( command , sms_message_details[sms_message].sms_string , 2 ) == 0 )
                //if( strncmp( command , sms_message_details[sms_message] , 2 ) == 0 )
                    break;
            }
            if( sms_message >= NUMBER_OF_SMS_MESSAGE )
                send_sms_to_sender( "ERROR - UNKNOWN COMMAND" );
            else
            {
                switch (sms_message)
                {
                	case HP_SMS_MESSAGE: // "HP"
                                        send_sms_message_to_sender(HP_SMS_MESSAGE);
                                    	break;

                    case OK_SMS_MESSAGE:// "OK"
                                        send_rf_message( TRANSFER_WAGON_VERSION_MESSAGE , message , 0 );
                                        HAL_Delay(2000);
                                        receive_rf_message();
                                        send_sms_message_to_sender(OK_SMS_MESSAGE);
                                        //transfer_wagon_version_already_received = true;
                                        break;
                            
                    case ST_SMS_MESSAGE:// "ST"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && set_alarm( parameter , true ) == true ) )
                                            send_sms_message_to_sender(ST_SMS_MESSAGE);
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - PARAMETER IS:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
                
                    case SF_SMS_MESSAGE: // "SF"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && set_activation_days_frequency( atoi(parameter) ) == true ) ) 
                                            send_sms_message_to_sender(SF_SMS_MESSAGE);
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - UNVALID PARAMETER:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
                        
                    case SC_SMS_MESSAGE: // "SC"
                                        send_start_cleaning_message();
                                        break;
        
                    case PA_SMS_MESSAGE: // "PA"
                                        pause_system();
                                        break;
                   
                    case CO_SMS_MESSAGE: // "CO"
                                        continue_system();
                                        break;
                        
                    case CT_SMS_MESSAGE: // "CT"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && set_cleaning_wagon_battery_voltage_threshold( atof(parameter) ) == true ) ) 
                                            send_sms_message_to_sender(CT_SMS_MESSAGE);
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - UNVALID PARAMETER:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
                    
                    case TT_SMS_MESSAGE: // "TT"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && battery_set_transfer_wagon_voltage_threshold( atof(parameter ) * 10 ) == true ) )
                                        {
                                            message[0] = (uint8_t)( (int)( atof( parameter ) * 10 ) / 256 );
                                            message[1] = (uint8_t)( (int)( atof( parameter ) * 10 ) % 256 );
                                            send_rf_message( SET_TRANSFER_WAGON_POWER_SUPPLY_THRESHOLD_MESSAGE , message , 2 );
                                        }
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - UNVALID PARAMETER:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
        
                    case AD_SMS_MESSAGE: // "AD"
                                        send_rf_message( TRANSFER_WAGON_POWER_SUPPLY_BITS_MESSAGE , message , 0 );
                                        //send_sms_to_sender( get_sms_message(CT_SMS_MESSAGE) );
                                        break;
                    
                    case LC_SMS_MESSAGE: // "LC"
                                        send_sms_message_to_sender(LC_SMS_MESSAGE);
                                        break;
                    
                    case WF_SMS_MESSAGE: // "WF"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && set_minimum_water_flow_threshold( atoi(parameter) ) == true ) ) 
                                            send_sms_message_to_sender(WF_SMS_MESSAGE);
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - UNVALID PARAMETER:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
        
                    case PM_SMS_MESSAGE: // "PM"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && activate_water_pump_motor_precentages( atoi(parameter) ) == true ) ) 
                                            send_sms_message_to_sender(PM_SMS_MESSAGE);
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - UNVALID PARAMETER:%s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;
                                    
                    case SP_SMS_MESSAGE: // "SP"
                                        set_cleaning_process_stopped_by_user();
                                        send_rf_message( STOP_CLEANING_PROCESS_MESSAGE , message , 0 );
                                        break;
        
                    case GF_SMS_MESSAGE: // "GF"
                                        if( check_if_cleaning_wagon_active_now() == false )
                                            send_rf_message( GO_FORWARD_MESSAGE , message , 0 );
                                        else
                                            send_sms_to_sender( "TRANSFER WAGON CANNOT GO FORWARD MANUALY WITHOUT CLEANING WAGON ON IT !!!" );
                                        break;
        
                    case GB_SMS_MESSAGE: // "GB"
                                        if( check_if_cleaning_wagon_active_now() == false )
                                            send_rf_message( GO_BACKWARD_MESSAGE , message , 0 );
                                        else
                                            send_sms_to_sender( "TRANSFER WAGON CANNOT GO BACKWARD MANUALY WITHOUT CLEANING WAGON ON IT !!!" );
                                        break;
        
                    case RS_SMS_MESSAGE: // "RS"
                                        reset_system();
                                        send_rf_message( RESET_SYSTEM_MESSAGE , message , 0 );
                                        send_sms_message_to_sender(RS_SMS_MESSAGE);
                                        break;
                    
                    case DB_SMS_MESSAGE: // "DB"
                                        if( command_type == READ_COMMAND || ( command_type == WRITE_COMMAND && ( atoi(parameter) == 0 || atoi(parameter) == 1 ) ) ) 
                                        {
                                            message[0] = (uint8_t)( atoi(parameter) );
                                            send_rf_message( DEBUG_MESSAGE , message , 1 );
                                        }
                                        else
                                        {
                                            sprintf( error_string ,"ERROR - WRONG PARAMETER: %s" , parameter );
                                            send_sms_to_sender( error_string );
                                        }
                                        break;

                    default:			break;
                }
            }
        }
        else
            send_sms_to_sender( "ERROR - INVALID SMS COMMAND" );
    }      
}

void sms_message_reception_interrupt_enable()
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);

	/*Configure GPIO pins : PE2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void sms_message_reset_number_of_waiting_messages()
{
	number_of_waiting_messages = 0;
}

void EXTI2_IRQHandler(void)
{
    if( __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET )
    {
        /* Clear the EXTI line 2 pending bit */
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
        number_of_waiting_messages++;
    }
}

