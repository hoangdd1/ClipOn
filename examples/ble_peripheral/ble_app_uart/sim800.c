/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : SOLAR ROBOT                                                                                    *
 *  File name  : sim5360e.c                                                                                     *
 *  Abstract   : GSM driver.                                                                                    *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : MARCH 2017                                                                                     *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "stm32f4xx_hal.h"
#include "common.h"
#include "sms_message.h"

/*-------------------------------------------------------------------------------------------*/
/*----------------------------- DEFINITIONS AND ENUMARTIONS ---------------------------------*/
/*-------------------------------------------------------------------------------------------*/
typedef enum
{
    NETWORK_NOT_REGISTRATED_NOT_SEARCHING_NEW_OPERATOR = '0',
    NETWORK_REGISTRATED                                = '1',
    NETWORK_NOT_REGISTRATED_SEARCHING_NEW_OPERATOR     = '2',
    NETWORK_REGISTRATION_DENIED                        = '3',
    UNKNOWN_STATE                                      = '4',
    ROAMING                                            = '5'  
} NETWORK_REGISTRATION_STATUS;

typedef struct
{
    char manufactor[20];
    char model[16];
    char revision[47];
    char qcn[10];
    char imei[16];
    char imsi[16];
    char request_overall_capabilities[20];
    char device_information[11];
    char cpin[12];
    char signal_quality[7];
    char network_registration[7];
    char phone_functionality[10];
    char ue_system_informatio[200];
    char te_character_set[16];
    char sms_service_centre_address[50];
    char operator_selection[50];
} GSM_INFORMATION;

#define UART_TIMEOUT                  1000  
#define DELAY_BETWEEN_COMMANDS        50  
#define MAXIMUM_MESSAGES_IN_RESPONSE  10
#define MAXIMUM_MESSAGE_LENGTH        40
#define CR_LF                         "\r\n"
#define NETWORK_REGISTRATED_SUBSTRING ",1\r\n"
#define ATI_STATUS                    "ATI\r\n"
#define AT_CSCS_STATUS                "AT+CSCS?\r\n"
#define AT_CSCA_STATUS                "AT+CSCA?\r\n"
#define AT_CREG_STATUS                "AT+CREG?\r\n" 
#define AT_COPS_STATUS                "AT+COPS?\r\n" 
#define AT_CPIN_STATUS                "AT+CPIN?\r\n" 
#define AT_CFUN_STATUS                "AT+CFUN?\r\n" 
#define AT_CPSI_STATUS                "AT+CPSI?\r\n"
#define AT_CTZR_STATUS				  "AT+CTZR=?\r\n"
#define AT_CTZR_COMMAND				  "AT+CTZR=1\r\n"
#define AT_COPS_COMMAND               "AT+COPS=0,0\r\n"
#define AT_CREG_COMMAND               "AT+CREG=1\r\n"  
#define AT_CSCS_COMMAND               "AT+CSCS=\"GSM\"\r\n"
#define AT_CIMI_COMMAND               "AT+CIMI\r\n"
#define AT_CFGRI_STATUS               "AT+CFGRI?\r\n"
#define AT_CFGRI_COMMAND              "AT+CFGRI=1,1\r\n"
#define AT_CMGF_COMMAND               "AT+CMGF=1\r\n"
#define AT_CMGRD_COMMAND              "AT+CMGRD=0\r\n"
#define AT_SIMCOMATI_STATUS           "AT+SIMCOMATI\r\n"
#define AT_CSQ_STATUS                 "AT+CSQ\r\n"
#define AT_CFUN_COMMAND_0             "AT+CFUN=0\r\n"  
#define AT_CFUN_COMMAND_1             "AT+CFUN=1\r\n"  
#define AT_ATE0_COMMAND               "ATE0\r\n"  
#define AT_CRIRS_COMMAND              "AT+CRIRS\r\n"
#define AT_CGFUNC_COMMAND             "AT+CGFUNC=17,1\r\n"
#define AT_CPMS_COMMAND               "AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n"
#define AT_CTZR 					  "AT+CTZR=0\r\n"

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern          UART_HandleTypeDef huart1;
static char     response[MAXIMUM_MESSAGES_IN_RESPONSE][MAXIMUM_MESSAGE_LENGTH];
GSM_INFORMATION gsm_information;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void parse_response( char *response_string )
{
    char     *temporal_string = &response_string[1];  
    char     *previous_string = &response_string[1];
    uint8_t  message_index = 0;

    while( ( temporal_string = strstr( previous_string , CR_LF ) ) != NULL )
    {
        strncpy( response[message_index] , previous_string , temporal_string - previous_string );
        response[message_index++][temporal_string - previous_string] = '\0'; 
        previous_string = temporal_string + 2;
    }
    return;
}
  
static void power_on_sequence()
{
    HAL_GPIO_WritePin( GPIOF , GPIO_PIN_14 , GPIO_PIN_SET );
    HAL_Delay(500);
    HAL_GPIO_WritePin( GPIOF , GPIO_PIN_14 , GPIO_PIN_RESET );
}

void sim5360e_read_information()
{
    uint8_t information[138];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)ATI_STATUS , 5   , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , information           , 137 , UART_TIMEOUT );
    parse_response( (char *)information );
    strcpy( gsm_information.manufactor , &response[1][14] );
    strcpy( gsm_information.model      , &response[2][7]  );
    strcpy( gsm_information.revision   , &response[3][10] );
    strcpy( gsm_information.imei       , &response[4][6]  );
}

void sim5360_disable_command_echo()
{
    uint8_t response[12];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_ATE0_COMMAND  , sizeof( AT_ATE0_COMMAND ) - 1 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , response                    , 11 , UART_TIMEOUT );
    return;
}

void sim5360e_read_simcom_information()
{
    uint8_t information[210];
    char    revision_second_part[28];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_SIMCOMATI_STATUS , 14  , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , information                    , 210 , UART_TIMEOUT );
    parse_response( (char *)information );
    strcpy( gsm_information.manufactor                   , &response[0][14] );
    strcpy( gsm_information.model                        , &response[1][7]  );
    strcpy( gsm_information.revision                     , &response[2][10] );
    strcpy( revision_second_part                         , &response[3][0]  );
    strcat( gsm_information.revision                     , " " );        
    strcat( gsm_information.revision                     , revision_second_part );        
    strcpy( gsm_information.qcn                          , &response[4][5]  );
    strcpy( gsm_information.imei                         , &response[5][6]  );
    strcpy( gsm_information.request_overall_capabilities , &response[6][7]  );
    strcpy( gsm_information.device_information           , &response[7][12] );
    return;
}

void sim5360e_set_ri_interrupt()
{
    uint8_t ri_interrupt_configuration[20];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CFGRI_COMMAND , 14 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , ri_interrupt_configuration  ,  6 , UART_TIMEOUT );
}

void sim5360e_read_select_te_character_set()
{
    uint8_t te_character_set[21];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CSCS_COMMAND , 15 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , te_character_set           , 6  , UART_TIMEOUT );
}

void sim5360e_read_cpin()
{
    uint8_t cpin_string[50];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CPIN_STATUS , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , cpin_string               , 22 , UART_TIMEOUT );
    parse_response( (char *)cpin_string );
    strcpy( gsm_information.cpin , &response[1][7] );
}

void sim5360e_read_signal_quality()
{
    uint8_t signal_quality[50];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CSQ_STATUS ,  8 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , signal_quality           , 10 , UART_TIMEOUT );
    parse_response( (char *)signal_quality );
    strcpy( gsm_information.signal_quality , &response[1][7] );
}

void sim5360e_sms_service_centre_address()
{
    uint8_t sms_service_centre_address[50];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CSCA_STATUS  , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , sms_service_centre_address , 50 , UART_TIMEOUT );
    parse_response( (char *)sms_service_centre_address );
    strcpy( gsm_information.sms_service_centre_address , &response[1][0]  );
}

void sim5360e_request_international_mobile_subscriber_identify()
{
    uint8_t international_mobile_subscriber_identify[34];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CIMI_COMMAND , 9  , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , international_mobile_subscriber_identify , 33 , UART_TIMEOUT );
    parse_response( (char *)international_mobile_subscriber_identify );
    strcpy( gsm_information.imsi , &response[1][0] );
}

void sim5360e_select_sms_message_format()
{
    uint8_t sms_message_format[17];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CMGF_COMMAND , 11 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , sms_message_format         ,  6 , UART_TIMEOUT );
    parse_response( (char *)sms_message_format );
}

bool sim5360e_network_registration_status()
{
    uint8_t network_registration[30];
    
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CREG_STATUS , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , network_registration      , 20 , UART_TIMEOUT );
    return( ( strstr( (char *)network_registration , NETWORK_REGISTRATED_SUBSTRING ) != NULL ) ? true : false );
}

void sim5360e_network_registration()
{
    uint8_t network_registration[30];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CREG_COMMAND , 11 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , network_registration       , 6  , UART_TIMEOUT );
}

void sim5360e_preferred_message_storage()
{
    uint8_t response[32];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CPMS_COMMAND , 24 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , response                   , 31 , UART_TIMEOUT );
}

/*
void sim5360e_network_registration_status()
{
    uint8_t network_registration[30];

    HAL_UART_Transmit( &huart1 , AT_CREG_STATUS    , 10 , UART_TIMEOUT );      
    HAL_UART_Receive(  &huart1 , network_registration , 29 , UART_TIMEOUT );
    parse_response( (char *)network_registration );
    strcpy( gsm_information.network_registration , &response[1][0] );
    return;
}
*/
void sim5360e_phone_functionality_command(uint8_t *phone_functionality_command)
{
    uint8_t phone_functionality[30];

    HAL_UART_Transmit( &huart1 , phone_functionality_command , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , phone_functionality         , 6 , UART_TIMEOUT );
    parse_response( (char *)phone_functionality );
    strcpy( gsm_information.phone_functionality , &response[1][0] );
}

void sim5360e_phone_functionality_status()
{
    uint8_t phone_functionality[30];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CFUN_STATUS , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , phone_functionality       , 18 , UART_TIMEOUT );
    parse_response( (char *)phone_functionality );
    strcpy( gsm_information.phone_functionality , &response[1][0] );
}

void sim5360e_operator_selection_status()
{
    uint8_t operator_selection[18];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_COPS_STATUS , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , operator_selection        , 17 , UART_TIMEOUT );
    parse_response( (char *)operator_selection );
    strcpy( gsm_information.operator_selection , &response[1][0] );
}

void sim5360e_inquiring_ue_system_information()
{
    uint8_t ue_system_informatio[200];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CPSI_STATUS , 10  , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , ue_system_informatio      , 200 , UART_TIMEOUT );
    parse_response( (char *)ue_system_informatio );
    strcpy( gsm_information.ue_system_informatio , &response[1][0] );
}

void sim5360e_reset_ri_pin_of_serial_port()
{
    uint8_t response[6];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CRIRS_COMMAND , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , response                    , 6  , UART_TIMEOUT );
}

void sim5360e_enable_the_function_for_the_special_gpio()
{
    uint8_t response[6];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CGFUNC_COMMAND , 16 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , response                     , 6  , UART_TIMEOUT );
}

void sim5360e_operator_selection()
{
    uint8_t operator_selection[36];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_COPS_COMMAND , 13 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , operator_selection         ,  6 , UART_TIMEOUT );
    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_COPS_STATUS  , 10 , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , operator_selection         , 18 , UART_TIMEOUT );
    parse_response( (char *)operator_selection );
    strcpy( gsm_information.operator_selection , &response[1][0] );
    return;
}

void sim5360e_send_sms_message( char *phone_number , char *sms_message )
{
    uint8_t  sms_send_message[512];
    static   uint16_t messages_counter = 1;

    sprintf( (char *)sms_send_message , "AT+CMGS=\"%s\",129\r\r\n" , phone_number );
    HAL_UART_Transmit( &huart1 , (uint8_t *)sms_send_message , 27 , UART_TIMEOUT );  
    HAL_UART_Receive(  &huart1 , sms_send_message , 4 , 50 );
    if( sms_message != NULL )
    	sprintf( (char *)sms_send_message , "%d.%s%c" , messages_counter++ , sms_message , 0x1A );
    else
    	sprintf( (char *)sms_send_message , "%d.%s%c" , messages_counter++ , "EMPTY STRING" , 0x1A );
    HAL_UART_Transmit( &huart1 , (uint8_t *)sms_send_message ,strlen( (const char *)sms_send_message ) , UART_TIMEOUT ); 
    HAL_UART_Receive(  &huart1 , sms_send_message , 20 , UART_TIMEOUT );
}

void sim5360e_read_received_message( char *received_data )
{
    uint8_t response[257];

    HAL_UART_Transmit( &huart1 , (uint8_t *)AT_CMGRD_COMMAND , 12  , UART_TIMEOUT );
    HAL_UART_Receive(  &huart1 , response                    , 256 , UART_TIMEOUT );
    //parse_response( (char *)operator_selection );
    //strcpy( gsm_information.operator_selection , &response[1][0] );
    strcpy( received_data , (char *)response );    
    return;
}

static void enable_sms_message_receive_interrupt(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    __HAL_RCC_GPIOE_CLK_ENABLE();
  
    /* Configure PE2 pin as input interrupt */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( GPIOE , &GPIO_InitStructure );
    HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);  
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

//char phone_number[] = "0544868213";
char phone_number[] = "0523969145";
//char phone_number[] = "0559404581";
//char sms_message[] = "HAIM - I'M THE ROBOT - OFER IS WAITING FOR RESPONSE";
char sms_message[] = "SHARONI HACHMUDA";
void sim5360e_initialization()
{
    power_on_sequence();
    HAL_Delay(12000);
    sim5360_disable_command_echo();
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_read_select_te_character_set(); // CSCS = "GSM"
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_network_registration();         // CREG
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_operator_selection();           // COPS
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    //sim5360e_read_information();             // ATI
    //HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_select_sms_message_format();    // CMGF
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_set_ri_interrupt();             // CFGRI
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_enable_the_function_for_the_special_gpio(); // CGFUNC
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_preferred_message_storage();
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    //sim5360e_request_international_mobile_subscriber_identify(); // CIMI
    //HAL_Delay(DELAY_BETWEEN_COMMANDS);
    //sim5360e_sms_service_centre_address();   // CSCA
    //HAL_Delay(DELAY_BETWEEN_COMMANDS);
    ////////////////////////////////////////////////////
    /*sim5360e_read_simcom_information();      // SIMCOMATI
    HAL_Delay(DELAY_BETWEEN_COMMANDS);*/
//#if OFER
    sim5360e_read_cpin();                    // CPIN?
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_read_signal_quality();          // CSQ
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_network_registration_status();  // CREG?  
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_phone_functionality_status();   // CFUN?  
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sim5360e_operator_selection_status();    // COPS?
    HAL_Delay(DELAY_BETWEEN_COMMANDS);//*/
//#endif
    sim5360e_phone_functionality_command((uint8_t *)AT_CFUN_COMMAND_1);  // CFUN=1
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    enable_sms_message_receive_interrupt();
    HAL_Delay(DELAY_BETWEEN_COMMANDS);
    sms_message_reception_interrupt_enable();
    //HAL_Delay(DELAY_BETWEEN_COMMANDS);
    //sim5360e_send_sms_message( phone_number , sms_message );
    return;
}
