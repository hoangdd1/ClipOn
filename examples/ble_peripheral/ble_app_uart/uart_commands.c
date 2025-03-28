/*---------------------------------------------------------------------------------------*
 *  G.T.E technology                                                                     *
 *                                                                                       *
 *  PROJECT   :  Water Sense                                                             *
 *  File name :                                                                          *
 *  Abstract  :                                                                          *
 *  Writen by : Zvika Grinberg                                                           *
 *  Date      : Mars 2020                                                                *
 *---------------------------------------------------------------------------------------*/



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
//#include "fsl_debug_console.h"
//#include "board.h"
//#include "fsl_pdb.h"
//#include "fsl_adc16.h"
//#include "fsl_edma.h"
//#include "fsl_sim.h"   
//#include "pin_mux.h"
//#include "clock_config.h"   
//#include "datastruct_stack.h"
//#include "global_timer.h"
#include "i2c_bus.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "hitless_data.h"
#include "nrf_calendar.h"
#include "uart_commands.h"
#include "boards.h"

extern void push_event(uint8_t *str_event , int16_t str_event_len);
extern void power_on_reset();
typedef void    (*UART_MONITOR_COMMAND_FUNCTION_POINTER)();
typedef uint8_t (*UART_SET_COMMAND_FUNCTION_POINTER)(char * , char * , int *);
extern void nrf_delay_ms_wdt(uint32_t x_msec);

typedef struct 
{
   char          *command_message;
   void          *UART_command_function_ptr;
} UART_COMMANDS;



uint8_t UART_access_system_information(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_read_sensor(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_VALVE_OPEN(char *parameter_pointer ,char * output_msg , int * output_length);                
//uint8_t UART_access_VALVE_ON(char *parameter_pointer ,char * output_msg , int * output_length);                
uint8_t UART_access_VALVECAP_ON(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_device_mode(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_server_rcv_ack(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_sysuptime(char *parameter_pointer, char * output_msg , int * output_length);
uint8_t UART_access_date(char *parameter_pointer, char * output_msg , int * output_length);
uint8_t UART_access_battery_level(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_modem_test(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_keep_alive_SMS_interval(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_finish_session_SMS(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_phone_number(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_set_detection_limit(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_set_alert_limit(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_get_detection_result(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_get_unusage_result(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_get_leakage_result(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_accees_auto_drip_test(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_flow_controller(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_accees_stop_pizuz_detect(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_change_sign(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_upload_data(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_buffer_size(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_reset_reason(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_acsses_ADC(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_acsses_cellular_net(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_acsses_params_calibrat(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_acsses_test_limit(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_acsses_leak_status(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_reset_ti(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_network_operator(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_system_dfu(char *parameter_pointer ,char * output_msg , int * output_length);
uint8_t UART_access_sensor_read(char *parameter_pointer ,char * output_msg , int * output_length);//shir
uint8_t UART_access_WD_reset(char *parameter_pointer ,char * output_msg , int * output_length);//shir


UART_COMMANDS uart_commands[] =// SY RD OP ON CP MO MT AC CK DT BT KP PN AL DL DR UR FC CT SD CS UD BF RR AD CN FN PC TL LS
{//               {STRING, {              COMMAND_TYPE              } ,
  /* SY       */ { "SY" , (void *)&UART_access_system_information   } ,
  /* RD       */ { "RD" , (void *)&UART_access_read_sensor          } ,
  /* OP       */ { "OP" , (void *)&UART_access_VALVE_OPEN           } ,  
//  /* ON       */ { "ON" , (void *)&UART_access_VALVE_ON           } ,
  /* CP       */ { "CP" , (void *)&UART_access_VALVECAP_ON          } ,
  /* DF       */ { "DF" , (void *)&UART_access_system_dfu           } ,  
  /* MO       */ { "MO" , (void *)&UART_access_device_mode          } ,
  /* MT       */ { "MT" , (void *)&UART_access_modem_test           } ,  
  /* AC       */ { "AC" , (void *)&UART_access_server_rcv_ack       } , 
  /* CK       */ { "CK" , (void *)&UART_access_sysuptime            } ,
  /* DT       */ { "DT" , (void *)&UART_access_date                 } ,
  /* BT       */ { "BT" , (void *)&UART_access_battery_level        } , 
  /* KP       */ { "KP" , (void *)&UART_keep_alive_SMS_interval     } ,
  /* PN       */ { "PN" , (void *)&UART_access_phone_number         } ,//shir 
  /* AL       */ { "AL" , (void *)&UART_access_set_alert_limit      } ,//shir  
  /* DL       */ { "DL" , (void *)&UART_access_set_detection_limit  } ,//shir  
  /* DR       */ { "DR" , (void *)&UART_access_get_detection_result } ,//shir 
  /* UR       */ { "UR" , (void *)&UART_access_get_unusage_result   } ,//shir 
  /* LR       */ { "LR" , (void *)&UART_access_get_leakage_result   } ,//shir 
  /* FC       */ { "FC" , (void *)&UART_access_flow_controller      } ,//shir
  /* CT       */ { "CT" , (void *)&UART_accees_auto_drip_test       } ,//shir
  /* SD       */ { "SD" , (void *)&UART_accees_stop_pizuz_detect    } ,//shir
  /* CS       */ { "CS" , (void *)&UART_access_change_sign          } ,//shir
  /* UD       */ { "UD" , (void *)&UART_access_upload_data          } ,//shir
  /* BF       */ { "BF" , (void *)&UART_access_buffer_size          } ,//shir
  /* RR       */ { "RR" , (void *)&UART_access_reset_reason         } ,//shir
  /* AD       */ { "AD" , (void *)&UART_acsses_ADC                  } ,//shir
  /* CN       */ { "CN" , (void *)&UART_acsses_cellular_net         } ,//shir
  /* PC       */ { "PC" , (void *)&UART_acsses_params_calibrat      } ,//shir
  /* TL       */ { "TL" , (void *)&UART_acsses_test_limit           } ,//shir
  /* LS       */ { "LS" , (void *)&UART_acsses_leak_status          } ,//shir
  /* RT       */ { "RT" , (void *)&UART_access_reset_ti             } ,//shir
  /* NO       */ { "NO" , (void *)&UART_access_network_operator     } ,//shir
  /* FN       */ { "FN" , (void *)&UART_finish_session_SMS          } ,
  /* SR       */ { "SR" , (void *)&UART_access_sensor_read          } ,//shir
  /* WD       */ { "WD" , (void *)&UART_access_WD_reset             }  //shir
};

int insert_sysuptime(char * input_string)
{
        int ret_val = 0;
        uint32_t sysuptime_temp;
        sysuptime_temp = atol(input_string);

        if( sysuptime_temp > 1512401470/* 04-12-2017 */ && sysuptime_temp < 4070951606 /* 01-01-2099 */ )
        {
          nrf_cal_set_UNIX_timestemp ( sysuptime_temp ) ;          
          ret_val = 1;
        }
        return ret_val;
}
char print_state[128];
extern uint32_t nus_send_data_buff(uint8_t *send_buff, int send_buff_length);
/* SR COMMAND FOR MANAGE THE TI COMPONENT CLIBRATE-READ-ENABLE-DISABLE */
uint8_t UART_access_sensor_read(char *parameter_pointer, char * output_msg , int * output_length)
{ 
    int rcv_interval = 0;  
    if(parameter_pointer != NULL)
    {
          rcv_interval = atoi(parameter_pointer);
       
          if ( rcv_interval >= 1 && rcv_interval <= 111 ){
            if ( rcv_interval == 1 ){//All the actions in one command.
             
                /* Print data via UART */
               sprintf(print_state,"calibrate!");
               nus_send_data_buff((uint8_t *)print_state,strlen(print_state));
               StartTiRead();//enable
               /* Print data via UART */
               sprintf(print_state,"enable!");
               nus_send_data_buff((uint8_t *)print_state,strlen(print_state));
               nrf_delay_ms(100);
               for( int j=0 ; j < 6 ; j++ ){
                 TiReadData_byI2c_command();//reading and print by uart.
                 nrf_delay_ms_wdt(125);
                }
                StopTiRead();//disable
                 /* Print data via UART */
                sprintf(print_state,"disable!");
                nus_send_data_buff((uint8_t *)print_state,strlen(print_state));
                sprintf(output_msg+2," READ DONE!\r\n");
            }
            if ( rcv_interval == 2 ){//CLIBRATE
                Ti_calibrate();
                hitless_ptr()->initTIDone ? sprintf(output_msg+2," CALIBRATION DONE\r\n") : sprintf(output_msg+2," CALIBRATION FAILED\r\n");
            }
             if ( rcv_interval == 4 ){//ENABLE
              StartTiRead();
              sprintf(output_msg+2," ENABLE DONE\r\n");
            }
            if ( rcv_interval == 5 ){//DISABLE
              StopTiRead();
              sprintf(output_msg+2," DISABLE DONE\r\n");
            }
            if ( rcv_interval >= 6 && rcv_interval <= 100 ){//READ 
              for( int j=0 ; j < rcv_interval ; j++ ){
                 TiReadData_byI2c_command();
                 nrf_delay_ms_wdt(250);
              }
              sprintf(output_msg+2," READ DONE\r\n");
            }
            if ( rcv_interval == 101 ){//read from TI - Firmware Version
              Firmware_Version();
              sprintf(output_msg+2," Firmware Version\r\n");
            }
            if ( rcv_interval == 102 ){//Put the TI into BSL 
              Bootloader_Mode();
              sprintf(output_msg+2," Bootloader Mode\r\n");
            }
            if ( rcv_interval == 103 ){//Delete flash - when TI in the BSL
               Mass_Erase_1();
               sprintf(output_msg+2," Erase Mass_1\r\n");
            }
            if ( rcv_interval == 104 ){//get the response of Sending Password to Open the BSL 
              int response = Password_to_BSL_2();
              sprintf(output_msg+2," Password to BSL_2[%d]\r\n",response);
            }
            if ( rcv_interval == 105 ){//Executing New/Current Firmware - when TI in the BSL
              Executing_Firmware();
              sprintf(output_msg+2," Executing Firmware\r\n");
            }
            if ( rcv_interval == 106 ){//Reading the BSL Version
              Read_BSL_Version();
              sprintf(output_msg+2," Read BSL Version\r\n");
            }
            if ( rcv_interval == 107 ){//Delete flash - when TI in the BSL
              int response = Mass_Erase_2();
              sprintf(output_msg+2," Erase Mass_2 [%d]\r\n",response);
            }
             if ( rcv_interval == 108 ){//HW enter TI to the BSL mode
              enter_BSL_Mode_by_HW();
              sprintf(output_msg+2," HW enter BSL\r\n");
            }
            if ( rcv_interval == 109 ){//Sending Password to Open the BSL 
              Password_to_BSL_1();
              sprintf(output_msg+2," Password to BSL_1\r\n");
            }
           if ( rcv_interval == 110 ){//Sending specipic Password to Open the BSL 
              Password_to_BSL_1_original();
              sprintf(output_msg+2," Password to BSL_1_origin\r\n");
            }
           if ( rcv_interval == 111 ){//write the data struct to bsl.  
              write_DataStruct();
              sprintf(output_msg+2," Write Data\r\n");
            }
          }
          else 
            sprintf(output_msg+2," FAIL\r\n");
    }
    else
    {
          sprintf(output_msg+2," FAIL\r\n");
    }
    *output_length = strlen(output_msg);
    return 1;  
}
extern void TI_reset();
/* TI reset pin MSP430_RES(1,11) */
uint8_t UART_access_reset_ti(char *parameter_pointer ,char * output_msg , int * output_length)
{
    TI_reset();
    sprintf(output_msg+2 , " RESET TI");
    *output_length = strlen(output_msg);
    return 1;
}

void OP(char x)
{
    if ( x == '0')
    {
       nrf_gpio_pin_clear(VALVE_OPEN);
       nrf_gpio_pin_set(VALVE_CLOSE);
    }
    else if (x == '1')
    {
       nrf_gpio_pin_set(VALVE_OPEN);
       nrf_gpio_pin_clear(VALVE_CLOSE);
    }
}

//void ON(char x)
//{
//    if ( x == '0')
//    {
//       nrf_gpio_pin_clear(VALVE_ON);
//    }
//    else if (x == '1')
//    {
//       nrf_gpio_pin_set(VALVE_ON);
//    }
//}

void CP(char x)
{
    if (x == '0')
    {
       nrf_gpio_pin_clear(VALVECAP_ON);
    }
    else if (x == '1')
    {
       nrf_gpio_pin_set(VALVECAP_ON);
    }
}
void change_water_flow(char param){//shir
#ifdef IMPROVING_EDITING_VER
    CP('1');
    nrf_delay_ms(3000);  
    OP(param);
    nrf_delay_ms(100); 
    CP('0');
    nrf_gpio_pin_clear(VALVE_OPEN);
    nrf_gpio_pin_clear(VALVE_CLOSE);
#else
    CP('1');
    nrf_delay_ms(3000);                        
    CP('0');
    nrf_delay_ms(10); 
    OP(param);
    nrf_delay_ms(10); 
    ON('1');
    nrf_delay_ms(100); 
    ON('0');
#endif
}

EXISTANCE process_uart_command(char * input_msg, int input_length , char * output_msg , int *output_length)
{
    uint8_t      command_index/*, read_command, parameters_validity = 0*/; // 0 means all parameters are valid otherwise the number of invalid parameter.
    uint8_t      number_of_commands;
    EXISTANCE return_value = NOT_EXIST;
    char     *parameter_pointer = NULL;
    int str_input_length = 0;
    //char     result_string_before_writing[40];
    //char     result_string_after_writing[40];
    char input_cmd_uppercase[3];
    str_input_length = strlen(input_msg);
    //if(input_msg[2] == '=')
    //  input_msg[2]=' ';
    
    if((str_input_length > 3) && ((input_msg[2] == ' ') || (input_msg[2] == '='))|| (str_input_length == 2))
    {        
        number_of_commands = ( sizeof(uart_commands) / sizeof(UART_COMMANDS) );
	for (command_index = 0 ; command_index < number_of_commands ; command_index++)        {
            input_cmd_uppercase[0] = toupper(input_msg[0]);
            input_cmd_uppercase[1] = toupper(input_msg[1]);
            if (strncmp(input_cmd_uppercase, uart_commands[command_index].command_message , 2) == 0)
		break;
        }
	if (command_index < number_of_commands)
        {  // command is found - execute it.
            sprintf(output_msg, uart_commands[command_index].command_message);
                                     
            
            if( input_msg[3] != '?' && input_msg[3] != '\0' && input_msg[3] != '\r' && input_msg[3] != '\n' )
              parameter_pointer = (char *)&input_msg[3];

      
             uart_commands[command_index].UART_command_function_ptr != NULL ? (*(UART_SET_COMMAND_FUNCTION_POINTER)uart_commands[command_index].UART_command_function_ptr)(parameter_pointer,output_msg,output_length) : 1; 
            //;//printf("%1s", output_msg );
               
        return_value = EXIST;
	}
    }
    return(return_value);
}

uint32_t day,month,year,hour,min,sec;  
uint8_t UART_access_date(char *parameter_pointer, char * output_msg , int * output_length)
{   
    if(parameter_pointer != NULL)
    {            
        //sscanf( parameter_pointer,"%d/%d/%d %d:%d:%d", &day, &month, &year, &hour, &min, &sec);
        //year+=2000;
        //if ( year >= 2020 && month <=12 && day <=31 && hour <=12 && min <=60 && sec <=60 )
        //{         
        //  nrf_cal_set_time(year, month, day, hour, min, sec);          
        //  sprintf(output_msg, "DT OK\r\n");
        //}
    }
    else
    {
      output_msg[2] = ' ';
      nrf_cal_get_time_string(0,&output_msg[3]);    
    }
    
  *output_length = strlen(output_msg);
  return 1;  
}


uint8_t UART_access_sysuptime(char *parameter_pointer, char * output_msg , int * output_length)
{   
    if(parameter_pointer != NULL)
    {
        if (insert_sysuptime(parameter_pointer))
          sprintf(output_msg, "CK OK\r\n");
        else
          sprintf(output_msg,"CK %ld",nrf_cal_get_UNIX_timestemp());

    }
    else
    {
      sprintf(output_msg,"CK %ld",nrf_cal_get_UNIX_timestemp());
    }
    
  *output_length = strlen(output_msg);
  return 1;  
}

uint8_t UART_finish_session_SMS(char *parameter_pointer, char * output_msg , int * output_length)
{   
  sys_info.finish_sms_session = 1;
  sprintf(output_msg, "FN OK\r\n");
  *output_length = strlen(output_msg);
  return 1;  
}

uint8_t UART_keep_alive_SMS_interval(char *parameter_pointer, char * output_msg , int * output_length)
{   
    int rcv_interval = 0;
    if(parameter_pointer != NULL)
    {
        rcv_interval = atoi(parameter_pointer);
        if (rcv_interval > 60 && rcv_interval < 40000 )
        {
          hitless_ptr()->message_coutdown_interval = rcv_interval;
          hitless_ptr()->check_message_coutdown = rcv_interval;
          sprintf(output_msg, "KP OK\r\n");
        }
        else if ( rcv_interval == 0 )
        {
           hitless_ptr()->check_message_coutdown = 10;
           hitless_ptr()->device_state = INIT_SLEEP_STATE;
           sprintf(output_msg, "KP TEST\r\n");
           *output_length = strlen(output_msg);
           return 1;  
        }
        else
        {
          sprintf(output_msg,"KP %d(%d)",hitless_ptr()->message_coutdown_interval,(hitless_ptr()->check_message_coutdown < 0 )? 0 :hitless_ptr()->check_message_coutdown);
        }

    }
    else
    {
      sprintf(output_msg,"KP %d(%d)",hitless_ptr()->message_coutdown_interval,(hitless_ptr()->check_message_coutdown < 0 )? 0 :hitless_ptr()->check_message_coutdown );
    }
    
  *output_length = strlen(output_msg);
  return 1;  
}


uint8_t UART_access_modem_test(char *parameter_pointer ,char * output_msg , int * output_length)
{
  hitless_ptr()->device_state = PRE_SENDING_VIA_CELLULAR_MODEM;   
  sprintf(output_msg+2,"OK\r\n");
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_access_server_rcv_ack(char *parameter_pointer ,char * output_msg , int * output_length)
{
  sprintf(output_msg+2,"AC\n%s\r\n",hitless_ptr()->BLE_main_task_countdown == 0 ? "BT INTERVAL LOW" : "BT INTERVAL HIGHT" );//shir add the BT value for debuging
  *output_length = strlen(output_msg);
  return 1;
}
bool update_msg = 0;
uint8_t UART_access_system_information(char *parameter_pointer ,char * output_msg , int * output_length)
{
  char commu_protocol[4];
  char modem[2];
  char sensor[16];
  
 // LEDS_ON(BSP_LED_1_MASK);
 // LEDS_ON(BSP_LED_0_MASK);
  
#ifdef TCP_COMMU
   sprintf(commu_protocol,"TCP"); 
#else
   sprintf(commu_protocol,"UDP"); 
#endif
#if defined(SIMCOM7080G) || defined(SIMCOM7670)
   sprintf(modem,"G4"); 
#else
   sprintf(modem,"G2"); 
#endif
#ifdef MAX3510_SENSOR
   sprintf(sensor,"MAX3510"); 
#elif TI_SENSOR
   sprintf(sensor,"TI_SENSOR[%d]",hitless_ptr()->ti_sensor_status);
#else  
   sprintf(sensor,"NO_SENSOR");
#endif
   sprintf(output_msg+2,"\r\nHW_Ver: %s\r\nSW_Ver: %s\r\nModem: %s-%s\r\nSensor: %s\r\nSMS Tx[%d]Rx[%d]\r\n" ,HW_VERSION ,VERSION ,modem ,commu_protocol ,sensor, hitless_ptr()->messages_counter ,hitless_ptr()->messages_counter_rx); 
  *output_length = strlen(output_msg);
  return 1;
}
extern int ble_device_during_session;

uint8_t UART_access_device_mode(char *parameter_pointer ,char * output_msg , int * output_length)
{
    uint8_t parameters_validity = 0;
    char device_temp_mode[6];
    
    if(hitless_ptr()->device_state == SENDING_VIA_CELLULAR_MODEM)//for disable MO_0,MO_1 command by SMS.
    { 
       sprintf(output_msg, "MO Disable Via Modem");  
       *output_length = strlen(output_msg); 
       return 1;
    }
    if(parameter_pointer != NULL)
    {
          if (parameter_pointer[0] == '0')
          {                   
               hitless_ptr()->device_state = SLEEP_STATE;
               //power modem off
               nrf_gpio_pin_clear(MODEM_POWER_ON);
               nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
               nrf_gpio_pin_clear(MODEM_RESET);
               ble_device_during_session = 1;
               NVIC_SystemReset ();
          }
          else if (parameter_pointer[0] == '1')
          {
              
              hitless_ptr()->device_state = WORKING_DEVICE_STATE;
              ble_device_during_session = 1;            
              NVIC_SystemReset ();
          }
    }
    
    sprintf(device_temp_mode , "sleep\0");
    if ( hitless_ptr()->device_state == INIT_WORKING_DEVICE_STATE || hitless_ptr()->device_state == WORKING_DEVICE_STATE )
        sprintf(device_temp_mode ,"work\0");
            
    sprintf(output_msg+2," %s mode\r\n",device_temp_mode);  
    *output_length = strlen(output_msg);
    return parameters_validity;
}
  
uint8_t UART_access_battery_level(char *parameter_pointer ,char * output_msg , int * output_length)
{
//    float temp = (float)hitless_ptr()->vbat;
//    sprintf(output_msg+2," %.2f[V] (%d), USB (%d), VDD (%.2f)",((float)hitless_ptr()->vbat/10),hitless_ptr()->vbat, hitless_ptr()->v_usb, (float)hitless_ptr()->v_vdd/100);
      sprintf(output_msg+2," vdd : %d [V]\r\n",hitless_ptr()->v_vdd);  

    *output_length = strlen(output_msg);
    return 1;
}

uint8_t UART_access_system_dfu(char *parameter_pointer ,char * output_msg , int * output_length)
{
  int rcv_var = 0;
  if (parameter_pointer != NULL)
  {    
      rcv_var = atoi(parameter_pointer);
      if(rcv_var == 1233)
      {
        hitless_ptr()->DFU_flag = 1232;        
        sprintf(output_msg+2,"\r\n DF OK" );       
      }
  }
  else
  {
    NVIC_SystemReset(); 
  }  
  *output_length = strlen(output_msg);
  return 1;
}


uint8_t UART_access_read_sensor(char *parameter_pointer ,char * output_msg , int * output_length)
{
   
    uint8_t parameters_validity = 0;

    if(parameter_pointer != NULL)
    {
          if (parameter_pointer[0] == '0')
          { 
             ble_device_during_session = 0;
          }
          else if (parameter_pointer[0] == '1')
          {              
              ble_device_during_session = 1;
          }
    }

    sprintf(output_msg+2, " %d\r\n",ble_device_during_session);  
    *output_length = strlen(output_msg);
    return parameters_validity;
}
                
uint8_t UART_access_VALVE_OPEN(char *parameter_pointer ,char * output_msg , int * output_length)
{
   
    uint8_t parameters_validity = 0;

    if(parameter_pointer != NULL )
    {
/*          
    CP-1 , ValveCap_On-'1' P1_10
    Wait for 3 Sec.
    OP-1 OR OP-0 control 
    CP-0 , ValveCap_On-'0' P1_10
    Wait for 10mSec
    ON-1 , VALVE_ON-'1' P1_11
    Wait for 100mSec
    ON-0 , VALVE_ON-'0' P1_11
*/     
          if ( parameter_pointer[0] == '0' || parameter_pointer[0] == '1')
          {         
              OP(toupper(parameter_pointer[0]));
          }    
          else if ( (toupper(parameter_pointer[0]) == 'S') &&
                ( parameter_pointer[1] == '0' || parameter_pointer[1] == '1'))
          {
            change_water_flow(parameter_pointer[1]);
          }          
    }

    sprintf(output_msg+2, " %d\r\n",nrf_gpio_pin_out_read(VALVE_OPEN));  
    *output_length = strlen(output_msg);
    return parameters_validity;
}
                
//uint8_t UART_access_VALVE_ON(char *parameter_pointer ,char * output_msg , int * output_length)
//{
//   
//    uint8_t parameters_validity = 0;
//
//    if(parameter_pointer != NULL)
//    {
//          ON(parameter_pointer[0]);
//    }
//
//    sprintf(output_msg+2, " %d\r\n",nrf_gpio_pin_out_read(VALVE_ON));  
//    *output_length = strlen(output_msg);
//    return parameters_validity;
//}
  
                
uint8_t UART_access_VALVECAP_ON(char *parameter_pointer ,char * output_msg , int * output_length)
{
   
    uint8_t parameters_validity = 0;

    if(parameter_pointer != NULL)
    {
          CP(parameter_pointer[0]);
    }

    sprintf(output_msg+2, " %d\r\n",nrf_gpio_pin_out_read(VALVECAP_ON));  
    *output_length = strlen(output_msg);
    return parameters_validity;
}
  

uint8_t UART_access_phone_number(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    {
      sprintf( hitless_ptr()->phone_number_of_administrator, parameter_pointer);
      sprintf(output_msg,"PN OK\r\n");
    }
    else
    {
      sprintf( output_msg,"PN %s", hitless_ptr()->phone_number_of_administrator );
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_access_set_alert_limit(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    { 
      if(atoi(parameter_pointer) <= 15 && atoi(parameter_pointer) > 0 ){
        hitless_ptr()->unusage_limit= atof(parameter_pointer);
        sprintf(output_msg,"AL OK\r\n");
      }
      else{
         sprintf(output_msg,"AL FAIL\r\n");     
      }
    }
    else
    {
      sprintf( output_msg,"AL %.2f", hitless_ptr()->unusage_limit );
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_access_set_detection_limit(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    {
       if( atoi(parameter_pointer) >= 20 ){
        hitless_ptr()->detection_limit= atof(parameter_pointer);
        sprintf(output_msg,"DL OK\r\n");
       }
       else{
         sprintf(output_msg,"DL FAIL\r\n");     
      }
    }
    else
    {
      sprintf( output_msg,"DL %.2f", hitless_ptr()->detection_limit );
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_access_get_unusage_result(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    {
      hitless_ptr()->unusage_limit_counter[0]= atoi(parameter_pointer);
      sprintf(output_msg,"UR OK\r\n");
    }
    else
    {
      sprintf( output_msg,"UR [%d/%d] next time:[%d]", hitless_ptr()->unusage_limit_counter[1], hitless_ptr()->unusage_limit_counter[0], hitless_ptr()->detect_pizuz_flags[0]);
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_access_get_leakage_result(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    {
      hitless_ptr()->leak_counter[0] = atoi(parameter_pointer);
      sprintf(output_msg,"LR OK\r\n");
    }
    else
    {
      sprintf( output_msg,"LR [%d/%d] next time:[%d]", hitless_ptr()->leak_counter[1], hitless_ptr()->leak_counter[0], hitless_ptr()->detect_pizuz_flags[0]);
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_access_get_detection_result(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
    
    if(parameter_pointer != NULL)
    {
      hitless_ptr()->limit_count_result[0]= atof(parameter_pointer);
      sprintf(output_msg,"DR OK\r\n");
    }
    else
    {
      sprintf( output_msg,"DR [%.0f/%.0f] next time:[%d][%d]", hitless_ptr()->limit_count_result[1], hitless_ptr()->limit_count_result[0], hitless_ptr()->detect_pizuz_flags[0],hitless_ptr()->detect_pizuz_flags[1]);
    }
    *output_length = strlen(output_msg);
    return 1;  
}

uint8_t UART_accees_auto_drip_test(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
   if(parameter_pointer != NULL)
    {
      if (hitless_ptr()->send_critical_msg != 0 )
      {
        sprintf( output_msg,"CT Canceled following an explosion");
      }
      else if (atoi(parameter_pointer) == 0)//shir cancelled the timer option, Because of the bug.
      {
         hitless_ptr()->drip_test_stages = atoi(parameter_pointer); 
         //MO_0
         hitless_ptr()->device_state = SLEEP_STATE;
         //power modem off
         nrf_gpio_pin_clear(MODEM_POWER_ON);
         nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
         nrf_gpio_pin_clear(MODEM_RESET);
         NVIC_SystemReset ();
      }
      else
      {
         sprintf( output_msg,"CT FAIL\r\n");
      }
    }
    else
    {
       sprintf( output_msg,"CT %d",hitless_ptr()->drip_test_stages);
    }
    *output_length = strlen(output_msg);
     return 1;  
    
}

uint8_t UART_access_flow_controller(char *parameter_pointer ,char * output_msg , int * output_length)//shir
{
   uint8_t parameters_validity = 0;
   if( parameter_pointer != NULL )
    {
      if ( (parameter_pointer[0] == '0' || parameter_pointer[0] == '1' ) && parameter_pointer[1] == '\0' )
          {    
              hitless_ptr()->flow_controller = atoi(parameter_pointer);
              hitless_ptr()->device_state = WORKING_DEVICE_STATE;//MO_1
              NVIC_SystemReset ();                           
           }
           else
           {
             hitless_ptr()->flow_controller = -1;
           }          
    }
    sprintf(output_msg+2, " %d\r\n", hitless_ptr()->flow_controller);  
    *output_length = strlen(output_msg);
    return parameters_validity;
}

uint8_t UART_accees_stop_pizuz_detect(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
   if(parameter_pointer != NULL)
   {
      if(atoi(parameter_pointer) >= 0)
      {
          hitless_ptr()->detect_pizuz_flags[2]=(atoi(parameter_pointer)*60);//For x minutes the explosion detection will shut down.
          if ( hitless_ptr()->critic_msg[1] == '1' && hitless_ptr()->send_critical_msg >= 1 )//if the system going stop the flow.
          {
            hitless_ptr()->send_critical_msg =0;//Canceling the water closure.
            hitless_ptr()->check_message_coutdown=300;//the next modem session.
          }
          sprintf(output_msg,"SD OK\r\n");  
      }
      else
      {
         sprintf(output_msg,"SD FAIL\r\n");     
      }
   }
   else
   {
       sprintf( output_msg,"SD %d",(hitless_ptr()->detect_pizuz_flags[2]/60));
   }
    *output_length = strlen(output_msg);
     return 1;  
}

uint8_t UART_access_change_sign(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
  if(parameter_pointer != NULL)
    {
      if(atoi(parameter_pointer) == 1 || atoi(parameter_pointer) == -1 )
      {
          hitless_ptr()->change_flow_sign= atoi(parameter_pointer);//change the sign of Flow data
          sprintf(output_msg,"CS OK\r\n");
      }
      else
         sprintf(output_msg,"CS FAIL\r\n");  
    }
  else
    {
      sprintf(output_msg,"CS %d\r\n",hitless_ptr()->change_flow_sign);  
    }
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_access_upload_data(char *parameter_pointer ,char * output_msg , int * output_length){//shir

  if(parameter_pointer != NULL)
   {
      if(atoi(parameter_pointer) == 1 )
      {
          hitless_ptr()->upload_data[0]=1;   //if the buffer not empty -> will upload data to the cloud once.
          sprintf(output_msg,"UD OK\r\n");
      }
      else if(atoi(parameter_pointer) >= 280 && atoi(parameter_pointer) <= 3000  )
      {
          hitless_ptr()->upload_data[0]=atoi(parameter_pointer);     //if the buffer not empty -> will upload data to the cloud every x seconds.
          hitless_ptr()->upload_data[1]=atoi(parameter_pointer); 
          sprintf(output_msg,"UD OK\r\n");
      }
      else if(atoi(parameter_pointer) == 0 )
      {
          hitless_ptr()->upload_data[0]=0;   //if the buffer not empty -> will upload data to the cloud once.
          hitless_ptr()->upload_data[1]=0;
          sprintf(output_msg,"UD OK\r\n");
      }
      else
      {
         sprintf(output_msg,"UD FAIL\r\n");  
      }
   }
  else
   {
     sprintf(output_msg,"UD [%d|%d]\r\n", hitless_ptr()->upload_data[0],hitless_ptr()->upload_data[1]);  
   }
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_access_buffer_size(char *parameter_pointer ,char * output_msg , int * output_length){//shir

    sprintf(output_msg+2," [%d/%d]", hitless_ptr()->event_buff.event_buffer_index, NUMBER_OF_EVENTS );
    *output_length = strlen(output_msg);
    return 1;
}

uint8_t UART_access_reset_reason(char *parameter_pointer ,char * output_msg , int * output_length){//shir
 
    char temp[20];
    switch(hitless_ptr()->reset_reason){
      case 0:
        sprintf(temp,"POWER_ON_RESET");
        break;
      case 1:
        sprintf(temp,"WATCHDOG_RESET");
        break;
      case 2:
        sprintf(temp,"EXTERNAL_RESET");
        break;
      case 3:
        sprintf(temp,"SOFTWARE_RESET");
        break;
      case 4:
        sprintf(temp,"WAKE_UP_RESET");
        break;
      default:
        sprintf(temp,"ERROR_RESET");
    }
 
    sprintf(output_msg+2," %s",temp);
    *output_length = strlen(output_msg);
    return 1;
}

uint8_t UART_acsses_ADC(char *parameter_pointer ,char * output_msg , int * output_length){//shir

  if(parameter_pointer != NULL)
   {
/*      if(atoi(parameter_pointer) > 2 && atoi(parameter_pointer) <= 1800)
//      {
//          hitless_ptr()->ADC = atoi(parameter_pointer); 
//          sprintf(output_msg,"AD OK\r\n");
//      }
//      else*/ 
      if(atoi(parameter_pointer) == 1 && hitless_ptr()->check_message_coutdown > 5 )
      {
          hitless_ptr()->ADC=1;  //enable ADC();
          sprintf(output_msg,"AD OK\r\n");
      }
      else if (atoi(parameter_pointer) == 0 && hitless_ptr()->check_message_coutdown > 5)
      {
         hitless_ptr()->ADC=0;  //disable ADC();  
          sprintf(output_msg,"AD OK\r\n");
      }
      else
      {
         sprintf(output_msg,"AD FAIL\r\n");  
      }
   }
  else
   {
     sprintf(output_msg,"AD %d\r\n", hitless_ptr()->ADC);  
   }
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_acsses_cellular_net(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
   if(parameter_pointer != NULL)
   {
      if( NUM_OF_NETWORKS > atoi(parameter_pointer) && atoi(parameter_pointer) >= 0 ) // 0..NUM_OF_NETWORKS
      {
          hitless_ptr()->cellular_net = (CELLULAR_NET)atoi(parameter_pointer);  
          sprintf(output_msg,"CN OK\r\n");
      }
      else
      {
         sprintf(output_msg,"CN FAIL\r\n");  
      }
   }
  else
   {
       char temp[50];
       const char *network_names[NUM_OF_NETWORKS] = {
        "MONO",
        "GOLAN",
        "BOUYG",
        "ORANGE",
        "SFR",
        "FREE",
        "PART"
       };
       
       sprintf(output_msg,"CN [%s]\r\nValues:",network_names[hitless_ptr()->cellular_net]);
       
       for (int i = 0; i < NUM_OF_NETWORKS; i++) {
         snprintf(temp, sizeof(temp), "\r\n(%d)%s", i, network_names[i]);
         strncat(output_msg, temp, sizeof(output_msg) - strlen(output_msg) - 1);
       }
        
   }
  *output_length = strlen(output_msg);
  return 1;
}

//The length of the string received in an sms message is limited
uint8_t UART_acsses_params_calibrat(char *parameter_pointer ,char * output_msg , int * output_length){//shir

  if(parameter_pointer != NULL){ //received - pc ud|kp|dr|dl|tl|al|ur 

     char * parameters = strtok(parameter_pointer, "|"); // Extract the first parameter
   
     hitless_ptr()->upload_data[0] = hitless_ptr()->upload_data[1] = (parameters && ((atoi(parameters)*50 >= 280 && atoi(parameters)*50 <= 72000) || atoi(parameters)==0 )) ? atoi(parameters)*50 : hitless_ptr()->upload_data[1];//*3000/60
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->check_message_coutdown = hitless_ptr()->message_coutdown_interval = (parameters && (atoi(parameters)*55 > 60) && (atoi(parameters)*55 < 40000)) ? atoi(parameters)*55 : hitless_ptr()->message_coutdown_interval;//*3300/60
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->limit_count_result[0] = (parameters && (atof(parameters) > 0)) ? atof(parameters)*1.6 : hitless_ptr()->limit_count_result[0];
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->detection_limit = (parameters && (atof(parameters)> 0)) ? atof(parameters) : hitless_ptr()->detection_limit;
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->drip_test_limit = (parameters && (atoi(parameters)> 0)) ? atoi(parameters) : hitless_ptr()->drip_test_limit;
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->unusage_limit = (parameters && (atof(parameters)> 0)) ? atof(parameters) : hitless_ptr()->unusage_limit;
     parameters = strtok(NULL, "|");//Extract the next paremeter
     hitless_ptr()->unusage_limit_counter[0] = (parameters && (atoi(parameters)> 0)) ? atoi(parameters)*105 : hitless_ptr()->unusage_limit_counter[0];
     
       //The length of the string received in a sms message is limited:
//     hitless_ptr()->change_flow_sign = parameters &&(atoi(parameters) == 1 || atoi(parameters) == -1)? atoi(parameters) : hitless_ptr()->change_flow_sign;
//     parameters = strtok(NULL, "|");//Extract the next paremeter
//     hitless_ptr()->ADC = parameters && (atoi(parameters)==0 || atoi(parameters)==1) ? atoi(parameters) : hitless_ptr()->ADC;
//     parameters = strtok(NULL, "|");//Extract the next paremeter
//     hitless_ptr()->cellular_net = parameters && ( atoi(parameters)< NUM_OF_NETWORKS && atoi(parameters)>= 0 ) ? (CELLULAR_NET)atoi(parameters) : hitless_ptr()->cellular_net;
//     parameters = strtok(NULL, "|");//Extract the next paremeter
    
     sprintf(output_msg,"PC [%d]%d]%.0f]%.0f]%d]%.0f]%d]%d]%d]%d]%s]%s]\r\n", hitless_ptr()->upload_data[1]/50,hitless_ptr()->message_coutdown_interval/55,hitless_ptr()->limit_count_result[0]/1.6,
             hitless_ptr()->detection_limit,hitless_ptr()->drip_test_limit, hitless_ptr()->unusage_limit,hitless_ptr()->unusage_limit_counter[0]/105, hitless_ptr()->change_flow_sign,hitless_ptr()->ADC/3000,hitless_ptr()->cellular_net,HW_VERSION,VERSION);  

  }
  else{
      sprintf(output_msg,"PC [%d]%d]%.0f]%.0f]%d]%.0f]%d]%d]%d]%d]%s]%s]\r\n", hitless_ptr()->upload_data[1]/50,hitless_ptr()->message_coutdown_interval/55,hitless_ptr()->limit_count_result[0]/1.6,
             hitless_ptr()->detection_limit,hitless_ptr()->drip_test_limit, hitless_ptr()->unusage_limit,hitless_ptr()->unusage_limit_counter[0]/105, hitless_ptr()->change_flow_sign,hitless_ptr()->ADC/3000,hitless_ptr()->cellular_net,HW_VERSION,VERSION);  

  }
  
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_acsses_test_limit(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
   if( parameter_pointer != NULL ){
     if ( atoi(parameter_pointer)> 0 ){
        hitless_ptr()->drip_test_limit = atoi(parameter_pointer);
        sprintf(output_msg,"TL OK\r\n");
     }
     else 
       sprintf(output_msg,"TL FAIL\r\n");
  }
  else{
      sprintf(output_msg,"TL %d\r\n" , hitless_ptr()->drip_test_limit);
   }
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_acsses_leak_status(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
   if( parameter_pointer != NULL )
       sprintf(output_msg,"LS FAIL\r\n");
  else
      sprintf(output_msg,"LS %d\r\n" , hitless_ptr()->leak_status);
   
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_access_network_operator(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
   if( parameter_pointer != NULL )
       sprintf(output_msg,"NO FAIL\r\n");
  else
    sprintf(output_msg,"NO\r\nOperator: %s\r\nSignal Quality:(%d)\r\n" , hitless_ptr()->network_operator, hitless_ptr()->modem_signal_quality);
   
  *output_length = strlen(output_msg);
  return 1;
}

uint8_t UART_access_WD_reset(char *parameter_pointer ,char * output_msg , int * output_length){//shir
  
  if( parameter_pointer != NULL && atoi(parameter_pointer) == 1 ){
    for(;;)//for WD reset the device 
    sprintf(output_msg,"WD OK\r\n");
  }
  else
     sprintf(output_msg,"WD FAIL\r\n");
 
  *output_length = strlen(output_msg);
  return 1;
}