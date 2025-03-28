#include "modem.h"
#include "general.h"
#include "system_info.h"
#include "hitless_data.h"
#include "message_handler.h"
#include "uart_commands.h"
#include "nrf_calendar.h"
#include <stdlib.h>

char modem_msg[MODEM_RX_BUF_SIZE];
int modem_counter;
extern void set_phone_number_of_sender( char *international_phone_number_of_sender );
extern void reset_uart_rx_buff();
extern void gets_uart_rx_buff(char * ret_buf);
extern int gets_uart_rx_buff_without_clr(char * ret_buf);
extern void wdt_feeding(void);
extern void wrap_NVIC_SystemReset();
static bool Modem_CheackResult( char *pStr, uint16_t Timeout );

extern void stam_delay_100mSec( uint16_t multi_y);

char *cellular_networks_APN[NUM_OF_NETWORKS] = { 
  "data.mono",                      //Monogoto
  "internet.golantelecom.net.il",   //Golan
  "mmsbouygtel.com",                //Bouygues Telecom- tzarfat
  "orange",                         //Orange- tzarfat
  "sl2sfr",                         //SFR- tzarfat
  "free",                           //Free Mobile- tzarfat
  "uinternet",                      //Partner

//  "internet.pelephone.net.il",      //Pelephone 3G 
//  "net.hotm",                       //HOT Mobile
//  "internetg",                      //Cellcom 3G
//  "internet.rl",                    //Rami Levy Internet 
//  "uwap.orange.co.il"               //012 Mobile
};

char *setup_commands[9] = {
    "AT+CPIN?\r\n",
#if SIMCOM7670     
    "AT+CEREG=1\r\n",
#endif //SIMCOM7670     
    "AT+CEREG?\r\n",
#ifdef SIMCOM7080G    
    "AT+CMEE=1\r\n",
    "AT+CIPSHUT\r\n",
    "AT+CGATT=1\r\n",
    "AT+CSTT\r\n", //AT+CGDCONT=1,"IP","data.mono"
    "AT+CIICR\r\n",
    "AT+CIFSR\r\n",
    "AT+CIPSTART=\"UDP\",\"ec2-34-207-11-203.compute-1.amazonaws.com\",\"20001\"\r\n"
#elif SIMCOM7670 
    "AT+CNMI=2,1,0,0,0\r\n",    
    "AT+CGACT=1,1\r\n",
    "AT+CGDCONT=1,\"IP\",\"%s\"\r\n",
    "AT+CIPMODE=0\r\n", // Set TCP/IP Application Mode Non transparent mode
    "AT+CIPTIMEOUT=3000,3000,3000\r\n"
    "AT+NETOPEN\r\n",
    "AT\r\n",
    //"AT\r\n"      
#else
    "AT+CMEE=1\r\n",
    "AT+CIPSHUT\r\n",
    "AT+CGATT=1\r\n",
    "AT+CSTT\r\n", //AT+CGDCONT=1,"IP","data.mono"
    "AT+CIICR\r\n",asda
    "AT+CIFSR\r\n",
    "AT+CIPSTART=\"UDP\",\"ec2-34-207-11-203.compute-1.amazonaws.com\",\"20001\"\r\n"      
#endif      
};   
char *setup_ResultCommands[9] = {
    "\r\n+CPIN: READY\r\n\r\nOK\r\n",
    "\r\nOK\r\n"                    ,
    "\r\n+CEREG: 1,5\r\n\r\nOK\r\n" ,
    "\r\nOK\r\n"                    ,    
    "\r\nOK\r\n"                    ,
    "\r\nOK\r\n"                    ,
    "\r\nOK\r\n"                    , // Set TCP/IP Application Mode Non transparent mode
    "\r\nOK\r\n\r\n+NETOPEN: 0\r\n" ,
    "\r\nOK\r\n"                    ,
    
};   
char cmd_AT[]= "AT\r\n";
char cmd_ATE0[] = "ATE0\r\n";
char cmd_ATE1[] = "ATE1\r\n";
char cmd_TEXT_MODE[] = "AT+CMGF=1\r\n"; 
char cmd_CNMI[] = "AT+CNMI=0,0,0,0\r\n"; //Asaf
char cmd_DEL_ALL_SMS[] = "AT+cmgd=,4\r\n";
char cmd_CNMP[] = "AT+CNMP=38\r\n";	    //LTE_mode
char cmd_CMNB[] = "AT+CMNB=1\r\n";          //CAT_M_scan
char cmd_CMNB_READ[] = "AT+CMNB?\r\n";      //Read_CAT_M_mode
char cmd_CFUN[] = "AT+CFUN=1,1\r\n";	    //REBOOT_MODEM 
char cmd_COPS[] = "AT+COPS=0\r\n";          //Auto_net_select
char cmd_COPS_READ[] = "AT+COPS?\r\n";      //READ_operator
char cmd_CEREG[] =  "AT+CEREG?\r\n";        //Network_register_status
char cmd_CSQ[] =  "AT+CSQ\r\n";            // Signal Quality Report 
char cmd_CEREG_[] =  "AT+CEREG=1\r\n";        //Network_register_Enable

int scan_for_response(char* haystack, char* needle, size_t len) {
    size_t needle_len = strlen(needle);

    if (needle_len == 0) {
        return 0;
    }

    for (size_t i = 0; i <= len - needle_len; i++) {
        if (strncmp(&haystack[i], needle, needle_len) == 0) {
            return 1;
        }
    }

    return 0;
}


int scan_for_CMGL(char* response) {
   
   
    if (response == NULL) {
        return -1;
    }
    int response_len = strlen(response);

    for (size_t i = 0; i <= response_len&& i < 12; i++) {
        int index = 0;
        if (strncmp(&response[i], "+CMGL: ", 7) == 0) {
            sscanf(&response[i], "+CMGL: %d,", &index);//save new index to read!!!
            return index;
        }
    }

    return -1;
}

void nrf_delay_ms_wdt(uint32_t x_msec)
{
  //if (x_msec > 10 ){
    wdt_feeding();  // "feed"  WDT
  //} 
   nrf_delay_ms(x_msec);
}

void close_app_network( void );
int modem_setup() //Set modem settings
{
    int err = 0;

    put_command("AT+CRESET\r\n",0);
    if( Modem_CheackResult( "\r\nSMS DONE\r\n", 2 ) == false )nrf_delay_ms_wdt(100);   
                              
    put_command(cmd_AT,0);
    LEDS_INVERT(BSP_LED_0_MASK);
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(100);   
    put_command(cmd_ATE0,0);
    LEDS_INVERT(BSP_LED_0_MASK);
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(100);
    LEDS_INVERT(BSP_LED_0_MASK);
//    put_command(cmd_TEXT_MODE,0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);    
//    put_command(cmd_DEL_ALL_SMS,0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);  
//    put_command(cmd_CNMP,0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);  
    
#ifndef SMS_DEBUG    
#ifdef SIMCOM7080G   
    int a = 0;
    for (int j = 0; j < 5; j++)
    {
         if (j==2){//AT+CSTT       
           char command[50];
           if( hitless_ptr()->cellular_net == 0 /* monogoto */ || hitless_ptr()->cellular_net == 7 /* webbing */)//command for IOT sims
              sprintf(command,"AT+CGDCONT=1,\"IP\",\"data.mono\"\r\n");//APN Configuration 
           else
              sprintf(command,"AT+CSTT=\"%s\"\r\n",cellular_networks_APN[hitless_ptr()->cellular_net]);//APN Configuration 
           put_command(command,0);
        }
         else{        
          //empty_modem_buffer();
          put_command(setup_commands[j],0);
          

          for ( int k = 0 ; k < 9 ; k++ )
          {
              LEDS_INVERT(BSP_LED_0_MASK);
              nrf_delay_ms_wdt(250);
          }
          
          LEDS_INVERT(BSP_LED_0_MASK);
          nrf_delay_ms_wdt(500);
          
          gets_uart_rx_buff((char*)&modem_msg);
          
          for ( a = 0; a < 9 && j == 4; a++)
          {           
              err = check_response(modem_msg+a, j ,0);            
              if (err == 0)
              {
                  LEDS_ON(BSP_LED_0_MASK);
                  break;
              }
          }     
          if (err != 0)
          {
              sys_info.system_error = j;
              //printf("\r\nERROR!!!(cmd %d)\r\n",j);
          }
        }
    }      

#elif SIMCOM7670
    int time_out = 72; //60 sec *6 min
    for (int j = 0; j < 9; j++)
    {    
      nrf_delay_ms_wdt(1);
          //empty_modem_buffer();
        //put_command(setup_commands[j],0);
        if( j == 2 ){
            while (time_out-- )
            {
              put_command(setup_commands[j],0);
              if(Modem_CheackResult( setup_ResultCommands[j], 2 ))
                break;
            }
        }
        else if( j == 5 ){
           char command[50];
           memset(command, 0, 50 );
           sprintf(command,setup_commands[j],cellular_networks_APN[hitless_ptr()->cellular_net]);//APN Configuration 
           put_command(command,0);
        }
        else put_command(setup_commands[j],0);
        
        if( j != 2 )if( Modem_CheackResult( setup_ResultCommands[j], 2 ) == false )nrf_delay_ms_wdt(100);
        
 //       nrf_delay_ms_wdt(10);
        
//        for ( int k = 0 ; k < 9 ; k++ )
//        {
//            LEDS_INVERT(BSP_LED_0_MASK);
//            nrf_delay_ms_wdt(250);
//        }
        
//        LEDS_INVERT(BSP_LED_0_MASK);
//        nrf_delay_ms_wdt(500);
        
//        if( j == 6 ){
//          LEDS_INVERT(BSP_LED_0_MASK);
//          nrf_delay_ms_wdt(500);        
//          LEDS_INVERT(BSP_LED_0_MASK);
//          nrf_delay_ms_wdt(500);
//          LEDS_INVERT(BSP_LED_0_MASK);
//          nrf_delay_ms_wdt(500);
//          LEDS_INVERT(BSP_LED_0_MASK);
//          nrf_delay_ms_wdt(500); 
//        }
        
       //Asaf gets_uart_rx_buff((char*)&modem_msg);                 
    }      
#else
    for (int j = 0; j < 9; j++)
    {
        //empty_modem_buffer();
       
        if (j==5){//AT+CSTT
           //  AT+CGDCONT=1,"IP","data.mono"
           char command[50];
           sprintf(command,"AT+CSTT=\"%s\"\r\n",cellular_networks_APN[hitless_ptr()->cellular_net]);//APN Configuration 
           put_command(command,0);
        }
        else 
          put_command(setup_commands[j],0);
        
        if ( j == 7 || j == 3 || true)
        {
            for ( int k = 0 ; k < 20 ; k++ )
            {
                LEDS_INVERT(BSP_LED_0_MASK);
                nrf_delay_ms_wdt(250);
            }
        }
        LEDS_INVERT(BSP_LED_0_MASK);
        nrf_delay_ms_wdt(500);
        
        gets_uart_rx_buff((char*)&modem_msg);
      
        for ( a = 0; a < 9; a++)
        {
            
            err = check_response(modem_msg+a, j ,0);
            
            if (err == 0)
            {
                LEDS_ON(BSP_LED_0_MASK);
                break;
            }
        }     
        if (err != 0)
        {
            sys_info.system_error = j;
            //printf("\r\nERROR!!!(cmd %d)\r\n",j);
        }        
    }   
#endif    
#endif    
    LEDS_ON(BSP_LED_0_MASK);
    return err;
}
void send_modem_open_connection()
{
#if defined(SIMCOM7080G) || defined(SIMCOM7670) 
    char cmd[100];
    memset( cmd, 0, 100 );
#ifdef SIMCOM7080G 
        
#ifdef TCP_COMMU
          sprintf(cmd,"AT+CAOPEN=0,0,\"TCP\",\"34.207.11.203\",\"8001\"\r\n");
#else    
          sprintf(cmd,"AT+CAOPEN=0,0,\"UDP\",\"34.207.11.203\",\"20001\"\r\n"); 
#endif
#endif 
#ifdef SIMCOM7670         
        sprintf(cmd,"AT+CIPOPEN=0,\"TCP\",\"34.207.11.203\",\"8001\"\r\n");        
#endif   
    put_command(cmd,0);
    //nrf_delay_ms_wdt(1000); 
    if( Modem_CheackResult( "\r\nOK\r\n\r\n+CIPOPEN: 0,0\r\n", 2 ) == false )nrf_delay_ms_wdt(100);
#endif             
}

void send_modem_close_connection()
{
#ifdef SIMCOM7080G
    char cmd[100];
    sprintf(cmd,"AT+CACLOSE=0\r\n");
    put_command(cmd,1);
    nrf_delay_ms_wdt(1000);  
#elif SIMCOM7670
    char cmd[100];
    memset( cmd, 0, 100 );
    sprintf(cmd,"AT+CIPCLOSE=0\r\n"); // AT+NETCLOSE
    put_command(cmd,0);
    //nrf_delay_ms_wdt(1000);  
    if( Modem_CheackResult( "OK+CIPCLOSE: 0,0", 2 ) == false )nrf_delay_ms_wdt(100);
#endif    
}

void open_app_network()
{	
#ifdef SIMCOM7080G
    char cmd[100];
    sprintf(cmd,"AT+CNACT=0,1\r\n");
    put_command(cmd,1);
    nrf_delay_ms_wdt(4000); 	
    nrf_delay_ms_wdt(4000);     
#elif SIMCOM7670 
    char cmd[100];
    memset( cmd, 0, 100 );
    sprintf(cmd,"AT+NETOPEN\r\n");
    put_command(cmd,0);
    //nrf_delay_ms_wdt(4000); 	
    //nrf_delay_ms_wdt(4000);  
    if( Modem_CheackResult( "\r\nOK\r\n\r\n+NETOPEN: 0\r\n", 2 ) == false ){
      if( Modem_CheackResult( "\r\n+IP ERROR: Network is already opened\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
    }   
#endif    
}

void close_app_network()
{	
#ifdef SIMCOM7080G
    char cmd[100];
    sprintf(cmd,"AT+CNACT=0,0\r\n");
    put_command(cmd,1);
    nrf_delay_ms_wdt(1000); 
    nrf_delay_ms_wdt(1000);
#elif SIMCOM7670
    char cmd[100];
    memset( cmd, 0, 100 );
    sprintf(cmd,"AT+NETCLOSE\r\n");
    put_command(cmd,0);
    //nrf_delay_ms_wdt(4000); 	
    //nrf_delay_ms_wdt(4000);  
    if( Modem_CheackResult( "\r\nOK\r\n\r\n+NETCLOSE: 0\r\n", 2 ) == false ){
       if( Modem_CheackResult( "\r\n+NETCLOSE: 2\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
    }
#endif 	
}    

int send_modem_packet(uint8_t * p_data, uint16_t data_len , int uuid_tag) //Send data via UDP/TCP
{
    
    char cmd[EVENT_SIZE+1]; /*"AT+CIPSEND=0,5,\"13.81.203.86\",8000";*/
    memset( cmd, 0, EVENT_SIZE+1 );
    uint8_t data_buffer[EVENT_SIZE+1];
    memset( data_buffer, 0, EVENT_SIZE+1 );
#ifdef SIMCOM7670    
    //uint16_t counter = 1000           ;
    char buff[500]                    ;
    memset            ( buff, 0, 500 );
    reset_uart_rx_buff(              );
    //uint16_t index = 0                 ;
#endif //SIMCOM7670

    //int retry = 2; 
    //char search_str[20];
    //sprintf(cmd,"AT+UDPSEND=%d,\"%s\",%s",data_len,ee_db[_IP_ADDRESS].ee_buff,ee_db[_UDP_PORT].ee_buff);

    //memcpy(&data_buffer[0],p_data,data_len);	            
    //sprintf(cmd,"\r\nAT+CIPSEND=0,%d,\"34.68.215.44\",20001",data_len-1);
    
#ifdef SIMCOM7080G
    nrf_delay_ms_wdt(1);        
    sprintf(cmd,"AT+CASEND=0,%d\r",data_len);  
    put_command(cmd,0);
    nrf_delay_ms_wdt(175);     
#elif SIMCOM7670   
    
    sprintf(cmd,"AT+CIPSEND=0,%d\r",data_len);
    put_command(cmd,0);
    if( Modem_CheackResult( ">", 2 ) == false )return( 1 );
#else
     
    sprintf(cmd,"AT+CIPSEND=%d\r",data_len);
    put_command(cmd,0);
    nrf_delay_ms_wdt(20);      
#endif

    //memcpy(data_buffer ,p_data,strlen((char*)p_data));    
    
#if !defined(SIMCOM7080G) && !defined(SIMCOM7670)   
    app_uart_put('\r');
    nrf_delay_ms_wdt(1);
    app_uart_put('\n');             
#else
    sprintf             ( cmd, "OK+CIPSEND: 0,%d,%d", data_len, data_len    );
    put_command_by_len  ( (char *)p_data, data_len                          );

   if( Modem_CheackResult( (char *)cmd, 2  ) == false )return( 1 );
   if( Modem_CheackResult( "200 OK",    3  ) == false )return( 1 );
   
//    while( counter ){
//       index = gets_uart_rx_buff_without_clr( (char*)buff );
//
//       if( index == 256 )break;
//       if( strlen(buff) ){
//         
//         if( strstr( buff, "+CIPSEND: 0" ) ){
//           if( strstr( buff, "RECV FROM:" ) ){
//             if( strstr( buff, "200 OK" ) )break;
//           }
//         }
//       }
//       
//       nrf_delay_ms_wdt( 1 ); 
//       counter--            ;
//    }
#endif 
    
    //reset_uart_rx_buff( ) ;
    
    return( 0 );
}

static bool Modem_CheackResult( char *pStr, uint16_t Timeout ){
  uint16_t counter  = Timeout * 1000;
  uint16_t index    = 0             ;
  char buff[256]                    ;
  
  memset( buff, 0, 256 );
  
  while( counter ){
    index = gets_uart_rx_buff_without_clr( (char*)buff );

    if( index >= 256 )break;
    if( strlen(buff) ){
      if( strstr( buff, pStr ) )break;
    }
    
    nrf_delay_ms_wdt(2);
    counter--;
  }
  
  reset_uart_rx_buff( ); 
  
  return( counter ? true : false );
}

//retVal 0 if modem returns expected feedback
//       1 Error
int check_response(char * response, int step , int parameter) 
{
    int ret_value = 1;
    char search_str[16];
    memset(search_str,0,16);
  
    switch (step)
    {
        case 0://"AT+CPIN?"
            if (strncmp(response, "+CPIN: READY",12) == 0)
            {
                ret_value = 0;
            }
            break;

        case 1://"AT+CREG?"  (G3)
            if (strncmp(response, "+CEREG: 0,5",10) == 0)
            {
                ret_value = 0;
            }
            else if (strncmp(response, "+CEREG: 0,1",10) == 0)
            {
                ret_value = 0;
            }
            break;        
#if !defined(SIMCOM7080G) && !defined(SIMCOM7670)     
        case 2://"AT+CMEE=1"
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;
            }
            break;


        case 3://"AT+CIPSHUT"
            if (strncmp(response, "SHUT OK",7) == 0)
            {
                ret_value = 0;
            }
            break;

        case 5://"AT+CGATT=1"
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;
            }
            break;
      
        case 6://"AT+CSTT="internetg"
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;
            }
            break;
         
        case 7://"AT+CIICR"
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;
            }
            break;
       case 8://AT+CIFSR          
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;
            }
            break;                        
        case 9://"AT+CIPSTART=\"UDP\",\"34.68.215.44\",\"20001\"      
            if (strncmp(response, "OK",2) == 0)
            {
                ret_value = 0;               
            }
            break;            
#else //Asaf
       case 4://AT+CNACT=0,1          
            sprintf(search_str,"OK");
            ret_value = scan_for_response(response, search_str, strlen(search_str) + 4);
            break;               
       case 10://"AT+CEREG? 
            if (strncmp(response,"+CEREG: 1,5",11) == 0)//5: Registered, roaming. (G4)
            {
                ret_value = 0;
            }
            if (strncmp(response,"+CEREG: 1,1",11) == 0)//1: Registered, home network. (G4)
            {
                ret_value = 0;
            }
            break; 
        case 11://"AT+COPS?
            if( strlen(response) > 10 ) //+COPS: 0,0,"Partner",7     ( 7: LTE )
            {
               ret_value = 0;
               //Set the GSM network operator on variable.
            }
            break;  
//            if (strncmp(response, "+COPS: 2",8) == 0)//2: manual deregister from network 
//            {
//                ret_value = 1;
//            }
//            else if (strncmp(response, "+COPS: 0",8) == 0)//0: Automatic mode
//            {
//                ret_value = 1;
//            }
         
#endif  
      case 99:
            if (strncmp(response, "PB DONE",7) == 0)
            {
                ret_value = 0;
            }      
      break;
     }
    return ret_value;
}

int retry_modem_cmd(char command[], int num_retry, int step) //Retry to send command num_retry times. retVal 0 if modem returns expected feedback, 1 O.W
{
    int err = 1;
    for (int i = 0; i < num_retry; i++)
    {
        nrf_delay_ms_wdt(2000);
        put_command(command,0);
        nrf_delay_ms_wdt(100);
        gets_uart_rx_buff(modem_msg);

        for ( int k = 0 ; k < 20 && err == 1; k++)
#if !defined(SIMCOM7080G) && !defined(SIMCOM7670)       
            err = check_response(modem_msg+k, 8 , 0);
#else
            err = check_response(modem_msg+k, 2 , 0);
#endif
        if (err == 0)
        {
            return 0;
        }
        //printf("ERROR!!!: %s\r\n", modem_msg[1]);
    }
    return 1;
}

void modem_reset(int msec_delay)
{
  nrf_gpio_pin_clear(MODEM_RESET);
  nrf_delay_ms_wdt(500);
  nrf_gpio_pin_set(MODEM_RESET);
  nrf_delay_ms_wdt(msec_delay);
}


int retry_data_send_by_len(char command[], char data[], int data_len  ,int num_retry) //Retry to send data num_retry times. retVal 0 if modem returns expected feedback, 1 O.W
{
    int err = 1;
    for (int i = 0; i < num_retry; i++)
    {
        nrf_delay_ms_wdt(2000);
        put_command(command,0);
        nrf_delay_ms_wdt(2000);
        put_command_by_len(data,data_len);
        nrf_delay_ms_wdt(100);
        gets_uart_rx_buff(modem_msg);
        for ( int k = 0 ; k < 20 && err == 1; k++)
#if !defined(SIMCOM7080G) && !defined(SIMCOM7670)        
            err = check_response(modem_msg+k, 8 , 0);
#else
        err = check_response(modem_msg+k, 2 , 0);
#endif
        if (err == 0)
        {
            return 0;
        }
    }
    return 1;
}

int retry_data_send(char command[], char data[],  int num_retry) //Retry to send data num_retry times. retVal 0 if modem returns expected feedback, 1 O.W
{
    int err = 1;
    for (int i = 0; i < num_retry; i++)
    {
        nrf_delay_ms_wdt(2000);
        put_command(command,0);
        nrf_delay_ms_wdt(2000);
        put_command(data,0);
        nrf_delay_ms_wdt(100);
        gets_uart_rx_buff(modem_msg);
        for ( int k = 0 ; k < 20 && err == 1; k++)
#if defined(SIMCOM7080G) || defined(SIMCOM7670)         
           err = check_response(modem_msg+k, 2 , 0);
#else
           err = check_response(modem_msg+k, 8 , 0);
#endif
        if (err == 0)
        {
            return 0;
        }
    }
    return 1;
}
//retVal true if sim register to network
//       false Error
bool sim_registration_process(){
    /* Variables */
    char *token;
    char parameters[4][20];
    bool network_status = false;
    int time_out = 72;//360 //60 sec *6 min
    
   /* PUT COMMAND TO MODEM BY UART */
    put_command(cmd_AT,0);    
    LEDS_INVERT(BSP_LED_0_MASK);
   // nrf_delay_ms_wdt(1000);  
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
    
    put_command(cmd_ATE0,0);//ECHO
    LEDS_INVERT(BSP_LED_0_MASK);
    //nrf_delay_ms_wdt(1000);
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
#if !defined(SIMCOM7670)   
    put_command(cmd_CMNB,0); //select CAT-M service
    LEDS_INVERT(BSP_LED_0_MASK);
    nrf_delay_ms_wdt(1000);
#endif //!defined(SIMCOM7670)   
    
//    put_command(cmd_CFUN,0); //modem reset
//    LEDS_INVERT(BSP_LED_0_MASK);
//    
//    if( Modem_CheackResult( "\r\nOK\r\n" ) == false )nrf_delay_ms_wdt(1000);
    
    //nrf_delay_ms_wdt(1000);
    //nrf_delay_ms_wdt(1000);
    //nrf_delay_ms_wdt(1000);
    //nrf_delay_ms_wdt(1000);
    //nrf_delay_ms_wdt(1000);
    //nrf_delay_ms_wdt(1000);
    
    put_command(cmd_COPS,0);//network selection - Automatic mode
    LEDS_INVERT(BSP_LED_0_MASK);
    //nrf_delay_ms_wdt(1000);  
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
    
    put_command(cmd_CEREG_,0);//checking if the sim is register now(G4)
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
    
/* WAIT FOR NETWORK REGISTERATION */
    while (time_out-- ){
#if defined(SIMCOM7080G) || defined(SIMCOM7670)
  
      put_command(cmd_CEREG,0);//checking if the sim is register now(G4)
      if(Modem_CheackResult( "\r\n+CEREG: 1,5\r\n\r\nOK\r\n", 2 )){
        LEDS_ON(BSP_LED_0_MASK);
        network_status = true;
        break;
      }
    }
    
    LEDS_INVERT(BSP_LED_0_MASK);
    nrf_delay_ms_wdt(1000);

    
#else
      put_command(setup_commands[1],0);//checking if the sim is register now(G3)  
#endif
//      LEDS_INVERT(BSP_LED_0_MASK);
//      nrf_delay_ms_wdt(1000);   
//            
//      gets_uart_rx_buff((char*)&modem_msg);
//            
//      for ( int a = 0; a < 20; a++)
//      {
//#if defined(SIMCOM7080G) || defined(SIMCOM7670)
//        network_status = check_response(modem_msg+a, 10 ,0) == 0 ? true: false; // "AT+CEREG?" (G4)
//#else
//        network_status = check_response(modem_msg+a,1,0) == 0 ? true: false;  //"AT+CREG?"  (G3)
//#endif
//          if (network_status == true)
//          {
//             //NETWORK IS REGISTERED
//              LEDS_ON(BSP_LED_0_MASK);
//              break;
//          }
//      }     
   // }
     if (network_status == false)
    {
        sys_info.system_error = 15;
        //operator = error!!!
        sprintf( hitless_ptr()->network_operator,"NETWORK ERROR\0");
    }
    else {
      put_command(cmd_COPS_READ,0);//read the operator of current network 
      LEDS_INVERT(BSP_LED_0_MASK);
      nrf_delay_ms_wdt(1000);  
      gets_uart_rx_buff((char*)&modem_msg);
      check_response(modem_msg , 11 ,0); //"AT+COPS?\r\n"; 
      //response parser for get the name of operator 
      token = strtok(modem_msg, " ");
      for (int i = 0; i < 4 ; i++) {
          token = strtok(NULL, ",");
          if (token != NULL) {
              strcpy(parameters[i], token);
          } else {
              strcpy(parameters[i], "");  // Handle case where token is NULL
          }
      }
      snprintf(hitless_ptr()->network_operator, 16 , "%s%c" , parameters[2], parameters[3][0]);//+COPS: 0,0,"Partner",7 
      
    }  
    return network_status;
}

void read_signal_quality(){
  
      // Variables
      char *token;
      char parameters[2][10];
      
      // Read the Signal Quality 
      put_command(cmd_CSQ,0);
      LEDS_INVERT(BSP_LED_0_MASK);
      nrf_delay_ms_wdt(1000);  
      gets_uart_rx_buff((char*)&modem_msg);
    
      //Response parser for get the Signal Quality - "AT+CSQ\r\r\n+CSQ:\r\n\r\n31,99OK"
      token = strtok(modem_msg, " ");
      for (int i = 0; i < 2 ; i++) {
          token = strtok(NULL, ",");
          if (token != NULL) {
              strcpy(parameters[i], token);
          } else {
              strcpy(parameters[i], "");  // Handle case where token is NULL
          }
      }
      hitless_ptr()->modem_signal_quality = atoi(parameters[0]); //The range of values is 0-31
}
bool modem_registration_check()
{
    char *setup_commands[2];
    int j,a;
    bool err;
    bool result_status[2] = {false,false};
    setup_commands[0] = "AT+CPIN?\r\n";
    setup_commands[1] = "AT+CEREG?\r\n"; 
         
    for (j = 0; j < 2 && result_status[j] == 0; j++)
    {
        //empty_modem_buffer();
       
        
        put_command(setup_commands[j],0);

        LEDS_INVERT(BSP_LED_0_MASK);
        nrf_delay_ms_wdt(500);
        
        gets_uart_rx_buff((char*)&modem_msg);
      
        for ( a = 0; a < 10; a++)
        {
            
            err = check_response(modem_msg+a, j ,0);
            
            if (err == 0)
            {
                LEDS_ON(BSP_LED_0_MASK);
                result_status[j] = true;
            }
        }
    }
    
    LEDS_ON(BSP_LED_0_MASK);
    
    return (result_status[0]&result_status[1]);
}
int modem_ready() //Sends command to modem via UART
{
    int retray_modem_ready,ret_val;
    int a,err , length_of_string;
    ret_val = 0;
    
    gets_uart_rx_buff((char*)&modem_msg);
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();
    
   
    for ( retray_modem_ready = 0; retray_modem_ready  < 120 && ret_val == 0 ; retray_modem_ready ++)
    {
        LEDS_INVERT(BSP_LED_0_MASK); //TURN ON MODEM
        nrf_delay_ms_wdt(500);      
        
        length_of_string = gets_uart_rx_buff_without_clr((char*)&modem_msg);
        for ( a = 0; a < MODEM_RX_BUF_SIZE && a < length_of_string && ret_val == 0; a++)
        {
            err = check_response((char*)&modem_msg[a], 99 , 0);

            if (err == 0)
            {
                LEDS_ON(BSP_LED_0_MASK);
                ret_val = 1;
            }
        } 
    }
    
    
    if (ret_val == 1)
    {
      ret_val = 0;
      for ( retray_modem_ready = 0; retray_modem_ready  < 60 && ret_val == 0; retray_modem_ready ++)
      { 
        LEDS_INVERT(BSP_LED_0_MASK);
        if (modem_registration_check() == 1)
          ret_val = 1;
      }
    }
    LEDS_ON(BSP_LED_0_MASK);
    return ret_val;       
}

void put_command_by_len(char * command, int len) //Sends command to modem via UART
{
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();
    
    int _len    = len   ;
   // int Length  = 0     ;
    int i       = 0     ;
    
    while( _len ){
        if (command[i] != 0){
          while( app_uart_put(command[i])  == NRF_ERROR_NO_MEM );
        }
        
        i++     ;
        _len--  ;
      }
}

void put_command(char * command , int add_CRLF) //Sends command to modem via UART
{
    wdt_feeding(); // "feed"  WDT
    int len = strlen(command);
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();
    
    for (int i = 0; i < len; i++)
    {
        //app_uart_put
      app_uart_put_fifo(command[i]);
        //nrf_delay_ms_wdt(1);
    }
   
    app_uart_put_send();
    
//    if (add_CRLF == 1)
//    {
//      app_uart_put('\r');
//      //nrf_delay_ms_wdt(1);
//      app_uart_put('\n');
//      //nrf_delay_ms_wdt(1);
//    }
}


void reset_modem()
{
    put_command("AT+CRESET\r\n",0);
   if( Modem_CheackResult( "\r\nSMS DONE\r\n", 2 ) == false )nrf_delay_ms_wdt(1000);
   
    modem_setup();
    nrf_delay_ms_wdt(1500);
}


void modem_power_off()
{
    modem_reset(5000);
#if 0    
    put_command("AT\r\n",0); //0.798mA sleep
    nrf_delay_ms_wdt(1000);
    put_command("AT\r\n",0); //0.798mA sleep
    nrf_delay_ms_wdt(1000);
    put_command("AT\r\n",0); //0.798mA sleep
    nrf_delay_ms_wdt(1000);
    put_command("AT\r\n",0); //0.798mA sleep
    nrf_delay_ms_wdt(1000);    
    put_command("AT+CFUN=1\r\n",0); //0.798mA sleep
    nrf_delay_ms_wdt(1000);    
    put_command("AT+CSCLK=1\r\n",0);// enter to sleep ( wakeup by reset to LOW )
    nrf_delay_ms_wdt(1000);
    LEDS_INVERT(BSP_LED_0_MASK); //TURN ON MODEM
#endif    
}


void print_modem_buffer()
{
    int j = 0;
    printf("\r\n");
    for (int i = 0; i < MODEM_RX_BUF_SIZE; i++)
    {
        while (modem_msg[i] != '\r')
        {
            printf("%c", modem_msg[i]);
            j++;
        }
        printf("\r\n");
    }
}


int power_up_delay = 100;
void modem_init()
{
  sys_info.modem_state = MODEM_POWERUP;  
  power_up_delay = 100;
}
void sms_message_loop();
extern void modem_uart_init(void);
extern void push_event(uint8_t *str_event , int16_t str_event_len);

#define SEND_PACKET_SIZE        (1300)
char SendPacket[SEND_PACKET_SIZE];
int Index = 0;
bool IsNotEmpty = true;
bool Retry = false;
int NumOfRetry = 3;

void send_modem_open__connection()
{
#if defined(SIMCOM7080G) || defined(SIMCOM7670) 
    char cmd[100];
    memset( cmd, 0, 100 );
#ifdef SIMCOM7080G 
        
#ifdef TCP_COMMU
          sprintf(cmd,"AT+CAOPEN=0,0,\"TCP\",\"34.207.11.203\",\"8001\"\r\n");
#else    
          sprintf(cmd,"AT+CAOPEN=0,0,\"UDP\",\"34.207.11.203\",\"20001\"\r\n"); 
#endif
#endif 
#ifdef SIMCOM7670         
        sprintf(cmd,"AT+CIPOPEN=0,\"TCP\",\"34.207.11.203\",\"8001\"\r\n");        
#endif   
    put_command(cmd,0);
    //nrf_delay_ms_wdt(1000); 
    if( Modem_CheackResult( "OK+CIPOPEN: 0,0", 2 ) == false )nrf_delay_ms_wdt(100);
#endif             
}
MODEM_STATE modem_handller()
{
          uint8_t temp[128];
          int length_of_pop_data;
          int send_msg_buff_counter = 0;         
           switch ( sys_info.modem_state )
           {
              case MODEM_POWERUP:
                  //power modem on
                   nrf_gpio_pin_set(HV_EN);     //for new hardware version HV_EN
                   nrf_delay_ms(10);        // moderate the voltage drop
                   nrf_gpio_pin_set(SENSOR_ON); //for new hardware version SENSOR_ON
                   nrf_delay_ms_wdt(10);        // moderate the voltage drop
                   nrf_gpio_pin_set(MODEM_POWER_ON);
                   
                   power_up_delay--;
                   if (power_up_delay == 50)
                     nrf_gpio_pin_clear(MODEM_RESET);
                   else
                      nrf_gpio_pin_set(MODEM_RESET);
                   if (power_up_delay > 0)
                    break;
                   
                   power_up_delay = 100;
                   modem_uart_init();
                   if( hitless_ptr()->modem_event == MODEM_SMS_EVENT)
                   {
                      hitless_ptr()->modem_event = NO_EVENT;
                      sys_info.modem_state = MODEM_SETUP_SMS;                  
                   }
                   else if ( hitless_ptr()->modem_event == MODEM_CLOUD_EVENT )//shir 
                   {
                      hitless_ptr()->modem_event = NO_EVENT;
                      sys_info.modem_state = MODEM_SETUP;                  
                   }             
                   else //if modem_event not SMS_EVENT or CLOUD_EVENT so POWER_OFF.
                   {
                      hitless_ptr()->modem_event = NO_EVENT;
                      sys_info.modem_state = MODEM_POWER_OFF;                   
                   }                
                   break;
                  
             case MODEM_SETUP:
                    modem_setup();    
                    send_modem_open_connection( );
                    sys_info.modem_state = MODEM_TRANSMMISION_BUFFER;                                                         
                    break;
             case MODEM_SETUP_SMS:
                    if(sim_registration_process())
                    {
                      read_signal_quality();
                      sms_message_loop();    
                    }
                    else
                      hitless_ptr()->skip_modem_flag = 0;//flag off
                    sys_info.modem_state = MODEM_FINISH;  
                    return sys_info.modem_state;
                    break;
                  
              case MODEM_TRANSMMISION_BUFFER:
                  send_msg_buff_counter = 0                         ;
                  IsNotEmpty            = event_buffer_not_empty( ) ;
                  
                  while( IsNotEmpty || Retry ){ 
                    IsNotEmpty = event_buffer_not_empty( );
                    
                    if( IsNotEmpty )top_event(temp, &length_of_pop_data );
                    
                    if ( temp[0] == NULL &&  IsNotEmpty && (Retry == false) )
                    { /* Exit - error in the buffer */
                            pop_event();       
                    }
                    else 
                    {     
#if 0//defined(SIMCOM7080G) || defined(SIMCOM7670)
                      
                        if(send_msg_buff_counter%600 == 0 && send_msg_buff_counter > 0){
                          
                              send_modem_close_connection(); 							  
                              close_app_network();
                              open_app_network();
                              send_modem_open_connection();  								
                        }                       
                        else if(send_msg_buff_counter%100 == 0){
                          
                           if (send_msg_buff_counter > 0 ) 
                             send_modem_close_connection(); 
                           send_modem_open_connection();  
                        }
#endif                  
                        if( IsNotEmpty && (Retry == false) ){
                          length_of_pop_data = strlen((char *)temp);
                        }
                        
                        if( (Index + length_of_pop_data < SEND_PACKET_SIZE) && IsNotEmpty && (Retry == false) ){
                          memcpy( &SendPacket[Index], temp, length_of_pop_data );
                          Index += length_of_pop_data                           ;
                          pop_event( )                                          ;
                        }
                        else{
                          if( send_modem_packet( (uint8_t *)SendPacket, Index, 0 ) == 0 ){
                            Retry = false                                           ;
                            Index = 0                                               ;  
                            memset( SendPacket, 0, SEND_PACKET_SIZE )               ;
                            send_msg_buff_counter++                                 ;
                          }
                          else if(NumOfRetry--){
                            Retry = true;
                          }
                          else {
                            break;
                           }
                         }   
                      }
                    }
#ifdef CLOUD_CMD                   
                  sprintf((char *)temp,"<CMD>\n");                
                  length_of_pop_data = strlen((char *)temp);
                  push_event(temp , length_of_pop_data);  
                  sys_info.modem_state = MODEM_WAIT_FOR_COMMAND; 
                  sys_info.countdown_sleep_seconds = 10;                  
#else                   
                  sys_info.modem_state = MODEM_POWER_OFF; 
#if defined(SIMCOM7080G) || defined(SIMCOM7670) 
                  close_app_network();
#endif                 
                  hitless_ptr()->event_buff.event_buffer_index = 0; //shir, all the buff upload so its size==0.
#endif                  
                  break;
                  
           case MODEM_WAIT_FOR_COMMAND:                                       
                /* Finish to send wait 10Sec */
#ifndef DBG_BUFFER_NOT_EMPTY                  
                sys_info.countdown_sleep_seconds--;
#endif                          
                if ( sys_info.countdown_sleep_seconds <= 0 )
                {
                  sprintf((char *)temp,"</CMD>\n");                
                  length_of_pop_data = strlen((char *)temp);
                  push_event(temp , length_of_pop_data);
                  sys_info.modem_state = MODEM_POWER_OFF;                  
                } 
                
                if (event_buffer_not_empty())
                {                    
                  top_event(temp, &length_of_pop_data);
                  
                  if ( temp[0] == NULL  )
                  { /* Exit - error in the buffer */
                          pop_event();                             
                  }
                  else
                  { 
                       
                        length_of_pop_data = strlen((char *)temp);
#ifndef SMS_DEBUG
                        if (send_modem_packet(temp, length_of_pop_data , 0) == 0)
#endif                        
                          pop_event();                           
                  }
                  nrf_delay_ms_wdt(10);
                }
                  
                nrf_delay_ms_wdt(500);
               
                break;
              
              case MODEM_POWER_OFF:                     
              case MODEM_ERROR_POWERUP:
              case MODEM_SETUP_ERROR:
                 nrf_gpio_pin_clear(MODEM_POWER_ON);
                 nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
                 nrf_gpio_pin_clear(SENSOR_ON); //for new hardware version SENSOR_ON 
                 nrf_gpio_pin_clear(MODEM_RESET);
                 sys_info.modem_state = MODEM_FINISH;
                 break;
                 
              default :                 
                 //modem_power_off();
                 nrf_gpio_pin_clear(MODEM_POWER_ON);
                 nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
                 nrf_gpio_pin_clear(SENSOR_ON); //for new hardware version SENSOR_ON 
                 nrf_gpio_pin_clear(MODEM_RESET);
                 sys_info.modem_state = MODEM_FINISH;
                 events_stack_init(POWER_ON_RESET);
                 break;
           }
           
           return sys_info.modem_state;
}                    
                        
#define Ctrl_Z 0x1A

void send_sms_message( char *phone_number , char *sms_message , int index_flag )
{
    char  sms_send_message[128];    
    //Asaf put_command("AT\r\n",0);
    //nrf_delay_ms_wdt(500);
    sprintf( sms_send_message,/*\r\n*/"AT+CMGS=\"%s\"\r", phone_number ); 
    
    
    //nrf_delay_ms_wdt(100);
    put_command(sms_send_message,0);  
    if( Modem_CheackResult( "\r\n> ", 2 ) == false )nrf_delay_ms_wdt(100);
    
//    nrf_delay_ms_wdt(1000);   
//    nrf_delay_ms_wdt(1000);
//    nrf_delay_ms_wdt(1000);
    
    memset( sms_send_message, 0, 128);
    
    if(index_flag)
      sprintf( (char *)sms_send_message , "T[%d]R[%d] %s\r\n" , hitless_ptr()->messages_counter++ ,hitless_ptr()->messages_counter_rx++, sms_message); 
    else
      sprintf( (char *)sms_send_message , "%s\r\n" , sms_message );
    
    sms_send_message[strlen(sms_send_message)] = Ctrl_Z;
    put_command(sms_send_message,0); 
        
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(100);
}
#define RCV_CMD_BUFF 257
char response_CMGR[RCV_CMD_BUFF+1];
int start_CMGR = 0;
int response_len_CMGR = 0;
void read_received_message( char *received_data ,int msg_index )
{
#if 1    
    uint8_t response[RCV_CMD_BUFF];
    char cmd_read[100];
    
    gets_uart_rx_buff((char*)&modem_msg);
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();
    /* read SMS by index */
    //Asaf sprintf(cmd_read,"AT+CMGR=%d,0\r\n",msg_index); 
    sprintf(cmd_read,"AT+CMGR=%d\r\n",msg_index); 
    put_command(cmd_read,0);  
    nrf_delay_ms_wdt(1000);
    memset(response,0,RCV_CMD_BUFF);
    gets_uart_rx_buff((char*)&response);
    strncpy( received_data , (char *)response , RCV_CMD_BUFF ); 
#else
    char cmd_read[100];
    sprintf(cmd_read,"AT+CMGR=%d,0\r\n",msg_index);    
    gets_uart_rx_buff((char*)&modem_msg);
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();    
    put_command(cmd_read,0); 
    // Read the response from the modem
    memset(response_CMGR,0,RCV_CMD_BUFF);    
    for ( int k = 0 ; k < 5000 ; k ++)
    {             
      gets_uart_rx_buff_without_clr((char*)&response_CMGR);
      nrf_delay_ms_wdt(1); 
      if(strlen(response_CMGR) > 48 ){
        gets_uart_rx_buff((char*)&response_CMGR);
        break;       
      }
    } 
    
    response_len_CMGR = strlen((char*)response_CMGR);   
    for (int index = 0; index <= response_len_CMGR && index < 32; index++) {        
        if (strncmp((char*)&response_CMGR[index], "+CMGR: ", 7) == 0) {           
            start_CMGR = index;
            break;
        }
    }     
    strncpy( received_data , (char *)&response_CMGR[start_CMGR] , RCV_CMD_BUFF - start_CMGR);     
    
#endif    
}


#ifdef DEBUG_COMMU
char phone_number_of_administrator[12] = "0549432262\0";
char phone_number_of_sender[12]        = "0549432262\0";
#else
char phone_number_of_administrator[12] = "0587781751\0";
char phone_number_of_sender[12]        = "0587781751\0";
#endif
uint8_t number_of_waiting_messages = 0;

void set_phone_number_of_sender( char *international_phone_number_of_sender )
{ 
    memcpy( phone_number_of_sender , &international_phone_number_of_sender[3] , strlen(international_phone_number_of_sender) - 3 );
    phone_number_of_sender[0] = '0';  // First digit of a phone number in local calling
}


bool received_sms_message_parsing( char * sms_message , char * command , char * command_parameter )
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
    

    
/*    
+CMGR: "REC UNREAD","+972549432262","","20/07/28,09:44:13+12"
Sy

OK
*/  
    /* search for + "+CMGR" take pointer in the sms_message*/
    start_of_relavent_information = (char*)memchr( sms_message     , '+' , 10 );
    /* delete the first */
    *start_of_relavent_information=' ';
    
    /* search for + "+9725494" take pointer in the sms_message*/
    start_of_relavent_information = (char*)memchr( sms_message     , '+' , 32 );
 
    
    /* copy all the relevat SMS to temprary string */
    strncpy( temporary_string , start_of_relavent_information ,256);
    
    /* stop condition */
    temporary_string[255] = '\0';
    
    /* search to NULL */
    end_of_relavent_information = (char*)memchr( start_of_relavent_information , '\0' , strlen(start_of_relavent_information) + 1 );
    
    index = 0;
    while( &start_of_relavent_information[index] != end_of_relavent_information  )
    {
        if( start_of_relavent_information[index] == ','
         || start_of_relavent_information[index] == '"' 
         || start_of_relavent_information[index] == '\r' 
         || start_of_relavent_information[index] == '\n' )
            temporary_string[index] = ' ';
        else
            temporary_string[index] = start_of_relavent_information[index];
        index++;
        if( index >= 255 )
          return false;        
    }

    
    sscanf( temporary_string , "%s%s%s%s%s%s" , phone_number , date , time , sms_text , parameter , ok_string );      
    //set_date( &date[0] );
    //set_time( &time[0] );
    if( strncmp( parameter , "OK" , 2 ) == 0)
    {
        set_phone_number_of_sender( phone_number );
        strcpy( command , sms_text );
        strcpy( command_parameter , "" );        
        return true;
    }
    if ( strncmp( ok_string , "OK" , 2 ) == 0)
    {
        set_phone_number_of_sender( phone_number );
        strcpy( command , sms_text );
        strcpy( command_parameter , parameter );        
        return true;
    }      
    
    return false;
    
}

void send_critical_msg(char * output_msg2){//Function for sending a critical msg -> if return; : continu to listening status. O.W :modem power off.
      
       int critical_msg_index = hitless_ptr()->critic_msg[1]-'0';
       hitless_ptr()->send_critical_msg = 0; 
       /* Sending a critical msg */       
       sprintf( output_msg2,"%s\nTx[%d]Rx[%d]",hitless_ptr()->critic_msg, hitless_ptr()->messages_counter++,hitless_ptr()->messages_counter_rx);       
       send_sms_message( hitless_ptr()->phone_number_of_administrator ,output_msg2 , 0); 
       for ( int k = 0 ; k < 10 ; k ++)
       {
          //put_command("AT\r\n",0); //0.798mA sleep
          nrf_delay_ms_wdt(1000); 
       }
       
#ifdef VALVE_ASSEMBLED //Water cannot be shut off if there is no water VALVE.
       
       switch(critical_msg_index){
       case 1:  //If the critical message announces the water closure.
         hitless_ptr()->flow_controller = 1;            //When the modem will shut off, flow will be closed by FC 1 command
         hitless_ptr()->check_message_coutdown = 250;   //The next modem session(in case the client want back to running water).
         break;
       case 2:  //If the critical message msg is waiting for SD command.
         hitless_ptr()->send_critical_msg = 3;  
         sprintf( hitless_ptr()->critic_msg,"[1]Acute flow!\nDisconnect the water flow now");  
         hitless_ptr()->check_message_coutdown = 30; //The next modem session for send sd command.
         return;
         break;
       default:
         break;
       }
#endif 
      //MO_0 
      hitless_ptr()->device_state = SLEEP_STATE;
      //power modem off
      nrf_gpio_pin_clear(MODEM_POWER_ON);
      nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
      nrf_gpio_pin_clear(MODEM_RESET);
      nrf_gpio_pin_clear(SENSOR_ON);      
      wrap_NVIC_SystemReset();
}
char response_CMGL[RCV_CMD_BUFF+1]; 

int get_process_new_index_message() 
{
 
    modem_counter = gets_uart_rx_buff_without_clr((char*)modem_msg);
    
    if( modem_counter < strlen("+CMTI: \"SM\", ") )return( 0xFF );
    
    char *pTr     = strstr( modem_msg, "+CMTI: \"SM\"," )   ;
    uint8_t index = 0xFF                                    ;
        
    if( pTr != NULL ){
      pTr += strlen("+CMTI: \"SM\",");
      
      if( *pTr < 0x30 || *pTr > 0x39 )return( 0xFF );
      
      index = *pTr - 0x30;
    }
    else return( 0xFF );
    
    modem_counter = 0     ;
    reset_uart_rx_buff( ) ; 
    
    return( index );
}

int process_new_index_message() 
{
    char cmd_read_all_unread[] = "AT+CMGL=\"REC UNREAD\",1\r";
    char temp[256];    
    gets_uart_rx_buff((char*)&modem_msg);
    memset(modem_msg,0,MODEM_RX_BUF_SIZE);
    reset_uart_rx_buff();    
    put_command(cmd_read_all_unread,0); 
    // Read the response from the modem
    memset(response_CMGL,0,RCV_CMD_BUFF+1);    
    for ( int k = 0 ; k < 5000 ; k ++)
    {             
        gets_uart_rx_buff_without_clr((char*)&response_CMGL);
        nrf_delay_ms_wdt(1); 
        if(strlen(response_CMGL) > 48 ){
          gets_uart_rx_buff((char*)&response_CMGL);
          break; 
        }
    }
    /* wait for all list of SMS rcvievd */
    for ( int k = 0 ; k < 100 ; k++){
      memset(temp,0,256);
      nrf_delay_ms_wdt(100);       
      gets_uart_rx_buff((char*)&temp);
      if(strlen(temp) < 1 )
        break;      
    } 
   
    return scan_for_CMGL(response_CMGL);
/*    
    scan_for_CMGL(response);    
    int len = strlen(response);
    if (len > 0) {
        char *token = strtok(response, "\n");
        if (token != NULL) {
            if (strstr(token, "+CMGL:") != NULL) {          
                sscanf(token, "+CMGL: %d,", &index);//save new index to read!!!
                }
        }
    }
*/    
}

void sms_message_loop()
{
    hitless_ptr()->skip_modem_flag = 0;//flag off
    int index_msg = 0;
    int max_index_msg = 15;   //This value indicates how match time the modem will be listening.
    int rcv_msg_index_from_modem = -1;
    char command[RCV_CMD_BUFF];
    char parameter[30];

    char output_msg2[128];
    char sms_text[RCV_CMD_BUFF];
    int output_length2;
    
    
    char input_msg[128];
    int input_length = 0;
    int return_val = 0;
    bool rcv_valid_msg = false;
   

//     put_command("AT+CRESET\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(2000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command(cmd_AT,0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
////    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command(cmd_ATE0,0);
//    if( Modem_CheackResult( "\r\nOK\r\n" ) == false )nrf_delay_ms_wdt(100);   
    
//        put_command(cmd_AT,0);    
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);   
    
    //
//    put_command("AT+SIMCOMATI\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    
//    put_command("AT+CPIN?\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command("AT+CMGF=1\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command("AT+CNMI=2,1,0,0\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command("AT+CNMI?\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command("AT+CSQ\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
//    put_command("AT+COPS?\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    
//    
//    put_command("AT+CEREG=1\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    
//    put_command("AT+CEREG?\r\n",0);
//    LEDS_INVERT(BSP_LED_0_MASK);
//    nrf_delay_ms_wdt(1000);
//    LEDS_INVERT(BSP_LED_0_MASK);
    
    /*Asaf
    put_command(cmd_CNMI,0);
    LEDS_INVERT(BSP_LED_0_MASK);
    nrf_delay_ms_wdt(1000);  
    */
    
    put_command(cmd_TEXT_MODE,0);
    LEDS_INVERT(BSP_LED_0_MASK);
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(100);    
    
    if (hitless_ptr()->send_critical_msg == 1){ //if there is a critical message to send.
      send_critical_msg(output_msg2);
      max_index_msg = 10; //with listenning state.
    }  
    else if (hitless_ptr()->send_critical_msg > 1){ //if the modem in listenning state.
      hitless_ptr()->check_message_coutdown = (hitless_ptr()->send_critical_msg == 3) ? 60 : 10; //the next modem session.
      hitless_ptr()->send_critical_msg--;
      max_index_msg = 10; //with listenning state.
      sprintf(output_msg2,"[9]Send 'SD 1' command Tx[%d]Rx[%d]",hitless_ptr()->messages_counter++,hitless_ptr()->messages_counter_rx);      
      send_sms_message(  hitless_ptr()->phone_number_of_administrator ,output_msg2 , 0); 
    }
    else //Normal situation.
    { 
      sprintf(output_msg2,"Hello Tx[%d]Rx[%d] %s(%d)",hitless_ptr()->messages_counter++,hitless_ptr()->messages_counter_rx,hitless_ptr()->network_operator,hitless_ptr()->modem_signal_quality);      
      send_sms_message(  hitless_ptr()->phone_number_of_administrator ,output_msg2 , 0); 
    }
    
   
    while( (index_msg < max_index_msg) && (sys_info.finish_sms_session != 1))     //New Message has arrived by SMS
    {
      
        nrf_delay_ms_wdt(10); 
      
        memset(sms_text,0,RCV_CMD_BUFF);
        
        rcv_msg_index_from_modem = get_process_new_index_message();  
        
        if (rcv_msg_index_from_modem != 0xFF)
        {          
          read_received_message(sms_text,rcv_msg_index_from_modem);  
          sms_text[RCV_CMD_BUFF-1] = 0;
        }     
  
      
        if ( strlen(sms_text)> 16 && strlen(sms_text) < RCV_CMD_BUFF/2 )
        {
             rcv_valid_msg = received_sms_message_parsing( sms_text , command , parameter );

             if (rcv_valid_msg == true )
             {
                 if(strlen(parameter) > 0)
                    sprintf(input_msg,"%s %s",command,parameter);
                 else
                    sprintf(input_msg,"%s \r",command);
                   
                  input_length = strlen((char *)&input_msg[0]);  
                  memset(output_msg2,0,128);
                  return_val = process_uart_command( (char *)&input_msg[0], input_length , output_msg2 , &output_length2);
               
                  if ( return_val == EXIST)
                  {
                    send_sms_message( (char *)&phone_number_of_sender[0],output_msg2 , 1 );            
                  }
            }
            sprintf(sms_text,"AT+CMGD=%d,0\r\n",rcv_msg_index_from_modem );//DELETE ALL READ SMS
            put_command(sms_text,0);  
       
            index_msg = 0;// scan again all the messages if new message recived
        }
        else
        {
          index_msg++;
          nrf_delay_ms_wdt(1000); 
        }  
          
    }
    
    sprintf(sms_text,"AT+CMGD=,4\r\n");//DELETE ALL SMS
    put_command(sms_text,0);          
    if( Modem_CheackResult( "\r\nOK\r\n", 2 ) == false )nrf_delay_ms_wdt(100); 
}    