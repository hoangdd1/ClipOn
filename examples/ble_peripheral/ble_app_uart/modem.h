#ifndef MODEM_H__
#define MODEM_H__

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_delay.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_soc.h"
//#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "system_info.h"

#define MODEM_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define MODEM_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

void read_received_message( char *received_data ,int msg_index);
void set_phone_number_of_sender( char *international_phone_number_of_sender );
bool received_sms_message_parsing( char * sms_message , char * command , char * command_parameter);
void send_sms_message( char *phone_number , char *sms_message, int index_flag  );
extern char phone_number_of_administrator[12];
extern char phone_number_of_sender[12];

extern char modem_msg[MODEM_RX_BUF_SIZE];
extern int modem_counter;


int modem_setup(); //Set modem settings
void send_modem_open_connection();
void send_modem_close_connection();
int send_modem_packet(uint8_t * p_data, uint16_t data_len , int uuid_tag);//Send data via UDP

int check_response(char * response, int step , int parameter); //retVal 0 if modem returns expected feedback, 1 O.W

int retry_modem_cmd(char command[], int num_retry, int step); //Retry to send command num_retry times. retVal 0 if modem returns expected feedback, 1 O.W

int retry_data_send(char command[], char data[],  int num_retry); //Retry to send data num_retry times. retVal 0 if modem returns expected feedback, 1 O.W
int retry_data_send_by_len(char command[], char data[], int data_len  ,int num_retry);
void put_wifi_uart(char * command , int ms_delay);
void put_command(char * command , int add_CRLF); //Sends command to modem via UART
void put_command_by_len(char * command, int len); //Sends command to modem via UART


void reset_modem(); //Reset and reconfigure modem
void empty_modem_buffer(); //0 out all modem_msg[]
void modem_init();
void sms_message_loop();

MODEM_STATE modem_handller();

#endif // MODEM_H__