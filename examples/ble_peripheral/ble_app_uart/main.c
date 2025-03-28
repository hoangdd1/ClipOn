/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "uart_commands.h"
#include "nrf_delay.h"
#include "hitless_data.h"
#include "system_info.h"
#include "message_handler.h"  
#include "modem.h"
#include "nrf_power.h"
#include "nrf_calendar.h"
#include "i2c_bus.h"

#if 0
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#endif
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define UICR_REGOUT0_VOUT_3V3 (5UL) /*!< 3.3 V */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Water_Sensor"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#ifdef DEBUG_COMMU
#define APP_ADV_INTERVAL                64    
#define APP_ADV_INTERVAL_LOW            64      
#else
#define APP_ADV_INTERVAL_LOW            1000 /* 4000   */                            /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_INTERVAL                64 /*1500 64   */                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#endif

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


void wrap_NVIC_SystemReset(){
  unsigned int   last_check_time = hitless_ptr()->rtc_var.m_time;
  int max_wait_time = 3000;
  /* check for RTC finish */
  /* counter for the reset */
  /* correct the RTC */
  if(hitless_ptr()->reset_reason == SOFTWARE_RESET){    
    while(last_check_time  == hitless_ptr()->rtc_var.m_time && --max_wait_time > 0){ 
      nrf_delay_ms(1);
    }
    hitless_ptr()->number_of_software_reset++;
  }
  NVIC_SystemReset();
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */

int start_sending_streaming = 0;


#if 0
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
 #if 0 
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
 #endif
      start_sending_streaming = 1;
    }

}
#endif

#if 1

int ble_device_during_session = 0;
int on_ble_evt_connected_coundown = 0;

#define TX_BUFFER 256

uint32_t nus_send_data_buff(uint8_t *send_buff, int send_buff_length)
{
  uint32_t err_code;

  err_code = ble_nus_data_send(&m_nus, send_buff, (uint16_t *)&send_buff_length, m_conn_handle);
  for ( uint32_t k = 0xfffff ; k ; k--); 
  if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
  {
    APP_ERROR_CHECK(err_code);
  }  
  return err_code;
  
} 

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint32_t err_code;
    //uint32_t uart_err_code;
    //ret_code_t ret_val;
    //uint8_t ACK_STRING[]="OK";
    char ret_buff[TX_BUFFER];
    EXISTANCE cmd_exist;
    int16_t ret_buff_length;
    uint16_t ret_buff_fix[1];
    int ret_buff_length_index;
    //int temp_index;
    //int temp;
    uint8_t * p_data;
    uint16_t length;
    ret_buff_fix[0] = 100;

    p_data = (uint8_t *) p_evt->params.rx_data.p_data;
    length = p_evt->params.rx_data.length;
    p_data[length] = 0;
    p_data[length+1] = 0;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
         
      //do{         
          cmd_exist = process_uart_command((char *)p_data, length , ret_buff , (int *)&ret_buff_length);
          if ( cmd_exist == EXIST )
          {
              on_ble_evt_connected_coundown += 10;
              if (ret_buff_length < TX_BUFFER)
              {
                    ret_buff_length_index = 0;
                 
                    for (ret_buff_length_index = 0 ; ret_buff_length > 100 ; ret_buff_length-=100)
                    {
                        err_code = ble_nus_data_send(&m_nus, (uint8_t *)&ret_buff[ret_buff_length_index], ret_buff_fix, m_conn_handle);
                        for ( uint32_t k = 0xfffff ; k ; k--);
                        ret_buff_length_index +=100;
                        if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                        {
                            APP_ERROR_CHECK(err_code);
                        }
    }
                    if( ret_buff_length > 0 )
                    {                                                                                
                        err_code = ble_nus_data_send(&m_nus, (uint8_t *)&ret_buff[ret_buff_length_index], (uint16_t *)&ret_buff_length, m_conn_handle);
                        for ( uint32_t k = 0xfffff ; k ; k--);
                        if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    }
                }
          }
          memset(p_data,0x0,length);
      //} while (err_code == NRF_ERROR_BUSY);
    }
}
/**@snippet [Handling the data received over BLE] */


#endif
unsigned char buffer_test[6][20] = {
  {'a','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'},
  {'b','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1'},
  {'c','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2','2'},
  {'d','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3','3'},
  {'w','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4','4'},
  {'x','5','5','5','5','5','5','5','5','5','5','5','5','5','5','5','5','5','5','5'}};

void sending_ble_data()
{ 
      int i= 0;
      uint32_t           err_code;
      //if (start_sending_streaming == 1)
      {
          for( i=0; i<6; i++)
          {
              //ble_nus_string_send(&m_nus, buffer_test[i],  20);
              do
              {
                  uint16_t length = 20;
                  err_code = ble_nus_data_send(&m_nus, buffer_test[i],&length, m_conn_handle);
                  if ((err_code != NRF_ERROR_INVALID_STATE) &&
                      (err_code != NRF_ERROR_RESOURCES) &&
                      (err_code != NRF_ERROR_NOT_FOUND))
                  {
                      APP_ERROR_CHECK(err_code);
                  }
              } while (err_code == NRF_ERROR_RESOURCES);
              //nrf_delay_ms(5);
          }
      }
      //nrf_delay_ms(10);
}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
extern bool update_msg;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    if ( p_ble_evt->header.evt_id != BLE_GAP_EVT_CONNECTED)
       update_msg = 0;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:            
            //NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:            
            //NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ble_device_during_session = 0;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            ble_device_during_session = 0;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            ble_device_during_session = 0;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    //NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
     //             p_gatt->att_mtu_desired_central,
     //             p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
               
#define UART_RX_BUFF_SIZE 256
uint8_t data_array[UART_RX_BUFF_SIZE];
int index_data_array = 0;

void reset_uart_rx_buff()
{
   memset(data_array,0x0,UART_RX_BUFF_SIZE);
   app_uart_flush();
   index_data_array = 0;
}

void gets_uart_rx_buff(char * ret_buf)
{
    memcpy( ret_buf, data_array, index_data_array);
    reset_uart_rx_buff();        
}

int gets_uart_rx_buff_without_clr(char * ret_buf)
{
    memcpy( ret_buf, data_array, index_data_array);
    return index_data_array;
}
char output_msg[64];
int output_length = 0;
extern void push_event(uint8_t *str_event , int16_t str_event_len);

void modem_uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index_data_array]));
            index_data_array++;                       
            
            if((sys_info.modem_state == MODEM_TRANSMMISION_BUFFER || sys_info.modem_state == MODEM_WAIT_FOR_COMMAND)&& data_array[index_data_array-1] == '\n')
            {
                if(data_array[index_data_array-1] == '\n')
                  index_data_array--;
                if(data_array[index_data_array-1] == '\r')
                  index_data_array--;
                
                if (process_uart_command( (char *)data_array, index_data_array , output_msg , &output_length) == EXIST)
                {
                  sys_info.countdown_sleep_seconds = 15;
                  //send_modem_packet((uint8_t *)output_msg,strlen(output_msg) , 0); 
                  push_event((uint8_t *)output_msg , strlen(output_msg));
                  //put_command(output_msg,0);
                }
                //Asaf
                //index_data_array = 0;
            }
                
            if (index_data_array > UART_RX_BUFF_SIZE )
            {   
                index_data_array = 0;
            }
                        
              
            break;

        case APP_UART_COMMUNICATION_ERROR:
            reset_uart_rx_buff();// APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            reset_uart_rx_buff();//APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

void modem_uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       modem_uart_event_handle,
                       3/*APP_IRQ_PRIORITY_HIGHEST APP_IRQ_PRIORITY_LOWEST*/,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
            {
               // NRF_LOG_DEBUG("Ready to send data over BLE NUS");
               // NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) &&
                         (err_code != NRF_ERROR_NOT_FOUND) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization]  */
#if 0
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
#endif
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void advertising_init_low(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL_LOW;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{    
    bsp_event_t startup_event;
    
#ifdef IMPROVING_EDITING_VER 
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler); 
    APP_ERROR_CHECK(err_code);
#else    
    uint32_t err_code = bsp_init(BSP_INIT_LEDS , bsp_event_handler); 
    APP_ERROR_CHECK(err_code);
#endif

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
*/

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
extern void init_water_sense(void);
extern void update_water_sense(void);
uint32_t uicr_temp = 5;

void init_reset_pin()
{
    if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
        ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))){
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_UICR->PSELRESET[0] = 18;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_UICR->PSELRESET[1] = 18;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        wrap_NVIC_SystemReset();
    }
}

void vddInit()
{
  if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V3) 
  {
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;    //write enable
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3;                        //configurate REGOUT0
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        wrap_NVIC_SystemReset();                                               // Reset device
  } 
}
/*
void reset_reason(void)
{
    
    //Reset reason 
    uint32_t rr = nrf_power_resetreas_get();  
    
    NRF_LOG_INFO("Reset reasons:");
    if (0 == rr)
    {
        NRF_LOG_INFO("- NONE");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK))
    {
        NRF_LOG_INFO("- RESETPIN");
        hitless_ptr()->reset_reason = POWER_ON_RESET;
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK     ))
    {
        NRF_LOG_INFO("- DOG");    
        hitless_ptr()->reset_reason = WATCHDOG_RESET;
    }
    if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK    ))
    {
      if (hitless_ptr()->reset_reason != POWER_ON_RESET && counter == 1)
      {
        NRF_LOG_INFO("- SREQ");
        hitless_ptr()->reset_reason = SOFTWARE_RESET;
      }
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LOCKUP_MASK  ))
    {
        NRF_LOG_INFO("- LOCKUP");
        hitless_ptr()->reset_reason = SOFTWARE_RESET;
    }
    if (0 != (rr & NRF_POWER_RESETREAS_OFF_MASK     ))
    {
        NRF_LOG_INFO("- OFF");
        hitless_ptr()->reset_reason = WAKE_UP_RESET;
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LPCOMP_MASK  ))
    {
        NRF_LOG_INFO("- LPCOMP");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DIF_MASK     ))
    {
        NRF_LOG_INFO("- DIF");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_NFC_MASK     ))
    {
        NRF_LOG_INFO("- NFC");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_VBUS_MASK    ))
    {
        NRF_LOG_INFO("- VBUS");
    }
    nrf_power_resetreas_clear(0xFFFFFFFF);
}*/
extern void nrf_delay_ms_wdt(uint32_t x_msec);
void TI_reset(){
  
    hitless_ptr()->initTIDone = 0;
    nrf_gpio_cfg_output(MSP430_RES);
    nrf_gpio_pin_clear(MSP430_RES);
    nrf_delay_ms(10);
    nrf_gpio_pin_set(MSP430_RES);
    nrf_delay_ms_wdt(100);
}

#define HARDWARE_WATCHDOG_TIMEOUT_SECS 30
void wdt_init(void)
{
#ifndef DEBUG_4G
      //Configure WDT.
      NRF_WDT->CONFIG         =  WDT_CONFIG_BEHAVIOUR;           // Configure WDT to run when CPU is asleep
      NRF_WDT->CRV = 32768 * HARDWARE_WATCHDOG_TIMEOUT_SECS;     // CRV = timeout * 32768 + 1
      NRF_WDT->RREN           = 0x01;                            // Enable the RR[0] reload register
      NRF_WDT->TASKS_START    = 1;                               // Start WDT       
#endif      
}

void wdt_feeding(void){
//#if 0
#ifndef DEBUG_4G  
   const uint32_t reload_magic_value = 0x6E524635;
   NRF_WDT->RR[0] = reload_magic_value; // "feed"  WDT
#endif 
//#endif 
}

APP_TIMER_DEF(operation_mode_timer_id); 
bool go_to_sleep = 0;

int task_main();
int task_modem();

typedef enum TASK_ID_
{
  TASK_MAIN,
  TASK_MODEM,
  NUM_OF_TASK
}TASK_ID;

#define UPDATE_BUFF_SIZE 5

#define BOOT_TO_DFU 1232
#define BOOT_TO_APP 1233
TASK_ID starting_task_ID = TASK_MAIN;
extern void CP(char x);
extern void init_ADC();
void main_operation_mode_task(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    //bool erase_bonds;
#ifdef DEBUG_COMMU  
    LEDS_INVERT(BSP_LED_1_MASK);
#endif
    sys_info.sysuptime++;  
   
    wdt_feeding();  // "feed"  WDT 
    
    //FORCE TO ENTER TO THE BOOT after CMD command
    if(hitless_ptr()->DFU_flag == BOOT_TO_DFU){
      NVIC_SystemReset ();
    }
    
    switch ( hitless_ptr()->device_state )  
    {
      case WORKING_DEVICE_STATE:
         //power modem off
        nrf_gpio_pin_clear(MODEM_POWER_ON);
        nrf_gpio_pin_clear(MODEM_RESET);
        
       case INIT_WORKING_DEVICE_STATE:
        starting_task_ID = TASK_MAIN;
        
        if( hitless_ptr()->BLE_main_task_countdown > 0 ){
           hitless_ptr()->BLE_main_task_countdown--;
           if( hitless_ptr()->BLE_main_task_countdown == 0 ){
             hitless_ptr()->device_state = SLEEP_STATE; //MO_0
             nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
             wrap_NVIC_SystemReset();   
           }
        }
        break;

      case SLEEP_STATE:
      case INIT_SLEEP_STATE:
        starting_task_ID = TASK_MAIN;
        
        //for "CT" command. 
        if ( hitless_ptr()->drip_test_stages == 0 
            && hitless_ptr()->send_critical_msg==0 
            && hitless_ptr()->check_message_coutdown > 5){
            hitless_ptr()->drip_test_stages = 1;               //start the leak test.
            hitless_ptr()->device_state = WORKING_DEVICE_STATE;//MO_1
            wrap_NVIC_SystemReset ();  
         }
        
        //SD command-the user ask to stop the pizuz detection.
        if(hitless_ptr()->detect_pizuz_flags[2]>0) 
          hitless_ptr()->detect_pizuz_flags[2]--;
        else if(hitless_ptr()->detect_pizuz_flags[2]==0){
        //countdown timer of pizuz detection.
          if(hitless_ptr()->detect_pizuz_flags[0] > 0 && hitless_ptr()->check_message_coutdown > 5 )
              hitless_ptr()->detect_pizuz_flags[0]--;
          if( hitless_ptr()->detect_pizuz_flags[0] == 0 
             && hitless_ptr()->send_critical_msg == 0 
             && hitless_ptr()->check_message_coutdown > 5 
             && hitless_ptr()->detect_pizuz_flags[1] == 0 )
          {
#ifdef MAX3510_SENSOR
             hitless_ptr()->device_state = WORKING_DEVICE_STATE;//MO_1
             hitless_ptr()->detect_pizuz_flags[1] = UPDATE_BUFF_SIZE;          //start the pizuz detection     
#elif TI_SENSOR
             hitless_ptr()->detect_pizuz_flags[1] = UPDATE_BUFF_SIZE;         //start the pizuz detection
#endif
             wrap_NVIC_SystemReset();
          }
        }

        //"FC" critical msg has sended.
        if(hitless_ptr()->flow_controller == 1 && hitless_ptr()->check_message_coutdown > 5){
          hitless_ptr()->device_state = WORKING_DEVICE_STATE;//MO_1
          wrap_NVIC_SystemReset ();     
        }
        if(hitless_ptr()->ADC > 1)//reading battery voltage counter
          hitless_ptr()->ADC--;
        
#ifndef DBG_BUFFER_NOT_EMPTY
        if( event_check_message() || event_buffer_upload())
#endif      
        {
#ifdef DEBUG_BYPASS_MODEM
          while (event_buffer_not_empty())
            pop_event();                  
        wrap_NVIC_SystemReset ();
#endif          
            hitless_ptr()->device_state = PRE_SENDING_VIA_CELLULAR_MODEM;          
            wrap_NVIC_SystemReset ();
        }
        else
        {          
#ifdef DEBUG_COMMU           
              nrf_gpio_pin_clear(MODEM_POWER_ON);
              nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
              nrf_gpio_pin_clear(MODEM_RESET);
#endif              
        }
        break;
        
      case PRE_SENDING_VIA_CELLULAR_MODEM:
         starting_task_ID = TASK_MODEM;
         hitless_ptr()->device_state = SENDING_VIA_CELLULAR_MODEM;
         break;
         
      case SENDING_VIA_CELLULAR_MODEM:   
         if (modem_handller() == MODEM_FINISH )
         {
           hitless_ptr()->device_state = SLEEP_STATE;
           //power modem off
           nrf_gpio_pin_clear(MODEM_POWER_ON);
           nrf_gpio_pin_clear(HV_EN); //for new hardware version HV_EN 
           nrf_gpio_pin_clear(MODEM_RESET);
#ifndef IMPROVING_EDITING_VER 
           nrf_gpio_pin_clear(VALVECAP_ON);  // Avraham Experimental hardware version - CP(0).
#endif
           starting_task_ID = TASK_MAIN;            
           wrap_NVIC_SystemReset ();
         }
         else
         {
           starting_task_ID = TASK_MODEM;
         }
         break;
      
      default:
        starting_task_ID = TASK_MAIN;
        wrap_NVIC_SystemReset ();
        break;
    }
} 

static bool run_time_updates = false;
void print_current_time()
{
    char time_string[120];
    nrf_cal_get_time_string(false ,time_string);
    //printf("Uncalibrated time:\t%s\r\n", time_string);
    nrf_cal_get_time_string(true , time_string);
    //printf("Calibrated time:\t%s\r\n", time_string);
}


void calendar_updated()
{
    if(run_time_updates)
    {
        print_current_time();
    }
}

int main(void)
{
    RESET_REASON rr = check_reset_reason();
    nrf_gpio_cfg_output(HV_EN); //for new hardware version 2.0 HV_EN 
    nrf_gpio_pin_set(HV_EN);
  
    hitless_ptr()->DFU_flag = BOOT_TO_APP;
    
    if ( hitless_ptr()->reset_reason == EXTERNAL_RESET || hitless_ptr()->reset_reason == WATCHDOG_RESET 
        || hitless_ptr()->reset_reason <= POWER_ON_RESET || hitless_ptr()->reset_reason >= WAKE_UP_RESET )
    {
      /* after POWER ON RESET INIT */
      memset((void *)hitless_ptr() , 0x0 , sizeof(HITLESS_STRUCTURE));
      events_stack_init(POWER_ON_RESET);
      hitless_ptr()->reset_reason =  rr;
      
      if (hitless_ptr()->reset_reason == EXTERNAL_RESET || 
          hitless_ptr()->reset_reason <= POWER_ON_RESET || hitless_ptr()->reset_reason >= WAKE_UP_RESET){
          hitless_ptr()->BLE_main_task_countdown = 100;
          hitless_ptr()->device_state = WORKING_DEVICE_STATE;
       }
      
      /* Init Variables */
     
      sprintf( hitless_ptr()->phone_number_of_administrator,"+972529439589");  // server number   
      hitless_ptr()->detect_pizuz_flags[0] = 30;                               // the first time the sensor is reading 
      //hitless_ptr()->detect_pizuz_flags[2] = 3000;                           // SD counter (3000~50)
      hitless_ptr()->detection_limit = 800;                                    // water threshold for pizuz(Liter per Hour)
      hitless_ptr()->unusage_limit = 10;                                        // water threshold for unusage alert(Liter per Hour)
      hitless_ptr()->drip_test_limit = 6;                                      // ( 0.000002*LITER_PER_HOUR_CONVERSION ) threshold of leakage
      hitless_ptr()->limit_count_result[0] = 50;                               // num of times go over threshold, untill detection of pizuz(Half an hour)
      hitless_ptr()->unusage_limit_counter[0] = 1300;                          // num of times go over threshold, untill unusage alert is on(12 h)
      hitless_ptr()->leak_counter[0] = 100;                                    // num of times go over threshold, untill leakage alert is on(4.5 h)
      hitless_ptr()->message_coutdown_interval = 16000;                        // KP every +-6 hours
      hitless_ptr()->check_message_coutdown = 20;                              // first time of modem setion in 1 min
      hitless_ptr()->flow_controller  = -1;                                    // FC off
      hitless_ptr()->drip_test_stages = -1;                                    // CT 0 off
      hitless_ptr()->change_flow_sign = 1;                                     // the flow data duplicate at 1
      hitless_ptr()->ADC = 1;
      hitless_ptr()->modem_signal_quality = -1;                                //start on -1(no signal)
      
      //power modem off
      nrf_gpio_pin_clear(MODEM_POWER_ON);
      nrf_gpio_pin_clear(MODEM_RESET);
      
      wrap_NVIC_SystemReset();
    }
    wdt_init();  //Configure WDT.
    
    if(  hitless_ptr()->device_state == SENDING_VIA_CELLULAR_MODEM)
    {
      hitless_ptr()->device_state = PRE_SENDING_VIA_CELLULAR_MODEM;
      wrap_NVIC_SystemReset();
    }
    nrf_cal_init();
    nrf_cal_set_callback(calendar_updated, 1);
    nrf_cal_init_set_params();                 
    
#ifndef DEBUG_COMMU    
    //vddInit();
#endif
    init_reset_pin(); 

    Sensor_Init( );

    // Initialize.  
    main_operation_mode_task(NULL);
    if ( starting_task_ID == TASK_MODEM )
      task_modem();
    else      
      task_main();
    
}

extern void nrf_delay_ms_wdt(uint32_t x_msec);
void init_all_output_as_input()
{
    nrf_gpio_cfg_input(MODEM_RESET, NRF_GPIO_PIN_PULLDOWN);
    //nrf_gpio_cfg_input(LED_3, NRF_GPIO_PIN_PULLDOWN);
    //nrf_gpio_cfg_input(LED_4, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(1,0), NRF_GPIO_PIN_PULLDOWN);
}
extern void change_water_flow(char param);
int task_main()
{   
    bool     erase_bonds;  
    
    if (hitless_ptr()->device_state == WORKING_DEVICE_STATE ) 
    {
#ifdef MAX3510_SENSOR
      init_water_sense();
#endif    
      LEDS_ON(BSP_LED_0_MASK);//Green off
      LEDS_OFF(BSP_LED_1_MASK);//Red off
    }
    
    DenitI2CBUS( );
//    if( hitless_ptr()->detect_pizuz_flags[1] == UPDATE_BUFF_SIZE ){// on first sensor reading 
//        /* RESET TI*/
//         TI_reset(); 
//    }
    system_info_init();
    //uart_init();       
    timers_init();
    app_timer_create(&operation_mode_timer_id,APP_TIMER_MODE_REPEATED,main_operation_mode_task); 
    app_timer_start(operation_mode_timer_id, 32768, NULL);    
#ifdef DEBUG_COMMU    
    buttons_leds_init(&erase_bonds);
#else    
    if (hitless_ptr()->device_state != SLEEP_STATE )
    {
        buttons_leds_init(&erase_bonds);
    }
    else
    {   
      init_all_output_as_input();
      //reading battery voltage every -+6 hours
      if( hitless_ptr()->ADC == 1 )
         init_ADC();
    }
#endif    
    power_management_init();
    ble_stack_init();
    services_init();   
    
        
      gap_params_init();
      gatt_init();
      if ( hitless_ptr()->BLE_main_task_countdown > 0 ) 
        advertising_init();
      else 
        advertising_init_low();
    
      conn_params_init();
    
      // Start execution.
      advertising_start();
    
    // Enter main loop.
    for (;;)
    {       
        //idle_state_handle();
      
        //for "FC" command
        if(( hitless_ptr()->flow_controller == 0 || hitless_ptr()->flow_controller == 1) && (hitless_ptr()->device_state == WORKING_DEVICE_STATE)){
          if (hitless_ptr()->flow_controller == 0){
            change_water_flow('0');}
          else if (hitless_ptr()->flow_controller == 1){
            change_water_flow('1');}
          hitless_ptr()->flow_controller  = -1;
          //MO_0
          hitless_ptr()->device_state = SLEEP_STATE;
          wrap_NVIC_SystemReset ();              
        } 
            
        if ( ble_device_during_session == 1 ||  hitless_ptr()->drip_test_stages > 0 || hitless_ptr()->detect_pizuz_flags[1] > 0 || hitless_ptr()->BLE_main_task_countdown > 0/* || hitless_ptr()->ADC > 2*/ )
        {
//          if( hitless_ptr()->ADC > 2 && hitless_ptr()->BLE_main_task_countdown <= 0 ){
//             StartTiRead();//enable
//             nrf_delay_ms(100);
//             for( int j=0 ; j < 1 ; j++ ){
//               TiReadData_byI2c_command();//reading and print by uart.
//               nrf_delay_ms_wdt(125);
//              }
//              StopTiRead();//disable
//              hitless_ptr()->ADC --;
//          }
//           if (ble_device_during_session == 1){
//#ifdef TI_SENSOR
//             StartTiRead();//enable
//             nrf_delay_ms(100);       
//             TiReadData_byI2c_command();//reading and print by uart.
//             nrf_delay_ms(125);          
//             StopTiRead();//disable
//#endif  
//           }
            /* update water sensor */
           if( hitless_ptr()->detect_pizuz_flags[1] > 0 ){
#ifdef MAX3510_SENSOR 
           nrf_gpio_pin_set(HV_EN);
           nrf_delay_ms(10);  // moderate the voltage drop
           nrf_gpio_pin_set(SENSOR_ON); 
           update_water_sense();  
#elif TI_SENSOR
            update_water_sense_ti();
#endif     
            }
            /*leds on*/
            if (hitless_ptr()->BLE_main_task_countdown <= 0){
               LEDS_OFF(BSP_LED_0_MASK);//Green off    
            }
            else{
              nrf_gpio_pin_set(HV_EN);
              nrf_delay_ms(10);  // moderate the voltage drop
              nrf_gpio_pin_set(SENSOR_ON); 
              LEDS_ON(BSP_LED_0_MASK);//Green on 
            }
        }   
        else         
        {
           nrf_gpio_pin_clear(SENSOR_ON);
           nrf_gpio_pin_clear(HV_EN); //for new hardware version 2.0 HV_EN 
           idle_state_handle();         
        }  
    }
}

void main_operation_mode_task();
APP_TIMER_DEF(operation_gateway_mode_timer_id); 

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
   uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int task_modem(void)
{
      uint32_t err_code;  
      bool     erase_bonds;

      modem_init();
      // nrf_delay_ms(1000);         // wait for 1 Sec to wifi finish init transmission
      
  
      ble_stack_init();
      services_init();


      err_code = app_timer_init();
      APP_ERROR_CHECK(err_code);
      
      app_timer_create(&operation_gateway_mode_timer_id,APP_TIMER_MODE_REPEATED,(void(*)(void *))&main_operation_mode_task); 
      err_code = app_timer_start(operation_gateway_mode_timer_id, 1000, NULL);    
      APP_ERROR_CHECK(err_code);
      
      
      buttons_leds_init(&erase_bonds);
      
      
    for (;;)
    {
      power_manage();
    }
}
