/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "general.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "nordic_common.h"
#include "nrf.h"
#include "system_info.h"
#include "hitless_data.h"



SYSTEM_INFO sys_info;
uint32_t last_counter_50mSec;

system_info_Packet_t system_info_event;


int8_t calc_hour(uint32_t Unix_sysuptime )
{

  time_t curtime;
  struct tm *loctime;

  /* Get the current time. */
 
  curtime = (time_t)Unix_sysuptime;
  
  /* Convert it to local time representation. */
  loctime = localtime (&curtime);
  
  return (int8_t)loctime->tm_hour;
}


int8_t calc_hour_section(uint32_t Unix_sysuptime )
{
  time_t curtime;
  struct tm *loctime;

  /* Get the current time. */
 
  curtime = (time_t)Unix_sysuptime;
  
  /* Convert it to local time representation. */
  loctime = localtime (&curtime);
  return (int8_t)(loctime->tm_min/20);
}

int8_t calc_day_in_month(uint32_t Unix_sysuptime )
{
  time_t curtime;
  struct tm *loctime;

  /* Get the current time. */
 
  curtime = (time_t)Unix_sysuptime;
  
  /* Convert it to local time representation. */
  loctime = localtime (&curtime);
  return (int8_t)loctime->tm_mday;
}

int8_t calc_day_in_week(uint32_t Unix_sysuptime )
{
  time_t curtime;
  struct tm *loctime;

  /* Get the current time. */
 
  curtime = (time_t)Unix_sysuptime;
  
  /* Convert it to local time representation. */
  loctime = localtime (&curtime);
  return (int8_t)loctime->tm_wday;  
}


SYSTEM_INFO sys_info;


void system_info_init()
{
    uint8_t temp_uuid;
    sys_info.idCodeShort.uid[1] =  NRF_FICR->DEVICEADDR[1];
    sys_info.idCodeShort.uid[0] =  NRF_FICR->DEVICEADDR[0];
    
    temp_uuid = sys_info.idCodeShort.buff[5]|0xC0;
    sys_info.idCodeShort.buff[5] = temp_uuid;
}


void actiavte_flash_protection(int act_flag)
{
#ifdef FLASH_READ_PROTECT
        NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        if ( act_flag == 1)
            NRF_UICR->APPROTECT = 0xFFFFFF00;
        else
            NRF_UICR->APPROTECT = 0xFFFFFFFF;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
#endif
}
