/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#include "nrf_calendar.h"
#include "nrf.h"
#include "hitless_data.h"
#include "boards.h"


static struct tm time_struct, m_tm_return_time; 
//static time_t m_time; 
//static time_t m_last_calibrate_time = 0;
//static float m_calibrate_factor = 0.0f;
//static uint32_t m_rtc_increment = 60;
static void (*cal_event_callback)(void) = 0;
 
void nrf_cal_init(void)
{
    // Select the 32 kHz crystal and start the 32 kHz clock
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    
    // Configure the RTC for 1 minute wakeup (default)
    CAL_RTC->PRESCALER = 0xFFF;
    CAL_RTC->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    CAL_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    CAL_RTC->CC[0] = hitless_ptr()->rtc_var.m_rtc_increment * 8;
    CAL_RTC->TASKS_START = 1;
    NVIC_SetPriority(CAL_RTC_IRQn, CAL_RTC_IRQ_Priority);
    NVIC_EnableIRQ(CAL_RTC_IRQn);  
}

void nrf_cal_set_callback(void (*callback)(void), uint32_t interval)
{
    // Set the calendar callback, and set the callback interval in seconds
    cal_event_callback = callback;
    hitless_ptr()->rtc_var.m_rtc_increment = interval;
    hitless_ptr()->rtc_var.m_time += CAL_RTC->COUNTER / 8;
    CAL_RTC->TASKS_CLEAR = 1;
    CAL_RTC->CC[0] = interval * 8;  
}

void nrf_cal_init_set_params()
{
  struct tm *t = localtime(&hitless_ptr()->rtc_var.m_time);
  if(hitless_ptr()->rtc_var.m_time < 1000000 )
  {
      hitless_ptr()->rtc_var.m_last_calibrate_time = 0;
      hitless_ptr()->rtc_var.m_calibrate_factor = 0.0f;
      hitless_ptr()->rtc_var.m_rtc_increment = 60;  
      nrf_cal_set_time(1900,1,1,1,0,0); 
  }
  else
  {
   ;
  }
}

void nrf_cal_set_UNIX_timestemp(time_t set_timestemp )
{
    NVIC_DisableIRQ(CAL_RTC_IRQn); 
    hitless_ptr()->rtc_var.m_last_calibrate_time = hitless_ptr()->rtc_var.m_time = set_timestemp;
    NVIC_EnableIRQ(CAL_RTC_IRQn); 
}
    
time_t nrf_cal_get_UNIX_timestemp()
{
    return hitless_ptr()->rtc_var.m_time;
}

void nrf_cal_set_time(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second)
{
    static time_t uncal_difftime, difftime, newtime;
    time_struct.tm_year = year - 1900;
    time_struct.tm_mon = month;
    time_struct.tm_mday = day;
    time_struct.tm_hour = hour;
    time_struct.tm_min = minute;
    time_struct.tm_sec = second;   
    newtime = mktime(&time_struct);
    CAL_RTC->TASKS_CLEAR = 1; 
    NVIC_DisableIRQ(CAL_RTC_IRQn); 
    hitless_ptr()->rtc_var.m_last_calibrate_time = 0;
    hitless_ptr()->rtc_var.m_calibrate_factor = 0.0f;
    hitless_ptr()->rtc_var.m_rtc_increment = 60;  
    
    // Calculate the calibration offset 
    if(hitless_ptr()->rtc_var.m_last_calibrate_time != 0)
    {
        difftime = newtime - hitless_ptr()->rtc_var.m_last_calibrate_time;
        uncal_difftime = hitless_ptr()->rtc_var.m_time - hitless_ptr()->rtc_var.m_last_calibrate_time;
        hitless_ptr()->rtc_var.m_calibrate_factor = (float)difftime / (float)uncal_difftime;
    }
    
    // Assign the new time to the local time variables
    hitless_ptr()->rtc_var.m_time = hitless_ptr()->rtc_var.m_last_calibrate_time = newtime;
    NVIC_EnableIRQ(CAL_RTC_IRQn); 
}    

struct tm *nrf_cal_get_time(void)
{
    time_t return_time;
    return_time = hitless_ptr()->rtc_var.m_time + CAL_RTC->COUNTER / 8;
    m_tm_return_time = *localtime(&return_time);
    return &m_tm_return_time;
}

struct tm *nrf_cal_get_time_calibrated(void)
{
    time_t uncalibrated_time, calibrated_time;
    if(hitless_ptr()->rtc_var.m_calibrate_factor != 0.0f)
    {
        uncalibrated_time = hitless_ptr()->rtc_var.m_time + CAL_RTC->COUNTER / 8;
        calibrated_time = hitless_ptr()->rtc_var.m_last_calibrate_time + (time_t)((float)(uncalibrated_time - hitless_ptr()->rtc_var.m_last_calibrate_time) * hitless_ptr()->rtc_var.m_calibrate_factor + 0.5f);
        m_tm_return_time = *localtime(&calibrated_time);
        return &m_tm_return_time;
    }
    else return nrf_cal_get_time();
}

void nrf_cal_get_time_string(bool calibrated , char *cal_string)
{    
    strftime(cal_string, 80, " %d/%m/%y %H:%M:%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));    
}
 
void CAL_RTC_IRQHandler(void)
{
#ifdef DEBUG_COMMU
    LEDS_INVERT(BSP_LED_1_MASK);
#endif
    if(CAL_RTC->EVENTS_COMPARE[0])
    {
        CAL_RTC->EVENTS_COMPARE[0] = 0;
        
        CAL_RTC->TASKS_CLEAR = 1;
        
        hitless_ptr()->rtc_var.m_time += hitless_ptr()->rtc_var.m_rtc_increment;
        if(cal_event_callback) cal_event_callback();
    }
}
