
/* Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "nordic_common.h"
#include "nrf.h"
#include "hitless_data.h"
#include "nrf_power.h"


/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#ifdef HITLESS_STATIC_ADDRESS
#pragma location = HITLESS_START_ADDRESS
__no_init volatile HITLESS_STRUCTURE hitless_struct;
#else
HITLESS_STRUCTURE hitless_struct;
#endif
void delete_friend( int friend_name_index );


void reset_hitless_data()
{
  memset((char *)&hitless_struct,0,sizeof(HITLESS_STRUCTURE));
}

HITLESS_STRUCTURE * hitless_ptr(){
  return (HITLESS_STRUCTURE *)&hitless_struct;}

RESET_REASON  check_reset_reason(void) 
{ 
    uint32_t rr = nrf_power_resetreas_get();  

    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK )) 
    {
        hitless_ptr()->reset_reason = EXTERNAL_RESET;
    }
    else if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK ))
    {     
        hitless_ptr()->reset_reason = WATCHDOG_RESET;
    }
    else if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK ))
    {     
        hitless_ptr()->reset_reason = SOFTWARE_RESET;
    }
else {
    hitless_ptr()->reset_reason = POWER_ON_RESET;
    }
    nrf_power_resetreas_clear(0xFFFFFFFF);

    return hitless_ptr()->reset_reason;
//      uint32_t temp;
//	RESET_REASON power_on_reset_flag;
//        temp =  NRF_POWER->RESETREAS; 
//        if ( temp == 0 )
//          power_on_reset_flag = POWER_ON_RESET;
//        else
//          power_on_reset_flag = SOFTWARE_RESET; 
//     return power_on_reset_flag;
}  
