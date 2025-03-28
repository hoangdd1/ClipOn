#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__
#include "nrf_drv_gpiote.h"
void gpio_init(void);
void saadc_gpio_init(void);
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

#endif
