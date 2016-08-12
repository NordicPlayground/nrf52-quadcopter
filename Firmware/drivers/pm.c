#include "pm.h"
#include "quadcopter.h"
#include "max17043.h"
#include "nrf_gpio.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "SEGGER_RTT.h"

charge_current_t charge_current = current_500mA;

void pm_config(void)
{
	max17043_config();
	
	nrf_gpio_cfg_input(PM_PGOOD, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PM_CHG, NRF_GPIO_PIN_PULLUP);
	
	nrf_gpio_cfg_output(PM_CH_EN);
	nrf_gpio_cfg_output(PM_EN1);
	nrf_gpio_cfg_output(PM_EN2);
	
	nrf_gpio_pin_clear(PM_CH_EN);
	pm_set_charge_current(current_500mA);
}

void pm_set_charge_current(charge_current_t current) 
{
	if(current == current_500mA) {
		nrf_gpio_pin_set(PM_EN1);
		nrf_gpio_pin_clear(PM_EN2);
		charge_current = current_500mA;
	} else {
		nrf_gpio_pin_clear(PM_EN1);
		nrf_gpio_pin_clear(PM_EN2);
		charge_current = current_100mA;
	}
}

void pm_check_status(void) 
{	
	if(!nrf_gpio_pin_read(PM_PGOOD)) {
		// TODO: make variable to control printing
		//SEGGER_RTT_printf(0,"Power source connected.\r\n");
		
		if(!nrf_gpio_pin_read(PM_CHG)) {
			led_blink();
			//SEGGER_RTT_printf(0,"Battery is charging at ");
			if(charge_current == current_500mA) {
				//SEGGER_RTT_printf(0,"500mA\r\n");
			} else {
				//SEGGER_RTT_printf(0,"100mA\r\n");
			}
			//SEGGER_RTT_printf(0,"Battery: %d \n",max17043_get_soc());
		} else {
			if(max17043_get_vcell() < 3.7f) {
				//SEGGER_RTT_printf(0,"Battery is not charging...\r\n");
			} else {
				//SEGGER_RTT_printf(0,"Battery is fully charged!\r\n");
				led_blink_clear();
			}
		}
		
	} else {
		//SEGGER_RTT_printf(0,"Power source disconnected...\r\n");
		//SEGGER_RTT_printf(0,"Battery: %.1f%% \r\n",max17043_get_soc());
	}
}
