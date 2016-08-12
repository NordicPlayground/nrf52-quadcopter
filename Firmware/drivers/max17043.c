#include "max17043.h"
#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "i2cdev.h"

#include "include_common.h"
#include "command_input.h"

void max17043_config(void) 
{
	max17043_reset();
	nrf_delay_ms(250);
	max17043_quick_start();
	nrf_delay_ms(250);
}

float max17043_get_vcell(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_register(VCELL_REGISTER, &MSB, &LSB);
	int value = (MSB << 4) | (LSB >> 4);
	//return map(value, 0x000, 0xFFF, 0, 50000) / 10000.0f;
	return (float)value * 0.00125f;
}

uint8_t max17043_get_soc(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_register(SOC_REGISTER, &MSB, &LSB);
	float decimal = LSB / 256.0f;
	return (MSB);	
}

int max17043_get_version(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_register(VERSION_REGISTER, &MSB, &LSB);
	return (MSB << 8) | LSB;
}

uint8_t max17043_get_compensate_value(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_config_register(&MSB, &LSB);
	return MSB;
}

uint8_t max17043_get_alert_threshold(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_config_register(&MSB, &LSB);	
	return 32 - (LSB & 0x1F);
}

void max17043_set_alert_threshold(uint8_t threshold) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_config_register(&MSB, &LSB);	
	if(threshold > 32) threshold = 32;
	threshold = 32 - threshold;
	
	max17043_write_register(CONFIG_REGISTER, MSB, (LSB & 0xE0) | threshold);
}

bool max17043_in_alert(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_config_register(&MSB, &LSB);	
	return LSB & 0x20;
}

void max17043_clear_alert(void) 
{
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	
	max17043_read_config_register(&MSB, &LSB);	
}

void max17043_reset(void) 
{
	max17043_write_register(COMMAND_REGISTER, 0x00, 0x54);
}

void max17043_quick_start(void) 
{
	max17043_write_register(MODE_REGISTER, 0x40, 0x00);
}

void max17043_read_config_register(uint8_t *MSB, uint8_t *LSB) 
{
	max17043_read_register(CONFIG_REGISTER, MSB, LSB);
}

void max17043_read_register(uint8_t start_address, uint8_t *MSB, uint8_t *LSB) 
{	
	uint8_t data[2];
	i2cdev_readBytes(MAX17043_ADDRESS,start_address,2,data);
	*MSB = data[0];
	*LSB = data[1];
}

void max17043_write_register(uint8_t address, uint8_t MSB, uint8_t LSB) 
{	
	uint8_t data[2];
  data[0] = MSB;
	data[1] = LSB;
	i2cdev_writeBytes(MAX17043_ADDRESS,address,2,data);
}
