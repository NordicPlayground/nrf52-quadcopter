#ifndef _MAX17043_H
#define _MAX17043_H

#include <stdbool.h>
#include <stdint.h>

//Device address
#define MAX17043_ADDRESS		0x36

//Internal registers
#define VCELL_REGISTER			0x02
#define SOC_REGISTER				0x04
#define MODE_REGISTER				0x06
#define VERSION_REGISTER		0x08
#define CONFIG_REGISTER			0x0C
#define COMMAND_REGISTER		0xFE

void max17043_config(void);
void max17043_reset(void);
void max17043_quick_start(void);

float max17043_get_vcell(void);
uint8_t max17043_get_soc(void);
int max17043_get_version(void);
uint8_t max17043_get_compensate_value(void);
uint8_t max17043_get_alert_threshold(void);
void max17043_set_alert_threshold(uint8_t threshold);
bool max17043_in_alert(void);
void max17043_clear_alert(void);

void max17043_read_config_register(uint8_t *MSB, uint8_t *LSB);
void max17043_read_register(uint8_t start_address, uint8_t *MSB, uint8_t *LSB);
void max17043_write_register(uint8_t address, uint8_t MSB, uint8_t LSB);

#endif
