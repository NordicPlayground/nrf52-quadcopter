#ifndef COMMAND_INPUT_H__
#define COMMAND_INPUT_H__



#include "include_common.h"
#include "stabilizer.h"

#define CMD_INPUT_MAX 65536
#define CMD_INPUT_MIN 0
#define CMD_OUTPUT_MAX 65536
#define CMD_OUTPUT_MIN 65536


uint32_t cmd_init(void);

attitude cmd_get_setpoints(void);
void cmd_on_ble_write(uint8_t *data, uint16_t length);

float cmd_get_throttle(void);
void set_attitude_setpoints(uint8_t * controldata);

#endif //COMMAND_INPUT_H__
