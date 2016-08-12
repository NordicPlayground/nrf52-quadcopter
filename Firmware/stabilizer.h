#ifndef STABILIZER_H__
#define STABILIZER_H__

#include "include_common.h"
#include "command_input.h"

#define SP_QUEUE_LENGTH 5
#define SP_QUEUE_SIZE 8
#define STABILIZER_INTERVAL_MS 1000/STAB_FREQUENCY
#define QUAD_FORMATION_X

void stab_init(void);
void set_calibration(void);
void cmd_set_reg_mode(uint8_t* mode_data);
void fail_safe_handler(void);
void set_user_control(void);
void fault_stop_motors(void);


#endif // STABILIZER_H__
