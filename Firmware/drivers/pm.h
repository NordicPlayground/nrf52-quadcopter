#ifndef PM_H
#define PM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum {
	current_500mA,
	current_100mA
} charge_current_t;

void pm_config(void);
void pm_check_status(void);
void pm_set_charge_current(charge_current_t max);

#endif
