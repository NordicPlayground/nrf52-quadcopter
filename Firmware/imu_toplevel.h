#ifndef IMU_H__
#define IMU_H__
#include "imu.h"
#include "include_common.h"
void imu_init(void);
void imu_periodic(void);
void imu_get_values(Axis3f *gyro, attitude *attitude);


#endif // IMU_H__
