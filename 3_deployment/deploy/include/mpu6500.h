#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>

void mpu6500_init(void);
void mpu6500_read_data(int16_t *accel, int16_t *gyro);

#endif