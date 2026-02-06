#include "include/mpu6500.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// Função auxiliar interna 
static void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, buf, 2, false);
}

void mpu6500_init(void) {
    mpu_write(PWR_MGMT_1, 0x00); // Acorda o sensor
    sleep_ms(100);
}

void mpu6500_read_data(int16_t *accel, int16_t *gyro) {
    uint8_t reg = ACCEL_XOUT_H;
    uint8_t buffer[14];
    
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ADDR, buffer, 14, false);
    
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}