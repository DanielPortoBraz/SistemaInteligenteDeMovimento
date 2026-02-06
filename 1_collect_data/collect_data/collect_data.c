#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Configurações I2C
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU6500_ADDR 0x68

// Registradores MPU-6500
#define MPU6500_WHO_AM_I 0x75
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_XOUT_H 0x3B

// Função para escrever em um registrador
void mpu6500_write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, buf, 2, false);
}

// Função para ler de um registrador
uint8_t mpu6500_read_register(uint8_t reg) {
    uint8_t data;
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ADDR, &data, 1, false);
    return data;
}

// Função para ler múltiplos registradores
void mpu6500_read_registers(uint8_t reg, uint8_t *buffer, uint8_t len) {
    i2c_write_blocking(I2C_PORT, MPU6500_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ADDR, buffer, len, false);
}

// Inicializar MPU-6500
bool mpu6500_init() {
    // Wake up MPU-6500 (sai do modo sleep)
    mpu6500_write_register(MPU6500_PWR_MGMT_1, 0x00);
    sleep_ms(100);
    
    // Verificar comunicação (WHO_AM_I deve retornar 0x70)
    uint8_t who_am_i = mpu6500_read_register(MPU6500_WHO_AM_I);
    if (who_am_i != 0x70) {
        printf("ERROR: MPU6500 not found! WHO_AM_I = 0x%02X\n", who_am_i);
        return false;
    }
    
    // Configurar acelerômetro: ±2g (mais sensível para atividades humanas)
    mpu6500_write_register(MPU6500_ACCEL_CONFIG, 0x00);
    
    // Configurar giroscópio: ±250°/s
    mpu6500_write_register(MPU6500_GYRO_CONFIG, 0x00);
    
    sleep_ms(100);
    printf("MPU6500 initialized successfully!\n");
    return true;
}

// Ler dados do acelerômetro e giroscópio
void mpu6500_read_data(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[14];
    
    // Ler 14 bytes a partir do registrador ACCEL_XOUT_H
    // Ordem: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, TEMP_H, TEMP_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
    mpu6500_read_registers(MPU6500_ACCEL_XOUT_H, buffer, 14);
    
    // Acelerômetro (combinar high e low bytes)
    accel[0] = (buffer[0] << 8) | buffer[1];  // AX
    accel[1] = (buffer[2] << 8) | buffer[3];  // AY
    accel[2] = (buffer[4] << 8) | buffer[5];  // AZ
    
    // Giroscópio
    gyro[0] = (buffer[8] << 8) | buffer[9];   // GX
    gyro[1] = (buffer[10] << 8) | buffer[11]; // GY
    gyro[2] = (buffer[12] << 8) | buffer[13]; // GZ
}

int main() {
    // Inicializar USB Serial
    stdio_init_all();
    
    // Configurar I2C
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Aguardar 3 segundos para abrir o Serial Monitor
    sleep_ms(3000);
    
    printf("\n=== MPU6500 Data Collector ===\n");
    printf("Waiting for MPU6500...\n");
    
    // Inicializar sensor
    if (!mpu6500_init()) {
        printf("Initialization failed! Check connections.\n");
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("\n=== READY TO COLLECT DATA ===\n");
    printf("Format: AX,AY,AZ,GX,GY,GZ\n");
    printf("Starting in 3 seconds...\n");
    sleep_ms(3000);
    
    int16_t accel[3];
    int16_t gyro[3];
    
    // Loop de coleta (50Hz = 20ms por amostra)
    while (true) {
        mpu6500_read_data(accel, gyro);
        
        // Imprimir em formato CSV (fácil de salvar e processar)
        printf("%d,%d,%d,%d,%d,%d\n", 
               accel[0], accel[1], accel[2],
               gyro[0], gyro[1], gyro[2]);
        
        sleep_ms(20); // 50Hz de amostragem
    }
    
    return 0;
}
