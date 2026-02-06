#ifndef CONFIG_H
#define CONFIG_H

// Hardware
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU6500_ADDR 0x68

// Modelo e Amostragem
#define WINDOW_SIZE 20
#define SAMPLE_INTERVAL_MS 50
#define NUM_FEATURES 14
#define NUM_CLASSES 4

#endif // CONFIG_H