#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "config.h"
#include "include/mpu6500.h"
#include "include/features.h"
#include "include/ai_core.h" 

int main() {
    stdio_init_all();
    
    // Inicialização I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(2000);
    printf("Sistema em C Iniciando...\n");

    // Inicializa Módulos
    mpu6500_init();
    
    if (!ai_init()) {
        printf("Erro ao iniciar IA!\n");
        return -1;
    }

    // Variáveis
    WindowBuffer janela;
    window_init(&janela);
    
    int16_t accel[3], gyro[3];
    float features[NUM_FEATURES];
    uint32_t last_time = 0;

    printf("Loop iniciado!\n");

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_time >= SAMPLE_INTERVAL_MS) {
            last_time = now;
            
            // Ler Sensor
            mpu6500_read_data(accel, gyro);
            
            //Adicionar na Janela 
            window_add_sample(&janela, accel, gyro);
            
            // Processar
            if (window_is_ready(&janela)) {
                extract_features(&janela, features);
                
                //Chamar a IA
                float confianca = 0;
                const char* atividade = ai_run_inference(features, &confianca);
                
                printf("Atividade: %s (%.1f%%)\n", atividade, confianca);
            }
        }
        sleep_ms(1);
    }
    return 0;
}