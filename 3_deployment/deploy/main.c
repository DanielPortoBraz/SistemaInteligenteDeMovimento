#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "config.h"
#include "include/mpu6500.h"
#include "include/features.h"
#include "include/ai_core.h" 

/* ---------- LEDs ---------- */
#define LED_R 13
#define LED_G 11
#define LED_B 12

static inline void set_led(bool r, bool g, bool b) {
    gpio_put(LED_R, r);
    gpio_put(LED_G, g);
    gpio_put(LED_B, b);
}

int main() {
    stdio_init_all();

    /* ---------- Inicialização dos LEDs ---------- */
    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);

    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);

    gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);

    set_led(false, false, false);

    /* ---------- Inicialização I2C ---------- */
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    sleep_ms(2000);
    printf("Sistema em C iniciando...\n");

    /* ---------- Inicialização dos módulos ---------- */
    mpu6500_init();

    ai_init();

    /* ---------- Variáveis ---------- */
    WindowBuffer janela;
    window_init(&janela);

    int16_t accel[3], gyro[3];
    float features[NUM_FEATURES];

    uint32_t last_time = 0;

    printf("Loop iniciado!\n");

    /* ---------- Loop principal ---------- */
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - last_time >= SAMPLE_INTERVAL_MS) {
            last_time = now;

            /* Ler sensor */
            mpu6500_read_data(accel, gyro);

            /* Adicionar à janela */
            window_add_sample(&janela, accel, gyro);

            /* Processar quando a janela estiver cheia */
            if (window_is_ready(&janela)) {
                extract_features(&janela, features);

                float confianca = 0.0f;
                const char* atividade = ai_run_inference(features, &confianca);

                printf("Atividade: %s (%.1f%%)\n", atividade, confianca);

                /* Feedback por LED */
                if (strcmp(atividade, "parado") == 0) {
                    set_led(true, false, false);

                } else if (strcmp(atividade, "caminhando") == 0) {
                    set_led(false, true, false);

                } else if (strcmp(atividade, "correndo") == 0) {
                    set_led(false, false, true);

                } else if (strcmp(atividade, "pulando") == 0) {
                    set_led(true, true, false);

                } else {
                    set_led(false, false, false);
                }
            }
        }

        /* Pequeno delay para reduzir uso de CPU */
        sleep_ms(1);
    }

    return 0;
}
