#include "include/features.h"
#include <math.h>
#include <string.h>

// Funções matemáticas auxiliares
static float calc_mean(int16_t *data, int len) {
    float sum = 0;
    for(int i=0; i<len; i++) sum += data[i];
    return sum / len;
}

static float calc_std(int16_t *data, int len) {
    float mean = calc_mean(data, len);
    float sum_sq = 0;
    for(int i=0; i<len; i++) {
        float diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    return sqrtf(sum_sq / len);
}

static float calc_range(int16_t *data, int len) {
    int16_t min = data[0], max = data[0];
    for(int i=1; i<len; i++) {
        if(data[i] < min) min = data[i];
        if(data[i] > max) max = data[i];
    }
    return (float)(max - min);
}

static float calc_zcr(int16_t *data, int len) {
    float mean = calc_mean(data, len);
    int crossings = 0;
    for(int i=1; i<len; i++) {
        if (((data[i-1] - mean) > 0) != ((data[i] - mean) > 0)) {
            crossings++;
        }
    }
    return crossings / 2.0f;
}

// Implementação da Janela
void window_init(WindowBuffer *win) {
    win->index = 0;
    win->is_full = false;
    memset(win->ax, 0, sizeof(win->ax)); // Limpa memória
}

void window_add_sample(WindowBuffer *win, int16_t *accel, int16_t *gyro) {
    win->ax[win->index] = accel[0];
    win->ay[win->index] = accel[1];
    win->az[win->index] = accel[2];
    win->gx[win->index] = gyro[0];
    win->gy[win->index] = gyro[1];
    win->gz[win->index] = gyro[2];

    win->index++;
    if (win->index >= WINDOW_SIZE) {
        win->index = 0;
        win->is_full = true;
    }
}

bool window_is_ready(WindowBuffer *win) {
    return win->is_full;
}

void extract_features(WindowBuffer *win, float *f) {
    //Desvio Padrão
    f[0] = calc_std(win->ax, WINDOW_SIZE);
    f[1] = calc_std(win->ay, WINDOW_SIZE);
    f[2] = calc_std(win->az, WINDOW_SIZE);
    f[3] = calc_std(win->gx, WINDOW_SIZE);
    f[4] = calc_std(win->gy, WINDOW_SIZE);
    f[5] = calc_std(win->gz, WINDOW_SIZE);

    //Magnitude Média
    float sum_mag_a = 0, sum_mag_g = 0;
    for(int i=0; i<WINDOW_SIZE; i++) {
        sum_mag_a += sqrtf(win->ax[i]*win->ax[i] + win->ay[i]*win->ay[i] + win->az[i]*win->az[i]);
        sum_mag_g += sqrtf(win->gx[i]*win->gx[i] + win->gy[i]*win->gy[i] + win->gz[i]*win->gz[i]);
    }
    f[6] = sum_mag_a / WINDOW_SIZE;
    f[7] = sum_mag_g / WINDOW_SIZE;

    //Range e ZCR
    f[8] = calc_range(win->ax, WINDOW_SIZE);
    f[9] = calc_range(win->ay, WINDOW_SIZE);
    f[10] = calc_range(win->az, WINDOW_SIZE);
    f[11] = calc_zcr(win->ax, WINDOW_SIZE);
    f[12] = calc_zcr(win->ay, WINDOW_SIZE);
    f[13] = calc_zcr(win->az, WINDOW_SIZE);
}