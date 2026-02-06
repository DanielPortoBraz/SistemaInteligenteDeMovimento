#ifndef FEATURES_H
#define FEATURES_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef struct {
    int16_t ax[WINDOW_SIZE], ay[WINDOW_SIZE], az[WINDOW_SIZE];
    int16_t gx[WINDOW_SIZE], gy[WINDOW_SIZE], gz[WINDOW_SIZE];
    int index;
    bool is_full;
} WindowBuffer;

void window_init(WindowBuffer *win);
void window_add_sample(WindowBuffer *win, int16_t *accel, int16_t *gyro);
bool window_is_ready(WindowBuffer *win);
void extract_features(WindowBuffer *win, float *features_out);

#endif