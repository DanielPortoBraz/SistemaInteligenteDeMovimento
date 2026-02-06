#ifndef AI_CORE_H
#define AI_CORE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ai_init(void);
const char* ai_run_inference(float *features, float *confidence_out);

#ifdef __cplusplus
}
#endif

#endif