#ifndef REVIEW_FREERTOS_H
#define REVIEW_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
