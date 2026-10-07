#ifndef REVIEW_TASK_H
#define REVIEW_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *, TickType_t);
static inline uint32_t uxTaskGetStackHighWaterMark(void *task) { (void)task; return 256U; }
#endif
