#ifndef INC_0929LASER_LASER_TASK_H
#define INC_0929LASER_LASER_TASK_H

#include <stdint.h>

#define LASER_SAMPLE_NOT_READY 0x01U /* 本工程状态，不是ADS8688寄存器位 */
#define LASER_SAMPLE_IO_ERROR 0x02U          /* SPI或通道校验失败 */
#define LASER_SAMPLE_CALIBRATION_ERROR 0x04U /* 校准参数不能产生有效距离 */
#define LASER_SAMPLE_ADC_RAIL 0x08U          /* raw为0或65535，仅提示端点，不等于故障 */
#define LASER_SAMPLE_STALE 0x10U             /* 距采样开始已超过250ms */

typedef struct {
    float distance_mm;
    uint16_t raw; /* AIN1原始码 */
    uint8_t valid; /* SPI传输及通道校验通过 */
    uint8_t status; /* 状态可叠加，用 status & LASER_SAMPLE_xxx 检查某一项 */
    uint32_t tick; /* 采样开始时RTOS tick；当前1 tick=1 ms */
} LaserSample;

/* 本工程接口：读取一致的AIN1快照；超过250ms未更新时返回STALE和NAN。 */
void Laser_GetSample(LaserSample *sample);
/* 单位mm；未就绪、通信失败或校准无效时为NAN。 */
extern volatile float g_distance_mm;
extern volatile uint16_t g_ads_ain1_raw; /* 有效性请读取Laser_GetSample */
extern volatile uint8_t g_ads_init_ok;

void app_init(void);
void app_task_10ms(void); /* 每10ms调用，内部每100ms采集AIN1 */
void laser_task(void *argument);

#endif
