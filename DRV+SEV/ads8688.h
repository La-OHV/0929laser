#ifndef INC_0929LASER_ADS8688_H
#define INC_0929LASER_ADS8688_H

#include <stdint.h>

/* 页码均指ads8688_en.pdf（SBAS582C）印刷页码；模块接口见原理图p.1。 */
#define ADS_INPUT_CHANNEL 1U       /* ADS8688通道号1，对应模块P2-5/pin18 */
#define ADS_REFERENCE_VOLTAGE 4.096f /* p.10：内部参考典型值4.096V */
#define ADS_DISTANCE_LOW_MM 100.0f    /* 应用端点：须与实际传感器设置一致 */
#define ADS_DISTANCE_HIGH_MM 3000.0f

/* 驱动仅由laser_task调用；其他任务通过Laser_GetSample读取结果。
 * g_ads_spi_ok包含SPI、寄存器和通道校验结果，raw=0不能单独表示失败。
 * extern是声明，变量实体在.c中；volatile不保证多个变量的一致性。
 */
extern volatile uint8_t g_ads_spi_ok;
uint8_t ADS_Init(void);             /* p.44～51：软件复位并配置AIN1；1成功/0失败 */
uint8_t ADS_CheckConfiguration(void); /* 本工程运行时配置检查 */
uint16_t ADS_ReadRaw(void);         /* p.42：仅采集AIN1 */
float ADS_ReadVoltage(void);

typedef struct {
    uint16_t raw_low;  /* ADS_DISTANCE_LOW_MM对应码值 */
    uint16_t raw_high; /* ADS_DISTANCE_HIGH_MM对应码值，可小于raw_low */
    uint8_t enabled;   /* 启用后端点相等返回NAN */
} ADS_Calibration;

/* 在采样任务停止时修改，或用临界区一次更新整个结构。 */
extern volatile ADS_Calibration g_ads_calibration;
float ADS_RawToDistance_mm(uint16_t raw); /* 默认0..10V线性映射，不额外读SPI */
float ADS_GetDistance_mm(void);

#endif
