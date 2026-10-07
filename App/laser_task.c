#include "laser_task.h"
#include "../DRV+SEV/ads8688.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

/* 应用层：laser_task -> 每10ms调度 -> 每100ms检查配置/采AIN1/发布距离。
 * 周期、重试、距离端点均由工程决定；ADS通信页码见ads8688.c。
 */
volatile float g_distance_mm = NAN;
volatile uint16_t g_ads_ain1_raw;
volatile uint8_t g_ads_init_ok;
static uint8_t sample_count;
static uint8_t init_retry_count;
static uint8_t failed_samples;
static LaserSample published = {.distance_mm = NAN, .status = LASER_SAMPLE_NOT_READY};
//最新结果存入
static void publish_sample(const LaserSample *sample)
{
    /* 临界区内整组复制，避免其他任务读到新raw配旧距离；这里不做SPI通信。 */
    taskENTER_CRITICAL();
    published = *sample;
    g_distance_mm = sample->distance_mm;
    g_ads_ain1_raw = sample->raw;
    taskEXIT_CRITICAL();
}

void Laser_GetSample(LaserSample *sample)
{
    if (sample == NULL) return;
    taskENTER_CRITICAL();
    *sample = published;
    taskEXIT_CRITICAL();
    /* 仅使返回的快照失效，不改全局变量；无符号时间差可跨tick回绕。 */
    if ((TickType_t)(xTaskGetTickCount() - sample->tick) > pdMS_TO_TICKS(250U)) {
        sample->status |= LASER_SAMPLE_STALE;
        sample->distance_mm = NAN;
        sample->valid = 0U;
    }
}
//发布无数据包裹 在采集失败或者是重试时发送一个nan包裹
static void publish_unavailable(void)
{
    /* NAN表示没有可用距离；raw=0本身也可能是正常0V，应结合valid/status判断。 */
    LaserSample sample = {.distance_mm = NAN, .status = LASER_SAMPLE_NOT_READY,
                          .tick = xTaskGetTickCount()};
    publish_sample(&sample);
}
//初始化
void app_init(void)
{
    /* 本工程逻辑：启动时初始化ADS8688，失败后每秒自动重试。 */
    sample_count = 0U;
    publish_unavailable();
    g_ads_init_ok = ADS_Init();
    init_retry_count = 0U;
    failed_samples = 0U;
}
//10ms
void app_task_10ms(void)
{
    if (++sample_count < 10U) return;
    sample_count = 0U;
    /* 本工程逻辑：任务10ms运行一次，每10次采集一次，即100ms。 */
    if (!g_ads_init_ok) {
        /* 每100ms计一次，10次约1秒；恢复成功后下一轮再发布采样值。 */
        if (++init_retry_count >= 10U) {
            init_retry_count = 0U;
            g_ads_init_ok = ADS_Init();
            failed_samples = 0U;
        }
        publish_unavailable();
        return;
    }
    if (!ADS_CheckConfiguration()) {
        g_ads_init_ok = 0U;
        init_retry_count = 0U;
        publish_unavailable();
        return;
    }

    /* ADS手册只定义raw；距离和状态发布属于本工程应用层。 */
    LaserSample sample = {.distance_mm = NAN, .tick = xTaskGetTickCount()};
    sample.raw = ADS_ReadRaw();
    sample.valid = g_ads_spi_ok;
    if (!sample.valid) {
        sample.status |= LASER_SAMPLE_IO_ERROR;
        /* 单次失败输出NAN；连续3次失败再重新初始化，避免频繁复位。 */
        if (++failed_samples >= 3U) {
            g_ads_init_ok = 0U;
            init_retry_count = 0U;
            sample.status |= LASER_SAMPLE_NOT_READY;
        }
    } else {
        failed_samples = 0U;
        sample.distance_mm = ADS_RawToDistance_mm(sample.raw);
        if (!isfinite(sample.distance_mm)) sample.status |= LASER_SAMPLE_CALIBRATION_ERROR;
        if (sample.raw == 0U || sample.raw == 65535U) sample.status |= LASER_SAMPLE_ADC_RAIL;
    }
    publish_sample(&sample);
}
int a=0;
//绝对时间等待-执行时间包含
void laser_task(void *argument)
{
    (void)argument;
    app_init();
    TickType_t last_wake = xTaskGetTickCount();

    LaserSample my_package;


    for (;;) {
        /* 相对上次计划唤醒时间等待，减少执行耗时造成的周期漂移。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10U));
        app_task_10ms();
        TickType_t now = xTaskGetTickCount();
        /* 超时后重建周期基准，避免补跑压缩初始化重试间隔。 */
        if ((TickType_t)(now - last_wake) >= pdMS_TO_TICKS(10U)) last_wake = now;
        Laser_GetSample(&my_package);

        //示例使用-本应放置于其他任务中
        if (my_package.valid) {
            a=1;
        }

    }
}
