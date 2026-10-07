#include "ads8688.h"
#include <math.h>
#include "spi.h"

/* 手册：ads8688_en.pdf，SBAS582C（2015-04），以下p.均为印刷页码。
 * 阅读顺序：ADS_Init -> ADS_ReadRaw -> ADS_XferConversion -> ADS_Transfer。
 * 硬件：PB12=CS，PB13=SCLK，PB14=SDO，PB15=SDI；AIN1接模块P2-5。
 * 模块原理图p.1：RST由R6上拉，当前不外接C板3.3V；芯片RST/PD见手册p.33。
 */
#define ADS_CMD_NOOP 0x0000U          /* p.37、p.45：NO_OP，保持当前模式 */
#define ADS_CMD_RESET 0x8500U         /* p.44图87、p.45表6：软件复位程序寄存器 */
#define ADS_CMD_MANUAL_AIN1 0xC400U  /* p.42、p.45：MAN_Ch_1，选择AIN1 */
#define ADS_CHANNEL_PD_REG 0x02U      /* p.47、p.49：通道掉电寄存器 */
#define ADS_CHANNEL_PD_MASK (1U << ADS_INPUT_CHANNEL)
#define ADS_RANGE_REG 0x06U           /* p.47、p.51：AIN1量程寄存器 */
#define ADS_RANGE_UNIPOLAR 0x05U      /* p.51：0～2.5×VREF，内部VREF=4.096V */
#define ADS_FEATURE_REG 0x03U         /* p.47、p.50：SDO格式/功能寄存器 */
#define ADS_FEATURE_MASK 0xC7U        /* p.50表12：只比较位7:6和2:0，忽略保留位 */
#define ADS_FEATURE_TAG 0x01U         /* p.50表13：SDO=001，结果后附4位通道号 */
#define ADS_SPI_TIMEOUT_MS 10U        /* 本工程设置：单帧HAL传输超时10ms */

volatile uint8_t g_ads_spi_ok;
volatile ADS_Calibration g_ads_calibration;
static uint8_t ads_ready; /* static：仅本文件使用；初始化完成后才允许采样 */
static uint8_t ads_last_tag;
static uint8_t last_rx[5];
//底层通信
//只把字节发出去
static uint8_t ADS_Transfer(const uint8_t *tx, uint8_t size)
{
    uint8_t rx[5] = {0};
    /* p.11：CS拉低开始一帧，收发完成后拉高；寄存器帧3字节，转换帧5字节。 */
    HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, size,
                                                       ADS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
    for (uint8_t i = 0; i < 5U; ++i) last_rx[i] = rx[i];
    if (status != HAL_OK) {
        g_ads_spi_ok = 0U;
        return 0U;
    }
    return 1U;
}
//中间层
//把字节解析为命令帧与寄存器帧
static uint16_t ADS_XferConversion(uint16_t command)
{
    /* p.34：本帧CS下降沿采样，通道由上一帧选择；本帧内读出本次结果。
     * p.50表13/14：16位命令 + 16位结果 + 4位通道号，补齐为5字节(40时钟)。
     */
    const uint8_t tx[5] = {(uint8_t)(command >> 8), (uint8_t)command, 0U, 0U, 0U};
    if (!ADS_Transfer(tx, 5U)) return 0U;
    ads_last_tag = last_rx[4] >> 4; /* 第5字节高4位，AIN1应为1 */
    return ((uint16_t)last_rx[2] << 8) | last_rx[3]; /* 高字节左移8位，再合并低字节 */
}
//读寄存器
static uint8_t ADS_ReadRegister(uint8_t address)
{
    /* p.46图89：首字节=(7位地址<<1)|0(读)，再送两个0提供时钟，读回第3字节。 */
    const uint8_t tx[3] = {(uint8_t)(address << 1), 0U, 0U};
    if (!ADS_Transfer(tx, 3U)) return 0U;
    return last_rx[2];
}
//回读校验
static uint8_t ADS_ExpectValue(uint8_t actual, uint8_t expected, uint8_t mask)
{
    /* mask中为1的位才参与比较；传输失败时不接受接收缓冲中的值。 */
    if (!g_ads_spi_ok) return 0U;
    if ((actual & mask) != (expected & mask)) {
        g_ads_spi_ok = 0U;
        return 0U;
    }
    return 1U;
}
//写寄存器
static uint8_t ADS_WriteRegister(uint8_t address, uint8_t value, uint8_t mask)
{
    /* p.46图88：首字节=(地址<<1)|1(写)，第2字节写值，第3字节读回显。 */
    const uint8_t tx[3] = {(uint8_t)((address << 1) | 1U), value, 0U};
    if (!ADS_Transfer(tx, 3U)) return 0U;
    return ADS_ExpectValue(last_rx[2], value, mask);
}
//上层业务
//初始化
uint8_t ADS_Init(void)
{
    ads_ready = 0U;
    g_ads_spi_ok = 1U;
    HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
    /* 清理上一次超时后可能残留的数据、错误标志和HAL busy状态。 */
    if (HAL_SPI_Abort(&hspi2) != HAL_OK) {
        g_ads_spi_ok = 0U;
        return 0U;
    }
    /* p.10：内部参考启动典型15ms；工程在上电和软件复位后各留25ms等待。 */
    HAL_Delay(25U);
    ADS_XferConversion(ADS_CMD_RESET);
    if (!g_ads_spi_ok) return 0U;
    HAL_Delay(25U);

    /* 英文p.50表12保留位为000；旧中文p.48为101。保留读值，校验功能位。 */
    uint8_t reserved = ADS_ReadRegister(ADS_FEATURE_REG) & 0x38U;
    if (!g_ads_spi_ok) return 0U;
    if (reserved != 0U && reserved != 0x28U) {
        g_ads_spi_ok = 0U;
        return 0U;
    }
    if (!ADS_WriteRegister(ADS_RANGE_REG, ADS_RANGE_UNIPOLAR, 0xFFU)) return 0U;
    if (!ADS_ExpectValue(ADS_ReadRegister(ADS_RANGE_REG), ADS_RANGE_UNIPOLAR, 0xFFU))
        return 0U;
    /* p.49：AIN1掉电位必须为0，否则MAN_Ch_1会输出无效数据。 */
    if (!ADS_WriteRegister(ADS_CHANNEL_PD_REG, 0U, 0xFFU)) return 0U;
    if (!ADS_ExpectValue(ADS_ReadRegister(ADS_CHANNEL_PD_REG), 0U, ADS_CHANNEL_PD_MASK))
        return 0U;
    HAL_Delay(1U);
    if (!ADS_WriteRegister(ADS_FEATURE_REG, reserved | ADS_FEATURE_TAG, ADS_FEATURE_MASK))
        return 0U;
    if (!ADS_ExpectValue(ADS_ReadRegister(ADS_FEATURE_REG), ADS_FEATURE_TAG, ADS_FEATURE_MASK))
        return 0U;
    ADS_XferConversion(ADS_CMD_MANUAL_AIN1);
    if (!g_ads_spi_ok) return 0U;
    /* p.51表16：3Fh只返回命令高8位，C400h应读回C4h。 */
    if (!ADS_ExpectValue(ADS_ReadRegister(0x3FU), 0xC4U, 0xFFU)) return 0U;
    ads_ready = 1U;
    return 1U;
}
//运行时配置检查 定期查看芯片关键寄存器有没有被改写
uint8_t ADS_CheckConfiguration(void)
{
    if (!ads_ready) return 0U;
    g_ads_spi_ok = 1U;
    /* 运行中防止器件被复位或配置被改写；只检查AIN1相关寄存器。 */
    if (!ADS_ExpectValue(ADS_ReadRegister(ADS_CHANNEL_PD_REG), 0U, ADS_CHANNEL_PD_MASK)
        || !ADS_ExpectValue(ADS_ReadRegister(ADS_RANGE_REG), ADS_RANGE_UNIPOLAR, 0xFFU)
        || !ADS_ExpectValue(ADS_ReadRegister(ADS_FEATURE_REG), ADS_FEATURE_TAG, ADS_FEATURE_MASK)) {
        ads_ready = 0U;
        return 0U;
    }
    return 1U;
}
//读取原始码值
uint16_t ADS_ReadRaw(void)
{
    g_ads_spi_ok = ads_ready;
    if (!ads_ready) return 0U;
    /* p.42图83/84：先发MAN1选通道(丢弃其结果)，下一帧NO_OP采样并读出AIN1。 */
    ADS_XferConversion(ADS_CMD_MANUAL_AIN1);
    if (!g_ads_spi_ok) return 0U;
    uint16_t raw = ADS_XferConversion(ADS_CMD_NOOP);
    if (g_ads_spi_ok && ads_last_tag != ADS_INPUT_CHANNEL) g_ads_spi_ok = 0U;
    return raw;
}
//读电压
float ADS_ReadVoltage(void)
{
    /* p.51量程 + p.10参考值：V=raw*10.24/65536，5V约为32000码。
     * 此函数会重新采样；应用任务已取得raw，直接使用换算函数即可。
     */
    uint16_t raw = ADS_ReadRaw();
    return g_ads_spi_ok ? (float)raw * (2.5f * ADS_REFERENCE_VOLTAGE) / 65536.0f : NAN;
}
//原始码值转电压
float ADS_RawToDistance_mm(uint16_t raw)
{
    ADS_Calibration calibration = g_ads_calibration;
    if (calibration.enabled) {
        /* 本工程两点线性校准：raw_low->50mm，raw_high->10000mm；允许反向。 */
        int32_t span = (int32_t)calibration.raw_high - (int32_t)calibration.raw_low;
        if (span == 0) return NAN;
        return ADS_DISTANCE_LOW_MM + ((float)raw - calibration.raw_low)
            * (ADS_DISTANCE_HIGH_MM - ADS_DISTANCE_LOW_MM) / (float)span;
    }
    /* 本工程默认：距离(mm)=50+电压(V)*995，假定传感器0..10V对应50..10000mm。
     * 该距离关系不是ADS手册规定；量程0..10.24V来自p.51表15。
     */
    return ADS_DISTANCE_LOW_MM + (float)raw * (2.5f * ADS_REFERENCE_VOLTAGE)
        * ((ADS_DISTANCE_HIGH_MM - ADS_DISTANCE_LOW_MM) / 9.94f) / 65536.0f;
}
//获取距离
float ADS_GetDistance_mm(void)
{
    /* 便利接口：重新采样并换算；周期任务用已有raw换算，避免多采一次。 */
    uint16_t raw = ADS_ReadRaw();
    return g_ads_spi_ok ? ADS_RawToDistance_mm(raw) : NAN;
}
