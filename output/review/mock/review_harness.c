/* Host regression of the AIN1 production sources; no electrical timing model. */
#include "spi.h"
#include "task.h"
#include "ads8688.h"
#include "laser_task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi2;
static uint8_t regs[64], reserved, selected, last_command;
static uint16_t raw;
static int cs_high = 1, fail_read, fail_command, bad_tag, corrupt_echo, disconnected;
static unsigned conversions, writes, resets, aborts, cases;
static uint32_t mock_tick;

void HAL_GPIO_WritePin(void *port, uint16_t pin, int state)
{
    (void)port; (void)pin;
    if (!state) assert(cs_high);
    cs_high = state;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi)
{
    (void)spi; ++aborts; return HAL_OK;
}
void HAL_Delay(uint32_t ms) { (void)ms; }
TickType_t xTaskGetTickCount(void) { return mock_tick; }
void vTaskDelayUntil(TickType_t *tick, TickType_t period) { *tick += period; }

static void device_reset(void)
{
    memset(regs, 0, sizeof(regs));
    regs[1] = 0xFF; regs[3] = reserved;
    selected = 0; last_command = 0;
}
static void scenario(uint8_t feature_reserved)
{
    reserved = feature_reserved; device_reset();
    raw = 32000; mock_tick = 0;
    fail_read = fail_command = -1;
    bad_tag = corrupt_echo = disconnected = 0;
    conversions = writes = resets = aborts = 0;
    g_ads_calibration = (ADS_Calibration){0};
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, const uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout)
{
    (void)spi;
    assert(!cs_high && timeout == 10);
    memset(rx, 0, size);
    if (disconnected) { memset(rx, 0xFF, size); return HAL_OK; }
    uint16_t cmd = (uint16_t)((tx[0] << 8) | tx[1]);
    if (cmd == 0x8500) {
        assert(size == 5);
        ++resets; device_reset(); return HAL_OK;
    }
    if (size == 3) {
        unsigned address = tx[0] >> 1;
        /* No other input range is read or written. */
        assert(address == 2 || address == 3 || address == 6 || address == 0x3F);
        assert(tx[2] == 0);
        if (tx[0] & 1) {
            assert(address != 0x3F);
            if (address == 6) assert(tx[1] == 5);
            if (address == 2) assert(tx[1] == 0);
            regs[address] = address == 3 ? ((tx[1] & 0xC7) | reserved) : tx[1];
            rx[2] = regs[address];
            if (corrupt_echo) { rx[2] ^= 1U; corrupt_echo = 0; }
            ++writes;
        } else {
            assert(tx[1] == 0);
            if (fail_read == (int)address) { fail_read = -1; return HAL_TIMEOUT; }
            rx[2] = address == 0x3F ? last_command : regs[address];
        }
        return HAL_OK;
    }
    assert(size == 5 && (cmd == 0xC400 || cmd == 0));
    assert(tx[2] == 0 && tx[3] == 0 && tx[4] == 0);
    ++conversions;
    if (fail_command == cmd) { fail_command = -1; return HAL_TIMEOUT; }
    if (cmd == 0) assert(selected == 1);
    rx[2] = (uint8_t)(raw >> 8); rx[3] = (uint8_t)raw;
    if ((regs[3] & 7) == 1) rx[4] = (uint8_t)((bad_tag ? 7 : selected) << 4);
    if (cmd == 0xC400) { selected = 1; last_command = 0xC4; }
    return HAL_OK;
}
static void scan(void) { for (int i = 0; i < 10; ++i) app_task_10ms(); }
static LaserSample snapshot(void)
{
    LaserSample sample;
    Laser_GetSample(&sample);
    assert(sample.raw == g_ads_ain1_raw);
    return sample;
}
static void pass(const char *name) { ++cases; printf("PASS: %s\n", name); }
int main(void)
{
    scenario(0x28); app_init();
    assert(g_ads_init_ok && writes == 3 && regs[3] == 0x29);
    unsigned before = conversions;
    for (int i = 0; i < 9; ++i) app_task_10ms();
    assert(conversions == before && isnan(g_distance_mm));
    app_task_10ms();
    assert(conversions - before == 2 && g_ads_ain1_raw == 32000 && snapshot().valid);
    assert(fabsf(g_distance_mm - 5025.0f) < 0.01f && cs_high);
    pass("legacy feature bits, AIN1-only configuration, 100ms cadence, MAN1+NOOP");

    scenario(0); app_init(); scan();
    assert(g_ads_init_ok && regs[3] == 1 && snapshot().valid);
    pass("newer feature readback 0x01 accepted");

    raw = 0; scan();
    assert(g_ads_ain1_raw == 0 && fabsf(g_distance_mm - 50) < 0.01f && snapshot().valid);
    assert(snapshot().status & LASER_SAMPLE_ADC_RAIL);
    raw = 64000; scan();
    assert(g_ads_ain1_raw == 64000 && fabsf(g_distance_mm - 10000) < 0.01f);
    raw = 32000;
    assert(fabsf(ADS_ReadVoltage() - 5.0f) < 0.001f);
    assert(fabsf(ADS_GetDistance_mm() - 5025.0f) < 0.01f);
    pass("0V/5V/10V scale, raw zero is valid, existing AIN1 service functions");

    scenario(0); app_init();
    raw = 31000; g_ads_calibration = (ADS_Calibration){1000, 61000, 1};
    before = conversions; scan();
    assert(conversions - before == 2 && fabsf(g_distance_mm - 5025) < 0.01f);
    assert(fabsf(g_distance_mm - ADS_GetDistance_mm()) < 0.01f);
    pass("calibration uses the published raw without an extra conversion");

    scenario(0); app_init();
    g_ads_calibration = (ADS_Calibration){61000, 1000, 1}; raw = 31000; scan();
    assert(fabsf(g_distance_mm - 5025) < 0.01f);
    assert(fabsf(ADS_RawToDistance_mm(61000) - 50) < 0.01f);
    assert(fabsf(ADS_RawToDistance_mm(1000) - 10000) < 0.01f);
    g_ads_calibration.raw_high = 61000; scan();
    assert(isnan(g_distance_mm) && (snapshot().status & LASER_SAMPLE_CALIBRATION_ERROR));
    pass("reverse calibration and equal-endpoint rejection");

    scenario(0); fail_read = 6; app_init();
    assert(!g_ads_init_ok);
    for (int i = 0; i < 99; ++i) app_task_10ms();
    assert(aborts == 1 && isnan(g_distance_mm));
    for (int i = 0; i < 11; ++i) app_task_10ms();
    assert(aborts == 2 && g_ads_init_ok && snapshot().valid);
    pass("startup timeout retries after one second");

    scenario(0); app_init(); regs[6] = 0;
    before = conversions; scan();
    assert(!g_ads_init_ok && isnan(g_distance_mm) && conversions == before);
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && regs[6] == 5 && snapshot().valid);
    pass("AIN1 range corruption invalidates output and recovers");

    scenario(0); app_init(); regs[2] = 2; scan();
    assert(!g_ads_init_ok && isnan(g_distance_mm));
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && regs[2] == 0 && snapshot().valid);
    /* Unused channels do not control whether the AIN1 result is accepted. */
    regs[2] = 0xFD; regs[5] = 0; regs[7] = 6; scan();
    assert(g_ads_init_ok && snapshot().valid);
    pass("AIN1 power-down protection; other-channel configuration is irrelevant");

    scenario(0); app_init(); regs[3] = 0; scan();
    assert(!g_ads_init_ok && isnan(g_distance_mm));
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && regs[3] == 1 && snapshot().valid);
    pass("SDO format corruption invalidates output and recovers");

    scenario(0); app_init(); fail_command = 0; scan();
    assert(!snapshot().valid && isnan(g_distance_mm));
    assert(snapshot().status & LASER_SAMPLE_IO_ERROR);
    scan(); assert(snapshot().valid && isfinite(g_distance_mm));
    pass("NOOP transfer failure rejects raw and next sample recovers");

    scenario(0); app_init(); fail_command = 0xC400;
    before = conversions; scan();
    assert(conversions - before == 1 && !snapshot().valid && isnan(g_distance_mm));
    scan(); assert(snapshot().valid);
    pass("MAN1 transfer failure stops before NOOP");

    scenario(0); app_init(); bad_tag = 1;
    scan(); assert(!snapshot().valid && isnan(g_distance_mm) && g_ads_init_ok);
    scan(); scan(); assert(!g_ads_init_ok && (snapshot().status & LASER_SAMPLE_NOT_READY));
    bad_tag = 0;
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && snapshot().valid && resets == 2);
    pass("tag mismatch rejected; three failures trigger reconfiguration");

    scenario(0); corrupt_echo = 1; app_init();
    assert(!g_ads_init_ok && !g_ads_spi_ok);
    pass("write echo mismatch rejects initialization");

    scenario(0); disconnected = 1; app_init();
    for (int i = 0; i < 200; ++i) app_task_10ms();
    assert(!g_ads_init_ok && isnan(g_distance_mm) && aborts == 3);
    disconnected = 0;
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && snapshot().valid);
    pass("disconnected bus uses one-second backoff then recovers");

    scenario(0); app_init(); scan(); device_reset(); scan();
    assert(!g_ads_init_ok && isnan(g_distance_mm));
    for (int i = 0; i < 110; ++i) app_task_10ms();
    assert(g_ads_init_ok && snapshot().valid && regs[6] == 5);
    pass("ADC reset during operation restores AIN1 configuration");

    scenario(0); app_init(); mock_tick = UINT32_MAX - 50U; scan();
    mock_tick += 100U;
    LaserSample sample = snapshot();
    assert(sample.valid && !(sample.status & LASER_SAMPLE_STALE));
    mock_tick += 151U; sample = snapshot();
    assert(!sample.valid && isnan(sample.distance_mm) && (sample.status & LASER_SAMPLE_STALE));
    raw = 65535; scan(); assert(snapshot().status & LASER_SAMPLE_ADC_RAIL);
    Laser_GetSample(NULL);
    pass("coherent single-channel snapshot, stale across tick wrap, rail status");

    printf("%u AIN1 regression scenarios passed. Hardware timing is not simulated.\n", cases);
    return 0;
}
