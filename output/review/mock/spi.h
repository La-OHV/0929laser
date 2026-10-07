#ifndef REVIEW_SPI_H
#define REVIEW_SPI_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
extern SPI_HandleTypeDef hspi2;
#define ADS_CS_GPIO_Port ((void *)0)
#define ADS_CS_Pin 12U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
void HAL_GPIO_WritePin(void *, uint16_t, int);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
#endif
