#ifndef I2C_H
#define I2C_H
#include "stm32f4xx_hal.h"
void I2CWrite(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress,uint16_t memAddr, uint8_t data, uint8_t is8Bit);
uint8_t I2C_Read(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress, uint16_t memAddr, uint8_t is8Bit);
void I2CWriteIT(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress,uint16_t memAddr, uint8_t data, uint8_t is8Bit);
uint8_t I2C_ReadIT(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress, uint16_t memAddr, uint8_t is8Bit);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);


#endif

