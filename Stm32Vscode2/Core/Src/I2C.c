#include "I2C.h"



/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void I2CWriteIT(I2C_HandleTypeDef* i2cHandler,uint8_t deviceAddress,uint16_t memAddr, uint8_t data, uint8_t is8Bit){
    if (i2cHandler == NULL) {
        return HAL_ERROR;
    }
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
    HAL_StatusTypeDef writeStatus;
    writeStatus = HAL_I2C_Mem_Write_IT(
    		i2cHandler,
			deviceAddress,
			memAddr,
			addressSize,
			&data,
			1
			);
}
uint8_t I2C_ReadIT(I2C_HandleTypeDef *i2cHandler, uint8_t deviceAddress, uint16_t memAddr, uint8_t is8Bit) {
    if (i2cHandler == NULL) {
        return HAL_ERROR;
    }
	uint8_t data = 21;
    HAL_StatusTypeDef readStatus;
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
	readStatus = HAL_I2C_Mem_Read_IT(
			i2cHandler,
			deviceAddress | 0x01,         // Dirección EEPROM + bit de lectura (0xA1)
			memAddr,                   // Dirección de memoria (2 bytes)
			addressSize,     // 8 bits para la dirección
			&data,                     // Buffer para almacenar el dato leído
			1                         // Tamaño del dato (1 byte)
			);
    if(readStatus != HAL_OK) return readStatus;
    return data;
}


void I2CWrite(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress,uint16_t memAddr, uint8_t data, uint8_t is8Bit){
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
    HAL_StatusTypeDef writeStatus;
     writeStatus = HAL_I2C_Mem_Write(
        i2cHandler,                    // Handler I2C
        deviceAddress,                // Dirección EEPROM + bit de escritura
        memAddr,                   // Dirección de memoria (2 bytes)
        addressSize,     // Usamos 16 bits para la dirección
        &data,                     // Dato a escribir
        1,                         // Tamaño del dato (1 byte)
        HAL_MAX_DELAY              // Timeout
    );
    HAL_Delay(1);  // Espera obligatoria para la escritura
}

uint8_t I2C_Read(I2C_HandleTypeDef *i2cHandler,uint8_t deviceAddress, uint16_t memAddr, uint8_t is8Bit) {

    uint8_t data = 21;
    HAL_StatusTypeDef readStatus;
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
    readStatus = HAL_I2C_Mem_Read(
        i2cHandler,                    // Handler I2C
        deviceAddress,         // Dirección EEPROM + bit de lectura (0xA1)
        memAddr,                   // Dirección de memoria (2 bytes)
        addressSize,     // 8 bits para la dirección
        &data,                     // Buffer para almacenar el dato leído
        1,                         // Tamaño del dato (1 byte)
        HAL_MAX_DELAY              // Timeout
    );
    if(readStatus != HAL_OK) return readStatus;
    return data;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c); // Reinicio completo
    }
}

