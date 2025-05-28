/*
 * pulseData.c
 *
 *  Created on: May 13, 2025
 *      Author: vicen
 */
#include "pulseData.h"
#include "I2C.h"
#include "pulseOximetry.h"
static uint16_t writeIndex=0;
uint8_t readValue(pulseData* buffer){
	uint8_t wtrPtr=0;
	uint8_t readPtr = 0;
	uint8_t samples =0;
	uint8_t partID =0;
	HAL_StatusTypeDef status = 0;
	//Get pointers
	// readPtr = I2C_Read(&hi2c1,PULSEOXIMETRY_ADDRESS,FIFO_RD_PTR,1);
	// wtrPtr =I2C_Read(&hi2c1,PULSEOXIMETRY_ADDRESS,FIFO_WR_PTR,1);
	// status = HAL_I2C_Mem_Read(&hi2c1,PULSEOXIMETRY_ADDRESS,PARTID,I2C_MEMADD_SIZE_8BIT,&partID,1,HAL_MAX_DELAY);
	status = HAL_I2C_Mem_Read(&hi2c1,PULSEOXIMETRY_ADDRESS,FIFO_RD_PTR,I2C_MEMADD_SIZE_8BIT,&readPtr,1,HAL_MAX_DELAY);
	status = HAL_I2C_Mem_Read(&hi2c1,PULSEOXIMETRY_ADDRESS,FIFO_WR_PTR,I2C_MEMADD_SIZE_8BIT,&wtrPtr,1,HAL_MAX_DELAY);
	samples = (wtrPtr >= readPtr) ? (wtrPtr - readPtr) : (wtrPtr + 32 - readPtr);
	if(samples>0){
	uint8_t rawData[samples*9];
	HAL_I2C_Mem_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x07, I2C_MEMADD_SIZE_8BIT, rawData, 9*samples, HAL_MAX_DELAY);
	for (uint8_t i = 0; i < samples; i++) {
		uint8_t* p = &rawData[i * 9];
		pulseData d;
		d.red   = (((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]) >> 6;
		d.ir    = (((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5]) >> 6;
		d.green = (((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 8) | p[8]) >> 6;
		d.index = writeIndex;
		//d.sampleNumber = samples;
		buffer[writeIndex++]=d;
		if(writeIndex>= BUFFER_SIZE){
			return 1;
		}
	}
	return 0;
	}
	//   newData.red   = ((uint32_t)pulseData[0] << 16 | (uint32_t)pulseData[1] << 8 | pulseData[2]) >> 6;
	//   newData.ir    = ((uint32_t)pulseData[3] << 16 | (uint32_t)pulseData[4] << 8 | pulseData[5]) >> 6;
	//   newData.green = ((uint32_t)pulseData[6] << 16 | (uint32_t)pulseData[7] << 8 | pulseData[8]) >> 6;
	//   if(writeIndex < BUFFER_SIZE){
	// 	  buffer[writeIndex++]=newData;
	// 	  return 0;
	//   }


}
uint8_t isDevActive(){
	uint8_t pulseData[3];
	struct pulseData partialData;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x07, I2C_MEMADD_SIZE_8BIT, pulseData, 3, HAL_MAX_DELAY);
	partialData.ir =((uint32_t)pulseData[0] << 16 | (uint32_t)pulseData[1] << 8 | pulseData[2]) >> 6;
	asm("NOP");
	if(partialData.ir<10000) return 0;
	return 1;
}