/*
 * pulseData.c
 *
 *  Created on: May 13, 2025
 *      Author: vicen
 */
#include "pulseData.h"
#include "I2C.h"
#include "pulseOximetry.h"
static uint8_t writeIndex=0;
uint8_t readValue(pulseData* buffer){
	  uint8_t pulseData[9];
	  uint32_t red = 0;
	  uint32_t ir = 0;
	  uint32_t green = 0;
	  uint32_t red2 = 0;
	  uint32_t ir2 = 0;
	  uint32_t green2 = 0;
	  struct pulseData newData;
	  HAL_I2C_Mem_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x07, I2C_MEMADD_SIZE_8BIT, pulseData, 9, HAL_MAX_DELAY);
	  newData.red   = ((uint32_t)pulseData[0] << 16 | (uint32_t)pulseData[1] << 8 | pulseData[2]) >> 6;
	  newData.ir    = ((uint32_t)pulseData[3] << 16 | (uint32_t)pulseData[4] << 8 | pulseData[5]) >> 6;
	  newData.green = ((uint32_t)pulseData[6] << 16 | (uint32_t)pulseData[7] << 8 | pulseData[8]) >> 6;
	  if(writeIndex < BUFFER_SIZE){
		  buffer[writeIndex++]=newData;
		  return 0;
	  }
	  return 1;
}

