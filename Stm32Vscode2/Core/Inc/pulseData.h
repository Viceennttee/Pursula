#ifndef PULSEDATA_H
#define PULSEDATA_H
#include "main.h"
#define BUFFER_SIZE 1023*3
typedef struct pulseData{
	uint32_t red;
	uint32_t ir;
	uint32_t green;
	uint16_t index;
	//uint8_t sampleNumber;
} pulseData;
uint8_t readValue(pulseData* buffer);
uint8_t isDevActive();
#endif
