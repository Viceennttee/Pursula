#ifndef PULSEDATA_H
#define PULSEDATA_H
#include "main.h"
#define BUFFER_SIZE 128
typedef struct pulseData{
	uint32_t red;
	uint32_t ir;
	uint32_t green;
} pulseData;

uint8_t readValue(pulseData* buffer);

#endif
