#ifndef PULSEOXIMETRY_H
#define PULSEOXIMETRY_H
#include "main.h"

#define PULSEOXIMETRY_ADDRESS 0x57<<1
#define MODE_CONFIGURATION 0x09
#define PARTID 0xFF

//SpO2 Configuration
#define SPO2_CONFIG 0x0A

//leds current config
#define RLED_CURRENT 0x0C
#define IRLED_CURRENT 0x0D
#define GLED_CURRENT 0x0E
//leds timeSlot config
#define SLOT1_2 0x11
#define SLOT3_4 0x12

//leds for timeSlot
#define LED1RED 0x1
#define LED2IR 0x2
#define LED3GREEN 0x3

//FIFO registers
#define FIFO_DATA           0x07
#define FIFO_WR_PTR         0x04
#define FIFO_RD_PTR         0x06
#define FIFO_CONFIG 0x08



void pulseOxInit();
#endif
