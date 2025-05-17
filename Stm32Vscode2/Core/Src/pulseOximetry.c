#include "pulseOximetry.h"
#include "I2C.h"
#include "main.h"

void pulseOxInit(){
	uint8_t reset = 1<<6; //Bit 6: Reset Control (RESET)
	uint8_t multiLed = 7; //111 Multi-LED mode Green, Red, and/or IR
	uint8_t test =0;
	//uint8_t spo2Config =
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, reset, 1);
	//delay
	HAL_Delay(100);
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, multiLed, 1);
	test = I2C_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, PARTID, 1);
	asm("NOP");
	//missing parameters of led amplitude and all that stuff
	uint8_t spo2 = (0b11 << 5) | (0b000 << 2) | (0b00); // 18-bit, 50Hz, 2048nA
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SPO2_CONFIG, spo2, 1);
	// LED pulse amplitudes, value*0.2=mA
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, RLED_CURRENT,0xFF, 1); // LED1 (Rojo)
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, IRLED_CURRENT, 0xFF, 1); // LED2 (IR)
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, GLED_CURRENT, 0xFF, 1); // LED3 (Verde)

	// Multi-LED slots (activa 3 LEDs)
	uint8_t slot1_2 = (LED2IR << 4) | LED1RED; // SLOT2 = LED2, SLOT1 = LED1
	uint8_t slot3_4 = (0x00 << 4) | LED3GREEN; // SLOT3 = LED3, SLOT4 = apagado
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SLOT1_2, slot1_2, 1);
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SLOT3_4, slot3_4, 1);

	//Fifo config
	uint8_t fifo = (0b010 << 5) | (1 << 4); // promedio = 4, rollover activado
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, FIFO_CONFIG, fifo, 1);
}
