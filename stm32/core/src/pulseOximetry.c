#include "pulseOximetry.h"
#include "I2C.h"
#include "main.h"
void pulseOxInit(){
	uint8_t reset = 1<<6; //Bit 6: Reset Control (RESET)
	uint8_t multiLed = 7; //111 Multi-LED mode Green, Red, and/or IR
	uint8_t test =0;
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, reset, 1);
	//delay
	HAL_Delay(100);
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, multiLed, 1);
	test = I2C_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, PARTID, 1);
	asm("NOP");
	//missing parameters of led amplitude and all that stuff

	// LED pulse amplitudes
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x0C, 0x24, 1); // LED1 (Rojo)
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x0D, 0x24, 1); // LED2 (IR)
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x0E, 0x24, 1); // LED3 (Verde)

	// Multi-LED slots (activa 3 LEDs)
	uint8_t slot1_2 = (0x02 << 4) | 0x01; // SLOT2 = LED2, SLOT1 = LED1
	uint8_t slot3_4 = (0x00 << 4) | 0x03; // SLOT3 = LED3, SLOT4 = apagado
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x11, slot1_2, 1);
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x12, slot3_4, 1);
}
