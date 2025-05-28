#include "pulseOximetry.h"
#include "I2C.h"
#include "main.h"




void enableInt(uint8_t intFlag){

	uint8_t int_en2;
    // Leer el valor actual de INT_ENABLE_2 (registro 0x03)
    HAL_I2C_Mem_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, INT_STATUS1, I2C_MEMADD_SIZE_8BIT, &int_en2, 1, HAL_MAX_DELAY);

    // Activar solo el bit 6 (ALC_OVF_EN)
    int_en2 |= (1 << 6);

    // Escribir de nuevo el valor
    HAL_I2C_Mem_Write(&hi2c1, PULSEOXIMETRY_ADDRESS, 0x03, I2C_MEMADD_SIZE_8BIT, &int_en2, 1, HAL_MAX_DELAY);
}
void pulseOxInit(){
	uint8_t reset = 1<<6; //Bit 6: Reset Control (RESET)
	uint8_t multiLed = 7; //111 Multi-LED mode Green, Red, and/or IR
	uint8_t status =0; 
	//uint8_t spo2Config =
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, reset, 1);
	//delay
	HAL_Delay(100);
	I2CWrite(&hi2c1,PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, multiLed, 1);
	status = I2C_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, PARTID, 1);
	asm("NOP");
	//missing parameters of led amplitude and all that stuff
	uint8_t spo2 = (0b11 << 5) | (0b001 << 2) | (0b00); // 18-bit, 100Hz, 2048nA
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

	uint8_t fifo = (0b000<< 5) | (1<< 4); // 32 promedio, rollover activado
	I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, FIFO_CONFIG, fifo, 1);
}

void pulseOxLowPower(){
    // 1. Reset
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, 1 << 6, 1);
    HAL_Delay(50);

    // 2. Modo SpO2 (no multi-LED)
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, MODE_CONFIGURATION, 0x03, 1); // 0x03 = SpO2 mode

    // 3. Configurar SPO2: 15-bit (69us), 50 Hz, ADC range 2048nA
    uint8_t spo2 = (0b10 << 5) | (0b000 << 2) | (0b00);  // 15-bit, 50 Hz, 2048nA
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SPO2_CONFIG, spo2, 1);

    // 4. Corriente de LEDs: solo IR encendido, rojo y verde apagados
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, RLED_CURRENT, 0x00, 1); // Apagado
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, IRLED_CURRENT, 0x20, 1); // ~6.4 mA
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, GLED_CURRENT, 0x00, 1); // Apagado

    // 5. Asignar solo SLOT1 = IR (LED2)
    uint8_t slot1_2 = (0x00 << 4) | LED2IR; // SLOT1 = LED2 (IR), SLOT2 = apagado
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SLOT1_2, slot1_2, 1);

    // SLOT3 y SLOT4 apagados
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, SLOT3_4, 0x00, 1);

    // 6. FIFO: promedio de 4, rollover activado
    uint8_t fifo = (0b010 << 5) | (1 << 4); // promedio = 4, rollover activado
    I2CWrite(&hi2c1, PULSEOXIMETRY_ADDRESS, FIFO_CONFIG, fifo, 1);
}