#include "mpu.h"
#include "main.h"


void mpuWakeUP(){
	uint8_t data = 0x00; //Internal 8MHz oscillator
	I2CWrite(&hi2c1,IMUADDRESS,POWERMANAGEMENT,data,1);
}
void mpuCycleInit(){
/*
 * The MPU-60X0 can be put into Accelerometer Only Low Power Mode using the following steps:
(i) Set CYCLE bit to 1
(ii) Set SLEEP bit to 0
(iii) Set TEMP_DIS bit to 1
(iv) Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 * */
	//cycle mode config
	uint8_t cycleMode = 1<<6;	//cycle mode to 1
	uint8_t sleepTo0 = 0<<7;  //Set SLEEP bit to 0
	uint8_t disableTemp= 1<<4;  //Set TEMP_DIS bit to 1
	uint8_t data = cycleMode | sleepTo0 | disableTemp;
	I2CWrite(&hi2c1,IMUADDRESS,POWERMANAGEMENT,data,1);

	uint8_t justAcc = 0x07; //disables all gyroscopes by setting them to one
	uint8_t wakeUpFrec = 2<<6; //20Hz, 60uA
	uint8_t data2 = justAcc | wakeUpFrec;
	I2CWrite(&hi2c1, IMUADDRESS, POWERMANAGEMENT2, data2, 1);


	//enable cycle mode
}
void sleepImu(){
	uint8_t sleepMode=0x40; //sleepMode
	I2CWrite(&hi2c1, IMUADDRESS, POWERMANAGEMENT, sleepMode, 1);

}
void checkMPUState() {
    // 1. Verificar WHO_AM_I (debe ser 0x68)
    uint8_t whoAmI = I2C_Read(&hi2c1,IMUADDRESS, 0x75, 1);
    //printf("WHO_AM_I: 0x%02X\n", whoAmI);

    // 2. Leer PWR_MGMT_1 (0x6B) - Asegurarse de que no esté en SLEEP
    uint8_t pwrMgmt1 = I2C_Read(&hi2c1,IMUADDRESS, 0x6B, 1);
    //printf("PWR_MGMT_1: 0x%02X\n", pwrMgmt1);  // Debe ser 0x00 (activo)

    // 3. Leer ACCEL_CONFIG (0x1C) - Rango del acelerómetro
    uint8_t accelConfig = I2C_Read(&hi2c1,IMUADDRESS, 0x1C, 1);
    //printf("ACCEL_CONFIG: 0x%02X\n", accelConfig);  // 0x00 = ±2g, 0x08 = ±4g, etc.

    // 4. Leer FIFO_EN (0x23) - Asegurarse de que no esté habilitado (debe ser 0x00)
    uint8_t fifoEn = I2C_Read(&hi2c1,IMUADDRESS, 0x23, 1);
   // printf("FIFO_EN: 0x%02X\n", fifoEn);
}

void mpuIntConfig(){
	uint8_t intLevel = 1<<7;
	I2CWrite(&hi2c1,IMUADDRESS,INT_PIN_CFG,intLevel,1);
	uint8_t dataReadyEn = 1;
	I2CWrite(&hi2c1, IMUADDRESS, INT_ENABLE,dataReadyEn, 1);
}
