
#ifndef MPU_H
#define MPU_H
//Registros de la IMU
#define IMUADDRESS (0x68 << 1)
#define POWERMANAGEMENT 0X6B
#define POWERMANAGEMENT2 0x6C
//registros de todas las aceleraciones para sacar ángulo
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E

#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define WHO_AM_I_REG 0x75


//interrupt register
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A




void mpuWakeUP();
void checkMPUState();
void mpuCycleInit();
void mpuIntConfig();

#endif
