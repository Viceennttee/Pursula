#include "imuData.h"
#include "main.h"
#include "I2C.h"
#include "mpu.h"

  //IMU variables data
  uint8_t highByteAccelX = 0;
  uint8_t lowByteAccelX = 0;

  uint8_t highByteAccelY = 0;
  uint8_t lowByteAccelY = 0;

  uint8_t highByteAccelZ = 0;
  uint8_t lowByteAccelZ = 0;
  uint8_t whoAmIValue = 0;
void getAngle(IMUData* imuData){

	 if(!imuData){
		 Error_Handler();
	 }
	 whoAmIValue = I2C_Read(&hi2c1,IMUADDRESS, WHO_AM_I_REG,1);

	 highByteAccelX = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_XOUT_H,1);
	 lowByteAccelX = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_XOUT_L,1);
	 highByteAccelY = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_YOUT_H,1);
	 lowByteAccelY = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_YOUT_L,1);
	 highByteAccelZ = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_ZOUT_H,1);
	 lowByteAccelZ= I2C_Read(&hi2c1,IMUADDRESS, ACCEL_ZOUT_L,1);

	 imuData->rawAccelX = (lowByteAccelX|(highByteAccelX<<8));
	 imuData->rawAccelY = (lowByteAccelY|(highByteAccelY<<8));
	 imuData->rawAccelZ = (lowByteAccelZ|(highByteAccelZ<<8));


	 imuData->accelX = imuData->rawAccelX /16384.0;
	 imuData->accelY = imuData->rawAccelY/16384.0;
	 imuData->accelZ = imuData->rawAccelZ/16384.0;

	 //calculco del angulo
	 imuData->angleRadians = atan2(imuData->accelY,sqrt(imuData->accelX*imuData->accelX+imuData->accelZ*imuData->accelZ));
	 imuData->angleDegrees =imuData->angleRadians*180.0/3.1416;
}

void getTotalAcc(IMUData* imuData){
	 if(!imuData){
		 Error_Handler();
	 }
	 highByteAccelX = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_XOUT_H,1);
	 lowByteAccelX = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_XOUT_L,1);
	 highByteAccelY = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_YOUT_H,1);
	 lowByteAccelY = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_YOUT_L,1);
	 highByteAccelZ = I2C_Read(&hi2c1,IMUADDRESS, ACCEL_ZOUT_H,1);
	 lowByteAccelZ= I2C_Read(&hi2c1,IMUADDRESS, ACCEL_ZOUT_L,1);

	 imuData->rawAccelX = (lowByteAccelX|(highByteAccelX<<8));
	 imuData->rawAccelY = (lowByteAccelY|(highByteAccelY<<8));
	 imuData->rawAccelZ = (lowByteAccelZ|(highByteAccelZ<<8));


	 imuData->accelX = imuData->rawAccelX /16384.0;
	 imuData->accelY = imuData->rawAccelY/16384.0;
	 imuData->accelZ = imuData->rawAccelZ/16384.0;

	 imuData->totalAcc = sqrt((imuData->accelX)*(imuData->accelX)+(imuData->accelY)*(imuData->accelY)+(imuData->accelZ)*(imuData->accelZ));
}


