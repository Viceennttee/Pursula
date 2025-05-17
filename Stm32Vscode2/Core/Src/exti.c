#include "exti.h"
#include "I2C.h"
#include "mpu.h"
#include "stm32f407xx.h"
volatile uint8_t max30Flag =0;
volatile uint8_t mpuFlag=0;
volatile uint8_t statusInt =0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  if(GPIO_Pin == GPIO_PIN_0){
	  mpuFlag=1;
	  statusInt = I2C_Read(&hi2c1,IMUADDRESS, INT_STATUS, 1);

  }
  if(GPIO_Pin == GPIO_PIN_1){
	  max30Flag =1;
	  mpuFlag = 0;
  }
}



