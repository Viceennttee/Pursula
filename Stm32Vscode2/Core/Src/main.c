/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "exti.h"
#include "I2C.h"
#include "initPeripheralsHal.h"
#include "imuData.h"
#include "max30102.h"
#include "mpu.h"
#include "uart.h"
#include "pulseOximetry.h"
#include "pulseData.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t txBuffer[]={65,66,67,68,69,70,71,72,73,74}; //ABCDEFGHIJ in ASCII code
pulseData pulseArray[BUFFER_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/*
 mpuWakeUP();
 mpuCycleInit();
 mpuIntConfig();
 checkMPUState();
*/

  HAL_StatusTypeDef testPulse=0;
  testPulse = HAL_I2C_IsDeviceReady(&hi2c1, PULSEOXIMETRY_ADDRESS, 1, HAL_MAX_DELAY);
  pulseOxInit();
  HAL_Delay(5000);
 //mpuFlag =1;
  char response[64] = {0};
  uint8_t whoAmIPulse =0;
  // Enviar comando AT
  JDY25_SendCommand("AT\r\n");

  HAL_UART_Transmit_IT(&huart2, txBuffer, 10);
  while (1)
  {
	  // whoAmIPulse =  I2C_Read(&hi2c1, PULSEOXIMETRY_ADDRESS, PARTID, 1);
	  // readValue(newData);

	  // sendLedValuesUARTDebug(red, ir, green);
	  readValue(pulseArray);
	  HAL_Delay(1000);
	  asm("NOP");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Clear SleepDeep bit to go into SLEEP mode instead of STOP mode
/*
	  //when too much ambient light on MAX30102, all devices go on low power mode
	  if(max30Flag == 1){
		sleepImu();
		MAX30102_LowPower();
	  }
	  //when new data available in MPU
	  if(mpuFlag == 1){
		//read MAX, cant use its flag cause its used to enter low power mode
		getTotalAcc(&imuData);
		if((imuData.totalAcc)>0.5){
			//send uart alert to smartphone
			  asm("NOP");
		  }
	  }
*/

	  //HAL_SuspendTick();
	  // Instrucción de bajo nivel para entrar en modo Sleep
	  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	  //HAL_ResumeTick();
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
