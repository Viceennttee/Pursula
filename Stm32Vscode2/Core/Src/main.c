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
#include "HC08.h"
#include "timer.h"
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
DMA_HandleTypeDef hdma_usart2_tx;


/* USER CODE BEGIN PV */
extern uint8_t mpuFlag;
extern uint8_t pulseOxFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t txBuffer[10]={65,66,67,68,69,70,71,72,73,74}; //ABCDEFGHIJ in ASCII code
uint8_t rxBuffer[10];
pulseData pulseArray[BUFFER_SIZE];
// uint32_t redIrGreen[9][BUFFER_SIZE];
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
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
  HAL_StatusTypeDef testMpu =0;
  HAL_StatusTypeDef status;
  HAL_DMA_StateTypeDef DMA_Status;
  testPulse = HAL_I2C_IsDeviceReady(&hi2c1, PULSEOXIMETRY_ADDRESS, 1, HAL_MAX_DELAY);
  //testMpu= HAL_I2C_IsDeviceReady(&hi2c1, IMUADDRESS, 1, HAL_MAX_DELAY);
  uint8_t devStatus=0;
  pulseOxInit(); //init max30101, 3Leds
  //pulseOxLowPower();
  hc08Init();
  // status = HAL_UART_Transmit_DMA(&huart2,txBuffer,10);
  // DMA_Status = HAL_DMA_GetState(&hdma_usart2_tx);
  HAL_Delay(500);
  uint8_t reading=0;
  while (1)
  {
	  reading = readValue(pulseArray);
    // uartWriteIt(txBuffer,9);
    HAL_Delay(100);
    if(reading ==1){
      asm("NOP");
    }
    if (reading ==2)
    {
      asm("NOP");
    }
    
	  turnOffMCU();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
