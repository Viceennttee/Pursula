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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BMP280_ADDR (0x76 << 1)  // Dirección I2C del BMP280 (0x76 si SDO=GND, 0x77 si SDO=VCC)
#define BMP280_REG_ID 0xD0       // Registro del ID (debe devolver 0x58)


#define EEPROM_ADDRESS 0xA0 //es la dirección 0x50 << 1, porque todas las A van a 0
#define TIMEOUT 100



//Registros de la IMU
#define IMUADDRESS (0x68 << 1)
//registros de todas las aceleraciones para sacar ángulo
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E

#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define WHO_AM_I_REG 0x75

//registros del max 30102
#define MAX30102_I2C_ADDRESS    0xAE  // Dirección de escritura del MAX30102
#define REG_INTR_ENABLE_1      0x02 //interrup enable1
#define REG_FIFO_CONFIG        0x08
#define REG_MODE_CONFIG        0x09
#define REG_SPO2_CONFIG        0x0A
#define REG_LED1_PA            0x0C
#define REG_LED2_PA            0x0D
#define REG_TEMP_CONFIG        0x21

// --- Valores para configuración ---
#define MODE_RESET             (1 << 6)
#define MODE_SPO2              0x03

#define ADC_RANGE_2048nA       (0x00 << 5)  // SPO2_ADC_RGE[1:0]
#define SAMPLE_RATE_50HZ       (0x00 << 2)  // SPO2_SR[2:0]
#define PULSE_WIDTH_69US       (0x00)        // LED_PW[1:0]

#define SMP_AVE_8              (0x03 << 5)  // 8 muestras promedio
#define FIFO_ROLLOVER_EN       (1 << 4)
#define FIFO_A_FULL_15         (0x0F)        // FIFO a "casi lleno"

#define LED_CURRENT_6MA        0x1F          // Corriente baja (~6mA)

#define INT_A_FULL_ENABLE      (1 << 7)
#define INT_PPG_RDY_ENABLE     (1 << 6)

#define TEMP_EN                (1 << 0)

//addreses from eeprom




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct IMUData{
	int16_t rawAccelX;
	int16_t rawAccelY;
	int16_t rawAccelZ;
	float accelX;
	float accelY;
	float accelZ;
	float angleRadians;
	int8_t angleDegrees;
};
volatile uint8_t rxBuffer[10];
volatile uint8_t isUartDone=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t txBuffer[]={65,66,67,68,69,70,71,72,73,74}; //ABCDEFGHIJ in ASCII code
void I2CWrite(uint8_t deviceAddress,uint16_t memAddr, uint8_t data, uint8_t is8Bit){
	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	    // Opcional: Resetear I2C si está bloqueado
	    HAL_I2C_DeInit(&hi2c1);
	    HAL_I2C_Init(&hi2c1);
	}
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
    HAL_StatusTypeDef writeStatus;
     writeStatus = HAL_I2C_Mem_Write(
        &hi2c1,                    // Handler I2C
        deviceAddress,                // Dirección EEPROM + bit de escritura
        memAddr,                   // Dirección de memoria (2 bytes)
        addressSize,     // Usamos 16 bits para la dirección
        &data,                     // Dato a escribir
        1,                         // Tamaño del dato (1 byte)
        HAL_MAX_DELAY              // Timeout
    );
    HAL_Delay(1);  // Espera obligatoria para la escritura
}

uint8_t I2C_Read(uint8_t deviceAddress, uint16_t memAddr, uint8_t is8Bit) {
	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	    // Opcional: Resetear I2C si está bloqueado
	    HAL_I2C_DeInit(&hi2c1);
	    HAL_I2C_Init(&hi2c1);
	}
    uint8_t data = 21;
    HAL_StatusTypeDef readStatus;
	uint32_t addressSize = I2C_MEMADD_SIZE_16BIT;
	if(is8Bit == 1) addressSize = I2C_MEMADD_SIZE_8BIT;
    readStatus = HAL_I2C_Mem_Read(
        &hi2c1,                    // Handler I2C
        deviceAddress | 0x01,         // Dirección EEPROM + bit de lectura (0xA1)
        memAddr,                   // Dirección de memoria (2 bytes)
        addressSize,     // 8 bits para la dirección
        &data,                     // Buffer para almacenar el dato leído
        1,                         // Tamaño del dato (1 byte)
        HAL_MAX_DELAY              // Timeout
    );
    HAL_Delay(1);
    if(readStatus != HAL_OK) return readStatus;
    return data;
}

void mpuWakeUP(){
	uint8_t registerAddress = 0x6B; //Register 107 – Power Management 1
	uint8_t data = 0x00; //Internal 8MHz oscillator
	I2CWrite(IMUADDRESS,registerAddress, data,1);
}


void MAX30102_Init()
{
    // 1. RESET
    I2CWrite(MAX30102_I2C_ADDRESS, REG_MODE_CONFIG, MODE_RESET, 1);

    // Esperar a que el RESET termine (polling al bit 6 de MODE_CONFIG)
    uint8_t mode_cfg;
    do {
        mode_cfg = MAX30102_ReadRegister(REG_MODE_CONFIG); // Debes tener una función de lectura
    } while (mode_cfg & MODE_RESET);

    // 2. Configurar FIFO
    uint8_t fifo_config = (SMP_AVE_8 | FIFO_ROLLOVER_EN | FIFO_A_FULL_15);
    I2CWrite(MAX30102_I2C_ADDRESS, REG_FIFO_CONFIG, fifo_config, 1);

    // 3. Configurar Modo SPO2
    I2CWrite(MAX30102_I2C_ADDRESS, REG_MODE_CONFIG, MODE_SPO2, 1);

    // 4. Configurar SPO2: Rango ADC, Sample Rate, Pulse Width
    uint8_t spo2_config = (ADC_RANGE_2048nA | SAMPLE_RATE_50HZ | PULSE_WIDTH_69US);
    I2CWrite(MAX30102_I2C_ADDRESS, REG_SPO2_CONFIG, spo2_config, 1);

    // 5. Corriente de LEDs
    I2CWrite(MAX30102_I2C_ADDRESS, REG_LED1_PA, LED_CURRENT_6MA, 1);  // LED rojo
    I2CWrite(MAX30102_I2C_ADDRESS, REG_LED2_PA, LED_CURRENT_6MA, 1);  // LED infrarrojo

    // 6. Habilitar interrupciones (nueva muestra lista y FIFO casi lleno)
    uint8_t int_enable = (INT_A_FULL_ENABLE | INT_PPG_RDY_ENABLE);
    I2CWrite(MAX30102_I2C_ADDRESS, REG_INTR_ENABLE_1, int_enable, 1);
}
void checkMPUState() {
    // 1. Verificar WHO_AM_I (debe ser 0x68)
    uint8_t whoAmI = I2C_Read(IMUADDRESS, 0x75, 1);
    //printf("WHO_AM_I: 0x%02X\n", whoAmI);

    // 2. Leer PWR_MGMT_1 (0x6B) - Asegurarse de que no esté en SLEEP
    uint8_t pwrMgmt1 = I2C_Read(IMUADDRESS, 0x6B, 1);
    //printf("PWR_MGMT_1: 0x%02X\n", pwrMgmt1);  // Debe ser 0x00 (activo)

    // 3. Leer ACCEL_CONFIG (0x1C) - Rango del acelerómetro
    uint8_t accelConfig = I2C_Read(IMUADDRESS, 0x1C, 1);
    //printf("ACCEL_CONFIG: 0x%02X\n", accelConfig);  // 0x00 = ±2g, 0x08 = ±4g, etc.

    // 4. Leer FIFO_EN (0x23) - Asegurarse de que no esté habilitado (debe ser 0x00)
    uint8_t fifoEn = I2C_Read(IMUADDRESS, 0x23, 1);
   // printf("FIFO_EN: 0x%02X\n", fifoEn);
}
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
  MX_I2S3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t idSensor=0x00;
  uint8_t dataReadEEprom = 0;
  //HAL_StatusTypeDef statusEEprom = HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_ADDRESS, 3,TIMEOUT);
  //HAL_StatusTypeDef statusIMU = HAL_I2C_IsDeviceReady(&hi2c2, IMUADDRESS, 3, TIMEOUT);
  uint16_t memAddr = 0x0001;
  uint8_t dataToWrite = 0xAB;
  uint8_t readData = 0;

  //IMU variables data
  uint8_t highByteAccelX = 0;
  uint8_t lowByteAccelX = 0;

  uint8_t highByteAccelY = 0;
  uint8_t lowByteAccelY = 0;

  uint8_t highByteAccelZ = 0;
  uint8_t lowByteAccelZ = 0;

  struct IMUData imuData;

  uint8_t whoAmIValue =1;


  mpuWakeUP();

  HAL_UART_Receive_IT(&huart2, rxBuffer, 10);  // Recibir 10 byte cada vez
  while (1)
  {
	 //EEPROM_Write(memAddr, dataToWrite);
	 //readData = EEPROM_Read(memAddr);
	 //HAL_Delay(10);
	 checkMPUState();
	 I2CWrite(EEPROM_ADDRESS, memAddr,0xF0,0);

	 whoAmIValue = I2C_Read(IMUADDRESS, WHO_AM_I_REG,1);
	 //HAL_Delay(10);
	 highByteAccelX = I2C_Read(IMUADDRESS, ACCEL_XOUT_H,1);
	 //HAL_Delay(10);
	 lowByteAccelX = I2C_Read(IMUADDRESS, ACCEL_XOUT_L,1);
	 //HAL_Delay(10);
	 highByteAccelY = I2C_Read(IMUADDRESS, ACCEL_YOUT_H,1);
	 //HAL_Delay(10);
	 lowByteAccelY = I2C_Read(IMUADDRESS, ACCEL_YOUT_L,1);
	 //HAL_Delay(10);

	 highByteAccelZ = I2C_Read(IMUADDRESS, ACCEL_ZOUT_H,1);
	 //HAL_Delay(10);
	 lowByteAccelZ= I2C_Read(IMUADDRESS, ACCEL_ZOUT_L,1);
	 //HAL_Delay(10);



	 readData = I2C_Read(EEPROM_ADDRESS, memAddr,0);




	 imuData.rawAccelX = (lowByteAccelX|(highByteAccelX<<8));
	 imuData.rawAccelY = (lowByteAccelY|(highByteAccelY<<8));
	 imuData.rawAccelZ = (lowByteAccelZ|(highByteAccelZ<<8));

	// imuData.rawAccelX = checkLimitAngle(imuData.rawAccelX);
	 //imuData.rawAccelY = checkLimitAngle(imuData.rawAccelY);
	// imuData.rawAccelZ = checkLimitAngle(imuData.rawAccelZ);

	 imuData.accelX = imuData.rawAccelX /16384.0;
	 imuData.accelY = imuData.rawAccelY/16384.0;
	 imuData.accelZ = imuData.rawAccelZ/16384.0;

	 //calculco del angulo
	 imuData.angleRadians = atan2(imuData.accelY,sqrt(imuData.accelX*imuData.accelX+imuData.accelZ*imuData.accelZ));
	 imuData.angleDegrees =imuData.angleRadians*180.0/3.1416;
	 HAL_UART_Transmit_IT(&huart2, txBuffer, 10);
/*	 if(isUartDone ==1){
		 HAL_UART_Transmit_IT(&huart1, txBuffer, 10);
		 HAL_Delay(1000);
		 isUartDone=0;
	 }
*/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
    //HAL_Delay(500);
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
    //HAL_Delay(500);
    //HAL_I2C_Slave_Receive_IT(&hi2c2 ,(uint8_t *)RX_Buffer, 1); //Receiving in Interrupt mode

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- Callback de Recepción ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    for (int i = 0; i < 10; i++) {
        printf("Dato recibido: %c\n", rxBuffer[i]);
    }
    HAL_UART_Receive_IT(&huart2, rxBuffer, 10);  // Recibir 1 byte cada vez

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	isUartDone = 1;
}

// --- Callback de Transmisión Completa ---

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
