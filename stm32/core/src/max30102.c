#include "max30102.h"
#include "main.h"
void MAX30102_Init()
{
    // 1. RESET
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_MODE_CONFIG, MODE_RESET, 1);
    /*
     * When the RESET bit is set to one, all configuration, threshold, and data registers are reset to their power-on-state through
	a power-on reset. The RESET bit is cleared automatically back to zero after the reset sequence is completed.
     * */

    // Esperar a que el RESET termine (polling al bit 6 de MODE_CONFIG)
    //no estoy seguro si es necesario
    uint8_t mode_cfg;
    do {
        mode_cfg = I2C_Read(&hi2c1,MAX30102_I2C_ADDRESS, REG_MODE_CONFIG, 1);
    } while (mode_cfg & MODE_RESET);


    // 2. Configurar FIFO
    // checar
    uint8_t fifo_config = (SMP_AVE_8 | FIFO_ROLLOVER_EN | FIFO_A_FULL_15);
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_FIFO_CONFIG, fifo_config, 1);

    // 3. Configurar Modo SPO2

    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_MODE_CONFIG, MODE_SPO2, 1);

    // 4. Configurar SPO2: Rango ADC, Sample Rate, Pulse Width
    uint8_t spo2_config = (ADC_RANGE_2048nA | SAMPLE_RATE_50HZ | PULSE_WIDTH_69US);
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_SPO2_CONFIG, spo2_config, 1);

    // 5. Corriente de LEDs
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_LED1_PA, LED_CURRENT_6MA, 1);  // LED rojo
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_LED2_PA, LED_CURRENT_6MA, 1);  // LED infrarrojo

    // 6. Habilitar interrupciones (nueva muestra lista y FIFO casi lleno)
    uint8_t int_enable = (INT_A_FULL_ENABLE | INT_PPG_RDY_ENABLE);
    I2CWrite(&hi2c1,MAX30102_I2C_ADDRESS, REG_INTR_ENABLE_1, int_enable, 1);
}

void MAX30102_LowPower(){}
