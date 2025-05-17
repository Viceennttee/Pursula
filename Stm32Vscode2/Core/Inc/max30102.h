#ifndef MAX30102_H
#define MAX30102_H

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



void MAX30102_Init();
void MAX30102_LowPower(); //just IR for low power
#endif
