#include "main.h"
#include "uart.h"
uint8_t isUartDone=1;


volatile uint8_t isSent = 1;
volatile uint8_t test=0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
     if (huart == &huart2) {
        isUartDone =1;
     }

}


void uartWriteAT(const char* cmd) {
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void uartWriteIt(const uint8_t* msg, uint8_t size){
    HAL_StatusTypeDef status;
    if(isUartDone ==1){
        status = HAL_UART_Transmit_IT(&huart2,msg,size);
        isUartDone =0;
    }
}

void sendLedValuesUARTDebug(uint32_t red, uint32_t ir, uint32_t green) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "RED: %lu | IR: %lu | GREEN: %lu\r\n", red, ir, green);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
