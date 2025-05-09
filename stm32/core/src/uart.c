#include "main.h"
#include "uart.h"
uint8_t rxBuffer[10];
uint8_t isUartDone=1;




void JDY25_SendCommand(const char* cmd) {
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void JDY25_ReadResponse(char* buffer, uint16_t bufferSize) {
    HAL_UART_Receive(&huart2, (uint8_t*)buffer, bufferSize, 100); // espera hasta 100 ms
}
