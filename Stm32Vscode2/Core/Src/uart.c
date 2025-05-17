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

void sendLedValuesUARTDebug(uint32_t red, uint32_t ir, uint32_t green) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "RED: %lu | IR: %lu | GREEN: %lu\r\n", red, ir, green);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
