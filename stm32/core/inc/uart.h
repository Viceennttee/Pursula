#ifndef UART_H
#define UART_H
extern uint8_t rxBuffer[10];
extern uint8_t isUartDone;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
uint8_t* uartRead(UART_HandleTypeDef *huart, uint8_t *buffer, uint8_t numberOfBytes);
void uartWrite(UART_HandleTypeDef *huart, uint8_t data);

void JDY25_SendCommand(const char* cmd);
void JDY25_ReadResponse(char* buffer, uint16_t bufferSize);
#endif
