#ifndef UART_H
#define UART_H
#include "main.h"
extern uint8_t rxBuffer[10];
extern uint8_t isUartDone;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
//wrappers
void uartWriteAT(const char* cmd); //write AT commands
void uartWriteIt(const uint8_t* msg, uint8_t size); //send info to HC08 non blocking
void sendLedValuesUART(uint32_t red, uint32_t ir, uint32_t green); //debug function
#endif
