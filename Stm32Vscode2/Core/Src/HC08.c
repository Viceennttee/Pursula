#include "HC08.h"
#include "uart.h"
#include "main.h"
const char mode1[] ="AT+MODE=1";
const char autoSet[] = "AT+AUST=10";
const char reset[] = "AT+RESET";
const char setDeviceName[] = "Pursula";
void hc08Init(){
  uartWriteAT(mode1);
  uartWriteAT(autoSet);
  uartWriteAT(setDeviceName);
  uartWriteAT(reset);
}