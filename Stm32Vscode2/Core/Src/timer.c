#include "timer.h"
#include "main.h"
volatile uint16_t counterTim6=0;
volatile uint8_t pulseOxFlag =0;
volatile uint8_t mpuFlag =0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    counterTim6++;
    if(counterTim6>TEN_SECS){
        pulseOxFlag = 1;
        mpuFlag = 0;
    }
}

void TIM6_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void turnOffMCU(){
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
}
