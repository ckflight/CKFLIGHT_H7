
#include "CK_TIME_HAL.h"
#include "DRIVERS/CK_SYSTEM.h"

uint32_t sysTickCounter = 0;

uint32_t timeout_num;

void CK_TIME_SetTimeOut(uint32_t time){

	timeout_num = time;

}

uint32_t CK_TIME_GetTimeOut(void){

	return timeout_num;

}

// Overwrite of weak HAL decleration
void HAL_IncTick(void){

	sysTickCounter++;

	if(timeout_num > 0){

		timeout_num--;

	}

}

// Overwrite of weak HAL decleration
uint32_t HAL_GetTick(void){

  return sysTickCounter ;

}

uint32_t CK_TIME_GetMicroSec_SYSTICK(void){

	// HAL_Init() Initialises SysTick->LOAD, SysTick->VAL, SysTick->CTRL registers

	uint32_t ticks ;
	uint32_t count ;

	SysTick->CTRL;

	do{
		ticks = SysTick->VAL;

		count = sysTickCounter;
	}
	while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

	return count * 1000 + (SysTick->LOAD + 1 - ticks) / (F_CPU / 1000000);


}

uint32_t CK_TIME_GetMilliSec_SYSTICK(void){

	return sysTickCounter;
}

uint32_t CK_TIME_GetMicroSec(void){

	// DWT might be implemented and called here with definition

	return CK_TIME_GetMicroSec_SYSTICK();

}

uint32_t CK_TIME_GetMilliSec(void){

	// DWT might be implemented and called here with definition

	return CK_TIME_GetMilliSec_SYSTICK();

}

void CK_TIME_DelayMilliSec(uint32_t msec){

	while(msec--)CK_TIME_DelayMicroSec(1000);

}

void CK_TIME_DelayMicroSec(uint32_t usec){

	uint32_t now = CK_TIME_GetMicroSec();

	while (CK_TIME_GetMicroSec() - now < usec);

}



