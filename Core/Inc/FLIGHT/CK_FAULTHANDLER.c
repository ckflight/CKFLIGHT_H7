
#include "FLIGHT/CK_FAULTHANDLER.h"
#include "FLIGHT/CK_ESC.h"

#include "DRIVERS/CK_LED.h"

void HardFault_Handler(void){

	CK_ESC_StopMotors();

	while(1){

		CK_LED_ToggleLedForMs(1, 1, 500);
		CK_LED_ToggleLedForMs(1, 2, 500);

	}

}

void MemManage_Handler(void){

	CK_ESC_StopMotors();

	while(1){

		CK_LED_ToggleLedForMs(1, 1, 500);
		CK_LED_ToggleLedForMs(1, 2, 500);

	}

}

