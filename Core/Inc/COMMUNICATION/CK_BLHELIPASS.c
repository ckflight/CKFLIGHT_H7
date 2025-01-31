
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "COMMUNICATION/CK_BLHELIPASS.h"
#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"

#include "stdio.h"


// WIRE PINS
#define ESC1_ONEWIRE_GPIO		MOTOR1_GPIO
#define ESC1_ONEWIRE_PIN		MOTOR1_PIN

#define ESC2_ONEWIRE_GPIO		MOTOR2_GPIO
#define ESC2_ONEWIRE_PIN		MOTOR2_PIN

#define ESC3_ONEWIRE_GPIO		MOTOR3_GPIO
#define ESC3_ONEWIRE_PIN		MOTOR3_PIN

#define ESC4_ONEWIRE_GPIO		MOTOR4_GPIO
#define ESC4_ONEWIRE_PIN		MOTOR4_PIN

// RX TX PINS
#define RX_GPIO					GPIOC
#define RX_PIN					0

#define TX_GPIO					GPIOC
#define TX_PIN					1

char esc_command = '.';

// This implementation rather than standalone version sometimes creates problem
// Unplug the ftdi cable then plug it and start passthrough mode by press p
// it works sometimes creates problems. If it stucks press e to reset system then try again it works eventually.

void CK_BLHELIPASS_Init(void){

	// ESC_Init initialized one wire esc pins as pwm alternate function so they must be reverted back
	// Directly resetting register values to their defaults as indicated in reference manual
	// After one wire system resets so there is no problem changing register to default values

	GPIO_TypeDef* gpiox = GPIOB;

	gpiox->MODER = 0x00000280;
	gpiox->OTYPER = 0;
	gpiox->PUPDR = 0x00000100;
	gpiox->AFR[0] = 0;
	gpiox->AFR[1] = 0;

	gpiox = GPIOA;

	gpiox->MODER &= ~(3u << ESC4_ONEWIRE_PIN * 2);
	gpiox->OTYPER &= ~(1u << ESC4_ONEWIRE_PIN);
	gpiox->PUPDR &= ~(3u << ESC4_ONEWIRE_PIN * 2);
	gpiox->AFR[0] &= ~(15u << 8);

	CK_GPIO_ClockEnable(GPIOA);
	CK_GPIO_ClockEnable(GPIOC);
	CK_GPIO_ClockEnable(GPIOB);

	TX_GPIO->MODER |= (1u << TX_PIN * 2);					// TX Output
	RX_GPIO->MODER &= ~(3u << RX_PIN * 2);					// RX Input / Default
	RX_GPIO->PUPDR |= (1u << RX_PIN * 2);					// RX Pullup

	CK_GPIO_SetPin(TX_GPIO ,TX_PIN);						// TX High

	CK_BLHELIPASS_ESC1_Passthrough();						// Start with first one

	while(1);
}

void CK_BLHELIPASS_ESC1_Passthrough(void){

	CK_BLHELIPASS_Set_ONEWIRE_Input(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);		// ONEWIRE Input
	CK_BLHELIPASS_ONEWIRE_PULLUP(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);			// ONEWIRE Pullup

	CK_PRINTER_PrintlnString("ESC1 Selected.");

	while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN));				// Wait RX go low / data incoming

	while(1){

		CK_BLHELIPASS_ReadUSBCommand();						// Check for which esc passthrough
		CK_BLHELIPASS_ApplyUSBCommand();						// Go to requested esc passthrough

		CK_BLHELIPASS_Set_ONEWIRE_Output(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);
		//ONEWIRE_CLEARPULLUP;

		CK_GPIO_ClearPin(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);

		CK_GPIO_ClearPin(TX_GPIO, TX_PIN);


		while(!CK_GPIO_ReadPin(RX_GPIO, RX_PIN));
		CK_GPIO_SetPin(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);

		CK_BLHELIPASS_Set_ONEWIRE_Input(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN);
		//ONEWIRE_PULLUP;

		while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN)){

			if(CK_GPIO_ReadPin(ESC1_ONEWIRE_GPIO, ESC1_ONEWIRE_PIN)){
				CK_GPIO_SetPin(TX_GPIO ,TX_PIN);
			}
			else{
				CK_GPIO_ClearPin(TX_GPIO ,TX_PIN);
			}

			CK_BLHELIPASS_ReadUSBCommand();							// Check for which esc passthrough
			CK_BLHELIPASS_ApplyUSBCommand();							// Go to requested esc passthrough
		}
	}
}

void CK_BLHELIPASS_ESC2_Passthrough(void){

	CK_BLHELIPASS_Set_ONEWIRE_Input(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);		// ONEWIRE Input
	CK_BLHELIPASS_ONEWIRE_PULLUP(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);		// ONEWIRE Pullup

	CK_PRINTER_PrintlnString("ESC2 Selected.");

	while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN));				// Wait RX go low / data incoming

	while(1){

		CK_BLHELIPASS_ReadUSBCommand();						// Check for which esc passthrough
		CK_BLHELIPASS_ApplyUSBCommand();						// Go to requested esc passthrough

		CK_BLHELIPASS_Set_ONEWIRE_Output(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);
		//ONEWIRE_CLEARPULLUP;

		CK_GPIO_ClearPin(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);

		CK_GPIO_ClearPin(TX_GPIO, TX_PIN);


		while(!CK_GPIO_ReadPin(RX_GPIO, RX_PIN));
		CK_GPIO_SetPin(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);

		CK_BLHELIPASS_Set_ONEWIRE_Input(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN);
		//ONEWIRE_PULLUP;

		while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN)){

			if(CK_GPIO_ReadPin(ESC2_ONEWIRE_GPIO, ESC2_ONEWIRE_PIN)){
				CK_GPIO_SetPin(TX_GPIO ,TX_PIN);
			}
			else{
				CK_GPIO_ClearPin(TX_GPIO ,TX_PIN);
			}

			CK_BLHELIPASS_ReadUSBCommand();							// Check for which esc passthrough
			CK_BLHELIPASS_ApplyUSBCommand();							// Go to requested esc passthrough
		}
	}
}

void CK_BLHELIPASS_ESC3_Passthrough(void){

	CK_BLHELIPASS_Set_ONEWIRE_Input(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);		// ONEWIRE Input
	CK_BLHELIPASS_ONEWIRE_PULLUP(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);		// ONEWIRE Pullup

	CK_PRINTER_PrintlnString("ESC3 Selected.");

	while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN));				// Wait RX go low / data incoming

	while(1){

		CK_BLHELIPASS_ReadUSBCommand();						// Check for which esc passthrough
		CK_BLHELIPASS_ApplyUSBCommand();						// Go to requested esc passthrough

		CK_BLHELIPASS_Set_ONEWIRE_Output(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);
		//ONEWIRE_CLEARPULLUP;

		CK_GPIO_ClearPin(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);

		CK_GPIO_ClearPin(TX_GPIO, TX_PIN);


		while(!CK_GPIO_ReadPin(RX_GPIO, RX_PIN));
		CK_GPIO_SetPin(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);

		CK_BLHELIPASS_Set_ONEWIRE_Input(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN);
		//ONEWIRE_PULLUP;

		while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN)){

			if(CK_GPIO_ReadPin(ESC3_ONEWIRE_GPIO, ESC3_ONEWIRE_PIN)){
				CK_GPIO_SetPin(TX_GPIO ,TX_PIN);
			}
			else{
				CK_GPIO_ClearPin(TX_GPIO ,TX_PIN);
			}

			CK_BLHELIPASS_ReadUSBCommand();							// Check for which esc passthrough
			CK_BLHELIPASS_ApplyUSBCommand();							// Go to requested esc passthrough
		}
	}
}

void CK_BLHELIPASS_ESC4_Passthrough(void){

	CK_BLHELIPASS_Set_ONEWIRE_Input(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);		// ONEWIRE Input
	CK_BLHELIPASS_ONEWIRE_PULLUP(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);		// ONEWIRE Pullup

	CK_PRINTER_PrintlnString("ESC4 Selected.");

	while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN));					// Wait RX go low / data incoming

	while(1){

		CK_BLHELIPASS_ReadUSBCommand();							// Check for which esc passthrough
		CK_BLHELIPASS_ApplyUSBCommand();							// Go to requested esc passthrough

		CK_BLHELIPASS_Set_ONEWIRE_Output(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);
		//ONEWIRE_CLEARPULLUP;

		CK_GPIO_ClearPin(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);

		CK_GPIO_ClearPin(TX_GPIO, TX_PIN);


		while(!CK_GPIO_ReadPin(RX_GPIO, RX_PIN));
		CK_GPIO_SetPin(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);

		CK_BLHELIPASS_Set_ONEWIRE_Input(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN);
		//ONEWIRE_PULLUP;

		while(CK_GPIO_ReadPin(RX_GPIO, RX_PIN)){

			if(CK_GPIO_ReadPin(ESC4_ONEWIRE_GPIO, ESC4_ONEWIRE_PIN)){
				CK_GPIO_SetPin(TX_GPIO ,TX_PIN);
			}
			else{
				CK_GPIO_ClearPin(TX_GPIO ,TX_PIN);
			}

			CK_BLHELIPASS_ReadUSBCommand();							// Check for which esc passthrough
			CK_BLHELIPASS_ApplyUSBCommand();							// Go to requested esc passthrough
		}
	}
}


void CK_BLHELIPASS_Set_ONEWIRE_Output(GPIO_TypeDef* GPIOx, uint8_t pin){

	GPIOx->MODER |= (1u << pin * 2);

}

void CK_BLHELIPASS_Set_ONEWIRE_Input(GPIO_TypeDef* GPIOx, uint8_t pin){

	GPIOx->MODER &= ~(3u << pin * 2);

}

void CK_BLHELIPASS_ONEWIRE_PULLUP(GPIO_TypeDef* GPIOx, uint8_t pin){

	GPIOx->PUPDR |= (1u << pin * 2);

}

void CK_BLHELIPASS_ONEWIRE_CLEARPULLUP(GPIO_TypeDef* GPIOx, uint8_t pin){

	GPIOx->PUPDR &= ~(3u << pin * 2);

}

void CK_BLHELIPASS_ReadUSBCommand(void){
	uint8_t data;
	if(CK_USBD_ReadData(&data)){
		esc_command = (char)data;
	}
}

void CK_BLHELIPASS_ApplyUSBCommand(void){

	if(esc_command == '.'){
		return;
	}
	else if(esc_command == '1'){
		esc_command = '.';
		CK_BLHELIPASS_ESC1_Passthrough();
	}
	else if(esc_command == '2'){
		esc_command = '.';
		CK_BLHELIPASS_ESC2_Passthrough();
	}
	else if(esc_command == '3'){
		esc_command = '.';
		CK_BLHELIPASS_ESC3_Passthrough();
	}
	else if(esc_command == '4'){
		esc_command = '.';
		CK_BLHELIPASS_ESC4_Passthrough();
	}
	else if(esc_command == 'e'){
		esc_command = '.';
		CK_PRINTER_PrintlnString("RESTARTING THE SYSTEM");
		CK_TIME_DelayMilliSec(1000);
		NVIC_SystemReset();
	}

	else{
		return;
	}
}
