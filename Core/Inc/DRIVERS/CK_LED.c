
#include "CK_GPIO.h"
#include "CK_LED.h"
#include "CK_TIME_HAL.h"

int led1_st = 0;
int led2_st = 0;

GPIO_TypeDef* CK_LED1_GPIO;
uint8_t CK_LED1_GPIO_PIN;

GPIO_TypeDef* CK_LED2_GPIO;
uint8_t CK_LED2_GPIO_PIN;

void CK_LED_Init1(GPIO_TypeDef* gpio_, uint8_t gpio_pin_){

	CK_LED1_GPIO		= gpio_;
	CK_LED1_GPIO_PIN	= gpio_pin_;

	CK_GPIO_Init(CK_LED1_GPIO, CK_LED1_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	CK_LED_DisableLed(1);
}

void CK_LED_Init2(GPIO_TypeDef* gpio_, uint8_t gpio_pin_){

	CK_LED2_GPIO		= gpio_;
	CK_LED2_GPIO_PIN	= gpio_pin_;

	CK_GPIO_Init(CK_LED2_GPIO, CK_LED2_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	CK_LED_DisableLed(2);
}

void CK_LED_EnableLed(int led_num){

	#if LED1_
	if(led_num == 1){
		if(LED1_ACTIVE_LOW){
			CK_GPIO_ClearPin(CK_LED1_GPIO, CK_LED1_GPIO_PIN);
		}else{
			CK_GPIO_SetPin(CK_LED1_GPIO, CK_LED1_GPIO_PIN);
		}

	    led1_st = 1;
	}
	#endif

	#if LED2_
	else if(led_num == 2){
		if(LED2_ACTIVE_LOW){
			CK_GPIO_ClearPin(CK_LED2_GPIO, CK_LED2_GPIO_PIN);
		}else{
			CK_GPIO_SetPin(CK_LED2_GPIO, CK_LED2_GPIO_PIN);
		}
	    led2_st = 1;
	}
	#endif
}

void CK_LED_DisableLed(int led_num){

	#if LED1_
	if(led_num == 1){
		if(LED1_ACTIVE_LOW){
			CK_GPIO_SetPin(CK_LED1_GPIO, CK_LED1_GPIO_PIN);
		}else{
			CK_GPIO_ClearPin(CK_LED1_GPIO, CK_LED1_GPIO_PIN);
		}
	    led1_st = 0;
	}
	#endif

	#if LED2_
	if(led_num == 2){
		if(LED2_ACTIVE_LOW){
			CK_GPIO_SetPin(CK_LED2_GPIO, CK_LED2_GPIO_PIN);
		}else{
			CK_GPIO_ClearPin(CK_LED2_GPIO, CK_LED2_GPIO_PIN);
		}
	    led2_st = 0;
	}
	#endif
}

void CK_LED_ToggleLed(int led_num){

	#if LED1_
	if(led_num == 1){
		if(led1_st){
			CK_LED_DisableLed(1);
		}
		else{
			CK_LED_EnableLed(1);
		}
	}
	#endif

	#if LED2_
	if(led_num == 2){
		if(led2_st){
			CK_LED_DisableLed(2);
		}
		else{
			CK_LED_EnableLed(2);
		}
	}
	#endif

}

void CK_LED_ToggleLedForMs(int led_num, uint32_t loop, uint32_t time){

	for(int i = 0; i < loop; i++){
		CK_LED_ToggleLed(led_num);
		CK_TIME_DelayMilliSec(time);
	}

}


