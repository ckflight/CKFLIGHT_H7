
#include "CK_BUZZER.h"

#include "CK_TIME_HAL.h"
#include "CK_GPIO.h"
#include "CK_SYSTEM.h"

#include "FLIGHT/CK_RECEIVER.h"

#include "stdbool.h"

#define BUZZER_TIME1    50
#define BUZZER_TIME2    25

// Buzzer is active low HAL library makes it BUZZER_ACTIVE_PWM_DUTY percent low which is what we want
#define BUZZER_ACTIVE_PWM_DUTY		50

typedef enum{

	BUZZER_ENABLED,
	BUZZER_DISABLED,
}buzzer_state_e;

typedef struct{

	bool is_init;
	buzzer_state_e state;
	buzzer_state_e toggle_state;

	syncTimer_t sync;

	buzzer_mode_e mode;

	uint32_t pwm_frequency;

}buzzer_s;

GPIO_TypeDef* _BUZZER_GPIO;
uint8_t _BUZZER_GPIO_PIN;
TIM_HandleTypeDef htim_buzzer;

buzzer_s buzzer;

void CK_BUZZER_Init(GPIO_TypeDef* gpio_, uint8_t gpio_pin_, buzzer_mode_e md){

	_BUZZER_GPIO = gpio_;

	_BUZZER_GPIO_PIN = gpio_pin_;

	buzzer.is_init = false;

	buzzer.mode = md;

	buzzer.state = BUZZER_DISABLED;

	buzzer.toggle_state = BUZZER_DISABLED;

#if BUZZER_PWM

    buzzer.pwm_frequency = 4000;

    /*
    BUZZER_TIM->PSC = 0; 				// Main clock prescalar
    BUZZER_TIM->ARR = (CK_SYSTEM_GetTIMERClock(BUZZER_TIM) / frequency ) - 1; // Freq. is fix for each ch on same timer

    if(BUZZER_TIM == TIM6 || BUZZER_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(BUZZER_TIM_CH == TIM_CHANNEL_1){
        	BUZZER_TIM->CCR1 = (BUZZER_TIM->ARR * 50) / 100 ; // %50
        	BUZZER_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
        	BUZZER_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(BUZZER_TIM_CH == TIM_CHANNEL_2){
    		BUZZER_TIM->CCR2 = (BUZZER_TIM->ARR * 50) / 100 ; // %50
    		BUZZER_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		BUZZER_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(BUZZER_TIM_CH == TIM_CHANNEL_3){
    		BUZZER_TIM->CCR3 = (BUZZER_TIM->ARR * 50) / 100 ; // %50
    		BUZZER_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		BUZZER_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(BUZZER_TIM_CH == TIM_CHANNEL_4){
    		BUZZER_TIM->CCR4 = (BUZZER_TIM->ARR * 50) / 100 ; // %50
    		BUZZER_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		BUZZER_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}

    }

    // CR1 default Edge Alligned, Upcounting
    BUZZER_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    BUZZER_TIM->CR1 |= (1u << 2); 	// Update request source

    BUZZER_TIM->DIER |= 1u << 0; 	// Update interrupt enable
    BUZZER_TIM->EGR |= 1u << 0; 	// Update generation enabled

    BUZZER_TIM->CR1 |= 1u << 0; 	// Counter enable
	*/

    htim_buzzer.Instance 				= BUZZER_TIM;
    htim_buzzer.Init.Prescaler 			= 0;
    htim_buzzer.Init.CounterMode 		= TIM_COUNTERMODE_UP;
    htim_buzzer.Init.Period 			= (CK_SYSTEM_GetTIMERClock(BUZZER_TIM) / buzzer.pwm_frequency) - 1;
    htim_buzzer.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV2;
    htim_buzzer.Init.RepetitionCounter 	= 0;
    htim_buzzer.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim_buzzer);

	HAL_TIM_PWM_Init(&htim_buzzer);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 			= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 			= 0;
	sConfigOC.OCPolarity 		= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity 		= TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode 		= TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState 		= TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState 		= TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim_buzzer, &sConfigOC, BUZZER_TIM_CH);

	HAL_TIMEx_PWMN_Start(&htim_buzzer, BUZZER_TIM_CH);

    /* H7_V1 PA15 HAL Init
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim_buzzer.Instance 				= TIM2;
    htim_buzzer.Init.Prescaler 			= 0;
    htim_buzzer.Init.CounterMode 		= TIM_COUNTERMODE_UP;
    htim_buzzer.Init.Period 			= 59999;
    htim_buzzer.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV2;
    htim_buzzer.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim_buzzer) != HAL_OK);

    sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_buzzer, &sMasterConfig) != HAL_OK);

    sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
    sConfigOC.Pulse 		= 0;
    sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim_buzzer, &sConfigOC, TIM_CHANNEL_1);

    // Start PWM on TIM2 CH1
    HAL_TIM_PWM_Start(&htim_buzzer, TIM_CHANNEL_1);
	*/

	buzzer.is_init = true;

	CK_BUZZER_Disable();

#endif

#if BUZZER_DC

	CK_BUZZER_Disable();

	CK_BUZZER_Enable();

	buzzer.is_init = true;

#endif
}

void CK_BUZZER_Toggle(void){

	if(buzzer.is_init){
		if(buzzer.toggle_state == BUZZER_DISABLED){
			CK_BUZZER_Enable();
			buzzer.toggle_state = BUZZER_ENABLED;
		}
		else if(buzzer.toggle_state == BUZZER_ENABLED){
			CK_BUZZER_Disable();
			buzzer.toggle_state = BUZZER_DISABLED;
		}
	}
}

void CK_BUZZER_Enable(void){

	if(buzzer.is_init){

	#if BUZZER_PWM
		if(BUZZER_TIM_CH == TIM_CHANNEL_1){
			BUZZER_TIM->CCR1 = (BUZZER_TIM->ARR * BUZZER_ACTIVE_PWM_DUTY) / 100 ;
		}
		else if(BUZZER_TIM_CH == TIM_CHANNEL_2){
			BUZZER_TIM->CCR2 = (BUZZER_TIM->ARR * BUZZER_ACTIVE_PWM_DUTY) / 100 ;
		}
		else if(BUZZER_TIM_CH == TIM_CHANNEL_3){
			BUZZER_TIM->CCR3 = (BUZZER_TIM->ARR * BUZZER_ACTIVE_PWM_DUTY) / 100 ;
		}
		else if(BUZZER_TIM_CH == TIM_CHANNEL_4){
			BUZZER_TIM->CCR4 = (BUZZER_TIM->ARR * BUZZER_ACTIVE_PWM_DUTY) / 100 ;
		}
	#endif

	#if BUZZER_DC
		CK_GPIO_SetPin(_BUZZER_GPIO, _BUZZER_GPIO_PIN); // Set pin to low.
	#endif

		buzzer.state = BUZZER_ENABLED;
	}
}


void CK_BUZZER_Disable(void){

	if(buzzer.is_init){

		#if BUZZER_PWM

			// 0% idle is at 3.3 and in this way it draws less current.
			if(BUZZER_TIM_CH == TIM_CHANNEL_1){
				BUZZER_TIM->CCR1 = (BUZZER_TIM->ARR * 0) / 100 ; // %0
			}
			else if(BUZZER_TIM_CH == TIM_CHANNEL_2){
				BUZZER_TIM->CCR2 = (BUZZER_TIM->ARR * 0) / 100 ; // %0
			}
			else if(BUZZER_TIM_CH == TIM_CHANNEL_3){
				BUZZER_TIM->CCR3 = (BUZZER_TIM->ARR * 0) / 100 ; // %0
			}
			else if(BUZZER_TIM_CH == TIM_CHANNEL_4){
				BUZZER_TIM->CCR4 = (BUZZER_TIM->ARR * 0) / 100 ; // %0
			}


		#endif
		#if BUZZER_DC

			CK_GPIO_ClearPin(_BUZZER_GPIO, _BUZZER_GPIO_PIN); // Set pin to low.

		#endif

			buzzer.state = BUZZER_DISABLED;
	}

}

/*
 * Activate buzzer if BUZZER_FLAG is true.
 */
void CK_BUZZER_CheckBuzzer(void){

	if(flags.BUZZER){
		CK_BUZZER_Enable();
	}
	else{
		CK_BUZZER_Disable();
	}
}

// 3 beeps. To indicate the used method intitialized successfully.
// such as microcard initialized, gps is fixed etc.
void CK_BUZZER_Tone1(void){

	if(buzzer.is_init){

		// I put delay to prevent mixing to previous tones.
		CK_TIME_DelayMilliSec(BUZZER_TIME2 * 10);

		int i = 3;
		while(i--){
			CK_BUZZER_Enable();
			CK_TIME_DelayMilliSec(BUZZER_TIME1);
			CK_BUZZER_Disable();
			CK_TIME_DelayMilliSec(BUZZER_TIME1);
		}

	}

}


// 10 quick beeps. To indicate loop is starting.
void CK_BUZZER_Tone2(void){

	if(buzzer.is_init){

		// I put delay to prevent mixing to previous tones.
		CK_TIME_DelayMilliSec(BUZZER_TIME2 * 10);

		int i = 10;
		while(i--){
			CK_BUZZER_Enable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2);
			CK_BUZZER_Disable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2);
		}

	}

}


// A quick buzz like electric noise mainly used to indicate timeout occured.
// do not use much delay because if timeout occurs it will keep buzzing
// at every refreshing of that sensor. Longer delay will affect loop frequency
void CK_BUZZER_Tone3(void){

	if(buzzer.is_init){

		// I put delay to prevent mixing to previous tones.
		CK_TIME_DelayMilliSec(BUZZER_TIME2 * 10);

		int i = 5;
		while(i--){
			CK_BUZZER_Enable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2/10);
			CK_BUZZER_Disable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2/10);
		}

	}

}

// 3 beeps. To indicate the used method intitialized successfully.
// this is the delayless version of tone 1 for microcard logging done indication.
void CK_BUZZER_Tone4(void){

	if(buzzer.is_init){

		int i = 3;
		while(i--){
			CK_BUZZER_Enable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2);
			CK_BUZZER_Disable();
			CK_TIME_DelayMilliSec(BUZZER_TIME2);
		}

	}

}











