
#include "FLIGHT/CK_PWM.h"

#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_SYSTEM.h"

#define CK_RCC_APB1ENR_TIM2_ENABLE          1u << 0
#define CK_RCC_APB1ENR_TIM3_ENABLE          1u << 1
#define CK_RCC_APB1ENR_TIM4_ENABLE          1u << 2

CK_PWM_Mode pwm_mode;

int pwm_freq;
int pwm_duty_div;
int pwm_min_microsec;
int pwm_max_microsec;

void CK_PWM_Init(CK_PWM_Mode md){

	pwm_mode = md;

	if(pwm_mode == PWM_ONESHOT42){
		// 8KHz max freq, 125 microsecond, min command 1000 is 41 microsec and max 2000 is 82
		pwm_freq = 8000;
		pwm_min_microsec = 42;
		pwm_max_microsec = 125;

	}
	else if(pwm_mode == PWM_ONESHOT125){
		// 3.7KHz max freq, 250 microsecond, min command 1000 is 125 microsec and max 2000 is 250
		pwm_freq = 4000;
		pwm_min_microsec = 125;
		pwm_max_microsec = 250;
	}

	pwm_duty_div = PWM_MIN_RANGE * pwm_max_microsec / pwm_min_microsec;

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	CK_PWM_Init1();
	CK_PWM_Init2();
	CK_PWM_Init3();
	CK_PWM_Init4();

	// Send start signal to motors
	CK_PWM_SetMotor1(PWM_DISARM_VALUE);
	CK_PWM_SetMotor2(PWM_DISARM_VALUE);
	CK_PWM_SetMotor3(PWM_DISARM_VALUE);
	CK_PWM_SetMotor4(PWM_DISARM_VALUE);

}

void CK_PWM_InitEndPoints(float* outputLow, float* outputHigh, float* disarmMotorOutput){

	*outputLow = PWM_MIN_THROTTLE;

	*outputHigh = PWM_MAX_THROTTLE;

	*disarmMotorOutput = PWM_DISARM_VALUE;

}

void CK_PWM_Init1(void){

	CK_SYSTEM_TIMER_ClockEnable(MOTOR1_TIM);

	MOTOR1_TIM->PSC = 4; 				// Main clock prescalar
	MOTOR1_TIM->ARR = ((CK_SYSTEM_GetTIMERClock(MOTOR1_TIM) / (MOTOR1_TIM->PSC+1)) / pwm_freq) - 1; // Freq. is fix for each ch on same timer

    if(MOTOR1_TIM == TIM6 || MOTOR1_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(MOTOR1_TIM_CH == TIM_CHANNEL_1){
        	MOTOR1_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
        	MOTOR1_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(MOTOR1_TIM_CH == TIM_CHANNEL_2){
    		MOTOR1_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		MOTOR1_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(MOTOR1_TIM_CH == TIM_CHANNEL_3){
    		MOTOR1_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		MOTOR1_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(MOTOR1_TIM_CH == TIM_CHANNEL_4){
    		MOTOR1_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		MOTOR1_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}
    }
    // CR1 default Edge Alligned, Upcounting
    MOTOR1_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    MOTOR1_TIM->CR1 |= (1u << 2); 	// Update request source
    MOTOR1_TIM->EGR |= 1u << 0; 	// Update generation enabled
    MOTOR1_TIM->CR1 |= 1u << 0; 	// Counter enable

}

void CK_PWM_Init2(void){

	CK_SYSTEM_TIMER_ClockEnable(MOTOR2_TIM);

	MOTOR2_TIM->PSC = 4; 				// Main clock prescalar
	MOTOR2_TIM->ARR = ((CK_SYSTEM_GetTIMERClock(MOTOR2_TIM) / (MOTOR2_TIM->PSC+1)) / pwm_freq) - 1; // Freq. is fix for each ch on same timer

    if(MOTOR2_TIM == TIM6 || MOTOR2_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(MOTOR2_TIM_CH == TIM_CHANNEL_1){
        	MOTOR2_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
        	MOTOR2_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(MOTOR2_TIM_CH == TIM_CHANNEL_2){
    		MOTOR2_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		MOTOR2_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(MOTOR2_TIM_CH == TIM_CHANNEL_3){
    		MOTOR2_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		MOTOR2_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(MOTOR2_TIM_CH == TIM_CHANNEL_4){
    		MOTOR2_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		MOTOR2_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}
    }
    // CR1 default Edge Alligned, Upcounting
    MOTOR2_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    MOTOR2_TIM->CR1 |= (1u << 2); 	// Update request source
    MOTOR2_TIM->EGR |= 1u << 0; 	// Update generation enabled
    MOTOR2_TIM->CR1 |= 1u << 0; 	// Counter enable
}

void CK_PWM_Init3(void){

	CK_SYSTEM_TIMER_ClockEnable(MOTOR3_TIM);

	MOTOR3_TIM->PSC = 4; 				// Main clock prescalar
	MOTOR3_TIM->ARR = ((CK_SYSTEM_GetTIMERClock(MOTOR3_TIM) / (MOTOR3_TIM->PSC+1)) / pwm_freq) - 1; // Freq. is fix for each ch on same timer

    if(MOTOR3_TIM == TIM6 || MOTOR3_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(MOTOR3_TIM_CH == TIM_CHANNEL_1){
        	MOTOR3_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
        	MOTOR3_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(MOTOR3_TIM_CH == TIM_CHANNEL_2){
    		MOTOR3_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		MOTOR3_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(MOTOR3_TIM_CH == TIM_CHANNEL_3){
    		MOTOR3_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		MOTOR3_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(MOTOR3_TIM_CH == TIM_CHANNEL_4){
    		MOTOR3_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		MOTOR3_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}
    }
    // CR1 default Edge Alligned, Upcounting
    MOTOR3_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    MOTOR3_TIM->CR1 |= (1u << 2); 	// Update request source
    MOTOR3_TIM->EGR |= 1u << 0; 	// Update generation enabled
    MOTOR3_TIM->CR1 |= 1u << 0; 	// Counter enable
}

void CK_PWM_Init4(void){

	CK_SYSTEM_TIMER_ClockEnable(MOTOR4_TIM);

	MOTOR4_TIM->PSC = 4; 				// Main clock prescalar
	MOTOR4_TIM->ARR = ((CK_SYSTEM_GetTIMERClock(MOTOR4_TIM) / (MOTOR4_TIM->PSC+1)) / pwm_freq) - 1; // Freq. is fix for each ch on same timer

    if(MOTOR4_TIM == TIM6 || MOTOR4_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(MOTOR4_TIM_CH == TIM_CHANNEL_1){
        	MOTOR4_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
        	MOTOR4_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(MOTOR4_TIM_CH == TIM_CHANNEL_2){
    		MOTOR4_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		MOTOR4_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(MOTOR4_TIM_CH == TIM_CHANNEL_3){
    		MOTOR4_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		MOTOR4_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(MOTOR4_TIM_CH == TIM_CHANNEL_4){
    		MOTOR4_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		MOTOR4_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}
    }
    // CR1 default Edge Alligned, Upcounting
    MOTOR4_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    MOTOR4_TIM->CR1 |= (1u << 2); 	// Update request source
    MOTOR4_TIM->EGR |= 1u << 0; 	// Update generation enabled
    MOTOR4_TIM->CR1 |= 1u << 0; 	// Counter enable
}

void CK_PWM_SetMotor1(int dty){

	if(MOTOR1_TIM_CH == TIM_CHANNEL_1){
		MOTOR1_TIM->CCR1 = ((dty * MOTOR1_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_2){
		MOTOR1_TIM->CCR2 = ((dty * MOTOR1_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_3){
		MOTOR1_TIM->CCR3 = ((dty * MOTOR1_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_4){
		MOTOR1_TIM->CCR4 = ((dty * MOTOR1_TIM->ARR) / pwm_duty_div);
	}
}

void CK_PWM_SetMotor2(int dty){

	if(MOTOR2_TIM_CH == TIM_CHANNEL_1){
		MOTOR2_TIM->CCR1 = ((dty * MOTOR2_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_2){
		MOTOR2_TIM->CCR2 = ((dty * MOTOR2_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_3){
		MOTOR2_TIM->CCR3 = ((dty * MOTOR2_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_4){
		MOTOR2_TIM->CCR4 = ((dty * MOTOR2_TIM->ARR) / pwm_duty_div);
	}

}

void CK_PWM_SetMotor3(int dty){

	if(MOTOR3_TIM_CH == TIM_CHANNEL_1){
		MOTOR3_TIM->CCR1 = ((dty * MOTOR3_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR3_TIM_CH == TIM_CHANNEL_2){
		MOTOR3_TIM->CCR2 = ((dty * MOTOR3_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR3_TIM_CH == TIM_CHANNEL_3){
		MOTOR3_TIM->CCR3 = ((dty * MOTOR3_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR3_TIM_CH == TIM_CHANNEL_4){
		MOTOR3_TIM->CCR4 = ((dty * MOTOR3_TIM->ARR) / pwm_duty_div);
	}

}

void CK_PWM_SetMotor4(int dty){

	if(MOTOR4_TIM_CH == TIM_CHANNEL_1){
		MOTOR4_TIM->CCR1 = ((dty * MOTOR4_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR4_TIM_CH == TIM_CHANNEL_2){
		MOTOR4_TIM->CCR2 = ((dty * MOTOR4_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR4_TIM_CH == TIM_CHANNEL_3){
		MOTOR4_TIM->CCR3 = ((dty * MOTOR4_TIM->ARR) / pwm_duty_div);
	}
	else if(MOTOR4_TIM_CH == TIM_CHANNEL_4){
		MOTOR4_TIM->CCR4 = ((dty * MOTOR4_TIM->ARR) / pwm_duty_div);
	}
}



