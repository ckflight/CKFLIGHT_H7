
#include "CK_SYSTEM.h"

uint32_t F_CPU = 0;

void CK_SYSTEM_SetSystemClock(systemClock_e clk){

	if(clk == SYSTEM_CLK_480MHz){

		F_CPU = 480000000L;

		RCC_OscInitTypeDef RCC_OscInitStruct = {0};
		RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

		/** Supply configuration update enable
		*/
		HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
		/** Configure the main internal regulator output voltage
		*/
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

		while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
		/** Initializes the RCC Oscillators according to the specified parameters
		* in the RCC_OscInitTypeDef structure.
		*/
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
		RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
		RCC_OscInitStruct.PLL.PLLM = 1;
		RCC_OscInitStruct.PLL.PLLN = 120;
		RCC_OscInitStruct.PLL.PLLP = 2;
		RCC_OscInitStruct.PLL.PLLQ = 15;
		RCC_OscInitStruct.PLL.PLLR = 2;
		RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
		RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
		RCC_OscInitStruct.PLL.PLLFRACN = 0;
		HAL_RCC_OscConfig(&RCC_OscInitStruct);

		/** Initializes the CPU, AHB and APB buses clocks
		*/
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
								  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
		RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

		HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

	}
	else if(clk == SYSTEM_CLK_400MHz){

		F_CPU = 400000000L;


	}

	uint32_t freq = HAL_RCC_GetSysClockFreq(); // 480 MHz
	freq = HAL_RCC_GetHCLKFreq(); // 240 MHz
	freq = HAL_RCC_GetPCLK1Freq(); // 120 MHz
	freq = HAL_RCC_GetPCLK2Freq(); // 120 MHz
	UNUSED(freq);

}

uint32_t CK_SYSTEM_GetAPB1Clock(void){
	return HAL_RCC_GetPCLK1Freq();
}

uint32_t CK_SYSTEM_GetAPB2Clock(void){
	return HAL_RCC_GetPCLK2Freq();
}

uint32_t CK_SYSTEM_GetHCLKClock(void){
	return HAL_RCC_GetHCLKFreq();
}

uint32_t CK_SYSTEM_GetPCLK1Clock(void){
	return HAL_RCC_GetPCLK1Freq();
}

uint32_t CK_SYSTEM_GetPCLK2Clock(void){
	return HAL_RCC_GetPCLK2Freq();
}

void CK_SYSTEM_TIMER_ClockEnable(TIM_TypeDef* timer){

	if(timer == TIM1){
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
	else if(timer == TIM2){
		__HAL_RCC_TIM2_CLK_ENABLE();
	}
	else if(timer == TIM3){
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
	else if(timer == TIM4){
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
	else if(timer == TIM5){
		__HAL_RCC_TIM5_CLK_ENABLE();
	}
	else if(timer == TIM6){
		__HAL_RCC_TIM6_CLK_ENABLE();
	}
	else if(timer == TIM7){
		__HAL_RCC_TIM7_CLK_ENABLE();
	}
	else if(timer == TIM8){
		__HAL_RCC_TIM8_CLK_ENABLE();
	}
	else if(timer == TIM16){
		__HAL_RCC_TIM16_CLK_ENABLE();
	}

}

uint32_t CK_SYSTEM_GetTIMERClock(TIM_TypeDef* timer){

	uint32_t clock = 0;

	// Timers use x2 of APB1,2 clocks
	// TIM1,8 APB2 so APB2 Clock * 2
	// TIM2,3,4,5,6,7 APBL1 so APB1 Clock * 2

	if(timer == TIM1){
		clock = CK_SYSTEM_GetAPB2Clock();
	}
	else if(timer == TIM2){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM3){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM4){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM5){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM6){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM7){
		clock = CK_SYSTEM_GetAPB1Clock();
	}
	else if(timer == TIM8){
		clock = CK_SYSTEM_GetAPB2Clock();
	}
	else if(timer == TIM16){
		clock = CK_SYSTEM_GetAPB2Clock() / 2;
	}

	return clock * 2;

}

uint32_t CK_SYSTEM_GetUARTClock(USART_TypeDef* uart){

	uint32_t clock = 0;

	// USART1,6 PCLK2 120MHz
	// USART2,3,4,5,7,8 PCLK2 120MHz

	if(uart == USART1){
		clock = 120000000L;
	}
	else if(uart == USART2){
		clock = 120000000L;
	}
	else if(uart == USART3){
		clock = 120000000L;
	}
	else if(uart == UART4){
		clock = 120000000L;
	}
	else if(uart == UART5){
		clock = 120000000L;
	}
	else if(uart == USART6){
		clock = 120000000L;
	}
	else if(uart == UART7){
		clock = 120000000L;
	}
	else if(uart == UART8){
		clock = 120000000L;
	}

	return clock;

}












