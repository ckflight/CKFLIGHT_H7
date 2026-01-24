
#include "FLIGHT/CK_PERIPHERAL.h"

#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_ADC.h"

TIM_HandleTypeDef htim_main_interrupt;

// Initializing hardware from one place is better for controlling different boards and
// some sensors uses same peripheral so it must be initialised once.

void CK_PERIPHERAL_Init(targetFreq_e target_period){

	// If a gpio pin is jtag etc pin my gpio library could not set it as output.
	// Use hal library.
#if MAIN_INTERRUPT_
	CK_PERIPHERAL_MainInterruptInit(target_period);
#endif

#if GYRO1_SPI_
    CK_GPIO_ClockEnable(GYRO1_CS_PORT);
    CK_GPIO_Init(GYRO1_CS_PORT, GYRO1_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
    CK_GPIO_SetPin(GYRO1_CS_PORT, GYRO1_CS_PIN); //Set CS High for Idle

	#if GYRO1_USE_INT == 1
    CK_GPIO_ClockEnable(GYRO1_INT_PORT);
	CK_GPIO_Init(GYRO1_INT_PORT, GYRO1_INT_PIN, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	#endif
#endif

#if GYRO2_SPI_
    CK_GPIO_ClockEnable(GYRO2_CS_PORT);
	CK_GPIO_Init(GYRO2_CS_PORT, GYRO2_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(GYRO2_CS_PORT, GYRO2_CS_PIN); //Set CS High for Idle

	#if GYRO2_USE_INT == 1
	CK_GPIO_ClockEnable(GYRO2_INT_PORT);
	CK_GPIO_Init(GYRO2_INT_PORT, GYRO2_INT_PIN, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	#endif
#endif

#if EXT_SPI_

	#if EXT_CS1_
		CK_GPIO_ClockEnable(EXT_CS1_PORT);
		CK_GPIO_Init(EXT_CS1_PORT, EXT_CS1_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_SetPin(EXT_CS1_PORT, EXT_CS1_PIN); //Set CS High for Idle
	#endif

	#if EXT_CS2_
		CK_GPIO_ClockEnable(EXT_CS2_PORT);
		CK_GPIO_Init(EXT_CS2_PORT, EXT_CS2_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_SetPin(EXT_CS2_PORT, EXT_CS2_PIN); //Set CS High for Idle
	#endif

#endif

#if ACC1_SPI_
    CK_GPIO_ClockEnable(ACC1_CS_PORT);
    CK_GPIO_Init(ACC1_CS_PORT, ACC1_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(ACC1_CS_PORT, ACC1_CS_PIN); //Set CS High for Idle
#endif

#if ACC2_SPI_
	CK_GPIO_ClockEnable(ACC2_CS_PORT);
	CK_GPIO_Init(ACC2_CS_PORT, ACC2_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(ACC2_CS_PORT, ACC2_CS_PIN); //Set CS High for Idle
#endif

#if MAG_SPI_
    CK_GPIO_ClockEnable(MAG_CS_PORT);
	CK_GPIO_Init(MAG_CS_PORT, MAG_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(MAG_CS_PORT, MAG_CS_PIN); //Set CS High for Idle
#endif

#if BARO_SPI_
    CK_GPIO_ClockEnable(BARO_CS_PORT);
    CK_GPIO_Init(BARO_CS_PORT, BARO_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(BARO_CS_PORT, BARO_CS_PIN); //Set CS High for Idle
#endif

#if BARO_I2C_
	if(!CK_I2C_CheckInitialized(BARO_I2C)){
		CK_I2C_Init(BARO_I2C, CK_I2C_400Khz, USE_HAL_I2C);
	}
#endif

#if LOG_SPI_
    CK_GPIO_ClockEnable(MICROCARD_CS_PORT);
    CK_GPIO_Init(MICROCARD_CS_PORT, MICROCARD_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
    CK_GPIO_SetPin(MICROCARD_CS_PORT, MICROCARD_CS_PIN); //Set CS High for Idle

#if USE_MICROCARD_DETECT == true
    CK_GPIO_ClockEnable(MICROCARD_DETECT_PORT);
    CK_GPIO_Init(MICROCARD_DETECT_PORT, MICROCARD_DETECT_PIN, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#endif
#endif

#if LED1_
    CK_GPIO_ClockEnable(LED1_GPIO);
#endif

#if LED2_
    CK_GPIO_ClockEnable(LED2_GPIO);
#endif

#if OSD_ONBOARD_
    CK_GPIO_ClockEnable(OSD_CS_PORT);
    CK_GPIO_Init(OSD_CS_PORT, OSD_CS_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
    CK_GPIO_SetPin(OSD_CS_PORT, OSD_CS_PIN); //Set CS High for Idle
#endif

#if VTX_SWITCH_
    CK_GPIO_ClockEnable(VTX_SWITCH_GPIO);
	CK_GPIO_Init(VTX_SWITCH_GPIO, VTX_SWITCH_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(VTX_SWITCH_GPIO, VTX_SWITCH_GPIO_PIN);   // OFF
	CK_GPIO_ClearPin(VTX_SWITCH_GPIO, VTX_SWITCH_GPIO_PIN); // ON
#endif

#if CAMERA_SWITCH_
    CK_GPIO_ClockEnable(CAMERA_SWITCH_GPIO);
	CK_GPIO_Init(CAMERA_SWITCH_GPIO, CAMERA_SWITCH_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#if CAMERA_SELECT == 1
	CK_GPIO_ClearPin(CAMERA_SWITCH_GPIO, CAMERA_SWITCH_GPIO_PIN); // Select Camera 1
#elif CAMERA_SELECT == 2
	CK_GPIO_SetPin(CAMERA_SWITCH_GPIO, CAMERA_SWITCH_GPIO_PIN);   // Select Camera 2
#endif
#endif

#if SCOPE_CHECK_
    CK_GPIO_ClockEnable(SCOPE_CHECK_GPIO);
    CK_GPIO_Init(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);

#endif

    CK_SPIx_CR1_Fclk_Div clock_rate = 0;

#if GYRO1_SPI_
	if(!CK_SPI_CheckInitialized(GYRO1_SPI)){
		clock_rate = CK_SPI_GetClockRate(GYRO1_SPI, GYRO1_SPI_CLOCK);
		CK_SPI_Init(GYRO1_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}

#endif

#if GYRO2_SPI_
	if(!CK_SPI_CheckInitialized(GYRO2_SPI)){
		clock_rate = CK_SPI_GetClockRate(GYRO2_SPI, GYRO2_SPI_CLOCK);
		CK_SPI_Init(GYRO2_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if EXT_SPI_
	if(!CK_SPI_CheckInitialized(EXT_SPI)){
		clock_rate = CK_SPI_GetClockRate(EXT_SPI, EXT_SPI_CLOCK);
		CK_SPI_Init(EXT_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if ACC1_SPI_
	if(!CK_SPI_CheckInitialized(ACC1_SPI)){
		clock_rate = CK_SPI_GetClockRate(ACC1_SPI, ACC1_SPI_CLOCK);
		CK_SPI_Init(ACC1_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if ACC2_SPI_
	if(!CK_SPI_CheckInitialized(ACC2_SPI)){
		clock_rate = CK_SPI_GetClockRate(ACC2_SPI, ACC2_SPI_CLOCK);
		CK_SPI_Init(ACC2_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if MAG_SPI_
	if(!CK_SPI_CheckInitialized(MAG_SPI)){
		clock_rate = CK_SPI_GetClockRate(MAG_SPI, MAG_SPI_CLOCK);
		CK_SPI_Init(MAG_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if BARO_SPI_
	if(!CK_SPI_CheckInitialized(BARO_SPI)){
		clock_rate = CK_SPI_GetClockRate(BARO_SPI, BARO_SPI_CLOCK);
		CK_SPI_Init(BARO_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if LOG_FLASH_
	if(!CK_SPI_CheckInitialized(FLASH_SPI)){
		clock_rate = CK_SPI_GetClockRate(FLASH_SPI, FLASH_SPI_CLOCK);
		CK_SPI_Init(FLASH_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}
#endif

#if LOG_SPI_
    if(!CK_SPI_CheckInitialized(MICROCARD_SPI)){
		// MicroCard needs slow clock at initialization
		// It will be change to full speed after initialization
        CK_SPI_Init(MICROCARD_SPI, CK_SPIx_CR1_Fclk_Div128, CK_SPI_USE_BAREMETAL);
    }
#endif

	CK_GPIO_ClockEnable(ADC_LIPO_GPIO);
	CK_GPIO_ClockEnable(ADC_CURRENT_GPIO);

	CK_GPIO_Init(ADC_LIPO_GPIO, ADC_LIPO_GPIO_PIN, CK_GPIO_ANALOG, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP);
	CK_GPIO_Init(ADC_CURRENT_GPIO, ADC_CURRENT_GPIO_PIN, CK_GPIO_ANALOG, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP);

	CK_ADC_Init();

#if OSD_ONBOARD_

	if(!CK_SPI_CheckInitialized(OSD_SPI)){
		clock_rate = CK_SPI_GetClockRate(OSD_SPI, OSD_SPI_CLOCK);
		CK_SPI_Init(OSD_SPI, clock_rate, CK_SPI_USE_BAREMETAL);
	}

#endif

#if ACC_I2C_
	if(!CK_I2C_CheckInitialized(ACC_I2C)){
		CK_I2C_Init(ACC_I2C, CK_I2C_400Khz, USE_HAL_I2C);
	}
#endif

#if MAG_I2C_
	if(!CK_I2C_CheckInitialized(MAG_I2C)){
		CK_I2C_Init(MAG_I2C, CK_I2C_400Khz, USE_HAL_I2C);
	}
#endif

#if BNO055_
	if(!CK_I2C_CheckInitialized(BNO055_I2C)){
		CK_I2C_Init(BNO055_I2C, CK_I2C_400Khz, USE_HAL_I2C);
	}
#endif

	// Motor GPIO and clock init
	// DSHOT or PWM both uses
	CK_SYSTEM_TIMER_ClockEnable(MOTOR1_TIM);
	CK_SYSTEM_TIMER_ClockEnable(MOTOR2_TIM);
	CK_SYSTEM_TIMER_ClockEnable(MOTOR3_TIM);
	CK_SYSTEM_TIMER_ClockEnable(MOTOR4_TIM);

	CK_GPIO_ClockEnable(MOTOR1_GPIO);
	CK_GPIO_ClockEnable(MOTOR2_GPIO);
	CK_GPIO_ClockEnable(MOTOR3_GPIO);
	CK_GPIO_ClockEnable(MOTOR4_GPIO);

	CK_GPIO_Init(MOTOR1_GPIO, MOTOR1_PIN, CK_GPIO_AF_PP, MOTOR1_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_Init(MOTOR2_GPIO, MOTOR2_PIN, CK_GPIO_AF_PP, MOTOR2_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_Init(MOTOR3_GPIO, MOTOR3_PIN, CK_GPIO_AF_PP, MOTOR3_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_Init(MOTOR4_GPIO, MOTOR4_PIN, CK_GPIO_AF_PP, MOTOR4_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

#if RGB_

	CK_SYSTEM_TIMER_ClockEnable(RGB_TIM);

	CK_GPIO_ClockEnable(RGB_GPIO);

	CK_GPIO_Init(RGB_GPIO, RGB_PIN, CK_GPIO_AF_PP, RGB_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

#endif

#if BUZZER_PWM

	CK_SYSTEM_TIMER_ClockEnable(BUZZER_TIM);

	//CK_GPIO_ClockEnable(BUZZER_GPIO);

	//CK_GPIO_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, CK_GPIO_AF_PP, BUZZER_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	// TIM2 GPIO Configuration
	// PA15 (JTDI)     ------> TIM2_CH1

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#endif

#if BUZZER_DC

	CK_GPIO_ClockEnable(BUZZER_GPIO);

	CK_GPIO_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

#endif

}

void CK_PERIPHERAL_MainInterruptInit(targetFreq_e target_period){

	CK_SYSTEM_TIMER_ClockEnable(MAIN_INTERRUPT_TIM);

	htim_main_interrupt.Instance 				= MAIN_INTERRUPT_TIM;
	htim_main_interrupt.Init.Prescaler 			= 0;
	htim_main_interrupt.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim_main_interrupt.Init.Period 			= (CK_SYSTEM_GetTIMERClock(MAIN_INTERRUPT_TIM) / (1000000 / target_period)) - 1;
	htim_main_interrupt.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;
	htim_main_interrupt.Init.RepetitionCounter 	= 0;
	htim_main_interrupt.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim_main_interrupt);

	HAL_TIM_PWM_Init(&htim_main_interrupt);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 			= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 			= 0;
	sConfigOC.OCPolarity 		= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity 		= TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode 		= TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState 		= TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState 		= TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim_main_interrupt, &sConfigOC, MAIN_INTERRUPT_TIM_CH);

	HAL_NVIC_SetPriority(RGB_DMA_IRQn, PERIPHERAL_PreemptPriority, PERIPHERAL_SubPriority);
	HAL_NVIC_EnableIRQ(MAIN_INTERRUPT_IRQn);
}

void CK_PERIPHERAL_StartInterrupt(void){
	HAL_TIM_PWM_Start_IT(&htim_main_interrupt, MAIN_INTERRUPT_TIM_CH);
}






















