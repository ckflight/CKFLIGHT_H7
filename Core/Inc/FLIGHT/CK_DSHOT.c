
#include "FLIGHT/CK_DSHOT.h"

#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SYSTEM.h"

TIM_HandleTypeDef htim_dshot1;
TIM_HandleTypeDef htim_dshot2;
TIM_HandleTypeDef htim_dshot3;
TIM_HandleTypeDef htim_dshot4;

TIM_HandleTypeDef htim_interrupt;

DMA_HandleTypeDef hdma_dshot1;
DMA_HandleTypeDef hdma_dshot2;
DMA_HandleTypeDef hdma_dshot3;
DMA_HandleTypeDef hdma_dshot4;

#define DSHOT_BUFFER_SIZE		20
#define DSHOT_DCACHE_SIZE		128

#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)

#define TIMER_DMA_REQ_CH1		9
#define TIMER_DMA_REQ_CH2		10
#define TIMER_DMA_REQ_CH3		11
#define TIMER_DMA_REQ_CH4		12

#define TIMER_CH_EN_CH1			0
#define TIMER_CH_EN_CH2			4
#define TIMER_CH_EN_CH3			8
#define TIMER_CH_EN_CH4			12

#define DMA_CLEAR_FLAG_NUM		0x3D

#define DMA_CLEAR_FLAG_STREAM0_LIFCR_OFFSET		0
#define DMA_CLEAR_FLAG_STREAM1_LIFCR_OFFSET		6
#define DMA_CLEAR_FLAG_STREAM2_LIFCR_OFFSET		16
#define DMA_CLEAR_FLAG_STREAM3_LIFCR_OFFSET		22

#define DMA_CLEAR_FLAG_STREAM4_HIFCR_OFFSET		0
#define DMA_CLEAR_FLAG_STREAM5_HIFCR_OFFSET		6
#define DMA_CLEAR_FLAG_STREAM6_HIFCR_OFFSET		16
#define DMA_CLEAR_FLAG_STREAM7_HIFCR_OFFSET		22

#define DMA_CLEAR_FLAG_TCIF0_LISR_OFFSET		5
#define DMA_CLEAR_FLAG_TCIF1_LISR_OFFSET		11
#define DMA_CLEAR_FLAG_TCIF2_LISR_OFFSET		21
#define DMA_CLEAR_FLAG_TCIF3_LISR_OFFSET		27

#define DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET		5
#define DMA_CLEAR_FLAG_TCIF5_HISR_OFFSET		11
#define DMA_CLEAR_FLAG_TCIF6_HISR_OFFSET		21
#define DMA_CLEAR_FLAG_TCIF7_HISR_OFFSET		27

uint32_t dma_buffer_dshot1[DSHOT_DCACHE_SIZE];
uint32_t dma_buffer_dshot2[DSHOT_DCACHE_SIZE];
uint32_t dma_buffer_dshot3[DSHOT_DCACHE_SIZE];
uint32_t dma_buffer_dshot4[DSHOT_DCACHE_SIZE];

uint8_t motor1_clear_flag;
uint8_t motor2_clear_flag;
uint8_t motor3_clear_flag;
uint8_t motor4_clear_flag;

uint32_t motor1_tc_flag;
uint32_t motor2_tc_flag;
uint32_t motor3_tc_flag;
uint32_t motor4_tc_flag;

uint32_t motor1_timer_req_flag;
uint32_t motor2_timer_req_flag;
uint32_t motor3_timer_req_flag;
uint32_t motor4_timer_req_flag;

DSHOT_Mode_t dshot_mode_;

uint16_t digitalIdleOffsetValue = 550;

void CK_DSHOT_Init(DSHOT_Mode_t mode, targetFreq_e target_period){

	dshot_mode_ = mode;

	CK_DSHOT_Init1();
	CK_DSHOT_DMA_SetInterruptFlag(DSHOT1_DMA, DSHOT1_DMA_Stream, 1);
	CK_DSHOT_DMA_SetTCFlag(DSHOT1_DMA, DSHOT1_DMA_Stream, 1);
	CK_DSHOT_SetTimerRequest(MOTOR1_TIM_CH, 1);

	CK_DSHOT_Init2();
	CK_DSHOT_DMA_SetInterruptFlag(DSHOT2_DMA, DSHOT2_DMA_Stream, 2);
	CK_DSHOT_DMA_SetTCFlag(DSHOT2_DMA, DSHOT2_DMA_Stream, 2);
	CK_DSHOT_SetTimerRequest(MOTOR2_TIM_CH, 2);

	CK_DSHOT_Init3();
	CK_DSHOT_DMA_SetInterruptFlag(DSHOT3_DMA, DSHOT3_DMA_Stream, 3);
	CK_DSHOT_DMA_SetTCFlag(DSHOT3_DMA, DSHOT3_DMA_Stream, 3);
	CK_DSHOT_SetTimerRequest(MOTOR3_TIM_CH, 3);

	CK_DSHOT_Init4();
	CK_DSHOT_DMA_SetInterruptFlag(DSHOT4_DMA, DSHOT4_DMA_Stream, 4);
	CK_DSHOT_DMA_SetTCFlag(DSHOT4_DMA, DSHOT4_DMA_Stream, 4);
	CK_DSHOT_SetTimerRequest(MOTOR4_TIM_CH, 4);

	CK_DSHOT_Enable();

	// Send 0 for esc to start up
	// Dshot requires update, so until arming esc should receive 0 command

	uint32_t time_ = 4000000; // 4 sec
	time_ /= target_period;
	for(int i = 0; i < time_; i++){

		uint16_t packet = CK_DSHOT_PrepareDshotPacket((DSHOT_COMMAND_MOTOR_STOP & 0x7FF) , false);
		CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot1, 1, packet);
		CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot2, 1, packet);
		CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot3, 1, packet);
		CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot4, 1, packet);

		CK_DSHOT_Start1(DSHOT_BUFFER_SIZE);
		CK_DSHOT_Start2(DSHOT_BUFFER_SIZE);
		CK_DSHOT_Start3(DSHOT_BUFFER_SIZE);
		CK_DSHOT_Start4(DSHOT_BUFFER_SIZE);

		CK_TIME_DelayMicroSec(target_period);
	}

	if(mode == INLINE_MODE){

		CK_DSHOT_InitInterrupt(target_period);

	}

}

void CK_DSHOT_InitEndPoints(float outputLimit, float* outputLow, float* outputHigh, float* disarmMotorOutput){

	float outputLimitOffset = DSHOT_RANGE * (1 - outputLimit);
	const float motorIdlePercent = CONVERT_PARAMETER_TO_PERCENT(digitalIdleOffsetValue * 0.01f);

	*disarmMotorOutput = DSHOT_COMMAND_MOTOR_STOP;

	*outputLow = DSHOT_MIN_THROTTLE + motorIdlePercent * DSHOT_RANGE;
	*outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;

}

void CK_DSHOT_SetMotor1(int num){

	uint16_t packet = CK_DSHOT_PrepareDshotPacket((num & 0x7FF) , false);
	CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot1, 1, packet);

	if(dshot_mode_ == BLOCKING_MODE){
		CK_DSHOT_Start1(DSHOT_BUFFER_SIZE);
	}
}

void CK_DSHOT_SetMotor2(int num){

	uint16_t packet = CK_DSHOT_PrepareDshotPacket((num & 0x7FF) , false);
	CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot2, 1, packet);

	if(dshot_mode_ == BLOCKING_MODE){
		CK_DSHOT_Start2(DSHOT_BUFFER_SIZE);
	}
}

void CK_DSHOT_SetMotor3(int num){

	uint16_t packet = CK_DSHOT_PrepareDshotPacket((num & 0x7FF) , false);
	CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot3, 1, packet);

	if(dshot_mode_ == BLOCKING_MODE){
		CK_DSHOT_Start3(DSHOT_BUFFER_SIZE);
	}
}

void CK_DSHOT_SetMotor4(int num){

	uint16_t packet = CK_DSHOT_PrepareDshotPacket((num & 0x7FF) , false);
	CK_DSHOT_LoadDmaBufferDshot(dma_buffer_dshot4, 1, packet);

	if(dshot_mode_ == BLOCKING_MODE){
		CK_DSHOT_Start4(DSHOT_BUFFER_SIZE);
	}
}

uint16_t CK_DSHOT_PrepareDshotPacket(uint16_t value, bool requestTelemetry){

	uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
	requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row

	// compute checksum
	int csum = 0;
	int csum_data = packet;
	for (int i = 0; i < 3; i++) {
	csum ^=  csum_data;   // xor data by nibbles
	csum_data >>= 4;
	}
	csum &= 0xf;
	// append checksum
	packet = (packet << 4) | csum;

	return packet;

}

void CK_DSHOT_LoadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet){

	for (int i = 0; i < 16; i++){
		dmaBuffer[i * stride] = (packet & 0x8000) ? 14 : 7;  // MSB first
		packet <<= 1;
	}

}

uint32_t CK_DSHOT_GetChannel(uint32_t c){

	uint32_t resp;

	if(c == TIM_CHANNEL_1){
		resp = (1u << TIMER_CH_EN_CH1);
	}
	else if(c == TIM_CHANNEL_2){
		resp = (1u << TIMER_CH_EN_CH2);
	}
	else if(c == TIM_CHANNEL_3){
		resp = (1u << TIMER_CH_EN_CH3);
	}
	else if(c == TIM_CHANNEL_4){
		resp = (1u << TIMER_CH_EN_CH4);
	}

	return resp;

}

void CK_DSHOT_Enable(void){

    // Start PWM
	MOTOR1_TIM->CR1 |= 1u << 0; 					// Counter enable
	MOTOR1_TIM->CCER |= CK_DSHOT_GetChannel(MOTOR1_TIM_CH);

	MOTOR2_TIM->CR1 |= 1u << 0; 					// Counter enable
	MOTOR2_TIM->CCER |= CK_DSHOT_GetChannel(MOTOR2_TIM_CH);

	MOTOR3_TIM->CR1 |= 1u << 0; 					// Counter enable
	MOTOR3_TIM->CCER |= CK_DSHOT_GetChannel(MOTOR3_TIM_CH);

	MOTOR4_TIM->CR1 |= 1u << 0; 					// Counter enable
	MOTOR4_TIM->CCER |= CK_DSHOT_GetChannel(MOTOR4_TIM_CH);

}

void CK_DSHOT_SetTimerRequest(uint32_t c, int motor_num){

	uint32_t resp;

	if(c == TIM_CHANNEL_1){
		resp = 1u << TIMER_DMA_REQ_CH1; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_2){
		resp = 1u << TIMER_DMA_REQ_CH2; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_3){
		resp = 1u << TIMER_DMA_REQ_CH3; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_4){
		resp = 1u << TIMER_DMA_REQ_CH4; // DMA req CH enable
	}

	if(motor_num == 1){
		motor1_timer_req_flag = resp;
	}
	else if(motor_num == 2){
		motor2_timer_req_flag = resp;
	}
	else if(motor_num == 3){
		motor3_timer_req_flag = resp;
	}
	else if(motor_num == 4){
		motor4_timer_req_flag = resp;
	}

}

uint32_t CK_DSHOT_GetTimerRequest(uint32_t c){

	uint32_t resp;

	if(c == TIM_CHANNEL_1){
		resp = 1u << TIMER_DMA_REQ_CH1; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_2){
		resp = 1u << TIMER_DMA_REQ_CH2; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_3){
		resp = 1u << TIMER_DMA_REQ_CH3; // DMA req CH enable
	}
	else if(c == TIM_CHANNEL_4){
		resp = 1u << TIMER_DMA_REQ_CH4; // DMA req CH enable
	}

	return resp;

}

void CK_DSHOT_Start1(int num){

	#if USE_H7 == 1
	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot1, DSHOT_DCACHE_SIZE);

	#endif

	// DMA Stream Clear Interrupts
	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT1(DSHOT1_DMA, DSHOT1_DMA_Stream);

	// TIM Clear Interrupts
	MOTOR1_TIM->SR = 0;

	DSHOT1_DMA_Stream->NDTR = num;
	DSHOT1_DMA_Stream->CR |= (1u<<0); // DMA Stream start

	MOTOR1_TIM->DIER |= motor1_timer_req_flag;

}

void CK_DSHOT_Start2(int num){

	#if USE_H7 == 1
	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot2, DSHOT_DCACHE_SIZE);

	#endif

	// DMA Stream Clear Interrupts
	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT2(DSHOT2_DMA, DSHOT2_DMA_Stream);

	// TIM Clear Interrupts
	MOTOR2_TIM->SR = 0;

	DSHOT2_DMA_Stream->NDTR = num;
	DSHOT2_DMA_Stream->CR |= (1u<<0); // DMA Stream start

	MOTOR2_TIM->DIER |= motor2_timer_req_flag;

}

void CK_DSHOT_Start3(int num){

	#if USE_H7 == 1
	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot3, DSHOT_DCACHE_SIZE);

	#endif

	// DMA Stream Clear Interrupts
	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT3(DSHOT3_DMA, DSHOT3_DMA_Stream);

	// TIM Clear Interrupts
	MOTOR3_TIM->SR = 0;

	DSHOT3_DMA_Stream->NDTR = num;
	DSHOT3_DMA_Stream->CR |= (1u<<0); // DMA Stream start

	MOTOR3_TIM->DIER |= motor3_timer_req_flag;

}

void CK_DSHOT_Start4(int num){

	#if USE_H7 == 1
	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot4, DSHOT_DCACHE_SIZE);

	#endif

	// DMA Stream Clear Interrupts
	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT4(DSHOT4_DMA, DSHOT4_DMA_Stream);

	// TIM Clear Interrupts
	MOTOR4_TIM->SR = 0;

	DSHOT4_DMA_Stream->NDTR = num;
	DSHOT4_DMA_Stream->CR |= (1u<<0); // DMA Stream start

	MOTOR4_TIM->DIER |= motor4_timer_req_flag;

}

void CK_DSHOT_Init1(void){

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	// Dshot600 is 600.000 bits/sec. Period 20 * 600KHz = 12MHz.
	// 240MHz clock divided by 12MHz gives 600.000KHz pwm output.

	htim_dshot1.Instance 			    = MOTOR1_TIM;
	htim_dshot1.Init.Prescaler 	   	    = (CK_SYSTEM_GetTIMERClock(MOTOR1_TIM) / 12000000) - 1;
	htim_dshot1.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim_dshot1.Init.Period 			= DSHOT_BUFFER_SIZE - 1;
	htim_dshot1.Init.ClockDivision 	 	= TIM_CLOCKDIVISION_DIV1;
	htim_dshot1.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim_dshot1);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0; // duty cycle
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState 	= TIM_OUTPUTNSTATE_ENABLE;
	sConfigOC.OCNPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim_dshot1, &sConfigOC, MOTOR1_TIM_CH);

#if USE_F4 == 1
	// TIM DMA Init
	MOTOR1_TIM->CR2 |= 1u<<3; // DMA Request send when update event
	MOTOR1_TIM->CR1 |= 1u<<7; // Auto-reload preload enable
	MOTOR1_TIM->EGR |= 1u<<0; // Update generation enabled

	// DMA controller clock enable
	__HAL_RCC_DMA1_CLK_ENABLE();

	// Clear flags before enabling DMA
	CK_DSHOT_DMA_ClearInterruptFlags(DSHOT1_DMA, DSHOT1_DMA_Stream);

	// Set peripheral data register address
	if(MOTOR1_TIM_CH == TIM_CHANNEL_1){
		DSHOT1_DMA_Stream->PAR = (uint32_t)(&MOTOR1_TIM->CCR1);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_2){
		DSHOT1_DMA_Stream->PAR = (uint32_t)(&MOTOR1_TIM->CCR2);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_3){
		DSHOT1_DMA_Stream->PAR = (uint32_t)(&MOTOR1_TIM->CCR3);
	}
	else if(MOTOR1_TIM_CH == TIM_CHANNEL_4){
		DSHOT1_DMA_Stream->PAR = (uint32_t)(&MOTOR1_TIM->CCR4);
	}

	DSHOT1_DMA_Stream->M0AR = (uint32_t)(&dma_buffer_dshot1);

	// Enabling DMA again will load the same value
	DSHOT1_DMA_Stream->NDTR = DSHOT_BUFFER_SIZE;

	// Half and Full Transfer Complete
	DSHOT1_DMA_Stream->CR |= (1u << 4);

	// DMA is flow controller
	DSHOT1_DMA_Stream->CR |= (0u << 5);

	// Data transfer direction is Peripheral to Memory, Memory address increment after each data tx
	DSHOT1_DMA_Stream->CR |= (1u << 6) | (1 << 10);

	// Peripheral size 32 bit, Memory size 32 bit, Priority very high, channel 5 is selected
	DSHOT1_DMA_Stream->CR |= (2u << 11) | (2u << 13) | (3u << 16) | (DSHOT1_DMA_Stream_Ch << 25);

	// Enable interrupt
	HAL_NVIC_SetPriority(DSHOT1_DMA_IRQn, DSHOT_M1_PreemptPriority, DSHOT_M1_SubPriority);
	HAL_NVIC_EnableIRQ(DSHOT1_DMA_IRQn);

#endif

#if USE_H7 == 1

    // TIM DMA Init
	__HAL_RCC_DMA1_CLK_ENABLE();

	hdma_dshot1.Instance 					= DSHOT1_DMA_Stream;
	hdma_dshot1.Init.Request 				= DSHOT1_DMA_Request;
	hdma_dshot1.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
	hdma_dshot1.Init.PeriphInc 				= DMA_PINC_DISABLE;
	hdma_dshot1.Init.MemInc 				= DMA_MINC_ENABLE;
	hdma_dshot1.Init.PeriphDataAlignment	= DMA_PDATAALIGN_WORD; //was half
	hdma_dshot1.Init.MemDataAlignment 		= DMA_PDATAALIGN_WORD; //was half
	hdma_dshot1.Init.Mode 					= DMA_NORMAL;
	hdma_dshot1.Init.Priority 				= DMA_PRIORITY_HIGH;
	hdma_dshot1.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	hdma_dshot1.Init.FIFOThreshold 			= DMA_FIFO_THRESHOLD_FULL;
	hdma_dshot1.Init.MemBurst 				= DMA_MBURST_SINGLE;
	hdma_dshot1.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_dshot1);

    __HAL_LINKDMA(&htim_dshot1, hdma[DSHOT1_DMA_ID], hdma_dshot1);

    HAL_NVIC_SetPriority(DSHOT1_DMA_IRQn, DSHOT_M1_PreemptPriority, DSHOT_M1_SubPriority);
	HAL_NVIC_EnableIRQ(DSHOT1_DMA_IRQn);

	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot1, DSHOT_DCACHE_SIZE);

	HAL_TIM_PWM_Start_DMA(&htim_dshot1, MOTOR1_TIM_CH, dma_buffer_dshot1, DSHOT_BUFFER_SIZE);

#endif

}

void CK_DSHOT_Init2(void){

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	// Dshot600 is 600.000 bits/sec. Period 20 * 600KHz = 12MHz.
	// 240MHz clock divided by 12MHz gives 600.000KHz pwm output.

	htim_dshot2.Instance 			    = MOTOR2_TIM;
	htim_dshot2.Init.Prescaler 	   	    = (CK_SYSTEM_GetTIMERClock(MOTOR2_TIM) / 12000000) - 1;
	htim_dshot2.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim_dshot2.Init.Period 			= DSHOT_BUFFER_SIZE - 1;
	htim_dshot2.Init.ClockDivision 	 	= TIM_CLOCKDIVISION_DIV1;
	htim_dshot2.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim_dshot2);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0; // duty cycle
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState 	= TIM_OUTPUTNSTATE_ENABLE;
	sConfigOC.OCNPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim_dshot2, &sConfigOC, MOTOR2_TIM_CH);

#if USE_F4 == 1

    // TIM DMA Init
	MOTOR2_TIM->CR2 |= 1u<<3; // DMA Request send when update event
	MOTOR2_TIM->CR1 |= 1u<<7; // Auto-reload preload enable
	MOTOR2_TIM->EGR |= 1u<<0; // Update generation enabled

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	// Clear flags before enabling DMA
	CK_DSHOT_DMA_ClearInterruptFlags(DSHOT2_DMA, DSHOT2_DMA_Stream);

	// Set peripheral data register address
	if(MOTOR2_TIM_CH == TIM_CHANNEL_1){
		DSHOT2_DMA_Stream->PAR = (uint32_t)(&MOTOR2_TIM->CCR1);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_2){
		DSHOT2_DMA_Stream->PAR = (uint32_t)(&MOTOR2_TIM->CCR2);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_3){
		DSHOT2_DMA_Stream->PAR = (uint32_t)(&MOTOR2_TIM->CCR3);
	}
	else if(MOTOR2_TIM_CH == TIM_CHANNEL_4){
		DSHOT2_DMA_Stream->PAR = (uint32_t)(&MOTOR2_TIM->CCR4);
	}

	DSHOT2_DMA_Stream->M0AR = (uint32_t)(&dma_buffer_dshot2);

	// Enabling DMA again will load the same value
	DSHOT2_DMA_Stream->NDTR = DSHOT_BUFFER_SIZE;

	// Full Transfer Complete
	DSHOT2_DMA_Stream->CR |= (1u << 4);

	// DMA is flow controller
	DSHOT2_DMA_Stream->CR |= (0u << 5);

	// Data transfer direction is Peripheral to Memory, Memory address increment after each data tx
	DSHOT2_DMA_Stream->CR |= (1u << 6) | (1 << 10);

	// Peripheral size 32 bit, Memory size 32 bit, Priority very high, channel 2 is selected
	DSHOT2_DMA_Stream->CR |= (2u << 11) | (2u << 13) | (3u << 16) | (DSHOT2_DMA_Stream_Ch << 25);

	// Enable interrupt
	HAL_NVIC_SetPriority(DSHOT2_DMA_IRQn, DSHOT_M2_PreemptPriority, DSHOT_M2_SubPriority);
	HAL_NVIC_EnableIRQ(DSHOT2_DMA_IRQn);

#endif

#if USE_H7 == 1

    // TIM DMA Init
	__HAL_RCC_DMA1_CLK_ENABLE();

	hdma_dshot2.Instance 					= DSHOT2_DMA_Stream;
	hdma_dshot2.Init.Request 				= DSHOT2_DMA_Request;
	hdma_dshot2.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
	hdma_dshot2.Init.PeriphInc 				= DMA_PINC_DISABLE;
	hdma_dshot2.Init.MemInc 				= DMA_MINC_ENABLE;
	hdma_dshot2.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_WORD; //was half
	hdma_dshot2.Init.MemDataAlignment 		= DMA_PDATAALIGN_WORD; //was half
	hdma_dshot2.Init.Mode 					= DMA_NORMAL;
	hdma_dshot2.Init.Priority 				= DMA_PRIORITY_HIGH;
	hdma_dshot2.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	hdma_dshot2.Init.FIFOThreshold 			= DMA_FIFO_THRESHOLD_FULL;
	hdma_dshot2.Init.MemBurst 				= DMA_MBURST_SINGLE;
	hdma_dshot2.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_dshot2);

    __HAL_LINKDMA(&htim_dshot2, hdma[DSHOT2_DMA_ID], hdma_dshot2);

    HAL_NVIC_SetPriority(DSHOT2_DMA_IRQn, DSHOT_M2_PreemptPriority, DSHOT_M2_SubPriority);
	HAL_NVIC_EnableIRQ(DSHOT2_DMA_IRQn);

	SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot2, DSHOT_DCACHE_SIZE);

	HAL_TIM_PWM_Start_DMA(&htim_dshot2, MOTOR2_TIM_CH, dma_buffer_dshot2, DSHOT_BUFFER_SIZE);

#endif

}

void CK_DSHOT_Init3(void){

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	// Dshot600 is 600.000 bits/sec. Period 20 * 600KHz = 12MHz.
	// 240MHz clock divided by 12MHz gives 600.000KHz pwm output.

	htim_dshot3.Instance 			    = MOTOR3_TIM;
	htim_dshot3.Init.Prescaler 	   	    = (CK_SYSTEM_GetTIMERClock(MOTOR3_TIM) / 12000000) - 1;
	htim_dshot3.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim_dshot3.Init.Period 			= DSHOT_BUFFER_SIZE - 1;
	htim_dshot3.Init.ClockDivision 	 	= TIM_CLOCKDIVISION_DIV1;
	htim_dshot3.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim_dshot3);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0; // duty cycle
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState 	= TIM_OUTPUTNSTATE_ENABLE;
	sConfigOC.OCNPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim_dshot3, &sConfigOC, MOTOR3_TIM_CH);

	#if USE_F4 == 1

		MOTOR3_TIM->CR2 |= 1u<<3; // DMA Request send when update event
		MOTOR3_TIM->CR1 |= 1u<<7; // Auto-reload preload enable
		MOTOR3_TIM->EGR |= 1u<<0; // Update generation enabled

		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();

		// Clear flags before enabling DMA
		CK_DSHOT_DMA_ClearInterruptFlags(DSHOT3_DMA, DSHOT3_DMA_Stream);

		// Set peripheral data register address
		if(MOTOR3_TIM_CH == TIM_CHANNEL_1){
			DSHOT3_DMA_Stream->PAR = (uint32_t)(&MOTOR3_TIM->CCR1);
		}
		else if(MOTOR3_TIM_CH == TIM_CHANNEL_2){
			DSHOT3_DMA_Stream->PAR = (uint32_t)(&MOTOR3_TIM->CCR2);
		}
		else if(MOTOR3_TIM_CH == TIM_CHANNEL_3){
			DSHOT3_DMA_Stream->PAR = (uint32_t)(&MOTOR3_TIM->CCR3);
		}
		else if(MOTOR3_TIM_CH == TIM_CHANNEL_4){
			DSHOT3_DMA_Stream->PAR = (uint32_t)(&MOTOR3_TIM->CCR4);
		}

		DSHOT3_DMA_Stream->M0AR = (uint32_t)(&dma_buffer_dshot3);

		// Enabling DMA again will load the same value
		DSHOT3_DMA_Stream->NDTR = DSHOT_BUFFER_SIZE;

		// Half and Full Transfer Complete
		DSHOT3_DMA_Stream->CR |= (1u << 4);

		// DMA is flow controller
		DSHOT3_DMA_Stream->CR |= (0u << 5);

		// Data transfer direction is Peripheral to Memory, Memory address increment after each data tx
		DSHOT3_DMA_Stream->CR |= (1u << 6) | (1 << 10);

		// Peripheral size 32 bit, Memory size 32 bit, Priority very high, channel 2 is selected
		DSHOT3_DMA_Stream->CR |= (2u << 11) | (2u << 13) | (3u << 16) | (DSHOT3_DMA_Stream_Ch << 25);

		// Enable interrupt
		HAL_NVIC_SetPriority(DSHOT3_DMA_IRQn, DSHOT_M3_PreemptPriority, DSHOT_M3_SubPriority);
		HAL_NVIC_EnableIRQ(DSHOT3_DMA_IRQn);

	#endif

	#if USE_H7 == 1

		// TIM DMA Init
		__HAL_RCC_DMA1_CLK_ENABLE();

	    hdma_dshot3.Instance 					= DSHOT3_DMA_Stream;
	    hdma_dshot3.Init.Request 				= DSHOT3_DMA_Request;
	    hdma_dshot3.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
	    hdma_dshot3.Init.PeriphInc 				= DMA_PINC_DISABLE;
	    hdma_dshot3.Init.MemInc 				= DMA_MINC_ENABLE;
	    hdma_dshot3.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_WORD; //was half
	    hdma_dshot3.Init.MemDataAlignment 		= DMA_PDATAALIGN_WORD; //was half
	    hdma_dshot3.Init.Mode 					= DMA_NORMAL;
	    hdma_dshot3.Init.Priority 				= DMA_PRIORITY_HIGH;
	    hdma_dshot3.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	    hdma_dshot3.Init.FIFOThreshold 			= DMA_FIFO_THRESHOLD_FULL;
	    hdma_dshot3.Init.MemBurst 				= DMA_MBURST_SINGLE;
	    hdma_dshot3.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
	    HAL_DMA_Init(&hdma_dshot3);

	    __HAL_LINKDMA(&htim_dshot3, hdma[DSHOT3_DMA_ID], hdma_dshot3);

		HAL_NVIC_SetPriority(DSHOT3_DMA_IRQn, DSHOT_M3_PreemptPriority, DSHOT_M3_SubPriority);
		HAL_NVIC_EnableIRQ(DSHOT3_DMA_IRQn);

		SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot3, DSHOT_DCACHE_SIZE);

		HAL_TIM_PWM_Start_DMA(&htim_dshot3, MOTOR3_TIM_CH, dma_buffer_dshot3, DSHOT_BUFFER_SIZE);

	#endif

}

void CK_DSHOT_Init4(void){

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	// Dshot600 is 600.000 bits/sec. Period 20 * 600KHz = 12MHz.
	// 240MHz clock divided by 12MHz gives 600.000KHz pwm output.

	htim_dshot4.Instance 			    = MOTOR4_TIM;
	htim_dshot4.Init.Prescaler 	   	    = (CK_SYSTEM_GetTIMERClock(MOTOR4_TIM) / 12000000) - 1;
	htim_dshot4.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim_dshot4.Init.Period 			= DSHOT_BUFFER_SIZE - 1;
	htim_dshot4.Init.ClockDivision 	 	= TIM_CLOCKDIVISION_DIV1;
	htim_dshot4.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim_dshot4);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0; // duty cycle
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState 	= TIM_OUTPUTNSTATE_ENABLE;
	sConfigOC.OCNPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim_dshot4, &sConfigOC, MOTOR4_TIM_CH);

	#if USE_F4 == 1

		// TIM DMA Init

		MOTOR4_TIM->CR2 |= 1u<<3; // DMA Request send when update event
		MOTOR4_TIM->CR1 |= 1u<<7; // Auto-reload preload enable
		MOTOR4_TIM->EGR |= 1u<<0; // Update generation enabled

		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();

		// Clear flags before enabling DMA
		CK_DSHOT_DMA_ClearInterruptFlags(DSHOT4_DMA, DSHOT4_DMA_Stream);

		// Set peripheral data register address
		if(MOTOR4_TIM_CH == TIM_CHANNEL_1){
			DSHOT4_DMA_Stream->PAR = (uint32_t)(&MOTOR4_TIM->CCR1);
		}
		else if(MOTOR4_TIM_CH == TIM_CHANNEL_2){
			DSHOT4_DMA_Stream->PAR = (uint32_t)(&MOTOR4_TIM->CCR2);
		}
		else if(MOTOR4_TIM_CH == TIM_CHANNEL_3){
			DSHOT4_DMA_Stream->PAR = (uint32_t)(&MOTOR4_TIM->CCR3);
		}
		else if(MOTOR4_TIM_CH == TIM_CHANNEL_4){
			DSHOT4_DMA_Stream->PAR = (uint32_t)(&MOTOR4_TIM->CCR4);
		}

		DSHOT4_DMA_Stream->M0AR = (uint32_t)(&dma_buffer_dshot4);

		// Enabling DMA again will load the same value
		DSHOT4_DMA_Stream->NDTR = DSHOT_BUFFER_SIZE;

		// Half and Full Transfer Complete
		DSHOT4_DMA_Stream->CR |= (1u << 4);

		// DMA is flow controller
		DSHOT4_DMA_Stream->CR |= (0u << 5);

		// Data transfer direction is Peripheral to Memory, Memory address increment after each data tx
		DSHOT4_DMA_Stream->CR |= (1u << 6) | (1 << 10);

		// Peripheral size 32 bit, Memory size 32 bit, Priority very high, channel 3 is selected
		DSHOT4_DMA_Stream->CR |= (2u << 11) | (2u << 13) | (3u << 16) | (DSHOT4_DMA_Stream_Ch << 25);

		// Enable interrupt
		HAL_NVIC_SetPriority(DSHOT4_DMA_IRQn, DSHOT_M4_PreemptPriority, DSHOT_M4_SubPriority);
		HAL_NVIC_EnableIRQ(DSHOT4_DMA_IRQn);

	#endif

	#if USE_H7 == 1

	    // TIM DMA Init
		__HAL_RCC_DMA1_CLK_ENABLE();

		hdma_dshot4.Instance 					= DSHOT4_DMA_Stream;
		hdma_dshot4.Init.Request 				= DSHOT4_DMA_Request;
		hdma_dshot4.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
		hdma_dshot4.Init.PeriphInc 				= DMA_PINC_DISABLE;
		hdma_dshot4.Init.MemInc 				= DMA_MINC_ENABLE;
		hdma_dshot4.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_WORD; //was half
		hdma_dshot4.Init.MemDataAlignment 		= DMA_PDATAALIGN_WORD; //was half
		hdma_dshot4.Init.Mode 					= DMA_NORMAL;
		hdma_dshot4.Init.Priority 				= DMA_PRIORITY_HIGH;
		hdma_dshot4.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	    hdma_dshot4.Init.FIFOThreshold 			= DMA_FIFO_THRESHOLD_FULL;
	    hdma_dshot4.Init.MemBurst 				= DMA_MBURST_SINGLE;
	    hdma_dshot4.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
	    HAL_DMA_Init(&hdma_dshot4);

	    __HAL_LINKDMA(&htim_dshot4, hdma[DSHOT4_DMA_ID], hdma_dshot4);

	    HAL_NVIC_SetPriority(DSHOT4_DMA_IRQn, DSHOT_M4_PreemptPriority, DSHOT_M4_SubPriority);
		HAL_NVIC_EnableIRQ(DSHOT4_DMA_IRQn);

		SCB_CleanDCache_by_Addr((uint32_t*)dma_buffer_dshot4, DSHOT_DCACHE_SIZE);

		HAL_TIM_PWM_Start_DMA(&htim_dshot4, MOTOR4_TIM_CH, dma_buffer_dshot4, DSHOT_BUFFER_SIZE);

	#endif

}

void CK_DSHOT_InitInterrupt(targetFreq_e target_period){

	CK_SYSTEM_TIMER_ClockEnable(DSHOT_INTERRUPT_TIM);

    uint32_t frequency = (1000000 / target_period);

	#if USE_H7 == 1
    if(frequency <= 2000){
		DSHOT_INTERRUPT_TIM->PSC = 1; 				// Main clock prescalar
	}
	else if(frequency > 2000){
		DSHOT_INTERRUPT_TIM->PSC = 0; 				// Main clock prescalar
	}
	#endif
	#if USE_F4 == 1
    frequency /= 2;
    DSHOT_INTERRUPT_TIM->PSC = 0; 				// Main clock prescalar
	#endif
    DSHOT_INTERRUPT_TIM->ARR = (CK_SYSTEM_GetTIMERClock(DSHOT_INTERRUPT_TIM) / frequency ) - 1; // Freq. is fix for each ch on same timer

    if(DSHOT_INTERRUPT_TIM == TIM6 || DSHOT_INTERRUPT_TIM == TIM7){
    	// Basic timer does not have channel
    }
    else{
        if(DSHOT_INTERRUPT_TIM_CH == TIM_CHANNEL_1){
        	DSHOT_INTERRUPT_TIM->CCR1 = (DSHOT_INTERRUPT_TIM->ARR * 50) / 100 ; // %50
            DSHOT_INTERRUPT_TIM->CCMR1 |= (6u << 4) | (1u << 3); 				  // CH1 PWM Mode 1 upcounting, CH1 Preload Enable,
            DSHOT_INTERRUPT_TIM->CCER  |= (1u << 0);             				  // CH1 Enable
    	}
    	else if(DSHOT_INTERRUPT_TIM_CH == TIM_CHANNEL_2){
    		DSHOT_INTERRUPT_TIM->CCR2 = (DSHOT_INTERRUPT_TIM->ARR * 50) / 100 ; // %50
    		DSHOT_INTERRUPT_TIM->CCMR1 |= (6u << 12) | (1u << 11); 				  // CH2 PWM Mode 1 upcounting, CH2 Preload Enable,
    		DSHOT_INTERRUPT_TIM->CCER  |= (1u << 4);             				  // CH2 Enable
    	}
    	else if(DSHOT_INTERRUPT_TIM_CH == TIM_CHANNEL_3){
    		DSHOT_INTERRUPT_TIM->CCR3 = (DSHOT_INTERRUPT_TIM->ARR * 50) / 100 ; // %50
    		DSHOT_INTERRUPT_TIM->CCMR2 |= (6u << 4) | (1u << 3); 				  // CH3 PWM Mode 1 upcounting, CH3 Preload Enable,
    		DSHOT_INTERRUPT_TIM->CCER  |= (1u << 8);             				  // CH3 Enable
    	}
    	else if(DSHOT_INTERRUPT_TIM_CH == TIM_CHANNEL_4){
    		DSHOT_INTERRUPT_TIM->CCR4 = (DSHOT_INTERRUPT_TIM->ARR * 50) / 100 ; // %50
    		DSHOT_INTERRUPT_TIM->CCMR2 |= (6u << 12) | (1u << 11); 				  // CH4 PWM Mode 1 upcounting, CH4 Preload Enable,
    		DSHOT_INTERRUPT_TIM->CCER  |= (1u << 12);             				  // CH4 Enable
    	}

    }

    // CR1 default Edge Alligned, Upcounting
    DSHOT_INTERRUPT_TIM->CR1 |= (1u << 7); 	// Auto-reload preload enable
    DSHOT_INTERRUPT_TIM->CR1 |= (1u << 2); 	// Update request source

    DSHOT_INTERRUPT_TIM->DIER |= 1u << 0; 	// Update interrupt enable
    DSHOT_INTERRUPT_TIM->EGR |= 1u << 0; 	// Update generation enabled

    NVIC_EnableIRQ(DSHOT_INTERRUPT_IRQn);

    DSHOT_INTERRUPT_TIM->CR1 |= 1u << 0; // Counter enable

}

void CK_DSHOT_DMA_SetInterruptFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream, int motor_num){

	uint8_t num = 0;
	if(dma_stream == DMA1_Stream0){
		num = DMA_CLEAR_FLAG_STREAM0_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream1){
		num = DMA_CLEAR_FLAG_STREAM1_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream2){
		num = DMA_CLEAR_FLAG_STREAM2_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream3){
		num = DMA_CLEAR_FLAG_STREAM3_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream4){
		num = DMA_CLEAR_FLAG_STREAM4_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream5){
		num = DMA_CLEAR_FLAG_STREAM5_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream6){
		num = DMA_CLEAR_FLAG_STREAM6_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream7){
		num = DMA_CLEAR_FLAG_STREAM7_HIFCR_OFFSET;
	}

	if(motor_num == 1){
		motor1_clear_flag = num;
	}
	else if(motor_num == 2){
		motor2_clear_flag = num;
	}
	else if(motor_num == 3){
		motor3_clear_flag = num;
	}
	else if(motor_num == 4){
		motor4_clear_flag = num;
	}
}

void CK_DSHOT_DMA_ClearInterruptFlags(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream == DMA1_Stream0){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM0_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream1){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM1_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream2){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM2_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream3){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM3_LIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream4){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM4_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream5){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM5_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream6){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM6_HIFCR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream7){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM7_HIFCR_OFFSET;
	}

}

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT1(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream < DMA1_Stream4){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << motor1_clear_flag;
	}
	else if(dma_stream >= DMA1_Stream4){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << motor1_clear_flag;
	}
}

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT2(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream < DMA1_Stream4){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << motor2_clear_flag;
	}
	else if(dma_stream >= DMA1_Stream4){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << motor2_clear_flag;
	}
}

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT3(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream < DMA1_Stream4){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << motor3_clear_flag;
	}
	else if(dma_stream >= DMA1_Stream4){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << motor3_clear_flag;
	}
}

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT4(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream < DMA1_Stream4){
		dma->LIFCR = DMA_CLEAR_FLAG_NUM << motor4_clear_flag;
	}
	else if(dma_stream >= DMA1_Stream4){
		dma->HIFCR = DMA_CLEAR_FLAG_NUM << motor4_clear_flag;
	}
}

uint32_t CK_DSHOT_DMA_GetInterruptFlags(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	uint32_t flag = 0;

	if(dma_stream <= DMA1_Stream3){
		flag = dma->LISR;
	}
	else if(dma_stream >= DMA1_Stream4){
		flag = dma->HISR;
	}
	return flag;
}

void CK_DSHOT_DMA_SetTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream, int motor_num){

	uint8_t num = 0;
	if(dma_stream == DMA1_Stream0){
		num = DMA_CLEAR_FLAG_TCIF0_LISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream1){
		num = DMA_CLEAR_FLAG_TCIF1_LISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream2){
		num = DMA_CLEAR_FLAG_TCIF2_LISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream3){
		num = DMA_CLEAR_FLAG_TCIF3_LISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream4){
		num = DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream5){
		num = DMA_CLEAR_FLAG_TCIF5_HISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream6){
		num = DMA_CLEAR_FLAG_TCIF6_HISR_OFFSET;
	}
	else if(dma_stream == DMA1_Stream7){
		num = DMA_CLEAR_FLAG_TCIF7_HISR_OFFSET;
	}

	if(motor_num == 1){
		motor1_tc_flag = num;
	}
	else if(motor_num == 2){
		motor2_tc_flag = num;
	}
	else if(motor_num == 3){
		motor3_tc_flag = num;
	}
	else if(motor_num == 4){
		motor4_tc_flag = num;
	}
}

uint8_t CK_DSHOT_DMA_CheckTransferComplete(DMA_Stream_TypeDef* dma_stream, uint32_t status){

	uint8_t flag = 0;

	if(dma_stream == DMA1_Stream0){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF0_LISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF0_LISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream1){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF1_LISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF1_LISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream2){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF2_LISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF2_LISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream3){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF3_LISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF3_LISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream4){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream5){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF5_HISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF5_HISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream6){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF6_HISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF6_HISR_OFFSET ) == 1);
	}
	else if(dma_stream == DMA1_Stream7){
		flag = ((( status & (1u << DMA_CLEAR_FLAG_TCIF7_HISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF7_HISR_OFFSET ) == 1);
	}

	return flag;
}



void DSHOT_INTERRUPT_Handler(void){

	#if SCOPE_CHECK_DSHOT_INTERRUPT == 1
		CK_GPIO_TogglePin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
	#endif

	DSHOT_INTERRUPT_TIM->SR = 0;

	// After esc is started dma buffers is loaded with 0 command data and it is
	// sent to esc until mixer starts updating dma buffers
	CK_DSHOT_Start1(DSHOT_BUFFER_SIZE);
	CK_DSHOT_Start2(DSHOT_BUFFER_SIZE);
	CK_DSHOT_Start3(DSHOT_BUFFER_SIZE);
	CK_DSHOT_Start4(DSHOT_BUFFER_SIZE);
}


void DSHOT1_DMA_Handler(void){

	static uint32_t status = 0;

	status = CK_DSHOT_DMA_GetInterruptFlags(DSHOT1_DMA, DSHOT1_DMA_Stream);

	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT1(DSHOT1_DMA, DSHOT1_DMA_Stream);

	// Transfer Complete
	//if(CK_DSHOT_DMA_CheckTransferComplete(DSHOT1_DMA_Stream, status)){
	if((( status & (1u << motor1_tc_flag) ) >> motor1_tc_flag ) == 1){

		DSHOT1_DMA_Stream->CR &= ~(1u << 0); // DMA Stream stop

		//MOTOR1_TIM->DIER &= ~(CK_DSHOT_GetTimerRequest(MOTOR1_TIM_CH)); // dma req ch disable
		MOTOR1_TIM->DIER &= ~(motor1_timer_req_flag); // dma req ch disable


	}

}

void DSHOT2_DMA_Handler(void){

	static uint32_t status = 0;

	status = CK_DSHOT_DMA_GetInterruptFlags(DSHOT2_DMA, DSHOT2_DMA_Stream);

	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT2(DSHOT2_DMA, DSHOT2_DMA_Stream);

	// Transfer Complete
	//if(CK_DSHOT_DMA_CheckTransferComplete(DSHOT2_DMA_Stream, status)){ // Takes time
	if((( status & (1u << motor2_tc_flag) ) >> motor2_tc_flag ) == 1){

		DSHOT2_DMA_Stream->CR &= ~(1u << 0); // DMA Stream stop

		//MOTOR2_TIM->DIER &= ~(CK_DSHOT_GetTimerRequest(MOTOR2_TIM_CH)); // dma req ch disable
		MOTOR2_TIM->DIER &= ~(motor2_timer_req_flag); // dma req ch disable

	}

}

void DSHOT3_DMA_Handler(void){

	static uint32_t status = 0;

	status = CK_DSHOT_DMA_GetInterruptFlags(DSHOT3_DMA, DSHOT3_DMA_Stream);

	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT3(DSHOT3_DMA, DSHOT3_DMA_Stream);

	// Transfer Complete
	//if(CK_DSHOT_DMA_CheckTransferComplete(DSHOT3_DMA_Stream, status)){ // Takes time
	if((( status & (1u << motor3_tc_flag) ) >> motor3_tc_flag ) == 1){

		DSHOT3_DMA_Stream->CR &= ~(1u << 0); // DMA Stream stop

		//MOTOR3_TIM->DIER &= ~(CK_DSHOT_GetTimerRequest(MOTOR3_TIM_CH)); // dma req ch disable
		MOTOR3_TIM->DIER &= ~(motor3_timer_req_flag); // dma req ch disable

	}

}

void DSHOT4_DMA_Handler(void){

	static uint32_t status = 0;

	status = CK_DSHOT_DMA_GetInterruptFlags(DSHOT4_DMA, DSHOT4_DMA_Stream);

	CK_DSHOT_DMA_ClearInterruptFlagsDSHOT4(DSHOT4_DMA, DSHOT4_DMA_Stream);

	// Transfer Complete
	//if(CK_DSHOT_DMA_CheckTransferComplete(DSHOT4_DMA_Stream, status)){ // Takes time
	if((( status & (1u << motor4_tc_flag) ) >> motor4_tc_flag ) == 1){

		DSHOT4_DMA_Stream->CR &= ~(1u << 0); // DMA Stream stop

		//MOTOR4_TIM->DIER &= ~(CK_DSHOT_GetTimerRequest(MOTOR4_TIM_CH)); // dma req ch disable
		MOTOR4_TIM->DIER &= ~(motor4_timer_req_flag); // dma req ch disable

	}

}
