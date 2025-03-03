
#include "DRIVERS/CK_RGB.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_SYSTEM.h"

#define LED_NUM			4
#define LED_BITS		24
#define RGB_BUFFER_SIZE	(LED_BITS * LED_NUM) + 4
#define RGB_DCACHE_SIZE	256

#define LED_TIME		200 // 200 millisec

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

TIM_HandleTypeDef htim_rgb;
DMA_HandleTypeDef hdma_rgb;

uint32_t dma_buffer_rgb[RGB_DCACHE_SIZE];

RGB_COLOR_ rgb_color_;

TIM_TypeDef* CK_RGB_TIM;

DMA_TypeDef* CK_RGB_DMA;

DMA_Stream_TypeDef* CK_RGB_DMA_STREAM;

void CK_RGB_Init(TIM_TypeDef* rgb_tim_, DMA_TypeDef* rgb_dma_, DMA_Stream_TypeDef* rgb_dma_stream_ ,RGB_COLOR_ color){

	CK_RGB_TIM = rgb_tim_;

	CK_RGB_DMA = rgb_dma_;

	CK_RGB_DMA_STREAM = rgb_dma_stream_;

	rgb_color_ = color;

	// GPIO and clocks are initialized at CK_PERIPHERAL.c

	htim_rgb.Instance 			    = RGB_TIM;
	htim_rgb.Init.Prescaler 	   	= (CK_SYSTEM_GetTIMERClock(RGB_TIM) / 16000000) - 1;
	htim_rgb.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	//htim_rgb.Init.Period 		 	= 19;
	htim_rgb.Init.Period 			= RGB_BUFFER_SIZE - 1;
	htim_rgb.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	htim_rgb.Init.AutoReloadPreload	= TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim_rgb);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0; // duty cycle
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState 	= TIM_OUTPUTNSTATE_ENABLE;
	sConfigOC.OCNPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim_rgb, &sConfigOC, RGB_TIM_CH);

	CK_RGB_TIM->BDTR |= (1u << 15); // Main output enable is required to enable tim1 and tim8

    // TIM DMA Init
	__HAL_RCC_DMA1_CLK_ENABLE();

	hdma_rgb.Instance 					= RGB_DMA_Stream;
	hdma_rgb.Init.Request 				= DMA_REQUEST_TIM1_CH1;
	hdma_rgb.Init.Direction 			= DMA_MEMORY_TO_PERIPH;
	hdma_rgb.Init.PeriphInc 			= DMA_PINC_DISABLE;
	hdma_rgb.Init.MemInc 				= DMA_MINC_ENABLE;
	hdma_rgb.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_WORD; //was half
	hdma_rgb.Init.MemDataAlignment 		= DMA_PDATAALIGN_WORD; //was half
	hdma_rgb.Init.Mode 					= DMA_NORMAL;
	hdma_rgb.Init.Priority 				= DMA_PRIORITY_HIGH;
	hdma_rgb.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	hdma_rgb.Init.FIFOThreshold 		= DMA_FIFO_THRESHOLD_FULL;
	hdma_rgb.Init.MemBurst 				= DMA_MBURST_SINGLE;
	hdma_rgb.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_rgb);

    __HAL_LINKDMA(&htim_rgb, hdma[RGB_DMA_ID], hdma_rgb);

	HAL_NVIC_SetPriority(RGB_DMA_IRQn, RGB_PreemptPriority, RGB_SubPriority);
	HAL_NVIC_EnableIRQ(RGB_DMA_IRQn);

	#if (USE_H7 == 1)

	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)dma_buffer_rgb) & ~(uint32_t)0x1F), RGB_DCACHE_SIZE+32);

	HAL_TIM_PWM_Start_DMA(&htim_rgb, RGB_TIM_CH, dma_buffer_rgb, RGB_BUFFER_SIZE);

	#endif

	CK_RGB_Enable();

	// Each led takes 24 bit of data, therefore to set 4 led send 24*4 + 0 bits = 100
	// New data can be send after 50 microsecond to set new color to led.

	// Colors close to white draws more current
	// 4 white color 140 mA
	// 4 blue color 40 mA
	// 4 red color 40 mA
	// 4 cyan color 80 mA
	// 4 green 48 mA

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, COLOR_WHITE);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

	CK_TIME_DelayMilliSec(LED_TIME);

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, COLOR_BLUE);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

	CK_TIME_DelayMilliSec(LED_TIME);

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, COLOR_CYAN);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

	CK_TIME_DelayMilliSec(LED_TIME);

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, COLOR_GREEN);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

	CK_TIME_DelayMilliSec(LED_TIME);

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, COLOR_RED);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

	CK_TIME_DelayMilliSec(LED_TIME);

	for(int i = 0; i < LED_NUM; i++){

		CK_RGB_LoadDmaBuffer(dma_buffer_rgb, i+1, (uint32_t)rgb_color_);

		CK_RGB_Start(RGB_BUFFER_SIZE);

	}

}

void CK_RGB_LoadDmaBuffer(uint32_t *dmaBuffer, int led , uint32_t packet){

	for (int i = 0; i < LED_BITS; i++){
		uint32_t c = (packet & 0x800000) ? 14 : 7;  // MSB first
		dmaBuffer[i + (LED_BITS * (led-1))] = c;
		packet <<= 1;
	}


}

void CK_RGB_Enable(void){

	CK_RGB_TIM->CR1 |= 1u << 0; 					// Counter enable
	CK_RGB_TIM->CCER |= (1u << TIMER_CH_EN_CH1);	// CH Enable

}



void CK_RGB_Start(int num){

	#if (USE_H7 == 1)
	// H7 has cache which makes it 3,4 times faster

	// Clean before tx operation when dcache is enabled
	SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)dma_buffer_rgb) & ~(uint32_t)0x1F), RGB_DCACHE_SIZE+32);

	#endif

	// DMA Stream Clear Interrupts
	CK_RGB_DMA->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM4_HIFCR_OFFSET;

	// TIM Clear Interrupts
	CK_RGB_TIM->SR = 0;

	CK_RGB_DMA_STREAM->NDTR = num;

	CK_RGB_DMA_STREAM->CR |= (1u << 0); // DMA Stream start

	CK_RGB_TIM->DIER |= 1u << TIMER_DMA_REQ_CH1; // dma req ch enable


}

#if RGB_
void RGB_DMA_Handler(void){

	static uint32_t status = 0;
	status = CK_RGB_DMA->HISR;

	CK_RGB_DMA->HIFCR = DMA_CLEAR_FLAG_NUM << DMA_CLEAR_FLAG_STREAM4_HIFCR_OFFSET;

	// Transfer Complete
	if( ( ( status & (1u << DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET) ) >> DMA_CLEAR_FLAG_TCIF4_HISR_OFFSET ) == 1){

		CK_RGB_DMA_STREAM->CR &= ~(1u<<0); // DMA Stream stop

		CK_RGB_TIM->DIER &= ~(1u << TIMER_DMA_REQ_CH1); // dma req ch disable

	}

}
#endif
