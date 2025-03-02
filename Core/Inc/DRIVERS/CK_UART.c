
#include "CK_UART.h"
#include "CK_SYSTEM.h"

#define RCC_DMA1EN              	1u<<21 // DMA1 Clock enable
#define RCC_DMA2EN              	1u<<22 // DMA2 Clock enable

#define DMA_STREAM0_CLEAR       	0x3D<<0
#define DMA_STREAM1_CLEAR       	0x3D<<6
#define DMA_STREAM2_CLEAR       	0x3D<<16
#define DMA_STREAM3_CLEAR       	0x3D<<22

#define DMA_STREAM4_CLEAR       	0x3D<<0
#define DMA_STREAM5_CLEAR       	0x3D<<6
#define DMA_STREAM6_CLEAR       	0x3D<<16
#define DMA_STREAM7_CLEAR       	0x3D<<22

#define DMA_STREAM0_TC_CLEAR       	0x20<<0
#define DMA_STREAM1_TC_CLEAR       	0x20<<6
#define DMA_STREAM2_TC_CLEAR       	0x20<<16
#define DMA_STREAM3_TC_CLEAR       	0x20<<22

#define DMA_STREAM4_TC_CLEAR       	0x20<<0
#define DMA_STREAM5_TC_CLEAR       	0x20<<6
#define DMA_STREAM6_TC_CLEAR       	0x20<<16
#define DMA_STREAM7_TC_CLEAR       	0x20<<22

#define DMA_STREAM0_TCIF       		1u<<5
#define DMA_STREAM1_TCIF        	1u<<11
#define DMA_STREAM2_TCIF        	1u<<21
#define DMA_STREAM3_TCIF        	1u<<27
#define DMA_STREAM4_TCIF        	1u<<5
#define DMA_STREAM5_TCIF        	1u<<11
#define DMA_STREAM6_TCIF        	1u<<21
#define DMA_STREAM7_TCIF        	1u<<27

#define DMA_CR_Enable           	1u<<0

int circularBufferSize = 512;

/*
 * Default 8bit 1 start 1 stop bit, no parity
 * USART1,6 Clocked by PCLK2
 * USART2,3,4,5 Clocked by PCLK1
 *
 * UART5 PART PD2=RX(USED),PC12=TX
 * TX Not Used in design so Not Initialized Here
 */

void CK_UART_Init(USART_CONFIGURATION_* config, circularBuffer_t* circular_buf){

	int uart_clock = 0;

	if(config->use_circular_buffer){
		CK_CIRCULARBUFFER_Init(circular_buf, circularBufferSize); // Initialize Circular Buffer
	}

	uart_clock = CK_SYSTEM_GetUARTClock(config->usart);

	if(config->rx_gpio_type != config->tx_gpio_type){
		CK_GPIO_ClockEnable(config->rx_gpio_type);
		CK_GPIO_ClockEnable(config->tx_gpio_type);
	}
	else{
		CK_GPIO_ClockEnable(config->rx_gpio_type);
	}

	if(config->mode == RX_ONLY){
		CK_GPIO_Init(config->rx_gpio_type, config->rx_gpio_pin, CK_GPIO_AF_PP, config->rx_af, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP); //RX
	}
	else if(config->mode == TX_ONLY){
		CK_GPIO_Init(config->tx_gpio_type, config->tx_gpio_pin, CK_GPIO_AF_PP, config->tx_af, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP);  //TX
	}
	else{
		CK_GPIO_Init(config->rx_gpio_type, config->rx_gpio_pin, CK_GPIO_AF_PP, config->rx_af, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP); //RX
		CK_GPIO_Init(config->tx_gpio_type, config->tx_gpio_pin, CK_GPIO_AF_PP, config->tx_af, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP);  //TX
	}

	if(config->usart == USART1){

		__HAL_RCC_USART1_CLK_ENABLE();

		NVIC_EnableIRQ(USART1_IRQn);

		HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
	}
	else if(config->usart == USART2){

		__HAL_RCC_USART2_CLK_ENABLE();

		NVIC_EnableIRQ(USART2_IRQn);

		HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);

	}
	else if(config->usart == USART3){

		__HAL_RCC_USART3_CLK_ENABLE();

		NVIC_EnableIRQ(USART3_IRQn);

		HAL_NVIC_SetPriority(USART3_IRQn, 3, 3);

	}
	else if(config->usart == UART4){

		__HAL_RCC_UART4_CLK_ENABLE();

		NVIC_EnableIRQ(UART4_IRQn);

		HAL_NVIC_SetPriority(UART4_IRQn, 3, 3);

	}
	else if(config->usart == USART6){

		__HAL_RCC_USART6_CLK_ENABLE();

		NVIC_EnableIRQ(USART6_IRQn);

		HAL_NVIC_SetPriority(USART6_IRQn, 3, 3);

	}
	else if(config->usart == UART7){

		__HAL_RCC_UART7_CLK_ENABLE();

		NVIC_EnableIRQ(UART7_IRQn);

		HAL_NVIC_SetPriority(UART7_IRQn, 3, 3);

	}
	else if(config->usart == UART8){

		__HAL_RCC_UART8_CLK_ENABLE();

		NVIC_EnableIRQ(UART8_IRQn);

		HAL_NVIC_SetPriority(UART8_IRQn, 3, 3);

	}


	int baud_num = uart_clock / config->baudrate;

	config->usart->BRR = baud_num;

	uint32_t reg_ = 0;

	reg_ |= CK_USART_CR1_UE;

	if(config->parity == PARITY_EVEN){

		reg_ |= CK_USART_CR1_PCE;	// Parity enable
		reg_ &= ~(CK_USART_CR1_PS);	// Parity even
	}
	else if(config->parity == PARITY_ODD){

		reg_ |= CK_USART_CR1_PCE;	// Parity enable
		reg_ |= CK_USART_CR1_PS;	// Parity odd
	}

	config->usart->CR1 |= reg_;
	config->usart->CR2 |= config->stop_bit << CK_USART_CR2_STOPBIT;

}

void CK_UART_DMA_InitTX(DMA_Stream_TypeDef* dma_stream, uint32_t ch){

	//HAL_UART_Transmit_DMA
	//Enable the DMA to handle bufferable transfers. It says to set it to 1 for usart
	dma_stream->CR |= (1u<<20);
	dma_stream->CR |= (1u<<6) | (1<<10) | (0u<<16); // Memory to Peripheral, Memory increment, High priority, channel
}

void CK_UART_RXDMAInit(void){
	//HAL_UART_Transmit_DMA
}

void CK_UART_DMA_EnableClock(DMA_TypeDef* dma){

	if(dma == DMA1){
		RCC->AHB1ENR |= RCC_DMA1EN; // DMA1 Clock enable
	}
	else if(dma == DMA2){
		RCC->AHB1ENR |= RCC_DMA2EN; // DMA2 Clock enable
	}
}

// This will be called before starting the transfer by enabling dma.
void CK_UART_DMA_SetBuffer(DMA_Stream_TypeDef* dma_stream, uint8_t* dma_buffer, uint32_t transferSize){

	dma_stream->M0AR  = (uint32_t)dma_buffer;  // Set data buffer address
	dma_stream->NDTR  = transferSize;          // Set number of data to transfer

}

void CK_UART_DMA_SetPeripheralAddress(DMA_Stream_TypeDef* dma_stream, uint32_t address){
	dma_stream->PAR = (uint32_t)address;
}

void CK_UART_DMA_TCInterruptEnable(DMA_Stream_TypeDef* dma_stream){
	dma_stream->CR |= 1u << 4; // TCIE
}

void CK_UART_DMA_TXEnable(USART_TypeDef* uart){

    uart->CR3 |= 1u<<7; // DMA enable transmitter
}

void CK_UART_DMA_TXDisable(USART_TypeDef* uart){

    uart->CR3 &= ~(1u<<7); // DMA disable transmitter
}

void CK_UART_DMA_RXEnable(USART_TypeDef* uart){

    uart->CR3 |= 1u<<6; // DMA enable receiver
}

void CK_UART_DMA_RXDisable(USART_TypeDef* uart){

    uart->CR3 &= ~(1u<<6); // DMA disable receiver
}

void CK_UART_DMA_Enable(DMA_Stream_TypeDef* dma_stream){

    dma_stream->CR |= DMA_CR_Enable; //Enable
}

void CK_UART_DMA_Disable(DMA_Stream_TypeDef* dma_stream){

    dma_stream->CR &= ~DMA_CR_Enable; //Disable
}

void CK_UART_DMA_ClearFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream == DMA1_Stream0)dma->LIFCR = DMA_STREAM0_CLEAR;
	else if(dma_stream == DMA1_Stream1)dma->LIFCR = DMA_STREAM1_CLEAR;
	else if(dma_stream == DMA1_Stream2)dma->LIFCR = DMA_STREAM2_CLEAR;
	else if(dma_stream == DMA1_Stream3)dma->LIFCR = DMA_STREAM3_CLEAR;
	else if(dma_stream == DMA1_Stream4)dma->HIFCR = DMA_STREAM4_CLEAR;
	else if(dma_stream == DMA1_Stream5)dma->HIFCR = DMA_STREAM5_CLEAR;
	else if(dma_stream == DMA1_Stream6)dma->HIFCR = DMA_STREAM6_CLEAR;
	else if(dma_stream == DMA1_Stream7)dma->HIFCR = DMA_STREAM7_CLEAR;

	else if(dma_stream == DMA2_Stream0)dma->LIFCR = DMA_STREAM0_CLEAR;
	else if(dma_stream == DMA2_Stream1)dma->LIFCR = DMA_STREAM1_CLEAR;
	else if(dma_stream == DMA2_Stream2)dma->LIFCR = DMA_STREAM2_CLEAR;
	else if(dma_stream == DMA2_Stream3)dma->LIFCR = DMA_STREAM3_CLEAR;
	else if(dma_stream == DMA2_Stream4)dma->HIFCR = DMA_STREAM4_CLEAR;
	else if(dma_stream == DMA2_Stream5)dma->HIFCR = DMA_STREAM5_CLEAR;
	else if(dma_stream == DMA2_Stream6)dma->HIFCR = DMA_STREAM6_CLEAR;
	else if(dma_stream == DMA2_Stream7)dma->HIFCR = DMA_STREAM7_CLEAR;
}

void CK_UART_DMA_ClearTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma_stream == DMA1_Stream0)dma->LIFCR |= DMA_STREAM0_TC_CLEAR;
	else if(dma_stream == DMA1_Stream1)dma->LIFCR |= DMA_STREAM1_TC_CLEAR;
	else if(dma_stream == DMA1_Stream2)dma->LIFCR |= DMA_STREAM2_TC_CLEAR;
	else if(dma_stream == DMA1_Stream3)dma->LIFCR |= DMA_STREAM3_TC_CLEAR;
	else if(dma_stream == DMA1_Stream4)dma->HIFCR |= DMA_STREAM4_TC_CLEAR;
	else if(dma_stream == DMA1_Stream5)dma->HIFCR |= DMA_STREAM5_TC_CLEAR;
	else if(dma_stream == DMA1_Stream6)dma->HIFCR |= DMA_STREAM6_TC_CLEAR;
	else if(dma_stream == DMA1_Stream7)dma->HIFCR |= DMA_STREAM7_TC_CLEAR;

	else if(dma_stream == DMA2_Stream0)dma->LIFCR |= DMA_STREAM0_TC_CLEAR;
	else if(dma_stream == DMA2_Stream1)dma->LIFCR |= DMA_STREAM1_TC_CLEAR;
	else if(dma_stream == DMA2_Stream2)dma->LIFCR |= DMA_STREAM2_TC_CLEAR;
	else if(dma_stream == DMA2_Stream3)dma->LIFCR |= DMA_STREAM3_TC_CLEAR;
	else if(dma_stream == DMA2_Stream4)dma->HIFCR |= DMA_STREAM4_TC_CLEAR;
	else if(dma_stream == DMA2_Stream5)dma->HIFCR |= DMA_STREAM5_TC_CLEAR;
	else if(dma_stream == DMA2_Stream6)dma->HIFCR |= DMA_STREAM6_TC_CLEAR;
	else if(dma_stream == DMA2_Stream7)dma->HIFCR |= DMA_STREAM7_TC_CLEAR;
}

uint8_t CK_UART_DMA_IsTransferComplete(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

	if(dma->LISR & DMA_STREAM0_TCIF)return 1;
	if(dma->LISR & DMA_STREAM1_TCIF)return 1;
	if(dma->LISR & DMA_STREAM2_TCIF)return 1;
	if(dma->LISR & DMA_STREAM3_TCIF)return 1;
	if(dma->HISR & DMA_STREAM4_TCIF)return 1;
	if(dma->HISR & DMA_STREAM5_TCIF)return 1;
	if(dma->HISR & DMA_STREAM6_TCIF)return 1;
	if(dma->HISR & DMA_STREAM7_TCIF)return 1;

    return 0;
}

void CK_UART_Reset(USART_TypeDef* uart){

	if(uart == USART1){
		__HAL_RCC_USART1_CLK_DISABLE();
	}
	else if(uart == USART2){
		__HAL_RCC_USART2_CLK_DISABLE();
	}
	else if(uart == USART3){
		__HAL_RCC_USART3_CLK_DISABLE();
	}
	else if(uart == UART4){
		__HAL_RCC_UART4_CLK_DISABLE();
	}
	else if(uart == USART6){
		__HAL_RCC_USART6_CLK_DISABLE();
	}
	else if(uart == UART7){
		__HAL_RCC_UART7_CLK_DISABLE();
	}
	else if(uart == UART8){
		__HAL_RCC_UART8_CLK_DISABLE();
	}

    uart->CR1 = 0;
    uart->BRR = 0;

}

// This method waits until it finishes.
// It takes around 1.5ms to send 16 bytes.
void CK_UART_SendPolling(USART_TypeDef* uart, uint8_t data){

    uart->CR1 |= CK_USART_CR1_TE;
    uart->TDR = data;

    while((uart->ISR & CK_USART_SR_TXE) == 0);
    while((uart->ISR & CK_USART_SR_TC) == 0);
}

void CK_UART_SendInterrupt(USART_TypeDef* uart, uint8_t data){

	CK_UART_TCInterruptEnable(uart);
    uart->TDR = data;
}

void CK_UART_ClearFlags(USART_TypeDef* uart){

    uart->ICR = 0xFFFFFFFF;
}

void CK_UART_TXEnable(USART_TypeDef* uart){

	uart->CR1 |= CK_USART_CR1_TE;
}

void CK_UART_RXEnable(USART_TypeDef* uart){

	uart->CR1 |= CK_USART_CR1_RE;

}

void CK_UART_TXEInterruptEnable(USART_TypeDef* uart){
    uart->CR1 |= CK_USART_CR1_TXEIE;
}

void CK_UART_TXEInterruptDisable(USART_TypeDef* uart){
    uart->CR1 &= ~(CK_USART_CR1_TXEIE);
}

void CK_UART_TCInterruptEnable(USART_TypeDef* uart){
    uart->CR1 |= CK_USART_SR_TCIE;
}

void CK_UART_TCInterruptDisable(USART_TypeDef* uart){
    uart->CR1 &= ~(CK_USART_SR_TCIE);
}

void CK_UART_RXInterruptEnable(USART_TypeDef* uart){
    uart->CR1 |= CK_USART_CR1_RXNEIE;
}

void CK_UART_RXInterruptDisable(USART_TypeDef* uart){
    uart->CR1 &= ~(CK_USART_CR1_RXNEIE);
}

void CK_UART_DMAEnable(USART_TypeDef* uart){

    uart->CR3 |= 1u<<7; // DMA enable transmitter
}

void CK_UART_DMADisable(USART_TypeDef* uart){

    uart->CR3 &= ~(1u<<7); // DMA disable transmitter
}







