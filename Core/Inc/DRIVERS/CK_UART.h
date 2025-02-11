
#ifndef CK_UART_H_
#define CK_UART_H_

#include "CK_CIRCULARBUFFER.h"
#include "CK_DEFINITIONS.h"
#include "CK_GPIO.h"

#define CK_USART_CR1_TE         		1u<<3    // TX ENABLE
#define CK_USART_CR1_RE         		1u<<2    // RX ENABLE
#define CK_USART_CR1_RXNEIE				1u<<5    // Receive Interrupt Enable
#define CK_USART_CR1_TXEIE				1u<<7    // Transfer Interrupt Enable
#define CK_USART_CR1_TCIE				1u<<6    // Transfer Complete Enable
#define CK_USART_CR1_UE          		1u<<0    // USART ENABLE
#define CK_USART_CR1_PCE      		    1u<<10   // PARITY ENABLED
#define CK_USART_CR1_PS      		    1u<<9    // PARITY SELECT 0=EVEN, 1=ODD

#define CK_USART_SR_TCIE                1u<<6    // TRANSFER COMPLETE

#define CK_USART_CR2_STOP_2Bit      	2u<<12   // 2 Stop Bits
#define CK_USART_CR2_STOPBIT      		12

#define CK_USART_SR_TXE          		1u<<7    // TX BUFFER EMPTY
#define CK_USART_SR_TC          		1u<<6    // TRANSFER COMPLETE
#define CK_USART_SR_RXNE          		1u<<5    // RECEIVE COMPLETE

typedef enum{

	RX_ONLY,
	TX_ONLY,
	RX_TX

}USART_DATA_MODE_;

typedef enum{

	RX_INTERRUPT,
	TX_INTERRUPT,
	RX_TX_INTERUPT

}USART_INTERRUPT_;

typedef enum{
	PARITY_EVEN = 0,
	PARITY_ODD,
	NO_PARITY

}USART_PARITY_;

typedef enum{
	STOP_BIT1 = 0,
	STOP_BIT0_5,
	STOP_BIT2,
	STOP_BIT1_5,

}USART_STOPBIT_;

typedef struct{

	USART_DATA_MODE_ 	mode;
	USART_INTERRUPT_ 	interrupt;
	USART_PARITY_ 		parity;
	USART_STOPBIT_ 		stop_bit;

	uint32_t 			baudrate;

	USART_TypeDef* 		usart;

	bool				use_circular_buffer;

	GPIO_TypeDef*		tx_gpio_type;
	uint16_t 			tx_gpio_pin;
	CK_GPIOx_AFx		tx_af;

	GPIO_TypeDef*		rx_gpio_type;
	uint16_t 			rx_gpio_pin;
	CK_GPIOx_AFx		rx_af;

}USART_CONFIGURATION_;

void CK_UART_Init(USART_CONFIGURATION_* config, circularBuffer_t* circular_buf);

void CK_UART_DMA_InitTX(DMA_Stream_TypeDef* dma_stream, uint32_t ch);

void CK_UART_RXDMAInit(void);

void CK_UART_DMA_EnableClock(DMA_TypeDef* dma);

void CK_UART_DMA_SetBuffer(DMA_Stream_TypeDef* dma_stream, uint8_t* dma_buffer, uint32_t transferSize);

void CK_UART_DMA_SetPeripheralAddress(DMA_Stream_TypeDef* dma_stream, uint32_t address);

void CK_UART_DMA_TCInterruptEnable(DMA_Stream_TypeDef* dma_stream);

void CK_UART_DMA_TXEnable(USART_TypeDef* uart);

void CK_UART_DMA_TXDisable(USART_TypeDef* uart);

void CK_UART_DMA_RXEnable(USART_TypeDef* uart);

void CK_UART_DMA_RXDisable(USART_TypeDef* uart);

void CK_UART_DMA_Enable(DMA_Stream_TypeDef* dma_stream);

void CK_UART_DMA_Disable(DMA_Stream_TypeDef* dma_stream);

void CK_UART_DMA_ClearFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_UART_DMA_ClearTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

uint8_t CK_UART_DMA_IsTransferComplete(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_UART_Reset(USART_TypeDef* uart);

void CK_UART_SendPolling(USART_TypeDef* uart, uint8_t data);

void CK_UART_SendInterrupt(USART_TypeDef* uart, uint8_t data);

void CK_UART_ClearFlags(USART_TypeDef* uart);

void CK_UART_TXEnable(USART_TypeDef* uart);

void CK_UART_RXEnable(USART_TypeDef* uart);

void CK_UART_TXEInterruptEnable(USART_TypeDef* uart);

void CK_UART_TXEInterruptDisable(USART_TypeDef* uart);

void CK_UART_TCInterruptEnable(USART_TypeDef* uart);

void CK_UART_TCInterruptDisable(USART_TypeDef* uart);

void CK_UART_RXInterruptEnable(USART_TypeDef* uart);

void CK_UART_RXInterruptDisable(USART_TypeDef* uart);

void CK_UART_DMAEnable(USART_TypeDef* uart);

void CK_UART_DMADisable(USART_TypeDef* uart);

#endif /* CK_UART_H_ */
