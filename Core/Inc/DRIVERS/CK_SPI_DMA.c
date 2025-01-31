
#include "CK_SPI_DMA.h"
#include "CK_SPI.h"
#include "string.h"			// memcpy

#define DMA_BUFFER_SIZE         512
#define RCC_DMA1EN              1u<<21 // DMA1 Clock enable
#define RCC_DMA2EN              1u<<22 // DMA2 Clock enable

#define DMA_STREAM0_CLEAR       0x3D<<0
#define DMA_STREAM1_CLEAR       0x3D<<6
#define DMA_STREAM2_CLEAR       0x3D<<16
#define DMA_STREAM3_CLEAR       0x3D<<22

#define DMA_STREAM4_CLEAR       0x3D<<0
#define DMA_STREAM5_CLEAR       0x3D<<6
#define DMA_STREAM6_CLEAR       0x3D<<16
#define DMA_STREAM7_CLEAR       0x3D<<22

#define DMA_STREAM0_TC_CLEAR       0x20<<0
#define DMA_STREAM1_TC_CLEAR       0x20<<6
#define DMA_STREAM2_TC_CLEAR       0x20<<16
#define DMA_STREAM3_TC_CLEAR       0x20<<22

#define DMA_STREAM4_TC_CLEAR       0x20<<0
#define DMA_STREAM5_TC_CLEAR       0x20<<6
#define DMA_STREAM6_TC_CLEAR       0x20<<16
#define DMA_STREAM7_TC_CLEAR       0x20<<22

#define DMA_STREAM0_TCIF        1u<<5
#define DMA_STREAM1_TCIF        1u<<11
#define DMA_STREAM2_TCIF        1u<<21
#define DMA_STREAM3_TCIF        1u<<27
#define DMA_STREAM4_TCIF        1u<<5
#define DMA_STREAM5_TCIF        1u<<11
#define DMA_STREAM6_TCIF        1u<<21
#define DMA_STREAM7_TCIF        1u<<27

#define DMA_CR_Enable           1u<<0

uint8_t dma_tx_buffer[DMA_BUFFER_SIZE];

uint16_t dmaBufferIndex;

DMA_HandleTypeDef hdma_handler_rx;
DMA_HandleTypeDef hdma_handler_tx;

/*
 * 1. CK_SPI_DMA_Init use it once to configure spi dma
 * Then before each transfer
 *
 * 2. CK_SPI_DMA_SetBuffer to setup tranfer buffer and its size
 * 3. CK_SPI_DMA_Enable
 * 4. CK_SPI_EnableDMA
 * 5. When finished it will go to interrupt.
 */

void CK_SPI_DMA_InitTX(DMA_Stream_TypeDef* dma_stream, SPI_TypeDef* SPI_, uint32_t request){

	// SPI is initialized for DMA related spi in CK_PERIPHERALS.c file

	// DMA Config
	hdma_handler_tx.Instance                 = dma_stream;
	hdma_handler_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_handler_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_handler_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
	hdma_handler_tx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
	hdma_handler_tx.Init.Request             = request;
	hdma_handler_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_handler_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_handler_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_handler_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_handler_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_handler_tx.Init.Mode                = DMA_NORMAL;
	hdma_handler_tx.Init.Priority            = DMA_PRIORITY_HIGH;

	HAL_DMA_Init(&hdma_handler_tx);

	// Associate the initialized DMA handle to the the SPI handle
	if(SPI_ == SPI1)__HAL_LINKDMA(&hspi1, hdmatx, hdma_handler_tx);
	else if(SPI_ == SPI2)__HAL_LINKDMA(&hspi2, hdmatx, hdma_handler_tx);
	else if(SPI_ == SPI3)__HAL_LINKDMA(&hspi3, hdmatx, hdma_handler_tx);
	else if(SPI_ == SPI4)__HAL_LINKDMA(&hspi4, hdmatx, hdma_handler_tx);

}

void CK_SPI_DMA_InitRX(DMA_Stream_TypeDef* dma_stream, SPI_TypeDef* SPI_, uint32_t request){

	// SPI is initialized for DMA related spi in CK_PERIPHERALS.c file

	// DMA Config
	hdma_handler_rx.Instance                 = dma_stream;
	hdma_handler_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_handler_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_handler_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
	hdma_handler_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
	hdma_handler_rx.Init.Request             = request;
	hdma_handler_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_handler_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_handler_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_handler_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_handler_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_handler_rx.Init.Mode                = DMA_NORMAL;
	hdma_handler_rx.Init.Priority            = DMA_PRIORITY_HIGH;

	HAL_DMA_Init(&hdma_handler_rx);

	// Associate the initialized DMA handle to the the SPI handle
	if(SPI_ == SPI1)__HAL_LINKDMA(&hspi1, hdmarx, hdma_handler_rx);
	else if(SPI_ == SPI2)__HAL_LINKDMA(&hspi2, hdmarx, hdma_handler_rx);
	else if(SPI_ == SPI3)__HAL_LINKDMA(&hspi3, hdmarx, hdma_handler_rx);
	else if(SPI_ == SPI4)__HAL_LINKDMA(&hspi4, hdmarx, hdma_handler_rx);

}


void CK_SPI_DMA_EnableClock(DMA_TypeDef* DMA_){
	if(DMA_ == DMA1){
		__HAL_RCC_DMA1_CLK_ENABLE();
	}
	else if(DMA_ == DMA2){
		__HAL_RCC_DMA2_CLK_ENABLE();
	}
}

// This will be called before starting the transfer by enabling dma.
void CK_SPI_DMA_SetBuffer(DMA_Stream_TypeDef* dma_stream, uint8_t* dma_buffer, uint32_t transferSize){

	dma_stream->M0AR  = (uint32_t)dma_buffer;  // Set data buffer address
	dma_stream->NDTR  = transferSize;          // Set number of data to transfer

}

void CK_SPI_DMA_SetPeripheralAddress(DMA_Stream_TypeDef* dma_stream, uint32_t address){
	dma_stream->PAR = (uint32_t)address;
}

void CK_SPI_DMA_TCInterruptEnable(DMA_Stream_TypeDef* dma_stream){
	dma_stream->CR |= 1u << 4; 	// TCIE
}

void CK_SPI_DMA_ClearFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

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

void CK_SPI_DMA_ClearTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

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

void CK_SPI_DMA_Enable(DMA_Stream_TypeDef* dma_stream){

    dma_stream->CR |= DMA_CR_Enable; //Enable
}

void CK_SPI_DMA_Disable(DMA_Stream_TypeDef* dma_stream){

    dma_stream->CR &= ~DMA_CR_Enable; //Disable
}

uint8_t CK_SPI_DMA_IsTransferComplete(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream){

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

uint16_t CK_SPI_DMA_NumberOfDataLeft(DMA_Stream_TypeDef* dma_stream){

    return dma_stream->NDTR;
}















