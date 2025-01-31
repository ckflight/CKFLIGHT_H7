
#ifndef CK_SPI_DMA_H_
#define CK_SPI_DMA_H_

#include "CK_DEFINITIONS.h"

#if USE_H7 == 1
#include "stm32h7xx.h"
#endif

#if USE_F4 == 1
#include "stm32f4xx.h"
#endif

extern DMA_HandleTypeDef hdma_handler_rx;
extern DMA_HandleTypeDef hdma_handler_tx;

void CK_SPI_DMA_InitTX(DMA_Stream_TypeDef* dma_stream, SPI_TypeDef* SPI_, uint32_t request);

void CK_SPI_DMA_InitRX(DMA_Stream_TypeDef* dma_stream, SPI_TypeDef* SPI_, uint32_t request);

void CK_SPI_DMA_EnableClock(DMA_TypeDef* DMA_);

void CK_SPI_DMA_SetBuffer(DMA_Stream_TypeDef* dma_stream, uint8_t* dma_buffer, uint32_t transferSize);

void CK_SPI_DMA_SetPeripheralAddress(DMA_Stream_TypeDef* dma_stream, uint32_t address);

void CK_SPI_DMA_TCInterruptEnable(DMA_Stream_TypeDef* dma_stream);

void CK_SPI_DMA_ClearFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_SPI_DMA_ClearTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_SPI_DMA_Enable(DMA_Stream_TypeDef* dma_stream);

void CK_SPI_DMA_Disable(DMA_Stream_TypeDef* dma_stream);

uint8_t CK_SPI_DMA_IsTransferComplete(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

uint16_t CK_SPI_DMA_NumberOfDataLeft(DMA_Stream_TypeDef* dma_stream);

#endif /* CK_SPI_DMA_H_ */
