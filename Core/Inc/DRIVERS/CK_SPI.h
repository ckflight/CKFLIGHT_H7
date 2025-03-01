
#ifndef INC_CK_SPI_H_
#define INC_CK_SPI_H_

#include "CK_DEFINITIONS.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

typedef enum
{
	CK_SPIx_CR1_Fclk_Div2                 = 0,
	CK_SPIx_CR1_Fclk_Div4                 = 1,
	CK_SPIx_CR1_Fclk_Div8                 = 2,
	CK_SPIx_CR1_Fclk_Div16                = 3,
	CK_SPIx_CR1_Fclk_Div32                = 4,
	CK_SPIx_CR1_Fclk_Div64                = 5,
	CK_SPIx_CR1_Fclk_Div128               = 6,
	CK_SPIx_CR1_Fclk_Div256               = 7

}CK_SPIx_CR1_Fclk_Div;

typedef enum
{
	CK_SPI_USE_HAL,
	CK_SPI_USE_BAREMETAL

}CK_SPIx_LibraryType;

void CK_SPI_Init(SPI_TypeDef* spi_n, uint32_t clock, CK_SPIx_LibraryType type);

void CK_SPI_Enable(SPI_TypeDef* SPI_);

void CK_SPI_Disable(SPI_TypeDef* SPI_);

void CK_SPI_EnableTXDMA(SPI_TypeDef* SPI_);

void CK_SPI_EnableRXDMA(SPI_TypeDef* SPI_);

void CK_SPI_DisableTXDMA(SPI_TypeDef* SPI_);

void CK_SPI_DisableRXDMA(SPI_TypeDef* SPI_);

void CK_SPI_StartTransfer(SPI_TypeDef* SPI_, uint32_t tsize);

void CK_SPI_ChangeClock(SPI_TypeDef* SPI_, CK_SPIx_CR1_Fclk_Div clk);

uint8_t CK_SPI_WriteRegister(uint8_t reg, uint8_t data, SPI_TypeDef* SPI_, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin);

void CK_SPI_ReadRegisterMulti(uint8_t reg, SPI_TypeDef* SPI_, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin, uint8_t* dataIn, int count);

uint8_t CK_SPI_Transfer(SPI_TypeDef* SPI_, uint8_t data);

void CK_SPI_MultiTransfer(SPI_TypeDef* SPI_, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t len);

uint8_t CK_SPI_WaitTransfer(SPI_TypeDef* SPI_);

int CK_SPI_CheckInitialized(SPI_TypeDef* SPI_);

void CK_SPI_TimeOutCounter(SPI_TypeDef* SPI_);

uint32_t CK_SPI_GetTimeOut(SPI_TypeDef* SPI_);

void CK_SPI_ResetTimeOut(SPI_TypeDef* SPI_);

CK_SPIx_CR1_Fclk_Div CK_SPI_GetClockRate(SPI_TypeDef* SPI_, uint32_t spi_clock);

#endif /* INC_CK_SPI_H_ */
