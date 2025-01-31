
#ifndef CK_MS5607_H_
#define CK_MS5607_H_

#include "CK_DEFINITIONS.h"

typedef enum{
	MS5607_PRESSURE_ADC,
	MS5607_TEMPERATURE_ADC

}adcTypeMS5607_e;

void CK_MS5607_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t baroFreq);

void CK_MS5607_CycleUpdate(void);

void CK_MS5607_Calculate();

void CK_MS5607_ReadPROM(void);

void CK_MS5607_SendCommand(uint8_t command);

void CK_MS5607_ReadADC(adcTypeMS5607_e adc);

int CK_MS5607_IsOneCycleCompleted();

#endif /* CK_MS5607_H_ */
