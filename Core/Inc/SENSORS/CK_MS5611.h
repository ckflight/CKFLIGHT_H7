
#ifndef CK_MS5611_H_
#define CK_MS5611_H_

#include "CK_DEFINITIONS.h"

typedef enum{
	MS5611_PRESSURE_ADC						=0,
	MS5611_TEMPERATURE_ADC					=1

}adcTypeMS5611_e;

void CK_MS5611_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t baroFreq);

void CK_MS5611_CycleUpdate(void);

void CK_MS5611_Calculate();

void CK_MS5611_ReadPROM(void);

void CK_MS5611_SendCommand(uint8_t command);

void CK_MS5611_ReadADC(adcTypeMS5611_e adc);

int CK_MS5611_IsOneCycleCompleted();


#endif /* CK_MS5611_H_ */
