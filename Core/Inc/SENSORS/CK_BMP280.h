
#ifndef CK_BMP280_H_
#define CK_BMP280_H_

#include "CK_DEFINITIONS.h"

void CK_BMP280_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t baroFreq);

void CK_BMP280_Calculate(void);

void CK_BMP280_CalculateTemperature();

void CK_BMP280_CalculatePressure();

void CK_BMP280_ReadADC(void);

#endif /* CK_BMP280_H_ */
