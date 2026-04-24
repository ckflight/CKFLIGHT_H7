
#ifndef CK_ICM45686_H_
#define CK_ICM45686_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_ICM45686_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

uint8_t CK_ICM45686_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);

bool CK_ICM45686_isGyroSensorInitialized(void);

#endif /* CK_ICM45686_H_ */
