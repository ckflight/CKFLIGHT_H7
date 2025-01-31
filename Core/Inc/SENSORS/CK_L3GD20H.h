
#ifndef CK_L3GD20H_H_
#define CK_L3GD20H_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_L3GD20H_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

void CK_L3GD20H_AlignGyro(int x, int y, int z);

void CK_L3GD20H_ReadGyroRaw(void);

bool CK_L3GD20H_isGyroSensorInitialized(void);

#endif /* CK_L3GD20H_H_ */
