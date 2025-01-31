
#ifndef CK_ICM20602_H_
#define CK_ICM20602_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_ICM20602_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

uint8_t CK_ICM20602_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);

void CK_ICM20602_AlignGyro(int x, int y, int z);

void CK_ICM20602_AlignAcc(int x, int y, int z);

void CK_ICM20602_ReadSensorRaw_DMA(void);

void CK_ICM20602_ReadGyroRaw(void);

void CK_ICM20602_ReadAccRaw(void);

bool CK_ICM20602_isGyroSensorInitialized(void);

bool CK_ICM20602_isAccSensorInitialized(void);

#endif /* CK_ICM20602_H_ */
