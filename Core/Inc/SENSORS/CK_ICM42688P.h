
#ifndef CK_ICM42688P_H_
#define CK_ICM42688P_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_ICM42688P_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

uint8_t CK_ICM42688P_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);

void CK_ICM42688P_AlignGyro(int x, int y, int z);

void CK_ICM42688P_AlignAcc(int x, int y, int z);

void CK_ICM42688P_ReadGyroRaw(void);

void CK_ICM42688P_ReadAccRaw(void);

void CK_ICM42688P_ReadSensorRaw_DMA(void);

float CK_ICM42688P_ReadTempRaw(void);

bool CK_ICM42688P_isGyroSensorInitialized(void);

bool CK_ICM42688P_isAccSensorInitialized(void);

#endif /* CK_ICM42688P_H_ */
