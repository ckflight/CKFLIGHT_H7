
#ifndef CK_IIM42652_H_
#define CK_IIM42652_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_IIM42652_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

uint8_t CK_IIM42652_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);

void CK_IIM42652_AlignGyro(int x, int y, int z);

void CK_IIM42652_AlignAcc(int x, int y, int z);

void CK_IIM42652_ReadGyroRaw(void);

void CK_IIM42652_ReadAccRaw(void);

void CK_IIM42652_ReadSensorRaw_DMA(void);

float CK_IIM42652_ReadTempRaw(void);

bool CK_IIM42652_isGyroSensorInitialized(void);

bool CK_IIM42652_isAccSensorInitialized(void);

#endif
