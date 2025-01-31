
#ifndef CK_ICM42605_H_
#define CK_ICM42605_H_

#include "CK_DEFINITIONS.h"

uint8_t CK_ICM42605_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);

uint8_t CK_ICM42605_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);

void CK_ICM42605_AlignGyro(int x, int y, int z);

void CK_ICM42605_AlignAcc(int x, int y, int z);

void CK_ICM42605_ReadGyroRaw(void);

void CK_ICM42605_ReadAccRaw(void);

bool CK_ICM42605_isGyroSensorInitialized(void);

bool CK_ICM42605_isAccSensorInitialized(void);

#endif /* CK_ICM42688P_H_ */
