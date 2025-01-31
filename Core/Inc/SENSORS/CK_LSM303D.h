
#ifndef CK_LSM303D_H_
#define CK_LSM303D_H_

#include "CK_DEFINITIONS.h"

void CK_LSM303D_AccInit(I2C_TypeDef* I2Cn, uint32_t accFreq);

void CK_LSM303D_MagInit(I2C_TypeDef* I2Cn, uint32_t magFreq);

void CK_LSM303D_AlignAcc(int x, int y, int z);

void CK_LSM303D_AlignMag(int x, int y, int z);

void CK_LSM303D_ReadAccRaw(void);

void CK_LSM303D_ReadMagRaw(void);

bool CK_LSM303D_isAccSensorInitialized(void);

bool CK_LSM303D_isMagSensorInitialized(void);

#endif /* CK_LSM303D_H_ */
