
#ifndef CK_HMC5983_H_
#define CK_HMC5983_H_

#include "CK_DEFINITIONS.h"

void CK_HMC5983_Init(I2C_TypeDef* I2Cn, uint32_t magFreq);

void CK_HMC5983_AlignMag(int x, int y, int z);

void CK_HMC5983_ReadMagRaw(void);

bool CK_HMC5983_isMagSensorInitialized(void);

#endif /* CK_HMC5983_H_ */
