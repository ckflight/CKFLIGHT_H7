
#ifndef CK_MAG3110_H_
#define CK_MAG3110_H_

#include "CK_DEFINITIONS.h"

void CK_MAG3110_Init(I2C_TypeDef* I2Cn, uint32_t magFreq);

void CK_MAG3110_AlignMag(int x, int y, int z);

void CK_MAG3110_ReadMagRaw(void);

bool CK_MAG3110_isMagSensorInitialized(void);

#endif /* CK_MAG3110_H_ */
