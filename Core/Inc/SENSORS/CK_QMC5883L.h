
#ifndef CK_QMC5883L_H_
#define CK_QMC5883L_H_

#include "CK_DEFINITIONS.h"

void CK_QMC5883L_Init(I2C_TypeDef* I2Cn, uint32_t magFreq);

void CK_QMC5883L_AlignMag(int x, int y, int z);

void CK_QMC5883L_ReadMagRaw(void);

bool CK_QMC5883L_isMagSensorInitialized(void);

#endif /* CK_QMC5883L_H_ */
