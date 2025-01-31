
#ifndef CK_FXOS8700CQ_H_
#define CK_FXOS8700CQ_H_

#include "CK_DEFINITIONS.h"

void CK_FXOS8700CQ_AccInit(I2C_TypeDef* I2Cn, uint32_t accFreq);

void CK_FXOS8700CQ_MagInit(I2C_TypeDef* I2Cn, uint32_t magFreq);

void CK_FXOS8700CQ_AlignAcc(int x, int y, int z);

void CK_FXOS8700CQ_AlignMag(int x, int y, int z);

void CK_FXOS8700CQ_ReadAccRaw(void);

void CK_FXOS8700CQ_ReadMagRaw(void);

bool CK_FXOS8700CQ_isAccSensorInitialized(void);

bool CK_FXOS8700CQ_isMagSensorInitialized(void);

#endif /* CK_FXOS8700CQ_H_ */
