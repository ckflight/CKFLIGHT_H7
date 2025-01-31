
#ifndef INC_FLIGHT_CK_ADJUSTMENT_H_
#define INC_FLIGHT_CK_ADJUSTMENT_H_

#include "CK_DEFINITIONS.h"

void CK_ADJUSTMENT_Init(uint32_t adjustmentTime, uint32_t mainTime);

void CK_ADJUSTMENT_Update(void);

int CK_ADKUSTMENT_GetParameters(int increment_multiplier);

void CK_ADJUSTMENT_SetPIDParameters(void);

void CK_ADJUSTMENT_SetOtherParameters(void);

void CK_ADJUSTMENT_SetAntiGravityParameters(void);

void CK_ADJUSTMENT_SaveParameters(void);

uint8_t CK_ADJUSTEMENT_IsAdjustmentModeOn(void);


#endif /* INC_FLIGHT_CK_ADJUSTMENT_H_ */
