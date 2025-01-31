
#ifndef CK_ALTITUDE_H_
#define CK_ALTITUDE_H_

#include "CK_DEFINITIONS.h"

void CK_ALTITUDE_Update(uint32_t currentTime);

void CK_ALTITUDE_CalculateVelocityAndAltitude(uint32_t currentTime);

int32_t CK_ALTITUDE_CalculateThrottleAdjustment_AltitudeHold(int32_t estimatedVel, int32_t estimatedAlt, float accZ_tmp, float accZ_old);

int32_t CK_ALTITUDE_CalculateThrottleAdjustment_Landing(int32_t estimatedVel, float accZ_tmp, float accZ_old);

void CK_ALTITUDE_ApplyAltitudeHoldAdjustment(void);

int32_t CK_ALTITUDE_GetEstimatedAltitude(void);

int32_t CK_ALTITUDE_GetEstimatedVelocity(void);

int32_t CK_ALTITUDE_GetThrottleAdjustment_AltitudeHold(void);

int32_t CK_ALTITUDE_GetThrottleAdjustment_Landing(void);

float CK_ALTITUDE_GetAccVelocity(void);

float CK_ALTITUDE_GetFailsafeVelocity(void);

#endif /* CK_ALTITUDE_H_ */
