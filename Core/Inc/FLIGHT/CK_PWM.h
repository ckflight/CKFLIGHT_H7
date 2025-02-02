
#ifndef INC_FLIGHT_CK_PWM_H_
#define INC_FLIGHT_CK_PWM_H_

#include "CK_DEFINITIONS.h"

#define PWM_MIN_RANGE		1000
#define PWM_MAX_RANGE		2000

#define PWM_MIN_THROTTLE	1050
#define PWM_MAX_THROTTLE	2000

#define PWM_DISARM_VALUE	1000

typedef enum
{
	PWM_ONESHOT125,
	PWM_ONESHOT42

}CK_PWM_Mode;

void CK_PWM_Init(CK_PWM_Mode md);

void CK_PWM_InitEndPoints(float outputLimit, float* outputLow, float* outputHigh, float* disarmMotorOutput);

void CK_PWM_Init1(void);

void CK_PWM_Init2(void);

void CK_PWM_Init3(void);

void CK_PWM_Init4(void);

void CK_PWM_SetMotor1(int dty);

void CK_PWM_SetMotor2(int dty);

void CK_PWM_SetMotor3(int dty);

void CK_PWM_SetMotor4(int dty);

#endif /* INC_FLIGHT_CK_PWM_H_ */
