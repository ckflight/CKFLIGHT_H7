
#ifndef CK_ESC_H_
#define CK_ESC_H_

#include "CK_DEFINITIONS.h"
#include "CK_PWM.h"
#include "CK_DSHOT.h"

typedef enum
{
	PWM_MODE,
	DSHOT_MODE

}CK_ESC_Mode;

extern CK_ESC_Mode esc_mode;

void CK_ESC_Init(CK_ESC_Mode md, targetFreq_e main_time);

void CK_ESC_SetMotor(int num1, int num2, int num3, int num4);

void CK_ESC_StopMotors(void);

void CK_ESC_MOTOR_TEST_MODE_Enable(int num);

void CK_ESC_MOTOR_TEST_MODE_Disable(void);

#endif /* CK_ESC_H_ */
