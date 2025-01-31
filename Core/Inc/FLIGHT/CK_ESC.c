
#include <COMMON/maths.h>
#include "FLIGHT/CK_ESC.h"
#include "FLIGHT/CK_PWM.h"
#include "FLIGHT/CK_DSHOT.h"
#include "FLIGHT/CK_RECEIVER.h"


CK_ESC_Mode esc_mode;

bool is_motor_test_mode = false;
int esc_test_motor_num = 0;

void CK_ESC_Init(CK_ESC_Mode md, targetFreq_e main_time){

	esc_mode = md;

	is_motor_test_mode = false;

	esc_test_motor_num = 0;

	if(esc_mode == PWM_MODE){
		CK_PWM_Init(PWM_ONESHOT42);

	}
	else if(esc_mode == DSHOT_MODE){
		CK_DSHOT_Init(INLINE_MODE, main_time);

	}
}

void CK_ESC_SetMotor(int num1, int num2, int num3, int num4){

	if(esc_mode == PWM_MODE){
		CK_PWM_SetMotor1(num1);
		CK_PWM_SetMotor2(num2);
		CK_PWM_SetMotor3(num3);
		CK_PWM_SetMotor4(num4);

	}
	else if(esc_mode == DSHOT_MODE){

		// MOTOR TEST MODE
		if(is_motor_test_mode){

			// ESC REVERSE MOUNT
			// if esc is reversed onboard 180 degrees then 1 4 is swapped and 2 3 is swapped
			if(MIXER_ESC_REVERSED){
				if(esc_test_motor_num == 1){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(num1);
				}
				else if(esc_test_motor_num == 2){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(num2);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
				else if(esc_test_motor_num == 3){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(num3);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
				else if(esc_test_motor_num == 4){
					CK_DSHOT_SetMotor1(num4);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
			}

			// ESC NORMAL MOUNT
			else{
				if(esc_test_motor_num == 1){
					CK_DSHOT_SetMotor1(num1);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
				else if(esc_test_motor_num == 2){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(num2);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
				else if(esc_test_motor_num == 3){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(num3);
					CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);
				}
				else if(esc_test_motor_num == 4){
					CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
					CK_DSHOT_SetMotor4(num4);
				}
			}

		}

		// NORMAL FLIGHT MODE
		else{

			// ESC REVERSE MOUNT
			// if esc is reversed onboard 180 degrees then 1 4 is swapped and 2 3 is swapped
			if(MIXER_ESC_REVERSED){
				CK_DSHOT_SetMotor1(num4);
				CK_DSHOT_SetMotor2(num3);
				CK_DSHOT_SetMotor3(num2);
				CK_DSHOT_SetMotor4(num1);
			}

			// ESC NORMAL MOUNT
			else{
				CK_DSHOT_SetMotor1(num1);
				CK_DSHOT_SetMotor2(num2);
				CK_DSHOT_SetMotor3(num3);
				CK_DSHOT_SetMotor4(num4);
			}

		}

	}
}

void CK_ESC_StopMotors(void){

	if(esc_mode == PWM_MODE){
		CK_PWM_SetMotor1(PWM_MIN_RANGE);
		CK_PWM_SetMotor2(PWM_MIN_RANGE);
		CK_PWM_SetMotor3(PWM_MIN_RANGE);
		CK_PWM_SetMotor4(PWM_MIN_RANGE);

	}
	else if(esc_mode == DSHOT_MODE){
		CK_DSHOT_SetMotor1(DSHOT_COMMAND_MOTOR_STOP);
		CK_DSHOT_SetMotor2(DSHOT_COMMAND_MOTOR_STOP);
		CK_DSHOT_SetMotor3(DSHOT_COMMAND_MOTOR_STOP);
		CK_DSHOT_SetMotor4(DSHOT_COMMAND_MOTOR_STOP);

	}

}

void CK_ESC_MOTOR_TEST_MODE_Enable(int num){

	is_motor_test_mode = true;

	esc_test_motor_num = num;
}

void CK_ESC_MOTOR_TEST_MODE_Disable(void){

	is_motor_test_mode = false;
}

