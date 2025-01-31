
#include "FLIGHT/CK_ADJUSTMENT.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/pid_init.h"
#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"

#include "FLASH/CK_FLASH.h"

#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/CK_PRINTER.h"

#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_TIME_HAL.h"

int PID_PARAMETERS[PID_ARRAY_ROW][PID_ARRAY_COLUMN] = {

			// P   I   D   F   Dmax
	[0] 	= {0,  0,  0,  0,  0}, // Roll
	[1] 	= {0,  0,  0,  0,  0}, // Pitch
	[2] 	= {0,  0,  0,  0,  0}  // Yaw

};

uint16_t tpa_breakpoint_adjustment = 0;
uint8_t tpa_rate_adjustment = 0;

syncTimer_t adjustment_sync;

DEBUG_TIME_t adjustment_debug;

uint8_t is_adjustment_mode_on = 0;
/*
 * In flight adjustment mode will be used to configure pid parameters
 * by transmitter radio.
 *
 * Disarm the quad with aux1 and take aux3 to 3
 * aux4 is responsible of selecting: 1->pid, 2->other modes
 * aux7 and 8 is responsible of selecting roll pitch yaw and p, i, d numbers.
 * aux6 rotation is responsible of dec. inc.
 *
 * The aux map is:
 *
 *
 * Aux1 = disarm, aux3 = 3 start inflight adjustment.
 *
 * Aux4 = 1 is PID mode:
 * Aux7 = 1 is Roll, 2 is Pitch, 3 is Yaw
 * Aux8 = 1 is P term, 2 is I term, 3 is D term.
 *
 * Aux4 = 2 is Other mode:
 * Aux7 = 1 is TPA  --> Aux8 = 1 is TPA rate, 2 is TPA breakpoint
 * Aux7 = 2 is Dmin --> Aux8 = 1 is Roll Dmin, 2 is Pitch Dmin, 3 Yaw Dmin
 *
 *
 * Each new number is updated on pid side. Quad can fly without saving these parameters to eeprom.
 * To write to eeprom take aux2 to 2. Quad will indicate it with buzzer and restart.
 * After restart there is no need for unplug-plug the battery.
 */

void CK_ADJUSTMENT_Init(uint32_t adjustmentTime, uint32_t mainTime){

    adjustment_sync.syncCounter = 0;

    adjustment_sync.targetLoopTime = adjustmentTime;

    adjustment_sync.syncRate = adjustmentTime / mainTime;

	uint8_t adjustment_buffer[128];

	// Read pid parameters
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, adjustment_buffer, CONFIG_PID_BYTES, CONFIG_PID_OFFSET);

	for(int r = 0; r < PID_ARRAY_ROW; r++){

		for(int c = 0; c < PID_ARRAY_COLUMN; c++){

			PID_PARAMETERS[r][c] = adjustment_buffer[(r*PID_ARRAY_COLUMN) + c];

		}
	}

	// Read tpa parameters
	tpa_breakpoint_adjustment = (adjustment_buffer[PID_ARRAY_ROW * PID_ARRAY_COLUMN] << 8) | adjustment_buffer[PID_ARRAY_ROW * PID_ARRAY_COLUMN + 1];
	tpa_rate_adjustment 	  = adjustment_buffer[PID_ARRAY_ROW * PID_ARRAY_COLUMN + 2];


}

void CK_ADJUSTMENT_Update(void){

	adjustment_sync.syncCounter++;

	if(adjustment_sync.syncCounter >= adjustment_sync.syncRate){

        #if defined(DEBUG_TIMING)
		adjustment_debug.start_time = CK_TIME_GetMicroSec();
        #endif

		adjustment_sync.syncCounter = 0;

		// Disarmed
		if(receiver_aux.aux1 == 1){

			// switch 1 is high and switch 3 is 3 enter in flight adjustment mode
			if(receiver_aux.aux3 == 3 && receiver_aux.aux2 == 1){

				// See the map in the description above.
				if(receiver_aux.aux4 == 1){
					CK_ADJUSTMENT_SetPIDParameters();
				}
				else if(receiver_aux.aux4 == 2){
					CK_ADJUSTMENT_SetOtherParameters();
				}

				is_adjustment_mode_on = 1;
			}

			// switch 2 is high and switch 3 is 3 save adjustment parameters
			else if(receiver_aux.aux3 == 3 && receiver_aux.aux2 == 2){

				// Save all parameters. Unchanged parameters will be rewritten as first read values.
				CK_ADJUSTMENT_SaveParameters();

				CK_BUZZER_Tone4();

				NVIC_SystemReset();
			}

		}
		else{
			is_adjustment_mode_on = 0;
		}

	#if defined(DEBUG_TIMING)
		adjustment_debug.update_time = CK_TIME_GetMicroSec() - adjustment_debug.start_time;
    #endif

	}

}

// Each parameter requires different amount of increment and decrement
// increment_multiplier will set it.
int CK_ADJUSTMENT_GetParameters(int increment_multiplier){

	int multiplier = 1;
	int adjust_num = 0;

	// Get configuration parameter
	int aux6_num = getRCDataRaw(AUX6) - rc_config.midrc;

	if(aux6_num < 0){
		multiplier = -1;
		aux6_num *= -1;
	}
	else{
		multiplier = 1;
	}

	if(aux6_num > 50 && aux6_num <= 125){

		adjust_num += 1 * increment_multiplier;
		adjust_num *= multiplier;
	}
	else if(aux6_num > 125 && aux6_num <= 300){

		adjust_num += 2 * increment_multiplier;
		adjust_num *= multiplier;
	}
	else if(aux6_num > 300){

		adjust_num += 5 * increment_multiplier;
		adjust_num *= multiplier;
	}

	return adjust_num;
}

void CK_ADJUSTMENT_SetPIDParameters(void){

	// Get the current pid number for selected axis
	int current_parameter = PID_PARAMETERS[receiver_aux.aux7 - 1][receiver_aux.aux8 - 1];

	int n1 = receiver_aux.aux7 - 1; // subtract 1 for array starting from 0
	int n2 = receiver_aux.aux8 - 1;

	// Add adjustment number to current pid parameter
	current_parameter += CK_ADJUSTMENT_GetParameters(1);

	// This is needed for saving parameters to eeprom
	PID_PARAMETERS[n1][n2] = current_parameter;

	pidUpdateParameter(current_parameter, n1, n2);

}

void CK_ADJUSTMENT_SetOtherParameters(void){

	int current_parameter = 0;

	// TPA Breakpoint
	if(receiver_aux.aux7 == 1 && receiver_aux.aux8 == 1){

		current_parameter = tpa_breakpoint_adjustment;

		// Add adjustment number to current
		current_parameter += CK_ADJUSTMENT_GetParameters(10);

		// This is needed for saving parameters to eeprom
		tpa_breakpoint_adjustment = current_parameter;

		pidUpdateTPAParameters(1, tpa_breakpoint_adjustment);
	}

	// TPA Rate
	else if(receiver_aux.aux7 == 1 && receiver_aux.aux8 == 2){

		current_parameter = tpa_rate_adjustment;

		// Add adjustment number to current
		current_parameter += CK_ADJUSTMENT_GetParameters(1);

		// This is needed for saving parameters to eeprom
		tpa_rate_adjustment = current_parameter;

		pidUpdateTPAParameters(2, tpa_rate_adjustment);
	}

	//
	else if(receiver_aux.aux7 == 2){

		current_parameter = PID_PARAMETERS[3][receiver_aux.aux8 - 1];

		//int n2 = receiver_aux.aux8 - 1;

		// Add adjustment number to current
		//current_parameter += CK_ADJUSTMENT_GetParameters(1);

		// This is needed for saving parameters to eeprom
		//PID_PARAMETERS[3][n2] = current_parameter;

		//CK_PID_UpdateParameter(current_parameter, AXIS_DMIN, n2);
	}

	// ANTIGRAVITY
	else if(receiver_aux.aux7 == 3){

	}

}

void CK_ADJUSTMENT_SetAntiGravityParameters(void){



}

void CK_ADJUSTMENT_SaveParameters(void){

	uint8_t config_buffer[EEPROM_BUFFER_SIZE];

	// Read the content of flash first to get parameters that are not changed.
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config_buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	// PID: Update axis parameters on its buffer location
	for(int r = 0; r < PID_ARRAY_ROW + 1; r++){

		for(int c = 0; c < PID_ARRAY_COLUMN; c++){

			config_buffer[CONFIG_PID_OFFSET + (r*PID_ARRAY_COLUMN) + c] = PID_PARAMETERS[r][c];
		}

	}

	// TPA Parameters are located starting from CONFIG_PID_OFFSET + NUM_OF_AXIS * NUM_OF_PID_COLUMN PID parameters
	config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN)] = (tpa_breakpoint_adjustment >> 8) & 0xFF;
	config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN + 1)] = tpa_breakpoint_adjustment & 0xFF;
	config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN + 2)] = tpa_rate_adjustment;

	// Antigravity


	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config_buffer, EEPROM_BUFFER_SIZE);

}

uint8_t CK_ADJUSTEMENT_IsAdjustmentModeOn(void){

	return is_adjustment_mode_on;
}























