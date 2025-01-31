
#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_LAND.h"


#define ARRAY_SIZE                  50 // 50Hz barometer update rate takes 1 sec to fill all.
#define GROUND_DEBOUNCE_BREAK       8
#define THROTTLE_DECREASE           3

#define ALTITUDE_FIX_THREASHOLD     15
#define MOTOR_STOP_SEC              2


int32_t altitudeArray[ARRAY_SIZE];
uint16_t arrayIndex = 0;

uint8_t motorStopCounter = 0;
int trimThrottle = 0;

bool landing_stopped    = false;
bool landing_reset_once = false;
bool ifLandingUsed      = false;



void CK_LAND_LandingHandle(uint32_t currentTime){

	static uint32_t previousTime = 0;
	float dTime = currentTime - previousTime;
	static float vel = 0;

	// Landing can be either selected with switch which sets LANDING_FLAG or
	// In Failsafe condition LANDING_FLAG will be automatically set in CK_RECEIVER_Update()
	if(flags.LANDING){

	    landing_reset_once = false;
	    ifLandingUsed = true;

	    if(!overwrite_flags.FORCE_TO_ANGLE_MODE || !overwrite_flags.FORCE_TO_GPS_POS_HOLD){

	    	// Forcing will be active as of next loop cycle until cancelation.
            overwrite_flags.FORCE_TO_ANGLE_MODE     = true;
            overwrite_flags.FORCE_TO_GPS_POS_HOLD   = true;
        }

	    // Calculate corrections at 50Hz
		if(dTime > TARGET_50HZ_US){

			previousTime = currentTime;

			vel = CK_ALTITUDE_GetFailsafeVelocity();

			//vel = CK_MATH_ApplyDeadband(vel, 10);

			if(vel > 0){

				// Ascending decrease throttle
				trimThrottle += THROTTLE_DECREASE * GROUND_DEBOUNCE_BREAK; // Decreased debouncing on ground
			}

			// Check if landed.
			CK_LAND_CheckLanding();

		}

		// Keep setting last calculated values since main loop is at 10KHz.
		int thr = 1500 + CK_ALTITUDE_GetThrottleAdjustment_Landing();

		thr -= trimThrottle;

		setRCCommand(THROTTLE, constrain(thr, RECEIVER_MIN_THROTTLE, RECEIVER_MAX_THROTTLE));

		if(landing_stopped){
		    if(!overwrite_flags.FORCE_TO_DISARM){
		        overwrite_flags.FORCE_TO_DISARM = true;
		    }
		}
    }
	else{
		/*
		 * After the start of landing i might decide not to land so in this condition
		 * if these parameters wont be resetted the drone will fall on second landing attempt
		 * Also landing pid i term errorVelocityI_land resetted
		 */
	    if(ifLandingUsed){
	        if(!landing_reset_once){
	            landing_reset_once = true;
	            ifLandingUsed = false;

	            overwrite_flags.FORCE_TO_ANGLE_MODE     = false;
	            overwrite_flags.FORCE_TO_GPS_POS_HOLD   = false;
	            overwrite_flags.FORCE_TO_DISARM         = false;

	            landing_stopped                         = false;

	            trimThrottle                            = 0;
	            motorStopCounter                        = 0;
	            arrayIndex                              = 0;
	        }
	    }
	}
}

/* Altitude check to decide if stopped or moving
 * Array has 50 elements which gives 1 seconds of comparison between first and last element
 * delay of altitude compansated in this way
 */

void CK_LAND_CheckLanding(void){

    if(arrayIndex < ARRAY_SIZE){
        altitudeArray[arrayIndex++] = CK_ALTITUDE_GetEstimatedAltitude();
    }
    else{

        if(ABS(altitudeArray[arrayIndex - 1] - altitudeArray[0]) < ALTITUDE_FIX_THREASHOLD){
            motorStopCounter++;

        }
        arrayIndex = 0;
    }

    if(motorStopCounter >= MOTOR_STOP_SEC){
        landing_stopped = true; // LANDED
    }
}

bool CK_LAND_IsLanded(void){
    return landing_stopped;
}


