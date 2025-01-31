
#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_GPS.h"

#include "MOTION/CK_BAROMETER.h"
#include "MOTION/CK_ACC.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define LANDING_DESCENDING_CM       110

typedef struct{

    int32_t estimatedAltitude;
    int32_t estimatedVelocity;

    float velocity_failsafe;
    float accVelocity;

    int32_t altHoldThrottleAdjustment;
    int32_t landingThrottleAdjustment;

    bool setAltitudeHold;

}altitude_variables_t;

altitude_variables_t altitude ={
    .estimatedAltitude = 0,
    .estimatedVelocity = 0,

    .velocity_failsafe = 0,
    .accVelocity = 0,

    .altHoldThrottleAdjustment = 0,
    .landingThrottleAdjustment = 0
};

DEBUG_TIME_t altitude_debug;

void CK_ALTITUDE_Update(uint32_t currentTime){

    // Gps height reading is not reliable do not directly fuse it.
    // Find a method to make it reliable than fuse it to final estimated altitude

    // If no barometer than only acc is used
    CK_ALTITUDE_CalculateVelocityAndAltitude(currentTime);

    // If no baro sensor it is not applied
    CK_ALTITUDE_ApplyAltitudeHoldAdjustment();
}

/*
 * Calculates altitude and vertical velocity(ascending descending velocity not ground speed velocity).
 * Altitude is calculated with barometer and acc in cm.
 * Than the gps altitude is fused to fused baro+acc altitude.
 */
void CK_ALTITUDE_CalculateVelocityAndAltitude(uint32_t currentTime){

	static uint32_t previousTime = 0;
	float dTime = currentTime - previousTime;

	static float velocity = 0.0f;
	static float accAltitude = 0.0f;

	#if defined(DEBUG_TIMING)
		altitude_debug.start_time = CK_TIME_GetMicroSec();
	#endif

#if USE_BARO_
	if(dTime > TARGET_50HZ_US && barometer.isBarometerReady){

	    previousTime = currentTime;
		float deltaT = dTime * 0.000001f;

		int32_t baroAlt = CK_BAROMETER_CalculateAltitude();
		altitude.estimatedAltitude = baroAlt;

#else
	if(dTime > TARGET_50HZ_US){
		previousTime = currentTime;
		float deltaT = dTime * 0.000001f;

		int32_t baroAlt = 0;
		altitude.estimatedAltitude = baroAlt;
#endif

		// Hold at a point where i enabled altitude hold switch
		if(flags.ALTITUDE_HOLD && !altitude.setAltitudeHold){

			// Set altitude hold level to current estimatedAltitude when it is enabled
			barometer.altitudeHold = altitude.estimatedAltitude;
			altitude.setAltitudeHold = true;
		}
		else if(!flags.ALTITUDE_HOLD && altitude.setAltitudeHold){
		    altitude.setAltitudeHold = false;
		}

		float accZ = 0.0f;
		if(acc.accADCEarthSumCounter){
			accZ = acc.accADCEarthSum[Z] / acc.accADCEarthSumCounter; // get accZ
			CK_ACC_ResetAccEarthSum();
		}

		// Calculate acc velocity and altitude
		//altitude.accVelocity = (accZ / acc.acc1G) * (9.80665f * 100) * deltaT; // velocity in cm/s

		// Upper line was working but now i am getting z offset so
		// rather than dividing to fix 2048 dividing to offset
		altitude.accVelocity = (accZ / (float)acc.accADCZero[Z]) * (9.80665f * 100) * deltaT; // velocity in cm/s

		accAltitude += altitude.accVelocity * deltaT; // distance in cm

		// Fuse acc altitude + baro altitude and hold result in accAltitude variable
		accAltitude = accAltitude * 0.965f + (float)baroAlt * (1.0f - 0.965f); // in cm

		velocity += altitude.accVelocity;

		altitude.velocity_failsafe = velocity;

		// Assign fused altitude result to estimatedAltitude variable
		altitude.estimatedAltitude = accAltitude;

#if GPS_
		// Fuse gps altitude and baro+acc's fused result
		//altitude.estimatedAltitude = (float)altitude.estimatedAltitude * 0.965f + (float)gps.heightDifference * (1.0f - 0.965f); // in cm
#endif
		// Calculate barometer velocity
		static int32_t lastBaroAlt = 0;
		int32_t baroVel = (baroAlt - lastBaroAlt) / deltaT; // v = x / t (cm/s)
		lastBaroAlt = baroAlt;
		baroVel = constrain(baroVel, -1500, 1500); // barometer velocity +/-1500cm
		applyDeadband(baroVel, 20);

		// Fuse acc velocity + baro velocity
		velocity = velocity * 0.985f + baroVel * (1.0f - 0.985f);

		// Assign fused velocity result to estimatedVelocity variable
		altitude.estimatedVelocity = lrintf(velocity);

	    static float accZ_old = 0.0f;

	    // This is for altitude hold
	    altitude.altHoldThrottleAdjustment = CK_ALTITUDE_CalculateThrottleAdjustment_AltitudeHold(altitude.estimatedVelocity, altitude.estimatedAltitude, accZ, accZ_old);

	    // This is for landing
	    altitude.landingThrottleAdjustment = CK_ALTITUDE_CalculateThrottleAdjustment_Landing(altitude.estimatedVelocity, accZ, accZ_old);

	    accZ_old = accZ;

        #if defined(DEBUG_TIMING)
	    altitude_debug.update_time = CK_TIME_GetMicroSec() - altitude_debug.start_time;
        #endif
	}

}

int32_t CK_ALTITUDE_CalculateThrottleAdjustment_AltitudeHold(int32_t estimatedVel, int32_t estimatedAlt, float accZ_tmp, float accZ_old){

    static int32_t errorVelocityI_alt = 0;

    if(!altitude.setAltitudeHold){
        return 0;
    }

    int32_t result = 0;
    int32_t error;
    int32_t setVel = 0;

    // Altitude P-Controller
    error = constrain(barometer.altitudeHold - estimatedAlt, -500, 500);
    error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position

    // altitude will have its own parameters which will include pid related parameters as well
    //setVel = CK_MATH_Constrain((PID_SELECTED_PROFILE[AXIS_ALTHOLD][P_Column] * error / 128), -300, +300); // limit velocity to +/- 3 m/s

    // Velocity PID-Controller
    // P
    error = setVel - estimatedVel;


    // altitude will have its own parameters which will include pid related parameters as well
    //result = CK_MATH_Constrain((PID_SELECTED_PROFILE[AXIS_VELOCITY][P_Column] * error / 32), -300, +300);

    // I

    // altitude will have its own parameters which will include pid related parameters as well
    //errorVelocityI_alt += (PID_SELECTED_PROFILE[AXIS_VELOCITY][I_Column] * error);
    errorVelocityI_alt = constrain(errorVelocityI_alt, -(8192 * 200), (8192 * 200));
    result += errorVelocityI_alt / 8192;     // I in range +/-200

    // D

    // altitude will have its own parameters which will include pid related parameters as well
    //result -= CK_MATH_Constrain(PID_SELECTED_PROFILE[AXIS_VELOCITY][D_Column] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}

int32_t CK_ALTITUDE_CalculateThrottleAdjustment_Landing(int32_t estimatedVel, float accZ_tmp, float accZ_old){

	/*
    int32_t result = 0;
    //int32_t error = 0;
    int32_t setVel;

    static int32_t errorVelocityI_land = 0;

    if(!flags.LANDING_FLAG){
        errorVelocityI_land = 0;
        return 0;
    }

    setVel = -LANDING_DESCENDING_CM; // 110 cm/s descending for landing

    // Velocity PID-Controller
    // P
    error = setVel - estimatedVel;

    // altitude will have its own parameters which will include pid related parameters as well
    //result = CK_MATH_Constrain((PID_SELECTED_PROFILE[AXIS_VELOCITY][P_Column] * error / 32), -300, +300);

    // I
    // altitude will have its own parameters which will include pid related parameters as well
    //errorVelocityI_land += (PID_SELECTED_PROFILE[AXIS_VELOCITY][I_Column] * error);
    errorVelocityI_land = CK_MATH_Constrain(errorVelocityI_land, -(8192 * 200), (8192 * 200));
    result += errorVelocityI_land / 8192;     // I in range +/-200

    // D
    // altitude will have its own parameters which will include pid related parameters as well
    //result -= CK_MATH_Constrain(PID_SELECTED_PROFILE[AXIS_VELOCITY][D_Column] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;

    */

	return 0;
}

void CK_ALTITUDE_ApplyAltitudeHoldAdjustment(void){

	if(flags.ALTITUDE_HOLD){

        int thr = getRCCommand(THROTTLE) + altitude.altHoldThrottleAdjustment;

        setRCCommand(THROTTLE, constrain(thr, RECEIVER_MIN_THROTTLE, RECEIVER_MAX_THROTTLE));
    }
}

int32_t CK_ALTITUDE_GetEstimatedAltitude(void){
	return altitude.estimatedAltitude;
}

int32_t CK_ALTITUDE_GetEstimatedVelocity(void){
	return altitude.estimatedVelocity;
}

int32_t CK_ALTITUDE_GetThrottleAdjustment_AltitudeHold(void){
	return altitude.altHoldThrottleAdjustment;
}

int32_t CK_ALTITUDE_GetThrottleAdjustment_Landing(void){
	return altitude.landingThrottleAdjustment;
}

float CK_ALTITUDE_GetAccVelocity(void){
	return altitude.accVelocity;
}

float CK_ALTITUDE_GetFailsafeVelocity(void){
	return altitude.velocity_failsafe;
}


