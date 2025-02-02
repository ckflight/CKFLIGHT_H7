
#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_ESC.h"
#include "FLIGHT/CK_LAND.h"
#include "FLIGHT/CK_DSHOT.h"
#include "FLIGHT/CK_PWM.h"

#include "COMMON/CK_FILTERS.h"

#include "MOTION/CK_GYRO.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

float throttle;

float motorOutputMin;
float motorOutputRange;

float motorRangeMin;
float motorRangeMax;

bool airmodeEnabled = false;

float motorFinalResult[MAX_SUPPORTED_MOTORS];

float motorMix[MAX_SUPPORTED_MOTORS];

static int throttleAngleCorrection;

static float rcThrottle = 0;

static float motorMixRange;

/*
 * These signs does not change the motor orientation and does not swap them when esc is mounted reverse
 * It assigns which motor is added decreased for roll pitch yaw
 * Esc orientation is corrected with esc swapping and remapping
 */

/*
 * QuadX 1234 configuration
 *
 * 1    2
 *
 * 4	3
 *
 */
float Mixer_Signs[MAX_SUPPORTED_MOTORS][FLIGHT_DYNAMICS_INDEX_COUNT] = {

		//ROLL   PITCH  YAW
	    { 1.0f, -1.0f, -1.0f },          // Motor 1 Front Left
		{-1.0f, -1.0f,  1.0f },          // Motor 2 Front Right
	    {-1.0f,  1.0f, -1.0f },          // Motor 3 Rare Right
	    { 1.0f,  1.0f,  1.0f }           // Motor 4 Rare Left

};

/*
 * QuadX configuration
 *
 * 4	2
 *
 * 3	1
 *
 */
float Mixer_Signs2[MAX_SUPPORTED_MOTORS][FLIGHT_DYNAMICS_INDEX_COUNT] = {

		//ROLL   PITCH  YAW
		{-1.0f,  1.0f, -1.0f },          // Motor 1 Rare Right
		{-1.0f, -1.0f,  1.0f },          // Motor 2 Front Right
		{ 1.0f,  1.0f,  1.0f },          // Motor 3 Rare Left
		{ 1.0f, -1.0f, -1.0f }           // Motor 4 Front Left

};

/*
 * QuadX reversed esc 180 configuration 3
 *
 * 4	2
 *
 * 3	1
 *
 */
float Mixer_Signs3[MAX_SUPPORTED_MOTORS][FLIGHT_DYNAMICS_INDEX_COUNT] = {

		//ROLL   PITCH  YAW
		{-1.0f,  1.0f,  1.0f },          // Motor 1 Rare Right
		{-1.0f, -1.0f, -1.0f },          // Motor 2 Front Right
		{ 1.0f,  1.0f, -1.0f },          // Motor 3 Rare Left
		{ 1.0f, -1.0f,  1.0f }           // Motor 4 Front Left
};


float Mixer_Signs_Current[MAX_SUPPORTED_MOTORS][FLIGHT_DYNAMICS_INDEX_COUNT];

DEBUG_TIME_t mixer_debug;

mixerRuntime_t mixerRuntime;

void CK_MIXER_Init(void){

	airmodeEnabled = true;

	mixerRuntime.motorCount = MAX_SUPPORTED_MOTORS;

	if (pidProfile.motor_output_limit > 100 || pidProfile.motor_output_limit == 0) {
		pidProfile.motor_output_limit = 100;
	}

	float motorOutputLimit = 1.0f;

	if (pidProfile.motor_output_limit < 100) {
		motorOutputLimit = pidProfile.motor_output_limit / 100.0f;
	}

	if(esc_mode == PWM_MODE){

		CK_PWM_InitEndPoints(motorOutputLimit, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput);
	}
	else if(esc_mode == DSHOT_MODE){

		CK_DSHOT_InitEndPoints(motorOutputLimit, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput);
	}

	for(int i = 0; i < MAX_SUPPORTED_MOTORS; i++){
		for(int j = 0; j < FLIGHT_DYNAMICS_INDEX_COUNT; j++){

#if MIXER_ORIENTATION == 1
			Mixer_Signs_Current[i][j] = Mixer_Signs1[i][j];
#elif MIXER_ORIENTATION == 2
			Mixer_Signs_Current[i][j] = Mixer_Signs2[i][j];
#elif MIXER_ORIENTATION == 3
			Mixer_Signs_Current[i][j] = Mixer_Signs3[i][j];
#endif

		}
	}

	mixerRuntime.mixer_type = MIXER_LEGACY;


	#ifdef USE_DYN_IDLE
	mixerRuntime.useDshotTelemetry = false;

    if (mixerRuntime.useDshotTelemetry) {
        mixerRuntime.dynIdleMinRps = pidProfile.dyn_idle_min_rpm * 100.0f / 60.0f;
    }
    else {
        mixerRuntime.dynIdleMinRps = 0.0f;
    }

    mixerRuntime.dynIdlePGain = pidProfile.dyn_idle_p_gain * 0.00015f;
    mixerRuntime.dynIdleIGain = pidProfile.dyn_idle_i_gain * 0.01f * pidGetDT();
    mixerRuntime.dynIdleDGain = pidProfile.dyn_idle_d_gain * 0.0000003f * pidGetPidFrequency();
    mixerRuntime.dynIdleMaxIncrease = pidProfile.dyn_idle_max_increase * 0.001f;
    mixerRuntime.minRpsDelayK = 800 * pidGetDT() / 20.0f; //approx 20ms D delay, arbitrarily suits many motors
    if (mixerRuntime.dynIdleMinRps) {
        mixerRuntime.motorOutputLow = DSHOT_MIN_THROTTLE; // Override value set by initEscEndpoints to allow zero motor drive
    }
	#endif

	#ifdef USE_DYN_IDLE
    mixerRuntime.idleThrottleOffset = CK_DSHOT_GetDigitalIdleOffset();
    mixerRuntime.dynIdleI = 0.0f;
    mixerRuntime.prevMinRps = 0.0f;
	#endif

    mixerRuntime.ezLandingThreshold = 2.0f * pidProfile.ez_landing_threshold / 100.0f;
	mixerRuntime.ezLandingLimit = pidProfile.ez_landing_limit / 100.0f;
	mixerRuntime.ezLandingSpeed = 2.0f * pidProfile.ez_landing_speed / 10.0f;

}

void CK_MIXER_Update(uint32_t currentTimeUs){

    #if defined(DEBUG_TIMING)
    mixer_debug.start_time = CK_TIME_GetMicroSec();
    #endif

    CK_MIXER_MixTable(currentTimeUs);

    CK_MIXER_ApplyMixToMotors();

    #if defined(DEBUG_TIMING)
    mixer_debug.update_time = CK_TIME_GetMicroSec() - mixer_debug.start_time;
    #endif

}

void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled)
{
    float airmodeTransitionPercent = 1.0f;
    float motorDeltaScale = 0.5f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
        motorDeltaScale *= airmodeTransitionPercent; // this should be half of the motor authority allowed
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    const float motorMixDelta = motorDeltaScale * motorMixRange;

    float minMotor = FLT_MAX;
    float maxMotor = FLT_MIN;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (mixerRuntime.mixer_type == MIXER_LINEAR) {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
        } else {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + fabsf(motorMix[i]), motorMix[i] - fabsf(motorMix[i]));
        }
        motorMix[i] *= motorMixNormalizationFactor;

        maxMotor = MAX(motorMix[i], maxMotor);
        minMotor = MIN(motorMix[i], minMotor);
    }

    // constrain throttle so it won't clip any outputs
    throttle = constrainf(throttle, -minMotor, 1.0f - maxMotor);
}

void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled)
{
	#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0.0f;
	#endif

    float airmodeTransitionPercent = 1.0f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= motorMixNormalizationFactor;
    }

    const float normalizedMotorMixMin = motorMixMin * motorMixNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * motorMixNormalizationFactor;
    throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

	#ifdef USE_AIRMODE_LPF
    airmodeThrottleChange = constrainf(unadjustedThrottle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax) - unadjustedThrottle;
    pidUpdateAirmodeLpf(airmodeThrottleChange);
	#endif
}

static float applyThrottleLimit(float throttle)
{
    if (rc_config.throttle_limit_percent < 100 && !RPM_LIMIT_ACTIVE) {
        const float throttleLimitFactor = rc_config.throttle_limit_percent / 100.0f;
        switch (rc_config.throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

void CK_MIXER_MixTable(uint32_t currentTimeUs){

	CK_MIXER_CalculateThrottleAndMotorRange();

	// FlipOverAfterCrash

	// Launch Control
	#ifdef USE_LAUNCH_CONTROL
    if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY)) {
        activeMixer = &mixerRuntime.launchControlMixer[0];
    }
	#endif

	const float scaledPidRollSum  = constrainf(pidData[FD_ROLL].Sum,  -pidProfile.pidSumLimit, pidProfile.pidSumLimit) / PID_MIXER_SCALING;
	const float scaledPidPitchSum = constrainf(pidData[FD_PITCH].Sum, -pidProfile.pidSumLimit, pidProfile.pidSumLimit) / PID_MIXER_SCALING;

	uint16_t yawPidSumLimit = pidProfile.pidSumLimitYaw;

	#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
	#endif

    float scaledPidYawSum = constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    //if (!mixerConfig()->yaw_motors_reversed) {
	//	scaledAxisPidYaw = -scaledAxisPidYaw;
	//}

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (rc_config.throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

	// use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
	pidUpdateAntiGravityThrottleFilter(throttle);

	pidUpdateTpaFactor(throttle);

	#ifdef USE_DYN_LPF
	// keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
	updateDynLpfCutoffs(currentTimeUs, throttle);
	#endif

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

	#ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = CK_MATH_MAX(throttle, 0.01f);
    }
	#endif

    // Thrust linearization
	#ifdef USE_THRUST_LINEARIZATION
	// reduce throttle to offset additional motor output
	throttle = pidCompensateThrustLinearization(throttle);
	#endif


	float motorMixMax = 0, motorMixMin = 0;

	for(int currentMotor = 0; currentMotor < mixerRuntime.motorCount; currentMotor++){

		float mix = scaledPidRollSum  * Mixer_Signs_Current[currentMotor][FD_ROLL]  +

				   	scaledPidPitchSum * Mixer_Signs_Current[currentMotor][FD_PITCH] +

				    scaledPidYawSum   * Mixer_Signs_Current[currentMotor][FD_YAW];

		if(mix > motorMixMax){
			motorMixMax = mix;
		}
		else if(mix < motorMixMin){
			motorMixMin = mix;
		}

		motorMix[currentMotor] = mix;
	}

	#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;
    }
	#endif

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (launchControlActive) {
        throttle = 0.0f;
    }
#endif


    motorMixRange = motorMixMax - motorMixMin;

    if (mixerRuntime.mixer_type > MIXER_LEGACY) {
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
    }
    else {
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
    }


    motorMixRange = motorMixMax - motorMixMin;
    if (mixerRuntime.mixer_type > MIXER_LEGACY) {
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
    }
    else {
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
    }

}

void CK_MIXER_ApplyMixToMotors(void){

	for(int currentMotor = 0; currentMotor < mixerRuntime.motorCount; currentMotor++){

		float motorOutput = motorMix[currentMotor] + throttle;

	#ifdef USE_THRUST_LINEARIZATION
			motorOutput = pidApplyThrustLinearization(motorOutput);
	#endif

		// else below
		motorOutput = motorOutputMin + motorOutputRange * motorOutput;



		motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);

		motorFinalResult[currentMotor] = motorOutput;

	}

	// DISARM HANDLE
	if(!flags.ARMED){

		for(int currentMotor = 0; currentMotor < mixerRuntime.motorCount; currentMotor++){

			motorFinalResult[currentMotor] = mixerRuntime.disarmMotorOutput;

			//If we do not set these to 0 when we armed again it continue from where it left
			//But setting them to 0 will make it like first time we start the code
			//it has no negative affect unless we disarm in the air.
			pidResetResultParameters();

		}
	}

	CK_MIXER_ApplyFinalToMotors();

}

void CK_MIXER_ApplyFinalToMotors(void){

	CK_ESC_SetMotor((int)motorFinalResult[0], (int)motorFinalResult[1], (int)motorFinalResult[2], (int)motorFinalResult[3]);

}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

void CK_MIXER_CalculateThrottleAndMotorRange(void){

	float currentThrottleRange = 0;
	static float motorRangeMinIncrease = 0;

	throttle = getRCCommand(THROTTLE) - PWM_RANGE_MIN + throttleAngleCorrection;

	currentThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;


	#ifdef USE_DYN_IDLE
	if (mixerRuntime.dynIdleMinRps > 0.0f) {
		const float maxIncrease = isAirmodeActivated() ? mixerRuntime.dynIdleMaxIncrease : 0.05f;

		//float minRps = getMinMotorFrequency();
		float minRps = 0;
		float rpsError = mixerRuntime.dynIdleMinRps - minRps;

		// PT1 type lowpass delay and smoothing for D
		minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);

		float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
		mixerRuntime.prevMinRps = minRps;

		float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
		rpsError = CK_MATH_MAX(-0.1f, rpsError); //I rises fast, falls slowly
		mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
		mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
		motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);
	}
	else {
		motorRangeMinIncrease = 0;
	}
	#endif

	motorRangeMax = mixerRuntime.motorOutputHigh;

	motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);

	motorOutputMin = motorRangeMin;

	motorOutputRange = motorRangeMax - motorRangeMin;


	throttle = constrainf(throttle / currentThrottleRange, 0.0f, 1.0f);
	rcThrottle = throttle;

}

float getMotorMixRange(void)
{
    return motorMixRange;
}

float mixerGetRcThrottle(void)
{
    return rcThrottle;
}

bool CK_MIXER_IsMixerSaturated(void){

	return motorMixRange > 1.0f;
}

int CK_MIXER_GetMotorFinalResult(int motorNumber){
	return (int)motorFinalResult[motorNumber - 1];
}

float CK_MIXER_GetMotorMixResult(int motorNumber){
	return motorMix[motorNumber - 1];
}

#ifdef USE_DYN_LPF
void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif
