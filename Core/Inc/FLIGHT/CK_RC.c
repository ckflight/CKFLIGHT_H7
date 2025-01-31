#include <COMMON/maths.h>
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_SBUS.h"
#include "FLIGHT/CK_CRSF.h"
#include "FLIGHT/CK_PID.h"

#include "MOTION/CK_GYRO.h"

#include "COMMON/CK_FILTERS.h"

#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/CK_PRINTER.h"

#include "FLASH/CK_FLASH.h"

#define SETPOINT_RATE_LIMIT 1998
#define RX_MID_USEC 1500
#define THROTTLE_LOOKUP_LENGTH 12

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_CUTOFF_MIN_HZ              15    // Minimum rc smoothing cutoff frequency
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_FEEDFORWARD_INITIAL_HZ     100   // The value to use for "auto" when interpolated feedforward is enabled

static rcSmoothingFilter_t rcSmoothingData;
static float rcDeflectionSmoothed[3];
static float maxRcRate[3];
#endif

#ifdef USE_FEEDFORWARD
static float feedforwardSmoothed[3];
static float feedforwardRaw[3];
static uint16_t feedforwardAveraging;
typedef struct laggedMovingAverageCombined_s {
    laggedMovingAverage_t filter;
    float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  feedforwardDeltaAvg[XYZ_AXIS_COUNT];
static pt1Filter_t feedforwardYawHoldLpf;
#endif

// Smooth
// 1.10 0.68 0.25 -> 688 degrees
// 1.10 0.67 0.25 -> 667 degrees
// 1.10 0.65 0.25 -> 629 degrees
// 1.10 0.64 0.25 -> 611 degrees

control_rate_config_t rc_config = {

		.deadband 				= 10,
		.yaw_deadband			= 50,  // yaw ignores movement smaller than deadband

		.midrc					= 1500,
#if defined(MIXER_ESC_REVERSED)
		.yaw_control_reversed	= true,
#else
		.yaw_control_reversed	= false,
#endif
		.rate_limit[ROLL] 		= SETPOINT_RATE_LIMIT,
		.rate_limit[PITCH] 		= SETPOINT_RATE_LIMIT,
		.rate_limit[YAW] 		= SETPOINT_RATE_LIMIT,

		.rcRates[ROLL] 			= 110, // divide 100 (110 means 1.10)
		.rcRates[PITCH] 		= 110,
		.rcRates[YAW] 			= 110,

		.rcExpo[ROLL]			= 20,   // divide 100
		.rcExpo[PITCH]			= 20,
		.rcExpo[YAW]			= 25,

		.rates[ROLL]			= 62,  // divide 100 (66 means 0.66)
		.rates[PITCH]			= 62,
		.rates[YAW]				= 60,

		.thrMid8 				= 50,
		.thrExpo8				= 0,

        .rc_smoothing_mode 					= 1,
        .rc_smoothing_setpoint_cutoff 		= 0,
        .rc_smoothing_feedforward_cutoff 	= 0,
        .rc_smoothing_throttle_cutoff 		= 0,
        .rc_smoothing_debug_axis 			= ROLL,
        .rc_smoothing_auto_factor_rpy 		= 30,
        .rc_smoothing_auto_factor_throttle 	= 30,

		.levelExpo[ROLL] 		= 0,
		.levelExpo[PITCH] 		= 0,

		.midrc					= RX_MID_USEC,

		.fpvCamAngleDegrees		= 25,

};

int rcDataRaw[TOTAL_CHANNEL_COUNT];
float rcCommandDelta[XYZ_AXIS_COUNT];
int rcCommandPrev[XYZ_AXIS_COUNT];
float rcCommand[XYZ_AXIS_COUNT + 1]; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

static float rawSetpoint[XYZ_AXIS_COUNT];
static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3]; // deflection range -1 to 1
static float maxRcDeflectionAbs;

float throttlePIDAttenuation;

static uint16_t currentRxIntervalUs;  // packet interval in microseconds, constrained to above range
static uint16_t previousRxIntervalUs; // previous packet interval in microseconds
static float currentRxRateHz;         // packet interval in Hz, constrained as above
static bool isRxRateValid = false;

uint32_t rc_mainTargetTime = 0;
static bool isRxDataNew = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

DEBUG_TIME_t rc_debug;
DEBUG_TIME_t rc_setpoint_debug;


void CK_RC_Init(uint32_t mainTargetTime){

	rc_mainTargetTime = mainTargetTime;

	CK_RC_LoadParameters();

	rcCommandDivider 	= 500.0f - rc_config.deadband;
	rcCommandYawDivider = 500.0f - rc_config.yaw_deadband;

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - rc_config.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - rc_config.thrMid8;
        if (tmp < 0)
            y = rc_config.thrMid8;
        lookupThrottleRC[i] = 10 * rc_config.thrMid8 + tmp * (100 - rc_config.thrExpo8 + (int32_t) rc_config.thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + PWM_RANGE * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

#ifdef USE_FEEDFORWARD
    feedforwardAveraging = pidRuntime.feedforwardAveraging;
    pt1FilterInit(&feedforwardYawHoldLpf, 0.0f);
#endif // USE_FEEDFORWARD

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        maxRcRate[i] = applyBetaflightRates(i, 1.0f, 1.0f);
#ifdef USE_FEEDFORWARD
        feedforwardSmoothed[i] = 0.0f;
        feedforwardRaw[i] = 0.0f;
        if (feedforwardAveraging) {
        	laggedMovingAverageInit(&feedforwardDeltaAvg[i].filter, feedforwardAveraging + 1, (float *)&feedforwardDeltaAvg[i].buf[0]);
        }
#endif // USE_FEEDFORWARD
    }

	#ifdef USE_YAW_SPIN_RECOVERY
    const int maxYawRate = (int)CK_RC_ApplyBetaflightRates(FD_YAW, 1.0f, 1.0f);
    initYawSpinRecovery(maxYawRate);
	#endif
}

uint16_t CK_RC_GetDefaultParameters(uint8_t* rc_buffer){

	uint16_t rc_buffer_size = 0;

	rc_buffer[rc_buffer_size++] = rc_config.deadband;
	rc_buffer[rc_buffer_size++] = rc_config.yaw_deadband;

	rc_buffer[rc_buffer_size++] = (rc_config.midrc >> 8) & 0xFF;
	rc_buffer[rc_buffer_size++] = rc_config.midrc & 0xFF;

	rc_buffer[rc_buffer_size++] = (rc_config.rate_limit[ROLL] >> 8) & 0xFF;
	rc_buffer[rc_buffer_size++] = rc_config.rate_limit[ROLL] & 0xFF;

	rc_buffer[rc_buffer_size++] = (rc_config.rate_limit[PITCH] >> 8) & 0xFF;
	rc_buffer[rc_buffer_size++] = rc_config.rate_limit[PITCH] & 0xFF;

	rc_buffer[rc_buffer_size++] = (rc_config.rate_limit[YAW] >> 8) & 0xFF;
	rc_buffer[rc_buffer_size++] = rc_config.rate_limit[YAW] & 0xFF;

	rc_buffer[rc_buffer_size++] = rc_config.rcRates[ROLL];
	rc_buffer[rc_buffer_size++] = rc_config.rcRates[PITCH];
	rc_buffer[rc_buffer_size++] = rc_config.rcRates[YAW];

	rc_buffer[rc_buffer_size++] = rc_config.rcExpo[ROLL];
	rc_buffer[rc_buffer_size++] = rc_config.rcExpo[PITCH];
	rc_buffer[rc_buffer_size++] = rc_config.rcExpo[YAW];

	rc_buffer[rc_buffer_size++] = rc_config.rates[ROLL];
	rc_buffer[rc_buffer_size++] = rc_config.rates[PITCH];
	rc_buffer[rc_buffer_size++] = rc_config.rates[YAW];

	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_mode;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_setpoint_cutoff;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_feedforward_cutoff;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_throttle_cutoff;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_debug_axis;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_auto_factor_rpy;
	rc_buffer[rc_buffer_size++] = rc_config.rc_smoothing_auto_factor_throttle;

	return rc_buffer_size;

}

void CK_RC_LoadParameters(void){

	uint8_t rc_buffer[CONFIG_RC_BYTES];

	/*
	 * RC directly reads and uses eeprom parameters.
	 * It is not responsible of checking if eeprom has correct parameters or not.
	 * CK_CONFIGURATOR_Init does that once after firmware flash.
	 */
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, rc_buffer, CONFIG_RC_BYTES, CONFIG_RC_OFFSET);

	rc_config.deadband 			= rc_buffer[0];
	rc_config.yaw_deadband 		= rc_buffer[1];

	rc_config.midrc 			= (rc_buffer[2] << 8) | rc_buffer[3];

	rc_config.rate_limit[ROLL] 	= (rc_buffer[4] << 8) | rc_buffer[5];
	rc_config.rate_limit[PITCH] = (rc_buffer[6] << 8) | rc_buffer[7];
	rc_config.rate_limit[YAW] 	= (rc_buffer[8] << 8) | rc_buffer[9];

	rc_config.rcRates[ROLL] 	= rc_buffer[10];
	rc_config.rcRates[PITCH] 	= rc_buffer[11];
	rc_config.rcRates[YAW] 		= rc_buffer[12];

	rc_config.rcExpo[ROLL] 		= rc_buffer[13];
	rc_config.rcExpo[PITCH] 	= rc_buffer[14];
	rc_config.rcExpo[YAW]		= rc_buffer[15];

	rc_config.rates[ROLL] 		= rc_buffer[16];
	rc_config.rates[PITCH] 		= rc_buffer[17];
	rc_config.rates[YAW]		= rc_buffer[18];

	rc_config.rc_smoothing_mode					= rc_buffer[19];
	rc_config.rc_smoothing_setpoint_cutoff		= rc_buffer[20];
	rc_config.rc_smoothing_feedforward_cutoff	= rc_buffer[21];
	rc_config.rc_smoothing_throttle_cutoff		= rc_buffer[22];
	rc_config.rc_smoothing_debug_axis			= rc_buffer[23];
	rc_config.rc_smoothing_auto_factor_rpy		= rc_buffer[24];
	rc_config.rc_smoothing_auto_factor_throttle	= rc_buffer[25];

}

void CK_RC_Update(void){

    #if defined(DEBUG_TIMING)
    rc_debug.start_time = CK_TIME_GetMicroSec();
    #endif

    isRxDataNew = isRxReceivingSignal();

    // 1. Main calls CK_RECEIVER_Update
    // Data is received and checked by receiver and channels flags are assigned and data is written to rcDataRaw
    // and refresh rate is updated with updateRcRefreshRate method
    // 2. Main calls CK_RC_Update
    // if a valid data is received then first rccommands are calculated and rccommands are processed
    if(isRxDataNew){
    	updateRcCommands();
    }

    // This function checks isRxDataNew itself.
    // Some part is not executed if no new data but some parts are executed
    processRcCommand();

    #if defined(DEBUG_TIMING)
    rc_debug.update_time = CK_TIME_GetMicroSec() - rc_debug.start_time;
    #endif

}

#ifdef USE_FEEDFORWARD
void calculateFeedforward(const pidRuntime_t *pid, flight_dynamics_index_t axis)
{
    const float rxInterval = currentRxIntervalUs * 1e-6f; // seconds
    float rxRate = currentRxRateHz;                 // 1e6f / currentRxIntervalUs;
    static float prevRcCommand[3];                  // for rcCommandDelta test
    static float prevRcCommandDeltaAbs[3];          // for duplicate interpolation
    static float prevSetpoint[3];                   // equals raw unless extrapolated forward
    static float prevSetpointSpeed[3];              // for setpointDelta calculation
    static float prevSetpointSpeedDelta[3];         // for duplicate extrapolation
    static bool isPrevPacketDuplicate[3];             // to identify multiple identical packets

    const float rcCommandDelta = rcCommand[axis] - prevRcCommand[axis];
    prevRcCommand[axis] = rcCommand[axis];
    float rcCommandDeltaAbs = fabsf(rcCommandDelta);

    const float setpoint = rawSetpoint[axis];
    const float setpointDelta = setpoint - prevSetpoint[axis];
    prevSetpoint[axis] = setpoint;

    float setpointSpeed = 0.0f;
    float setpointSpeedDelta = 0.0f;
    float feedforward = 0.0f;

    if (pid->feedforwardInterpolate) {
        static float prevRxInterval;
        // for Rx links which send frequent duplicate data packets, use a per-axis duplicate test
        // extrapolate setpointSpeed when a duplicate is detected, to minimise steps in feedforward
        const bool isDuplicate = rcCommandDeltaAbs == 0;
        if (!isDuplicate) {
            // movement!
            // but, if the packet before this was also a duplicate,
            // calculate setpointSpeed over the last two intervals
            if (isPrevPacketDuplicate[axis]) {
                rxRate = 1.0f / (rxInterval + prevRxInterval);
            }
            setpointSpeed = setpointDelta * rxRate;
            isPrevPacketDuplicate[axis] = isDuplicate;
        } else {
            // no movement
            if (!isPrevPacketDuplicate[axis]) {
                // extrapolate a replacement setpointSpeed value for the first duplicate after normal movement
                // but not when about to hit max deflection
                if (fabsf(setpoint) < 0.90f * maxRcRate[axis]) {
                    // this is a single packet duplicate, and we assume that it is of approximately normal duration
                    // hence no multiplication of prevSetpointSpeedDelta by rxInterval / prevRxInterval
                    setpointSpeed = prevSetpointSpeed[axis] + prevSetpointSpeedDelta[axis];
                    // pretend that there was stick movement also, to hold the same jitter value
                    rcCommandDeltaAbs = prevRcCommandDeltaAbs[axis];
                }
            } else {
                // for second and all subsequent duplicates...
                // force setpoint speed to zero
                setpointSpeed = 0.0f;
                // zero the acceleration by setting previous speed to zero
                // feedforward will smoothly decay and be attenuated by the jitter reduction value for zero rcCommandDelta
                prevSetpointSpeed[axis] = 0.0f; // zero acceleration later on
            }
            isPrevPacketDuplicate[axis] = isDuplicate;
        }
        prevRxInterval = rxInterval;
    } else {
        // don't interpolate for radio systems that rarely send duplicate packets, eg CRSF/ELRS
        setpointSpeed = setpointDelta * rxRate;
    }

    // calculate jitterAttenuation factor
    // The intent is to attenuate feedforward when absolute rcCommandDelta is small, ie when sticks move very slowly
    // Greater feedforward_jitter_factor values widen the attenuation range, and increase the suppression at center
    // Stick input is the average of the previous two absolute rcCommandDelta values
    // Output is jitterAttenuator, a value 0-1.0 that is a simple multiplier of the final feedforward value
    // For the CLI user setting of feedforward_jitter_factor:
    // User setting of 0 returns feedforwardJitterFactorInv = 1.0 (and disables the function)
    // User setting of 1 returns feedforwardJitterFactorInv = 0.5
    // User setting of 9 returns feedforwardJitterFactorInv = 0.1
    // rcCommandDelta has 500 unit values either side of center stick position
    // For a 250Hz link, a one second stick sweep center->max returns rcCommandDelta around 2.0
    // For a user jitter reduction setting of 2, the jitterAttenuator value ranges linearly
    // from 0.33 when rcCommandDelta is close to zero, up to 1.0 for rcCommandDelta of 2.0 or more
    // For a user jitter reduction setting of 9, the jitterAttenuator value ranges linearly
    // from 0.1 when rcCommandDelta is close to zero, up to 1.0 for rcCommandDelta is 9.0 or more
    // note that the jitter reduction multiplies the final smoothed value of feedforward
    // allowing residual smooth feedforward offsets even if the sticks are not moving
    // this is an improvement on the previous version which 'chopped' FF to zero when sticks stopped moving
    float jitterAttenuator = ((rcCommandDeltaAbs + prevRcCommandDeltaAbs[axis]) * 0.5f + 1.0f) * pid->feedforwardJitterFactorInv;
    jitterAttenuator = MIN(jitterAttenuator, 1.0f);
    prevRcCommandDeltaAbs[axis] = rcCommandDeltaAbs;

    // smooth the setpointSpeed value
    setpointSpeed = prevSetpointSpeed[axis] + pid->feedforwardSmoothFactor * (setpointSpeed - prevSetpointSpeed[axis]);

    // calculate setpointDelta from smoothed setpoint speed
    setpointSpeedDelta = setpointSpeed - prevSetpointSpeed[axis];
    prevSetpointSpeed[axis] = setpointSpeed;

    // smooth the setpointDelta element (effectively a second order filter since incoming setpoint was already smoothed)
    setpointSpeedDelta = prevSetpointSpeedDelta[axis] + pid->feedforwardSmoothFactor * (setpointSpeedDelta - prevSetpointSpeedDelta[axis]);
    prevSetpointSpeedDelta[axis] = setpointSpeedDelta;

    // apply gain factor to delta and adjust for rxRate
    const float feedforwardBoost = setpointSpeedDelta * rxRate * pid->feedforwardBoostFactor;

    feedforward = setpointSpeed;

    if (axis == FD_ROLL || axis == FD_PITCH) {
        // for pitch and roll, add feedforwardBoost to deal with motor lag
        feedforward += feedforwardBoost;
        // apply jitter reduction multiplier to reduce noise by attenuating when sticks move slowly
        feedforward *= jitterAttenuator;
        // pull feedforward back towards zero as sticks approach max if in same direction
        // to avoid overshooting on the outwards leg of a fast roll or flip
        if (pid->feedforwardMaxRateLimit && feedforward * setpoint > 0.0f) {
            const float limit = (maxRcRate[axis] - fabsf(setpoint)) * pid->feedforwardMaxRateLimit;
            feedforward = (limit > 0.0f) ? constrainf(feedforward, -limit, limit) : 0.0f;
        }

    } else {
        // for yaw, apply jitter reduction only to the base feedforward delta element
        // can't be applied to the 'sustained' element or jitter values will divide it down too much when sticks are still
        feedforward *= jitterAttenuator;

        // instead of adding setpoint acceleration, which is too aggressive for yaw,
        // add a slow-fading high-pass filtered setpoint element
        // this provides a 'sustained boost' with low noise
        // it mimics the normal sustained yaw motor drive requirements, reducing P and I and hence reducing bounceback
        // this doesn't add significant noise to feedforward
        // too little yaw FF causes iTerm windup and slow bounce back when stopping a hard yaw
        // too much causes fast bounce back when stopping a hard yaw

        // calculate lowpass filter gain factor from user specified time constant
        const float gain = pt1FilterGainFromDelay(pid->feedforwardYawHoldTime, rxInterval);
        pt1FilterUpdateCutoff(&feedforwardYawHoldLpf, gain);
        const float setpointLpfYaw = pt1FilterApply(&feedforwardYawHoldLpf, setpoint);
        // subtract lowpass from input to get highpass of setpoint for sustained yaw 'boost'
        const float feedforwardYawHold = pid->feedforwardYawHoldGain * (setpoint - setpointLpfYaw);

        //DEBUG_SET(DEBUG_FEEDFORWARD, 6, lrintf(feedforward * 0.01f));  // basic yaw feedforward without hold element
        //DEBUG_SET(DEBUG_FEEDFORWARD, 7, lrintf(feedforwardYawHold * 0.01f));  // yaw feedforward hold element

        feedforward += feedforwardYawHold;
        // NB: yaw doesn't need max rate limiting since it rarely overshoots
    }

    // apply feedforward transition, if configured. Archaic (better to use jitter reduction)
    const bool useTransition = (pid->feedforwardTransition != 0.0f) && (rcDeflectionAbs[axis] < pid->feedforwardTransition);
    if (useTransition) {
        feedforward *= rcDeflectionAbs[axis] * pid->feedforwardTransitionInv;
    }

    //if (axis == gyro.gyroDebugAxis) {
        //DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint));                       // un-smoothed (raw) setpoint value used for FF
        //DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(setpointSpeed * 0.01f));          // smoothed and extrapolated basic feedfoward element
        //DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(feedforwardBoost * 0.01f));       // acceleration (boost) smoothed
        //DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(rcCommandDelta * 10.0f));
        //DEBUG_SET(DEBUG_FEEDFORWARD, 4, lrintf(jitterAttenuator * 100.0f));      // jitter attenuation percent
        //DEBUG_SET(DEBUG_FEEDFORWARD, 5, (int16_t)(isPrevPacketDuplicate[axis]));   // previous packet was a duplicate

        //DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, lrintf(jitterAttenuator * 100.0f)); // jitter attenuation factor in percent
        //DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, lrintf(maxRcRate[axis]));           // max Setpoint rate (badly named)
        //DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, lrintf(setpoint));                  // setpoint used for FF
        //DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 3, lrintf(feedforward * 0.01f));       // un-smoothed final feedforward
    //}

    // apply averaging to final values, for additional smoothing if needed; not shown in logs
    if (feedforwardAveraging) {
        feedforward = laggedMovingAverageUpdate(&feedforwardDeltaAvg[axis].filter, feedforward);
    }

    feedforwardRaw[axis] = feedforward;
}
#endif // USE_FEEDFORWARD

void processRcCommand(void)
{
    if (isRxDataNew) {
        maxRcDeflectionAbs = 0.0f;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

            float angleRate;

#ifdef USE_GPS_RESCUE
            if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
                // If GPS Rescue is active then override the setpointRate used in the
                // pid controller with the value calculated from the desired heading logic.
                angleRate = gpsRescueGetYawRate();
                // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
                rcDeflection[axis] = 0;
                rcDeflectionAbs[axis] = 0;
            } else
#endif
            {
                // scale rcCommandf to range [-1.0, 1.0]
                float rcCommandf;
                if (axis == FD_YAW) {
                    rcCommandf = rcCommand[axis] / rcCommandYawDivider;
                } else {
                    rcCommandf = rcCommand[axis] / rcCommandDivider;
                }
                rcDeflection[axis] = rcCommandf;
                const float rcCommandfAbs = fabsf(rcCommandf);
                rcDeflectionAbs[axis] = rcCommandfAbs;
                maxRcDeflectionAbs = fmaxf(maxRcDeflectionAbs, rcCommandfAbs);

                angleRate = applyBetaflightRates(axis, rcCommandf, rcCommandfAbs);
            }

            rawSetpoint[axis] = constrainf(angleRate, -1.0f * CONTROL_RATE_CONFIG_RATE_LIMIT_MAX, 1.0f * CONTROL_RATE_CONFIG_RATE_LIMIT_MAX);
            //DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);

#ifdef USE_FEEDFORWARD
        calculateFeedforward(&pidRuntime, axis);
#endif // USE_FEEDFORWARD

        }
        // adjust unfiltered setpoint steps to camera angle (mixing Roll and Yaw)
        //if (rc_config.fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !flags.HEADFREE_MODE) {
        //    scaleRawSetpointToFpvCamAngle();
        //}
    }

#ifdef USE_RC_SMOOTHING_FILTER
    processRcSmoothingFilter();
#endif

    isRxDataNew = false;
}

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

void updateRcCommands(void)
{

    for (int axis = 0; axis < 3; axis++) {
        float rc = constrainf(rcDataRaw[axis] - rc_config.midrc, -500.0f, 500.0f); // -500 to 500
        float rcDeadband = 0;
        if (axis == ROLL || axis == PITCH) {
            rcDeadband = rc_config.deadband;
        }
        else {
            rcDeadband  = rc_config.yaw_deadband;
            rc *= -GET_DIRECTION(rc_config.yaw_control_reversed);
        }
        rcCommand[axis] = fapplyDeadband(rc, rcDeadband);
    }

    int32_t tmp;
    tmp = constrain(rcDataRaw[THROTTLE], PWM_RANGE_MIN_CHECK, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - PWM_RANGE_MIN_CHECK) * PWM_RANGE_MIN / (PWM_RANGE_MAX - PWM_RANGE_MIN_CHECK);

    //if (getLowVoltageCutoff()->enabled) {
    //    tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    //}

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

}

// receiver calls this method and updates
void updateRcRefreshRate(timeUs_t currentTimeUs, bool rxReceivingSignal)
{
    // rxReceivingSignal is true:
    // - every time a new frame is detected,
    // - if we stop getting data, at the expiry of RXLOSS_TRIGGER_INTERVAL since the last good frame
    // - if that interval is exceeded and still no data, every RX_FRAME_RECHECK_INTERVAL, until a new frame is detected
    static timeUs_t lastRxTimeUs = 0;
    timeDelta_t delta = 0;

    if (rxReceivingSignal) { // true while receiving data and until RXLOSS_TRIGGER_INTERVAL expires, otherwise false
        previousRxIntervalUs = currentRxIntervalUs;
        // use driver rx time if available, current time otherwise
        const timeUs_t rxTime = currentTimeUs; // i use current time directly

        if (lastRxTimeUs) {  // report delta only if previous time is available
            delta = cmpTimeUs(rxTime, lastRxTimeUs);
        }
        lastRxTimeUs = rxTime;
        //DEBUG_SET(DEBUG_RX_TIMING, 1, rxTime / 100);   // output value in tenths of ms
    } else {
        if (lastRxTimeUs) {
            // no packet received, use current time for delta
            delta = cmpTimeUs(currentTimeUs, lastRxTimeUs);
        }
    }

    // temporary debugs
    //DEBUG_SET(DEBUG_RX_TIMING, 4, MIN(delta / 10, INT16_MAX));   // time between frames based on rxFrameCheck
#ifdef USE_RX_LINK_QUALITY_INFO
    //DEBUG_SET(DEBUG_RX_TIMING, 6, rxGetLinkQualityPercent());    // raw link quality value
#endif
    //DEBUG_SET(DEBUG_RX_TIMING, 7, isRxReceivingSignal());        // flag to initiate RXLOSS signal and Stage 1 values

    // constrain to a frequency range no lower than about 15Hz and up to about 1000Hz
    // these intervals and rates will be used for RCSmoothing, Feedforward, etc.
    currentRxIntervalUs = constrain(delta, RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US);
    currentRxRateHz = 1e6f / currentRxIntervalUs;
    isRxRateValid = delta == currentRxIntervalUs; // delta is not constrained, therefore not outside limits

    //DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(delta / 10, INT16_MAX));   // output value in hundredths of ms
    //DEBUG_SET(DEBUG_RX_TIMING, 2, isRxRateValid);
    //DEBUG_SET(DEBUG_RX_TIMING, 3, MIN(currentRxIntervalUs / 10, INT16_MAX));
}

/*
 * This method calculates the set points
 * for Roll, Pitch and Yaw Channels
 * SuperRate and Expo curve contribution
 * is calculated here.
 */
void calculateRCSetpoint(void){

    #if defined(DEBUG_TIMING)
    rc_setpoint_debug.start_time = CK_TIME_GetMicroSec();
    #endif

	for(int axis = ROLL; axis <= YAW; axis++){

		rcCommandDelta[axis] = fabsf(rcCommand[axis] - rcCommandPrev[axis]);
		rcCommandPrev[axis] = rcCommand[axis];

		float angleRate;

		float rcCommandf = rcCommand[axis] / 500.0f; // divide to 500 - deadband!!! and - yawdeadband
		rcDeflection[axis] = rcCommandf;
		const float rcCommandfAbs = fabsf(rcCommandf);
		rcDeflectionAbs[axis] = rcCommandfAbs;

		angleRate = applyBetaflightRates(axis, rcCommandf, rcCommandfAbs);

		// Raw setpoint
		rawSetpoint[axis] = constrainf(angleRate, -rc_config.rate_limit[axis], rc_config.rate_limit[axis]);

	}

    #if defined(DEBUG_TIMING)
    rc_setpoint_debug.update_time = CK_TIME_GetMicroSec() - rc_setpoint_debug.start_time;
    #endif
}

#define RC_RATE_INCREMENTAL 14.54f

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs){

    if (rc_config.rcExpo[axis]) {
        const float expof = rc_config.rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = rc_config.rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }

    float angleRate = 200.0f * rcRate * rcCommandf;
    if (rc_config.rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (rc_config.rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

#ifdef USE_RC_SMOOTHING_FILTER

// Determine a cutoff frequency based on smoothness factor and calculated average rx frame time
int calcAutoSmoothingCutoff(int avgRxFrameTimeUs, uint8_t autoSmoothnessFactor){

	if (avgRxFrameTimeUs > 0) {
		const float cutoffFactor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
		float cutoff = (1 / (avgRxFrameTimeUs * 1e-6f));  // link frequency
		cutoff = cutoff * cutoffFactor;
		return lrintf(cutoff);
	}
	else{
		return 0;
	}
}

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    // in auto mode, calculate the RC smoothing cutoff from the smoothed Rx link frequency
    const uint16_t oldSetpointCutoff = smoothingData->setpointCutoffFrequency;
    const uint16_t oldFeedforwardCutoff = smoothingData->feedforwardCutoffFrequency;
    const uint16_t minCutoffHz = 15; // don't let any RC smoothing filter cutoff go below 15Hz
    if (smoothingData->setpointCutoffSetting == 0) {
        smoothingData->setpointCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorSetpoint));
    }
    if (smoothingData->throttleCutoffSetting == 0) {
        smoothingData->throttleCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorThrottle));
    }

    if (smoothingData->feedforwardCutoffSetting == 0) {
        smoothingData->feedforwardCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorFeedforward));
    }

    const float dT = targetPidLooptime * 1e-6f;
    if ((smoothingData->setpointCutoffFrequency != oldSetpointCutoff) || !smoothingData->filterInitialized) {
        // note that cutoff frequencies are integers, filter cutoffs won't re-calculate until there is > 1hz variation from previous cutoff
        // initialize or update the setpoint cutoff based filters
        const float setpointCutoffFrequency = smoothingData->setpointCutoffFrequency;
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if (i < THROTTLE) {
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                }
            } else {
                const float throttleCutoffFrequency = smoothingData->throttleCutoffFrequency;
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                }
            }
        }
        // initialize or update the RC Deflection filter
        for (int i = FD_ROLL; i < FD_YAW; i++) {
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            }
        }
    }
    // initialize or update the Feedforward filter
    if ((smoothingData->feedforwardCutoffFrequency != oldFeedforwardCutoff) || !smoothingData->filterInitialized) {
       for (int i = FD_ROLL; i <= FD_YAW; i++) {
            const float feedforwardCutoffFrequency = smoothingData->feedforwardCutoffFrequency;
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            }
        }
    }

    //DEBUG_SET(DEBUG_RC_SMOOTHING, 1, smoothingData->setpointCutoffFrequency);
    //DEBUG_SET(DEBUG_RC_SMOOTHING, 2, smoothingData->feedforwardCutoffFrequency);
}


// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely
bool rcSmoothingAutoCalculate(void)
{
    // if any rc smoothing cutoff is 0 (auto) then we need to calculate cutoffs
    if ((rcSmoothingData.setpointCutoffSetting == 0) || (rcSmoothingData.feedforwardCutoffSetting == 0) || (rcSmoothingData.throttleCutoffSetting == 0)) {
        return true;
    }
    return false;
}

void processRcSmoothingFilter(void)
{
    static float rxDataToSmooth[4];
    static bool initialized;
    static bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.smoothedRxRateHz = 0.0f;
        rcSmoothingData.sampleCount = 0;
        rcSmoothingData.debugAxis = rc_config.rc_smoothing_debug_axis;

        rcSmoothingData.autoSmoothnessFactorSetpoint = 1.5f / (1.0f + (rc_config.rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorFeedforward = 1.5f / (1.0f + (rc_config.rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorThrottle = 1.5f / (1.0f + (rc_config.rc_smoothing_auto_factor_throttle / 10.0f));

        rcSmoothingData.setpointCutoffSetting = rc_config.rc_smoothing_setpoint_cutoff;
        rcSmoothingData.throttleCutoffSetting = rc_config.rc_smoothing_throttle_cutoff;
        rcSmoothingData.feedforwardCutoffSetting = rc_config.rc_smoothing_feedforward_cutoff;

        rcSmoothingData.setpointCutoffFrequency = rcSmoothingData.setpointCutoffSetting;
        rcSmoothingData.feedforwardCutoffFrequency = rcSmoothingData.feedforwardCutoffSetting;
        rcSmoothingData.throttleCutoffFrequency = rcSmoothingData.throttleCutoffSetting;

        if (rc_config.rc_smoothing_mode) {
            calculateCutoffs = rcSmoothingAutoCalculate();
            // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
            if (!calculateCutoffs) {
                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                rcSmoothingData.filterInitialized = true;
            }
        }
    }

    if (isRxDataNew) {
        if (calculateCutoffs) {
            // for auto calculated filters, calculate the link interval and update the RC smoothing filters at regular intervals
            // this is more efficient than monitoring for significant changes and making comparisons to decide whether to update the filter
            const timeMs_t currentTimeMs = CK_TIME_GetMilliSec();
            int sampleState = 0;
            const bool ready = (currentTimeMs > 1000) && (targetPidLooptime > 0);
            if (ready) { // skip during FC initialization
                // Wait 1000ms after power to let the PID loop stabilize before starting average frame rate calculation
                if (isRxReceivingSignal() && isRxRateValid) {

                    if (abs(currentRxIntervalUs - previousRxIntervalUs) < (previousRxIntervalUs - (previousRxIntervalUs / 8))) {
                        // exclude large steps, eg after dropouts or telemetry
                        // by using interval here, we catch a dropout/telemetry where the inteval increases by 100%, but accept
                        // the return to normal value, which is only 50% different from the 100% interval of a single drop, and 66% of a return after a double drop.
                        static float prevSmoothedRxRateHz;
                        // smooth the current Rx link frequency estimates
                        const float kF = 0.1f; // first order kind of lowpass smoothing filter coefficient
                        // add one tenth of the new estimate to the smoothed estimate.
                        const float smoothedRxRateHz = prevSmoothedRxRateHz + kF * (currentRxRateHz - prevSmoothedRxRateHz);
                        prevSmoothedRxRateHz = smoothedRxRateHz;

                        // recalculate cutoffs every 3 acceptable samples
                        if (rcSmoothingData.sampleCount) {
                            rcSmoothingData.sampleCount --;
                            sampleState = 1;
                        } else {
                            rcSmoothingData.smoothedRxRateHz = smoothedRxRateHz;
                            rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                            rcSmoothingData.filterInitialized = true;
                            rcSmoothingData.sampleCount = 3;
                            sampleState = 2;
                        }
                    }
                } else {
                    // either we stopped receiving rx samples (failsafe?) or the sample interval is unreasonable
                    // require a full re-evaluation period after signal is restored
                    rcSmoothingData.sampleCount = 0;
                    sampleState = 4;
                }
            }
            //DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxIntervalUs / 10);
            //DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.sampleCount);
            //DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.smoothedRxRateHz); // value used by filters
            //DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState); // guard time = 1, guard time expired = 2
            UNUSED(sampleState);
        }
        // Get new values to be smoothed
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            rxDataToSmooth[i] = i == THROTTLE ? rcCommand[i] : rawSetpoint[i];
            if (i < THROTTLE) {
                //DEBUG_SET(DEBUG_RC_INTERPOLATION, i, lrintf(rxDataToSmooth[i]));
            } else {
                //DEBUG_SET(DEBUG_RC_INTERPOLATION, i, ((lrintf(rxDataToSmooth[i])) - 1000));
            }
        }
    }

    //DEBUG_SET(DEBUG_RC_SMOOTHING, 0, rcSmoothingData.smoothedRxRateHz);
    //DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.sampleCount);

    // each pid loop, apply the last received channel value to the filter, if initialised - thanks @klutvott
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
        float *dst = i == THROTTLE ? &rcCommand[i] : &setpointRate[i];
        if (rcSmoothingData.filterInitialized) {
            *dst = pt3FilterApply(&rcSmoothingData.filterSetpoint[i], rxDataToSmooth[i]);
        } else {
            // If filter isn't initialized yet, as in smoothing off, use the actual unsmoothed rx channel data
            *dst = rxDataToSmooth[i];
        }
    }

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // Feedforward smoothing
        feedforwardSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterFeedforward[axis], feedforwardRaw[axis]);
        // Horizon mode smoothing of rcDeflection on pitch and roll to provide a smooth angle element
        const bool smoothRcDeflection = flags.HORIZON_MODE && rcSmoothingData.filterInitialized;
        if (smoothRcDeflection && axis < FD_YAW) {
            rcDeflectionSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterRcDeflection[axis], rcDeflection[axis]);
        } else {
            rcDeflectionSmoothed[axis] = rcDeflection[axis];
        }
    }
}

#endif

int getRCDataRaw(int axis){
	return rcDataRaw[axis];
}

void setRCDataRaw(int axis, int num){
	rcDataRaw[axis] = num;
}

float getRCCommand(int axis){
	return rcCommand[axis];
}

void setRCCommand(int axis, int num){
	rcCommand[axis] = num;
}

float getMaxRcRate(int axis)
{
    return maxRcRate[axis];
}

float getRcDeflection(int axis){

#ifdef USE_RC_SMOOTHING_FILTER
    return rcDeflectionSmoothed[axis];
#else
    return rcDeflection[axis];
#endif

}

#ifdef USE_FEEDFORWARD
float getFeedforward(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return feedforwardSmoothed[axis];
#else
    return feedforwardRaw[axis];
#endif
}
#endif // USE_FEEDFORWARD

void CK_RC_SetRcDeflection(int axis, float num){

    rcDeflection[axis] = num;
}

float getRcDeflectionAbs(int axis){

    return rcDeflectionAbs[axis];
}

void setRcDeflectionAbs(int axis, float num){

    rcDeflectionAbs[axis] = num;
}

float getMaxRcDeflectionAbs(void)
{
    return maxRcDeflectionAbs;
}

float getSetpointRate(int axis){

#ifdef USE_RC_SMOOTHING_FILTER
	return setpointRate[axis];
#else
    return rawSetpoint[axis];
#endif
}

float getRcCommandDelta(int axis){

    return rcCommandDelta[axis];
}

float getCurrentRxRefreshRate(void){

    return currentRxRateHz;
}











