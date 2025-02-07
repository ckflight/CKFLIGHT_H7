
#ifndef CK_RC_H_
#define CK_RC_H_

#include "CK_DEFINITIONS.h"
#include "COMMON/CK_FILTERS.h"
#include "COMMON/axis.h"

#define RX_INTERVAL_MIN_US     950 // 0.950ms to fit 1kHz without an issue
#define RX_INTERVAL_MAX_US   65500 // 65.5ms or 15.26hz

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12_RSSI
} rc_alias_e;

#define PRIMARY_CHANNEL_COUNT (THROTTLE + 1)

#define TOTAL_CHANNEL_COUNT (AUX12_RSSI + 1)

typedef enum {
    THROTTLE_LIMIT_TYPE_OFF = 0,
    THROTTLE_LIMIT_TYPE_SCALE,
    THROTTLE_LIMIT_TYPE_CLIP,
    THROTTLE_LIMIT_TYPE_COUNT   // must be the last entry
} throttleLimitType_e;

typedef struct{

	uint8_t deadband;
	uint8_t yaw_deadband;
	uint16_t midrc;                            // Some radios have not a neutral point centered on 1500. can be changed here

	bool yaw_control_reversed;

	uint16_t rate_limit[3];

	uint8_t rcExpo[3]; 							// stick sensitivity around middle
	uint8_t rcRates[3]; 						// general sensitivity of stick (higher increases response)
	uint8_t rates[3];							// stick sensitivity around end point

	uint8_t thrMid8;
	uint8_t thrExpo8;

    uint8_t rc_smoothing_mode;                 	// Whether filter based rc smoothing is on or off
    uint8_t rc_smoothing_setpoint_cutoff;      	// Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_feedforward_cutoff;   	// Filter cutoff frequency for the feedforward filter (0 = auto)
    uint8_t rc_smoothing_throttle_cutoff;      	// Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_debug_axis;           	// Axis to log as debug values when debug_mode = RC_SMOOTHING
    uint8_t rc_smoothing_auto_factor_rpy;      	// Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rc_smoothing_auto_factor_throttle; 	// Used to adjust the "smoothness" determined by the auto cutoff calculations

    uint8_t levelExpo[2];                   	// roll/pitch level mode expo

    uint8_t fpvCamAngleDegrees;                	// Camera angle to be scaled into rc commands

    uint8_t throttle_limit_type;            	// Sets the throttle limiting type - off, scale or clip
    uint8_t throttle_limit_percent;         	// Sets the maximum pilot commanded throttle limit

}control_rate_config_t;

typedef struct rcSmoothingFilterTraining_s {
    float sum;
    int count;
    uint16_t min;
    uint16_t max;
} rcSmoothingFilterTraining_t;

typedef struct rcSmoothingFilter_s {
    bool filterInitialized;
    pt3Filter_t filterSetpoint[4];
    pt3Filter_t filterRcDeflection[RP_AXIS_COUNT];
    pt3Filter_t filterFeedforward[3];

    uint8_t setpointCutoffSetting;
    uint8_t throttleCutoffSetting;
    uint8_t feedforwardCutoffSetting;

    uint16_t setpointCutoffFrequency;
    uint16_t throttleCutoffFrequency;
    uint16_t feedforwardCutoffFrequency;

    float smoothedRxRateHz;
    uint8_t sampleCount;
    uint8_t debugAxis;

    float autoSmoothnessFactorSetpoint;
    float autoSmoothnessFactorFeedforward;
    float autoSmoothnessFactorThrottle;
} rcSmoothingFilter_t;

extern control_rate_config_t rc_config;

void CK_RC_Init(uint32_t mainTargetTime);

uint16_t CK_RC_GetDefaultParameters(uint8_t* rc_buffer);

void CK_RC_LoadParameters(void);

void CK_RC_Update(void);

void processRcCommand(void);

void updateRcCommands(void);

void updateRcRefreshRate(timeUs_t currentTimeUs, bool rxReceivingSignal);

void calculateRCSetpoint(void);

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs);

int calcAutoSmoothingCutoff(int avgRxFrameTimeUs, uint8_t autoSmoothnessFactor);

void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData);

bool rcSmoothingAutoCalculate(void);

void processRcSmoothingFilter(void);

int getRCDataRaw(int axis);

void setRCDataRaw(int axis, int num);

float getRCCommand(int axis);

void setRCCommand(int axis, int num);

float getMaxRcRate(int axis);

float getRcDeflection(int axis);

float getFeedforward(int axis);

void CK_RC_SetRcDeflection(int axis, float num);

float getRcDeflectionAbs(int axis);

void setRcDeflectionAbs(int axis, float num);

float getMaxRcDeflectionAbs(void);

float getSetpointRate(int axis);

float getRcCommandDelta(int axis);

float getCurrentRxRefreshRate(void);

#endif /* CK_RC_H_ */
