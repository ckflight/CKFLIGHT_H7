
#ifndef CK_MIXER_H_
#define CK_MIXER_H_

#include "CK_DEFINITIONS.h"

#define MAX_SUPPORTED_MOTORS    4

typedef enum mixerType
{
    MIXER_LEGACY = 0,
    MIXER_LINEAR = 1,
    MIXER_DYNAMIC = 2,
} mixerType_e;

typedef struct{

    uint8_t motorCount;

    float motorOutputLow;
    float motorOutputHigh;

    float disarmMotorOutput;

    uint8_t mixer_type;

	#ifdef USE_DYN_IDLE
    float dynIdleMaxIncrease;
    float idleThrottleOffset;
    float dynIdleMinRps;
    float dynIdlePGain;
    float prevMinRps;
    float dynIdleIGain;
    float dynIdleDGain;
    float dynIdleI;
    float minRpsDelayK;

    bool useDshotTelemetry;

	#endif

}mixerRuntime_t;

extern mixerRuntime_t mixerRuntime;

void CK_MIXER_Init(void);

void CK_MIXER_Update(uint32_t currentTimeUs);

void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled);

void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled);

void CK_MIXER_MixTable(uint32_t currentTimeUs);

void CK_MIXER_ApplyMixToMotors(void);

void CK_MIXER_ApplyFinalToMotors(void);

void mixerSetThrottleAngleCorrection(int correctionValue);

void CK_MIXER_CalculateThrottleAndMotorRange(void);

float getMotorMixRange(void);

float mixerGetRcThrottle(void);

bool CK_MIXER_IsMixerSaturated(void);

int CK_MIXER_GetMotorFinalResult(int motorNumber);

float CK_MIXER_GetMotorMixResult(int motorNumber);

#ifdef USE_DYN_LPF
void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle);
#endif

#endif /* CK_MIXER_H_ */
