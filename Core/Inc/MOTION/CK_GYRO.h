
#ifndef CK_GYRO_H_
#define CK_GYRO_H_

#include "CK_DEFINITIONS.h"

#include "DRIVERS/CK_TIME_HAL.h"

#include "COMMON/CK_FILTERS.h"

#define GYRO_READ_ARRAY_SIZE	32

#define GYRO_LPF1_DYN_MIN_HZ_DEFAULT 250
#define GYRO_LPF1_DYN_MAX_HZ_DEFAULT 500

#define GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ 200

#ifdef USE_YAW_SPIN_RECOVERY
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950
#endif

typedef enum{

	DPS245					= 0,
	DPS250					= 1,
	DPS500					= 2,
	DPS1000					= 3,
	DPS2000					= 4

}gyroSensorDps_e;

enum {
    DYN_LPF_NONE = 0,
    DYN_LPF_PT1,
    DYN_LPF_BIQUAD,
    DYN_LPF_PT2,
    DYN_LPF_PT3,
};

typedef enum {
    GYRO_OVERFLOW_NONE = 0x00,
    GYRO_OVERFLOW_X = 0x01,
    GYRO_OVERFLOW_Y = 0x02,
    GYRO_OVERFLOW_Z = 0x04
} gyroOverflow_e;

enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
};

typedef enum {
    YAW_SPIN_RECOVERY_OFF,
    YAW_SPIN_RECOVERY_ON,
    YAW_SPIN_RECOVERY_AUTO
} yawSpinRecoveryMode_e;

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef struct gyroSensor_s{

    int16_t     gyroADCRaw[XYZ_AXIS_COUNT];
    int32_t     gyroADCSum[XYZ_AXIS_COUNT];
    int32_t     gyroADCZero[XYZ_AXIS_COUNT];

    float       gyroADCPreLPF[XYZ_AXIS_COUNT];
    float       gyroADCPreNotch[XYZ_AXIS_COUNT];

    float       gyroADCf[XYZ_AXIS_COUNT];
    float       gyroScale;
    int         gyroSign[XYZ_AXIS_COUNT];
    bool        isgyroSwapAxis;

    float       gyroADCfSum[XYZ_AXIS_COUNT];
    int         gyroADCfSumCounter;

    int 		gyro_reInit_counter;

    bool 		is_gyro_init;

    bool		use_lpf_filter;
    bool		use_notch1_filter;
    bool		use_notch2_filter;
    bool		use_notch3_filter;

    uint16_t 	gyro_lpf1_dyn_min_hz;
    uint16_t 	gyro_lpf1_dyn_max_hz;

    uint8_t 	gyro_lpf1_type;

    uint16_t 	gyro_lpf1_static_hz;
    uint16_t 	gyro_lpf2_static_hz;

    uint8_t 	gyro_lpf1_dyn_expo; // set the curve for dynamic gyro lowpass filter

    uint16_t 	gyro_soft_notch_hz_1;
    uint16_t 	gyro_soft_notch_cutoff_1;
    uint16_t 	gyro_soft_notch_hz_2;
    uint16_t 	gyro_soft_notch_cutoff_2;

    uint16_t 	gyro_soft_notch_hz_3;
    uint16_t 	gyro_soft_notch_cutoff_3;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    kalmanFilter_t gyro_kalmanFilter;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    uint8_t overflowAxisMask;
#endif

    uint8_t checkOverflow;

    pt1Filter_t imuGyroFilter[XYZ_AXIS_COUNT];

    bool gyroHasOverflowProtection;

    uint8_t yaw_spin_recovery;
    int16_t yaw_spin_threshold;

    syncTimer_t sync;

    sensorModel_e sensor;

}gyroSensor_t;

extern gyroSensor_t gyro;

void CK_GYRO_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, gyroSensorDps_e dps, uint32_t gyroT, uint32_t mainT);

void CK_GYRO_InitFilters(void);

float gyroGetFilteredDownsampled(int axis);

#ifdef USE_DYN_LPF
void dynLpfFilterInit(void);
#endif

bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime);

uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz);

void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz);

void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz);

void CK_GYRO_ReadRaw(void);

void CK_GYRO_AllignSensor(void);

void CK_GYRO_Update(uint32_t currentTimeUs);

void CK_GYRO_ResetGyroSum(void);

void CK_GYRO_PerformCalibration(void);

void CK_GYRO_KalmanInit(kalmanFilter_t* kalman, float q, float r, float p);

void CK_GYRO_BiquadNotchInit(biquadFilter_t* gyro_notch, uint16_t notchFreqHz, uint16_t notchCutoffHz, uint32_t refreshRate);

void CK_GYRO_CheckTimeout(void);

#ifdef USE_DYN_LPF
float dynThrottle(float throttle);
void dynLpfGyroUpdate(float throttle);
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
void handleOverflow(timeUs_t currentTimeUs);
void checkForOverflow(timeUs_t currentTimeUs);
bool gyroOverflowDetected(void);

#endif

#ifdef USE_YAW_SPIN_RECOVERY
void handleYawSpin(timeUs_t currentTimeUs);
void checkForYawSpin(timeUs_t currentTimeUs);
bool gyroYawSpinDetected(void);
void initYawSpinRecovery(int maxYawRate);
#endif


#endif /* CK_GYRO_H_ */
