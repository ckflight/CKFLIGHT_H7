
#ifndef CK_ACC_H_
#define CK_ACC_H_

#include "COMMON/CK_FILTERS.h"
#include "COMMON/vector.h"

#define ACC_READ_ARRAY_SIZE		16

#define ACC_FILTERCUTOFF_IMU_100HZ			25 // 25 Hz cutoff for IMU running at 100 Hz
#define ACC_FILTERCUTOFF_IMU_500HZ			50 // 50 Hz cutoff for IMU running at 500 Hz

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    bool acc_high_fsr;
    rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;

typedef struct accelSensor_s{
    int16_t     accADCRaw[XYZ_AXIS_COUNT];
    int16_t     accADCZero[XYZ_AXIS_COUNT];
    float       accADCf[XYZ_AXIS_COUNT];

    uint16_t    acc1G;
    float 		acc_1G_rec;
    int         accSign[XYZ_AXIS_COUNT];

    //float       accAccumulate[XYZ_AXIS_COUNT];
    //int         accAccumulateCount;

    float       accADCEarthSum[XYZ_AXIS_COUNT];
    int         accADCEarthSumCounter;

    int 		is_acc_init;

    int 		acc_reInit_counter;

    float accMagnitude;                     // in multiples of 1G
    float jerkMagnitude;                    // in multiples of 1G/s (measure of collision strength)
    vector3_t accADC;                       // rotated but unscaled ADC value
    vector3_t jerk;
    uint16_t sampleRateHz;

    accelerometerConfig_t accelerometerConfig;

    bool isAccelUpdatedAtLeastOnce;

    sensorModel_e sensor;

    syncTimer_t sync;

}accelSensor_t;

typedef enum{

	G2					= 0,
	G4					= 1,
	G8					= 2,
	G16					= 3

}accSensorG_e;

extern accelSensor_t acc;

void CK_ACC_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, accSensorG_e g, uint32_t accT, uint32_t mainT);

void CK_ACC_Init2(I2C_TypeDef* i2cn_, sensorModel_e sensor, accSensorG_e g, uint32_t accT, uint32_t mainT);

void CK_ACC_Update(void);

void CK_ACC_UpdateFilter(uint8_t cutoff);

void CK_ACC_ReadACCRaw(void);

void CK_ACC_ResetAccEarthSum(void);

void CK_ACC_PerformCalibration(int16_t* acc_buffer);

void CK_ACC_BiquadLPFInit(biquadFilter_t* filterType, uint16_t filterFreq1, uint16_t filterFreq2, uint16_t refreshRate);

void CK_ACC_CheckTimeout(void);

#endif /* CK_ACC_H_ */
