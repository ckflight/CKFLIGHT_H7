
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"

#include "SENSORS/CK_ICM20602.h"
#include "SENSORS/CK_IIM42652.h"
#include "SENSORS/CK_ICM42688P.h"
#include "SENSORS/CK_ICM42605.h"
#include "SENSORS/mpu6000.h"
#include "SENSORS/CK_L3GD20H.h"

#include "MOTION/CK_GYRO.h"

#include "FLIGHT/CK_PID.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define TIMEOUT_THREASHOLD                  10 // 10 Timeout starts reInit process
#define TIMEOUT_REINIT_THREASHOLD           3  // 3 Times max tries to reInit

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 	30340  // 92.5% full scale (1850dps for 2000dps gyro)

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

gyroSensor_t gyro;

pt1Filter_t lpf_gyro[XYZ_AXIS_COUNT];
biquadFilter_t gyro_biquadNotch1[XYZ_AXIS_COUNT];
biquadFilter_t gyro_biquadNotch2[XYZ_AXIS_COUNT];
biquadFilter_t gyro_biquadNotch3[XYZ_AXIS_COUNT];

DEBUG_TIME_t gyro_debug;

SPI_TypeDef* MOTION_GYRO_SPI;
GPIO_TypeDef* MOTION_GYRO_CS_PORT;
uint8_t MOTION_GYRO_CS_PIN;

static bool overflowDetected;
#ifdef USE_GYRO_OVERFLOW_CHECK
static timeUs_t overflowTimeUs;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static bool yawSpinRecoveryEnabled;
static int yawSpinRecoveryThreshold;
static bool yawSpinDetected;
static timeUs_t yawSpinTimeUs;
#endif

static float gyroFilteredDownsampled[XYZ_AXIS_COUNT];

/*
 * GYRO ORIENTATION:
 *
 * Pitch Down Y+
 * Roll Right X+
 * Right rotation Z+
 *
 * Important Note: On gyro sensor datasheet the arrow is not the axis.
 * The rotation around that arrow is the axis.
 *
 * -----> Y means when rotating in pitch direction it is chaning Y data
 */


void CK_GYRO_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, gyroSensorDps_e dps, uint32_t gyroT, uint32_t mainT){

	MOTION_GYRO_SPI 	= spin_;
	MOTION_GYRO_CS_PORT	= cs_gpio_;
	MOTION_GYRO_CS_PIN	= cs_pin_;

    gyro.sync.syncCounter = 0;

    gyro.sync.targetLoopTime = gyroT;

    gyro.sync.syncRate = gyroT / mainT;

	gyro.sensor = sensor;

	gyro.gyro_reInit_counter = 0;

	gyro.is_gyro_init = false;

	gyro.use_lpf_filter = true;
	gyro.use_notch1_filter = true;
	gyro.use_notch2_filter = true;
	gyro.use_notch3_filter = false;

    gyro.gyro_soft_notch_hz_1 		= 190;
    gyro.gyro_soft_notch_cutoff_1 	= 100;

    gyro.gyro_soft_notch_hz_2 		= 300;
	gyro.gyro_soft_notch_cutoff_2 	= 200;

	gyro.gyro_soft_notch_hz_3 		= 0;
	gyro.gyro_soft_notch_cutoff_3 	= 0;

	// If dyn min = 0 then dynamc lpf is not used
    gyro.gyro_lpf1_dyn_min_hz = 0;//GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
    gyro.gyro_lpf1_dyn_max_hz = GYRO_LPF1_DYN_MAX_HZ_DEFAULT;

    gyro.gyro_lpf1_type = FILTER_PT1;

    gyro.gyro_lpf1_static_hz = 80; // GYRO_LPF1_DYN_MIN_HZ_DEFAULT;

    gyro.checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;

	#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyro.checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        gyro.overflowAxisMask = GYRO_OVERFLOW_Z;
    }
    else if (gyro.checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        gyro.overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    }
    else {
        gyro.overflowAxisMask = 0;
    }
	#endif

    gyro.gyroHasOverflowProtection = true;

    gyro.yaw_spin_recovery = YAW_SPIN_RECOVERY_AUTO;
    gyro.yaw_spin_threshold = 1950;

#if USE_DMA_SENSOR

	#if USE_F4
		CK_SPI_DMA_EnableClock(SENSOR_DMA);

		CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);
		CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

		CK_SPI_DMA_InitTX(SENSOR_DMA_TX_Stream, SENSOR_DMA_TX_Channel);
		CK_SPI_DMA_InitRX(SENSOR_DMA_RX_Stream, SENSOR_DMA_RX_Channel);

		CK_SPI_DMA_TCInterruptEnable(SENSOR_DMA_TX_Stream);
		CK_SPI_DMA_TCInterruptEnable(SENSOR_DMA_RX_Stream);

		HAL_NVIC_EnableIRQ(SENSOR_DMA_TX_IRQn);
		HAL_NVIC_EnableIRQ(SENSOR_DMA_RX_IRQn);

		CK_SPI_DMA_SetPeripheralAddress(SENSOR_DMA_TX_Stream, (uint32_t)(&MOTION_GYRO_SPI->DR));
		CK_SPI_DMA_SetPeripheralAddress(SENSOR_DMA_RX_Stream, (uint32_t)(&MOTION_GYRO_SPI->DR));

	#endif

	#if USE_H7
		CK_SPI_DMA_EnableClock(SENSOR_DMA);

		CK_SPI_DMA_InitTX(SENSOR_DMA_TX_Stream, MOTION_GYRO_SPI, SENSOR_DMA_Request1);
		CK_SPI_DMA_InitRX(SENSOR_DMA_RX_Stream, MOTION_GYRO_SPI, SENSOR_DMA_Request2);

		HAL_NVIC_EnableIRQ(SENSOR_DMA_TX_IRQn);
		HAL_NVIC_EnableIRQ(SENSOR_DMA_RX_IRQn);

		CK_SPI_DMA_TCInterruptEnable(SENSOR_DMA_TX_Stream);
		CK_SPI_DMA_TCInterruptEnable(SENSOR_DMA_RX_Stream);

		CK_SPI_DMA_SetPeripheralAddress(SENSOR_DMA_TX_Stream, (uint32_t)(&MOTION_GYRO_SPI->TXDR));
		CK_SPI_DMA_SetPeripheralAddress(SENSOR_DMA_RX_Stream, (uint32_t)(&MOTION_GYRO_SPI->RXDR));

	#endif
#endif

	uint8_t reinit_count = 5;

	if(gyro.sensor == ICM20602_GYRO){

		CK_PRINTER_PrintlnString("ICM20602_GYRO");

		if(!CK_ICM20602_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM20602_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		if(dps == DPS250){
			gyro.gyroScale = 1.0f/131.0f;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 1.0f/65.5f;
		}
		else if(dps == DPS1000){
			gyro.gyroScale = 1.0f/32.8f;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 1.0f/16.4f;
		}

	}
	else if(gyro.sensor == MPU6000_GYRO){

		CK_PRINTER_PrintlnString("MPU6000_GYRO");

		if(!CK_MPU6000_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_MPU6000_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		if(dps == DPS250){
			gyro.gyroScale = 1.0f/131.0f;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 1.0f/65.5f;
		}
		else if(dps == DPS1000){
			gyro.gyroScale = 1.0f/32.8f;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 1.0f/16.4f;
		}

	}
	else if(gyro.sensor == IIM42652_GYRO){

		CK_PRINTER_PrintlnString("IIM42652_GYRO");

		if(!CK_IIM42652_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_IIM42652_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");

				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		if(dps == DPS250){
			gyro.gyroScale = 1.0f/131.0f;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 1.0f/65.5f;
		}
		else if(dps == DPS1000){
			gyro.gyroScale = 1.0f/32.8f;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 1.0f/16.4f;
		}

	}
	else if(gyro.sensor == ICM42688P_GYRO){

		CK_PRINTER_PrintlnString("ICM42688P_GYRO");

		if(!CK_ICM42688P_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM42688P_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");

				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		if(dps == DPS250){
			gyro.gyroScale = 1.0f/131.0f;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 1.0f/65.5f;
		}
		else if(dps == DPS1000){
			gyro.gyroScale = 1.0f/32.8f;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 1.0f/16.4f;
		}

	}
	else if(gyro.sensor == ICM42605_GYRO){

		CK_PRINTER_PrintlnString("ICM42605_GYRO");

		if(!CK_ICM42605_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM42605_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		if(dps == DPS250){
			gyro.gyroScale = 1.0f/131.0f;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 1.0f/65.5f;
		}
		else if(dps == DPS1000){
			gyro.gyroScale = 1.0f/32.8f;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 1.0f/16.4f;
		}

	}
	else if(gyro.sensor == L3GD20H_GYRO){

		CK_PRINTER_PrintlnString("L3GD20H_GYRO");

		if(!CK_L3GD20H_isGyroSensorInitialized()){
			while(reinit_count--){
				if(CK_L3GD20H_Init(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime)){
					reinit_count = 0;
					gyro.is_gyro_init = true;

					CK_PRINTER_PrintlnString("GYRO INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("GYRO ERROR");
			}
		}

		//sensitivity milidps/digit
		if(dps == DPS245){
			gyro.gyroScale = 0.00875;
		}
		else if(dps == DPS500){
			gyro.gyroScale = 0.0175;
		}
		else if(dps == DPS2000){
			gyro.gyroScale = 0.07;
		}

	}

	gyro.gyroSign[X] 	= GYRO_ORIENTATION_X_SIGN;
	gyro.gyroSign[Y] 	= GYRO_ORIENTATION_Y_SIGN;
	gyro.gyroSign[Z] 	= GYRO_ORIENTATION_Z_SIGN;
	gyro.isgyroSwapAxis 	= GYRO_ORIENTATION_SWAP_XY;

	// Perform Gyro Calibration
	CK_GYRO_PerformCalibration();

	CK_GYRO_InitFilters();

	CK_GYRO_ResetGyroSum();

}

void CK_GYRO_InitFilters(void){

	// Initialize Filters

	// Low pass filter

	// If dynamic filter USE_DYN_LPF is not defined then static hz is used
    uint16_t gyro_lpf1_init_hz = gyro.gyro_lpf1_static_hz;

	#ifdef USE_DYN_LPF
    if (gyro.gyro_lpf1_dyn_min_hz > 0) {
        gyro_lpf1_init_hz = gyro.gyro_lpf1_dyn_min_hz;
    }
	#endif

    if(gyro.use_lpf_filter){
    	gyroInitLowpassFilterLpf(FILTER_LPF1, gyro.gyro_lpf1_type, gyro_lpf1_init_hz, gyro.sync.targetLoopTime);
    }

    // Notch filters
    if(gyro.use_notch1_filter){
    	gyroInitFilterNotch1(gyro.gyro_soft_notch_hz_1, gyro.gyro_soft_notch_cutoff_1);
    }

    if(gyro.use_notch2_filter){
    	gyroInitFilterNotch2(gyro.gyro_soft_notch_hz_2, gyro.gyro_soft_notch_cutoff_2);
	}

	#ifdef USE_DYN_LPF
	dynLpfFilterInit();
	#endif

	const float k = pt1FilterGain(GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ, gyro.sync.targetLoopTime * 1e-6f);
	for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
		pt1FilterInit(&gyro.imuGyroFilter[axis], k);
	}

	//CK_GYRO_KalmanInit(&gyro.gyro_kalmanFilter, 100, 100, 25);
}

float gyroGetFilteredDownsampled(int axis)
{
    return gyroFilteredDownsampled[axis];
}

#ifdef USE_DYN_LPF
void dynLpfFilterInit(void){

    if (gyro.gyro_lpf1_dyn_min_hz > 0) {
        switch (gyro.gyro_lpf1_type) {
        case FILTER_PT1:
            gyro.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            gyro.dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        case FILTER_PT2:
            gyro.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            gyro.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            gyro.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    }
    else {
        gyro.dynLpfFilter = DYN_LPF_NONE;
    }
    gyro.dynLpfMin = gyro.gyro_lpf1_dyn_min_hz;
    gyro.dynLpfMax = gyro.gyro_lpf1_dyn_max_hz;
    gyro.dynLpfCurveExpo = gyro.gyro_lpf1_dyn_expo;
}
#endif

bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LPF1:
        lowpassFilterApplyFn = &gyro.lowpassFilterApplyFn;
        lowpassFilter = gyro.lowpassFilter;
        break;

    case FILTER_LPF2:
        lowpassFilterApplyFn = &gyro.lowpass2FilterApplyFn;
        lowpassFilter = gyro.lowpass2Filter;
        break;

    default:
        return false;
    }

    bool ret = false;

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
    const float gyroDt = looptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    if (lpfHz) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
            	pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_BIQUAD:
            if (lpfHz <= gyroFrequencyNyquist) {
				#ifdef USE_DYN_LPF
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
				#else
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
				#endif
                for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
                	biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, looptime);
                }
                ret = true;
            }
            break;
        case FILTER_PT2:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt2FilterApply;
            for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
            	pt2FilterInit(&lowpassFilter[axis].pt2FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_PT3:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt3FilterApply;
            for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
            	pt3FilterInit(&lowpassFilter[axis].pt3FilterState, gain);
            }
            ret = true;
            break;
        }
    }
    return ret;

}

uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.sync.targetLoopTime;

    if (notchHz > gyroFrequencyNyquist) {

    	if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        }
        else {
            notchHz = 0;
        }
    }

    return notchHz;
}

void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyro.notchFilter1[axis], notchHz, gyro.sync.targetLoopTime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
        	biquadFilterInit(&gyro.notchFilter2[axis], notchHz, gyro.sync.targetLoopTime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

void CK_GYRO_ReadRaw(void){

#if USE_DMA_SENSOR

	if(gyro.sensor == ICM20602_GYRO){

		CK_ICM20602_ReadSensorRaw_DMA();

	}
	else if(gyro.sensor == IIM42652_GYRO){

		CK_IIM42652_ReadSensorRaw_DMA();

	}
	else if(gyro.sensor == ICM42688P_GYRO){

		CK_ICM42688P_ReadSensorRaw_DMA();

	}
	else if(gyro.sensor == L3GD20H_GYRO){



	}


#else

	// Read Raw gyro values
	if(gyroSensor == ICM20602_GYRO){

		CK_ICM20602_ReadGyroRaw();

	}
	else if(gyroSensor == IIM42652_GYRO){

		CK_IIM42652_ReadGyroRaw();

	}
	else if(gyroSensor == ICM42688P_GYRO){

		CK_ICM42688P_ReadGyroRaw();

	}
	else if(gyroSensor == L3GD20H_GYRO){

		CK_L3GD20H_ReadGyroRaw();

	}

#endif

}

void CK_GYRO_AllignSensor(void){

	if(gyro.isgyroSwapAxis){

		float temp_x = gyro.gyroADCf[0];
		float temp_y = gyro.gyroADCf[1];

		gyro.gyroADCf[X] = temp_y;
		gyro.gyroADCf[Y] = temp_x;

	}

	gyro.gyroADCf[X] *= gyro.gyroSign[X];
	gyro.gyroADCf[Y] *= gyro.gyroSign[Y];
	gyro.gyroADCf[Z] *= gyro.gyroSign[Z];

}

void CK_GYRO_Update(uint32_t currentTimeUs){

	gyro.sync.syncCounter++;

	if(gyro.sync.syncCounter >= gyro.sync.syncRate){

        #if defined(DEBUG_TIMING)
	    gyro_debug.start_time = CK_TIME_GetMicroSec();
        #endif

	    gyro.sync.syncCounter = 0;

		// Read Raw gyro values
	    CK_GYRO_ReadRaw();

		for(int axis = X; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++){

			// Apply calibration values
			gyro.gyroADCf[axis]  = gyro.gyroADCRaw[axis]  - gyro.gyroADCZero[axis];

		}

		CK_GYRO_AllignSensor();

		// Apply Filters
		for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

			// Scale gyro data
			float gyroADCf = gyro.gyroADCf[axis]  * gyro.gyroScale;

			// Notch Filters
			gyro.gyroADCPreNotch[axis] = gyroADCf;

			// apply static notch filters and software lowpass filters
			gyroADCf = gyro.notchFilter1ApplyFn((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
			gyroADCf = gyro.notchFilter2ApplyFn((filter_t *)&gyro.notchFilter2[axis], gyroADCf);

			// Low Pass Filter
			gyro.gyroADCPreLPF[axis] = gyroADCf;

			gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);


			// Apply Kalman Filter
			//gyroADCf = CK_FILTER_KalmanApply(&gyro_kalmanFilter, gyroADCf, axis);

			gyro.gyroADCf[axis] = gyroADCf;

			gyro.gyroADCfSum[axis] += gyro.gyroADCf[axis];

		}

		gyro.gyroADCfSumCounter++;

		CK_GYRO_CheckTimeout();

		#ifdef USE_GYRO_OVERFLOW_CHECK
		if (gyro.checkOverflow && !gyro.gyroHasOverflowProtection) {
			checkForOverflow(currentTimeUs);
		}
		#endif

		#ifdef USE_YAW_SPIN_RECOVERY
		if (yawSpinRecoveryEnabled) {
			checkForYawSpin(currentTimeUs);
		}
		#endif

		if (!overflowDetected) {
			for (int axis = X; axis < XYZ_AXIS_COUNT; axis++) {
				gyroFilteredDownsampled[axis] = pt1FilterApply(&gyro.imuGyroFilter[axis], gyro.gyroADCf[axis]);
			}
		}

		#if !defined(USE_GYRO_OVERFLOW_CHECK) && !defined(USE_YAW_SPIN_RECOVERY)
		UNUSED(currentTimeUs);
		#endif

        #if defined(DEBUG_TIMING)
		gyro_debug.update_time = CK_TIME_GetMicroSec() - gyro_debug.start_time;
        #endif

	}

}

void CK_GYRO_ResetGyroSum(void){
	gyro.gyroADCfSum[0] = 0.0f;
	gyro.gyroADCfSum[1] = 0.0f;
	gyro.gyroADCfSum[2] = 0.0f;

	gyro.gyroADCfSumCounter = 0;
}

void CK_GYRO_PerformCalibration(void){

    CK_PRINTER_PrintString("Gyro Calibration");
	// Set sum to zero
	for(int axis = FD_ROLL; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++){

		gyro.gyroADCSum[axis]  = 0;

	}

	// 1 second read
	int32_t num_of_calibration = 1000000 / gyro.sync.targetLoopTime;

	// Start calibration
	for(int i = 0; i < num_of_calibration; i++){

		// Read Raw gyro values
		// Read calibration with normal spi

		if(gyro.sensor == ICM20602_GYRO){

			CK_ICM20602_ReadGyroRaw();

		}
		else if(gyro.sensor == IIM42652_GYRO){

			CK_IIM42652_ReadGyroRaw();

		}
		else if(gyro.sensor == ICM42688P_GYRO){

			CK_ICM42688P_ReadGyroRaw();

		}
		else if(gyro.sensor == L3GD20H_GYRO){

			CK_L3GD20H_ReadGyroRaw();

		}

		// Accumulate to sum
		for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

			gyro.gyroADCSum[axis]  += gyro.gyroADCRaw[axis];

		}

		CK_TIME_DelayMicroSec(gyro.sync.targetLoopTime);
		if(i % 200 == 0){
		    CK_PRINTER_PrintString(".");
		}

	}

	// Get Final Calibration Values
	for(int axis = X; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++){

		gyro.gyroADCZero[axis]  = gyro.gyroADCSum[axis]  / num_of_calibration;

	}
	CK_PRINTER_PrintlnString("");

}

void CK_GYRO_KalmanInit(kalmanFilter_t* kalman, float q, float r, float p){
	for(int axis = FD_ROLL; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++){

		// Initialize Kalman Filter for each axis
		kalmanInit(kalman, q, r, p, axis);

	}
}

void CK_GYRO_BiquadNotchInit(biquadFilter_t* gyro_notch, uint16_t notchFreqHz, uint16_t notchCutoffHz, uint32_t refreshRate){

	// Calculate Nyquist adjusted Notch Hz, not necessary if frequencies are set correctly
	const uint32_t gyroFreqNyquist =  1000000 / 2 / refreshRate; // gyroFreqNyquist = gyroSampleRate/2
	if(notchFreqHz > gyroFreqNyquist){
		if(notchCutoffHz < gyroFreqNyquist){
			notchFreqHz = 	gyroFreqNyquist;
		}
		else{
			notchFreqHz = 	0;
		}
	}

	if(notchFreqHz != 0 && notchCutoffHz != 0){

		const float notchQ = filterGetNotchQ(notchFreqHz, notchCutoffHz);

		for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){

			// Initialize Notch Filter for each axis
			biquadFilterInit(&gyro_notch[axis], notchFreqHz, refreshRate, notchQ, FILTER_NOTCH, 1);

		}
	}
}

void CK_GYRO_CheckTimeout(void){

	// Check if any timeout occured
	if(CK_SPI_GetTimeOut(MOTION_GYRO_SPI) == TIMEOUT_THREASHOLD){

		CK_BUZZER_Tone3();
	    CK_SPI_ResetTimeOut(MOTION_GYRO_SPI);

	    if(gyro.gyro_reInit_counter < TIMEOUT_REINIT_THREASHOLD){

	    	if(gyro.sensor == ICM20602_GYRO){
	    		CK_ICM20602_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime);
	    	}
	    	else if(gyro.sensor == IIM42652_GYRO){
				CK_IIM42652_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime);
			}
	    	else if(gyro.sensor == ICM42688P_GYRO){
	    		CK_ICM42688P_GyroInit(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime);
			}
	    	else if(gyro.sensor == L3GD20H_GYRO){
	    		CK_L3GD20H_Init(MOTION_GYRO_SPI, MOTION_GYRO_CS_PORT, MOTION_GYRO_CS_PIN, gyro.sync.targetLoopTime);
			}
	        gyro.gyro_reInit_counter++;
	    }
	}
}

#ifdef USE_DYN_LPF

float dynThrottle(float throttle) {
    return throttle * (1 - (throttle * throttle) / 3.0f) * 1.5f;
}

void dynLpfGyroUpdate(float throttle)
{
    if (gyro.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (gyro.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, gyro.dynLpfMin, gyro.dynLpfMax, gyro.dynLpfCurveExpo);
        }
        else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * gyro.dynLpfMax, gyro.dynLpfMin);
        }

        const float gyroDt = gyro.sync.targetLoopTime * 1e-6f;
        switch (gyro.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt1FilterState, pt1FilterGain(cutoffFreq, gyroDt));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            	biquadFilterUpdateLPF(&gyro.lowpassFilter[axis].biquadFilterState, cutoffFreq, gyro.sync.targetLoopTime);
            }
            break;
        case  DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            	pt2FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt2FilterState, pt2FilterGain(cutoffFreq, gyroDt));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            	pt3FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt3FilterState, pt3FilterGain(cutoffFreq, gyroDt));
            }
            break;
        }
    }
}
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
void handleOverflow(timeUs_t currentTimeUs)
{
    // This will need to be revised if we ever allow different sensor types to be
    // used simultaneously. In that case the scale might be different between sensors.
    // It's complicated by the fact that we're using filtered gyro data here which is
    // after both sensors are scaled and averaged.
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyro.gyroScale;

    if ((fabsf(gyro.gyroADCf[X]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Y]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Z]) < gyroOverflowResetRate)) {

    	// if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, overflowTimeUs) > 50000) {
            overflowDetected = false;
        }
    }
    else {
        // not a consecutive OK value, so reset the overflow time
        overflowTimeUs = currentTimeUs;
    }
}

void checkForOverflow(timeUs_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (overflowDetected) {
        handleOverflow(currentTimeUs);
    }
    else {
		#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;

        // This will need to be revised if we ever allow different sensor types to be
        // used simultaneously. In that case the scale might be different between sensors.
        // It's complicated by the fact that we're using filtered gyro data here which is
        // after both sensors are scaled and averaged.
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyro.gyroScale;

        if (fabsf(gyro.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (fabsf(gyro.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (fabsf(gyro.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & gyro.overflowAxisMask) {
            overflowDetected = true;
            overflowTimeUs = currentTimeUs;
			#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = false;
			#endif
        }
		#endif
    }
}

bool gyroOverflowDetected(void)
{
	#ifdef USE_GYRO_OVERFLOW_CHECK
    return overflowDetected;
	#else
    return false;
	#endif
}

#endif

#ifdef USE_YAW_SPIN_RECOVERY
void handleYawSpin(timeUs_t currentTimeUs)
{
    const float yawSpinResetRate = yawSpinRecoveryThreshold - 100.0f;
    if (fabsf(gyro.gyroADCf[Z]) < yawSpinResetRate) {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if (cmpTimeUs(currentTimeUs, yawSpinTimeUs) > 20000) {
            yawSpinDetected = false;
        }
    }
    else {
        // reset the yaw spin time
        yawSpinTimeUs = currentTimeUs;
    }
}

void checkForYawSpin(timeUs_t currentTimeUs)
{
    // if not in overflow mode, handle yaw spins above threshold
	#ifdef USE_GYRO_OVERFLOW_CHECK
    if (overflowDetected) {
        yawSpinDetected = false;
        return;
    }
	#endif // USE_GYRO_OVERFLOW_CHECK

    if (yawSpinDetected) {
        handleYawSpin(currentTimeUs);
    }
    else {
	#ifndef SIMULATOR_BUILD
        // check for spin on yaw axis only
         if (CK_MATH_ABS((int)gyro.gyroADCf[Z]) > yawSpinRecoveryThreshold) {
            yawSpinDetected = true;
            yawSpinTimeUs = currentTimeUs;
        }
	#endif // SIMULATOR_BUILD
    }
}

bool gyroYawSpinDetected(void)
{
    return yawSpinDetected;
}

void initYawSpinRecovery(int maxYawRate)
{
    bool enabledFlag;
    int threshold;

    switch (gyro.yaw_spin_recovery) {
    case YAW_SPIN_RECOVERY_ON:
        enabledFlag = true;
        threshold = gyro.yaw_spin_threshold;
        break;
    case YAW_SPIN_RECOVERY_AUTO:
        enabledFlag = true;
        const int overshootAllowance = CK_MATH_MAX(maxYawRate / 4, 200); // Allow a 25% or minimum 200dps overshoot tolerance
        threshold = CK_MATH_Constrain(maxYawRate + overshootAllowance, YAW_SPIN_RECOVERY_THRESHOLD_MIN, YAW_SPIN_RECOVERY_THRESHOLD_MAX);
        break;
    case YAW_SPIN_RECOVERY_OFF:
    default:
        enabledFlag = false;
        threshold = YAW_SPIN_RECOVERY_THRESHOLD_MAX;
        break;
    }

    yawSpinRecoveryEnabled = enabledFlag;
    yawSpinRecoveryThreshold = threshold;
}


#endif
