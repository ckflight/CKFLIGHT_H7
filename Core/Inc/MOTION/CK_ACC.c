
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_I2C.h"

#include "SENSORS/CK_ICM20602.h"
#include "SENSORS/CK_IIM42652.h"
#include "SENSORS/CK_ICM42688P.h"
#include "SENSORS/CK_ICM42605.h"

#include "SENSORS/CK_LSM303D.h"
#include "SENSORS/CK_FXOS8700CQ.h"

#include "MOTION/CK_ACC.h"

#include "FLASH/CK_FLASH.h"

#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/CK_PRINTER.h"

#define TIMEOUT_THREASHOLD                  10 // 10 Timeout starts reInit process
#define TIMEOUT_REINIT_THREASHOLD           3  // 3 Times max tries to reInit


accelSensor_t acc;

pt2Filter_t acc_pt2filter[XYZ_AXIS_COUNT];

DEBUG_TIME_t acc_debug;

I2C_TypeDef* MOTION_ACC_I2C;
SPI_TypeDef* MOTION_ACC_SPI;
GPIO_TypeDef* MOTION_ACC_CS_PORT;
uint8_t MOTION_ACC_CS_PIN;

vector3_t accAdcPrev;

void CK_ACC_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, accSensorG_e g, uint32_t accT, uint32_t mainT){

	MOTION_ACC_SPI 		= spin_;
	MOTION_ACC_CS_PORT 	= cs_gpio_;
	MOTION_ACC_CS_PIN 	= cs_pin_;

	acc.is_acc_init = false;

    acc.sync.syncCounter = 0;

    acc.sync.targetLoopTime = accT;

    acc.sync.syncRate = accT / mainT;

    acc.sampleRateHz = 1000000 / acc.sync.targetLoopTime;

	acc.sensor = sensor;

	acc.accelerometerConfig.acc_hardware = sensor;
	acc.accelerometerConfig.acc_lpf_hz = 25; // ATTITUDE/IMU runs at 100Hz (acro) or 500Hz (level modes) so we need to set 50 Hz (or lower) to avoid aliasing
	acc.accelerometerConfig.acc_high_fsr = false;
	acc.accelerometerConfig.accelerometerTrims.raw[0] = 0;
	acc.accelerometerConfig.accelerometerTrims.raw[1] = 0;

	acc.acc_reInit_counter = 0;

	uint8_t reinit_count = 5;

	if(acc.sensor == ICM20602_ACC){

		if(!CK_ICM20602_isAccSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM20602_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime)){
					reinit_count = 0;
					CK_PRINTER_PrintlnString("ACC INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("ACC ERROR");
			}
		}

		if(g == G2){
			acc.acc1G = 16384.0f;
		}
		else if(g == G4){
			acc.acc1G = 8192.0f;
		}
		else if(g == G8){
			acc.acc1G = 4096.0f;
		}
		else if(g == G16){
			acc.acc1G = 2048.0f;

		}

	}

	else if(acc.sensor == IIM42652_ACC){

		if(!CK_IIM42652_isAccSensorInitialized()){
			while(reinit_count--){
				if(CK_IIM42652_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime)){
					reinit_count = 0;
					CK_PRINTER_PrintlnString("ACC INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("ACC ERROR");
			}
		}

		if(g == G2){
			acc.acc1G = 16384.0f;
		}
		else if(g == G4){
			acc.acc1G = 8192.0f;
		}
		else if(g == G8){
			acc.acc1G = 4096.0f;
		}
		else if(g == G16){
			acc.acc1G = 2048.0f;

		}

	}

	else if(acc.sensor == ICM42688P_ACC){

		if(!CK_ICM42688P_isAccSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM42688P_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime)){
					reinit_count = 0;
					CK_PRINTER_PrintlnString("ACC INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("ACC ERROR");
			}
		}

		if(g == G2){
			acc.acc1G = 16384.0f;
		}
		else if(g == G4){
			acc.acc1G = 8192.0f;
		}
		else if(g == G8){
			acc.acc1G = 4096.0f;
		}
		else if(g == G16){
			acc.acc1G = 2048.0f;

		}

	}

	else if(acc.sensor == ICM42605_ACC){

		if(!CK_ICM42605_isAccSensorInitialized()){
			while(reinit_count--){
				if(CK_ICM42605_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime)){
					reinit_count = 0;
					CK_PRINTER_PrintlnString("ACC INIT CORRECT");
				}
				else CK_PRINTER_PrintlnString("ACC ERROR");
			}
		}

		if(g == G2){
			acc.acc1G = 16384.0f;
		}
		else if(g == G4){
			acc.acc1G = 8192.0f;
		}
		else if(g == G8){
			acc.acc1G = 4096.0f;
		}
		else if(g == G16){

			acc.acc1G = 9.807f * 16.0f/32767.5f;

		}

	}

	acc.acc_1G_rec = 1.0f / acc.acc1G;

	acc.accSign[X] 	= ACC_ORIENTATION_X_SIGN;
	acc.accSign[Y] 	= ACC_ORIENTATION_Y_SIGN;
	acc.accSign[Z] 	= ACC_ORIENTATION_Z_SIGN;

	// Initialize Filters
	if (ACC_FILTERCUTOFF_IMU_100HZ) {
		const float k = pt2FilterGain(ACC_FILTERCUTOFF_IMU_100HZ, HZ_TO_INTERVAL(HZ_TO_INTERVAL_US(acc.sync.targetLoopTime)));
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			pt2FilterInit(&acc_pt2filter[axis], k);
		}
	}

	// Calibration should be perfomed at flat surface.
	uint8_t acc_buffer[CONFIG_ACC_BYTES];
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, acc_buffer, CONFIG_ACC_BYTES, CONFIG_ACC_OFFSET);

	acc.accADCZero[X] = (acc_buffer[0] << 8) | acc_buffer[1];
	acc.accADCZero[Y] = (acc_buffer[2] << 8) | acc_buffer[3];
	acc.accADCZero[Z] = (acc_buffer[4] << 8) | acc_buffer[5];

	// Do not calibrate acc at start because quadcopter can start from a tilted surface
	// Calibrate the quad using the terminal to save that to eeprom
	//int16_t temp_buf[3];
	//CK_ACC_PerformCalibration(temp_buf);

	CK_ACC_ResetAccEarthSum();

	acc.is_acc_init = true;


}

void CK_ACC_Init2(I2C_TypeDef* i2cn_, sensorModel_e sensor, accSensorG_e g, uint32_t accT, uint32_t mainT){

	MOTION_ACC_I2C = i2cn_;

	acc.is_acc_init = false;

	acc.sync.syncCounter = 0;

	acc.sync.targetLoopTime = accT;

	acc.sync.syncRate = accT / mainT;

    acc.sensor = sensor;

	acc.accelerometerConfig.acc_hardware = sensor;
	acc.accelerometerConfig.acc_lpf_hz = 25; // ATTITUDE/IMU runs at 100Hz (acro) or 500Hz (level modes) so we need to set 50 Hz (or lower) to avoid aliasing
	acc.accelerometerConfig.acc_high_fsr = false;
	acc.accelerometerConfig.accelerometerTrims.raw[0] = 0;
	acc.accelerometerConfig.accelerometerTrims.raw[1] = 0;

	acc.acc_reInit_counter = 0;

	if(acc.sensor == LSM303D_ACC){

		if(!CK_LSM303D_isAccSensorInitialized()){
			CK_LSM303D_AccInit(MOTION_ACC_I2C, acc.sync.targetLoopTime);
		}

		// Calculation divides so i write 1 over number it will be multiplication
		acc.acc1G = 1 / 0.004f; // for +-8g

	}
	else if(acc.sensor == FXOS8700CQ_ACC){

		if(!CK_FXOS8700CQ_isAccSensorInitialized()){
			CK_FXOS8700CQ_AccInit(MOTION_ACC_I2C, acc.sync.targetLoopTime);
		}

		acc.acc1G = 1 / 0.000488f; // for +-4g 0.488 mg/LSB
	}

	acc.acc_1G_rec = 1.0f / acc.acc1G;

	acc.accSign[X] 	= ACC_ORIENTATION_X_SIGN;
	acc.accSign[Y] 	= ACC_ORIENTATION_Y_SIGN;
	acc.accSign[Z] 	= ACC_ORIENTATION_Z_SIGN;

	// Initialize Filters
	if (ACC_FILTERCUTOFF_IMU_100HZ) {
		const float k = pt2FilterGain(ACC_FILTERCUTOFF_IMU_100HZ, HZ_TO_INTERVAL(HZ_TO_INTERVAL_US(acc.sync.targetLoopTime)));
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			pt2FilterInit(&acc_pt2filter[axis], k);
		}
	}

	// Calibration should be perfomed at flat surface.
	uint8_t acc_buffer[CONFIG_ACC_BYTES];
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, acc_buffer, CONFIG_ACC_BYTES, CONFIG_ACC_OFFSET);

	acc.accADCZero[X] = (acc_buffer[0] << 8) | acc_buffer[1];
	acc.accADCZero[Y] = (acc_buffer[2] << 8) | acc_buffer[3];

	// Do not calibrate acc at start because quadcopter can start from a tilted surface
	// Calibrate the quad using the terminal to save that to eeprom
	//int16_t temp_buf[3];
	//CK_ACC_PerformCalibration(temp_buf);

	CK_ACC_ResetAccEarthSum();

	acc.is_acc_init = true;

}

void CK_ACC_Update(void){

	if(acc.is_acc_init){

		acc.sync.syncCounter++;

		if(acc.sync.syncCounter >= acc.sync.syncRate){

	        #if defined(DEBUG_TIMING)
		    acc_debug.start_time = CK_TIME_GetMicroSec();
	        #endif

		    acc.sync.syncCounter = 0;

			// Read Raw Acc values
		    CK_ACC_ReadACCRaw();

			for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

				acc.accADC.v[axis] = acc.accADCRaw[axis];

				// Align Acc Axises
				acc.accADC.v[axis] *= acc.accSign[axis];

				if(axis != Z){
					acc.accADC.v[axis] -= acc.accADCZero[axis];
				}
				// Filter
				if(ACC_FILTERCUTOFF_IMU_100HZ){
					acc.accADC.v[axis] = pt2FilterApply(&acc_pt2filter[axis], acc.accADC.v[axis]);
				}

				// Calculate derivative of acc (jerk)
				acc.jerk.v[axis] = (acc.accADC.v[axis] - accAdcPrev.v[axis]) * acc.sampleRateHz;
				accAdcPrev.v[axis] = acc.accADC.v[axis];

				acc.accADCf[axis] = acc.accADC.v[axis];
				// Add to sum for IMU
				//acc.accAccumulate[axis] += acc.accADCf[axis];

			}


			acc.accMagnitude = vector3Norm(&acc.accADC) * acc.acc_1G_rec;
			acc.jerkMagnitude = vector3Norm(&acc.jerk) * acc.acc_1G_rec;

			//acc.accAccumulateCount++;

			CK_ACC_CheckTimeout();

	        #if defined(DEBUG_TIMING)
			acc_debug.update_time = CK_TIME_GetMicroSec() - acc_debug.start_time;
	        #endif

			acc.isAccelUpdatedAtLeastOnce = true;

		}

	}
}

void CK_ACC_UpdateFilter(uint8_t cutoff){

	const float k = pt2FilterGain(cutoff, HZ_TO_INTERVAL(HZ_TO_INTERVAL_US(acc.sync.targetLoopTime)));
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		pt2FilterUpdateCutoff(&acc_pt2filter[axis], k);
	}

}

void CK_ACC_ReadACCRaw(void){

#if USE_DMA_SENSOR

	if(acc.sensor == ICM20602_ACC){

		// Gyro read all sensor data at once in dma mode
		//CK_ICM20602_ReadSensorRaw_DMA();

	}
	else if(acc.sensor == IIM42652_ACC){

		// Gyro read all sensor data at once in dma mode
		//CK_IIM42652_ReadSensorRaw_DMA();
	}
	else if(acc.sensor == ICM42688P_ACC){
		// Gyro read all sensor data at once in dma mode
		//CK_ICM42688P_ReadSensorRaw_DMA();
	}
	else if(acc.sensor == LSM303D_ACC){

	}
	else if(acc.sensor == FXOS8700CQ_ACC){

	}

#else

	if(accSensor == ICM20602_ACC){

		CK_ICM20602_ReadAccRaw();

	}
	else if(accSensor == IIM42652_ACC){

		CK_IIM42652_ReadAccRaw();

	}
	else if(accSensor == ICM42688P_ACC){

		CK_ICM42688P_ReadAccRaw();

	}
	else if(accSensor == LSM303D_ACC){

		CK_LSM303D_ReadAccRaw();

	}
	else if(accSensor == FXOS8700CQ_ACC){

		CK_FXOS8700CQ_ReadAccRaw();

	}

#endif

}

void CK_ACC_ResetAccEarthSum(void){

	acc.accADCEarthSum[X] = 0.0f;
	acc.accADCEarthSum[Y] = 0.0f;
	acc.accADCEarthSum[Z] = 0.0f;
	acc.accADCEarthSumCounter = 0;
}

void CK_ACC_PerformCalibration(int16_t* acc_buffer){

	int32_t sum[XYZ_AXIS_COUNT] = {0,0,0};

	// 1 second read
	int32_t num_of_calibration = 1000000 / acc.sync.targetLoopTime;

	// Start calibration
	for(int i = 0; i < num_of_calibration; i++){

		// Read Raw Acc values
		// Read calibration with normal spi
		if(acc.sensor == ICM20602_ACC){

			CK_ICM20602_ReadAccRaw();

		}
		else if(acc.sensor == IIM42652_ACC){

			CK_IIM42652_ReadAccRaw();

		}
		else if(acc.sensor == ICM42688P_ACC){

			CK_ICM42688P_ReadAccRaw();

		}
		else if(acc.sensor == LSM303D_ACC){

			CK_LSM303D_ReadAccRaw();

		}
		else if(acc.sensor == FXOS8700CQ_ACC){

			CK_FXOS8700CQ_ReadAccRaw();

		}

		for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

			// Accumulate sum
			sum[axis] += acc.accADCRaw[axis];

		}
		CK_TIME_DelayMicroSec(acc.sync.targetLoopTime);
		if(i % 200 == 0){
		    CK_PRINTER_PrintString(".");
		}
	}

	// Get Final Calibration Values
	for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

		// Accumulate sum
		acc.accADCZero[axis]  = sum[axis] / num_of_calibration;

	}

	// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
	acc.accADCZero[X] = (sum[X] + (num_of_calibration / 2)) / num_of_calibration;
	acc.accADCZero[Y] = (sum[Y] + (num_of_calibration / 2)) / num_of_calibration;
	acc.accADCZero[Z] = (sum[Z] + (num_of_calibration / 2)) / num_of_calibration - acc.acc1G;

	acc_buffer[0] = acc.accADCZero[X];
	acc_buffer[1] = acc.accADCZero[Y];
	acc_buffer[2] = acc.accADCZero[Z];

}

void CK_ACC_BiquadLPFInit(biquadFilter_t* filterType, uint16_t filterFreq1, uint16_t filterFreq2, uint16_t refreshRate){

	const float notchQ = filterGetNotchQ(filterFreq1, filterFreq2);

	for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){

		// Initialize Notch Filter for each axis
		biquadFilterInit(filterType, filterFreq1, refreshRate, notchQ, FILTER_NOTCH, 1.0f);
	}
}

void CK_ACC_CheckTimeout(void){

	if(acc.is_acc_init){

		if(acc.sensor == ICM20602_ACC){

			// Check if any timeout occured
			if(CK_SPI_GetTimeOut(MOTION_ACC_SPI) == TIMEOUT_THREASHOLD){

				CK_BUZZER_Tone3();
				CK_SPI_ResetTimeOut(MOTION_ACC_SPI);

				if(acc.acc_reInit_counter  < TIMEOUT_REINIT_THREASHOLD){
					CK_ICM20602_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime);
					acc.acc_reInit_counter ++;
				}
			}
		}
		else if(acc.sensor == IIM42652_ACC){

			// Check if any timeout occured
			if(CK_SPI_GetTimeOut(MOTION_ACC_SPI) == TIMEOUT_THREASHOLD){

				CK_BUZZER_Tone3();
				CK_SPI_ResetTimeOut(MOTION_ACC_SPI);

				if(acc.acc_reInit_counter  < TIMEOUT_REINIT_THREASHOLD){
					CK_IIM42652_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime);
					acc.acc_reInit_counter ++;
				}
			}
		}
		else if(acc.sensor == ICM42688P_ACC){

			// Check if any timeout occured
			if(CK_SPI_GetTimeOut(MOTION_ACC_SPI) == TIMEOUT_THREASHOLD){

				CK_BUZZER_Tone3();
				CK_SPI_ResetTimeOut(MOTION_ACC_SPI);

				if(acc.acc_reInit_counter  < TIMEOUT_REINIT_THREASHOLD){
					CK_ICM42688P_AccInit(MOTION_ACC_SPI, MOTION_ACC_CS_PORT, MOTION_ACC_CS_PIN, acc.sync.targetLoopTime);
					acc.acc_reInit_counter ++;
				}
			}
		}


	}

	if(acc.is_acc_init){

		if(acc.sensor == LSM303D_ACC || acc.sensor == FXOS8700CQ_ACC){

			// Check if any timeout occured
			if(CK_I2C_GetTimeOut(MOTION_ACC_I2C) == TIMEOUT_THREASHOLD){

				CK_BUZZER_Tone3();
				CK_I2C_ResetTimeOut(MOTION_ACC_I2C);

				if(acc.acc_reInit_counter  < TIMEOUT_REINIT_THREASHOLD){
					if(acc.sensor == LSM303D_ACC){
						CK_LSM303D_AccInit(MOTION_ACC_I2C, acc.sync.targetLoopTime);
					}
					else if(acc.sensor == FXOS8700CQ_ACC){
						CK_FXOS8700CQ_AccInit(MOTION_ACC_I2C, acc.sync.targetLoopTime);
					}
					acc.acc_reInit_counter ++;
				}
			}
		}
	}
}

