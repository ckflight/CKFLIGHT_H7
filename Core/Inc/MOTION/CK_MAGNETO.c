
#include <COMMON/maths.h>
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_SPI.h"

#include "SENSORS/CK_MAG3110.h"
#include "SENSORS/CK_FXOS8700CQ.h"
#include "SENSORS/CK_LSM303D.h"
#include "SENSORS/CK_HMC5983.h"
#include "SENSORS/CK_QMC5883L.h"
#include "SENSORS/CK_MLX90393.h"

#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_IMU.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define TIMEOUT_THREASHOLD                  10 // 10 Timeout starts reInit process
#define TIMEOUT_REINIT_THREASHOLD           3  // 3 Times max tries to reInit

magnetoSensor_t mag;

/********** MLX90393 Calibration Parameters **********/

// Offsets applied to raw x/y/z values
float mlx90393_offset[3]             = {-14.10f, -11.24f, +32.07f};

float mlx90393_softiron_matrix[3][3] = {{+0.835, +0.010, -0.027},
                                       {+0.011, +0.847, -0.007},
                                       {-0.028, -0.008, +1.409}};

float mlx90393_field_strength        = 23.02f;

/********** MLX90393 Calibration Parameters **********/

// Offsets applied to raw x/y/z values
float mlx90393_offset2[3]             = {-3.43f, +6.27f, +13.09f};

float mlx90393_softiron_matrix2[3][3] = {{+0.871, +0.008, -0.020},
                                       {+0.008, +0.857, +0.049},
                                       {-0.020, +0.049, +1.343}};

float mlx90393_field_strength2        = 22.00f;

/********** MAG3110 Calibration Parameters **********/

// Offsets applied to raw x/y/z values
float mag3110_offset[3]             = {+197.47f, -235.56f, +71.40f};

float mag3110_softiron_matrix[3][3] = {{+0.970, -0.032, +0.060},
                                       {-0.032, +1.051, +0.041},
                                       {+0.060, +0.041, +0.987}};

float mag3110_field_strength        = 50.30f;


/********** LSM303D Calibration Parameters **********/

// Offsets applied to raw x/y/z values
float lsm303d_offset[3]             = {13.21f, 10.74f, 32.22f};

// Soft iron error compensation matrix
float lsm303d_softiron_matrix[3][3] = {{+0.962f, -0.006f, +0.018f},
                                       {-0.006f, +0.996f, +0.002f},
                                       {+0.018f, +0.002f, +1.043f}};

float lsm303d_field_strength        = 56.16f;


/********** QMC5883L Calibration Parameters ***********/

// Offsets applied to raw x/y/z values
float qmc5883l_offset[3]             = {-7.12f, 53.44f, -0.45f};

// Soft iron error compensation matrix
float qmc5883l_softiron_matrix[3][3] = {{+0.988f, -0.005f, -0.010f},
                                        {-0.005f, +0.983f, -0.016f},
                                        {-0.010f, -0.016f, +1.031f}};

float qmc5883l_field_strength        = 63.99f;


/********** FXO8700CQ Calibration Parameters ***********/

// Offsets applied to raw x/y/z values
float fxos8700cq_offset[3]             = {18.92f, -9.66f, 55.33f};

// Soft iron error compensation matrix
float fxos8700cq_softiron_matrix[3][3] = {{+0.982, -0.004, +0.006},
                                          {-0.004, +0.992, +0.003},
                                          {+0.006, +0.003, +1.027}};

float fxos8700cq_field_strength        = 42.43f;


/********** HMC5983 Calibration Parameters ***********/

// Offsets applied to raw x/y/z values
float hmc5983_offset[3]             = {13.02f, -11.73f, -16.16f};

// Soft iron error compensation matrix
float hmc5983_softiron_matrix[3][3] = {{+1.029, +0.027, +0.038},
                                       {+0.027, +1.034, -0.002},
                                       {+0.038, -0.002, +0.941}};

float hmc5983_field_strength        = 52.97f;



magnetometer_calibratio_t calibration_parameters;

DEBUG_TIME_t mag_debug;

SPI_TypeDef* MOTION_MAG_SPI;
GPIO_TypeDef* MOTION_MAG_CS_PORT;
uint8_t MOTION_MAG_CS_PIN;

I2C_TypeDef* MOTION_MAG_I2C;


/*
 *  Magnetometer iron calibration is a must and each board has different calibration.
 *  After the calibration IMU heading starts working.
 *  Calibrated data changes sign axis. It is not just about alligning at correct orientation.
 *
 *  IMU allignment is X down Y right Z up
 *
 *  Use MotionCal program. Printer 'c' prints according to its format.
 *  Notes -> Motion Calibration -> MotionCal.dmg
 *
 *  Some magnetometers' raw data fits to Calibration tools but some don't.
 *  Scale data if it is necessary
 *
 */

void CK_MAGNETO_Init(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, sensorModel_e sensor, uint32_t magT, uint32_t mainT){

	MOTION_MAG_SPI 		= spin_;
	MOTION_MAG_CS_PORT 	= cs_gpio_;
	MOTION_MAG_CS_PIN  	= cs_pin_;

	mag.is_mag_spi_init = false;

    mag.sync.syncCounter = 0;

    mag.sync.targetLoopTime = magT;

    mag.sync.syncRate = magT / mainT;

	mag.sensor = sensor;

	mag.mag_reInit_counter = 0;

	if(mag.sensor == MLX90393_MAGNETO){

		if(!CK_MLX90393_isMagSensorInitialized()){
			CK_MLX90393_Init(MOTION_MAG_SPI, MOTION_MAG_CS_PORT, MOTION_MAG_CS_PIN, mag.sync.targetLoopTime);
		}

	}

	mag.magSign[X] 	= MAG_ORIENTATION_X_SIGN;
	mag.magSign[Y] 	= MAG_ORIENTATION_Y_SIGN;
	mag.magSign[Z] 	= MAG_ORIENTATION_Z_SIGN;
	mag.is_magswapaxis 		= MAG_ORIENTATION_SWAP_XY;

	CK_MAGNETO_LoadCalibrationParameters();

	mag.is_mag_spi_init = true;

}

void CK_MAGNETO_Init2(I2C_TypeDef* i2cn_, sensorModel_e sensor, uint32_t magT, uint32_t mainT){

	MOTION_MAG_I2C = i2cn_;

	mag.is_mag_i2c_init = false;

    mag.sync.syncCounter = 0;

    mag.sync.targetLoopTime = magT;

    mag.sync.syncRate = magT / mainT;

	mag.sensor = sensor;

	mag.mag_reInit_counter = 0;

	if(mag.sensor == MAG3110_MAGNETO){

	    if(!CK_MAG3110_isMagSensorInitialized()){
	        CK_MAG3110_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	    }

	}
	else if(mag.sensor == HMC5983_MAGNETO){

	    if(!CK_HMC5983_isMagSensorInitialized()){
	        CK_HMC5983_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	    }

	}
	else if(mag.sensor == QMC5883L_MAGNETO){

	    if(!CK_QMC5883L_isMagSensorInitialized()){
	        CK_QMC5883L_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	    }

	}
	else if(mag.sensor == LSM303D_MAGNETO){

		if(!CK_LSM303D_isMagSensorInitialized()){
			CK_LSM303D_MagInit(MOTION_MAG_I2C, mag.sync.targetLoopTime);
		}

	}
	else if(mag.sensor == FXOS8700CQ_MAGNETO){

		if(!CK_FXOS8700CQ_isMagSensorInitialized()){
			CK_FXOS8700CQ_MagInit(MOTION_MAG_I2C, mag.sync.targetLoopTime);
		}

	}

	mag.magSign[X] 	= MAG_ORIENTATION_X_SIGN;
	mag.magSign[Y] 	= MAG_ORIENTATION_Y_SIGN;
	mag.magSign[Z] 	= MAG_ORIENTATION_Z_SIGN;
	mag.is_magswapaxis 		= MAG_ORIENTATION_SWAP_XY;

	CK_MAGNETO_LoadCalibrationParameters();

	mag.is_mag_i2c_init = true;

}

void CK_MAGNETO_LoadCalibrationParameters(void){

    if(mag.sensor == MLX90393_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = mlx90393_offset2[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = mlx90393_softiron_matrix2[i][a];
            }
        }

        calibration_parameters.field_strength = mlx90393_field_strength2;
    }
    else if(mag.sensor == MAG3110_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = mag3110_offset[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = mag3110_softiron_matrix[i][a];
            }
        }

        calibration_parameters.field_strength = mag3110_field_strength;
    }
    else if(mag.sensor == QMC5883L_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = qmc5883l_offset[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = qmc5883l_softiron_matrix[i][a];
            }
        }

        calibration_parameters.field_strength = qmc5883l_field_strength;
    }
    else if(mag.sensor == LSM303D_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = lsm303d_offset[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = lsm303d_softiron_matrix[i][a];
            }
        }

        calibration_parameters.field_strength = lsm303d_field_strength;
    }
    else if(mag.sensor == HMC5983_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = hmc5983_offset[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = hmc5983_softiron_matrix[i][a];
            }
        }

        calibration_parameters.field_strength = hmc5983_field_strength;
    }
    else if(mag.sensor == FXOS8700CQ_MAGNETO){
        for(int i = 0; i < 3; i++){
            calibration_parameters.offset[i] = fxos8700cq_offset[i];
        }
        for(int i = 0; i < 3; i++){
            for(int a = 0; a < 3; a++){
                calibration_parameters.softiron_matrix[i][a] = fxos8700cq_softiron_matrix[i][a];
            }
        }

        calibration_parameters.field_strength = fxos8700cq_field_strength;
    }

}

void CK_MAGNETO_Update(void){

	if(mag.is_mag_spi_init || mag.is_mag_i2c_init){

	    mag.sync.syncCounter++;

		if(mag.sync.syncCounter >= mag.sync.syncRate){

	        #if defined(DEBUG_TIMING)
		    mag_debug.start_time = CK_TIME_GetMicroSec();
	        #endif

		    mag.sync.syncCounter = 0;

		    if(mag.sensor == MAG3110_MAGNETO){

				CK_MAG3110_ReadMagRaw();

	            /*
	             * MAG3110 Placement in CK_BOARD1 and 2 is X right Y up but
	             * my IMU orientation is Y right X is down
	             * so i need to swap X and Y and X needs to be opposit direction.
	             *
	             */

				// Get raw readings
				mag.magADCf[X] = mag.magADCRaw[X];
				mag.magADCf[Y] = mag.magADCRaw[Y];
				mag.magADCf[Z] = mag.magADCRaw[Z];

				// Convert to microTesla 0.1 microTesla/LSB
				mag.magADCf[X] *= mag.magScale[0];// 0.1f
				mag.magADCf[Y] *= mag.magScale[0];
				mag.magADCf[Z] *= mag.magScale[0];

				// Apply mag offset compensation (base values is raw data)
				float x = mag.magADCf[X] - calibration_parameters.offset[X];
				float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
				float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

				// Apply mag soft iron error compensation
				mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
				mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
				mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

				if(mag.is_magswapaxis){

					float temp_x = mag.magADCf[X];
					float temp_y = mag.magADCf[Y];

					mag.magADCf[X] = temp_y;
					mag.magADCf[Y] = temp_x;

				}

				// Align Magnometer
				mag.magADCf[X] *= mag.magSign[X];
				mag.magADCf[Y] *= mag.magSign[Y];
				mag.magADCf[Z] *= mag.magSign[Z];


				CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

			}
			else if(mag.sensor == MLX90393_MAGNETO){

		    	static int counter = 1;

		    	if(counter == 0){
		    		counter++;
		    		CK_MLX90393_StartSingleConversion();
		    	}
		    	else{
		    		counter = 0;
		    		CK_MLX90393_StartSingleConversion();
		    		CK_MLX90393_ReadMag();

		    		// Get raw readings
					mag.magADCf[X] = mag.magADCRaw[X];
					mag.magADCf[Y] = mag.magADCRaw[Y];
					mag.magADCf[Z] = mag.magADCRaw[Z];


					mag.magADCf[X] *= mag.magScale[X];
					mag.magADCf[Y] *= mag.magScale[Y];
					mag.magADCf[Z] *= mag.magScale[Z];


					// Apply mag offset compensation (base values is raw data)
					float x = mag.magADCf[X] - calibration_parameters.offset[X];
					float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
					float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

					// Apply mag soft iron error compensation
					mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
					mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
					mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

					if(mag.is_magswapaxis){

						float temp_x = mag.magADCf[X];
						float temp_y = mag.magADCf[Y];

						mag.magADCf[X] = temp_y;
						mag.magADCf[Y] = temp_x;

					}

					// Align Magnometer
					mag.magADCf[X] *= mag.magSign[X];
					mag.magADCf[Y] *= mag.magSign[Y];
					mag.magADCf[Z] *= mag.magSign[Z];


					CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

		    	}

		    }

			else if(mag.sensor == QMC5883L_MAGNETO){

				CK_QMC5883L_ReadMagRaw();

	            /*
	             * IMU orientation is Y right X is down Z is up
	             * Only swapping is necessary and reversing the z direction. See TS100 hardware image.
	             *
	             * With the current calibration it works.
	             *
	             */

				// Get raw readings
				mag.magADCf[X] = mag.magADCRaw[X];
				mag.magADCf[Y] = mag.magADCRaw[Y];
				mag.magADCf[Z] = mag.magADCRaw[Z];

				// Convert raw to gauss
				mag.magADCf[X] *= mag.magScale[0];
				mag.magADCf[Y] *= mag.magScale[0];
				mag.magADCf[Z] *= mag.magScale[0];

				// Convert gauss to microTesla
				mag.magADCf[X] *= 100.0f;
				mag.magADCf[Y] *= 100.0f;
				mag.magADCf[Z] *= 100.0f;

				// Apply mag offset compensation (base values is raw data)
				float x = mag.magADCf[X] - calibration_parameters.offset[X];
				float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
				float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

				// Apply mag soft iron error compensation
				mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
				mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
				mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

				if(mag.is_magswapaxis){

					float temp_x = mag.magADCf[X];
					float temp_y = mag.magADCf[Y];

					mag.magADCf[X] = temp_y;
					mag.magADCf[Y] = temp_x;

				}

				// Align Magnometer
				mag.magADCf[X] *= mag.magSign[X];
				mag.magADCf[Y] *= mag.magSign[Y];
				mag.magADCf[Z] *= mag.magSign[Z];

				CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

			}

			else if(mag.sensor == HMC5983_MAGNETO){

				CK_HMC5983_ReadMagRaw();

				// Get raw readings
				mag.magADCf[X] = mag.magADCRaw[X];
				mag.magADCf[Y] = mag.magADCRaw[Y];
				mag.magADCf[Z] = mag.magADCRaw[Z];

				// Convert raw to gauss
				mag.magADCf[X] *= mag.magScale[0];
				mag.magADCf[Y] *= mag.magScale[0];
				mag.magADCf[Z] *= mag.magScale[0];

				// Convert gauss to microTesla
				mag.magADCf[X] *= 100.0f;
				mag.magADCf[Y] *= 100.0f;
				mag.magADCf[Z] *= 100.0f;

				// Apply mag offset compensation (base values is raw data)
				float x = mag.magADCf[X] - calibration_parameters.offset[X];
				float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
				float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

				// Apply mag soft iron error compensation
				mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
				mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
				mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

				if(mag.is_magswapaxis){

					float temp_x = mag.magADCf[X];
					float temp_y = mag.magADCf[Y];

					mag.magADCf[X] = temp_y;
					mag.magADCf[Y] = temp_x;

				}

				// Align Magnometer
				mag.magADCf[X] *= mag.magSign[X];
				mag.magADCf[Y] *= mag.magSign[Y];
				mag.magADCf[Z] *= mag.magSign[Z];

				CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

			}

			else if(mag.sensor == LSM303D_MAGNETO){

				CK_LSM303D_ReadMagRaw();

				/*
	             * IMU orientation is Y right X is down Z is up
	             *
	             * With the current calibration it works.
	             *
	             */

				// Get raw readings
				mag.magADCf[X] = mag.magADCRaw[X];
				mag.magADCf[Y] = mag.magADCRaw[Y];
				mag.magADCf[Z] = mag.magADCRaw[Z];

				// Convert to gauss 1100 LSB/gauss for x,y 980 LSB/gauss for z
				for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

					// Convert to microTesla
					if(axis == X || axis == Y){
						mag.magADCf[axis] *= mag.magScale[X];
					}
					else{
						mag.magADCf[axis] *= mag.magScale[Y];
					}
				}

				// Convert gauss to microTesla 1Gauss = 100microTesla
				mag.magADCf[X] *= 100.0f;
				mag.magADCf[Y] *= 100.0f;
				mag.magADCf[Z] *= 100.0f;


				// Apply mag offset compensation (base values is raw data)
				float x = mag.magADCf[X] - calibration_parameters.offset[X];
				float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
				float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

				// Apply mag soft iron error compensation
				mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
				mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
				mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

				// Align Magnometer
				mag.magADCf[X] *= mag.magSign[X];
				mag.magADCf[Y] *= mag.magSign[Y];
				mag.magADCf[Z] *= mag.magSign[Z];

				CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

			}

			else if(mag.sensor == FXOS8700CQ_MAGNETO){

				// Reading acc is slow later separate them
				CK_FXOS8700CQ_ReadMagRaw();

				// Get raw readings
				mag.magADCf[X] = mag.magADCRaw[X];
				mag.magADCf[Y] = mag.magADCRaw[Y];
				mag.magADCf[Z] = mag.magADCRaw[Z];

				// Convert to microTesla 0.1 microTesla/LSB
				mag.magADCf[X] *= mag.magScale[0];
				mag.magADCf[Y] *= mag.magScale[0];
				mag.magADCf[Z] *= mag.magScale[0];

				// Apply mag offset compensation (base values is raw data)
				float x = mag.magADCf[X] - calibration_parameters.offset[X];
				float y = mag.magADCf[Y] - calibration_parameters.offset[Y];
				float z = mag.magADCf[Z] - calibration_parameters.offset[Z];

				// Apply mag soft iron error compensation
				mag.magADCf[X] = x * calibration_parameters.softiron_matrix[0][0] + y * calibration_parameters.softiron_matrix[0][1] + z * calibration_parameters.softiron_matrix[0][2];
				mag.magADCf[Y] = x * calibration_parameters.softiron_matrix[1][0] + y * calibration_parameters.softiron_matrix[1][1] + z * calibration_parameters.softiron_matrix[1][2];
				mag.magADCf[Z] = x * calibration_parameters.softiron_matrix[2][0] + y * calibration_parameters.softiron_matrix[2][1] + z * calibration_parameters.softiron_matrix[2][2];

				if(mag.is_magswapaxis){

					float temp_x = mag.magADCf[X];
					float temp_y = mag.magADCf[Y];

					mag.magADCf[X] = temp_y;
					mag.magADCf[Y] = temp_x;

				}

				// Align Magnometer
				mag.magADCf[X] *= mag.magSign[X];
				mag.magADCf[Y] *= mag.magSign[Y];
				mag.magADCf[Z] *= mag.magSign[Z];


				CK_MAGNETO_CalculateHeading(mag.magADCf[X], mag.magADCf[Y], mag.magADCf[Z]);

			}

			CK_MAGNETO_CheckTimeout();

	        #if defined(DEBUG_TIMING)
			mag_debug.update_time = CK_TIME_GetMicroSec() - mag_debug.start_time;
	        #endif

		}


	}

}

void CK_MAGNETO_SwapXYAxis(void){

	float temp_x = mag.magADCf[X];

	mag.magADCf[X] = mag.magADCf[Y];

	mag.magADCf[Y] = temp_x;

}

void CK_MAGNETO_PerformCalibration(void){

    const int calibration_duration = 60;
    int32_t numOfCalibration = calibration_duration * (1000/100);
	CK_PRINTER_PrintlnString("Magneto Calibration Started for 60 sec.");
	CK_PRINTER_PrintlnString("Start moving drone in 8 figures.");

	// Initialize max min values at reverse max values
	for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

		mag.magADCMin[axis] = 32768;
		mag.magADCMax[axis] = -32768;
	}

	// Start calibration
	for(int i = 0; i < numOfCalibration; i++){

		if(mag.sensor == MLX90393_MAGNETO){
			static int counter = 0;

	    	if(counter == 0){
	    		counter++;
	    		CK_MLX90393_StartSingleConversion();
	    	}
	    	else{
	    		counter = 0;

	    		CK_MLX90393_ReadMag();
	    	}
		}
		else if(mag.sensor == MAG3110_MAGNETO){

			CK_MAG3110_ReadMagRaw();
		}
		else if(mag.sensor == HMC5983_MAGNETO){

			CK_HMC5983_ReadMagRaw();
		}
		else if(mag.sensor == QMC5883L_MAGNETO){

			CK_QMC5883L_ReadMagRaw();
		}
		else if(mag.sensor == LSM303D_MAGNETO){

			CK_LSM303D_ReadMagRaw();
		}
		else if(mag.sensor == FXOS8700CQ_MAGNETO){

			CK_FXOS8700CQ_ReadMagRaw();
		}

		for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

			if(mag.magADCRaw[axis] < mag.magADCMin[axis]){
				mag.magADCMin[axis] = mag.magADCRaw[axis];
			}
			if(mag.magADCRaw[axis] > mag.magADCMax[axis]){
				mag.magADCMax[axis] = mag.magADCRaw[axis];
			}
		}
		CK_TIME_DelayMilliSec(100);
		CK_PRINTER_PrintString(".");
		if(i % 100 == 0){
		    CK_PRINTER_PrintlnString("");
		}
	}


	// Calculate Hard Iron Offsets
	CK_PRINTER_PrintString("HardIron: ");
	for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){

	    mag.magHardIron[axis] = (float)((mag.magADCMax[axis] + mag.magADCMin[axis]) / 2);
	    CK_PRINTER_PrintFloat(mag.magHardIron[axis]);CK_PRINTER_PrintString(",");
	}
	CK_PRINTER_PrintlnString("");

	// Calculate Soft Iron Offsets
	CK_PRINTER_PrintString("SoftIron: ");
	int16_t mag_scale[XYZ_AXIS_COUNT];
    for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){
        mag_scale[axis] = (mag.magADCMax[axis] - mag.magADCMin[axis]) / 2;
    }

    float average_scale = (mag_scale[X] + mag_scale[Y] + mag_scale[Z]) / 3;

    for(int axis = X; axis < XYZ_AXIS_COUNT; axis++){
        mag.magSoftIron[axis] = (float)(average_scale / mag_scale[axis]);
        CK_PRINTER_PrintFloat(mag.magSoftIron[axis]);CK_PRINTER_PrintString(",");
    }
    CK_PRINTER_PrintlnString("");

    CK_PRINTER_PrintlnString("MAGNETOMETER CALIBRATION DONE!");

}

void CK_MAGNETO_CalculateHeading(float mx, float my, float mz){

	float roll_rad  = attitude.values.roll  / 572.9578f; //in radians/s
	float pitch_rad = attitude.values.pitch / 572.9578f; //in radians/s

	// This compensate tilt for fixed north direction and tilt in nort direction
	float mx_compensated = (mx * cosf(pitch_rad)) + (mz * sinf(pitch_rad));
	float my_compensated = (mx * sinf(roll_rad) * sinf(pitch_rad)) + (my * cosf(roll_rad)) - (mz * sinf(roll_rad) * cosf(pitch_rad));

	float magX = mx_compensated;
	float magY = my_compensated;

	float heading = 0;

	if(magX == 0){
		if(magY < 0){
		  heading = 90;
		}
		else if(magY >= 0){
		  heading = 0;
		}
	}
	else if(magX != 0){
		// Find here: http://www.magnetic-declination.com/
		// Magnetic declination: 5° 41' EAST (POSITIVE);  1 degreee = 0.0174532925 radians

		#define DEC_ANGLE 0.099
		heading = atan2(magY, magX);

#if defined(CK_BOARD) || defined(CK_BOARD2) || defined(CK_BOARD4)
		heading -= DEC_ANGLE;
#endif

		heading *= (180.0f / M_PIf);
	}

	if(heading > 360.0f){
		heading -= 360.0f;
	}
	if(heading < 0.0f){
		heading += 360.0f;
	}

#if defined(CK_BOARD) || defined(CK_BOARD2) || defined(CK_BOARD4)
	mag.magHeading = (int) (360.0f - heading);

#endif

}

void CK_MAGNETO_CheckTimeout(void){

	if(mag.is_mag_i2c_init){

		if(CK_I2C_GetTimeOut(MOTION_MAG_I2C) == TIMEOUT_THREASHOLD){

	    	CK_BUZZER_Tone3();
	        CK_I2C_ResetTimeOut(MOTION_MAG_I2C);

	        if(mag.mag_reInit_counter < TIMEOUT_REINIT_THREASHOLD){

	        	if(mag.sensor == MAG3110_MAGNETO){
	            	CK_MAG3110_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	        	}
	        	else if(mag.sensor == HMC5983_MAGNETO){
	        		CK_HMC5983_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	        	}
	        	else if(mag.sensor == QMC5883L_MAGNETO){
	        		CK_QMC5883L_Init(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	        	}
	        	else if(mag.sensor == LSM303D_MAGNETO){
	        		CK_LSM303D_MagInit(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	        	}
	        	else if(mag.sensor == FXOS8700CQ_MAGNETO){
	        		CK_FXOS8700CQ_MagInit(MOTION_MAG_I2C, mag.sync.targetLoopTime);
	        	}

	            mag.mag_reInit_counter++;
	        }
	    }
	}

	if(mag.is_mag_spi_init){

		if(CK_SPI_GetTimeOut(MOTION_MAG_SPI) == TIMEOUT_THREASHOLD){

			CK_BUZZER_Tone3();
			CK_SPI_ResetTimeOut(MOTION_MAG_SPI);

			if(mag.mag_reInit_counter < TIMEOUT_REINIT_THREASHOLD){

				if(mag.sensor == MLX90393_MAGNETO){
					CK_MLX90393_Init(MOTION_MAG_SPI, MOTION_MAG_CS_PORT, MOTION_MAG_CS_PIN, mag.sync.targetLoopTime);
				}
				mag.mag_reInit_counter++;
			}
		}
	}

}












