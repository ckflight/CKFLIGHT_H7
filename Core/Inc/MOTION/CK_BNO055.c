
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "MOTION/CK_BNO055.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define BNO055_ADDRESS              0x28
#define BNO055_WHO_AM_I_REG         0x00
#define BNO055_WHO_AM_I_ID          0xA0
#define BNO055_OPR_MODE_REG         0x3D
#define BNO055_TEMP_SOURCE_REG      0x40
#define BNO055_UNIT_SEL_REG         0x3B
#define BNO055_PWR_MODE_REG         0x3E
#define BNO055_SYS_TRIGGER_REG      0x3F
#define BNO055_CALIB_STAT_REG       0x35
#define BNO055_EUL_DATA_X_LSB       0x1A

#define TIMEOUT_THREASHOLD                  10 // 10 Timeout starts reInit process
#define TIMEOUT_REINIT_THREASHOLD           3  // 3 Times max tries to reInit
#define READ_ARRAY_SIZE                     22

bno055Sensor_t bno055;

// Send 'z' command to start calibration and save the results to variables below
// Comment three line when calibration will be called since it writes calibration results and
// calibration's while loop will be terminated when they are calibrated

int16_t acc_x_offset = -27;
int16_t acc_y_offset = 8;
int16_t acc_z_offset = 21;

int16_t mag_x_offset = 625;
int16_t mag_y_offset = 611;
int16_t mag_z_offset = 497;

int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = -1;
int16_t gyro_z_offset = 0;

int16_t acc_radius = 1000;
int16_t mag_radius = 728;

DEBUG_TIME_t bno055_debug;

I2C_TypeDef* MOTION_BNO055_I2C;

typedef struct{

    bool BNO055Init;

    uint8_t rxArray[READ_ARRAY_SIZE];

    int BNO055_reInitCounter;


}BNO055_PARAMETERS_t;

BNO055_PARAMETERS_t bno055_var = {
    .BNO055Init = false,
    .rxArray = {0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0},

    .BNO055_reInitCounter = 0,
};

uint32_t bno055_mainT; // needed if reinit

void CK_BNO055_Init(I2C_TypeDef* i2cn_, uint32_t bnoT, uint32_t mainT){

	MOTION_BNO055_I2C = i2cn_;

    bno055_mainT = mainT;

    bno055.sync.syncCounter = 0;

    bno055.sync.targetLoopTime = bnoT;

    bno055.sync.syncRate = bnoT / mainT;

	CK_I2C_ReadMulti(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_WHO_AM_I_REG, bno055_var.rxArray, 1);

	if(bno055_var.rxArray[0] == BNO055_WHO_AM_I_ID){

		bno055.angleScale = 16.0f;

		// Register Setup before Active Mode
		CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_OPR_MODE_REG, 0x0C); // NDOF Mode

		CK_TIME_DelayMilliSec(100);

		CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_TEMP_SOURCE_REG, 0x01); // Gyro sensor as temp source (page 37)

		CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_UNIT_SEL_REG, 0x01); // Fusion output windows,mg,dps,Euler Angles in degree,celcius (page 30)

		CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_PWR_MODE_REG, 0x00); // Normal

		CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_SYS_TRIGGER_REG, 0x80); // CLK_SEL external 32.768KHz

		// Comment these three lines when calibration will be called
		CK_BNO055_CONFIGMode();

		CK_BNO055_WriteSensorOffsetAndRadius();

		CK_BNO055_NDOFMode();

		bno055_var.BNO055Init = true;

	}
	else{
	    CK_PRINTER_PrintlnString("BNO055 ERROR");
	}

}

void CK_BNO055_Update(void){

	if(bno055_var.BNO055Init){

		bno055.sync.syncCounter++;

		if(bno055.sync.syncCounter >= bno055.sync.syncRate){

	        #if defined(DEBUG_TIMING)
		    bno055_debug.start_time = CK_TIME_GetMicroSec();
	        #endif

		    bno055.sync.syncCounter = 0;

			int16_t x_raw;

			int16_t y_raw;

			int16_t z_raw;

			CK_BNO055_ReadEulerRaw(&z_raw, &x_raw, &y_raw);


			bno055.eulerAngles[FD_ROLL]  = -x_raw / bno055.angleScale; // Roll was in negative direction

			bno055.eulerAngles[FD_PITCH] = y_raw / bno055.angleScale;

			bno055.eulerAngles[FD_YAW]   = z_raw / bno055.angleScale;

	        #if defined(DEBUG_TIMING)
			bno055_debug.update_time = CK_TIME_GetMicroSec() - bno055_debug.start_time;
	        #endif

		}

	}

}

void CK_BNO055_ReadEulerRaw(int16_t* p_x, int16_t* p_y, int16_t* p_z){

	CK_I2C_ReadMulti(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_EUL_DATA_X_LSB, bno055_var.rxArray, 6);

	*p_x = (int16_t)(bno055_var.rxArray[1] << 8 | bno055_var.rxArray[0]);

	*p_y = (int16_t)(bno055_var.rxArray[3] << 8 | bno055_var.rxArray[2]);

	*p_z = (int16_t)(bno055_var.rxArray[5] << 8 | bno055_var.rxArray[4]);

    if(CK_I2C_GetTimeOut(MOTION_BNO055_I2C) == TIMEOUT_THREASHOLD){
    	CK_BUZZER_Tone3();
        CK_I2C_ResetTimeOut(MOTION_BNO055_I2C);

        if(bno055_var.BNO055_reInitCounter < TIMEOUT_REINIT_THREASHOLD){
            CK_BNO055_Init(MOTION_BNO055_I2C, bno055.sync.targetLoopTime, bno055_mainT);
            bno055_var.BNO055_reInitCounter++;
        }

    }

}

void CK_BNO055_Calibrate(void){

    CK_PRINTER_PrintString("BNO055 Calibration Starting");

	while(bno055.calibration_sysStatus < 3 || bno055.calibration_magStatus < 3 ||
		  bno055.calibration_gyroStatus < 3 || bno055.calibration_accStatus < 3){

		CK_I2C_ReadMulti(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_CALIB_STAT_REG, bno055_var.rxArray, 1);

		bno055.calibration_magStatus  = (bno055_var.rxArray[0] & 0x03);

		bno055.calibration_accStatus  = (bno055_var.rxArray[0] & 0x0C) >> 2;

		bno055.calibration_gyroStatus = (bno055_var.rxArray[0] & 0x30) >> 4;

		bno055.calibration_sysStatus  = (bno055_var.rxArray[0] & 0xC0) >> 6;

		CK_PRINTER_PrintString("  Mag: ");CK_PRINTER_PrintInt(bno055.calibration_magStatus);

		CK_PRINTER_PrintString("  Acc: ");CK_PRINTER_PrintInt(bno055.calibration_accStatus);

		CK_PRINTER_PrintString("  Gyro: ");CK_PRINTER_PrintInt(bno055.calibration_gyroStatus);

		CK_PRINTER_PrintString("  Sys: ");CK_PRINTER_PrintlnInt(bno055.calibration_sysStatus);

		CK_TIME_DelayMilliSec(100);

	}

	/* 	BNO055 Datasheet Page 49
	 *
	 *	READING CALIBRATIN RESULTS:
	 *  The calibration profile includes sensor offsets and sensor radius.
	 *  Host system can read the offsets and radius only after a full calibration is achieved
	 *  and the operation mode is switched to CONFIG_MODE.
	 *  Refer to sensor offsets and sensor radius registers.
	 *
	 *  WRITING CALIBRATION RESULTS:
	 *  1. Select the operation mode to CONFIG_MODE
	 *	2. Write the corresponding sensor offsets and radius data
	 *	3. Change operation mode to fusion mode
	 *
	 */
	CK_BNO055_CONFIGMode();

	CK_BNO055_ReadSensorOffsetAndRadius();

	CK_BNO055_NDOFMode();

	CK_PRINTER_PrintString("BNO055 Calibration Done!");

}

void CK_BNO055_CONFIGMode(void){

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_OPR_MODE_REG, 0x00); // Config Mode

	CK_TIME_DelayMilliSec(100);

}

void CK_BNO055_NDOFMode(void){

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, BNO055_OPR_MODE_REG, 0x0C); // NDOF Mode

	CK_TIME_DelayMilliSec(100);

}

void CK_BNO055_ReadSensorOffsetAndRadius(void){

	// Calibration registers are starting from 0x55 to 0x6A 22 registers
	CK_I2C_ReadMulti(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x55, bno055_var.rxArray, 22);

	int16_t accX_offset  = (int16_t)((bno055_var.rxArray[1] << 8)  | bno055_var.rxArray[0]);
	int16_t accY_offset  = (int16_t)((bno055_var.rxArray[3] << 8)  | bno055_var.rxArray[2]);
	int16_t accZ_offset  = (int16_t)((bno055_var.rxArray[5] << 8)  | bno055_var.rxArray[4]);

	int16_t magX_offset  = (int16_t)((bno055_var.rxArray[7] << 8)  | bno055_var.rxArray[6]);
	int16_t magY_offset  = (int16_t)((bno055_var.rxArray[9] << 8)  | bno055_var.rxArray[8]);
	int16_t magZ_offset  = (int16_t)((bno055_var.rxArray[11] << 8) | bno055_var.rxArray[10]);

	int16_t gyroX_offset = (int16_t)((bno055_var.rxArray[13] << 8) | bno055_var.rxArray[12]);
	int16_t gyroY_offset = (int16_t)((bno055_var.rxArray[15] << 8) | bno055_var.rxArray[14]);
	int16_t gyroZ_offset = (int16_t)((bno055_var.rxArray[17] << 8) | bno055_var.rxArray[16]);

	int16_t accRadius    = (int16_t)((bno055_var.rxArray[19] << 8) | bno055_var.rxArray[18]);
	int16_t magRadius    = (int16_t)((bno055_var.rxArray[21] << 8) | bno055_var.rxArray[20]);


	CK_PRINTER_PrintString(" Acc_x: ");CK_PRINTER_PrintInt(accX_offset);
	CK_PRINTER_PrintString(" y: ");CK_PRINTER_PrintInt(accY_offset);
	CK_PRINTER_PrintString(" z: ");CK_PRINTER_PrintInt(accZ_offset);

	CK_PRINTER_PrintString(" Mag_x: ");CK_PRINTER_PrintInt(magX_offset);
	CK_PRINTER_PrintString(" y: ");CK_PRINTER_PrintInt(magY_offset);
	CK_PRINTER_PrintString(" z: ");CK_PRINTER_PrintInt(magZ_offset);

	CK_PRINTER_PrintString(" Gyro_x: ");CK_PRINTER_PrintInt(gyroX_offset);
	CK_PRINTER_PrintString(" y: ");CK_PRINTER_PrintInt(gyroY_offset);
	CK_PRINTER_PrintString(" z: ");CK_PRINTER_PrintInt(gyroZ_offset);

	CK_PRINTER_PrintString(" Acc_rad: ");CK_PRINTER_PrintInt(accRadius);
	CK_PRINTER_PrintString(" Mag_rad: ");CK_PRINTER_PrintlnInt(magRadius);

}

void CK_BNO055_WriteSensorOffsetAndRadius(void){

	// Acc Offsets
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x55, (uint8_t)(acc_x_offset & 0x00FF)); 	// ACC_OFFSET_X_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x56, (uint8_t)(acc_x_offset >> 8)); 		// ACC_OFFSET_X_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x57, (uint8_t)(acc_y_offset & 0x00FF)); 	// ACC_OFFSET_Y_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x58, (uint8_t)(acc_y_offset >> 8)); 		// ACC_OFFSET_Y_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x59, (uint8_t)(acc_z_offset & 0x00FF)); 	// ACC_OFFSET_Z_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5A, (uint8_t)(acc_z_offset >> 8)); 		// ACC_OFFSET_Z_MSB

	// Mag Offsets
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5B, (uint8_t)(mag_x_offset & 0x00FF)); 	// MAG_OFFSET_X_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5C, (uint8_t)(mag_x_offset >> 8)); 		// MAG_OFFSET_X_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5D, (uint8_t)(mag_y_offset & 0x00FF)); 	// MAG_OFFSET_Y_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5E, (uint8_t)(mag_y_offset >> 8)); 		// MAG_OFFSET_Y_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x5F, (uint8_t)(mag_z_offset & 0x00FF)); 	// MAG_OFFSET_Z_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x60, (uint8_t)(mag_z_offset >> 8)); 		// MAG_OFFSET_Z_MSB

	// Gyro Offsets
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x61, (uint8_t)(gyro_x_offset & 0x00FF)); 	// GYRO_OFFSET_X_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x62, (uint8_t)(gyro_x_offset >> 8));     	// GYRO_OFFSET_X_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x63, (uint8_t)(gyro_y_offset & 0x00FF)); 	// GYRO_OFFSET_Y_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x64, (uint8_t)(gyro_y_offset >> 8)); 	 	// GYRO_OFFSET_Y_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x65, (uint8_t)(gyro_z_offset & 0x00FF)); 	// GYRO_OFFSET_Z_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x66, (uint8_t)(gyro_z_offset >> 8)); 	 	// GYRO_OFFSET_Z_MSB

	// Radius
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x67, (uint8_t)(acc_radius & 0x00FF)); 	// ACC_RADIUS_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x68, (uint8_t)(acc_radius >> 8)); 	  	// ACC_RADIUS_MSB

	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x69, (uint8_t)(mag_radius & 0x00FF)); 	// MAG_RADIUS_LSB
	CK_I2C_Transfer(MOTION_BNO055_I2C, BNO055_ADDRESS, 0x6A, (uint8_t)(mag_radius >> 8)); 	  	// MAG_RADIUS_MSB

	CK_TIME_DelayMilliSec(100);

}


















