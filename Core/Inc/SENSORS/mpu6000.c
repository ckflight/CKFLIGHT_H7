/*
 * mpu6000.c
 *
 *  Created on: Feb 13, 2025
 *      Author: ck
 */

#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/mpu6000.h"

#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75 | 0x80
#define MPU6000_WHO_AM_I_ID				0x68

#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C

#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38

#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01

SPI_TypeDef * SPI_MPU6000;

GPIO_TypeDef* GPIO_CS_MPU6000;

uint8_t CS_PIN_MPU6000;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];


}MPU6000_PARAMETERS_t;

MPU6000_PARAMETERS_t mpu6000 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

uint8_t CK_MPU6000_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	for(int i = 0; i < GYRO_READ_ARRAY_SIZE; i++){

		mpu6000.rxArray[i] = 0xFF;
	}

	SPI_MPU6000 = SPIn;
	GPIO_CS_MPU6000 = GPIO_CSn;
	CS_PIN_MPU6000 = CS_PINn;

	uint8_t resp = 0;

	uint8_t id = CK_SPI_WriteRegister(MPU_RA_WHO_AM_I, 0xFF, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
	if(id == MPU6000_WHO_AM_I_ID){
		// Reset
		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMilliSec(100);

	    // Clock Source PPL with Z axis gyro reference
		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(10);

	    // Disable Primary I2C Interface
		CK_SPI_WriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_2, 0x00, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

		// Accel Sample Rate 1kHz
		// Gyroscope Output Rate =  1kHz when the DLPF is enabled
		//SMPLRT_DIV -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		CK_SPI_WriteRegister(MPU_RA_SMPLRT_DIV, 0, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

	    // Gyro +/- 2000 DPS Full Scale
		CK_SPI_WriteRegister(MPU_RA_GYRO_CONFIG, 3u << 3, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

	    // Accel +/- 16 G Full Scale
		CK_SPI_WriteRegister(MPU_RA_ACCEL_CONFIG, 3u << 3, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

		// Acc bw 260Hz 0 delay, 256Hz 0.98 delay DLPF_CFG = 0
		CK_SPI_WriteRegister(MPU_RA_CONFIG, 0, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

	    // INT_ANYRD_2CLEAR
		CK_SPI_WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000); //SMPLRT_DIV -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		CK_TIME_DelayMicroSec(15);

		CK_SPI_WriteRegister(MPU_RA_INT_ENABLE, 1u<<0, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000); //SMPLRT_DIV -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		CK_TIME_DelayMicroSec(15);


		mpu6000.GyroInit = true;

		mpu6000.HardwareInit = true;

		resp = 1;

	}
	else{
		resp = 0;
	}

	return resp;
}

uint8_t CK_MPU6000_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq){

	SPI_MPU6000 = SPIn;
	GPIO_CS_MPU6000 = GPIO_CSn;
	CS_PIN_MPU6000 = CS_PINn;

    uint8_t resp = 0;

	uint8_t id = CK_SPI_WriteRegister(MPU_RA_WHO_AM_I, 0xFF, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
	if(id == MPU6000_WHO_AM_I_ID && mpu6000.GyroInit && mpu6000.HardwareInit){

		mpu6000.AccInit = true;

		mpu6000.HardwareInit = true;

        resp = 1;

    }
    else{
    	resp = 0;
    }

    return resp;

}

void CK_MPU6000_AlignGyro(int x, int y, int z){

	gyro.gyroSign[X]  = x;

	gyro.gyroSign[Y] = y;

	gyro.gyroSign[Z]   = z;
}

void CK_MPU6000_AlignAcc(int x, int y, int z){

	acc.accSign[X] = x;

	acc.accSign[Y] = y;

	acc.accSign[Z] = z;

}

void CK_MPU6000_ReadGyroRaw(void){

    CK_SPI_ReadRegisterMulti(MPU6000_GYRO_XOUT_H, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000, mpu6000.rxArray, 6);

    gyro.gyroADCRaw[X]  = (int16_t)(mpu6000.rxArray[0] << 8 | mpu6000.rxArray[1]);

    gyro.gyroADCRaw[Y] = (int16_t)(mpu6000.rxArray[2] << 8 | mpu6000.rxArray[3]);

    gyro.gyroADCRaw[Z]   = (int16_t)(mpu6000.rxArray[4] << 8 | mpu6000.rxArray[5]);

}

void CK_MPU6000_ReadAccRaw(void){

    CK_SPI_ReadRegisterMulti(MPU6000_ACCEL_XOUT_H, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000, mpu6000.rxArray, 6);

    acc.accADCRaw[X] = (int16_t)(mpu6000.rxArray[0] << 8 | mpu6000.rxArray[1]);

    acc.accADCRaw[Y] = (int16_t)(mpu6000.rxArray[2] << 8 | mpu6000.rxArray[3]);

    acc.accADCRaw[Z] = (int16_t)(mpu6000.rxArray[4] << 8 | mpu6000.rxArray[5]);

}

void CK_MPU6000_ReadSensorRaw_DMA(void){

	// This function read all acc temp and gyro at once
	// Fill buffer before cleandcache
	mpu6000.rxArray[0] = MPU6000_ACCEL_XOUT_H | 0x80;

	#if USE_H7 == 1
	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)mpu6000.rxArray, GYRO_READ_ARRAY_SIZE + 32);
	#endif

	// TX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_TX_Stream, mpu6000.rxArray, 15);

	// RX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_RX_Stream, mpu6000.rxArray, 15);


	CK_GPIO_ClearPin(GPIO_CS_MPU6000, CS_PIN_MPU6000);

	CK_SPI_DMA_Enable(SENSOR_DMA_RX_Stream);
	CK_SPI_DMA_Enable(SENSOR_DMA_TX_Stream);

	CK_SPI_EnableRXDMA(SPI_MPU6000);
	CK_SPI_EnableTXDMA(SPI_MPU6000);


	#if USE_H7
	// tsize with read size is not working higher number works.
	CK_SPI_StartTransfer(SPI_MPU6000, 20);
	#endif

}

#if USE_DMA_SENSOR_MPU6000

void SENSOR_DMA_TX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_TX_Stream)){ // Transfer of one sector is done.

    	CK_SPI_DMA_Disable(SENSOR_DMA_TX_Stream);

        CK_SPI_DisableTXDMA(SPI_MPU6000);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

    }
}

void SENSOR_DMA_RX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_RX_Stream)){ // Transfer of one sector is done.

        CK_SPI_DisableRXDMA(SPI_MPU6000);

        CK_SPI_DMA_Disable(SENSOR_DMA_RX_Stream);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

		#if USE_H7
		CK_SPI_Disable(SPI_MPU6000);
		#endif

        CK_GPIO_SetPin(GPIO_CS_MPU6000, CS_PIN_MPU6000);

		#if USE_H7 == 1
    	// Invalidate before rx operation when dcache is enabled
        // DMA is done so data is ready on sram send it to cache so cpu can use it.
    	SCB_InvalidateDCache_by_Addr((uint32_t*)mpu6000.rxArray, GYRO_READ_ARRAY_SIZE + 32);
		#endif

        // First byte is response of read reg data write

        acc.accADCRaw[X] = (int16_t)(mpu6000.rxArray[1] << 8 | mpu6000.rxArray[2]);

        acc.accADCRaw[Y] = (int16_t)(mpu6000.rxArray[3] << 8 | mpu6000.rxArray[4]);

        acc.accADCRaw[Z] = (int16_t)(mpu6000.rxArray[5] << 8 | mpu6000.rxArray[6]);

        gyro.gyroacc_sensor_temperature = (float)((int16_t)(mpu6000.rxArray[7] << 8 | mpu6000.rxArray[8]) / 340.0f + 35.0f);

		gyro.gyroADCRaw[X]  = (int16_t)(mpu6000.rxArray[9] << 8 | mpu6000.rxArray[10]);

		gyro.gyroADCRaw[Y] = (int16_t)(mpu6000.rxArray[11] << 8 | mpu6000.rxArray[12]);

		gyro.gyroADCRaw[Z]   = (int16_t)(mpu6000.rxArray[13] << 8 | mpu6000.rxArray[14]);


    }
}

#endif

float CK_MPU6000_ReadTempRaw(void){

	CK_SPI_ReadRegisterMulti(MPU6000_TEMP_OUT_H, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000, mpu6000.rxArray, 2);

	int16_t value2 = ((uint16_t )mpu6000.rxArray[0]<<8) | (uint16_t )mpu6000.rxArray[1];
	return value2/326.8 + 25;
}

bool CK_MPU6000_isGyroSensorInitialized(void){

	return mpu6000.GyroInit;
}

bool CK_MPU6000_isAccSensorInitialized(void){

    return mpu6000.AccInit;
}























