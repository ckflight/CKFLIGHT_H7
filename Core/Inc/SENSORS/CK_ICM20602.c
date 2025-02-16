
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_ICM20602.h"

#define CK_ICM20602_SMPLRT_DIV_REG          0x19
#define CK_ICM20602_CONFIG_REG              0x1A
#define CK_ICM20602_GYRO_CONFIG_REG         0x1B
#define CK_ICM20602_ACCEL_CONFIG_REG        0x1C
#define CK_ICM20602_ACCEL_CONFIG2_REG       0x1D
#define CK_ICM20602_SIGNAL_PATH_RESET_REG   0x68
#define CK_ICM20602_USER_CONTROL_REG        0x6A
#define CK_ICM20602_PWR_MNG1_REG            0x6B
#define CK_ICM20602_I2C_IF_REG              0x70

#define ICM20602_WHO_AM_I_ID                0x12

#define ICM20602_RESET_BIT                  1u<<7
#define ICM20602_I2C_IF_DIS_BIT             1u<<6

#define CK_ICM20602_WHO_AM_I_READ_REG       0x75|0x80 // If we set MSB, indicates a read operation

#define CK_ICM20602_ACCEL_XOUT_H            0x3B
#define CK_ICM20602_ACCEL_XOUT_L            0x3C
#define CK_ICM20602_ACCEL_YOUT_H            0x3D
#define CK_ICM20602_ACCEL_YOUT_L            0x3E
#define CK_ICM20602_ACCEL_ZOUT_H            0x3F
#define CK_ICM20602_ACCEL_ZOUT_L            0x40
#define CK_ICM20602_TEMP_OUT_H             	0x41
#define CK_ICM20602_TEMP_OUT_L             	0x42
#define CK_ICM20602_GYRO_XOUT_H             0x43
#define CK_ICM20602_GYRO_XOUT_L             0x44
#define CK_ICM20602_GYRO_YOUT_H             0x45
#define CK_ICM20602_GYRO_YOUT_L             0x46
#define CK_ICM20602_GYRO_ZOUT_H             0x47
#define CK_ICM20602_GYRO_ZOUT_L             0x48

SPI_TypeDef * SPI_ICM20602;

GPIO_TypeDef* GPIO_CS_ICM20602;

uint8_t CS_PIN_ICM20602;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];


}ICM20602_PARAMETERS_t;

ICM20602_PARAMETERS_t icm20602 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

uint8_t CK_ICM20602_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	for(int i = 0; i < GYRO_READ_ARRAY_SIZE; i++){

		icm20602.rxArray[i] = 0xFF;
	}

	SPI_ICM20602 = SPIn;
	GPIO_CS_ICM20602 = GPIO_CSn;
	CS_PIN_ICM20602 = CS_PINn;

	uint8_t resp = 0;

	uint8_t id = CK_SPI_WriteRegister(CK_ICM20602_WHO_AM_I_READ_REG, 0xFF, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
	if(id == ICM20602_WHO_AM_I_ID){

		// ICM20602 Setup
		CK_TIME_DelayMilliSec(100);

		// Device Reset, Takes everyting to default
        CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, ICM20602_RESET_BIT, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
        CK_TIME_DelayMilliSec(100);

		// I2C Disable, SPI Only Mode
		CK_SPI_WriteRegister(CK_ICM20602_I2C_IF_REG, ICM20602_I2C_IF_DIS_BIT, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(10);

		// Reset Signal Path and Sensor Register
		CK_SPI_WriteRegister(CK_ICM20602_USER_CONTROL_REG, 0x05, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(100);

		// Clear PWR MNG1 Register
		//CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		//CK_TIME_DelayMilliSec(10);

		// CLK Selection for Best Gyro Performance
		CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, 0x01, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(10);

		// DLPF_CFG[00] 8KHz, Bit 7 set to 0, Set main clock to 8MHz
		CK_SPI_WriteRegister(CK_ICM20602_CONFIG_REG, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(10);

		uint8_t div_rate = 0;
		if(gyroFreq == TARGET_8KHZ_US){
			div_rate = 0;
		}
		else if(gyroFreq == TARGET_4KHZ_US){
			div_rate = 1;
		}
		else if(gyroFreq == TARGET_2KHZ_US){
			div_rate = 3;
		}
		else if(gyroFreq == TARGET_1KHZ_US){
			div_rate = 7;
		}
		else{
			div_rate = 0;
		}

		// SMPL RATE DIV = 0 8Khz/1+SMPL RATE DIV = 8Khz
		CK_SPI_WriteRegister(CK_ICM20602_SMPLRT_DIV_REG, div_rate, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(10);

		// FCHOICE_B [00], FSEL[11] CK_ICM20602_DPS2000
		CK_SPI_WriteRegister(CK_ICM20602_GYRO_CONFIG_REG, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
		CK_TIME_DelayMilliSec(10);

		icm20602.GyroInit = true;

		icm20602.HardwareInit = true;

		resp = 1;

	}
	else{

	    resp = 0;
	}

	return resp;

}

uint8_t CK_ICM20602_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq){

    SPI_ICM20602 = SPIn;
    GPIO_CS_ICM20602 = GPIO_CSn;
    CS_PIN_ICM20602 = CS_PINn;

    uint8_t resp = 0;

    if(CK_SPI_WriteRegister(CK_ICM20602_WHO_AM_I_READ_REG, 0xFF, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) == ICM20602_WHO_AM_I_ID){

        // ACCEL,TEMP SIGNAL PATH RESET
        CK_SPI_WriteRegister(CK_ICM20602_SIGNAL_PATH_RESET_REG, 0x03, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
        CK_TIME_DelayMilliSec(10);

        // ACCEL_FS_SEL[11] +-16g 0x18, 2g 0x00
        CK_SPI_WriteRegister(CK_ICM20602_ACCEL_CONFIG_REG, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
        CK_TIME_DelayMilliSec(10);

        uint8_t data = 0;
        if(accFreq == TARGET_4KHZ_US){
			data = 0x08;
		}
		else if(accFreq == TARGET_1KHZ_US){
			data = 0x01; // 218 hz bw is selected
		}
		else{
			data = 0x08;
		}

        // ACCEL_FCHOICE_B=1,A_DLPF_CFG=0
        CK_SPI_WriteRegister(CK_ICM20602_ACCEL_CONFIG2_REG, data, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);

        CK_TIME_DelayMilliSec(10);

        icm20602.AccInit = true;

        icm20602.HardwareInit = true;

        resp = 1;

    }
    else{
    	resp = 0;
    }

    return resp;

}

void CK_ICM20602_AlignGyro(int x, int y, int z){

	gyro.gyroSign[X]  = x;

	gyro.gyroSign[Y] = y;

	gyro.gyroSign[Z]   = z;
}

void CK_ICM20602_AlignAcc(int x, int y, int z){

	acc.accSign[X] = x;

	acc.accSign[Y] = y;

	acc.accSign[Z] = z;

}

void CK_ICM20602_ReadGyroRaw(void){

    CK_SPI_ReadRegisterMulti(CK_ICM20602_GYRO_XOUT_H, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602, icm20602.rxArray, 6);

    gyro.gyroADCRaw[X]  = (int16_t)(icm20602.rxArray[0] << 8 | icm20602.rxArray[1]);

    gyro.gyroADCRaw[Y] = (int16_t)(icm20602.rxArray[2] << 8 | icm20602.rxArray[3]);

    gyro.gyroADCRaw[Z]   = (int16_t)(icm20602.rxArray[4] << 8 | icm20602.rxArray[5]);

}

void CK_ICM20602_ReadAccRaw(void){

    CK_SPI_ReadRegisterMulti(CK_ICM20602_ACCEL_XOUT_H, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602, icm20602.rxArray, 6);

    acc.accADCRaw[X] = (int16_t)(icm20602.rxArray[0] << 8 | icm20602.rxArray[1]);

    acc.accADCRaw[Y] = (int16_t)(icm20602.rxArray[2] << 8 | icm20602.rxArray[3]);

    acc.accADCRaw[Z] = (int16_t)(icm20602.rxArray[4] << 8 | icm20602.rxArray[5]);

}

void CK_ICM20602_ReadSensorRaw_DMA(void){

	// This function read all acc temp and gyro at once
	// Fill buffer before cleandcache
	icm20602.rxArray[0] = CK_ICM20602_ACCEL_XOUT_H | 0x80;

	#if USE_H7 == 1
	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)icm20602.rxArray, GYRO_READ_ARRAY_SIZE + 32);
	#endif

	// TX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_TX_Stream, icm20602.rxArray, 15);

	// RX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_RX_Stream, icm20602.rxArray, 15);


	CK_GPIO_ClearPin(GPIO_CS_ICM20602, CS_PIN_ICM20602);

	CK_SPI_DMA_Enable(SENSOR_DMA_RX_Stream);
	CK_SPI_DMA_Enable(SENSOR_DMA_TX_Stream);

	CK_SPI_EnableRXDMA(SPI_ICM20602);
	CK_SPI_EnableTXDMA(SPI_ICM20602);


	#if USE_H7
	// tsize with read size is not working higher number works.
	CK_SPI_StartTransfer(SPI_ICM20602, 20);
	#endif

}

#if USE_DMA_SENSOR_ICM20602

void SENSOR_DMA_TX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_TX_Stream)){ // Transfer of one sector is done.

    	CK_SPI_DMA_Disable(SENSOR_DMA_TX_Stream);

        CK_SPI_DisableTXDMA(SPI_ICM20602);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

    }
}

void SENSOR_DMA_RX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_RX_Stream)){ // Transfer of one sector is done.

        CK_SPI_DisableRXDMA(SPI_ICM20602);

        CK_SPI_DMA_Disable(SENSOR_DMA_RX_Stream);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

		#if USE_H7
		CK_SPI_Disable(SPI_ICM20602);
		#endif

        CK_GPIO_SetPin(GPIO_CS_ICM20602, CS_PIN_ICM20602);

		#if USE_H7 == 1
    	// Invalidate before rx operation when dcache is enabled
        // DMA is done so data is ready on sram send it to cache so cpu can use it.
    	SCB_InvalidateDCache_by_Addr((uint32_t*)icm20602.rxArray, GYRO_READ_ARRAY_SIZE + 32);
		#endif

        // First byte is response of read reg data write

        acc.accADCRaw[X] = (int16_t)(icm20602.rxArray[1] << 8 | icm20602.rxArray[2]);

        acc.accADCRaw[Y] = (int16_t)(icm20602.rxArray[3] << 8 | icm20602.rxArray[4]);

        acc.accADCRaw[Z] = (int16_t)(icm20602.rxArray[5] << 8 | icm20602.rxArray[6]);

        gyro.gyroacc_sensor_temperature = (float)((int16_t)(icm20602.rxArray[7] << 8 | icm20602.rxArray[8]) / 326.8 + 25.0f);

		gyro.gyroADCRaw[X]  = (int16_t)(icm20602.rxArray[9] << 8 | icm20602.rxArray[10]);

		gyro.gyroADCRaw[Y] = (int16_t)(icm20602.rxArray[11] << 8 | icm20602.rxArray[12]);

		gyro.gyroADCRaw[Z]   = (int16_t)(icm20602.rxArray[13] << 8 | icm20602.rxArray[14]);


    }
}

#endif

float CK_ICM20602_ReadTempRaw(void){

	CK_SPI_ReadRegisterMulti(0x41, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602, icm20602.rxArray, 2);

	int16_t value2 = ((uint16_t )icm20602.rxArray[0]<<8) | (uint16_t )icm20602.rxArray[1];
	return value2/326.8 + 25.0f;
}

bool CK_ICM20602_isGyroSensorInitialized(void){

	return icm20602.GyroInit;
}

bool CK_ICM20602_isAccSensorInitialized(void){

    return icm20602.AccInit;
}





