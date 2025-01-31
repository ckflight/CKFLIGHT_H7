
#include <COMMON/maths.h>
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_IIM42652.h"


#define ICM42688P_RESET_BIT                 		1u<<7
#define ICM42688P_I2C_IF_DIS_BIT            		1u<<6

#define CK_IIM42652_WHO_AM_I_READ_REG	    		0x75	 // If we set MSB, indicates a read operation
#define IIM42652_WHO_AM_I_ID	      				0x6F 	 // If we set MSB, indicates a read operation

#define ICM426XX_RA_PWR_MGMT0                       0x4E
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_DEVICE_CONFIG                   0x11
#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05

#define ICM426XX_AAF_258HZ_DELT                     6
#define ICM426XX_AAF_258HZ_BITSHIFT                 10
#define ICM426XX_AAF_536HZ_DELT                     12
#define ICM426XX_AAF_536HZ_BITSHIFT                 8
#define ICM426XX_AAF_997HZ_DELT                     21
#define ICM426XX_AAF_997HZ_BITSHIFT                 6
#define ICM426XX_AAF_1962HZ_DELT                    37
#define ICM426XX_AAF_1962HZ_BITSHIFT                4

#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (14 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (14 << 0)

#define ICM426XX_RA_TEMP_DATA_X1                    0x1D
#define ICM426XX_RA_GYRO_DATA_X1                    0x25
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F

#define ICM426XX_RA_INT_CONFIG                      0x14
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define ICM426XX_ODR_32KHz				            1
#define ICM426XX_ODR_16KHz				            2
#define ICM426XX_ODR_8KHz				            3
#define ICM426XX_ODR_4KHz				            4
#define ICM426XX_ODR_2KHz				            5
#define ICM426XX_ODR_1KHz				            6


SPI_TypeDef * SPI_IIM42652;

GPIO_TypeDef* GPIO_CS_IIM42652;

uint8_t CS_PIN_IIM42652;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];

}ICM42688P_PARAMETERS_t;

ICM42688P_PARAMETERS_t iim42652 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};


uint8_t CK_IIM42652_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	for(int i = 0; i < GYRO_READ_ARRAY_SIZE; i++){

		iim42652.rxArray[i] = 0xFF;
	}

	SPI_IIM42652 		= SPIn;
	GPIO_CS_IIM42652 	= GPIO_CSn;
	CS_PIN_IIM42652 	= CS_PINn;

	uint8_t resp = 0;

	if(CK_SPI_WriteRegister(CK_IIM42652_WHO_AM_I_READ_REG | 0x80, 0xFF, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652) == IIM42652_WHO_AM_I_ID){

        // Reset the ICM42688
        CK_SPI_WriteRegister(ICM426XX_RA_DEVICE_CONFIG, 0x01, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
        CK_TIME_DelayMilliSec(100);

        uint8_t data = 0;
		if(gyroFreq == TARGET_8KHZ_US){
			data = ICM426XX_ODR_8KHz;
		}
		else if(gyroFreq == TARGET_4KHZ_US){
			data = ICM426XX_ODR_4KHz;
		}
		else if(gyroFreq == TARGET_2KHZ_US){
			data = ICM426XX_ODR_2KHz;
		}
		else if(gyroFreq == TARGET_1KHZ_US){
			data = ICM426XX_ODR_1KHz;
		}
		else{
			data = ICM426XX_ODR_8KHz;
		}

		// Gyro freq, 2000dps
		CK_SPI_WriteRegister(ICM426XX_RA_GYRO_CONFIG0, 0 << 5 | (data & 0x0F), SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

    	// Acc freq, 16g
		CK_SPI_WriteRegister(ICM426XX_RA_ACCEL_CONFIG0, 0 << 5 | (data & 0x0F), SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

		CK_SPI_WriteRegister(ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);

		CK_SPI_WriteRegister(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

		iim42652.GyroInit = true;

		iim42652.HardwareInit = true;

		resp = 1;

	}
	else{

	    resp = 0;
	}

	return resp;

}

uint8_t CK_IIM42652_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq){

    SPI_IIM42652 		= SPIn;
    GPIO_CS_IIM42652 	= GPIO_CSn;
    CS_PIN_IIM42652 	= CS_PINn;

    uint8_t resp = 0;

    if(CK_SPI_WriteRegister(CK_IIM42652_WHO_AM_I_READ_REG | 0x80, 0xFF, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652) == IIM42652_WHO_AM_I_ID){

        // Reset the ICM42688
        CK_SPI_WriteRegister(ICM426XX_RA_DEVICE_CONFIG, 0x01, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
        CK_TIME_DelayMilliSec(100);

        uint8_t data = 0;
		if(accFreq == TARGET_8KHZ_US){
			data = ICM426XX_ODR_8KHz;
		}
		else if(accFreq == TARGET_4KHZ_US){
			data = ICM426XX_ODR_4KHz;
		}
		else if(accFreq == TARGET_2KHZ_US){
			data = ICM426XX_ODR_2KHz;
		}
		else if(accFreq == TARGET_1KHZ_US){
			data = ICM426XX_ODR_1KHz;
		}
		else{
			data = ICM426XX_ODR_8KHz;
		}

		// 8KHz, 2000dps
		CK_SPI_WriteRegister(ICM426XX_RA_GYRO_CONFIG0, 0 << 5 | (data & 0x0F), SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

    	// 8KHz 16g
		CK_SPI_WriteRegister(ICM426XX_RA_ACCEL_CONFIG0, 0 << 5 | (data & 0x0F), SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

		CK_SPI_WriteRegister(ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);

		CK_SPI_WriteRegister(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652);
		CK_TIME_DelayMilliSec(100);

    	iim42652.AccInit = true;

    	iim42652.HardwareInit = true;

        resp = 1;

    }
    else{
    	resp = 0;
    }

    return resp;

}

void CK_IIM42652_AlignGyro(int x, int y, int z){

	gyro.gyroSign[X]  = x;

	gyro.gyroSign[Y] = y;

	gyro.gyroSign[Z]   = z;
}

void CK_IIM42652_AlignAcc(int x, int y, int z){

	acc.accSign[X] = x;

	acc.accSign[Y] = y;

	acc.accSign[Z] = z;

}

void CK_IIM42652_ReadGyroRaw(void){

	CK_SPI_ReadRegisterMulti(ICM426XX_RA_GYRO_DATA_X1, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652, iim42652.rxArray, 6);

    gyro.gyroADCRaw[X]  = (int16_t)(iim42652.rxArray[0] << 8 | iim42652.rxArray[1]);

    gyro.gyroADCRaw[Y] = (int16_t)(iim42652.rxArray[2] << 8 | iim42652.rxArray[3]);

    gyro.gyroADCRaw[Z]   = (int16_t)(iim42652.rxArray[4] << 8 | iim42652.rxArray[5]);


}

void CK_IIM42652_ReadAccRaw(void){

	CK_SPI_ReadRegisterMulti(ICM426XX_RA_ACCEL_DATA_X1, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652, iim42652.rxArray, 6);

    acc.accADCRaw[X] = (int16_t)(iim42652.rxArray[0] << 8 | iim42652.rxArray[1]);

    acc.accADCRaw[Y] = (int16_t)(iim42652.rxArray[2] << 8 | iim42652.rxArray[3]);

    acc.accADCRaw[Z] = (int16_t)(iim42652.rxArray[4] << 8 | iim42652.rxArray[5]);

}

void CK_IIM42652_ReadSensorRaw_DMA(void){

	// This function read all acc temp and gyro at once
	// Fill buffer before cleandcache
	iim42652.rxArray[0] = ICM426XX_RA_TEMP_DATA_X1 | 0x80;

	#if USE_H7 == 1
	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)iim42652.rxArray, GYRO_READ_ARRAY_SIZE + 32);
	#endif

	// TX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_TX_Stream, iim42652.rxArray, 15);

	// RX
	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

	CK_SPI_DMA_SetBuffer(SENSOR_DMA_RX_Stream, iim42652.rxArray, 15);


	CK_GPIO_ClearPin(GPIO_CS_IIM42652, CS_PIN_IIM42652);

	CK_SPI_DMA_Enable(SENSOR_DMA_RX_Stream);
	CK_SPI_DMA_Enable(SENSOR_DMA_TX_Stream);

	CK_SPI_EnableTXDMA(SPI_IIM42652);
	CK_SPI_EnableRXDMA(SPI_IIM42652);

	#if USE_H7
	// tsize with read size is not working higher number works.
	CK_SPI_StartTransfer(SPI_IIM42652, 20);
	#endif

}

#if USE_DMA_SENSOR_IIM42652

void SENSOR_DMA_TX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_TX_Stream)){ // Transfer of one sector is done.

        CK_SPI_DMA_Disable(SENSOR_DMA_TX_Stream);

        CK_SPI_DisableTXDMA(SPI_IIM42652);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_TX_Stream);

    }
}

void SENSOR_DMA_RX_Handler(void){

    if(CK_SPI_DMA_IsTransferComplete(SENSOR_DMA, SENSOR_DMA_RX_Stream)){ // Transfer of one sector is done.

        CK_SPI_DisableRXDMA(SPI_IIM42652);

        CK_SPI_DMA_Disable(SENSOR_DMA_RX_Stream);

    	CK_SPI_DMA_ClearFlag(SENSOR_DMA, SENSOR_DMA_RX_Stream);

		#if USE_H7
        CK_SPI_Disable(SPI_IIM42652);
		#endif

        CK_GPIO_SetPin(GPIO_CS_IIM42652, CS_PIN_IIM42652);

		#if USE_H7 == 1
    	// Invalidate before rx operation when dcache is enabled
    	// DMA is done so data is ready on sram send it to cache so cpu can use it.
    	SCB_InvalidateDCache_by_Addr((uint32_t*)iim42652.rxArray, GYRO_READ_ARRAY_SIZE + 32);
        #endif

        // First byte is response of read reg data write
        acc.accADCRaw[X] = (int16_t)(iim42652.rxArray[3] << 8 | iim42652.rxArray[4]);

        acc.accADCRaw[Y] = (int16_t)(iim42652.rxArray[5] << 8 | iim42652.rxArray[6]);

        acc.accADCRaw[Z] = (int16_t)(iim42652.rxArray[7] << 8 | iim42652.rxArray[8]);

		gyro.gyroADCRaw[X]  = (int16_t)(iim42652.rxArray[9] << 8 | iim42652.rxArray[10]);

		gyro.gyroADCRaw[Y] = (int16_t)(iim42652.rxArray[11] << 8 | iim42652.rxArray[12]);

		gyro.gyroADCRaw[Z]   = (int16_t)(iim42652.rxArray[13] << 8 | iim42652.rxArray[14]);


    }
}

#endif


float CK_IIM42652_ReadTempRaw(void){

	CK_SPI_ReadRegisterMulti(0x1D, SPI_IIM42652, GPIO_CS_IIM42652, CS_PIN_IIM42652, iim42652.rxArray, 2);

	int16_t value2 = ((uint16_t )iim42652.rxArray[0]<<8) | (uint16_t )iim42652.rxArray[1];
	return value2/132.48 + 25;
}

bool CK_IIM42652_isGyroSensorInitialized(void){

	return iim42652.GyroInit;
}

bool CK_IIM42652_isAccSensorInitialized(void){

    return iim42652.AccInit;
}





