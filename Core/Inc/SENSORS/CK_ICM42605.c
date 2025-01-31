
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_ICM42605.h"

#define CK_ICM42605_WHO_AM_I_READ_REG      	0x75|0x80 // If we set MSB, indicates a read operation
#define ICM42605_WHO_AM_I_ID	      		0x42

SPI_TypeDef * SPI_ICM42605;

GPIO_TypeDef* GPIO_CS_ICM42605;

uint8_t CS_PIN_ICM42605;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];


}ICM42605_PARAMETERS_t;

ICM42605_PARAMETERS_t icm42605 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};


uint8_t CK_ICM42605_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	SPI_ICM42605 		= SPIn;
	GPIO_CS_ICM42605 	= GPIO_CSn;
	CS_PIN_ICM42605 	= CS_PINn;

	uint8_t resp = 0;

	uint8_t id = CK_SPI_WriteRegister(CK_ICM42605_WHO_AM_I_READ_REG, 0xFF, SPI_ICM42605, GPIO_CS_ICM42605, CS_PIN_ICM42605);
	if(id == ICM42605_WHO_AM_I_ID){

        // Reset the ICM42688
        //CK_SPI_WriteRegister(DEVICE_CONFIG, 0x01, SPI_ICM42605, GPIO_CS_ICM42605, CS_PIN_ICM42605);
        CK_TIME_DelayMilliSec(100);

		icm42605.GyroInit = true;

		icm42605.HardwareInit = true;

		resp = 1;

	}
	else{
	    resp = 0;
	}

	return resp;

}

uint8_t CK_ICM42605_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq){

    SPI_ICM42605 = SPIn;
    GPIO_CS_ICM42605 = GPIO_CSn;
    CS_PIN_ICM42605 = CS_PINn;

    uint8_t resp = 0;

    uint8_t id = CK_SPI_WriteRegister(CK_ICM42605_WHO_AM_I_READ_REG, 0xFF, SPI_ICM42605, GPIO_CS_ICM42605, CS_PIN_ICM42605);
    if(id == ICM42605_WHO_AM_I_ID){

        icm42605.AccInit = true;

        icm42605.HardwareInit = true;

        resp = 1;

    }
    else{
    	resp = 0;
    }

    return resp;

}

void CK_ICM42605_AlignGyro(int x, int y, int z){

	gyro.gyroSign[X]  = x;

	gyro.gyroSign[Y] = y;

	gyro.gyroSign[Z]   = z;
}

void CK_ICM42605_AlignAcc(int x, int y, int z){

	acc.accSign[X] = x;

	acc.accSign[Y] = y;

	acc.accSign[Z] = z;

}

void CK_ICM42605_ReadGyroRaw(void){

    CK_SPI_ReadRegisterMulti(0x1D, SPI_ICM42605, GPIO_CS_ICM42605, CS_PIN_ICM42605, icm42605.rxArray, 14);

    gyro.gyroADCRaw[X]  = (int16_t)(icm42605.rxArray[0] << 8 | icm42605.rxArray[1]);

    gyro.gyroADCRaw[Y] = (int16_t)(icm42605.rxArray[2] << 8 | icm42605.rxArray[3]);

    gyro.gyroADCRaw[Z]   = (int16_t)(icm42605.rxArray[4] << 8 | icm42605.rxArray[5]);

}

void CK_ICM42605_ReadAccRaw(void){

    //CK_SPI_ReadRegisterMulti(ACCEL_OUT, SPI_ICM42605, GPIO_CS_ICM42605, CS_PIN_ICM42605, icm42605.rxArray, 6);

    acc.accADCRaw[X] = (int16_t)(icm42605.rxArray[0] << 8 | icm42605.rxArray[1]);

    acc.accADCRaw[Y] = (int16_t)(icm42605.rxArray[2] << 8 | icm42605.rxArray[3]);

    acc.accADCRaw[Z] = (int16_t)(icm42605.rxArray[4] << 8 | icm42605.rxArray[5]);

}

bool CK_ICM42605_isGyroSensorInitialized(void){

	return icm42605.GyroInit;
}

bool CK_ICM42605_isAccSensorInitialized(void){

    return icm42605.AccInit;
}





