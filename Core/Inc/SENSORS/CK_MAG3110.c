
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_MAG3110.h"

#include "MOTION/CK_MAGNETO.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define MAG3110_ADDRESS             0x0E
#define MAG3110_WHO_AM_I_ID         0xC4

#define MAG3110_CTRL_REG1           0x10
#define MAG3110_CTRL_REG2           0x11
#define MAG3110_WHO_AM_I_REG        0x07
#define MAG3110_OUT_X_MSB           0x01 // Auto Increments

#define MAG_RST                     1u<<4
#define MAG_RAW                     1u<<5

I2C_TypeDef* MAG3110_I2C;

typedef struct{

    bool MagInit;

    uint8_t rxArray[MAG_READ_ARRAY_SIZE];


}MAG3110_PARAMETERS_t;

MAG3110_PARAMETERS_t mag3110 = {
    .MagInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

void CK_MAG3110_Init(I2C_TypeDef* I2Cn, uint32_t magFreq){

    MAG3110_I2C = I2Cn;

	CK_I2C_ReadMulti(MAG3110_I2C, MAG3110_ADDRESS, MAG3110_WHO_AM_I_REG, mag3110.rxArray, 1);
	if(mag3110.rxArray[0] == MAG3110_WHO_AM_I_ID){

		// Register Setup before Active Mode
		CK_I2C_Transfer(MAG3110_I2C, MAG3110_ADDRESS, MAG3110_CTRL_REG2, MAG_RST | MAG_RAW);// Offset register values are not applied, i calculate and subtract offset

		CK_I2C_Transfer(MAG3110_I2C, MAG3110_ADDRESS, MAG3110_CTRL_REG1, 0x01);// 80Hz->0x01, 10Hz->0x19

		mag.magScale[0] = 0.10f;

		mag3110.MagInit = true;

	}
	else{
		CK_PRINTER_PrintlnString("MAG ERROR");
	}

}

void CK_MAG3110_AlignMag(int x, int y, int z){

	mag.magSign[X] = x;

	mag.magSign[Y] = y;

	mag.magSign[Z] = z;

}

void CK_MAG3110_ReadMagRaw(void){

	CK_I2C_ReadMulti(MAG3110_I2C, MAG3110_ADDRESS, MAG3110_OUT_X_MSB, mag3110.rxArray, 6);

	mag.magADCRaw[X] = (int16_t)(mag3110.rxArray[0] << 8 | mag3110.rxArray[1]);

	mag.magADCRaw[Y] = (int16_t)(mag3110.rxArray[2] << 8 | mag3110.rxArray[3]);

	mag.magADCRaw[Z] = (int16_t)(mag3110.rxArray[4] << 8 | mag3110.rxArray[5]);

}

bool CK_MAG3110_isMagSensorInitialized(void){

    return mag3110.MagInit;
}








