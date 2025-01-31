
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_QMC5883L.h"

#include "MOTION/CK_MAGNETO.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define QMC5883L_CHIP_ID_REGISTER       0x0D

#define QMC5883L_ADDRESS        0x0D

#define QMC5883L_CTRL_REG1      0x0B
#define QMC5883L_CTRL_REG2      0x09

#define QMC5883L_XOUT_MSB       0x00

I2C_TypeDef* QMC5883L_I2C;

typedef struct{

    bool MagInit;

    uint8_t rxArray[MAG_READ_ARRAY_SIZE];


}QMC5883L_PARAMETERS_t;

QMC5883L_PARAMETERS_t qmc5883l = {
    .MagInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

void CK_QMC5883L_Init(I2C_TypeDef* I2Cn, uint32_t magFreq){

    QMC5883L_I2C = I2Cn;

	CK_I2C_ReadMulti(QMC5883L_I2C, QMC5883L_ADDRESS, QMC5883L_CHIP_ID_REGISTER, qmc5883l.rxArray, 1);

	if(qmc5883l.rxArray[0] == 0xFF){
	    // Register Setup before Active Mode
	    CK_I2C_Transfer(QMC5883L_I2C, QMC5883L_ADDRESS, QMC5883L_CTRL_REG1, 0x01);

	    // 0x49 for 2G 100Hz Continuous Bandwith = 256
	    // 0x0D for 2G 200Hz Continuous Bandwith = 512
	    // 0x1D for 8G 200Hz Continuous Bandwith = 512
	    // 0x11 for 8G  10Hz Continuous Bandwith = 512
	    CK_I2C_Transfer(QMC5883L_I2C, QMC5883L_ADDRESS, QMC5883L_CTRL_REG2, 0x11);


	    //mag.magScale[0] = 0.000083f; //Digital resolution is 12000 for 2G

	    mag.magScale[0] = 0.000333f; //Digital resolution is 3000 LSB/Gauss for 8G

	    qmc5883l.MagInit = true;

	    CK_PRINTER_PrintlnString("MAG INITIALIZED");
	}
	else{
        CK_PRINTER_PrintlnString("MAG ERROR");
    }

}

void CK_QMC5883L_AlignMag(int x, int y, int z){

	mag.magSign[X] = x;

	mag.magSign[Y] = y;

	mag.magSign[Z] = z;

}

void CK_QMC5883L_ReadMagRaw(void){

	CK_I2C_ReadMulti(QMC5883L_I2C, QMC5883L_ADDRESS, QMC5883L_XOUT_MSB, qmc5883l.rxArray, 6);

	mag.magADCRaw[X] = (int16_t)(qmc5883l.rxArray[1] << 8 | qmc5883l.rxArray[0]);

	mag.magADCRaw[Y] = (int16_t)(qmc5883l.rxArray[3] << 8 | qmc5883l.rxArray[2]);

	mag.magADCRaw[Z] = (int16_t)(qmc5883l.rxArray[5] << 8 | qmc5883l.rxArray[4]);

}

bool CK_QMC5883L_isMagSensorInitialized(void){

    return qmc5883l.MagInit;
}







