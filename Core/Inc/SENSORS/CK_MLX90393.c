
#include "SENSORS/CK_MLX90393.h"

#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_MAGNETO.h"

#define MLX90393_DEFAULT_ADDR	0x0C	// Can also be 0x18, depending on IC

#define MLX90393_AXIS_ALL		0x0E    //X+Y+Z axis bits for commands.
#define MLX90393_CONF1			0x00    // Gain
#define MLX90393_CONF2			0x01    // Burst, comm mode
#define MLX90393_CONF3			0x02    // Oversampling, filter, res.
#define MLX90393_CONF4			0x03    // Sensitivty drift.
#define MLX90393_GAIN_SHIFT		4	    // Left-shift for gain bits.
#define MLX90393_HALL_CONF		0x0C    // Hall plate spinning rate adj.
#define MLX90393_STATUS_OK		0x00    // OK value for status response.
#define MLX90393_STATUS_SMMODE	0x08	// SM Mode status response.
#define MLX90393_STATUS_RESET	0x01	// Reset value for status response.
#define MLX90393_STATUS_ERROR	0xFF	// OK value for status response.
#define MLX90393_STATUS_MASK	0xFC   	// Mask for status OK checks.

#define MLX90393_REG_SB			0x10	// Start burst mode.
#define MLX90393_REG_SW			0x20	// Start wakeup on change mode.
#define MLX90393_REG_SM			0x30	// Start single-meas mode.
#define MLX90393_REG_RM			0x40	// Read measurement.
#define MLX90393_REG_RR			0x50	// Read register.
#define MLX90393_REG_WR			0x60	// Write register.
#define MLX90393_REG_EXIT		0x80	// Exit moode.
#define MLX90393_REG_HR			0xD0	// Memory recall.
#define MLX90393_REG_HS			0x70	// Memory store.
#define MLX90393_REG_RT			0xF0	// Reset.
#define MLX90393_REG_NOP		0x00	// NOP.

#define MLX90393_X_AXIS			0
#define MLX90393_Y_AXIS			1
#define MLX90393_Z_AXIS			2

SPI_TypeDef * SPI_MLX90393;

GPIO_TypeDef* GPIO_CS_MLX90393;

uint8_t CS_PIN_MLX90393;

enum mlx90393_gain _gain;
enum mlx90393_resolution _res_x, _res_y, _res_z;
enum mlx90393_filter _dig_filt;
enum mlx90393_oversampling _osr;

/** Lookup table to convert raw values to uT based on [HALLCONF][GAIN_SEL][RES].
 */
const float mlx90393_lsb_lookup[2][8][4][2] = {

    /* HALLCONF = 0xC (default) */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    /* HALLCONF = 0x0 */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }
};

typedef struct{

    bool MagInit;

    bool HardwareInit;

    uint8_t rxArray[10];


}MLX90393_PARAMETERS_t;

MLX90393_PARAMETERS_t mlx90393 = {
    .MagInit = false,
    .HardwareInit = false,
};

// Datasheet table 18 shows the update rate of sensor.
// setfilter 7 and oversampling 2 is 10Hz.
// setfilter 5 and oversampling 0 is 125Hz.
void CK_MLX90393_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t magFreq){

	SPI_MLX90393 		= SPIn;
	GPIO_CS_MLX90393 	= GPIO_CSn;
	CS_PIN_MLX90393 	= CS_PINn;

	CK_MLX90393_ExitMode();

	CK_MLX90393_Reset();

	CK_MLX90393_SetGain(MLX90393_GAIN_1X);

	CK_MLX90393_SetResolution(MLX90393_X_AXIS, MLX90393_RES_16);
	CK_MLX90393_SetResolution(MLX90393_Y_AXIS, MLX90393_RES_16);
	CK_MLX90393_SetResolution(MLX90393_Z_AXIS, MLX90393_RES_16);

	CK_MLX90393_SetOverSampling(MLX90393_OSR_0);

	CK_MLX90393_SetFilter(MLX90393_FILTER_5);

	mag.magScale[X] = mlx90393_lsb_lookup[0][_gain][_res_x][0];
	mag.magScale[Y] = mlx90393_lsb_lookup[0][_gain][_res_y][0];
	mag.magScale[Z] = mlx90393_lsb_lookup[0][_gain][_res_z][1];

	mlx90393.MagInit = true;

}

uint8_t CK_MLX90393_ExitMode(void){

	uint8_t resp = CK_SPI_WriteRegister(MLX90393_REG_EXIT, 0, SPI_MLX90393, GPIO_CS_MLX90393, CS_PIN_MLX90393);

	if(resp >> 2 != 0x00){
			//error
	}

	return resp;

}

uint8_t CK_MLX90393_Reset(void){

	uint8_t resp = CK_SPI_WriteRegister(MLX90393_REG_RT, 0, SPI_MLX90393, GPIO_CS_MLX90393, CS_PIN_MLX90393);

	if(resp >> 2 != 0x01){
		//error
	}

	CK_TIME_DelayMilliSec(5);

	return resp;

}

uint8_t CK_MLX90393_SetGain(mlx90393_gain_t gain){

	_gain = gain;

	uint16_t data;

	CK_MLX90393_ReadRegister(MLX90393_CONF1, &data);

	data &= ~0x0070;

	data |= gain << MLX90393_GAIN_SHIFT;

	uint8_t resp = CK_MLX90393_WriteRegister(MLX90393_CONF1, data);

	return resp;

}

uint8_t CK_MLX90393_SetResolution(int axis, enum mlx90393_resolution resolution){

	uint16_t data;

	CK_MLX90393_ReadRegister(MLX90393_CONF3, &data);

	if(axis == MLX90393_X_AXIS){
		_res_x = resolution;
		data &= ~0x0060;
		data |= resolution << 5;
	}
	else if(axis == MLX90393_Y_AXIS){
		_res_y = resolution;
		data &= ~0x0180;
		data |= resolution << 7;
	}
	else if(axis == MLX90393_Z_AXIS){
		_res_z = resolution;
		data &= ~0x0600;
		data |= resolution << 9;
	}

	return CK_MLX90393_WriteRegister(MLX90393_CONF3, data);

}

uint8_t CK_MLX90393_SetOverSampling(enum mlx90393_oversampling oversampling){

	_osr = oversampling;

	uint16_t data;

	CK_MLX90393_ReadRegister(MLX90393_CONF3, &data);

	data &= ~0x03;
	data |= oversampling;


	return CK_MLX90393_WriteRegister(MLX90393_CONF3, data);

}

uint8_t CK_MLX90393_SetFilter(enum mlx90393_filter filter){

	uint16_t data;

	CK_MLX90393_ReadRegister(MLX90393_CONF3, &data);

	data &= ~0x1C;
	data |= filter << 2;

	return CK_MLX90393_WriteRegister(MLX90393_CONF3, data);

}

uint8_t CK_MLX90393_Transceive(uint8_t* tx_buffer, uint8_t tx_len, uint8_t* rx_buffer, uint8_t rx_len, int delay){

	uint8_t rx_buffer2[rx_len + 2];

	CK_GPIO_ClearPin(GPIO_CS_MLX90393, CS_PIN_MLX90393);

	for(int i = 0; i < tx_len; i++){
		CK_SPI_Transfer(SPI_MLX90393, tx_buffer[i]);
	}

	for(int i = 0; i < rx_len + 1; i++){
		rx_buffer2[i] = CK_SPI_Transfer(SPI_MLX90393, 0xFF);
	}

	CK_GPIO_SetPin(GPIO_CS_MLX90393, CS_PIN_MLX90393);

	uint8_t status = rx_buffer2[0];

	for(int i = 0; i < rx_len; i++){
		rx_buffer[i] = rx_buffer2[i + 1];
	}

	CK_TIME_DelayMilliSec(delay);

	return status >> 2;


}

void CK_MLX90393_ReadRegister(uint8_t reg, uint16_t* data){

	uint8_t tx_buf[2] = {MLX90393_REG_RR, reg << 2}; // shift register itsel 2 bit.

	uint8_t rx_buf[2];

	CK_MLX90393_Transceive(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf), 0);

	*data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

}

uint8_t CK_MLX90393_WriteRegister(uint8_t reg, uint16_t data){

	uint8_t tx_buf[4] = {MLX90393_REG_WR,
						data >> 8,		// high byte
						data & 0xFF,	// low byte
						reg << 2};		// register itself shift 2 bit.

	return CK_MLX90393_Transceive(tx_buf, sizeof(tx_buf), NULL, 0, 0);

}

uint8_t CK_MLX90393_StartSingleConversion(void){

	uint8_t tx_buf[1] = {MLX90393_REG_SM | MLX90393_AXIS_ALL};


	uint8_t stat = CK_MLX90393_Transceive(tx_buf, sizeof(tx_buf), 0, 0, 0);

	if(stat == MLX90393_STATUS_OK || stat == MLX90393_STATUS_SMMODE){
		return 1;
	}
	return 0;

}

uint8_t CK_MLX90393_ReadMag(void){

	uint8_t tx_buf[1] = {MLX90393_REG_RM | MLX90393_AXIS_ALL};
	uint8_t rx_buf[6] = {0};

	uint8_t stat = CK_MLX90393_Transceive(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf), 0);

	int16_t xi, yi, zi;

	xi = (rx_buf[0] << 8) | rx_buf[1];

	yi = (rx_buf[2] << 8) | rx_buf[3];

	zi = (rx_buf[4] << 8) | rx_buf[5];

	if (_res_x == MLX90393_RES_18)
		xi -= 0x8000;
	if (_res_x == MLX90393_RES_19)
		xi -= 0x4000;
	if (_res_y == MLX90393_RES_18)
		yi -= 0x8000;
	if (_res_y == MLX90393_RES_19)
		yi -= 0x4000;
	if (_res_z == MLX90393_RES_18)
		zi -= 0x8000;
	if (_res_z == MLX90393_RES_19)
		zi -= 0x4000;


	mag.magADCRaw[X] = xi;

	mag.magADCRaw[Y] = yi;

	mag.magADCRaw[Z] = zi;


	return stat;
}

void CK_MLX90393_AlignMag(int x, int y, int z){

	mag.magSign[X] = x;

	mag.magSign[Y] = y;

	mag.magSign[Z] = z;

}

bool CK_MLX90393_isMagSensorInitialized(void){

    return mlx90393.MagInit;
}







