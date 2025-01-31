
#ifndef INC_SENSORS_CK_MLX90393_H_
#define INC_SENSORS_CK_MLX90393_H_

#include "CK_DEFINITIONS.h"

/** Gain settings for CONF1 register. */
typedef enum mlx90393_gain {
  MLX90393_GAIN_5X = (0x00),
  MLX90393_GAIN_4X,
  MLX90393_GAIN_3X,
  MLX90393_GAIN_2_5X,
  MLX90393_GAIN_2X,
  MLX90393_GAIN_1_67X,
  MLX90393_GAIN_1_33X,
  MLX90393_GAIN_1X
} mlx90393_gain_t;

/** Resolution settings for CONF3 register. */
typedef enum mlx90393_resolution {
  MLX90393_RES_16,
  MLX90393_RES_17,
  MLX90393_RES_18,
  MLX90393_RES_19,
} mlx90393_resolution_t;

/** Axis designator. */
typedef enum mlx90393_axis {
  MLX90393_X,
  MLX90393_Y,
  MLX90393_Z
} mlx90393_axis_t;

/** Digital filter settings for CONF3 register. */
typedef enum mlx90393_filter {
  MLX90393_FILTER_0,
  MLX90393_FILTER_1,
  MLX90393_FILTER_2,
  MLX90393_FILTER_3,
  MLX90393_FILTER_4,
  MLX90393_FILTER_5,
  MLX90393_FILTER_6,
  MLX90393_FILTER_7,
} mlx90393_filter_t;

typedef enum mlx90393_oversampling {
  MLX90393_OSR_0,
  MLX90393_OSR_1,
  MLX90393_OSR_2,
  MLX90393_OSR_3,
} mlx90393_oversampling_t;




void CK_MLX90393_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t magFreq);

uint8_t CK_MLX90393_ExitMode(void);

uint8_t CK_MLX90393_Reset(void);

uint8_t CK_MLX90393_SetGain(mlx90393_gain_t gain);

uint8_t CK_MLX90393_SetResolution(int axis, enum mlx90393_resolution resolution);

uint8_t CK_MLX90393_SetOverSampling(enum mlx90393_oversampling oversampling);

uint8_t CK_MLX90393_SetFilter(enum mlx90393_filter filter);

uint8_t CK_MLX90393_Transceive(uint8_t* tx_buffer, uint8_t tx_len, uint8_t* rx_buffer, uint8_t rx_len, int delay);

void CK_MLX90393_ReadRegister(uint8_t reg, uint16_t* data);

uint8_t CK_MLX90393_WriteRegister(uint8_t reg, uint16_t data);

uint8_t CK_MLX90393_StartSingleConversion(void);

uint8_t CK_MLX90393_ReadMag(void);

void CK_MLX90393_AlignMag(int x, int y, int z);

bool CK_MLX90393_isMagSensorInitialized(void);


#endif /* INC_SENSORS_CK_MLX90393_H_ */




