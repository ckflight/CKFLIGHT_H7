#ifndef CK_DEFINITIONS_H_
#define CK_DEFINITIONS_H_

#include "stdbool.h"
#include "string.h"
#include "git_commit_hash.h"

#define 		CKFLIGHT_F4			false
#define 		CKFLIGHT_H7			false
#define 		KAKUTE_H7_1v3		true
#define 		MATEKH743_SLIMV3	false
#define 		RF_REVOLT			false

#define COMMIT_HASH					CURRENT_COMMIT_HASH
#define CURRENT_VERSION_MAJOR		VERSION_MAJOR
#define CURRENT_VERSION_MINOR		VERSION_MINOR

// Interrupt Priorities
#define USB_PreemptPriority			0
#define USB_SubPriority				0

#define GYRO_TX_PreemptPriority		0
#define GYRO_TX_SubPriority			1

#define GYRO_RX_PreemptPriority		0
#define GYRO_RX_SubPriority			2

#define RECEIVER_PreemptPriority	0
#define RECEIVER_SubPriority		3

#define DSHOT_M1_PreemptPriority	1
#define DSHOT_M1_SubPriority		0

#define DSHOT_M2_PreemptPriority	1
#define DSHOT_M2_SubPriority		1

#define DSHOT_M3_PreemptPriority	1
#define DSHOT_M3_SubPriority		2

#define DSHOT_M4_PreemptPriority	1
#define DSHOT_M4_SubPriority		3

#define SDCARD_PreemptPriority		2
#define SDCARD_SubPriority			0

#define GPS_PreemptPriority			3
#define GPS_SubPriority				1

#define OSD_PreemptPriority			4
#define OSD_SubPriority				0

#define MAX7456_PreemptPriority		4
#define MAX7456_SubPriority			1

#define ADC_PreemptPriority			5
#define ADC_SubPriority				0

#define RGB_PreemptPriority			5
#define RGB_SubPriority				1

#define PERIPHERAL_PreemptPriority	6
#define PERIPHERAL_SubPriority		0

#if MATEKH743_SLIMV3 == true
#define TARGET_BOARD	"MATEKH743_SLIMV3"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if CKFLIGHT_H7 == true
#define TARGET_BOARD	"CKFLIGHT_H7"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if KAKUTE_H7_1v3 == true
#define TARGET_BOARD	"KAKUTE_H7_v1.3"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if CKFLIGHT_F4 == true
#define TARGET_BOARD			"CKFLIGHT_F4"
#define TARGET_MCU 				"F4"

// For dma cache etc so copy paste wont create problem
#define USE_H7	0
#define USE_F4	1
#endif

#if RF_REVOLT == true
#define TARGET_BOARD			"RF_REVOLT"
#define TARGET_MCU 				"F4"

// For dma cache etc so copy paste wont create problem
#define USE_H7	0
#define USE_F4	1
#endif

#if USE_H7 == 1
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#endif

#if USE_F4 == 1
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#endif

#if CKFLIGHT_F4 == true
#include "config/ckflightf4_v4.h"
#endif

#if CKFLIGHT_H7 == true
#include "config/ckflighth7_v1.h"
#endif

#if KAKUTE_H7_1v3 == true
#include "config/kakuteh7_1v3.h"
#endif

#if MATEKH743_SLIMV3 == true
#include "config/matekh743slim_v3.h"
#endif

#if RF_REVOLT == true
#include "config/revolt.h"
#endif


typedef struct{
    uint16_t syncRate;

    uint16_t syncCounter;

    uint32_t targetLoopTime; // in microseconds 1k is 1000 microsecond 2k is 500 8k is 125 etc.

    uint32_t targetFrequency;

}syncTimer_t;


// Target times in microseconds 8k is 125 microsecond etc.
typedef enum
{
    TARGET_1HZ_US                              	= 1000000/1,
    TARGET_2HZ_US                              	= 1000000/2,
	TARGET_4HZ_US                              	= 1000000/4,
    TARGET_5HZ_US                              	= 1000000/5,
	TARGET_6HZ_US                              	= 1000000/6,
	TARGET_8HZ_US								= 1000000/8,
	TARGET_10HZ_US 								= 1000000/10,
	TARGET_15HZ_US                             	= 1000000/15,
	TARGET_20HZ_US 								= 1000000/20,
	TARGET_25HZ_US 								= 1000000/25,
	TARGET_50HZ_US 								= 1000000/50,
	TARGET_75HZ_US                             	= 1000000/75,
	TARGET_80HZ_US 								= 1000000/80,
	TARGET_100HZ_US 							= 1000000/100,
	TARGET_105HZ_US                            	= 1000000/105,
	TARGET_110HZ_US                            	= 1000000/110,
	TARGET_125HZ_US                            	= 1000000/125,
	TARGET_200HZ_US 							= 1000000/200,
	TARGET_220HZ_US                            	= 1000000/220,
	TARGET_250HZ_US                            	= 1000000/250,
	TARGET_300HZ_US 							= 1000000/300,
	TARGET_400HZ_US 							= 1000000/400,
	TARGET_500HZ_US 							= 1000000/500,
	TARGET_600HZ_US 							= 1000000/600,
	TARGET_800HZ_US 							= 1000000/800,
	TARGET_1KHZ_US 								= 1000000/1000,
	TARGET_2KHZ_US 								= 1000000/2000,
	TARGET_4KHZ_US 								= 1000000/4000,
	TARGET_8KHZ_US 								= 1000000/8000,
	TARGET_10KHZ_US 							= 1000000/10000,
	TARGET_12KHZ_US 							= 1000000/12000,
	TARGET_16KHZ_US                           	= 1000000/16000,
	TARGET_32KHZ_US                           	= 1000000/32000

}targetFreq_e;


typedef enum
{
	ICM20602_GYRO,
	ICM42688P_GYRO,
	L3GD20H_GYRO,
	ICM42605_GYRO,
	IIM42652_GYRO,
	MPU6000_GYRO,

	ICM20602_ACC,
	ICM42688P_ACC,
	IIM42652_ACC,
	LSM303D_ACC,
	FXOS8700CQ_ACC,
	ICM42605_ACC,
	MPU6000_ACC,

	MAG3110_MAGNETO,
	LSM303D_MAGNETO,
	FXOS8700CQ_MAGNETO,
	HMC5983_MAGNETO,
	QMC5883L_MAGNETO,
	MLX90393_MAGNETO,

	MS5607_BAROMETER,
	BMP280_BAROMETER,
	MS5611_BAROMETER,

	BNO055_IMU,

	GPS_UBLOX7,
	GPS_UBLOX8,

	SENSOR_NONE

}sensorModel_e;



#endif /* CK_DEFINITIONS_H_ */






