
#include "CK_SETTINGS.h"

#define VOLT_CALIBRATION_MULTIPLIER		10.09f

#define TARGET_MAIN_FREQ			TARGET_4KHZ
#define TARGET_GYRO_FREQ			TARGET_4KHZ
#define TARGET_ACC_FREQ				TARGET_4KHZ
#define TARGET_DSHOT_FREQ			TARGET_8KHZ  // max 22 khz update rate with current dshot timing
#define TARGET_LOG_FREQ				TARGET_2KHZ
#define TARGET_FLASH				FLASH_STM32F4_

// Updates receiver and rc for response
// Interrupt cannot be used for all method updates
#define MAIN_INTERRUPT_				0
#define MAIN_INTERRUPT_TIM			TIM2
#define MAIN_INTERRUPT_TIM_CH		TIM_CHANNEL_1
#define MAIN_INTERRUPT_Handler		TIM2_IRQHandler
#define MAIN_INTERRUPT_IRQn			TIM2_IRQn

// Define all SPI pins that board have.
#define SPI1_SCK_GPIO 	GPIOA
#define SPI1_SCK_PIN  	5
#define SPI1_SCK_AF		CK_GPIO_AF5

#define SPI1_MISO_GPIO	GPIOA
#define SPI1_MISO_PIN	6
#define SPI1_MISO_AF	CK_GPIO_AF5

#define SPI1_MOSI_GPIO	GPIOA
#define SPI1_MOSI_PIN	7
#define SPI1_MOSI_AF	CK_GPIO_AF5


#define SPI3_SCK_GPIO 	GPIOC
#define SPI3_SCK_PIN  	10
#define SPI3_SCK_AF		CK_GPIO_AF6

#define SPI3_MISO_GPIO	GPIOC
#define SPI3_MISO_PIN	11
#define SPI3_MISO_AF	CK_GPIO_AF6

#define SPI3_MOSI_GPIO	GPIOC
#define SPI3_MOSI_PIN	12
#define SPI3_MOSI_AF	CK_GPIO_AF6

	// GYROSCOPE
#if GYRO1_SPI_
	#define USE_SPI1					true
	#define GYRO1_SPI           		SPI1
	#define GYRO1_CS_PORT				GPIOA
	#define GYRO1_CS_PIN				4
	#define TARGET_GYRO1				ICM20602_GYRO

	#define GYRO1_USE_INT				0
	#define GYRO1_INT_PORT				GPIOC
	#define GYRO1_INT_PIN				4

	#define USE_DMA_SENSOR				0
	#define USE_DMA_SENSOR_ICM42688P	0
	#define USE_DMA_SENSOR_ICM20602		1
	#define USE_DMA_SENSOR_IIM42652		0
	#define SENSOR_DMA					DMA2
	#define SENSOR_DMA_TX_Stream		DMA2_Stream5
	#define SENSOR_DMA_RX_Stream		DMA2_Stream0
	#define SENSOR_DMA_TX_Handler		DMA2_Stream5_IRQHandler
	#define SENSOR_DMA_RX_Handler		DMA2_Stream0_IRQHandler
	#define SENSOR_DMA_TX_Channel		3u
	#define SENSOR_DMA_RX_Channel		3u

	#define SENSOR_DMA_TX_IRQn			DMA2_Stream5_IRQn
	#define SENSOR_DMA_RX_IRQn			DMA2_Stream0_IRQn

	#define GYRO1_SPI_CLOCK				10000000L
#endif

#if GYRO2_SPI_
#endif

	// ACCELEROMETER
#if ACC1_SPI_
	#define ACC1_SPI           			SPI1
	#define ACC1_CS_PORT				GPIOB
	#define ACC1_CS_PIN					12
	#define TARGET_ACC1					ICM20602_ACC

	#define ACC1_SPI_CLOCK				10000000L
#endif

#if ACC2_SPI_
#endif

#if EXT_SPI_
#endif

#if EXT_CS1_
#endif

#if EXT_CS2_
#endif

#if ACC_I2C_
	#define ACC_I2C          			I2C1
#endif

	// BAROMETER
#if BARO_SPI_
#endif

	// MAGNETOMETER
#if MAG_SPI_
#endif

#if MAG_I2C_
	#define MAG_I2C           			I2C1
#endif

	// BNO055
#if BNO055_
	#define BNO055_I2C		        	I2C1
#endif

// GPS
#if GPS_
	#define GPS_UART					USART5
	#define USE_INTERRUPT_GPS			1
	#define GPS_INTERRUPT_				5
#endif

	// BUZZER
	#define BUZZER_GPIO					GPIOB
	#define BUZZER_GPIO_PIN				4

#if BUZZER_PWM

	#define BUZZER_TIM					TIM3
	#define BUZZER_TIM_CH				TIM_CHANNEL_1
	#define BUZZER_AF					CK_GPIO_AF2

#endif

	// MICROCARD
#if LOG_SPI_
#endif

#if LOG_SDIO_
#endif

#if LOG_FLASH_
	#define USE_SPI3					true
	#define FLASH_SPI					SPI3
	#define FLASH_CS_PORT				GPIOB
	#define FLASH_CS_PIN				3

	#define FLASH_SPI_CLOCK				25000000L
#endif

	// DSHOT and PWM PROTOCOL
	#define MOTOR1_TIM					TIM8
	#define MOTOR2_TIM					TIM8
	#define MOTOR3_TIM					TIM2
	#define MOTOR4_TIM					TIM2

	#define MOTOR1_TIM_CH				TIM_CHANNEL_2
	#define MOTOR2_TIM_CH				TIM_CHANNEL_3
	#define MOTOR3_TIM_CH				TIM_CHANNEL_4
	#define MOTOR4_TIM_CH				TIM_CHANNEL_3

	#define MOTOR1_GPIO					GPIOB
	#define MOTOR1_PIN					0
	#define MOTOR1_AF					CK_GPIO_AF3

	#define MOTOR2_GPIO					GPIOB
	#define MOTOR2_PIN					1
	#define MOTOR2_AF					CK_GPIO_AF3

	#define MOTOR3_GPIO					GPIOA
	#define MOTOR3_PIN					3
	#define MOTOR3_AF					CK_GPIO_AF1

	#define MOTOR4_GPIO					GPIOA
	#define MOTOR4_PIN					2
	#define MOTOR4_AF					CK_GPIO_AF1

	#define DSHOT1_DMA					DMA2
	#define DSHOT1_DMA_Stream			DMA2_Stream3
	#define DSHOT1_DMA_Handler			DMA2_Stream3_IRQHandler
	#define DSHOT1_DMA_IRQn				DMA2_Stream3_IRQn
	#define DSHOT1_DMA_Stream_Ch		7

	#define DSHOT2_DMA					DMA2
	#define DSHOT2_DMA_Stream			DMA2_Stream4
	#define DSHOT2_DMA_Handler			DMA2_Stream4_IRQHandler
	#define DSHOT2_DMA_IRQn				DMA2_Stream4_IRQn
	#define DSHOT2_DMA_Stream_Ch		7

	#define DSHOT3_DMA					DMA1
	#define DSHOT3_DMA_Stream			DMA1_Stream7
	#define DSHOT3_DMA_Handler			DMA1_Stream7_IRQHandler
	#define DSHOT3_DMA_IRQn				DMA1_Stream7_IRQn
	#define DSHOT3_DMA_Stream_Ch		3

	#define DSHOT4_DMA					DMA1
	#define DSHOT4_DMA_Stream			DMA1_Stream1
	#define DSHOT4_DMA_Handler			DMA1_Stream1_IRQHandler
	#define DSHOT4_DMA_IRQn				DMA1_Stream1_IRQn
	#define DSHOT4_DMA_Stream_Ch		3

	#define DSHOT_INTERRUPT_TIM			TIM7				// Basic timer
	#define DSHOT_INTERRUPT_TIM_CH		TIM_CHANNEL_ALL 	// No channel on basic timer, so i used this decleration
	#define DSHOT_INTERRUPT_Handler		TIM7_IRQHandler
	#define DSHOT_INTERRUPT_IRQn		TIM7_IRQn

	// RGB
#if RGB_
	#define RGB_TIM						TIM4

	#define RGB_TIM_CH					TIM_CHANNEL_1

	#define RGB_GPIO					GPIOB
	#define RGB_PIN						6
	#define RGB_AF						CK_GPIO_AF2

	#define RGB_DMA						DMA1
	#define RGB_DMA_Stream				DMA1_Stream0
	#define RGB_DMA_Handler				DMA1_Stream0_IRQHandler
	#define RGB_DMA_IRQn				DMA1_Stream0_IRQn
	#define RGB_DMA_Stream_Ch			2
#endif

	// LED
#if LED1_
	#define LED1_GPIO					GPIOB
	#define LED1_GPIO_PIN				5
	#define LED1_ACTIVE_LOW				0
#endif


#if LED2_
#endif

	#define ADC_LIPO_GPIO				GPIOC
	#define ADC_LIPO_GPIO_PIN			2

	#define ADC_CURRENT_GPIO			GPIOC
	#define ADC_CURRENT_GPIO_PIN		1

	// OSD
#if OSD_ONBOARD_
#endif

#if OSD_PDB_
	#define OSD_PDB_USART				UART4

	#define USE_INTERRUPT_OSD			0
	#define OSD_INTERRUPT_				4

	#define USE_DMA_OSD					0
	#define OSD_DMA						DMA1
	#define OSD_DMA_Stream				DMA1_Stream4
	#define OSD_DMA_Handler				DMA1_Stream4_IRQHandler
	#define OSD_DMA_IRQn				DMA1_Stream4_IRQn
	#define OSD_DMA_Channel				4u
#endif

#if OSD_DJI_
	#define OSD_DJI_USART				UART4

	#define USE_INTERRUPT_OSD			0
	#define OSD_INTERRUPT_				4

	#define USE_DMA_OSD					1
	#define OSD_DMA						DMA1
	#define OSD_DMA_Stream				DMA1_Stream4
	#define OSD_DMA_Handler				DMA1_Stream4_IRQHandler
	#define OSD_DMA_IRQn				DMA1_Stream4_IRQn
	#define OSD_DMA_Channel				4u
#endif

#if SBUS_
	#define SBUS_UART					USART6

	#define USE_INTERRUPT_SBUS			1
	#define SBUS_INTERRUPT_				6

	#define USE_DMA_SBUS				0
	#define SBUS_DMA					DMA2
	#define SBUS_DMA_Stream				DMA2_Stream1
	#define SBUS_DMA_Handler			DMA2_Stream1_IRQHandler
	#define SBUS_DMA_IRQn				DMA2_Stream1_IRQn
	#define SBUS_DMA_Channel			5u
#endif

#if CRSF_
	#define CRSF_UART					USART6

	#define USE_INTERRUPT_CRSF			1
	#define CRSF_INTERRUPT_				6

	#define USE_DMA_CRSF				0
	#define CRSF_DMA					DMA2
	#define CRSF_DMA_Stream				DMA2_Stream1
	#define CRSF_DMA_Handler			DMA2_Stream1_IRQHandler
	#define CRSF_DMA_IRQn				DMA2_Stream1_IRQn
	#define CRSF_DMA_Channel			5u
#endif

#if SMART_AUDIO_
	#define SMART_AUDIO_GPIO			GPIOA
	#define SMART_AUDIO_GPIO_PIN		9
#endif

#if VTX_SWITCH_
	#define VTX_SWITCH_GPIO				GPIOD
	#define VTX_SWITCH_GPIO_PIN			10
#endif

#if CAMERA_SWITCH_
	#define CAMERA_SWITCH_GPIO			GPIOD
	#define CAMERA_SWITCH_GPIO_PIN		11
	#define CAMERA_SELECT				1
#endif

#if SCOPE_CHECK_
	#define SCOPE_CHECK_GPIO			GPIOB // Solder pin at the back of the board
	#define SCOPE_CHECK_GPIO_PIN		13
#endif
