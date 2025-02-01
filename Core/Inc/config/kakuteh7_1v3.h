
#define VOLT_CALIBRATION_MULTIPLIER		11.1f

#define TARGET_MAIN_TIME_US			TARGET_8KHZ_US
#define TARGET_GYRO_TIME_US			TARGET_8KHZ_US
#define TARGET_ACC_TIME_US			TARGET_8KHZ_US
#define TARGET_DSHOT_TIME_US		TARGET_8KHZ_US // max 22 khz update rate with current dshot timing
#define TARGET_LOG_TIME_US			TARGET_8KHZ_US
#define TARGET_PERIPHERAL_TIME_US	TARGET_8KHZ_US
#define TARGET_FLASH				FLASH_STM32H7_

	// Updates receiver and rc for response
	// Interrupt cannot be used for all method updates
	#define MAIN_INTERRUPT_TIM			TIM16
	#define MAIN_INTERRUPT_TIM_CH		TIM_CHANNEL_1
	#define MAIN_INTERRUPT_Handler		TIM16_IRQHandler
	#define MAIN_INTERRUPT_IRQn			TIM16_IRQn

#if GYRO1_SPI_
	#define GYRO1_SPI           		SPI4
	#define GYRO1_CS_PORT				GPIOE
	#define GYRO1_CS_PIN				4
	#define TARGET_GYRO1				MPU6000_GYRO

	#define GYRO1_USE_INT				0
	#define GYRO1_INT_PORT				GPIOE
	#define GYRO1_INT_PIN				1

	#define USE_DMA_SENSOR				1
	#define USE_DMA_SENSOR_ICM42688P	0
	#define USE_DMA_SENSOR_ICM20602		0
	#define USE_DMA_SENSOR_IIM42652		0
	#define USE_DMA_SENSOR_MPU6000		1
	#define SENSOR_DMA					DMA1
	#define SENSOR_DMA_TX_Stream		DMA1_Stream6
	#define SENSOR_DMA_RX_Stream		DMA1_Stream7
	#define SENSOR_DMA_TX_Handler		DMA1_Stream6_IRQHandler
	#define SENSOR_DMA_RX_Handler		DMA1_Stream7_IRQHandler
	#define SENSOR_DMA_Request1			DMA_REQUEST_SPI4_TX
	#define SENSOR_DMA_Request2			DMA_REQUEST_SPI4_RX

	#define SENSOR_DMA_TX_IRQn			DMA1_Stream6_IRQn
	#define SENSOR_DMA_RX_IRQn			DMA1_Stream7_IRQn

	#define GYRO1_SPI_CLOCK				16000000L

#endif

#if ACC1_SPI_
	#define ACC1_SPI           			SPI4
	#define ACC1_CS_PORT				GPIOE
	#define ACC1_CS_PIN					4
	#define TARGET_ACC1					MPU6000_ACC

	#define ACC1_SPI_CLOCK				16000000L
#endif

#if GYRO2_SPI_
	#define GYRO2_SPI           		SPI4 // Checked on hardware
	#define GYRO2_CS_PORT				GPIOC // Checked on hardware
	#define GYRO2_CS_PIN				13
	#define TARGET_GYRO2				ICM42688P_GYRO

	#define GYRO2_USE_INT				0
	#define GYRO2_INT_PORT				GPIOE
	#define GYRO2_INT_PIN				15

	#define USE_DMA_SENSOR				1
	#define USE_DMA_SENSOR_ICM42688P	1
	#define USE_DMA_SENSOR_ICM20602		0
	#define USE_DMA_SENSOR_IIM42652		0
	#define SENSOR_DMA					DMA1
	#define SENSOR_DMA_TX_Stream		DMA1_Stream6
	#define SENSOR_DMA_RX_Stream		DMA1_Stream7
	#define SENSOR_DMA_TX_Handler		DMA1_Stream6_IRQHandler
	#define SENSOR_DMA_RX_Handler		DMA1_Stream7_IRQHandler
	#define SENSOR_DMA_Request1			DMA_REQUEST_SPI4_TX
	#define SENSOR_DMA_Request2			DMA_REQUEST_SPI4_RX

	#define SENSOR_DMA_TX_IRQn			DMA1_Stream6_IRQn
	#define SENSOR_DMA_RX_IRQn			DMA1_Stream7_IRQn

	#define GYRO2_SPI_CLOCK				16000000L
#endif

#if ACC2_SPI_
	#define ACC2_SPI           			SPI4
	#define ACC2_CS_PORT				GPIOC
	#define ACC2_CS_PIN					13
	#define TARGET_ACC2					ICM42688P_ACC

	#define ACC2_SPI_CLOCK				16000000L
#endif

#if EXT_SPI_
	#define EXT_SPI		           		SPI3

	#define USE_DMA_SENSOR				1
	#define USE_DMA_SENSOR_ICM42688P	0
	#define USE_DMA_SENSOR_ICM20602		1
	#define SENSOR_DMA					DMA1
	#define SENSOR_DMA_TX_Stream		DMA1_Stream6
	#define SENSOR_DMA_RX_Stream		DMA1_Stream7
	#define SENSOR_DMA_TX_Handler		DMA1_Stream6_IRQHandler
	#define SENSOR_DMA_RX_Handler		DMA1_Stream7_IRQHandler
	#define SENSOR_DMA_Request1			DMA_REQUEST_SPI3_TX
	#define SENSOR_DMA_Request2			DMA_REQUEST_SPI3_RX

	#define SENSOR_DMA_TX_IRQn			DMA1_Stream6_IRQn
	#define SENSOR_DMA_RX_IRQn			DMA1_Stream7_IRQn

	#define EXT_SPI_CLOCK				10000000L

#endif

#if EXT_CS1_
	#define EXT_CS1_PORT				GPIOD
	#define EXT_CS1_PIN					4
#endif

#if EXT_CS2_
	#define EXT_CS2_PORT				GPIOE
	#define EXT_CS2_PIN					2
#endif

#if ACC_I2C_
	#define ACC_I2C          			I2C2
#endif

#if BARO_SPI_
	#define BARO_SPI           			SPI1
	#define BARO_CS_PORT       			GPIOC
	#define BARO_CS_PIN        			4

	#define BARO_SPI_CLOCK				10000000L
	#define TARGET_BARO					BMP280_BAROMETER
#endif

#if BARO_I2C_
	#define BARO_I2C          			I2C1
	#define TARGET_BARO					BMP280_BAROMETER
#endif

#if MAG_SPI_
	#define MAG_SPI           			SPI1
	#define MAG_CS_PORT					GPIOB
	#define MAG_CS_PIN					1

	#define MAG_SPI_CLOCK				10000000L
	#define TARGET_MAG					QMC5883L_MAGNETO
#endif

#if MAG_I2C_
	#define MAG_I2C           			I2C1
	#define TARGET_MAG					QMC5883L_MAGNETO
#endif

#if BNO055_
	#define BNO055_I2C		        	I2C1
#endif

// GPS
#if GPS_
	#define GPS_UART					UART4
	#define USE_INTERRUPT_GPS			1
	#define GPS_INTERRUPT_				4
#endif

	#define BUZZER_GPIO					GPIOC // BUZ- pin is DC active low but i use pwm to set volume
	#define BUZZER_GPIO_PIN				13

#if BUZZER_PWM
	#define BUZZER_TIM					TIM2
	#define BUZZER_TIM_CH				TIM_CHANNEL_1
	#define BUZZER_AF					CK_GPIO_AF1
#endif

#if BUZZER_DC

	#define BUZZER_AF					CK_GPIO_NOAF

#endif

#if LOG_SPI_

	#define MICROCARD_DMA           	DMA2
	#define MICROCARD_DMA_STREAM    	DMA2_Stream0
	#define MICROCARD_SPI           	SPI1
	#define MICROCARD_CS_PORT       	GPIOA
	#define MICROCARD_CS_PIN        	4

	#define MICROCARD_DETECT_PORT		GPIOA
	#define MICROCARD_DETECT_PIN		3

	#define MICROCARD_DMA_TX_Handler	DMA2_Stream0_IRQHandler
	#define MICROCARD_DMA_Request1		DMA_REQUEST_SPI1_TX

	#define MICROCARD_DMA_TX_IRQn		DMA2_Stream0_IRQn

	#define MICROCARD_SPI_CLOCK			16000000L

#endif

#if LOG_SDIO_
	// SDIO has its own dma no need for dma setup
	#define MICROCARD_SDIO				SDMMC1
#endif

#if LOG_FLASH_
	#define FLASH_SPI					SPI1
	#define FLASH_GPIO					GPIOC
	#define FLASH_CS_PIN				13

	#define FLASH_SPI_CLOCK				10000000L
#endif

	// DSHOT and PWM PROTOCOL
	#define MOTOR1_TIM					TIM3
	#define MOTOR2_TIM					TIM3
	#define MOTOR3_TIM					TIM2
	#define MOTOR4_TIM					TIM2

	#define MOTOR1_TIM_CH				TIM_CHANNEL_3
	#define MOTOR2_TIM_CH				TIM_CHANNEL_4
	#define MOTOR3_TIM_CH				TIM_CHANNEL_2
	#define MOTOR4_TIM_CH				TIM_CHANNEL_3

	#define MOTOR1_GPIO					GPIOB
	#define MOTOR1_PIN					0
	#define MOTOR1_AF					CK_GPIO_AF2

	#define MOTOR2_GPIO					GPIOB
	#define MOTOR2_PIN					1
	#define MOTOR2_AF					CK_GPIO_AF2

	#define MOTOR3_GPIO					GPIOB
	#define MOTOR3_PIN					3
	#define MOTOR3_AF					CK_GPIO_AF1

	#define MOTOR4_GPIO					GPIOB
	#define MOTOR4_PIN					10
	#define MOTOR4_AF					CK_GPIO_AF1

	#define DSHOT1_DMA					DMA1
	#define DSHOT1_DMA_Stream			DMA1_Stream0
	#define DSHOT1_DMA_Request			DMA_REQUEST_TIM3_CH3
	#define DSHOT1_DMA_ID				TIM_DMA_ID_CC3
	#define DSHOT1_DMA_Handler			DMA1_Stream0_IRQHandler
	#define DSHOT1_DMA_IRQn				DMA1_Stream0_IRQn

	#define DSHOT2_DMA					DMA1
	#define DSHOT2_DMA_Stream			DMA1_Stream1
	#define DSHOT2_DMA_Request			DMA_REQUEST_TIM3_CH4
	#define DSHOT2_DMA_ID				TIM_DMA_ID_CC4
	#define DSHOT2_DMA_Handler			DMA1_Stream1_IRQHandler
	#define DSHOT2_DMA_IRQn				DMA1_Stream1_IRQn

	#define DSHOT3_DMA					DMA1
	#define DSHOT3_DMA_Stream			DMA1_Stream2
	#define DSHOT3_DMA_Request			DMA_REQUEST_TIM2_CH2
	#define DSHOT3_DMA_ID				TIM_DMA_ID_CC2
	#define DSHOT3_DMA_Handler			DMA1_Stream2_IRQHandler
	#define DSHOT3_DMA_IRQn				DMA1_Stream2_IRQn

	#define DSHOT4_DMA					DMA1
	#define DSHOT4_DMA_Stream			DMA1_Stream3
	#define DSHOT4_DMA_Request			DMA_REQUEST_TIM2_CH3
	#define DSHOT4_DMA_ID				TIM_DMA_ID_CC3
	#define DSHOT4_DMA_Handler			DMA1_Stream3_IRQHandler
	#define DSHOT4_DMA_IRQn				DMA1_Stream3_IRQn

	#define DSHOT_INTERRUPT_TIM			TIM7 				// Basic timer
	#define DSHOT_INTERRUPT_TIM_CH		TIM_CHANNEL_ALL 	// No channel on basic timer, so i used this decleration
	#define DSHOT_INTERRUPT_Handler		TIM7_IRQHandler
	#define DSHOT_INTERRUPT_IRQn		TIM7_IRQn


#if RGB_
	#define RGB_TIM						TIM4

	#define RGB_TIM_CH					TIM_CHANNEL_1

	#define RGB_GPIO					GPIOD
	#define RGB_PIN						12
	#define RGB_AF						CK_GPIO_AF2

	#define RGB_DMA						DMA1
	#define RGB_DMA_Stream				DMA1_Stream4
	#define RGB_DMA_Request				DMA_REQUEST_TIM4_CH1
	#define RGB_DMA_ID					TIM_DMA_ID_CC1
	#define RGB_DMA_Handler				DMA1_Stream4_IRQHandler
	#define RGB_DMA_IRQn				DMA1_Stream4_IRQn
#endif

#if LED1_
	#define LED1_GPIO					GPIOC
	#define LED1_GPIO_PIN				2
	#define LED1_ACTIVE_LOW				1
#endif

#if LED2_
	#define LED2_GPIO					GPIOE
	#define LED2_GPIO_PIN				4
	#define LED2_ACTIVE_LOW				1
#endif

	#define ADC_LIPO_GPIO				GPIOC
	#define ADC_LIPO_GPIO_PIN			0

	#define ADC_CURRENT_GPIO			GPIOC
	#define ADC_CURRENT_GPIO_PIN		1

#if OSD_ONBOARD_
	#define OSD_SPI						SPI2
	#define OSD_CS_PORT					GPIOB
	#define OSD_CS_PIN					12

	#define USE_DMA_MAX7456				1
	#define OSD_MAX7456_DMA				DMA1
	#define OSD_MAX7456_DMA_Stream		DMA1_Stream5
	#define OSD_MAX7456_DMA_Handler		DMA1_Stream5_IRQHandler
	#define OSD_MAX7456_DMA_Request		DMA_REQUEST_SPI2_TX
	#define OSD_MAX7456_DMA_IRQn		DMA1_Stream5_IRQn

	#define OSD_SPI_CLOCK				16000000L

#endif

#if OSD_PDB_
	#define OSD_PDB_USART				UART4

	#define USE_INTERRUPT_OSD			1
	#define OSD_INTERRUPT_				4

	#define USE_DMA_OSD					0
	#define OSD_DMA						DMA2
	#define OSD_DMA_Stream				DMA2_Stream2
	#define OSD_DMA_Handler				DMA2_Stream2_IRQHandler
	#define OSD_DMA_IRQn				DMA2_Stream2_IRQn
	#define OSD_DMA_Request				DMA_REQUEST_UART4_TX
#endif

#if OSD_DJI_
	#define OSD_DJI_USART				USART1

	#define USE_INTERRUPT_OSD			1
	#define OSD_INTERRUPT_				1

	#define USE_DMA_OSD					0
	#define OSD_DMA						DMA2
	#define OSD_DMA_Stream				DMA2_Stream2
	#define OSD_DMA_Handler				DMA2_Stream2_IRQHandler
	#define OSD_DMA_IRQn				DMA2_Stream2_IRQn
	#define OSD_DMA_Request				DMA_REQUEST_USART1_TX
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
	#define SBUS_DMA_Request			DMA_REQUEST_USART6_RX
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
	#define CRSF_DMA_Request			DMA_REQUEST_USART6_RX
#endif

#if SMART_AUDIO_
	#define SMART_AUDIO_GPIO			GPIOB
	#define SMART_AUDIO_GPIO_PIN		9
#endif

#if VTX_SWITCH_
	#define VTX_SWITCH_GPIO				GPIOD
	#define VTX_SWITCH_GPIO_PIN			10
#endif

#if CAMERA_SWITCH_
	#define CAMERA_SWITCH_GPIO			GPIE
	#define CAMERA_SWITCH_GPIO_PIN		9
	#define CAMERA_SELECT				1
#endif


#if SCOPE_CHECK_
	#define SCOPE_CHECK_GPIO			GPIOD // TX2
	#define SCOPE_CHECK_GPIO_PIN		5
#endif
