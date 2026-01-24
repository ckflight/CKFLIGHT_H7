#ifndef CK_SETTINGS_H_
#define CK_SETTINGS_H_

#define MAIN_INTERRUPT_			0

#define GYRO1_SPI_				1
#define ACC1_SPI_				1

#define GYRO2_SPI_				0
#define ACC2_SPI_				0

#define EXT_SPI_				0 // External spi connection pinout exist on board
#define EXT_CS1_				0 // External chip select pin exist on board
#define EXT_CS2_				0 // Second External chip select pin exist on board

#define ACC_I2C_				0

#define BARO_SPI_				0	// Init peripheral cs etc.
#define BARO_I2C_				1	// Init peripheral cs etc.
#define USE_BARO_				1

#define MAG_SPI_				0	// Init peripheral CS important for common SPI
#define MAG_I2C_				0
#define USE_MAG_				0

#define BNO055_					0

#define GPS_					1
#define GPS_MODULE				GPS_UBLOX8

#define LOG_SPI_				0
#define LOG_SDIO_				1
#define LOG_FLASH_				0
#define LOG_DUALBUFFER_			0
#define LOG_MULTIWRITE_			0// spi works in single mode
#define TEST_LOG				0

#define BUZZER_PWM				1
#define BUZZER_DC				0

#define OSD_ONBOARD_			0
#define OSD_PDB_				0
#define OSD_DJI_				1

#define RGB_					1

#define LED1_					1
#define LED2_					1

#define RX_PWM_					0
#define SBUS_					0
#define CRSF_					1

#define SMART_AUDIO_			0

#define VTX_SWITCH_				0
#define CAMERA_SWITCH_			0

#define SCOPE_CHECK_			0

#define BUZZER_WARNING_			0

#define BATTERY_CELL			6

#if SCOPE_CHECK_
	#define SCOPE_CHECK_OSD				0
	#define SCOPE_CHECK_LOG				1
	#define SCOPE_CHECK_MAIN			0
	#define SCOPE_CHECK_ADC				0
	#define SCOPE_CHECK_RX_PACKET		0
	#define SCOPE_CHECK_CRSF_PACKET		0
	#define SCOPE_CHECK_MAIN_INTERRUPT	0
	#define SCOPE_CHECK_DSHOT_INTERRUPT	0
#endif

#define USE_DSHOT
#define USE_RC_SMOOTHING_FILTER 	// Checked
#define USE_AIRMODE_LPF 			// Checked
#define USE_ITERM_RELAX				// Checked
#define USE_ABSOLUTE_CONTROL		// Checked
#define USE_DYN_LPF					// Checked
#define USE_INTEGRATED_YAW_CONTROL	// Checked
#define USE_AIRMODE					// Checked

#define USE_GYRO_OVERFLOW_CHECK

#define USE_THROTTLE_BOOST			// Checked
#define USE_FEEDFORWARD
#define USE_TPA_MODE
#define USE_D_MAX

#define USE_ACC
#define USE_THRUST_LINEARIZATION
#define USE_SIMPLIFIED_TUNING
//#define USE_BATTERY_VOLTAGE_SAG_COMPENSATION

// I will allocate 34 bytes left from 128 byte log for debugging parameters.
// I will implement debug section for each part i want to check and select them here
// Total bytes will be allocated in log section to not pass 34.

// Add each DEBUG Definition to CK_LOG_WriteInfoBuffer()'s static part
#define LOG_DEBUG_PIDLEVEL_PARAMETERS

//#define DEBUG_TIMING

/*
 * GYRO ORIENTATION:
 *
 * Pitch Down Y+
 * Roll Right X+
 * Right rotation Z+
 *
 * Important Note: On gyro sensor datasheet the arrow is not the axis.
 * The rotation around that arrow is the axis.
 *
 * -----> Y means when rotating in pitch direction it is chaning Y data
 */
#define GYRO_ORIENTATION_X_SIGN			1 		// -1, +1
#define GYRO_ORIENTATION_Y_SIGN			1 		// -1, +1
#define GYRO_ORIENTATION_Z_SIGN			-1 		// -1, +1
#define GYRO_ORIENTATION_SWAP_XY		false 	// true or false

#define ACC_ORIENTATION_X_SIGN			1 		// -1, +1
#define ACC_ORIENTATION_Y_SIGN			1 		// -1, +1
#define ACC_ORIENTATION_Z_SIGN			1 		// -1, +1

/*  IMU allignment is X down Y right Z up
 *  TS100 mag upside down so z is down x is right and y is down. Swap xy and reverse z
 */
#define MAG_ORIENTATION_X_SIGN			1 		// -1, +1
#define MAG_ORIENTATION_Y_SIGN			1 		// -1, +1
#define MAG_ORIENTATION_Z_SIGN			-1 		// -1, +1
#define MAG_ORIENTATION_SWAP_XY			true 	// true or false
/*
 * QuadX 1234 configuration 1
 *
 * 1    2
 *
 * 4	3
 *
 * QuadX configuration 2
 *
 * 4	2
 *
 * 3	1
 *
 * QuadX reversed esc 180 configuration 3
 *
 * 4	2
 *
 * 3	1
 *
 */

#define MIXER_ORIENTATION				3     // 2 false, 3 true is correct combination
#define MIXER_ESC_REVERSED				true  // select this if esc is pointing 180 degree opposite CK_ESC

#if USE_H7 == 1
#define ADC_BITS						16
#endif
#if USE_F4 == 1
#define ADC_BITS						12
#endif

#define CURRENT_RESISTOR				90.0f
#define MAH_CALIBRATION_MULTIPLIER		1.0f


#endif



