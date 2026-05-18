
#include "CK_DEFINITIONS.h"

#include "COMMUNICATION/CK_MSP.h"
#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/CK_INPUTSTREAM.h"
#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"

#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_LED.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_MICROCARD.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_RGB.h"
#include "DRIVERS/CK_SOFTSERIAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"
#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_BAROMETER.h"
#include "MOTION/CK_IMU.h"
#include "MOTION/CK_BNO055.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_SBUS.h"
#include "FLIGHT/CK_ESC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/pid_init.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_NAVIGATION.h"
#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_LAND.h"
#include "FLIGHT/CK_LOG.h"
#include "FLIGHT/CK_ADJUSTMENT.h"
#include "FLIGHT/CK_DSHOT.h"
#include "FLIGHT/CK_PERIPHERAL.h"
#include "FLIGHT/CK_SMARTAUDIO.h"
#include "FLIGHT/flight_monitor.h"

#include "COMMON/CK_FILTERS.h"

#include "FLASH/CK_FLASH.h"

#include "OSD/CK_OSD.h"
#include "OSD/CK_MAX7456.h"

uint32_t loopTimer = 0;

uint32_t computeStartTime, computeEndTime;

// todo: add motor stop when radio is off or check if it creates failsafe
// todo: implement elrs library as well and check if it has new features.

// todo: check feedforward and implement all of it if there is any missing code.
// todo: check gyro overflow.
// todo: gyro yaw spin recory: mostly implemented

// todo: F1000 elrs mixer values at idle become 600 from 160 so fast
//		 F500 is fine. Solve that missing package fault

// Check each parameter in pid profile. These two needs implementation if their macros are used
// todo: complete mixer ezlanding related things.
// todo: implement USE_ADVANCED_TPA if used by betaflight

// todo: mag not initialized it creates 250ms delay not a problem

// todo: add USE_MAG_ to imu correct 2 lines

// todo: dma reads gyro acc temp at the same time but acc is 1k gyro 8k check this!!!


// CK_IMU 508, 659 MAGNETO COMMENTED!!!

// CK_NAVIGATION 259, 260 COMMENTED!!!

// Python script run command for system:
// Windows: Properties -> C/C++ Build -> Settings -> Build Steps -> Command: python "${ProjDirPath}/Core/Inc/git_hash.py"
// Windows: Properties -> C/C++ Build -> Settings -> Build Steps -> Command: python "python3 "${ProjDirPath}/Core/Inc/git_hash.py"

uint32_t main_t1, main_t2;
int main(void){

	#if USE_H7 == 1

	// Enable I-Cache
	SCB_EnableICache();

	// Enable D-Cache
	SCB_EnableDCache();

	#endif

	#if USE_H7 == 1
	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_480MHz); // ALWAYS FIRST
	#endif

	#if USE_F4 == 1
	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_168MHz); // ALWAYS FIRST
	#endif

	HAL_Init();

	// Internal flash is used for parameters even if external flash is available
	CK_FLASH_Init_Internal(TARGET_FLASH);

    CK_PERIPHERAL_Init(TARGET_PERIPHERAL_TIME_US);

    CK_CONFIGURATION_Init(); // Mass Erase flash to change default parameters

    CK_USBD_Init();

#if BUZZER_PWM
    CK_BUZZER_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, BUZZER_MODE_PWM);
#endif

#if BUZZER_DC
    CK_BUZZER_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, BUZZER_MODE_DC);
#endif

    CK_ESC_Init(DSHOT_MODE, TARGET_DSHOT_TIME_US);

#if SMART_AUDIO_
    CK_SMARTAUDIO_Init(RACEBAND, CH1, mW_1000, TBS_UNIFY_PRO32);
#endif

#if LED1_
    CK_LED_Init1(LED1_GPIO, LED1_GPIO_PIN);
#endif

#if LED2_
    CK_LED_Init2(LED2_GPIO, LED2_GPIO_PIN);
#endif

#if LOG_SPI_ || LOG_SDIO_ || LOG_FLASH_
   	CK_LOG_Init(TARGET_MAIN_TIME_US, TARGET_LOG_TIME_US);

#if LOG_SDIO_ == 1
	#if LOG_MULTIWRITE_ == 1
   	CK_MICROCARD_Init(SDIO_DMA_INTERRUPT_MULTIBLOCK);
	#else
   	CK_MICROCARD_Init(SDIO_DMA_INTERRUPT_MULTIBLOCK);
	#endif
#endif

#if LOG_SPI_ == 1
   	#if LOG_MULTIWRITE_ == 1
   	CK_MICROCARD_Init(SPI_DMA_INTERRUPT_MULTIBLOCK);
	#else
   	CK_MICROCARD_Init(SPI_DMA_INTERRUPT_SINGLEBLOCK);
	#endif
#endif

#endif

    CK_RC_Init(TARGET_MAIN_TIME_US);

#if RGB_
    CK_RGB_Init(RGB_TIM, RGB_DMA, RGB_DMA_Stream, COLOR_BLUE2);
#endif

    CK_RECEIVER_Init(RX_CRSF);

    CK_MIXER_Init();

    CK_ADJUSTMENT_Init(TARGET_2HZ_US, TARGET_MAIN_TIME_US);

    CK_RECEIVER_WaitARM(); // Buzzer tone 1

#if GPS_
    CK_GPS_Init(GPS_UART, GPS_MODULE);
#endif

#if GYRO1_SPI_
    CK_GYRO_Init(GYRO1_SPI, GYRO1_CS_PORT, GYRO1_CS_PIN, TARGET_GYRO1, DPS2000, TARGET_GYRO_TIME_US, TARGET_MAIN_TIME_US);

    while(!gyro.is_gyro_init){
    	CK_LED_ToggleLedForMs(1, 10, 100);
    	CK_LED_ToggleLedForMs(2, 10, 100);
    }
#endif

#if ACC1_SPI_
    CK_ACC_Init(ACC1_SPI, ACC1_CS_PORT, ACC1_CS_PIN, TARGET_ACC1, G16, TARGET_ACC_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if GYRO2_SPI_
    CK_GYRO_Init(GYRO2_SPI, GYRO2_CS_PORT, GYRO2_CS_PIN, TARGET_GYRO2, DPS2000, TARGET_GYRO_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if ACC2_SPI_
    CK_ACC_Init(ACC2_SPI, ACC2_CS_PORT, ACC2_CS_PIN, TARGET_ACC2, G16, TARGET_ACC_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if EXT_SPI_
	#if EXT_CS1_
		CK_GYRO_Init(EXT_SPI, EXT_CS1_PORT, EXT_CS1_PIN, ICM20602_GYRO, DPS2000, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
		CK_ACC_Init(EXT_SPI, EXT_CS1_PORT, EXT_CS1_PIN, ICM20602_ACC, G16, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	#endif

	#if EXT_CS2_
		CK_GYRO_Init(EXT_SPI, EXT_CS2_PORT, EXT_CS2_PIN, ICM20602_GYRO, DPS2000, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
		CK_ACC_Init(EXT_SPI, EXT_CS2_PORT, EXT_CS2_PIN, ICM20602_ACC, G16, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	#endif

	while(!gyro.is_gyro_init){
		CK_LED_ToggleLedForMs(1, 10, 100);
		CK_LED_ToggleLedForMs(2, 10, 100);
	}
#endif

#if ACC_I2C_
    CK_ACC_Init2(ACC_I2C, ICM20602_ACC, G16, TARGET_1KHZ_US, TARGET_MAIN_TIME_US);
#endif

#if MAG_SPI_
    CK_MAGNETO_Init(MAG_SPI, MAG_CS_PORT, MAG_CS_PIN, TARGET_MAG, TARGET_250HZ_US, TARGET_MAIN_TIME_US);
#endif

#if MAG_I2C_
    CK_MAGNETO_Init2(MAG_I2C, TARGET_MAG, TARGET_10HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BARO_SPI_
    CK_BAROMETER_Init(BARO_SPI, BARO_CS_PORT, BARO_CS_PIN, TARGET_BARO, TARGET_100HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BARO_I2C_
    CK_BAROMETER_Init2(BARO_I2C, TARGET_BARO, TARGET_25HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BNO055_
   	CK_BNO055_Init(BNO055_I2C, TARGET_100HZ_US, TARGET_MAIN_TIME_US);
#endif

#if LOG_FLASH_
   	CK_FLASH_Init_External(FLASH_SPI, FLASH_CS_PORT, FLASH_CS_PIN, FLASH_W25Q128FV_);
#endif

   	pidInit(TARGET_MAIN_TIME_US);

    // Start after sensor initialization
    imuInit(TARGET_100HZ_US, TARGET_MAIN_TIME_US);

#if OSD_ONBOARD_ || OSD_PDB_ || OSD_DJI_
    CK_OSD_Init(TARGET_200HZ_US, TARGET_MAIN_TIME_US);
#endif

    flight_monitor_init(TARGET_250HZ_US, TARGET_MAIN_TIME_US);

    CK_PRINTER_Init(TARGET_MAIN_TIME_US);

#if GPS_
    //CK_GPS_WaitSatteliteFix();  // Buzzer tone 1
#endif

    CK_BUZZER_Tone2(); // Loop starting

#if MAIN_INTERRUPT_
    CK_PERIPHERAL_StartInterrupt();
#endif

    loopTimer = CK_TIME_GetMicroSec();

    while(1){

		#if SCOPE_CHECK_MAIN == 1
		CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
		#endif

        computeStartTime = CK_TIME_GetMicroSec();

        CK_INPUTSTREAM_Update();

		#if !MAIN_INTERRUPT_
		CK_RECEIVER_Update(CK_TIME_GetMicroSec());

		CK_RC_Update();
		#endif

        CK_ADJUSTMENT_Update();

        CK_GYRO_Update(CK_TIME_GetMicroSec());

        CK_ACC_Update();

		#if USE_MAG_
        CK_MAGNETO_Update();
		#endif

		#if USE_BARO_
        CK_BAROMETER_Update();
		#endif

		#if GPS_

        CK_GPS_Update();

		#endif

		#if BNO055_
        CK_BNO055_Update();
		#endif

        imuUpdateAttitude(CK_TIME_GetMicroSec());
        #if GPS_

        CK_NAVIGATION_GPSRescue();       // Rescue uses altitude hold and landing.

        CK_NAVIGATION_GPSPositionHold(); // GPS Position hold uses altitude hold.

        #endif

        #if USE_MAG_ || BNO055_

        CK_NAVIGATION_MAGHeadingHold();

        #endif

		#if (ACC1_SPI_ || ACC2_SPI_ || ACC_I2C_) && USE_BARO_

        CK_ALTITUDE_Update(CK_TIME_GetMicroSec());

        CK_LAND_LandingHandle(CK_TIME_GetMicroSec());

        #endif

        calculateRCSetpoint();

        pidController(CK_TIME_GetMicroSec());

        CK_MIXER_Update(CK_TIME_GetMicroSec());

        #if OSD_ONBOARD_ || OSD_PDB_ || OSD_DJI_

        // CORRECT HERE and its get parameters pid
        CK_OSD_Update(CK_TIME_GetMicroSec(), computeEndTime);

		#endif

        main_t1 = CK_TIME_GetMicroSec();
		#if LOG_SPI_ || LOG_SDIO_ || LOG_FLASH_

		CK_LOG_Update(computeEndTime);

		#endif
		main_t2 = CK_TIME_GetMicroSec() - main_t1;

		flight_monitor_update();

        computeEndTime = CK_TIME_GetMicroSec() - computeStartTime;

        CK_PRINTER_Update(EVERY_25MS, computeEndTime);

		#if SMART_AUDIO_
        CK_SMARTAUDIO_Update();
		#endif

		#if LED1_ & LED2_
			CK_LED_ToggleLed(1);
		#else
			CK_LED_ToggleLed(1);
		#endif

		#if SCOPE_CHECK_MAIN == 1
		CK_GPIO_ClearPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
		#endif

        while((CK_TIME_GetMicroSec() - loopTimer) < TARGET_MAIN_TIME_US);
        loopTimer = CK_TIME_GetMicroSec();

    }

}

#if MAIN_INTERRUPT_

void MAIN_INTERRUPT_Handler(void){

	HAL_TIM_IRQHandler(&htim_main_interrupt);

	#if SCOPE_CHECK_MAIN_INTERRUPT == 1
	CK_GPIO_TogglePin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
	#endif

    CK_RECEIVER_Update(CK_TIME_GetMicroSec());

    CK_RC_Update();

}
#endif







