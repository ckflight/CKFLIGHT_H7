
#include "DRIVERS/CK_ADC.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_GPIO.h"

#include "COMMUNICATION/CK_PRINTER.h"

#include "OSD/CK_MAX7456.h"
#include "OSD/CK_OSD.h"
#include "OSD/CK_FONT2.h"

// REGISTERS
#define MAX7456_VM0_REG              0x00
#define MAX7456_DMM_REG              0x04
#define MAX7456_DMAH_REG             0x05
#define MAX7456_DMAL_REG             0x06
#define MAX7456_DMDI_REG             0x07
#define MAX7456_CMM_REG              0x08
#define MAX7456_CMAH_REG             0x09
#define MAX7456_CMAL_REG             0x0A
#define MAX7456_CMDI_REG             0x0B
#define MAX7456_STAT_REG             0xA0

// PAL COMMANDS
#define MAX7456_RESET                0x02
#define MAX7456_SETUP                0x48
#define MAX7456_CLEAR                0x04

// SCREEN CONSTANTS
#define MAX7456_SCREEN_SIZE          480
#define MAX7456_SCREEN_COLUMNS        30
#define MAX7456_SCREEN_ROWS           16

#define MAX7456_LIPO_MAH_CAPACITY	1300

SPI_TypeDef* SENSOR_MAX7456_SPI;
GPIO_TypeDef* SENSOR_MAX7456_GPIO;
uint8_t SENSOR_MAX7456_PIN;

uint8_t MAX7456_OSD_BUFFER[MAX7456_SCREEN_SIZE + 1];

syncTimer_t max7456_sync;

DEBUG_TIME_t max7456_debug;

// orientation 1 plots from start, 0 from end.
int timer_line  = 15;
int timer_space = 5;
int timer_orientation = 0;
int timer_plot_freq = TARGET_10HZ_US;

int voltage_line  = 15;
int voltage_space = 1;
int voltage_orientation = 1;
int voltage_plot_freq = TARGET_10HZ_US; //8

int current_line  = 14;
int current_space = 1;
int current_orientation = 1;
int current_plot_freq = TARGET_10HZ_US; //8

int mah_line  = 14;
int mah_space = 6;
int mah_orientation = 0;
int mah_plot_freq = TARGET_10HZ_US; //8

int lipo_line = 14;
int lipo_space = 1;
int lipo_orientation = 0;
int lipo_icon_plot_freq = TARGET_10HZ_US; // 1

int firmware_rate_line  = 13;
int firmware_rate_space = 2;
int firmware_rate_orientation = 1;
int firmware_rate_plot_freq = TARGET_10HZ_US; // 4

int rssi_line = 3;
int rssi_space = 1;
int rssi_orientation = 0;
int rssi_plot_freq = TARGET_10HZ_US; // 4

int rssidBm_line = 3;
int rssidBm_space = 1;
int rssidBm_orientation = 1;
int rssi_dbm_plot_freq = TARGET_10HZ_US; // 4

int rssiLinkQuality_line = 4;
int rssiLinkQuality_space = 1;
int rssiLinkQuality_orientation = 1;
int rssi_linkQuality_plot_freq = TARGET_10HZ_US; // 4

int gps_satellite_line = 2;
int gps_satellite_space = 1;
int gps_satellite_orientation = 1;
int gps_sattelite_plot_freq = TARGET_10HZ_US; // 2

int gps_distance_line = 2;
int gps_distance_space = 5;
int gps_distance_orientation = 0;
int gps_distance_plot_freq = TARGET_10HZ_US; // 4

int gps_speed_line = 3;
int gps_speed_space = 5;
int gps_speed_orientation = 0;
int gps_speed_plot_freq = TARGET_10HZ_US; // 4

int gps_heading_destination_line = 5;
int gps_heading_destination_space = 7;
int gps_heading_destination_orientation = 0;
int gps_heading_destination_plot_freq = TARGET_10HZ_US; // 4

int gps_heading_motion_line = 6;
int gps_heading_motion_space = 7;
int gps_heading_motion_orientation = 0;
int gps_heading_motion_plot_freq = TARGET_10HZ_US; // 4

int core_temperature_line = 2;
int core_temperature_space = 3;
int core_temperature_orientation = 1;
int core_temperature_plot_freq = TARGET_10HZ_US; // 1

int altitude_line = 4;
int altitude_space = 5;
int altitude_orientation = 0;
int altitude_plot_freq = TARGET_10HZ_US; // 6

int pid_line = 5;
int pid_space = 9;
int pid_orientation = 0;
int pid_plot_freq = TARGET_10HZ_US; // 2

int tpa_line = 8;
int tpa_space = 8;
int tpa_orientation = 0;
int tpa_plot_freq = TARGET_10HZ_US; // 2

int imu_line = 14;
int imu_space = 16;
int imu_orientation = 0;
int imu_plot_freq = TARGET_10HZ_US;

int flight_mode_line = 15;
int flight_mode_space = 11;
int flight_mode_orientation = 1;
int flight_mode_plot_freq = TARGET_10HZ_US;

int altitude_mode_line = 8; // 2 lines 8 and 9
int altitude_mode_space = 1;
int altitude_mode_orientation = 1;
int altitude_mode_plot_freq = TARGET_10HZ_US;

int navigation_mode_line = 5; // 2 lines 5 and 6
int navigation_mode_space = 1;
int navigation_mode_orientation = 1;
int navigation_mode_plot_freq = TARGET_10HZ_US;

int failsafe_line = 12;
int failsafe_space = 21;
int failsafe_orientation = 1;
int failsafe_plot_freq = TARGET_10HZ_US;

int ckflight_line = 2;
int ckflight_space = 9;
int ckflight_orientation = 1;
int ckflight_plot_freq = TARGET_10HZ_US; // 1

int LOGO_START_ROW    = 4;
int LOGO_START_COLUMN = 8;

int LOGO_COLUMN_NUM   = 12;
int LOGO_ROW_NUM      = 8;

float mahSum = 0; // Both mah and lipo methods uses.

#define OSD_BUFFER_SIZE		512

uint8_t max7456_dma_buffer[OSD_BUFFER_SIZE];
uint16_t max7456_dma_index = 0;

int is_max7456_dma_done = 1;

float current_multiplier = MAH_CALIBRATION_MULTIPLIER;

float voltage_multiplier = VOLT_CALIBRATION_MULTIPLIER; // Voltage divider is calculated by multimeter and adc result

void CK_MAX7456_Init(SPI_TypeDef* spi_, GPIO_TypeDef* gpio_, uint8_t pin_){

	SENSOR_MAX7456_SPI = spi_;
	SENSOR_MAX7456_GPIO = gpio_;
	SENSOR_MAX7456_PIN = pin_;

#if USE_DMA_MAX7456

	#if USE_H7 == 1
	CK_SPI_DMA_EnableClock(OSD_MAX7456_DMA);

	CK_SPI_DMA_TCInterruptEnable(OSD_MAX7456_DMA_Stream);

	CK_SPI_DMA_InitTX(OSD_MAX7456_DMA_Stream, SENSOR_MAX7456_SPI, OSD_MAX7456_DMA_Request);

	HAL_NVIC_SetPriority(OSD_MAX7456_DMA_IRQn, MAX7456_PreemptPriority, MAX7456_SubPriority);
	HAL_NVIC_EnableIRQ(OSD_MAX7456_DMA_IRQn);

	CK_SPI_DMA_SetPeripheralAddress(OSD_MAX7456_DMA_Stream, (uint32_t)(&SENSOR_MAX7456_SPI->TXDR));

	#endif

	#if USE_F4

	CK_SPI_DMA_EnableClock(OSD_MAX7456_DMA);

	CK_SPI_DMA_TCInterruptEnable(OSD_MAX7456_DMA_Stream);

	CK_SPI_DMA_InitTX(OSD_MAX7456_DMA_Stream, OSD_MAX7456_DMA_Channel);

	HAL_NVIC_SetPriority(OSD_MAX7456_DMA_IRQn, MAX7456_PreemptPriority, MAX7456_SubPriority);
	HAL_NVIC_EnableIRQ(OSD_MAX7456_DMA_IRQn);

	CK_SPI_DMA_SetPeripheralAddress(OSD_MAX7456_DMA_Stream, (uint32_t)(&SENSOR_MAX7456_SPI->DR));

	#endif

#endif

	CK_MAX7456_Config();

	for(int i = 0; i < OSD_BUFFER_SIZE; i++){
		max7456_dma_buffer[i] = 0xFF;
	}
}

void CK_MAX7456_Config(void){

	CK_MAX7456_Reset();

	CK_SPI_WriteRegister(MAX7456_VM0_REG, MAX7456_SETUP, SENSOR_MAX7456_SPI, SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_MAX7456_ClearBuffer();

	CK_MAX7456_ClearScreen();

}

void CK_MAX7456_ShowFonts(void){

	CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_MAX7456_WriteRegister(MAX7456_DMM_REG, MAX7456_CLEAR); // Clear display

	CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_TIME_DelayMicroSec(100); // Clear display takes around 20 microsec


	CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_MAX7456_WriteRegister(MAX7456_DMM_REG, MAX7456_CLEAR); // Clear display

	CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_TIME_DelayMicroSec(100);


	// Print current chars to screen
    CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

    CK_MAX7456_WriteRegister(MAX7456_DMM_REG, 0x01); // Auto Increment Mode
    CK_MAX7456_WriteRegister(MAX7456_DMAH_REG, 0);   // Set Start Adress High
    CK_MAX7456_WriteRegister(MAX7456_DMAL_REG, 33);  // Set Start Adress Low

    for(int i = 0; i < 255; i++){
        CK_MAX7456_WriteRegister(MAX7456_DMDI_REG, (uint8_t)i);

        if ((i % 24) == 23){
        	for (int j = 0; j < 6; j++){
        		CK_MAX7456_WriteRegister(MAX7456_DMDI_REG, 0);
        	}
        }
    }

    CK_MAX7456_WriteRegister(MAX7456_DMDI_REG, 0xFF); // End String terminates auto increment.

    CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

}

uint8_t CK_MAX7456_CharUpdate(void){

	CK_MAX7456_Reset();

	CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	for(int i = 0; i < MAX7456_SCREEN_ROWS; i++){

		CK_MAX7456_WriteRegister(i + 0x10, 0x02); // Set white level

	}

	CK_MAX7456_WriteRegister(MAX7456_VM0_REG, MAX7456_SETUP); // Enable display

	CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_TIME_DelayMicroSec(1000);

	CK_MAX7456_ShowFonts();


    // Font data update
    if(sizeof(fontdata) != 16384){
    	return 0;
    }

	for(int ch = 0; ch < 256; ch++){

		CK_MAX7456_WriteChar(ch, (fontdata + (64 * ch)));

		CK_TIME_DelayMilliSec(30);

	}

    return 1;

}

void CK_MAX7456_WriteChar(uint8_t ch, const uint8_t* addr){

	CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_MAX7456_WriteRegister(MAX7456_VM0_REG, 0x40); // Disable display

	CK_MAX7456_WriteRegister(MAX7456_CMAH_REG, ch); // Set start address high

	for(int i = 0; i < 0x36; i++){

		CK_MAX7456_WriteRegister(MAX7456_CMAL_REG, i); // Set start address low

		CK_MAX7456_WriteRegister(MAX7456_CMDI_REG, *(addr + i));

	}

	CK_MAX7456_WriteRegister(MAX7456_CMM_REG, 0xA0); // Bytes from shadow ram to nvm

	while(CK_MAX7456_WriteRegister(MAX7456_STAT_REG, 0xFF) & 0x20); // Wait status busy

	CK_MAX7456_WriteRegister(MAX7456_VM0_REG, 0x4C); // Enable display vertical

	CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

}

void CK_MAX7456_Update(){

	if(is_max7456_dma_done == 1){

		#if USE_DMA_MAX7456
			is_max7456_dma_done = 0;
		#endif

		#if SCOPE_CHECK_OSD == 1
	    CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
		#endif

		uint32_t current_time = CK_TIME_GetMicroSec();

		CK_MAX7456_TimerPlot(current_time);

		CK_MAX7456_VoltagePlot(current_time);

		CK_MAX7456_CurrentPlot(current_time);

		CK_MAX7456_MahPlot(current_time);

		CK_MAX7456_LipoIconPlot(current_time);

		CK_MAX7456_FirmwareRatePlot(current_time);

		//CK_MAX7456_RssiPlot(current_time);

		CK_MAX7456_RssidBmPlot(current_time);

		CK_MAX7456_RssiLinkQualityPlot(current_time);

		//CK_MAX7456_GpsSattelitePlot(current_time);

		//CK_MAX7456_GpsDistanceToDestinationPlot(current_time);

		//CK_MAX7456_GpsGroundSpeedPlot(current_time);

		//CK_MAX7456_GpsHeadingToDestinationPlot(current_time);

		//CK_MAX7456_GpsHeadingOfMotionPlot(current_time);

		CK_MAX7456_CoreTemperaturePlot(current_time);

		CK_MAX7456_AltitudePlot(current_time);

		CK_MAX7456_PidPlot(current_time);

		CK_MAX7456_TPAPlot(current_time);

		CK_MAX7456_ImuHeadingPlot(current_time);

		CK_MAX7456_FlightModePlot(current_time);

		CK_MAX7456_AltitudeModePlot(current_time);

		//CK_MAX746_NavigationModePlot(current_time);

		CK_MAX7456_FailsafePlot(current_time);

		//CK_MAX7456_CKFLIGHTPlot(current_time);

		#if SCOPE_CHECK_OSD == 1
		CK_GPIO_ClearPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
		#endif

		#if USE_DMA_MAX7456

			#if USE_H7 == 1

			// Clean before tx operation when dcache is enabled
			// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
			SCB_CleanDCache_by_Addr ((uint32_t*)max7456_dma_buffer, OSD_BUFFER_SIZE + 32);

			// SPI DMA is not sending enough clock and byte so instead i am sending
			// fix number of bytes with 0xFF bytes for not used indexes. In this way
			// each osd data is sent correctly and rest 0xFF bytes are ignored by osd chip.

			CK_SPI_DMA_ClearFlag(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream);

			CK_SPI_DMA_SetBuffer(OSD_MAX7456_DMA_Stream, max7456_dma_buffer, OSD_BUFFER_SIZE);

			CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

			CK_SPI_DMA_Enable(OSD_MAX7456_DMA_Stream);

			CK_SPI_EnableTXDMA(SENSOR_MAX7456_SPI);

			CK_SPI_StartTransfer(SENSOR_MAX7456_SPI, OSD_BUFFER_SIZE);

			//HAL_SPI_Transmit_DMA(&hspi2, max7456_dma_buffer, max7456_dma_index);

			#endif

			#if USE_F4 == 1

			CK_SPI_DMA_ClearFlag(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream);

			CK_SPI_DMA_SetBuffer(OSD_MAX7456_DMA_Stream, max7456_dma_buffer, max7456_dma_index);

			CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

			CK_SPI_DMA_Enable(OSD_MAX7456_DMA_Stream);

			CK_SPI_EnableTXDMA(SENSOR_MAX7456_SPI);

			#endif

		#else

			CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

		#endif

	}
}

#if OSD_ONBOARD_

void OSD_MAX7456_DMA_Handler(void){

	#if USE_F4 == 1
    if(CK_SPI_DMA_IsTransferComplete(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream)){ // Transfer of one sector is done.

        CK_SPI_DMA_Disable(OSD_MAX7456_DMA_Stream);

        CK_SPI_DisableTXDMA(SENSOR_MAX7456_SPI);

        CK_SPI_DMA_ClearFlag(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream);

        CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

        is_max7456_dma_done = 1;

        max7456_dma_index = 0;

    }
	#endif

	#if USE_H7 == 1
    if(CK_SPI_DMA_IsTransferComplete(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream)){ // Transfer of one sector is done.

		CK_SPI_DMA_ClearFlag(OSD_MAX7456_DMA, OSD_MAX7456_DMA_Stream);

		CK_SPI_DisableTXDMA(SENSOR_MAX7456_SPI);

		CK_SPI_DMA_Disable(OSD_MAX7456_DMA_Stream);

		CK_SPI_Disable(SENSOR_MAX7456_SPI);

		CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

		is_max7456_dma_done = 1;

		max7456_dma_index = 0;

    }
	#endif
}

#endif

void CK_MAX7456_TimerPlot(uint32_t currentTime){

	static uint32_t preTime = 0;
	float delta = currentTime - preTime;

	static int tmpCounter = 0;
	static uint32_t timerCounter = 0;
	static uint8_t minutes, seconds;
	int timer_num1, timer_num2, timer_num3, timer_num4;

	int index = CK_MAX7456_GetPlotIndex(timer_line, timer_orientation, timer_space);

	int start_index = index;

	if(delta >= timer_plot_freq){

		preTime = currentTime;

		tmpCounter++;

		if(tmpCounter == 10){
			timerCounter++;
			tmpCounter = 0;
		}

		minutes = timerCounter / 60;
		seconds = timerCounter % 60;

		timer_num1 = minutes / 10;
		timer_num2 = minutes % 10;

		timer_num3 = seconds / 10;
		timer_num4 = seconds % 10;

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(timer_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(timer_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(':');

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(timer_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(timer_num4);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;

#else
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(timer_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(timer_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(':');
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(timer_num3);
		MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetCharacterAddress(timer_num4);

		CK_MAX7456_Write_BufferElements(start_index, index);
#endif
	}
}

void CK_MAX7456_VoltagePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	float voltage;
	int volt_temp;
	int volt_num1, volt_num2, volt_num3, volt_num4;

	int index = CK_MAX7456_GetPlotIndex(voltage_line, voltage_orientation, voltage_space);
	int start_index = index;

	if(delta >= voltage_plot_freq){

		preTime = current_time;

		voltage = (float)osd_packet.voltage;
		voltage /= 100.0f;

		voltage = voltage * voltage_multiplier;

		/* Get Each Float Digit(2 Decimal After Point) */
		volt_temp = voltage * 100;
		volt_num1 = volt_temp / 1000;
		volt_temp = volt_temp % 1000;

		volt_num2 = volt_temp / 100;
		volt_temp = volt_temp % 100;

		volt_num3 = volt_temp / 10;
		volt_num4 = volt_temp % 10;

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(volt_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(volt_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('.');

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(volt_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(volt_num4);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(VOLTAGE_SYMBOL);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;

#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(volt_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(volt_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('.');
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(volt_num3);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(volt_num4);
		MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(VOLTAGE_SYMBOL);

		CK_MAX7456_Write_BufferElements(start_index, index);
#endif

	}
}

void CK_MAX7456_CurrentPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	float current;
	int current_temp;
	int current_num1, current_num2, current_num3, current_num4;

	int index = CK_MAX7456_GetPlotIndex(current_line, current_orientation, current_space);
	int start_index = index;

	if(delta >= current_plot_freq){

		preTime = current_time;

		current = (float)osd_packet.current;
		current /= 100.0f; // osd multiplies with 100 for getting 2 decimal points

		/*
		 * When a 4in1 esc is used its current sensing parameters are not known. One thing can be done is
		 * to measure the end of flight mah for example for 1300mah battery if flight mah says 400mah note it.
		 * Charge the battery and check mah at the end of charging is complete.
		 * Calibration Multiply = ((Charger mah) * ((currentADC * 1000) / 52.5)) / flight_mah
		 */

		// I = (Vout * 1000) / (0.5 x resistor)
		current = (current * 1000) / (CURRENT_RESISTOR * 0.5f); // Current Sens Vout = I*0.5m*105K/1K = I*52.5m

		// Add calibration multiply here
		current *= current_multiplier;

		/* Get Each Float Digit(1 Decimal After Point) */
		current_temp = current * 10;

		if(current_temp >= 0 && current_temp <= 99){ // 0.0A to 9.9A

		current_num1 = ' ';// Blank Space
		current_num2 = ' ';// Blank Space

		current_num3 = current_temp / 10;
		current_num4 = current_temp % 10;

		}
		else if(current_temp > 99 && current_temp <= 999){ // 10.0A to 99.9A

		current_num1 = ' ';// Blank Space
		current_num2 = current_temp / 100;
		current_temp = current_temp % 100;

		current_num3 = current_temp / 10;
		current_num4 = current_temp % 10;

		}
		else if(current_temp > 999 && current_temp <= 9999){ // 100.0A to 999.9A

		current_num1 = current_temp / 1000;
		current_temp = current_temp % 1000;

		current_num2 = current_temp / 100;
		current_temp = current_temp % 100;

		current_num3 = current_temp / 10;
		current_num4 = current_temp % 10;

		}


#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(current_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(current_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(current_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('.');

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(current_num4);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(AMPER_SYMBOL);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;

#else
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(current_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(current_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(current_num3);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('.');
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(current_num4);
		MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetSpecialCharacterAddress(AMPER_SYMBOL);

		CK_MAX7456_Write_BufferElements(start_index, index);
#endif
	}

}

void CK_MAX7456_MahPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	float currentADC, mahAmpere;
	int mah_temp;
	int mah_num1, mah_num2, mah_num3, mah_num4;
	const int mAhUpdateMs = 100;

	int index = CK_MAX7456_GetPlotIndex(mah_line, mah_orientation, mah_space);
	int start_index = index;

	if(delta >= mah_plot_freq){

		preTime = current_time;

		currentADC = (float)osd_packet.current;
		currentADC /= 100.0f; // osd multiplies with 100 for getting 2 decimal point

		/*
		 * When a 4in1 esc is used its current sensing parameters are not known. One thing can be done is
		 * to measure the end of flight mah for example for 1300mah battery if flight mah says 400mah note it.
		 * Charge the battery and check mah at the end of charging is complete.
		 * Calibration Multiply = ((Charger mah) * ((currentADC * 1000) / 52.5)) / flight_mah
		 */

		// I = (Vout * 1000) / (0.5 x resistor)
		mahAmpere = (currentADC * 1000) / (CURRENT_RESISTOR * 0.5f); // Current Sens Vout = I*0.5m*105K/1K = I*52.5m

		// Add calibration multiply here
		mahAmpere *= current_multiplier;

		/* Ampere to milliAmpere */
		mahAmpere *= 1000;

		/* Add mA per 100ms (10Hz) to sum */
		mahAmpere /= 3600;                  // mA per second
		mahAmpere /= (1000 / mAhUpdateMs);  // mA per 100ms

		mahSum += mahAmpere; // mAh Consumed

		mah_temp = (int)mahSum;

		if(mah_temp >= 0 && mah_temp <= 9){
			mah_num1 = ' ';// Blank Space
			mah_num2 = ' ';// Blank Space
			mah_num3 = ' ';// Blank Space
			mah_num4 = mah_temp;

		}
		else if(mah_temp > 9 && mah_temp <= 99){
			mah_num1 = ' ';// Blank Space
			mah_num2 = ' ';// Blank Space
			mah_num3 = mah_temp / 10;
			mah_num4 = mah_temp % 10;
		}
		else if(mah_temp > 99 && mah_temp <= 999){
			mah_num1 = ' ';// Blank Space
			mah_num2 = mah_temp / 100;

			mah_temp = mah_temp % 100;

			mah_num3 = mah_temp / 10;
			mah_num4 = mah_temp % 10;

		}
		else if(mah_temp > 999 && mah_temp <= 9999){
			mah_num1 = mah_temp / 1000;
			mah_temp = mah_temp % 1000;

			mah_num2 = mah_temp / 100;
			mah_temp = mah_temp % 100;

			mah_num3 = mah_temp / 10;
			mah_num4 = mah_temp % 10;
		}

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(mah_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(mah_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(mah_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(mah_num4);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(MAH_SYMBOL);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;

#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(mah_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(mah_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(mah_num3);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(mah_num4);
		MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetSpecialCharacterAddress(MAH_SYMBOL);
		// 418 has LIPO Symbol, updated with lipoIconPlot

		CK_MAX7456_Write_BufferElements(start_index, index);

#endif

	}

}

void CK_MAX7456_LipoIconPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int lipoRange1 = 1 * (MAX7456_LIPO_MAH_CAPACITY / 6);
	int lipoRange2 = 2 * (MAX7456_LIPO_MAH_CAPACITY / 6);
	int lipoRange3 = 3 * (MAX7456_LIPO_MAH_CAPACITY / 6);
	int lipoRange4 = 4 * (MAX7456_LIPO_MAH_CAPACITY / 6);
	int lipoRange5 = 5 * (MAX7456_LIPO_MAH_CAPACITY / 6);
	int lipoRange6 = 6 * (MAX7456_LIPO_MAH_CAPACITY / 6);

	static int lipoCounter;

	int index = CK_MAX7456_GetPlotIndex(lipo_line, lipo_orientation, lipo_space);

	if(delta >= lipo_icon_plot_freq){

		preTime = current_time;

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(index >> 8);

		#endif

		if(mahSum < lipoRange1){
			/* LIPO FULL */
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_100);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_100);
			#endif
		}
		else if(mahSum >= lipoRange1 && mahSum < lipoRange2){
			/* LIPO 80 */
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_80);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_80);
			#endif
		}
		else if(mahSum >= lipoRange2 && mahSum < lipoRange3){
			/* LIPO 64 */
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_64);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_64);
			#endif

		}
		else if(mahSum >= lipoRange3 && mahSum < lipoRange4){
			/* LIPO 48 */
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_48);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_48);
			#endif
		}
		else if(mahSum >= lipoRange4 && mahSum < lipoRange5){
			/* LIPO 32 */
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_32);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_32);
			#endif
		}
		else if(mahSum >= lipoRange5 && mahSum < lipoRange6){
			/* LIPO 16 */
			lipoCounter++;
			if(lipoCounter == 50){
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');
				#else
					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetCharacterAddress(' ');
				#endif

			}
			else if(lipoCounter == 100){
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_16);
				#else
					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_16);
				#endif
				lipoCounter = 0;
			}
		}
		else if(mahSum >= lipoRange6){
			/* LIPO 0 */
			lipoCounter++;
			if(lipoCounter == 25){
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');
				#else
					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetCharacterAddress(' ');
				#endif
			}
			else if(lipoCounter == 40){
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetLipoAddress(LIPO_0);
				#else
					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetLipoAddress(LIPO_0);
				#endif

				lipoCounter = 0;
			}
		}

#if USE_DMA_MAX7456
		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;
#else
		CK_MAX7456_Write_BufferElements(index, index);
#endif
	}
}

void CK_MAX7456_FirmwareRatePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int rate_temp;
	int rate_num1, rate_num2, rate_num3;

	int index = CK_MAX7456_GetPlotIndex(firmware_rate_line, firmware_rate_orientation, firmware_rate_space);
	int start_index = index;

	if(delta >= firmware_rate_plot_freq){

		preTime = current_time;

		// 8000 will be printed as 8.0K so divide with 100
		rate_temp = osd_packet.freqResult / 100;

		if(rate_temp >= 0 && rate_temp < 10){
			rate_num1 = ' ';
			rate_num2 = 0;
			rate_num3 = rate_temp % 10;
		}
		else if(rate_temp >= 10 && rate_temp < 100){
			rate_num1 = ' ';
			rate_num2 = rate_temp / 10;
			rate_num3 = rate_temp % 10;
		}
		else if(rate_temp >= 100 && rate_temp < 1000){
			rate_num1 = rate_temp / 100;
			rate_temp = rate_temp % 100;

			rate_num2 = rate_temp / 10;
			rate_num3 = rate_temp % 10;
		}

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rate_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rate_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('.');

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rate_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('K');

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = 0xFF;

#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rate_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rate_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('.');
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rate_num3);
		MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetCharacterAddress('K');

		CK_MAX7456_Write_BufferElements(start_index, index);

#endif

	}
}

void CK_MAX7456_RssiPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	float rssi1, rssi2;
	int rssi_temp;
	int rssi_num1, rssi_num2, rssi_num3;
	static int rssiLowCounter;

	int index = CK_MAX7456_GetPlotIndex(rssi_line, rssi_orientation, rssi_space);
	int start_index = index;

	if(delta >= rssi_plot_freq){

		preTime = current_time;

		rssi1 = osd_packet.rssi * 100.0f;
		rssi2 = rssi1 / 2000.0f;
		rssi_temp = (int)rssi2;

		if(rssi_temp >= 0 && rssi_temp < 10){
			rssi_num1 = ' ';
			rssi_num2 = ' ';
			rssi_num3 = rssi_temp % 10;
		}
		else if(rssi_temp >= 10 && rssi_temp < 100){
			rssi_num1 = ' ';
			rssi_num2 = rssi_temp / 10;
			rssi_num3 = rssi_temp % 10;
		}
		else if(rssi_temp >= 100 && rssi_temp < 1000){
			rssi_num1 = rssi_temp / 100;
			rssi_temp = rssi_temp % 100;

			rssi_num2 = rssi_temp / 10;
			rssi_num3 = rssi_temp % 10;
		}

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);

#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);

#endif
		// If rssi level is lower than 50 blink rssi symbol
		if(rssi_temp < 50){
		rssiLowCounter++;
		if(rssiLowCounter == 2){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				#else

					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetCharacterAddress(' ');
				#endif
		}
		else if(rssiLowCounter == 4){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif

			rssiLowCounter = 0;
		}
		}
		else{
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif
		}

		#if USE_DMA_MAX7456
			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;
		#else
			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif

	}
}

void CK_MAX7456_RssidBmPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int rssi_temp;
	int rssi_num1, rssi_num2, rssi_num3, rssi_num4;
	static int rssiLowCounter;

	int index = CK_MAX7456_GetPlotIndex(rssidBm_line, rssidBm_orientation, rssidBm_space);
	int start_index = index;

	if(delta >= rssi_dbm_plot_freq){

		preTime = current_time;

		rssi_temp = osd_packet.rssi_dBm;

		if(rssi_temp < 0 && rssi_temp >= -9){
			rssi_num1 = '-';// Blank Space
			rssi_num2 = ' ';// Blank Space
			rssi_num3 = ' ';
			rssi_num4 = rssi_temp * -1;
		}
		else if(rssi_temp < -9 && rssi_temp >= -99){
			rssi_num1 = '-';// Blank Space
			rssi_num2 = ' ';
			rssi_num3 = (rssi_temp / 10) * -1;
			rssi_num4 = (rssi_temp % 10) * -1;
		}
		else if(rssi_temp < -99 && rssi_temp >= -999){
			rssi_num1 = '-';
			rssi_num2 = (rssi_temp / 100) * -1;
			rssi_num3 = ((rssi_temp % 100) / 10) * -1;
			rssi_num4 = ((rssi_temp % 100) % 10) * -1;
		}
#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num4);

#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num4);

#endif
		// If rssi level is lower than 50 blink rssi symbol
		if(rssi_temp < -100){
		rssiLowCounter++;
		if(rssiLowCounter == 2){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				#else

					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetCharacterAddress(' ');
				#endif
		}
		else if(rssiLowCounter == 4){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif

			rssiLowCounter = 0;
		}
		}
		else{
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif
		}

		#if USE_DMA_MAX7456
			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;
		#else
			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif

	}
}

void CK_MAX7456_RssiLinkQualityPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int rssi_temp;
	int rssi_num1, rssi_num2, rssi_num3;
	static int rssiLowCounter;

	int index = CK_MAX7456_GetPlotIndex(rssiLinkQuality_line, rssiLinkQuality_orientation, rssiLinkQuality_space);
	int start_index = index;

	if(delta >= rssi_linkQuality_plot_freq){

		preTime = current_time;

		rssi_temp = osd_packet.rssi_link_quality;

		if(rssi_temp >= 0 && rssi_temp < 10){
			rssi_num1 = ' ';
			rssi_num2 = ' ';
			rssi_num3 = rssi_temp % 10;
		}
		else if(rssi_temp >= 10 && rssi_temp < 100){
			rssi_num1 = ' ';
			rssi_num2 = rssi_temp / 10;
			rssi_num3 = rssi_temp % 10;
		}
		else if(rssi_temp >= 100 && rssi_temp < 1000){
			rssi_num1 = rssi_temp / 100;
			rssi_temp = rssi_temp % 100;

			rssi_num2 = rssi_temp / 10;
			rssi_num3 = rssi_temp % 10;
		}

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);


#else

		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num2);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(rssi_num3);

#endif
		// If rssi level is lower than 50 blink rssi symbol
		if(rssi_temp < -100){
		rssiLowCounter++;
		if(rssiLowCounter == 2){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				#else

					MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetCharacterAddress(' ');
				#endif
		}
		else if(rssiLowCounter == 4){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif

			rssiLowCounter = 0;
		}
		}
		else{
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#else
				MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(RSSI_SYMBOL1);
			#endif
		}

		#if USE_DMA_MAX7456
			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;
		#else
			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif

	}
}


void CK_MAX7456_GpsSattelitePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int gpsNumOfSat_temp;
	int gpsNumOfSat_num1, gpsNumOfSat_num2;
	static uint32_t satFixedCounter = 0;

	int index = CK_MAX7456_GetPlotIndex(gps_satellite_line, gps_satellite_orientation, gps_satellite_space);
	int start_index = index;

	if(delta >= gps_sattelite_plot_freq){

		preTime = current_time;

		gpsNumOfSat_temp = osd_packet.gpsNumOfSat;

		// First plot num of Sattelite than plot sattelite symbol
		if(gpsNumOfSat_temp >= 0 && gpsNumOfSat_temp < 10){
			gpsNumOfSat_num1 = ' ';
			gpsNumOfSat_num2 = gpsNumOfSat_temp % 10;
		}
		else if(gpsNumOfSat_temp >= 10 && gpsNumOfSat_temp < 100){
			gpsNumOfSat_num1 = gpsNumOfSat_temp / 10;
			gpsNumOfSat_num2 = gpsNumOfSat_temp % 10;
		}

#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
		max7456_dma_buffer[max7456_dma_index++] = 0x01;

		// Start address high
		max7456_dma_buffer[max7456_dma_index++] = 0x06;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

		// Start address low
		max7456_dma_buffer[max7456_dma_index++] = 0x05;
		max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsNumOfSat_num1);

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsNumOfSat_num2);

#else
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsNumOfSat_num1);
		MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsNumOfSat_num2);
#endif
		// If sat. is fixed symbol is solid stationary else it blinks.
		// Print sat logo even flight controller did not started yet.
		//static bool blinkOrder = false;

		if(osd_packet.gpsNumOfSat >= 2){
			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE1_SYMBOL);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE2_SYMBOL);

			#else
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE1_SYMBOL);
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE2_SYMBOL);
			#endif
		}
		else{
			satFixedCounter++;
			if(satFixedCounter == 3){
				// Clear symbol for blink effect.
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				#else
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
				#endif

			}
			if(satFixedCounter == 5){
				#if USE_DMA_MAX7456
					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE1_SYMBOL);

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE2_SYMBOL);

				#else
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE1_SYMBOL);
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetSpecialCharacterAddress(SATELITE2_SYMBOL);
				#endif

				satFixedCounter = 0;
			}
		}
		#if USE_DMA_MAX7456
			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;
		#else
			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif
	}
}

void CK_MAX7456_GpsDistanceToDestinationPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static int gpsDistance_temp;
	int gpsDistance_num1, gpsDistance_num2, gpsDistance_num3, gpsDistance_num4;

	int index = CK_MAX7456_GetPlotIndex(gps_distance_line, gps_distance_orientation, gps_distance_space);
	int start_index = index;

	if(delta >= gps_distance_plot_freq){

		preTime = current_time;

		gpsDistance_temp = osd_packet.gps_distanceToDestination;

		if(gpsDistance_temp >= 0 && gpsDistance_temp <= 9){
			gpsDistance_num1 = ' ';// Blank Space
			gpsDistance_num2 = ' ';// Blank Space
			gpsDistance_num3 = ' ';// Blank Space
			gpsDistance_num4 = gpsDistance_temp;
		}
		else if(gpsDistance_temp > 9 && gpsDistance_temp <= 99){
			gpsDistance_num1 = ' ';// Blank Space
			gpsDistance_num2 = ' ';// Blank Space
			gpsDistance_num3 = gpsDistance_temp / 10;
			gpsDistance_num4 = gpsDistance_temp % 10;
		}
		else if(gpsDistance_temp > 99 && gpsDistance_temp <= 999){
			gpsDistance_num1 = ' ';// Blank Space
			gpsDistance_num2 = gpsDistance_temp / 100;

			gpsDistance_num3 = (gpsDistance_temp % 100) / 10;
			gpsDistance_num4 = (gpsDistance_temp % 100) % 10;
		}
		else if(gpsDistance_temp > 999 && gpsDistance_temp <= 9999){
			gpsDistance_num1 = gpsDistance_temp / 1000;
			gpsDistance_num2 = (gpsDistance_temp % 1000) / 100;

			gpsDistance_num3 = ((gpsDistance_temp % 1000) % 100) / 10;
			gpsDistance_num4 = ((gpsDistance_temp % 1000) % 100) % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num4);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(METERS_SYMBOL);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num3);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsDistance_num4);
			MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetSpecialCharacterAddress(METERS_SYMBOL);

			CK_MAX7456_Write_BufferElements(start_index, index);

		#endif
	}
}

void CK_MAX7456_GpsGroundSpeedPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int gpsSpeed_temp;
	int gpsSpeed_num1, gpsSpeed_num2, gpsSpeed_num3, gpsSpeed_num4;

	int index = CK_MAX7456_GetPlotIndex(gps_speed_line, gps_speed_orientation, gps_speed_space);
	int start_index = index;

	if(delta >= gps_speed_plot_freq){

		preTime = current_time;

		gpsSpeed_temp = osd_packet.gps_groundSpeed / 100; // received data is cm/s convert to m/sec

		if(gpsSpeed_temp >= 0 && gpsSpeed_temp <= 9){
			gpsSpeed_num1 = ' ';// Blank Space
			gpsSpeed_num2 = ' ';// Blank Space
			gpsSpeed_num3 = ' ';// Blank Space
			gpsSpeed_num4 = gpsSpeed_temp;
		}
		else if(gpsSpeed_temp > 9 && gpsSpeed_temp <= 99){
			gpsSpeed_num1 = ' ';// Blank Space
			gpsSpeed_num2 = ' ';// Blank Space
			gpsSpeed_num3 = gpsSpeed_temp / 10;
			gpsSpeed_num4 = gpsSpeed_temp % 10;
		}
		else if(gpsSpeed_temp > 99 && gpsSpeed_temp <= 999){
			gpsSpeed_num1 = ' ';// Blank Space
			gpsSpeed_num2 = gpsSpeed_temp / 100;

			gpsSpeed_temp = gpsSpeed_temp % 100;

			gpsSpeed_num3 = gpsSpeed_temp / 10;
			gpsSpeed_num4 = gpsSpeed_temp % 10;
		}
		else if(gpsSpeed_temp > 999 && gpsSpeed_temp <= 9999){
			gpsSpeed_num1 = gpsSpeed_temp / 1000;
			gpsSpeed_temp = gpsSpeed_temp % 1000;

			gpsSpeed_num2 = gpsSpeed_temp / 100;
			gpsSpeed_temp = gpsSpeed_temp % 100;

			gpsSpeed_num3 = gpsSpeed_temp / 10;
			gpsSpeed_num4 = gpsSpeed_temp % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num4);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(MS_SYMBOL);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num3);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsSpeed_num4);
			MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetSpecialCharacterAddress(MS_SYMBOL);

			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif
	}
}

void CK_MAX7456_GpsHeadingToDestinationPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static int gpsHeading_temp;
	int gpsHeading_num1, gpsHeading_num2, gpsHeading_num3;

	int index = CK_MAX7456_GetPlotIndex(gps_heading_destination_line, gps_heading_destination_orientation, gps_heading_destination_space);
	int start_index = index;

	if(delta >= gps_heading_destination_plot_freq){

		preTime = current_time;

		gpsHeading_temp = osd_packet.gps_headingToDestination;

		// Plot heading number
		if(gpsHeading_temp >= 0 && gpsHeading_temp < 10){
			gpsHeading_num1 = ' ';
			gpsHeading_num2 = ' ';
			gpsHeading_num3 = gpsHeading_temp % 10;
		}
		else if(gpsHeading_temp >= 10 && gpsHeading_temp < 100){
			gpsHeading_num1 = ' ';
			gpsHeading_num2 = gpsHeading_temp / 10;
			gpsHeading_num3 = gpsHeading_temp % 10;
		}
		else if(gpsHeading_temp >= 100 && gpsHeading_temp < 1000){
			gpsHeading_num1 = gpsHeading_temp / 100;

			gpsHeading_num2 = (gpsHeading_temp % 100) / 10;
			gpsHeading_num3 = (gpsHeading_temp % 100) % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('H');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('D');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(':');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetArrowCharacter(gpsHeading_temp);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('H');
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('D');
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(':');

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num3);

			MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetArrowCharacter(gpsHeading_temp);

			CK_MAX7456_Write_BufferElements(start_index, index);

		#endif
	}
}

void CK_MAX7456_GpsHeadingOfMotionPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static int gpsHeading_temp;
	int gpsHeading_num1, gpsHeading_num2, gpsHeading_num3;

	int index = CK_MAX7456_GetPlotIndex(gps_heading_motion_line, gps_heading_motion_orientation, gps_heading_motion_space);
	int start_index = index;

	if(delta >= gps_heading_motion_plot_freq){

		preTime = current_time;

		gpsHeading_temp = osd_packet.gps_headingOfMotion;

		// Plot heading number
		if(gpsHeading_temp >= 0 && gpsHeading_temp < 10){
			gpsHeading_num1 = ' ';
			gpsHeading_num2 = ' ';
			gpsHeading_num3 = gpsHeading_temp % 10;
		}
		else if(gpsHeading_temp >= 10 && gpsHeading_temp < 100){
			gpsHeading_num1 = ' ';
			gpsHeading_num2 = gpsHeading_temp / 10;
			gpsHeading_num3 = gpsHeading_temp % 10;
		}
		else if(gpsHeading_temp >= 100 && gpsHeading_temp < 1000){
			gpsHeading_num1 = gpsHeading_temp / 100;

			gpsHeading_num2 = (gpsHeading_temp % 100) / 10;
			gpsHeading_num3 = (gpsHeading_temp % 100) % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('H');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress('M');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(':');

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetArrowCharacter(gpsHeading_temp);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('H');
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress('M');
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(':');

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(gpsHeading_num3);

			MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetArrowCharacter(gpsHeading_temp);

			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif
	}
}

void CK_MAX7456_CoreTemperaturePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int core_temperature_temp;
	int core_temperature_num1, core_temperature_num2;

	int index = CK_MAX7456_GetPlotIndex(core_temperature_line, core_temperature_orientation, core_temperature_space);
	int start_index = index;

	if(delta >= core_temperature_plot_freq){

		preTime = current_time;

		core_temperature_temp = osd_packet.cpu_core_temperature;

		// First plot num of Sattelite than plot sattelite symbol
		if(core_temperature_temp >= 0 && core_temperature_temp < 10){
			core_temperature_num1 = ' ';
			core_temperature_num2 = core_temperature_temp % 10;
		}
		else if(core_temperature_temp >= 10 && core_temperature_temp < 100){
			core_temperature_num1 = core_temperature_temp / 10;
			core_temperature_num2 = core_temperature_temp % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(core_temperature_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(core_temperature_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(CELCIUS_SYMBOL);

		#else
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(core_temperature_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(core_temperature_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetSpecialCharacterAddress(CELCIUS_SYMBOL);
		#endif

		#if USE_DMA_MAX7456
			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;
		#else
			CK_MAX7456_Write_BufferElements(start_index, index);
		#endif
	}
}

void CK_MAX7456_AltitudePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int altitude_temp;
	int altitude_num1, altitude_num2, altitude_num3, altitude_num4;

	int index = CK_MAX7456_GetPlotIndex(altitude_line, altitude_orientation, altitude_space);
	int start_index = index;

	if(delta >= altitude_plot_freq){

		preTime = current_time;

		altitude_temp = osd_packet.estimatedAltitude; // received height is in cm osd prints in meters.

		if(altitude_temp < 0 && altitude_temp >= -9){
			altitude_num1 = ' ';// Blank Space
			altitude_num2 = ' ';// Blank Space
			altitude_num3 = '-';
			altitude_num4 = altitude_temp * -1;
		}
		else if(altitude_temp < -9 && altitude_temp >= -99){
			altitude_num1 = ' ';// Blank Space
			altitude_num2 = '-';
			altitude_num3 = (altitude_temp / 10) * -1;
			altitude_num4 = (altitude_temp % 10) * -1;
		}
		else if(altitude_temp < -99 && altitude_temp >= -999){
			altitude_num1 = '-';
			altitude_num2 = (altitude_temp / 100) * -1;
			altitude_num3 = ((altitude_temp % 100) / 10) * -1;
			altitude_num4 = ((altitude_temp % 100) % 10) * -1;
		}
		else if(altitude_temp >= 0 && altitude_temp <= 9){
			altitude_num1 = ' ';// Blank Space
			altitude_num2 = ' ';// Blank Space
			altitude_num3 = ' ';// Blank Space
			altitude_num4 = altitude_temp;
		}
		else if(altitude_temp > 9 && altitude_temp <= 99){
			altitude_num1 = ' ';// Blank Space
			altitude_num2 = ' ';// Blank Space
			altitude_num3 = altitude_temp / 10;
			altitude_num4 = altitude_temp % 10;
		}
		else if(altitude_temp > 99 && altitude_temp <= 999){
			altitude_num1 = ' ';// Blank Space
			altitude_num2 = altitude_temp / 100;

			altitude_temp = altitude_temp % 100;

			altitude_num3 = altitude_temp / 10;
			altitude_num4 = altitude_temp % 10;
		}
		else if(altitude_temp > 999 && altitude_temp <= 9999){
			altitude_num1 = altitude_temp / 1000;
			altitude_temp = altitude_temp % 1000;

			altitude_num2 = altitude_temp / 100;
			altitude_temp = altitude_temp % 100;

			altitude_num3 = altitude_temp / 10;
			altitude_num4 = altitude_temp % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(altitude_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(altitude_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(altitude_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(altitude_num4);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetSpecialCharacterAddress(CM_SYMBOL);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(altitude_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(altitude_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(altitude_num3);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(altitude_num4);
			MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetSpecialCharacterAddress(CM_SYMBOL);

			CK_MAX7456_Write_BufferElements(start_index, index);

		#endif
	}
}

void CK_MAX7456_PidPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int index = 0;
	int start_index = 0;

	int pid_p, pid_i, pid_i2, pid_d;
	int pid_p_num1, pid_p_num2;
	int pid_i_num1, pid_i_num2, pid_i_num3;
	int pid_d_num1, pid_d_num2;

	if(osd_packet.is_adjustment_on){

		if(delta >= pid_plot_freq){

			for(int i = 0; i < 4; i++){

				if(i == 0){
					pid_p = osd_packet.pid_roll[0];
					pid_i = osd_packet.pid_roll[1];
					pid_i2 = pid_i;
					pid_d = osd_packet.pid_roll[2];
				}
				else if(i == 1){
					pid_p = osd_packet.pid_pitch[0];
					pid_i = osd_packet.pid_pitch[1];
					pid_i2 = pid_i;
					pid_d = osd_packet.pid_pitch[2];
				}
				else if(i == 2){
					pid_p = osd_packet.pid_yaw[0];
					pid_i = osd_packet.pid_yaw[1];
					pid_i2 = pid_i;
					pid_d = osd_packet.pid_yaw[2];
				}
				else if(i == 3){
					// These parameters are minimum d terms for roll pitch yaw axis.
					// Instead of defining new parameters and adding loops below i will use
					// p i and d integers.
					pid_p = osd_packet.pid_roll[4]; // dmax
					pid_i = osd_packet.pid_pitch[4];
					pid_i2 = pid_i;
					pid_d = osd_packet.pid_yaw[4];
				}

				index = CK_MAX7456_GetPlotIndex(pid_line + i, pid_orientation, pid_space);
				start_index = index;

				if(pid_p >= 0 && pid_p <= 9){
					pid_p_num1 = ' ';        // Blank Space
					pid_p_num2 = pid_p % 10; // Blank Space
				}
				else if(pid_p > 9 && pid_p <= 99){
					pid_p_num1 = pid_p / 10;
					pid_p_num2 = pid_p % 10;
				}

				if(pid_i >= 0 && pid_i <= 9){
					pid_i_num1 = ' ';        // Blank Space
					pid_i_num2 = pid_i % 10; // Blank Space
				}
				else if(pid_i > 9 && pid_i <= 99){
					pid_i_num1 = pid_i / 10;
					pid_i_num2 = pid_i % 10;
				}
				else if(pid_i >= 100 && pid_i < 1000){
					pid_i_num1 = pid_i / 100;
					pid_i = pid_i % 100;

					pid_i_num2 = pid_i / 10;
					pid_i_num3 = pid_i % 10;
				}

				if(pid_d >= 0 && pid_d <= 9){
					pid_d_num1 = ' ';        // Blank Space
					pid_d_num2 = pid_d % 10; // Blank Space
				}
				else if(pid_d > 9 && pid_d <= 99){
					pid_d_num1 = pid_d / 10;
					pid_d_num2 = pid_d % 10;
				}

				if(pid_i2 >= 100){

					#if USE_DMA_MAX7456

						max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
						max7456_dma_buffer[max7456_dma_index++] = 0x01;

						// Start address high
						max7456_dma_buffer[max7456_dma_index++] = 0x06;
						max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

						// Start address low
						max7456_dma_buffer[max7456_dma_index++] = 0x05;
						max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_p_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_p_num2);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_i_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_i_num2);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_i_num3);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_d_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_d_num2);

					#else

						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_p_num1);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_p_num2);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_i_num1);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_i_num2);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_i_num3);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_d_num1);
						MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetCharacterAddress(pid_d_num2);

					#endif
				}
				else{

					#if USE_DMA_MAX7456

						max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
						max7456_dma_buffer[max7456_dma_index++] = 0x01;

						// Start address high
						max7456_dma_buffer[max7456_dma_index++] = 0x06;
						max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

						// Start address low
						max7456_dma_buffer[max7456_dma_index++] = 0x05;
						max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_p_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_p_num2);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_i_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_i_num2);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_d_num1);

						max7456_dma_buffer[max7456_dma_index++] = 0x07;
						max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(pid_d_num2);

					#else

						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_p_num1);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_p_num2);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_i_num1);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_i_num2);
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
						MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(pid_d_num1);
						MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetCharacterAddress(pid_d_num2);
					#endif
				}


				#if USE_DMA_MAX7456

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = 0xFF;

				#else
					CK_MAX7456_Write_BufferElements(start_index, index);
				#endif

			}

		}

	}
	else{

		if(delta >= pid_plot_freq){

			for(int i = 0; i < 4; i++){

				index = CK_MAX7456_GetPlotIndex(pid_line + i, pid_orientation, pid_space);
				start_index = index;

				#if USE_DMA_MAX7456

					max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
					max7456_dma_buffer[max7456_dma_index++] = 0x01;

					// Start address high
					max7456_dma_buffer[max7456_dma_index++] = 0x06;
					max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

					// Start address low
					max7456_dma_buffer[max7456_dma_index++] = 0x05;
					max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				#else

					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
					MAX7456_OSD_BUFFER[index]   = CK_MAX7456_GetCharacterAddress(' ');

				#endif


				#if USE_DMA_MAX7456

					max7456_dma_buffer[max7456_dma_index++] = 0x07;
					max7456_dma_buffer[max7456_dma_index++] = 0xFF;

				#else
					CK_MAX7456_Write_BufferElements(start_index, index);
				#endif

			}

		}

	}

}

void CK_MAX7456_TPAPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	int tpa_breakpoint_temp, tpa_rate_temp;
	int tpa_breakpoint_num1, tpa_breakpoint_num2, tpa_breakpoint_num3, tpa_breakpoint_num4;
	int tpa_rate_num1, tpa_rate_num2;
	UNUSED(tpa_rate_num1);
	UNUSED(tpa_rate_num2);

	int index = CK_MAX7456_GetPlotIndex(tpa_line, tpa_orientation, tpa_space);
	int start_index = index;

	if(osd_packet.is_adjustment_on){

		if(delta >= tpa_plot_freq){

			preTime = current_time;

			tpa_breakpoint_temp = osd_packet.tpa_breakpoint;
			tpa_rate_temp = osd_packet.tpa_rate;

			tpa_breakpoint_num1 = tpa_breakpoint_temp / 1000;
			tpa_breakpoint_temp = tpa_breakpoint_temp % 1000;

			tpa_breakpoint_num2 = tpa_breakpoint_temp / 100;
			tpa_breakpoint_temp = tpa_breakpoint_temp % 100;

			tpa_breakpoint_num3 = tpa_breakpoint_temp / 10;
			tpa_breakpoint_num4 = tpa_breakpoint_temp % 10;

			tpa_rate_num1 = tpa_rate_temp / 10;
			tpa_rate_num2 = tpa_rate_temp % 10;

			#if USE_DMA_MAX7456

				max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
				max7456_dma_buffer[max7456_dma_index++] = 0x01;

				// Start address high
				max7456_dma_buffer[max7456_dma_index++] = 0x06;
				max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

				// Start address low
				max7456_dma_buffer[max7456_dma_index++] = 0x05;
				max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_rate_num1);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_rate_num2);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num1);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num2);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num3);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num4);

			#else
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num1);
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num2);
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num3);
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(tpa_breakpoint_num4);
			#endif

			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = 0xFF;
			#else
				CK_MAX7456_Write_BufferElements(start_index, index);
			#endif
		}

	}
	else{

		if(delta >= tpa_plot_freq){

			preTime = current_time;

			tpa_breakpoint_temp = osd_packet.tpa_breakpoint;
			tpa_rate_temp = osd_packet.tpa_rate;

			tpa_breakpoint_num1 = tpa_breakpoint_temp / 1000;
			tpa_breakpoint_temp = tpa_breakpoint_temp % 1000;

			tpa_breakpoint_num2 = tpa_breakpoint_temp / 100;
			tpa_breakpoint_temp = tpa_breakpoint_temp % 100;

			tpa_breakpoint_num3 = tpa_breakpoint_temp / 10;
			tpa_breakpoint_num4 = tpa_breakpoint_temp % 10;

			tpa_rate_num1 = tpa_rate_temp / 10;
			tpa_rate_num2 = tpa_rate_temp % 10;

			#if USE_DMA_MAX7456

				max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
				max7456_dma_buffer[max7456_dma_index++] = 0x01;

				// Start address high
				max7456_dma_buffer[max7456_dma_index++] = 0x06;
				max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

				// Start address low
				max7456_dma_buffer[max7456_dma_index++] = 0x05;
				max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(' ');

			#else
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
				MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(' ');
			#endif

			#if USE_DMA_MAX7456
				max7456_dma_buffer[max7456_dma_index++] = 0x07;
				max7456_dma_buffer[max7456_dma_index++] = 0xFF;
			#else
				CK_MAX7456_Write_BufferElements(start_index, index);
			#endif
		}
	}
}

void CK_MAX7456_ImuHeadingPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static int imuHeading_temp;
	int imuHeading_num1, imuHeading_num2, imuHeading_num3;

	int index = CK_MAX7456_GetPlotIndex(imu_line, imu_orientation, imu_space);
	int start_index = index;

	if(delta >= imu_plot_freq){

		preTime = current_time;

		imuHeading_temp = osd_packet.imu_heading;

		// Plot heading number
		if(imuHeading_temp >= 0 && imuHeading_temp < 10){
			imuHeading_num1 = ' ';
			imuHeading_num2 = ' ';
			imuHeading_num3 = imuHeading_temp % 10;
		}
		else if(imuHeading_temp >= 10 && imuHeading_temp < 100){
			imuHeading_num1 = ' ';
			imuHeading_num2 = imuHeading_temp / 10;
			imuHeading_num3 = imuHeading_temp % 10;
		}
		else if(imuHeading_temp >= 100 && imuHeading_temp < 1000){
			imuHeading_num1 = imuHeading_temp / 100;

			imuHeading_num2 = (imuHeading_temp % 100) / 10;
			imuHeading_num3 = (imuHeading_temp % 100) % 10;
		}

		#if USE_DMA_MAX7456

			max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
			max7456_dma_buffer[max7456_dma_index++] = 0x01;

			// Start address high
			max7456_dma_buffer[max7456_dma_index++] = 0x06;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_index;

			// Start address low
			max7456_dma_buffer[max7456_dma_index++] = 0x05;
			max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_index >> 8);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num1);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num2);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num3);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetArrowCharacter(imuHeading_temp);

			max7456_dma_buffer[max7456_dma_index++] = 0x07;
			max7456_dma_buffer[max7456_dma_index++] = 0xFF;

		#else

			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num1);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num2);
			MAX7456_OSD_BUFFER[index++] = CK_MAX7456_GetCharacterAddress(imuHeading_num3);

			MAX7456_OSD_BUFFER[index] = CK_MAX7456_GetArrowCharacter(imuHeading_temp);

			CK_MAX7456_Write_BufferElements(start_index, index);

		#endif

	}
}

void CK_MAX7456_FlightModePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	if(delta >= flight_mode_plot_freq){

		preTime = current_time;

		int index = CK_MAX7456_GetPlotIndex(flight_mode_line, flight_mode_orientation, flight_mode_space);

		#if USE_DMA_MAX7456

			CK_MAX7456_OSD_DMA_Packet(index, flight_mode_line, flight_mode_space, "        ", 8);

			if(osd_packet.currentFlightMode == 1){
				index = CK_MAX7456_GetPlotIndex(flight_mode_line, flight_mode_orientation, flight_mode_space + 2);
				CK_MAX7456_OSD_DMA_Packet(index, flight_mode_line, flight_mode_space + 2, "ACRO   ", 7);
			}
			else if(osd_packet.currentFlightMode == 2){
				index = CK_MAX7456_GetPlotIndex(flight_mode_line, flight_mode_orientation, flight_mode_space + 1);
				CK_MAX7456_OSD_DMA_Packet(index, flight_mode_line, flight_mode_space + 1, "HORIZON", 7);
			}
			else if(osd_packet.currentFlightMode == 3){
				index = CK_MAX7456_GetPlotIndex(flight_mode_line, flight_mode_orientation, flight_mode_space + 2);
				CK_MAX7456_OSD_DMA_Packet(index, flight_mode_line, flight_mode_space, "ANGLE  ", 7);
			}


		#else

			CK_MAX7456_OSD_FillBuffer(flight_mode_line, flight_mode_space, "        ", 8); // Clear 401 to 408

			if(osd_packet.currentFlightMode == 2){
				CK_MAX7456_OSD_FillBuffer(flight_mode_line, flight_mode_space + 1, "HORIZON", 7);
			}
			else if(osd_packet.currentFlightMode == 3){
				CK_MAX7456_OSD_FillBuffer(flight_mode_line, flight_mode_space + 2, "LEVEL  ", 7);
			}
			else{
				CK_MAX7456_OSD_FillBuffer(flight_mode_line, flight_mode_space + 2, "ACRO   ", 7);
			}

			CK_MAX7456_Write_BufferElements(index, index + 7);
		#endif

	}
}

void CK_MAX7456_AltitudeModePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	if(delta >= altitude_mode_plot_freq){

		preTime = current_time;

		#if USE_DMA_MAX7456

			int index1 = CK_MAX7456_GetPlotIndex(altitude_mode_line, altitude_mode_orientation, altitude_mode_space);
			int index2 = CK_MAX7456_GetPlotIndex(altitude_mode_line + 1, altitude_mode_orientation, altitude_mode_space);

			if(osd_packet.currentAltitudeMode == 1){
				CK_MAX7456_OSD_DMA_Packet(index1, altitude_mode_line, altitude_mode_space, "ALT ", 4);
				CK_MAX7456_OSD_DMA_Packet(index2, altitude_mode_line + 1, altitude_mode_space, "HOLD", 4);
			}
			else if(osd_packet.currentAltitudeMode == 2){
				CK_MAX7456_OSD_DMA_Packet(index1, altitude_mode_line, altitude_mode_space, "AUTO", 4);
				CK_MAX7456_OSD_DMA_Packet(index2, altitude_mode_line + 1, altitude_mode_space, "LAND", 4);
			}
			else{
				CK_MAX7456_OSD_DMA_Packet(index1, altitude_mode_line, altitude_mode_space, "    ", 4);
				CK_MAX7456_OSD_DMA_Packet(index2, altitude_mode_line + 1, altitude_mode_space, "    ", 4);
			}

		#else

			if(osd_packet.currentAltitudeMode == 1){
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line, altitude_mode_space, "ALT ", 4);
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line + 1, altitude_mode_space, "HOLD", 4);
			}
			else if(osd_packet.currentAltitudeMode == 2){
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line, altitude_mode_space, "AUTO", 4);
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line + 1, altitude_mode_space, "LAND", 4);
			}
			else{
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line, altitude_mode_space, "    ", 4); // 211 to 217
				CK_MAX7456_OSD_FillBuffer(altitude_mode_line + 1, altitude_mode_space, "    ", 4); // 241 to 247
			}

			int index1 = CK_MAX7456_GetPlotIndex(altitude_mode_line, altitude_mode_orientation, altitude_mode_space);
			int index2 = CK_MAX7456_GetPlotIndex(altitude_mode_line + 1, altitude_mode_orientation, altitude_mode_space);

			CK_MAX7456_Write_BufferElements(index1, index1 + 6);
			CK_MAX7456_Write_BufferElements(index2, index2 + 6);

		#endif
	}
}

void CK_MAX746_NavigationModePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	if(delta >= navigation_mode_plot_freq){

		preTime = current_time;

		#if USE_DMA_MAX7456

			int index1 = CK_MAX7456_GetPlotIndex(navigation_mode_line, navigation_mode_orientation, navigation_mode_space);
			int index2 = CK_MAX7456_GetPlotIndex(navigation_mode_line + 1, navigation_mode_orientation, navigation_mode_space);

			if(osd_packet.currentNavigationMode == 1){
				CK_MAX7456_OSD_DMA_Packet(index1, navigation_mode_line, navigation_mode_space, "GPS", 3);
				CK_MAX7456_OSD_DMA_Packet(index2, navigation_mode_line + 1, navigation_mode_space, "RESCUE", 6);
			}
			else if(osd_packet.currentNavigationMode == 2){
				CK_MAX7456_OSD_DMA_Packet(index1, navigation_mode_line, navigation_mode_space, "GPS", 3);
				CK_MAX7456_OSD_DMA_Packet(index2, navigation_mode_line + 1, navigation_mode_space, "HOLD  ", 6);
			}
			else if(osd_packet.currentNavigationMode == 3){
				CK_MAX7456_OSD_DMA_Packet(index1, navigation_mode_line, navigation_mode_space, "MAG", 3);
				CK_MAX7456_OSD_DMA_Packet(index2, navigation_mode_line + 1, navigation_mode_space, "HOLD  ", 6);
			}
			else{
				CK_MAX7456_OSD_DMA_Packet(index1, navigation_mode_line, navigation_mode_space, "   ", 3);
				CK_MAX7456_OSD_DMA_Packet(index2, navigation_mode_line + 1, navigation_mode_space, "      ", 6);
			}

		#else
			if(osd_packet.currentNavigationMode == 1){
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line, navigation_mode_space, "GPS", 3);
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line + 1, navigation_mode_space, "RESCUE", 6);
			}
			else if(osd_packet.currentNavigationMode == 2){
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line, navigation_mode_space, "GPS", 3);
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line + 1, navigation_mode_space, "HOLD", 4);
			}
			else if(osd_packet.currentNavigationMode == 3){
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line, navigation_mode_space, "MAG", 3);
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line + 1, navigation_mode_space, "HOLD", 4);
			}
			else{
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line, navigation_mode_space, "   ", 3);    // 121 to 123
				CK_MAX7456_OSD_FillBuffer(navigation_mode_line + 1, navigation_mode_space, "      ", 6); // 151 to 156
			}

			int index1 = CK_MAX7456_GetPlotIndex(navigation_mode_line, navigation_mode_orientation, navigation_mode_space);
			int index2 = CK_MAX7456_GetPlotIndex(navigation_mode_line + 1, navigation_mode_orientation, navigation_mode_space);

			CK_MAX7456_Write_BufferElements(index1, index1 + 2);
			CK_MAX7456_Write_BufferElements(index2, index2 + 5);

		#endif
	}
}

void CK_MAX7456_FailsafePlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static int failsafeCounter = 0;

	if(delta >= failsafe_plot_freq){

		preTime = current_time;

		#if USE_DMA_MAX7456

			int index1 = CK_MAX7456_GetPlotIndex(failsafe_line, failsafe_orientation, failsafe_space);

			if(osd_packet.isFailSafe == 1){

				failsafeCounter++;

				if(failsafeCounter == 5){
					CK_MAX7456_OSD_DMA_Packet(index1, failsafe_line, failsafe_space, "        ", 8);
				}
				else if(failsafeCounter == 10){
					CK_MAX7456_OSD_DMA_Packet(index1, failsafe_line, failsafe_space, "FAILSAFE", 8);
					failsafeCounter = 0;
				}
			}
			else{
				CK_MAX7456_OSD_DMA_Packet(index1, failsafe_line, failsafe_space, "        ", 8);
			}

		#else

			if(osd_packet.isFailSafe == 1){

				failsafeCounter++;

				if(failsafeCounter == 5){
					CK_MAX7456_OSD_FillBuffer(failsafe_line, failsafe_space, "        ", 8);
				}
				else if(failsafeCounter == 10){
					CK_MAX7456_OSD_FillBuffer(failsafe_line, failsafe_space, "FAILSAFE", 8);
					failsafeCounter = 0;
				}
			}
			else{
				CK_MAX7456_OSD_FillBuffer(failsafe_line, failsafe_space, "        ", 8);
			}

			int index = CK_MAX7456_GetPlotIndex(failsafe_line, failsafe_orientation, failsafe_space);
			CK_MAX7456_Write_BufferElements(index, index + 8);

		#endif
	}
}

void CK_MAX7456_CKFLIGHTPlot(uint32_t current_time){

	static uint32_t preTime = 0;
	float delta = current_time - preTime;

	static bool isArmedPlotted    = false;
	static bool isDisarmedPlotted = false;

	if(delta >= ckflight_plot_freq && !osd_packet.is_adjustment_on){

		preTime = current_time;

		if(osd_packet.isArmed && !isArmedPlotted){

			isArmedPlotted    = true;
			isDisarmedPlotted = false;

			#if USE_DMA_MAX7456

				int index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space);

				CK_MAX7456_OSD_DMA_Packet(index1, ckflight_line, ckflight_space, "           ", 11);

				int index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space);

				CK_MAX7456_OSD_DMA_Packet(index2, ckflight_line + 1, ckflight_space, "           ", 11);

				CK_MAX7456_ClearLogo(); // Normal spi

				index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space + 2);

				CK_MAX7456_OSD_DMA_Packet(index1, ckflight_line, ckflight_space + 2, "CKFLIGHT", 8);

			#else

				// Clear disarmed text
				CK_MAX7456_OSD_FillBuffer(ckflight_line, ckflight_space, "           ", 11);
				CK_MAX7456_OSD_FillBuffer(ckflight_line + 1, ckflight_space, "           ", 11);

				int index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space);
				int index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space);

				CK_MAX7456_Write_BufferElements(index1, index1 + 10);
				CK_MAX7456_Write_BufferElements(index2, index2 + 10);

				CK_MAX7456_ClearLogo();

				// More space would be good during flight.
				// Disarmed mode already plots logo etc. to indicate.
				CK_MAX7456_OSD_FillBuffer(ckflight_line, ckflight_space + 2, "CKFLIGHT", 8);

				index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space + 2);

				CK_MAX7456_Write_BufferElements(index1, index1 + 7);

			#endif

		}
		else if(!osd_packet.isArmed && !isDisarmedPlotted){

			isArmedPlotted    = false;
			isDisarmedPlotted = true;

			#if USE_DMA_MAX7456

				int index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space + 2);

				CK_MAX7456_OSD_DMA_Packet(index1, ckflight_line, ckflight_space + 2, "        ", 8);

				int index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space + 3);

				CK_MAX7456_OSD_DMA_Packet(index2, ckflight_line + 1, ckflight_space + 3, "     ", 5);


				index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space);

				CK_MAX7456_OSD_DMA_Packet(index1, ckflight_line, ckflight_space, "CKFLIGHT BY", 11);

				index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space);

				CK_MAX7456_OSD_DMA_Packet(index2, ckflight_line + 1, ckflight_space, "CENK KESKIN", 11);

				CK_MAX7456_PrintLogo(); // Normal spi

			#else

				// Clear
				CK_MAX7456_OSD_FillBuffer(ckflight_line, ckflight_space + 2, "        ", 8);
				CK_MAX7456_OSD_FillBuffer(ckflight_line + 1, ckflight_space + 3, "     ", 5);

				int index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space + 2);
				int index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space + 3);

				CK_MAX7456_Write_BufferElements(index1, index1 + 7);
				CK_MAX7456_Write_BufferElements(index2, index2 + 7);

				CK_MAX7456_OSD_FillBuffer(ckflight_line, ckflight_space, "CKFLIGHT BY", 11);
				CK_MAX7456_OSD_FillBuffer(ckflight_line + 1, ckflight_space, "CENK KESKIN", 11);

				index1 = CK_MAX7456_GetPlotIndex(ckflight_line, ckflight_orientation, ckflight_space);
				index2 = CK_MAX7456_GetPlotIndex(ckflight_line + 1, ckflight_orientation, ckflight_space);

				CK_MAX7456_Write_BufferElements(index1, index1 + 10);
				CK_MAX7456_Write_BufferElements(index2, index2 + 10);

				CK_MAX7456_PrintLogo();
			#endif



		}
	}
}

void CK_MAX7456_PrintLogo(void){

  uint8_t logo_address = 0x78; // START ADDRESS OF START_LOGO
  int idx;
  int tmp1;

  // Used image is 144x144 so 12*12(columns) and 18*8(rows)
  for(int row = 0; row < LOGO_ROW_NUM; row++){

      idx = ((LOGO_START_ROW - 1) * 30) + (row * 30);
      idx += LOGO_START_COLUMN;

      tmp1 = idx;
      for(int column = 0; column < LOGO_COLUMN_NUM; column++){

    	  MAX7456_OSD_BUFFER[idx] = logo_address;
          idx += 1;
          logo_address++;
      }

      CK_MAX7456_Write_BufferElements(tmp1, tmp1 + LOGO_COLUMN_NUM);
  }

}

void CK_MAX7456_ClearLogo(void){

  int idx;
  int tmp1;

  // Used image is 144x144 so 12*12(columns) and 18*8(rows)
  for(int row = 0; row < LOGO_ROW_NUM; row++){

      idx = ((LOGO_START_ROW - 1) * 30) + (row * 30);
      idx += LOGO_START_COLUMN;

      tmp1 = idx;
      for(int column = 0; column < LOGO_COLUMN_NUM; column++){

    	  MAX7456_OSD_BUFFER[idx] = 0;
          idx += 1;
      }

      CK_MAX7456_Write_BufferElements(tmp1, tmp1 + LOGO_COLUMN_NUM);
  }

}


void CK_MAX7456_Write_BufferElements(int startElement, int lastElement){

	int x, local_count, start_address;
    uint8_t osd_char;

    local_count = (lastElement - startElement) + 1;
    start_address = startElement;

    CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

    CK_MAX7456_WriteRegister(MAX7456_DMM_REG, 0x01);                  		 // Auto Increment Mode
    CK_MAX7456_WriteRegister(MAX7456_DMAL_REG, (uint8_t)(start_address));    // Set Start Adress Low
    CK_MAX7456_WriteRegister(MAX7456_DMAH_REG, (uint8_t)(start_address>>8)); // Set Start Adress High

    x = 0;
    while(local_count) // Write Line
    {
        osd_char =  MAX7456_OSD_BUFFER[start_address + x];
        CK_MAX7456_WriteRegister(MAX7456_DMDI_REG, osd_char);
        x++;
        local_count--;
    }

    CK_MAX7456_WriteRegister(MAX7456_DMDI_REG, 0xFF); // End String terminates auto increment.

    CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

}

void CK_MAX7456_ClearBuffer(void){

	for(int i = 0; i < MAX7456_SCREEN_SIZE; i++){
		MAX7456_OSD_BUFFER[i] = 0x00;
	}
}

void CK_MAX7456_OSD_DMA_Packet(int start_address, int osdRow, int osdColumn, const char str[], int len){

	max7456_dma_buffer[max7456_dma_index++] = 0x04; // Auto increment
	max7456_dma_buffer[max7456_dma_index++] = 0x01;

	// Start address high
	max7456_dma_buffer[max7456_dma_index++] = 0x06;
	max7456_dma_buffer[max7456_dma_index++] = (uint8_t)start_address;

	// Start address low
	max7456_dma_buffer[max7456_dma_index++] = 0x05;
	max7456_dma_buffer[max7456_dma_index++] = (uint8_t)(start_address >> 8);

	CK_MAX7456_OSD_FillBuffer(osdRow, osdColumn, str, len);

	max7456_dma_buffer[max7456_dma_index++] = 0x07;
	max7456_dma_buffer[max7456_dma_index++] = 0xFF;

}

void CK_MAX7456_OSD_FillBuffer(int osdRow, int osdColumn, const char str[], int len){

	for(int i = 0; i < len; i++){

		#if USE_DMA_MAX7456

		max7456_dma_buffer[max7456_dma_index++] = 0x07;
		max7456_dma_buffer[max7456_dma_index++] = CK_MAX7456_GetCharacterAddress((uint8_t) str[i]);

		#else
    		MAX7456_OSD_BUFFER[((osdRow - 1) * MAX7456_SCREEN_COLUMNS) + osdColumn + i] =  CK_MAX7456_GetCharacterAddress((uint8_t) str[i]);
		#endif
    }
}

void CK_MAX7456_ClearScreen(void){

	CK_GPIO_ClearPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_MAX7456_WriteRegister(MAX7456_DMM_REG, MAX7456_CLEAR);

	CK_GPIO_SetPin(SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

}

void CK_MAX7456_Reset(void){

	CK_SPI_WriteRegister(MAX7456_VM0_REG, MAX7456_RESET, SENSOR_MAX7456_SPI, SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN);

	CK_TIME_DelayMilliSec(100);
}

void CK_MAX7456_CheckReset(uint32_t current_ms){

	static uint32_t preTime = 0;
	static uint32_t delta = 0;

	delta = current_ms - preTime;

	if(delta >= 100){

		preTime = current_ms;

		uint8_t read_data = 0;

		CK_SPI_ReadRegisterMulti(MAX7456_VM0_REG | 0x80, SENSOR_MAX7456_SPI, SENSOR_MAX7456_GPIO, SENSOR_MAX7456_PIN, &read_data, 1);

		if(read_data != MAX7456_SETUP){

			CK_MAX7456_Config();
		}

	}

}

uint8_t CK_MAX7456_WriteRegister(uint8_t reg, uint8_t data){

	uint8_t resp = 0;

	CK_SPI_Transfer(SENSOR_MAX7456_SPI, reg);

	resp = CK_SPI_Transfer(SENSOR_MAX7456_SPI, data);

	return resp;

}

int CK_MAX7456_GetPlotIndex(int line, int orientation, int space){

	int index = 0;

	// Plot from start
	if(orientation){
		index = ((line - 1) * MAX7456_SCREEN_COLUMNS) + space;
	}
	// Plot from end of line
	else{
		index = ((line * MAX7456_SCREEN_COLUMNS) - 1) - space;
	}

	return index;

}

uint8_t CK_MAX7456_GetLipoAddress(MAX7456_Symbols symbol){

    uint8_t charAddress;

    if(symbol == LIPO_0){
        charAddress = 0x46;
    }
    else if(symbol == LIPO_16){
        charAddress = 0x45;
    }
    else if(symbol == LIPO_32){
        charAddress = 0x44;
    }
    else if(symbol == LIPO_48){
        charAddress = 0x43;
    }
    else if(symbol == LIPO_64){
        charAddress = 0x42;
    }
    else if(symbol == LIPO_80){
        charAddress = 0x41;
    }
    else if(symbol == LIPO_100){
        charAddress = 0x40;
    }
    else if(symbol == LIPO_BAT){
        charAddress = 0x47;
    }
    else{
        charAddress = 0x00; // Return blank space if error
    }
    return charAddress;
}

uint8_t CK_MAX7456_GetCharacterAddress(int character){

    // Check font_layout to see character mapping
    uint8_t charAddress;

    if (character == 32){ // Blank Space
        charAddress = 0x00;
    }
    else if (character == 45){ // -
        charAddress = 0x4D;
    }
    else if (character == 46){ // .
        charAddress = 0x49;
    }
    else if (character == 58){ // :
        charAddress = 0x48;
    }
    else if (character == 0){ // 0
        // Number 0 address is 0x01
        charAddress = 0x01;
    }
    else if ((character > 0) && (character < 10)){ // 1 to 9
        // Number 1-9 address is 0x02 to 0x0A
        charAddress = character + 1;
    }
    else if ((character > 64) && (character < 91)){ // A to Z
        // Uppercase Letters A to Z address is 0x0B to 0x24
        charAddress = (character - 54);
    }
    else{
        charAddress = 0x00; // Return blank space if error
    }
    return charAddress;
}

uint8_t CK_MAX7456_GetSpecialCharacterAddress(MAX7456_Symbols symbol){

    uint8_t charAddress;
    if(symbol == RSSI_SYMBOL1){
        charAddress = 0x25;
    }
    else if(symbol == RSSI_SYMBOL2){
        charAddress = 0x4A;
    }

    else if(symbol == CELCIUS_SYMBOL){
        charAddress = 0x26;
    }

    else if(symbol == SATELITE1_SYMBOL){
        charAddress = 0x27;
    }
    else if(symbol == SATELITE2_SYMBOL){
        charAddress = 0x28;
    }

    else if(symbol == NORTH_SYMBOL){
        charAddress = 0x29;
    }
    else if(symbol == SOUTH_SYMBOL){
        charAddress = 0x2A;
    }
    else if(symbol == EAST_SYMBOL){
        charAddress = 0x2B;
    }
    else if(symbol == WEST_SYMBOL){
        charAddress = 0x2C;
    }

    else if(symbol == AMPER_SYMBOL){
        charAddress = 0x2D;
    }
    else if(symbol == VOLTAGE_SYMBOL){
        charAddress = 0x2E;
    }
    else if(symbol == MAH_SYMBOL){
        charAddress = 0x2F;
    }
    else if(symbol == METERS_SYMBOL){
        charAddress = 0x4B;
    }
    else if(symbol == MS_SYMBOL){
        charAddress = 0x4C;
    }
    else if(symbol == CM_SYMBOL){
        charAddress = 0x4E;
    }

    else if(symbol == HEADING_ARROW_0){
        charAddress = 0x38;
    }
    else if(symbol == HEADING_ARROW_23){
        charAddress = 0x37;
    }
    else if(symbol == HEADING_ARROW_45){
        charAddress = 0x36;
    }
    else if(symbol == HEADING_ARROW_67){
        charAddress = 0x35;
    }
    else if(symbol == HEADING_ARROW_90){
        charAddress = 0x34;
    }
    else if(symbol == HEADING_ARROW_113){
        charAddress = 0x33;
    }
    else if(symbol == HEADING_ARROW_135){
        charAddress = 0x32;
    }
    else if(symbol == HEADING_ARROW_157){
        charAddress = 0x31;
    }
    else if(symbol == HEADING_ARROW_180){
        charAddress = 0x30;
    }
    else if(symbol == HEADING_ARROW_202){
        charAddress = 0x3f;
    }
    else if(symbol == HEADING_ARROW_225){
        charAddress = 0x3e;
    }
    else if(symbol == HEADING_ARROW_247){
        charAddress = 0x3d;
    }
    else if(symbol == HEADING_ARROW_270){
        charAddress = 0x3c;
    }
    else if(symbol == HEADING_ARROW_292){
        charAddress = 0x3b;
    }
    else if(symbol == HEADING_ARROW_315){
        charAddress = 0x3a;
    }
    else if(symbol == HEADING_ARROW_336){
        charAddress = 0x39;
    }


    else{
        charAddress = 0x00; // Return blank space if error
    }
    return charAddress;
}

uint8_t CK_MAX7456_GetArrowCharacter(int angle){

	if((angle > 337 && angle <= 360) || (angle >= 0 && angle <= 11)){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_0);
	}
	else if(angle > 11 && angle <= 33){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_23);
	}
	else if(angle > 33 && angle <= 55){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_45);
	}
	else if(angle > 55 && angle <= 77){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_67);
	}
	else if(angle > 77 && angle <= 99){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_90);
	}
	else if(angle > 99 && angle <= 121){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_113);
	}
	else if(angle > 121 && angle <= 143){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_135);
	}
	else if(angle > 143 && angle <= 165){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_157);
	}
	else if(angle > 165 && angle <= 192){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_180);
	}
	else if(angle > 192 && angle <= 214){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_202);
	}
	else if(angle > 214 && angle <= 236){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_225);
	}
	else if(angle > 236 && angle <= 258){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_247);
	}
	else if(angle > 258 && angle <= 280){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_270);
	}
	else if(angle > 280 && angle <= 304){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_292);
	}
	else if(angle > 304 && angle <= 326){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_315);
	}
	else if(angle > 326 && angle <= 337){
		return CK_MAX7456_GetSpecialCharacterAddress(HEADING_ARROW_336);
	}
	return 0;

}
















