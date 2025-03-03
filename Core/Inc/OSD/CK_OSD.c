
#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_ADC.h"

#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/CK_MSP.h"

#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/pid_init.h"
#include "FLIGHT/CK_ADJUSTMENT.h"
#include "FLIGHT/CK_CRSF.h"

#include "MOTION/CK_IMU.h"

#include "OSD/CK_OSD.h"
#include "OSD/CK_MAX7456.h"
#include "OSD/CK_MSP_OSD.h"

#define START_BYTE              0xCC
#define END_BYTE                0xAA
#define SEND_BUFFER_SIZE        512

uint8_t sendBuffer[SEND_BUFFER_SIZE];

int sendBufferCurrentIndex;

uint32_t mainLoopSum; // sum mainLoop microsecond results and send average to osd

DEBUG_TIME_t osd_debug;

USART_TypeDef* COMMUNICATION_OSD_UART;

SPI_TypeDef* COMMUNICATION_MAX7456_SPI;
GPIO_TypeDef* COMMUNICATION_MAX7456_GPIO;
uint8_t COMMUNICATION_MAX7456_PIN;

int channel_counter = 0;

OSD_PACKET_s osd_packet;

bool is_interrupt_started = false;

int tx_buffer_size = 0;
int osd_counter1 = 0;

void CK_OSD_Init(uint32_t osdT, uint32_t mainT){

	osd_packet.sync.syncCounter = 0;

	osd_packet.sync.targetLoopTime = osdT;

	osd_packet.sync.syncRate = osdT / mainT;

    osd_packet.mainLoopTime = mainT;

    mainLoopSum = 0;

    is_interrupt_started = false;

    #if OSD_PDB_

    	COMMUNICATION_OSD_UART = OSD_PDB_USART;

		USART_CONFIGURATION_ config;
	    config.tx_gpio_type			= OSD_PDB_UART_TX_GPIO;
		config.tx_gpio_pin			= OSD_PDB_UART_TX_PIN;
		config.tx_af				= OSD_PDB_UART_TX_AF;

		config.interrupt 			= TX_INTERRUPT;
		config.mode 				= TX_ONLY;
		config.parity 				= NO_PARITY;
		config.stop_bit 			= STOP_BIT1;
		config.baudrate 			= 115200;
		config.usart 				= COMMUNICATION_OSD_UART;
		config.use_circular_buffer 	= false;

		config.preempt_priority		= OSD_PreemptPriority;
		config.sub_priority			= OSD_SubPriority;

		CK_UART_Init(&config, NULL);

		CK_UART_TXEnable(COMMUNICATION_OSD_UART);

		#if USE_INTERRUPT_OSD
		// Nothing is needed
		#endif

		sendBufferCurrentIndex = 0;

    #endif

	#if OSD_ONBOARD_

		COMMUNICATION_MAX7456_SPI 	= OSD_SPI;
		COMMUNICATION_MAX7456_GPIO 	= OSD_CS_PORT;
		COMMUNICATION_MAX7456_PIN 	= OSD_CS_PIN;

		CK_MAX7456_Init(COMMUNICATION_MAX7456_SPI, COMMUNICATION_MAX7456_GPIO, COMMUNICATION_MAX7456_PIN);

	#endif

	#if OSD_DJI_

    	COMMUNICATION_OSD_UART = OSD_DJI_USART;

		USART_CONFIGURATION_ config;
		config.tx_gpio_type			= OSD_DJI_UART_TX_GPIO;
		config.tx_gpio_pin			= OSD_DJI_UART_TX_PIN;
		config.tx_af				= OSD_DJI_UART_TX_AF;

		config.interrupt 			= TX_INTERRUPT;
		config.mode 				= TX_ONLY;
		config.parity 				= NO_PARITY;
		config.stop_bit 			= STOP_BIT1;
		config.baudrate 			= 115200;
		config.usart 				= COMMUNICATION_OSD_UART;
		config.use_circular_buffer 	= false;

		config.preempt_priority		= OSD_PreemptPriority;
		config.sub_priority			= OSD_SubPriority;

		CK_UART_Init(&config, NULL);
		CK_UART_TXEnable(COMMUNICATION_OSD_UART);

		#if USE_INTERRUPT_OSD
		// Nothing is needed
		#endif

		sendBufferCurrentIndex = 0;

	#endif

	#if USE_DMA_OSD

		CK_UART_DMA_EnableClock(OSD_DMA);

		CK_UART_DMA_ClearFlag(OSD_DMA, OSD_DMA_Stream);

		#if USE_H7
		//CK_UART_DMA_InitTX(OSD_DMA_Stream, OSD_DMA_Channel);
		#endif

		#if USE_F4
		CK_UART_DMA_InitTX(OSD_DMA_Stream, OSD_DMA_Channel);
		#endif

		CK_UART_DMA_TCInterruptEnable(OSD_DMA_Stream);

		HAL_NVIC_EnableIRQ(OSD_DMA_IRQn);

		#if USE_H7
		CK_UART_DMA_SetPeripheralAddress(OSD_DMA_Stream, (uint32_t)(&COMMUNICATION_OSD_UART->TDR));
		#endif
		#if USE_F4
		CK_UART_DMA_SetPeripheralAddress(OSD_DMA_Stream, (uint32_t)(&COMMUNICATION_OSD_UART->DR));
		#endif
		CK_UART_DMA_TXEnable(COMMUNICATION_OSD_UART);

	#endif

}

void CK_OSD_Update(uint32_t currentTime, uint32_t loopTime){

	osd_packet.sync.syncCounter++;

    mainLoopSum += loopTime;

    if(osd_packet.sync.syncCounter >= osd_packet.sync.syncRate){

        #if defined(DEBUG_TIMING)
        osd_debug.start_time = CK_TIME_GetMicroSec();
        #endif

    	// For 8K loop, computeEndTime will be less than 125 microsec
		// because now loop appx. ends around 80 to 100 microsec so
		// OSD Loop freq will show average compute speed such as 12K not 8K
		mainLoopSum /= osd_packet.sync.syncCounter;

		osd_packet.sync.syncCounter = 0;

        //CK_OSD_GetFlightData();

		mainLoopSum = 0;

		#if OSD_PDB_

		//Sending with polling. It is working but slow.
		//CK_OSD_SendPacketPolling();

		if(!is_interrupt_started){

			CK_OSD_WriteUartBuffer();

			#if USE_INTERRUPT_OSD
			// Sending with interrupt. It is working.
			CK_OSD_SendPacketInterrupt();
			#endif

			#if USE_DMA_OSD
			CK_OSD_SendPacketDMA();
			#endif

		}

		#endif

		#if OSD_ONBOARD_

		CK_MAX7456_Update();

		#endif

		#if OSD_DJI_

		CK_MSP_OSD_Update(currentTime);

		if(!is_interrupt_started){

			tx_buffer_size = CK_MSP_OSD_PacketSequence(sendBuffer);

			#if USE_INTERRUPT_OSD
			// Sending with interrupt. It is working.
			CK_OSD_SendPacketInterrupt();
			#endif

			#if USE_DMA_OSD
			CK_OSD_SendPacketDMA();
			#endif
		}

		#endif

        #if defined(DEBUG_TIMING)
        osd_debug.update_time = CK_TIME_GetMicroSec() - osd_debug.start_time;
        #endif

    }
}

void CK_OSD_GetFlightData(void){

	osd_packet.gps_distanceToDestination = (uint16_t)gps.distanceToDestination/100; // sends m

    osd_packet.gps_headingToDestination = (uint16_t)gps.headingToDestination/100; // x100 precision deg

    osd_packet.gps_headingOfMotion = (uint16_t)(gps.groundCourse);

    osd_packet.gps_groundSpeed = (uint16_t)(gps.groundSpeed); // sends cm/s

    osd_packet.estimatedAltitude = (int32_t)(CK_ALTITUDE_GetEstimatedAltitude()); // sends cm

    osd_packet.gpsNumOfSat = (uint8_t)gps.numOfSattelite;

    osd_packet.gpsSatFix = (uint8_t)gps.satteliteFix;

#if CRSF_
    osd_packet.rssi_dBm = CK_CRSF_GetRSSI_dBm();
#endif
#if SBUS_
    osd_packet.rssi = CK_RCData[RC_RSSI_CHANNEL];
#endif

    osd_packet.currentFlightMode = CK_RECEIVER_GetFlightMode();

    osd_packet.currentNavigationMode = CK_RECEIVER_GetNavigationMode();

    osd_packet.currentAltitudeMode = CK_RECEIVER_GetAltitudeMode();

    osd_packet.isArmed = CK_RECEIVER_isArmed();

    osd_packet.isFailSafe = isFailsafeActive();

    osd_packet.freqResult = (1000000 / mainLoopSum);

    osd_packet.system_percent = (100 * (1000000 / mainLoopSum)) / osd_packet.mainLoopTime;

    //osd_packet.loopTime defined in init method

    osd_packet.imu_heading = (uint16_t)(attitude.values.yaw / 10.0f);

    uint8_t pid_buffer[PID_ARRAY_ROW*PID_ARRAY_COLUMN];
    pidGetCurrentProfile(pid_buffer);

    osd_packet.pid_roll[0] = pid_buffer[0];
    osd_packet.pid_roll[1] = pid_buffer[1];
    osd_packet.pid_roll[2] = pid_buffer[2];

    osd_packet.pid_pitch[0] = pid_buffer[3];
    osd_packet.pid_pitch[1] = pid_buffer[4];
    osd_packet.pid_pitch[2] = pid_buffer[5];

    osd_packet.pid_yaw[0] = pid_buffer[6];
    osd_packet.pid_yaw[1] = pid_buffer[7];
    osd_packet.pid_yaw[2] = pid_buffer[8];

    osd_packet.pid_dmin[0] = pid_buffer[9];
	osd_packet.pid_dmin[1] = pid_buffer[10];
	osd_packet.pid_dmin[2] = pid_buffer[11];

    osd_packet.is_adjustment_on = CK_ADJUSTEMENT_IsAdjustmentModeOn();

    osd_packet.cpu_core_temperature = (uint8_t)CK_ADC_GetTemperatureResult();

    osd_packet.rssi_link_quality = CK_CRSF_GetLinkQuality();

    osd_packet.tpa_breakpoint = pidProfile.tpa_breakpoint;

    osd_packet.tpa_rate = pidProfile.tpa_rate;

    osd_packet.voltage = (uint16_t)(CK_ADC_GetLipoResult() * 100.0f);

    osd_packet.current = (uint16_t)(CK_ADC_GetCurrentResult() * 100.0f);

}

#if USE_DMA_OSD
void CK_OSD_SendPacketDMA(void){

	#if USE_H7 == 1
	// Clean before tx operation when dcache is enabled
	// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
	SCB_CleanDCache_by_Addr((uint32_t*)sendBuffer, tx_buffer_size + 32);
	#endif

	// TX
	CK_UART_DMA_ClearFlag(OSD_DMA, OSD_DMA_Stream);

	CK_UART_DMA_SetBuffer(OSD_DMA_Stream, sendBuffer, tx_buffer_size);

	CK_UART_DMA_Enable(OSD_DMA_Stream);

	CK_UART_DMA_TXEnable(COMMUNICATION_OSD_UART);

	is_interrupt_started = true;

}
#endif

// Move this to external osd or use msp for powerboard osd as well
void CK_OSD_WriteUartBuffer(void){

    int idx = 0;

    sendBuffer[idx++]   = START_BYTE;

    #if GPS_
    sendBuffer[idx++]   = 1; // OSD will plot gps related data if defined.
    #else
    sendBuffer[idx++]   = 0; // GPS not used by flight controller
    #endif
    sendBuffer[idx++]   = (osd_packet.gps_distanceToDestination >> 8) & 0xFF;
    sendBuffer[idx++]   = osd_packet.gps_distanceToDestination & 0xFF;

    sendBuffer[idx++]   = (osd_packet.gps_headingToDestination >> 8) & 0xFF;
    sendBuffer[idx++]   = osd_packet.gps_headingToDestination & 0xFF;

    sendBuffer[idx++]   = (osd_packet.gps_headingOfMotion >> 8) & 0xFF;
    sendBuffer[idx++]   = osd_packet.gps_headingOfMotion & 0xFF;

    sendBuffer[idx++]   = (osd_packet.gps_groundSpeed >> 8) & 0xFF;
    sendBuffer[idx++]   = osd_packet.gps_groundSpeed & 0xFF;

    sendBuffer[idx++]  = (osd_packet.estimatedAltitude >> 24) & 0xFF;
    sendBuffer[idx++]  = (osd_packet.estimatedAltitude >> 16) & 0xFF;
    sendBuffer[idx++]  = (osd_packet.estimatedAltitude >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.estimatedAltitude & 0xFF;

    sendBuffer[idx++]  = osd_packet.gpsNumOfSat;

    sendBuffer[idx++]  = osd_packet.gpsSatFix;

#if CRSF_
    sendBuffer[idx++]  = (osd_packet.rssi_dBm >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.rssi_dBm & 0xFF;
#endif
#if SBUS_
    sendBuffer[idx++]  = (osd_packet.rssi >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.rssi & 0xFF;
#endif

    sendBuffer[idx++]  = osd_packet.currentFlightMode;

    sendBuffer[idx++]  = osd_packet.currentNavigationMode;

    sendBuffer[idx++]  = osd_packet.currentAltitudeMode;

    sendBuffer[idx++]  = osd_packet.isArmed;

    sendBuffer[idx++]  = osd_packet.isFailSafe;

    sendBuffer[idx++]  = (osd_packet.freqResult >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.freqResult & 0xFF;

    sendBuffer[idx++] = (osd_packet.imu_heading >> 8) & 0xFF;
    sendBuffer[idx++] = osd_packet.imu_heading & 0xFF;

    sendBuffer[idx++]  = osd_packet.pid_roll[0];
    sendBuffer[idx++]  = osd_packet.pid_roll[1];
    sendBuffer[idx++]  = osd_packet.pid_roll[2];

    sendBuffer[idx++]  = osd_packet.pid_pitch[0];
    sendBuffer[idx++]  = osd_packet.pid_pitch[1];
    sendBuffer[idx++]  = osd_packet.pid_pitch[2];

    sendBuffer[idx++]  = osd_packet.pid_yaw[0];
    sendBuffer[idx++]  = osd_packet.pid_yaw[1];
    sendBuffer[idx++]  = osd_packet.pid_yaw[2];

    sendBuffer[idx++]  = osd_packet.pid_dmin[0];
	sendBuffer[idx++]  = osd_packet.pid_dmin[1];
	sendBuffer[idx++]  = osd_packet.pid_dmin[2];

    sendBuffer[idx++]  = osd_packet.is_adjustment_on;

    sendBuffer[idx++]  = osd_packet.cpu_core_temperature;

    sendBuffer[idx++]  = osd_packet.rssi_link_quality;

    sendBuffer[idx++]  = (osd_packet.tpa_breakpoint >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.tpa_breakpoint & 0xFF;
    sendBuffer[idx++]  = osd_packet.tpa_rate;

    sendBuffer[idx++]  = (osd_packet.voltage >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.voltage & 0xFF;

    sendBuffer[idx++]  = (osd_packet.current >> 8) & 0xFF;
    sendBuffer[idx++]  = osd_packet.current & 0xFF;

    sendBuffer[idx]  = END_BYTE;

    tx_buffer_size = idx;

}

void CK_OSD_SendPacketPolling(void){

    for(int i = 0; i < tx_buffer_size; i++){

    	uint8_t data = sendBuffer[i];

        CK_UART_SendPolling(COMMUNICATION_OSD_UART, data);
    }

}

void CK_OSD_SendPacketInterrupt(void){

    CK_UART_ClearFlags(COMMUNICATION_OSD_UART);

    // Send first byte to start. Rest will be sent in interrupt handler.
    uint8_t data = sendBuffer[sendBufferCurrentIndex++];

    CK_UART_SendInterrupt(COMMUNICATION_OSD_UART, data);

    is_interrupt_started = true;

}

void CK_OSD_ResetBuffer(void){

    for(int i = 0; i < SEND_BUFFER_SIZE; i++){
        sendBuffer[i] = 0;
    }

}

#if OSD_PDB_ || OSD_DJI_

#if USE_INTERRUPT_OSD

#if OSD_INTERRUPT_ == 1
void USART1_IRQHandler(void){
#endif

#if OSD_INTERRUPT_ == 2
void USART2_IRQHandler(void){
#endif

#if OSD_INTERRUPT_ == 3
void USART3_IRQHandler(void){
#endif

#if OSD_INTERRUPT_ == 4
void UART4_IRQHandler(void){
#endif

#if OSD_INTERRUPT_ == 5
void UART5_IRQHandler(void){
#endif

#if OSD_INTERRUPT_ == 6
void USART6_IRQHandler(void){
#endif

	#if USE_H7 == 1
	if(COMMUNICATION_OSD_UART->ISR & CK_USART_SR_TC){
		COMMUNICATION_OSD_UART->ICR = 0xFFFFFFFF;
	#endif

	#if USE_F4 == 1
	if(COMMUNICATION_OSD_UART->SR & CK_USART_SR_TC){
	#endif


		// Byte transferred, send the next one until all is done
		if(sendBufferCurrentIndex < tx_buffer_size){

			uint8_t data = sendBuffer[sendBufferCurrentIndex++];
			CK_UART_SendInterrupt(COMMUNICATION_OSD_UART, data);

		}
		// All done
		else{
			is_interrupt_started = false;
			sendBufferCurrentIndex = 0;
			tx_buffer_size = 0;
			CK_UART_TCInterruptDisable(COMMUNICATION_OSD_UART);
		}

	}
}
#endif

#if USE_DMA_OSD

void OSD_DMA_Handler(void){

    if(CK_UART_DMA_IsTransferComplete(OSD_DMA, OSD_DMA_Stream)){ // Transfer of one sector is done.

        CK_UART_DMA_TXDisable(COMMUNICATION_OSD_UART);

        CK_UART_DMA_Disable(OSD_DMA_Stream);

    	CK_UART_DMA_ClearFlag(OSD_DMA, OSD_DMA_Stream);

    	is_interrupt_started = false;
    }
}

#endif

#endif
