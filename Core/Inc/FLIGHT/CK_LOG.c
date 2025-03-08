
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_MICROCARD.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_LED.h"
#include "DRIVERS/CK_GPIO.h"

#include "FLIGHT/CK_LOG.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_MIXER.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_IMU.h"
#include "MOTION/CK_ACC.h"

#include "COMMUNICATION/CK_PRINTER.h"

#define LOG_TIMEOUT_MS 				240

const uint8_t debug_start_byte = 0xC1;
const uint8_t debug_end_byte   = 0xC8;
uint8_t counter = debug_start_byte;

log_parameters_t flightLog;

log_parameters_t flightLog = {
	.buffer_index = 0,

	.log_start_byte = 'C',
	.log_end_byte = 'K',

	.info_sector_write_counter = 0,
	.is_info_write = false
};

DMA_Stream_TypeDef* CK_LOG_DMA_STREAM;

typedef enum{
	LOG_IDLE,
	LOG_DATA,
	LOG_START_TRANSFER,
	LOG_WAIT_DMA_TRANSFER,
	LOG_CHECK_SDCARD_BUSY,
	LOG_WRITE_INFO_DATA,
	LOG_INITIAL_WAIT,
	LOG_SPI_MULTI_STOP_TOKEN_BUSY

}log_states;

log_states log_state;

/*
 * While buffer 1 is being transfer by dma (200 microseconds it takes)
 * a few loop passes and those data needed to be
 * recorded so dual buffering is used.
 *
 *
 * Data packet format start token + 512 byte sector data + 2 byte CRC + 1 resp (MISO = 0xE5 means data accepted)
 */

bool is_transfer_ready_set = false;

uint32_t dma_timeout_t1 = 0;
uint32_t dma_timeout_t2	= 0;

uint32_t logStartTime, logEndTime;

uint32_t log_data_t1, log_data_t2, log_transfer_t1, log_transfer_t2, log_LOG_CHECK_SDCARD_BUSY_t1, log_LOG_CHECK_SDCARD_BUSY_t2, log_LOG_CHECK_SDCARD_BUSY_t3;

void CK_LOG_Init(uint32_t mainT, uint32_t logT){

	flightLog.log_mainTargetTime = mainT;
	flightLog.log_logTargetTime = logT;

	if(flightLog.log_mainTargetTime > flightLog.log_logTargetTime){
		flightLog.log_logTargetTime = flightLog.log_mainTargetTime;
	}
	flightLog.syncRate = flightLog.log_logTargetTime / flightLog.log_mainTargetTime;

	flightLog.log_target_counter = 0;

	flightLog.log_end_led_counter = 0;

	flightLog.log_delay_counter = 0;

    for(int i = 0; i < INFO_BUFFER_SIZE; i++){
        flightLog.info_buffer[i] = ' ';
    }

	log_state = LOG_IDLE;

	#if LOG_SPI_
	CK_LOG_DMA_STREAM = MICROCARD_DMA_STREAM;
	#endif

}

// IMPORTANT NOTE FOR SDIO timeout
// SDIO waits in polling with a while loop to get response from the card and the timeout is in milliseconds.
// My code keeps checking the card state therefore i changed it to microsecond timeout stm32h7xx_II_sdmmc.c 1266
// SDMMC_CMDTIMEOUT is 110 microsecond and SDMMC_STOPTRANSFERTIMEOUT is 1 sec in stm32h7xx_II_sdmmc.h line 311
// (110 is not a random number. It is decided by checking both cards initialization. Slower card worked with at least 110 microsec)
// If this is not done then the code stucks for milliseconds which is problematic for flight.
// Also the else statements in this file with LOG_TIMEOUT_MS are for ignoring previous sector log and restarting the logging.
// The timeout change increased the throughput as well

// Packet format is Start Token + 512 byte data + 2 CRC and also i add 1 resp byte for DMA to get resp directly
// Check resp data on MISO line. 0xE5 means data is accepted.
// After each packet transfer MISO 0x00 busy must be waited which is checked by CK_MICROCARD_CheckIsCardBusy()
// In multi mode, it is not correct to send 8 packet once with dma since there will be busy time between each frame
// In multi mode the tx does not stop until more than one sector is written but each packet is sent individually and busy time is waited.
// Therefore the difference between single and multi is busy time is shorter in multi mode.
// Single has 3 to 6 ms busy gap, multi is around 2ms.

void CK_LOG_Update(uint32_t currentLoopTime){

	if(!CK_MICROCARD_IsLoggingDone() && card.is_Initialized){

		switch(log_state){

			case LOG_IDLE:

				if(card.is_Initialized && card.is_card_fast && card.is_card_high_capacity){

					logStartTime = CK_TIME_GetMilliSec();
					log_state = LOG_DATA;
				}
				else{

					flightLog.log_end_led_counter++;

					if(flightLog.log_end_led_counter == 1000){
						flightLog.log_end_led_counter = 0;
						CK_LED_ToggleLed(2);
					}
					log_state = LOG_INITIAL_WAIT;
				}

				break;

			case LOG_INITIAL_WAIT:

				flightLog.log_delay_counter++;

				if(flightLog.log_delay_counter == 8000){
					flightLog.log_delay_counter = 0;
					log_state = LOG_DATA;
				}

				break;

			case LOG_START_TRANSFER:

				log_transfer_t1 = CK_TIME_GetMicroSec();

				#if LOG_SPI_

				#if USE_H7 == 1
				// Clean before tx operation when dcache is enabled
				// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
				SCB_CleanDCache_by_Addr((uint32_t*)flightLog.log_buffer_1, flightLog.buffer_index + 32);
				#endif

				CK_SPI_DMA_SetBuffer(CK_LOG_DMA_STREAM, flightLog.log_buffer_1, flightLog.buffer_index);
				#endif

				flightLog.buffer_index = 0;

				flightLog.info_sector_write_counter++;

				card.is_dma_ready = false;

				dma_timeout_t1 = CK_TIME_GetMilliSec();

            	CK_MICROCARD_WriteData(card.START_SECTOR + card.CURRENT_SECTOR);

				log_state = LOG_WAIT_DMA_TRANSFER;

				log_transfer_t2 = CK_TIME_GetMicroSec() - log_transfer_t1;

				break;

			case LOG_WRITE_INFO_DATA:

				flightLog.info_sector_write_counter = 0;

				card.is_infoSector_write = true; // Not written yet

				CK_LOG_WriteInfoBuffer();

				CK_MICROCARD_WriteInfoSector();

				log_state = LOG_WAIT_DMA_TRANSFER;


				break;

			// After the end of multi write, card will be busy
			// If info sector is written directly then it wont write since card is busy
			case LOG_SPI_MULTI_STOP_TOKEN_BUSY:

				#if SCOPE_CHECK_LOG == 1
				CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
				#endif

				// Card takes DO line low during busy flag which means MISO data is 0x00
				// So reading 0xFF means not busy
				if(CK_MICROCARD_CheckIsCardBusy() == HAL_OK){
					log_state = LOG_WRITE_INFO_DATA;

					#if SCOPE_CHECK_LOG == 1
					CK_GPIO_ClearPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
					#endif
				}

				break;

			case LOG_WAIT_DMA_TRANSFER:

				if(CK_MICROCARD_IsDMAReady()){

					log_state = LOG_CHECK_SDCARD_BUSY;

					dma_timeout_t1 = CK_TIME_GetMilliSec();

				}
				else{
					// Sometimes even if tx started the interrupt is not called
					// so after waiting enough time i restart tx and it is fine
					dma_timeout_t2 = CK_TIME_GetMilliSec() - dma_timeout_t1;
					if(dma_timeout_t2 > LOG_TIMEOUT_MS){

						log_state = LOG_DATA;

						card.is_dma_ready = true;

						// Relog last 16 sector
						CK_MICROCARD_DecrementCurrentSector(BLOCK_CACHE_SIZE);

						//CK_PRINTER_PrintlnString("DMA Timeout");
					}
				}
				break;

			case LOG_CHECK_SDCARD_BUSY:

				log_LOG_CHECK_SDCARD_BUSY_t1 = CK_TIME_GetMicroSec();
				// Card takes DO line low during busy flag which means MISO data is 0x00
				// So reading 0xFF means not busy
				if(CK_MICROCARD_CheckIsCardBusy() == HAL_OK){
				//if(1){

					log_LOG_CHECK_SDCARD_BUSY_t2 = CK_TIME_GetMicroSec() - log_LOG_CHECK_SDCARD_BUSY_t1;

					if(flightLog.info_sector_write_counter == WRITE_INFO_SECTOR){
						if(card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){

							// Stop multi before info write.
							CK_MICROCARD_SendStopToken();

							log_state = LOG_SPI_MULTI_STOP_TOKEN_BUSY;
						}
						else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK){

							log_state = LOG_WRITE_INFO_DATA;
						}
						else if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

							log_state = LOG_WRITE_INFO_DATA;
						}

					}
					else{
						log_state = LOG_DATA;
					}

				}
				else{
					// Sometimes even if tx started the interrupt is not called
					// so after waiting enough time i restart tx and it is fine
					dma_timeout_t2 = CK_TIME_GetMilliSec() - dma_timeout_t1;
					if(dma_timeout_t2 > LOG_TIMEOUT_MS){

						log_state = LOG_DATA;

						card.is_dma_ready = true;

						// Relog last 16 sector
						CK_MICROCARD_DecrementCurrentSector(BLOCK_CACHE_SIZE);

						//CK_PRINTER_PrintlnString("Card Busy Timeout");
					}
				}

				log_LOG_CHECK_SDCARD_BUSY_t3 = CK_TIME_GetMicroSec() - log_LOG_CHECK_SDCARD_BUSY_t1;
				break;

			case LOG_DATA:

				log_data_t1 = CK_TIME_GetMicroSec();

				flightLog.log_target_counter++;

				if(flightLog.log_target_counter == flightLog.syncRate){

					flightLog.log_target_counter = 0;

					#if TEST_LOG == 1

						if(card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){
							if(flightLog.buffer_index == 0){
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFC;
							}
						}
						else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK){

							/*
							if(flightLog.buffer_index == 0){

								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFF;

								flightLog.log_buffer_1[flightLog.buffer_index++]  = CMD24 | 0x40;
								flightLog.log_buffer_1[flightLog.buffer_index++]  = (card.START_SECTOR + card.CURRENT_SECTOR) >> 24;
								flightLog.log_buffer_1[flightLog.buffer_index++]  = (card.START_SECTOR + card.CURRENT_SECTOR) >> 16;
								flightLog.log_buffer_1[flightLog.buffer_index++]  = (card.START_SECTOR + card.CURRENT_SECTOR) >> 8;
								flightLog.log_buffer_1[flightLog.buffer_index++]  = (card.START_SECTOR + card.CURRENT_SECTOR);

								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0x00;

								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFF;
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFF;

							}

							if(flightLog.buffer_index == 9){
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFE;
							}
							*/
							if(flightLog.buffer_index == 0){
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFE;
							}
						}
						else if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

						}

						for(int i = 0; i < BYTES_PER_LOG; i++){

							flightLog.log_buffer_1[flightLog.buffer_index++] = counter;
						}

						if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

							if(flightLog.buffer_index == LOG_BUFFER_SIZE){

								counter++;
								if(counter == debug_end_byte){
									counter = debug_start_byte;
								}

							}

							if(flightLog.buffer_index == LOG_BUFFER_SIZE){

								log_state = LOG_START_TRANSFER;

							}
						}

						else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK || card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){

							if(flightLog.buffer_index == LOG_BUFFER_SIZE - 3){

								counter++;
								if(counter == debug_end_byte){
									counter = debug_start_byte;
								}

								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // Dummy crc
								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // Dummy crc
								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // response

							}

							if(flightLog.buffer_index == LOG_BUFFER_SIZE){

								log_state = LOG_START_TRANSFER;
							}
						}

					#else

						// LOG Bytes here at each loop
						if(card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){
							if(flightLog.buffer_index == 0){
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFC;
							}
						}
						else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK){

							if(flightLog.buffer_index == 0){
								flightLog.log_buffer_1[flightLog.buffer_index++]  = 0xFE;
							}

						}
						else if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

						}

						// 1 Byte start indicator.
						flightLog.log_buffer_1[flightLog.buffer_index++] = flightLog.log_start_byte;

						// All axis of gyro for raw and filtered results (30 bytes)
						for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){

							int16_t gyroRaw 	 = gyro.gyroADCRaw[axis];
							int16_t gyroFiltered = (int16_t)gyro.gyroADCf[axis];
							int16_t gyroPreLPF 	 = (int16_t)gyro.gyroADCPreLPF[axis];
							int16_t gyroPreNotch = (int16_t)gyro.gyroADCPreNotch[axis];
							int16_t gyroZero 	 = gyro.gyroADCZero[axis];

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroRaw >> 8);   	   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroRaw & 0xFF); 	   // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroFiltered >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroFiltered & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroPreLPF >> 8);     // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroPreLPF & 0xFF);   // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroPreNotch >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroPreNotch & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroZero >> 8);   	   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroZero & 0xFF); 	   // LowByte

						}

						// Altitude hold adjustment / 10 is sent (1 Byte)
						uint8_t altHold_throttle = ((uint16_t)CK_ALTITUDE_GetThrottleAdjustment_AltitudeHold()) / 10;
						flightLog.log_buffer_1[flightLog.buffer_index++] = altHold_throttle;

						// RCData and RCSetpoint is sent (4 Byte each for 3 axis and 2 byte for throttle rcData, 14 Bytes total)
						for(int axis = 0; axis <= THROTTLE; axis++){

							uint16_t rcData = (uint16_t)getRCDataRaw(axis);

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(rcData >> 8);   	   	// HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(rcData & 0xFF); 	   	// LowByte

							// Setpoint only for roll, pitch and yaw
							if(axis != THROTTLE){

								int16_t rcSetpoint = (int16_t)getSetpointRate(axis);

								flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(rcSetpoint >> 8);		// HighByte
								flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(rcSetpoint & 0xFF);	// LowByte
							}

						}

						// Motor final result / 10 is sent (1 Byte each, 4 Bytes total)
						uint8_t motor_result = 0;
						for(int i = 1; i <= 4; i++){

							motor_result = CK_MIXER_GetMotorFinalResult(i) / 10;

							flightLog.log_buffer_1[flightLog.buffer_index++] = motor_result;
						}

						// Imu results are sent same except yaw divided to 2 (1 Byte each, 3 Byte total)
						uint8_t imuAngle = 0;
						for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){

							if(axis != FD_YAW){

								imuAngle = attitude.raw[axis];

								flightLog.log_buffer_1[flightLog.buffer_index++] = imuAngle;
							}
							else{

								imuAngle = attitude.raw[axis] / 2;

								flightLog.log_buffer_1[flightLog.buffer_index++] = imuAngle;
							}
						}

						// Last loop cycle time (2 Bytes)
						uint16_t loopTime = (uint16_t)currentLoopTime;
						flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(loopTime >> 8);   // HighByte
						flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(loopTime & 0xFF); // LowByte

						// Flight flags (2 Bytes)
						uint16_t flags_encoded = 0; //flags.flags_encoded;
						flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(flags_encoded >> 8); 	// HighByte
						flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(flags_encoded & 0xFF); // LowByte

						// All axis of acc for filtered results (6 bytes)
						for(int axis = 0; axis < XYZ_AXIS_COUNT; axis++){

							int16_t accFiltered = acc.accADC.v[axis];

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(accFiltered >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(accFiltered & 0xFF); // LowByte

						}

						// 30 bytes total
						for(int i = 0; i < XYZ_AXIS_COUNT; i++){

							int16_t pid_p_result = pidData[i].P * 10.0f;
							int16_t pid_i_result = pidData[i].I * 10.0f;
							int16_t pid_d_result = pidData[i].D * 10.0f;
							int16_t pid_f_result = pidData[i].F * 10.0f;
							int16_t pid_result = pidData[i].Sum * 10.0f;

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_p_result >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_p_result & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_i_result >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_i_result & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_d_result >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_d_result & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_f_result >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_f_result & 0xFF); // LowByte

							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_result >> 8);   // HighByte
							flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(pid_result & 0xFF); // LowByte

						}

						// 34 bytes are available
						for(int i = 0; i < 34; i++){

							#if defined(LOG_DEBUG_PIDLEVEL_PARAMETERS)

							#endif

							flightLog.log_buffer_1[flightLog.buffer_index++] = 0;

						}

						// 1 Byte end indicator.
						flightLog.log_buffer_1[flightLog.buffer_index++] = flightLog.log_end_byte;


						if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

							if(flightLog.buffer_index == LOG_BUFFER_SIZE){

								log_state = LOG_START_TRANSFER;

							}
						}

						else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK || card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){

							if(flightLog.buffer_index == LOG_BUFFER_SIZE - 3){

								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // Dummy crc
								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // Dummy crc
								flightLog.log_buffer_1[flightLog.buffer_index++] = 0xFF; // response

							}

							if(flightLog.buffer_index == LOG_BUFFER_SIZE){

								log_state = LOG_START_TRANSFER;
							}
						}



					#endif
				}

				log_data_t2 = CK_TIME_GetMicroSec() - log_data_t1;

				break;

			default:
				break;
		}
	}
	else{
		flightLog.log_end_led_counter++;

		if(flightLog.log_end_led_counter == 400){
			flightLog.log_end_led_counter = 0;
			CK_LED_ToggleLed(2);
		}

	}

}

void CK_LOG_WriteInfoBuffer(void){

	static bool isStaticVariableWritten = false;

	// LOG_SPI uses this do not delete
	int idx = 0;

	#if LOG_SPI_
	// Info sector is always single block write
    card.transfer_mode = SPI_DMA_INTERRUPT_SINGLEBLOCK;

    /*
    flightLog.info_buffer[idx++]  = 0xFF;

	flightLog.info_buffer[idx++]  = CMD24 | 0x40;
	flightLog.info_buffer[idx++]  = card.INFO_SECTOR >> 24;
	flightLog.info_buffer[idx++]  = card.INFO_SECTOR >> 16;
	flightLog.info_buffer[idx++]  = card.INFO_SECTOR >> 8;
	flightLog.info_buffer[idx++]  = card.INFO_SECTOR;

	flightLog.info_buffer[idx++]  = 0x00;

	flightLog.info_buffer[idx++]  = 0xFF;
	flightLog.info_buffer[idx++]  = 0xFF;
	*/

	flightLog.info_buffer[idx++]  = 0xFE; // Start token
	#endif

	// For ease of following specific byte after adding new data i staticly indexed each one
	// This part is written only once
	if(!isStaticVariableWritten){

		// This is a text to check by python to know it is written correctly.
	    flightLog.info_buffer[0 + idx]  = 'C';
	    flightLog.info_buffer[1 + idx]  = 'K';
	    flightLog.info_buffer[2 + idx]  = 'F';
	    flightLog.info_buffer[3 + idx]  = 'L';
	    flightLog.info_buffer[4 + idx]  = 'I';
	    flightLog.info_buffer[5 + idx]  = 'G';
	    flightLog.info_buffer[6 + idx]  = 'H';
	    flightLog.info_buffer[7 + idx]  = 'T';

	    flightLog.info_buffer[8 + idx]  = CURRENT_VERSION_MAJOR;
	    flightLog.info_buffer[9 + idx]  = CURRENT_VERSION_MINOR;

	    flightLog.info_buffer[10+ idx]  = TARGET_MCU[0];
	    flightLog.info_buffer[11+ idx]  = TARGET_MCU[1];

        // Flight Controller loop freq
        // MSB first
        flightLog.info_buffer[12+ idx] = (uint8_t)(flightLog.log_mainTargetTime >> 8 & 0xFF);
        flightLog.info_buffer[13+ idx] = (uint8_t)(flightLog.log_mainTargetTime & 0xFF);

        // Gyro sign
        if(gyro.gyroSign[0] == -1)flightLog.info_buffer[14+ idx] = (uint8_t)(1 & 0xFF);
        else flightLog.info_buffer[14+ idx] = (uint8_t)(2 & 0xFF);

        if(gyro.gyroSign[1] == -1)flightLog.info_buffer[15+ idx] = (uint8_t)(1 & 0xFF);
        else flightLog.info_buffer[15+ idx] = (uint8_t)(2 & 0xFF);

        if(gyro.gyroSign[2] == -1)flightLog.info_buffer[16+ idx] = (uint8_t)(1 & 0xFF);
        else flightLog.info_buffer[16+ idx] = (uint8_t)(2 & 0xFF);

        // 1/16.4 * 10000
        uint16_t gyro_scale = gyro.gyroScale * 1e5;
        flightLog.info_buffer[17+ idx] = (uint8_t)(gyro_scale >> 8 & 0xFF);
        flightLog.info_buffer[18+ idx] = (uint8_t)(gyro_scale & 0xFF);

        flightLog.info_buffer[19+ idx] = (uint8_t)(BYTES_PER_LOG & 0xFF);

        // Filter settings
        flightLog.info_buffer[20+ idx] = (uint8_t)(gyro.use_lpf_filter & 0xFF);
        flightLog.info_buffer[21+ idx] = (uint8_t)(gyro.gyro_lpf1_static_hz & 0xFF);
    	#ifdef USE_DYN_LPF
        flightLog.info_buffer[21+ idx] = (uint8_t)(gyro.gyro_lpf1_dyn_min_hz & 0xFF);
    	#endif

        flightLog.info_buffer[22+ idx] = (uint8_t)(gyro.use_notch1_filter & 0xFF);
        flightLog.info_buffer[23+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_1 >> 8 & 0xFF);
        flightLog.info_buffer[24+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_1 & 0xFF);

		flightLog.info_buffer[25+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_1 >> 8 & 0xFF);
		flightLog.info_buffer[26+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_1  & 0xFF);


        flightLog.info_buffer[27+ idx] = (uint8_t)(gyro.use_notch2_filter & 0xFF);
        flightLog.info_buffer[28+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_2 >> 8 & 0xFF);
        flightLog.info_buffer[29+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_2 & 0xFF);

		flightLog.info_buffer[30+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_2 >> 8 & 0xFF);
		flightLog.info_buffer[31+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_2 & 0xFF);


        flightLog.info_buffer[32+ idx] = (uint8_t)(gyro.use_notch3_filter & 0xFF);
        flightLog.info_buffer[33+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_3 >> 8 & 0xFF);
        flightLog.info_buffer[34+ idx] = (uint8_t)(gyro.gyro_soft_notch_hz_3 & 0xFF);

		flightLog.info_buffer[35+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_3 >> 8 & 0xFF);
		flightLog.info_buffer[36+ idx] = (uint8_t)(gyro.gyro_soft_notch_cutoff_3 & 0xFF);

    	flightLog.info_buffer[37+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_ROLL][PID_P]);
    	flightLog.info_buffer[38+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_ROLL][PID_I]);
    	flightLog.info_buffer[39+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_ROLL][PID_D]);
    	flightLog.info_buffer[40+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_ROLL][PID_FF]);
    	flightLog.info_buffer[41+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_ROLL][PID_Dmax]);

    	flightLog.info_buffer[42+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_PITCH][PID_P]);
    	flightLog.info_buffer[43+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_PITCH][PID_I]);
    	flightLog.info_buffer[44+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_PITCH][PID_D]);
    	flightLog.info_buffer[45+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_PITCH][PID_FF]);
    	flightLog.info_buffer[46+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_PITCH][PID_Dmax]);

    	flightLog.info_buffer[47+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_YAW][PID_P]);
    	flightLog.info_buffer[48+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_YAW][PID_I]);
    	flightLog.info_buffer[49+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_YAW][PID_D]);
    	flightLog.info_buffer[50+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_YAW][PID_FF]);
    	flightLog.info_buffer[51+idx]  = (uint8_t)(PID_SELECTED_PROFILE[FD_YAW][PID_Dmax]);

    	/*
    	flightLog.info_buffer[52+idx]  = (uint8_t)(CK_PID_GetMaster_Multiplier() 	* 10.0f);
    	flightLog.info_buffer[53+idx]  = (uint8_t)(CK_PID_GetITerm_Multiplier() 	* 10.0f);
    	flightLog.info_buffer[54+idx]  = (uint8_t)(CK_PID_GetDTerm_Multiplier() 	* 10.0f);
    	flightLog.info_buffer[55+idx]  = (uint8_t)(CK_PID_GetFF_Multiplier() 		* 10.0f);
    	flightLog.info_buffer[56+idx]  = (uint8_t)(CK_PID_GetRoll_Multiplier() 		* 10.0f);
    	flightLog.info_buffer[57+idx]  = (uint8_t)(CK_PID_GetPitch_Multiplier() 	* 10.0f);
    	flightLog.info_buffer[58+idx]  = (uint8_t)(CK_PID_GetYaw_Multiplier() 		* 10.0f);
		*/
    	flightLog.info_buffer[52+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[53+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[54+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[55+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[56+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[57+idx]  = (uint8_t)(1.0f 	* 10.0f);
		flightLog.info_buffer[58+idx]  = (uint8_t)(1.0f 	* 10.0f);

    	// 1 is true 0 is false flags to know which parameters are debugged and logged for analyzer to know
    	//
		#if defined(LOG_DEBUG_PIDLEVEL_PARAMETERS)
    	flightLog.info_buffer[59+idx]  = 1;
    	#endif



        flightLog.info_buffer[194+ idx] = 'L';
        flightLog.info_buffer[195+ idx] = 'O';
        flightLog.info_buffer[196+ idx] = 'G';
        flightLog.info_buffer[197+ idx] = '_';
        flightLog.info_buffer[198+ idx] = 'O';
        flightLog.info_buffer[199+ idx] = 'K';

        isStaticVariableWritten = true;

    }

	// This part is updated not static
    // Number of sectors written in total
    // MSB first

    uint32_t total_sector = card.CURRENT_SECTOR;

    flightLog.info_buffer[200+idx]  = (uint8_t)(total_sector >> 24 & 0xFF);
    flightLog.info_buffer[201+idx]  = (uint8_t)(total_sector >> 16 & 0xFF);
    flightLog.info_buffer[202+idx] = (uint8_t)(total_sector >> 8 & 0xFF);
    flightLog.info_buffer[203+idx] = (uint8_t)(total_sector & 0xFF);

    uint32_t log_endTime = CK_TIME_GetMilliSec() - logStartTime;

	flightLog.info_buffer[204+idx]  = (uint8_t)(log_endTime >> 24 & 0xFF);
	flightLog.info_buffer[205+idx]  = (uint8_t)(log_endTime >> 16 & 0xFF);
	flightLog.info_buffer[206+idx] = (uint8_t)(log_endTime >> 8 & 0xFF);
	flightLog.info_buffer[207+idx] = (uint8_t)(log_endTime & 0xFF);

	uint32_t invalid_data = CK_RECEIVER_GetInvalidDataCounter();
	flightLog.info_buffer[208+idx]  = (uint8_t)(invalid_data >> 24 & 0xFF);
	flightLog.info_buffer[209+idx]  = (uint8_t)(invalid_data >> 16 & 0xFF);
	flightLog.info_buffer[210+idx]  = (uint8_t)(invalid_data >> 8 & 0xFF);
	flightLog.info_buffer[211+idx]  = (uint8_t)(invalid_data & 0xFF);

	#if LOG_SPI_
    flightLog.info_buffer[INFO_BUFFER_SIZE - 3]  = 0xFF; // Dummy crc
    flightLog.info_buffer[INFO_BUFFER_SIZE - 2]  = 0xFF; // Dummy crc
    flightLog.info_buffer[INFO_BUFFER_SIZE - 1]  = 0xFF; // response
	#endif

}












