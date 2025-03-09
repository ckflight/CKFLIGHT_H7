
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"

#include "COMMUNICATION/CK_MSP.h"
#include "COMMUNICATION/CK_PRINTER.h"

#include "OSD/CK_OSD.h"
#include "OSD/CK_MSP_OSD.h"

#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_TIME_HAL.h"

#define BUFFER_SIZE				512

uint8_t commandBuffer[128];
uint16_t commandBufferIndex = 0;
uint16_t commandByteNumber = 0;
int isCommand = 0;
int isRequest = 0;

msp_parameters_t msp;

/*
 *  Reques Message
 *
 *  $M<[data_length][code][data][crc]
 *
 *  Output Message
 *
 *  $M>[data_length][code][data][crc]
 *
 */

/*
 * The msp protocol can communicate through uart and usb
 * CK_INPUTSTREAM_Update sends received usb data to each class that uses usb
 * CK_MSP_DecodeInputStream will decode the received bytes and if it is msp related
 * then it will start msp usb communication.
 *
 * If osd uart is enabled then msp will send selected info over uart.
 * CK_MSP_ProcessCommand will write payload and crc to msp buffer
 * it will return the payload size
 * $M>[data_length][msp command][data][crc]
 */

void CK_MSP_DecodeInputStream(uint8_t* buffer, uint16_t size){

	// MSP protocol sends more than 3 bytes
	if(size > 3){

		// Once the main configurations are done.
		// Then usb-bluetooth device keeps sending
		// request to update parameters

		uint8_t id = CK_MSP_GetMessage(buffer, size, commandBuffer, &commandByteNumber);

		// Received Request, send response
		if(isRequest && !isCommand){
			CK_MSP_SendResponse(id);
			id = 0;
		}
		// Received Command
		else if(isCommand && !isRequest){

			CK_MSP_ProcessCommand(id, commandBuffer, commandByteNumber);
			id = 0;

		}

		commandBufferIndex = 0;

	}
}

uint8_t CK_MSP_GetMessage(uint8_t* buffer, uint16_t length, uint8_t* commandPayloadBuffer, uint16_t* len){

	uint8_t command_id = 0;
	uint8_t request_id = 0;
	uint8_t size = 0;
	uint8_t crc_check = 0;

	int state = 0;

	for(int i = 0; i < length; i++){

		uint8_t current_data = buffer[i];

		switch(state){
			case 0:

				isCommand = 0;
				isRequest = 0;

				if(current_data == '$'){
					state++;
					break;
				}
				state = 0;
				break;

			case 1:
				if(current_data == 'M'){
					state++;
					break;
				}
				state = 0;
				break;

			case 2:
				if(current_data == '<'){
					state++;
					break;
				}
				state = 0;
				break;


			case 3:
				size = current_data; 	// size
				if(size == 0){
					state = 7;			// it is a request go to request case
					break;
				}

				crc_check ^= current_data;
				*len = size;	// if say 3 then first 3 element of 128 byte buffer is the command payload.
				state++; 		// it is a command go to command case
				break;

			// COMMAND
			case 4:
				isCommand = 1;
				isRequest = 0;

				command_id = current_data; // type
				crc_check ^= current_data;
				state++;
				break;

			case 5:
				if(size > 1){
					*(commandPayloadBuffer + (commandBufferIndex++)) = current_data;
					crc_check ^= current_data;
					size--;
					break;
				}
				else if(size == 1){
					*(commandPayloadBuffer + (commandBufferIndex++)) = current_data;
					crc_check ^= current_data;
					size--;
					state++;
					break;
				}
				state++;
				break;

			case 6:
				if(crc_check == current_data){
					state = 0;
					break;
				}
				command_id = 0; // to indicate not correctly received.
				state = 0;
				break;

			// REQUEST
			case 7:
				isCommand = 0;
				isRequest = 1;
				request_id = current_data; // type
				state++;
				break;

			case 8:
				if(request_id == current_data){ // crc
					request_id = current_data;
					state = 0;
					break;
				}
				request_id = 0; // to indicate not correctly received.
				state = 0;
				break;

			default:
				break;

		}

	}

	if(isCommand && !isRequest){
		return command_id;
	}
	return request_id;

}

uint16_t CK_MSP_FormPacket(uint8_t command_id, uint8_t* buffer){

	uint16_t payloadSize = 0;
	uint8_t crc = 0;

	buffer[payloadSize++] = '$';
	buffer[payloadSize++] = 'M';
	buffer[payloadSize++] = '>';

	switch(command_id){

	case MSP_OSD_CONFIG:

		/*
		CK_MSP_WriteBufferU8(buffer, 217, payloadSize++, &crc); //  byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_OSD_CONFIG, payloadSize++, &crc);

		CK_MSP_WriteBufferU8(buffer, dji_osd.osdflags, payloadSize++, &crc);

		// HD osd system
		CK_MSP_WriteBufferU8(buffer, dji_osd.video_system, payloadSize++, &crc);

		CK_MSP_WriteBufferU8(buffer, dji_osd.units, payloadSize++, &crc);

		CK_MSP_WriteBufferU8(buffer, dji_osd.rssi_alarm, payloadSize++, &crc);

		CK_MSP_WriteBufferU16(buffer, dji_osd.cap_alarm, payloadSize, &crc);
		payloadSize += 2;

		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, dji_osd.osd_item_count, payloadSize++, &crc);

		CK_MSP_WriteBufferU16(buffer, dji_osd.alt_alarm, payloadSize, &crc);
		payloadSize += 2;

		// Element position and visibility
		for (int i = 0; i < dji_osd.osd_item_count; i++) {
			CK_MSP_WriteBufferU16(buffer, dji_osd.osd_pos_buffer[i], payloadSize, &crc);
			payloadSize += 2;
		}

		// Post flight statistics
		CK_MSP_WriteBufferU8(buffer, dji_osd.osd_stat_count, payloadSize++, &crc);
		for (int i = 0; i < dji_osd.osd_stat_count; i++) {
			CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		}

		// Timers
		CK_MSP_WriteBufferU8(buffer, dji_osd.osd_timer_count, payloadSize++, &crc);
		for (int i = 0; i < dji_osd.osd_timer_count; i++) {
			CK_MSP_WriteBufferU16(buffer, 0, payloadSize, &crc);
			payloadSize += 2;
		}

		// Enabled warnings
		// Send low word first for backwards compatibility (API < 1.41)
		CK_MSP_WriteBufferU16(buffer, dji_osd.enabledwarnings, payloadSize, &crc);
		payloadSize += 2;

		CK_MSP_WriteBufferU8(buffer, dji_osd.osd_warning_count, payloadSize++, &crc);

		CK_MSP_WriteBufferU32(buffer, dji_osd.enabledwarnings_1_41_plus, payloadSize, &crc);
		payloadSize += 4;

		CK_MSP_WriteBufferU8(buffer, dji_osd.osd_profile_count, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, dji_osd.osdprofileindex, payloadSize++, &crc);

		CK_MSP_WriteBufferU8(buffer, dji_osd.overlay_radio_mode, payloadSize++, &crc);

        // API >= 1.43
        // Add the camera frame element width/height
		CK_MSP_WriteBufferU8(buffer, dji_osd.camera_frame_width, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, dji_osd.camera_frame_height, payloadSize++, &crc);

		buffer[payloadSize++] = crc;
		*/
		break;

	case MSP_FC_VERSION:

		CK_MSP_WriteBufferU8(buffer, 3, payloadSize++, &crc); // 3 byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_FC_VERSION, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 4, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 4, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 1, payloadSize++, &crc);
		buffer[payloadSize++] = crc;
		break;

	case MSP_FC_VARIANT:

		CK_MSP_WriteBufferU8(buffer, 4, payloadSize++, &crc); // 4 byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_FC_VARIANT, payloadSize++, &crc);

		uint8_t fc_id[4] = {'B','T','F','L'};
		for(int i = 0; i < 4; i++){
			CK_MSP_WriteBufferU8(buffer, fc_id[i], payloadSize++, &crc);
		}

		buffer[payloadSize++] = crc;

		break;

	case MSP_ANALOG:

		CK_MSP_WriteBufferU8(buffer, 9, payloadSize++, &crc); // 9 byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_ANALOG, payloadSize++, &crc);

		float volt_ = (msp_osd_packet.voltage / 100.0f) * VOLT_CALIBRATION_MULTIPLIER;
		volt_ *= 10.0f; // 0.1v step
		CK_MSP_WriteBufferU8(buffer, (uint8_t)volt_, payloadSize++, &crc);

		uint16_t battery_capacity_mah_ = 0;
		CK_MSP_WriteBufferU16(buffer, battery_capacity_mah_, payloadSize, &crc);
		payloadSize += 2;

		uint16_t rssi = msp_osd_packet.rssi_dBm;
		CK_MSP_WriteBufferU16(buffer, rssi, payloadSize, &crc);
		payloadSize += 2;

		CK_MSP_WriteBufferU16(buffer, 0, payloadSize, &crc);
		payloadSize += 2;

		volt_ = (msp_osd_packet.voltage / 100.0f) * VOLT_CALIBRATION_MULTIPLIER;
		volt_ *= 100.0f; // 0.01v step
		CK_MSP_WriteBufferU16(buffer, (uint16_t)volt_, payloadSize, &crc);
		payloadSize += 2;

		buffer[payloadSize++] = crc;

		break;

	case MSP_BATTERY_STATE:

		CK_MSP_WriteBufferU8(buffer, 11, payloadSize++, &crc); // 11 byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_BATTERY_STATE, payloadSize++, &crc);

		uint8_t battery_cell_count = 6;
		CK_MSP_WriteBufferU8(buffer, battery_cell_count, payloadSize++, &crc);

		uint16_t battery_capacity_mah = 0;
		CK_MSP_WriteBufferU16(buffer, battery_capacity_mah, payloadSize, &crc);
		payloadSize += 2;

		float volt = (msp_osd_packet.voltage / 100.0f) * VOLT_CALIBRATION_MULTIPLIER;
		volt *= 10.0f; // 0.1v step
		CK_MSP_WriteBufferU8(buffer, (uint8_t)volt, payloadSize++, &crc);

		uint16_t mah = 0;
		CK_MSP_WriteBufferU16(buffer, mah, payloadSize, &crc);
		payloadSize += 2;

		float current = ((msp_osd_packet.current / 100.0f) * 1000.0f) / (CURRENT_RESISTOR * 0.5f); // Current Sens Vout = I*0.5m*105K/1K = I*52.5m
		current *= MAH_CALIBRATION_MULTIPLIER;
		current *= 100.0f; // 0.01A steps
		CK_MSP_WriteBufferU16(buffer, (int16_t)current, payloadSize, &crc);
		payloadSize += 2;

		uint8_t battery_alert = 0;
		CK_MSP_WriteBufferU8(buffer, battery_alert, payloadSize++, &crc);

		volt = (msp_osd_packet.voltage / 100.0f) * VOLT_CALIBRATION_MULTIPLIER;
		volt *= 100.0f; // 0.01v step
		CK_MSP_WriteBufferU16(buffer, (uint16_t)volt, payloadSize, &crc);
		payloadSize += 2;

		buffer[payloadSize++] = crc;

		break;

	case MSP_STATUS:
	case MSP_STATUS_EX:

		CK_MSP_WriteBufferU8(buffer, 22, payloadSize++, &crc); // 22 byte payload

		if(command_id == MSP_STATUS){
			CK_MSP_WriteBufferU8(buffer, MSP_STATUS, payloadSize++, &crc);
		}
		else{
			CK_MSP_WriteBufferU8(buffer, MSP_STATUS_EX, payloadSize++, &crc);
		}

		// 1 uint16_t cycleTime (microsec)
		CK_MSP_WriteBufferU16(buffer, osd_packet.mainLoopTime, payloadSize, &crc);
		payloadSize +=2;

		// 1 uint16_t i2c_errors_count
		uint16_t i2c_error = 0; // later i2c error counter
		CK_MSP_WriteBufferU16(buffer, i2c_error, payloadSize, &crc);
		payloadSize +=2;

		// 1 uint16_t sensors
		uint16_t sensors_used = 0;
		sensors_used |= ACC1_SPI_ 	<< 0; // acc
		sensors_used |= USE_BARO_ 	<< 1; // baro
		sensors_used |= USE_MAG_ 	<< 2; // mag
		sensors_used |= GPS_ 		<< 3; // gps
		sensors_used |= 0 			<< 4; // sonar
		sensors_used |= GYRO1_SPI_ 	<< 5; // gyro
		CK_MSP_WriteBufferU16(buffer, sensors_used, payloadSize, &crc);
		payloadSize +=2;


		// 4 uint8_t flightmodeflag
		uint32_t flightmodeflag = 0;
		if(msp_osd_packet.isArmed){
			flightmodeflag = 0x00000003;    // arm to start recording
		}
		else{
			flightmodeflag = 0x00000002;    // disarmed with 3 sec delay
		}

		CK_MSP_WriteBufferU32(buffer, flightmodeflag, payloadSize, &crc);
		payloadSize += 4;

		// 1 uint8_t pid profile index
		// 0 -> PID Profile 1 selected
		// 1 -> PID Profile 2 selected
		// 2 -> PID Profile 3 selected
		uint8_t current_pid_profile = 0;
		CK_MSP_WriteBufferU8(buffer, current_pid_profile, payloadSize++, &crc);

		// 1 uint16_t average system load 0 to 100
		CK_MSP_WriteBufferU16(buffer, msp_osd_packet.system_percent, payloadSize, &crc);
		payloadSize +=2;

		if(command_id == MSP_STATUS){

			CK_MSP_WriteBufferU16(buffer, 0, payloadSize, &crc);
			payloadSize +=2;
		}
		else{
			// 1 uint8_t pid profile count
			CK_MSP_WriteBufferU8(buffer, 3, payloadSize++, &crc);

			// 1 uint8_t currentProfileCountIndex
			// 0 -> Rate Profile 1 selected
			// 1 -> Rate Profile 2 selected
			// 2 -> Rate Profile 3 selected
			CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		}

		// This part is not correct because according to the betaflight
		// It writes many bytes but there is only 7 bytes left on uart bytes.
		// I will manually add this part till correct it.
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0x1A, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, 0, payloadSize++, &crc);

		buffer[payloadSize++] = crc;

		break;


	default:

		break;
	}

	return payloadSize;

}

void CK_MSP_ProcessCommand(uint8_t command_id, uint8_t* commandBuf, uint16_t payloadSize){

	switch(command_id){

	case MSP_COPY_PROFILE:

		break;

	case MSP_SET_PID:

		if(payloadSize == 15){

			CK_MSP_SendResponse(MSP_PID);

		}

		break;

	case MSP_SET_RTC:

		CK_MSP_SendResponse(MSP_RTC);

		break;

	case MSP_STATUS:
	case MSP_STATUS_EX:


		break;


	default:

		break;
	}

}

void CK_MSP_WriteBufferU8(uint8_t * buffer, uint8_t data, uint8_t idx, uint8_t* crc){

	buffer[idx] = data;
	*crc ^= data;
}

void CK_MSP_WriteBufferU16(uint8_t * buffer, uint16_t data, uint8_t idx, uint8_t* crc){

	CK_MSP_WriteBufferU8(buffer, data >> 0, idx, crc);

	CK_MSP_WriteBufferU8(buffer, data >> 8, idx+1, crc);

}

void CK_MSP_WriteBufferU32(uint8_t * buffer, uint16_t data, uint8_t idx, uint8_t* crc){

	CK_MSP_WriteBufferU8(buffer, data >> 0, idx, crc);

	CK_MSP_WriteBufferU8(buffer, data >> 8, idx+1, crc);

	CK_MSP_WriteBufferU8(buffer, data >> 16, idx+2, crc);

	CK_MSP_WriteBufferU8(buffer, data >> 24, idx+3, crc);

}

void CK_MSP_WriteU8(uint8_t data, uint8_t* crc){

	CK_USBD_WriteTxBuffer(data);
	*crc ^= data;

}

void CK_MSP_WriteU16(uint16_t data, uint8_t* crc){

	CK_MSP_WriteU8(data >> 0, crc);

	CK_MSP_WriteU8(data >> 8, crc);

}

void CK_MSP_WriteU32(uint32_t data, uint8_t* crc){

	CK_MSP_WriteU8(data >> 0, crc);

	CK_MSP_WriteU8(data >> 8, crc);

	CK_MSP_WriteU8(data >> 16, crc);

	CK_MSP_WriteU8(data >> 24, crc);

}

void CK_MSP_SendResponse(uint8_t resp){

	// General Request Format Is : $M<(size = 0)(type)(crc = type)
	// General Response Format Is : $M>(size)(type)(payload[size])(crc = size XOR type XOR payload[each element])

	uint8_t crc = 0;
	uint8_t type = 0;
	uint8_t size = 0;

	CK_USBD_WriteTxBuffer('$');
	CK_USBD_WriteTxBuffer('M');
	CK_USBD_WriteTxBuffer('>');

	switch(resp){
	case MSP_UID:

		// Payload is 12 bytes:
		// 1 uint32_t uid0
		// 1 uint32_t uid1
		// 1 uint32_t uid2

		size = 12;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_UID;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint32_t uid0
		CK_MSP_WriteU32(0, &crc);

		// 1 uint32_t uid1
		CK_MSP_WriteU32(0, &crc);

		// 1 uint32_t uid2
		CK_MSP_WriteU32(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_API_VERSION:

		// Payload is 3 bytes:

		// 1 uint8_t protocol_version
		// 1 uint8_t api_version_major
		// 1 uint8_t api_version_minor

		size = 3;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_API_VERSION;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t protocol_version
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t api_version_major
		CK_MSP_WriteU8(1, &crc);

		// 1 uint8_t api_version_minor
		CK_MSP_WriteU8(42, &crc);


		CK_USBD_WriteTxBuffer(crc);

		CK_MSP_SendResponse(MSP_TX_INFO);
		break;

	case MSP_BOARD_INFO:

		// Payload is 74 bytes:

		// 4 uint8_t board_identifier
		// 1 uint16_t revision
		// 1 uint8_t osd or not (2 is fc with osd, 0 is not)
		// 1 uint8_t target capabilities
		// 1 uint8_t target name length
		// 10 uint8_t target name
		// 1 uint8_t board name length max 20
		// 10 uint8_t target name
		// 1 uint8_t manifacturer id len 4
		// 10 uint8_t target name
		// 32 uint8_t signature write 0
		// 1 uint8_t mcu type id (3 for stm32f40x)

		size = 65;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_BOARD_INFO;
		CK_MSP_WriteU8(type, &crc);

		// 4 uint8_t board_identifier
		uint8_t board_identifier[4] = {'B','F','H','7'};
		for(int i = 0; i < 4; i++){
			CK_MSP_WriteU8(board_identifier[i], &crc);
		}

		// 1 uint16_t revision
		CK_MSP_WriteU16(0, &crc);

		// 1 uint8_t osd or not (2 is fc with osd, 0 is not)
		CK_MSP_WriteU8(2, &crc);

		// 1 uint8_t target capabilities
		uint8_t targetCapabilities = 0;
		targetCapabilities |= 1 << 0; 		// TARGET_HAS_VCP_BIT
		targetCapabilities |= 1 << 1; 		// TARGET_HAS_SOFT_SERIAL
		targetCapabilities |= 1 << 2; 		// TARGET_IS_UNIFIED_BIT
		CK_MSP_WriteU8(targetCapabilities, &crc);

		// 1 uint8_t target name length
		uint8_t name_len = 12;
		CK_MSP_WriteU8(name_len, &crc);

		// 12 uint8_t target name BetaFlightF4
		uint8_t target_name[12] = {'B','e','t','a','F','l','i','g','h','t','H','7'};
		for(int i = 0; i < 12; i++){
			CK_MSP_WriteU8(target_name[i], &crc);
		}

		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 32 uint8_t signature write 0
		for(int i = 0; i < 32; i++){
			CK_MSP_WriteU8(0, &crc);
		}

		// 1 uint8_t mcu type id (3 for stm32f40x)
		CK_MSP_WriteU8(3, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;


	case MSP_FC_VERSION:

		// Payload is 3 bytes:
		// 1 uint8_t FC_VERSION_MAJOR
		// 1 uint8_t FC_VERSION_MINOR
		// 1 uint8_t FC_VERSION_PATCH_LEVEL

		size = 3;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_FC_VERSION;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t FC_VERSION_MAJOR
		CK_MSP_WriteU8(4, &crc);
		// 1 uint8_t FC_VERSION_MINOR
		CK_MSP_WriteU8(2, &crc);
		// 1 uint8_t FC_VERSION_PATCH_LEVEL
		CK_MSP_WriteU8(9, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;


	case MSP_BUILD_INFO:

		// Payload 26 is bytes:
		// 11 uint8_t BUILD_DATE "MMM DD YYYY" "JUL 03 2019"
		// 8  uint8_t BUILD_TIME "HH:MM:SS" "23:23:00"
		// 7  uint8_t GIT_REVISION write 0

		size = 26;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_BUILD_INFO;
		CK_MSP_WriteU8(type, &crc);

		// 11 uint8_t BUILD_DATE "MMM DD YYYY" "JUL 03 2019"
		uint8_t build_date[11] = {'J','U','N',' ',0,5,' ',2,0,2,3};
		for(int i = 0; i < 11; i++){
			CK_MSP_WriteU8(build_date[i], &crc);
		}

		// 8  uint8_t BUILD_TIME "HH:MM:SS" "23:23:00"
		uint8_t build_time[8] = {0,1,':',0,1,':',0,1};
		for(int i = 0; i < 8; i++){
			CK_MSP_WriteU8(build_time[i], &crc);
		}

		// 7  uint8_t GIT_REVISION write 0
		for(int i = 0; i < 7; i++){
			CK_MSP_WriteU8(0, &crc);
		}

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_FC_VARIANT:

		// Payload 4 is  bytes:
		// 4 uint8_t fc identifier if i write "CKFL" it says not supported

		size = 4;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_FC_VARIANT;
		CK_MSP_WriteU8(type, &crc);

		// 4 uint8_t fc identifier if i write "CKFL" it says not supported
		uint8_t fc_id[4] = {'B','T','F','L'};
		for(int i = 0; i < 4; i++){
			CK_MSP_WriteU8(fc_id[i], &crc);
		}

		CK_USBD_WriteTxBuffer(crc);

		break;


	case MSP_STATUS_EX:
	case MSP_STATUS:

		// Payload 38 is bytes:

		// 1 uint16_t cycleTime (microsec)
		// 1 uint16_t i2c_errors_count
		// 1 uint16_t sensors
		// 4 uint8_t flag (this might be the rc switch related)
		// 1 uint8_t pid profile index
		// 1 uint16_t average system load 0 to 100
		// 1 uint8_t pid profile count
		// 1 uint8_t currentProfileCountIndex
		// 1 uint8_t flag byte count max 16
		// 16 uint8_t flag
		// 1 uint8_t arming disable flag count
		// 1 uint32_t for flags
		// 1 uint8_t config state flag

		size = 38;
		CK_MSP_WriteU8(size, &crc);

		if(resp == MSP_STATUS){
			type = MSP_STATUS;
		}
		else if(resp == MSP_STATUS_EX){
			type = MSP_STATUS_EX;
		}
		CK_MSP_WriteU8(type, &crc);

		// 1 uint16_t cycleTime (microsec)
		uint16_t cycle_time = 125;
		CK_MSP_WriteU16(cycle_time, &crc);

		// 1 uint16_t i2c_errors_count
		uint16_t i2c_error = 3; // later i2c error counter
		CK_MSP_WriteU16(i2c_error, &crc);

		// 1 uint16_t sensors
		uint16_t sensors_used = 0;
		sensors_used |= 1 << 0; // acc
		sensors_used |= 1 << 1; // baro
		sensors_used |= 1 << 2; // mag
		sensors_used |= 1 << 3; // gps
		sensors_used |= 1 << 4; // sonar
		sensors_used |= 1 << 5; // gyro
		CK_MSP_WriteU16(sensors_used, &crc);


		// 4 uint8_t flag (this might be the rc switch related)
		uint8_t flags[4] = {255,255,255,255}; // later correct
		for(int i = 0; i < 4; i++){
			CK_MSP_WriteU8(flags[i], &crc);
		}

		// 1 uint8_t pid profile index
		// 0 -> PID Profile 1 selected
		// 1 -> PID Profile 2 selected
		// 2 -> PID Profile 3 selected
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t average system load 0 to 100
		uint16_t average_systemLoad = 10;
		CK_MSP_WriteU16(average_systemLoad, &crc);


		if(resp == MSP_STATUS_EX){
			// 1 uint8_t pid profile count
			CK_MSP_WriteU8(3, &crc);

			// 1 uint8_t currentProfileCountIndex
			// 0 -> Rate Profile 1 selected
			// 1 -> Rate Profile 2 selected
			// 2 -> Rate Profile 3 selected
			CK_MSP_WriteU8(0, &crc);
		}
		else{
			CK_MSP_WriteU16(0, &crc);
		}


		// 1 uint8_t flag byte count max 16
		uint8_t byte_count = 16;
		CK_MSP_WriteU8(byte_count, &crc);

		// 16 uint8_t flag
		for(int i = 0; i < byte_count; i++){
			CK_MSP_WriteU8(0, &crc);
		}

		// 1 uint8_t arming disable flag count
		CK_MSP_WriteU8(23, &crc);

		// 1 uint32_t for flags
		uint32_t flight_flags = 0;
		CK_MSP_WriteU32(flight_flags, &crc);

		// 1 uint8_t config state flag
		CK_MSP_WriteU8(1, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_BOXIDS:

		// Payload 1 is bytes:
		// 1 uint8_t box id something write 0

		size = 1;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_BOXIDS;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t box id something write 0
		CK_MSP_WriteU8(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_DATAFLASH_SUMMARY:

		// Payload 13 is bytes:
		// 1 uint8_t  something write 0
		// 4 uint8_t  something write 0
		// 4 uint8_t  something write 0
		// 4 uint8_t  something write 0

		size = 13;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_DATAFLASH_SUMMARY;
		CK_MSP_WriteU8(type, &crc);

		for(int i = 0; i < 13; i++){
			CK_MSP_WriteU8(0, &crc);
		}

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_ANALOG:

		// Payload 7 is bytes:
		// 1 uint8_t VBAT 1/10 volts
		// 1 uint16_t inPowerMeterSum
		// 1 uint16_t rssi range 0 to 1023
		// 1 uint16_t amp

		size = 7;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_ANALOG;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t VBAT 1/10 volts send 10 times
		uint8_t vbat = 160; 		// 16V
		CK_MSP_WriteU8(vbat, &crc);

		// 1 uint16_t inPowerMeterSum
		uint16_t mah = 1000; 		// 100mAh
		CK_MSP_WriteU16(mah, &crc);

		// 1 uint16_t rssi range 0 to 1023
		uint16_t rssi = 1023; 		// %100
		CK_MSP_WriteU16(rssi, &crc);

		// 1 uint16_t amp * 100
		uint16_t amp = 110; 		// 1.1A
		CK_MSP_WriteU16(amp, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_FEATURE_CONFIG:

		// Payload 4 is bytes:
		// 1 uint32_t feature mask check later

		size = 4;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_FEATURE_CONFIG;
		CK_MSP_WriteU8(type, &crc);

		for(int i = 0; i < 4; i++){
			//CK_MSP_WriteU8(255, &crc);
		}
		CK_MSP_WriteU32(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_MIXER_CONFIG:

		// Payload 2 is bytes:
		// 1 uint8_t mixer mode 3 for quad x 4 motors
		// 1 uint8_t yaw motor reversed

		size = 2;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_MIXER_CONFIG;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t mixer mode 3 for quad x 4 motors
		CK_MSP_WriteU8(3, &crc);

		// 1 uint8_t yaw motor reversed
		CK_MSP_WriteU8(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_ATTITUDE:

		// Payload 6 is bytes:
		// 1 uint16_t imu roll
		// 1 uint16_t imu pitch
		// 1 uint16_t imu yaw

		size = 6;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_ATTITUDE;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint16_t imu roll * 10
		uint16_t imu_x = 0; 		// 0 degrees
		CK_MSP_WriteU16(imu_x, &crc);

		// 1 uint16_t imu pitch * 10
		uint16_t imu_y = 100; 		// 10 degrees
		CK_MSP_WriteU16(imu_y, &crc);

		// 1 uint16_t imu yaw
		uint16_t imu_z = 358; 		// 358 degrees
		CK_MSP_WriteU16(imu_z, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_PID_CONTROLLER:

		// Payload 1 is bytes:
		// 1 uint8_t PID_CONTROLLER_BETAFLIGHT 1

		size = 1;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_PID_CONTROLLER;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t PID_CONTROLLER_BETAFLIGHT 1
		CK_MSP_WriteU8(1, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_PID:

		// Payload 15 is bytes:
		// 5 times
		// 1 uint8_t P
		// 1 uint8_t I
		// 1 uint8_t D

		// Betaflight pid profile is
		// PID ROLL,
		// PID PITCH,
		// PID YAW,
		// PID LEVEL,
		// PID MAG,

		size = 15;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_PID;
		CK_MSP_WriteU8(type, &crc);

		for(int i = 0; i < 5; i++){
			//CK_MSP_WriteU8(pid_p[i], &crc); // P
			//CK_MSP_WriteU8(pid_i[i], &crc); // I
			//CK_MSP_WriteU8(pid_d[i], &crc); // D
		}

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_PIDNAMES:

		// Payload 20 is bytes:
		// "ROLL",
		// "PITCH",
		// "YAW",
		// "LEVEL",
		// "MAG",

		size = 20;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_PIDNAMES;
		CK_MSP_WriteU8(type, &crc);

		uint8_t name_roll[4] = {'R','O','L','L'};
		for(int i = 0; i < 4; i++){
			CK_MSP_WriteU8(name_roll[i], &crc);
		}

		uint8_t name_pitch[5] = {'P','I','T','C','H'};
		for(int i = 0; i < 5; i++){
			CK_MSP_WriteU8(name_pitch[i], &crc);
		}

		uint8_t name_yaw[3] = {'Y','A','W'};
		for(int i = 0; i < 3; i++){
			CK_MSP_WriteU8(name_yaw[i], &crc);
		}

		uint8_t name_level[5] = {'L','E','V','E','L'};
		for(int i = 0; i < 5; i++){
			CK_MSP_WriteU8(name_level[i], &crc);
		}

		uint8_t name_mag[3] = {'M','A','G'};
		for(int i = 0; i < 3; i++){
			CK_MSP_WriteU8(name_mag[i], &crc);
		}

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_RC_TUNING:

		// Payload 22 is bytes:

		// 1 uint8_t rcRates Roll
		// 1 uint8_t rcExpo Roll
		// 3 uint8_t rates roll, pitch, yaw
		// 1 uint8_t dynThrPid i dont have it send 0
		// 1 uint8_t thrMid8 i dont have it send 0
		// 1 uint8_t thrExpo8 i dont have it send 0
		// 1 uint16_t tpa breakpoint
		// 1 uint8_t rcExpo yaw
		// 1 uint8_t rcRates yaw
		// 1 uint8_t rcRates pitch
		// 1 uint8_t rcExpo pitch
		// 1 uint8_t throttle limit type    i dont have it send 0
		// 1 uint8_t throttle limit percent i dont have it send 0
		// 1 uint16_t rate_limit roll
		// 1 uint16_t rate_limit pitch
		// 1 uint16_t rate_limit yaw

		size = 22;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_RC_TUNING;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t rcRates Roll
		CK_MSP_WriteU8(155, &crc); // 1.55

		// 1 uint8_t rcExpo Roll
		CK_MSP_WriteU8(0, &crc);

		// 3 uint8_t rates roll, pitch, yaw
		for(int i = 0; i < 3; i++){
			CK_MSP_WriteU8(80, &crc); // 0.8
		}

		// 1 uint8_t dynThrPid i dont have it send 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t thrMid8 i dont have it send 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t thrExpo8 i dont have it send 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t tpa breakpoint
		uint16_t tpa_breakpoint = 1650;
		CK_MSP_WriteU16(tpa_breakpoint, &crc);

		// 1 uint8_t rcExpo yaw
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t rcRates yaw
		CK_MSP_WriteU8(155, &crc);

		// 1 uint8_t rcRates pitch
		CK_MSP_WriteU8(155, &crc);
		// 1 uint8_t rcExpo pitch
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t throttle limit type    i dont have it send 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t throttle limit percent i dont have it send 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t rate_limit roll
		uint16_t rate_limit_roll = 1998;
		CK_MSP_WriteU16(rate_limit_roll, &crc);

		// 1 uint16_t rate_limit pitch
		uint16_t rate_limit_pitch = 1998;
		CK_MSP_WriteU16(rate_limit_pitch, &crc);

		// 1 uint16_t rate_limit yaw
		uint16_t rate_limit_yaw = 1998;
		CK_MSP_WriteU16(rate_limit_yaw, &crc);


		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_FILTER_CONFIG:

		// Payload 37 is bytes:

		// 1 uint8_t gyro low pass hz
		// 1 uint16_t dTerm low pass hz
		// 1 uint16_t yaw low pass hz
		// 1 uint16_t gyro notch hz 1
		// 1 uint16_t gyro notch cutoff 1
		// 1 uint16_t dTerm notch hz
		// 1 uint16_t dTerm notch cutoff
		// 1 uint16_t gyro notch hz 2
		// 1 uint16_t gyro notch cutoff 2
		// 1 uint8_t dTerm filter type PT1 0, BIQUAD 1
		// 1 uint8_t gyro hardware lpf (sensor setting)
		// 1 uint8_t 0
		// 1 uint16_t gyro lowpass hz
		// 1 uint16_t gyro lowpass2 hz
		// 1 uint8_t gyro lowpass type PT1 0, BIQUAD 1
		// 1 uint8_t gyro lowpass2 type2 PT1 0, BIQUAD 1
		// 1 uint16_t dTerm lowpass2 hz
		// 1 uint8_t dTerm filter2 type PT1 0, BIQUAD 1
		// 1 uint16_t dynamic lpf gyro min hz
		// 1 uint16_t dynamic lpf gyro max hz
		// 1 uint16_t dynamic lpf dTerm min hz
		// 1 uint16_t dynamic lpf dTerm max hz

		size = 37;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_FILTER_CONFIG;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t gyro low pass hz
		uint8_t gyro_lowpass_hz = 90;
		CK_MSP_WriteU8(gyro_lowpass_hz, &crc);

		// 1 uint16_t dTerm low pass hz
		uint16_t dTerm_lowpass_hz = 70;
		CK_MSP_WriteU16(dTerm_lowpass_hz, &crc);

		// 1 uint16_t yaw low pass hz
		uint16_t yaw_lowpass_hz = 0;
		CK_MSP_WriteU16(yaw_lowpass_hz, &crc);

		// 1 uint16_t gyro notch hz 1
		uint16_t gyro_notch_hz_1 = 180;
		CK_MSP_WriteU16(gyro_notch_hz_1, &crc);

		// 1 uint16_t gyro notch cutoff 1
		uint16_t gyro_notch_cutoff_1 = 120;
		CK_MSP_WriteU16(gyro_notch_cutoff_1, &crc);

		// 1 uint16_t dTerm notch hz
		uint16_t dTerm_notch_hz = 0;
		CK_MSP_WriteU16(dTerm_notch_hz, &crc);

		// 1 uint16_t dTerm notch cutoff
		uint16_t dTerm_notch_cutoff = 0;
		CK_MSP_WriteU16(dTerm_notch_cutoff, &crc);

		// 1 uint16_t gyro notch hz 2
		uint16_t gyro_notch_hz_2 = 160;
		CK_MSP_WriteU16(gyro_notch_hz_2, &crc);

		// 1 uint16_t gyro notch cutoff 2
		uint16_t gyro_notch_cutoff_2 = 110;
		CK_MSP_WriteU16(gyro_notch_cutoff_2, &crc);

		// 1 uint8_t dTerm filter type PT1 0, BIQUAD 1
		uint8_t dTerm_type = 0;
		CK_MSP_WriteU8(dTerm_type, &crc);

		// 1 uint8_t gyro hardware lpf (sensor setting)
		// 0 normal, 1 1khz sample (not that much important)
		uint8_t gyro_hardware_lpf = 1;
		CK_MSP_WriteU8(gyro_hardware_lpf, &crc);

		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t gyro lowpass hz
		CK_MSP_WriteU16((uint16_t)gyro_lowpass_hz, &crc);

		// 1 uint16_t gyro lowpass2 hz
		uint16_t gyro_lowpass2_hz = 0;
		CK_MSP_WriteU16(gyro_lowpass2_hz, &crc);

		// 1 uint8_t gyro lowpass type PT1 0, BIQUAD 1
		uint8_t gyro_lowpass_type = 0;
		CK_MSP_WriteU8(gyro_lowpass_type, &crc);

		// 1 uint8_t gyro lowpass2 type2 PT1 0, BIQUAD 1
		uint8_t gyro_lowpass2_type = 0;
		CK_MSP_WriteU8(gyro_lowpass2_type, &crc);

		// 1 uint16_t dTerm lowpass2 hz
		uint16_t dTerm_lowpass2_hz = 0;
		CK_MSP_WriteU16(dTerm_lowpass2_hz, &crc);

		// 1 uint8_t dTerm filter2 type PT1 0, BIQUAD 1
		uint8_t dTerm_filter2_type = 0;
		CK_MSP_WriteU8(dTerm_filter2_type, &crc);

		// Dynamic filter i dont have send 0
		// 1 uint16_t dynamic lpf gyro min hz
		CK_MSP_WriteU16(0, &crc);
		// 1 uint16_t dynamic lpf gyro max hz
		CK_MSP_WriteU16(0, &crc);
		// 1 uint16_t dynamic lpf dTerm min hz
		CK_MSP_WriteU16(0, &crc);
		// 1 uint16_t dynamic lpf dTerm max hz
		CK_MSP_WriteU16(0, &crc);


		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_RC_DEADBAND:

		// Payload 5 is bytes:

		// 1 uint8_t dead band
		// 1 uint8_t yaw dead band
		// 1 uint8_t althold dead band
		// 1 uint16_t deadBand3d throttle


		size = 5;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_RC_DEADBAND;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t dead band (rc_idle_drift)
		uint8_t rc_idle_drift = 15;
		CK_MSP_WriteU8(rc_idle_drift, &crc);

		// 1 uint8_t yaw dead band
		uint8_t rc_idle_drift_yaw = 15;
		CK_MSP_WriteU8(rc_idle_drift_yaw, &crc);

		// 1 uint8_t althold dead band i dont use it
		uint8_t althold_deadband = 0;
		CK_MSP_WriteU8(althold_deadband, &crc);

		// 1 uint16_t deadBand3d throttle i dont have
		uint16_t deadBand3d_throttle = 0;
		CK_MSP_WriteU16(deadBand3d_throttle, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_RC:

		// Payload 36 is bytes:

		// 1 uint16_t rcData for 18 channels

		size = 36;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_RC;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint16_t rcData for 18 channels
		for(int i = 0; i < 18; i++){
			uint16_t rcData = 0;
			CK_MSP_WriteU16(rcData, &crc);
		}

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_PID_ADVANCED:

		// 1 uint16_t 0
		// 1 uint16_t 0
		// 1 uint16_t 0
		// 1 uint8_t 0
		// 1 uint8_t vbat pid compensation
		// 1 uint8_t feed forward transition
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint16_t rate acc limit
		// 1 uint16_t yaw rate acc limit
		// 1 uint8_t level angle limit
		// 1 uint8_t 0
		// 1 uint16_t iTerm throttle threshold
		// 1 uint16_t iTerm accelerator gain
		// 1 uint16_t 0
		// 1 uint8_t iTerm rotation
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t abs control gain
		// 1 uint8_t throttle boost
		// 1 uint8_t acro trainer angle limit
		// 1 uint16_t feedforward pid roll
		// 1 uint16_t feedforward pid pitch
		// 1 uint16_t feedforward pid yaw
		// 1 uint8_t anti gravitiy
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0
		// 1 uint8_t 0

		size = 46;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_PID_ADVANCED;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint16_t 0
		CK_MSP_WriteU16(0, &crc);
		// 1 uint16_t 0
		CK_MSP_WriteU16(0, &crc);
		// 1 uint16_t 0
		CK_MSP_WriteU16(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t vbat pid compensation
		uint8_t vBatPid = 0;
		CK_MSP_WriteU8(vBatPid, &crc);

		// 1 uint8_t feed forward transition
		uint8_t feedforward_transition = 50; // 50 in betaflight
		CK_MSP_WriteU8(feedforward_transition, &crc);

		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t rate acc limit
		CK_MSP_WriteU16(0, &crc);

		// 1 uint16_t yaw rate acc limit
		CK_MSP_WriteU16(0, &crc);

		// 1 uint8_t level angle limit
		uint8_t level_limit = 50;
		CK_MSP_WriteU8(level_limit, &crc);

		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint16_t iTerm throttle threshold
		CK_MSP_WriteU16(0, &crc);

		// 1 uint16_t iTerm accelerator gain
		CK_MSP_WriteU16(0, &crc);

		// 1 uint16_t 0
		CK_MSP_WriteU16(0, &crc);

		// 1 uint8_t iTerm rotation
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// I term relax i dont have 0
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t abs control gain
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t throttle boost
		CK_MSP_WriteU8(0, &crc);

		// 1 uint8_t acro trainer angle limit
		CK_MSP_WriteU8(0, &crc);


		// 1 uint16_t feedforward pid roll
		CK_MSP_WriteU16(200, &crc);
		// 1 uint16_t feedforward pid pitch
		CK_MSP_WriteU16(200, &crc);
		// 1 uint16_t feedforward pid yaw
		CK_MSP_WriteU16(200, &crc);

		// 1 uint8_t anti gravitiy
		// smooth -> 0, step -> 1
		CK_MSP_WriteU8(0, &crc);

		// Use dmin
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		// Use integrated yaw control
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);
		// 1 uint8_t 0
		CK_MSP_WriteU8(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_GPS_RESCUE_PIDS:

		size = 14;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_GPS_RESCUE_PIDS;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint16_t throttle p
		CK_MSP_WriteU16(10, &crc);
		// 1 uint16_t throttle i
		CK_MSP_WriteU16(20, &crc);
		// 1 uint16_t throttle d
		CK_MSP_WriteU16(30, &crc);

		// 1 uint16_t vel p
		CK_MSP_WriteU16(40, &crc);
		// 1 uint16_t vel i
		CK_MSP_WriteU16(50, &crc);
		// 1 uint16_t vel d
		CK_MSP_WriteU16(60, &crc);

		// 1 uint16_t yaw p
		CK_MSP_WriteU16(70, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_BATTERY_CONFIG:

		size = 13;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_BATTERY_CONFIG;
		CK_MSP_WriteU8(type, &crc);

		// 1 uint8_t
		CK_MSP_WriteU8(10, &crc);
		// 1 uint8_t
		CK_MSP_WriteU8(10, &crc);
		// 1 uint8_t
		CK_MSP_WriteU8(10, &crc);
		// 1 uint16_t
		CK_MSP_WriteU16(10, &crc);
		// 1 uint8_t
		CK_MSP_WriteU8(10, &crc);
		// 1 uint8_t
		CK_MSP_WriteU8(10, &crc);
		// 1 uint16_t
		CK_MSP_WriteU16(10, &crc);
		// 1 uint16_t
		CK_MSP_WriteU16(10, &crc);
		// 1 uint16_t
		CK_MSP_WriteU16(10, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_RTC:

		size = 9;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_RTC;
		CK_MSP_WriteU8(type, &crc);

		CK_MSP_WriteU16(0, &crc);
		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU16(0, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	case MSP_TX_INFO:

		size = 2;
		CK_MSP_WriteU8(size, &crc);

		type = MSP_TX_INFO;
		CK_MSP_WriteU8(type, &crc);

		CK_MSP_WriteU8(0, &crc);
		CK_MSP_WriteU8(0XFF, &crc);

		CK_USBD_WriteTxBuffer(crc);

		break;

	default:

		size = 0;
		size++;
		break;

	}

	CK_USBD_Transmit();

}


