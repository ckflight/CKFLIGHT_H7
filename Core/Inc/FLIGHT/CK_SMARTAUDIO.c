
#include <COMMON/maths.h>
#include <COMMON/crc.h>
#include "FLIGHT/CK_SMARTAUDIO.h"
#include "FLIGHT/CK_RECEIVER.h"

#include "DRIVERS/CK_SOFTSERIAL.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "COMMUNICATION/CK_PRINTER.h"


// cdm >> 1 version
#define SA_GET_SETTINGS		0x01
#define SA_GET_SETTINGS_V2	0x09
#define SA_SET_POWER		0x02
#define SA_SET_CHANNEL		0x03
#define SA_SET_FREQUENCY	0x04
#define SA_SET_MODE			0x05

#define SA_POWER_25MW 		0
#define SA_POWER_200MW 		1
#define SA_POWER_500MW 		2
#define SA_POWER_800MW 		3

#define SA_SYNC				0xAA
#define SA_HEADER			0x55

typedef enum {
  SA_NONE,
  SA_V1,
  SA_V2
}SMARTAUDIO_VERSION;

typedef struct {
	SMARTAUDIO_VERSION vtx_version;
	uint8_t channel;
	uint8_t powerLevel;
	uint8_t operation_mode;
	uint16_t frequency;
}SmartAudio_Settings;

#define VTX_WAIT		100

#define RECEPTION_SIZE	10

#define POWER_V1	0
#define POWER_V2	1

#define BAND_A		0
#define BAND_B		1
#define BAND_E		2
#define AIRWAVE		3
#define RACEBAND	4

#define CH1			0
#define CH2			1
#define CH3			2
#define CH4			3
#define CH5			4
#define CH6			5
#define CH7			6

#define mW_25		0
#define mW_200		1
#define mW_500		2
#define mW_800		3

int channel_table[5][8] = {
					// CH1, 2, 	3, 	4, 	5,	6, 	7, 	8
	[BAND_A]		= {0, 	1, 	2, 	3, 	4, 	5, 	6,	7},
	[BAND_B]		= {8, 	9, 	10, 11, 12, 13, 14, 15},
	[BAND_E]		= {16, 	17, 18, 19, 20, 21, 22, 23},
	[AIRWAVE]		= {24, 	25, 26, 27, 28, 29, 30, 31},
	[RACEBAND]		= {32, 	33, 34, 35, 36, 37, 38, 39}

};

int power_table[2][4] = {
					  //25mW, 100mW, 400mW, 1000mW
	[POWER_V1]		= {7, 	  16, 	 25,    40},
	[POWER_V2]		= {0, 	  1,     2,     3}

};

SmartAudio_Settings smart_audio;

uint8_t tx_buffer[32];
uint8_t tx_buffer_idx = 0;

uint8_t rx_buffer[32];
uint8_t rx_buffer_idx = 0;

bool is_powered_set = false;
SMARTAUDIO_POWER selected_power = 0;

bool is_configured = false;

int get_settings_bytes 	= 0; // was 9 for cyclone, tbs sends 18
int get_power_bytes 	= 0; // was 6 for cyclone, tbs sends 8
int get_channel_bytes 	= 0; // was 6 for cyclone, tbs sends 8

uart_idle_polarity_t pl;

void CK_SMARTAUDIO_Init(SMARTAUDIO_BAND band, SMARTAUDIO_CHANNEL channel, SMARTAUDIO_POWER power, VTX_Type type){

	bool isRxCorrect = false;

	selected_power = power;

	/*
	 * Cyclone is idle high between frames, and sends 1 between bytes as stop bit, 10ms between command and response
	 * Cyclone accepts commands even if it says resp is false because it does not send headers in response but sets up correctly.
	 * Cyclone does not send 0xAA 0x55 headers rest is same
	 */

	/*
	 * TBS is idle low between frames, but it sends 1 between bytes as stop bit as well, 100ms between command and response.
	 */

	uart_idle_polarity_t pl = IDLE_LOW;

	if(type == CYCLONE){
		get_settings_bytes 	= 9; // was 9 for cyclone, tbs sends 18
		get_power_bytes 	= 6; // was 6 for cyclone, tbs sends 8
		get_channel_bytes 	= 6; // was 6 for cyclone, tbs sends 8

		pl = IDLE_HIGH;

	}
	else if(type == TBS_UNIFY_PRO32){
		get_settings_bytes 	= 18; // was 9 for cyclone, tbs sends 18
		get_power_bytes 	= 8; // was 6 for cyclone, tbs sends 8
		get_channel_bytes 	= 8; // was 6 for cyclone, tbs sends 8

		pl = IDLE_LOW;

	}

	CK_SOFTSERIAL_Init(4800, 2, pl);

	CK_TIME_DelayMilliSec(VTX_WAIT * 5);

	CK_SOFTSERIAL_SetOutput();

	CK_SMARTAUDIO_TX_Packet(SA_GET_SETTINGS, 0);

	// VTX sends response after 1ms
	// TBS sends response after 100ms
	CK_SOFTSERIAL_SetInput();

	isRxCorrect = CK_SMARTAUDIO_RX_Packet(SA_GET_SETTINGS, get_settings_bytes); // Get 10 bytes
	CK_PRINTER_PrintString("SmartAudio Resp: ");
	CK_PRINTER_PrintlnInt(isRxCorrect);


	// Checked each power output with rf meter and it is working
	CK_TIME_DelayMilliSec(VTX_WAIT);

	CK_SOFTSERIAL_SetOutput();

	CK_SMARTAUDIO_TX_Packet(SA_SET_POWER, power_table[POWER_V2][mW_25]);

	// VTX sends response after 1ms
	CK_SOFTSERIAL_SetInput();

	isRxCorrect = CK_SMARTAUDIO_RX_Packet(SA_SET_POWER, get_power_bytes);
	CK_PRINTER_PrintString("SmartAudio Resp: ");
	CK_PRINTER_PrintlnInt(isRxCorrect);



	// Checked each channel with 2 for loops and both vtx and goggle is working
	CK_TIME_DelayMilliSec(VTX_WAIT);

	CK_SOFTSERIAL_SetOutput();

	CK_SMARTAUDIO_TX_Packet(SA_SET_CHANNEL, channel_table[band][channel]);

	// VTX sends response after 1ms
	CK_SOFTSERIAL_SetInput();

	isRxCorrect = CK_SMARTAUDIO_RX_Packet(SA_SET_CHANNEL, get_channel_bytes);
	CK_PRINTER_PrintString("SmartAudio Resp: ");
	CK_PRINTER_PrintlnInt(isRxCorrect);



	CK_TIME_DelayMilliSec(VTX_WAIT);

	CK_SOFTSERIAL_SetOutput();

	CK_SMARTAUDIO_TX_Packet(SA_GET_SETTINGS, 0);

	// VTX sends response after 1ms
	CK_SOFTSERIAL_SetInput();

	isRxCorrect = CK_SMARTAUDIO_RX_Packet(SA_GET_SETTINGS, get_settings_bytes);
	CK_PRINTER_PrintString("SmartAudio Resp: ");
	CK_PRINTER_PrintlnInt(isRxCorrect);



	is_powered_set = false; // set after arming

	is_configured = true; // Vtx receives the command but the decode of resp is not correct
}

void CK_SMARTAUDIO_Update(void){

	// DISARM HANDLE
	if(is_configured){

		if(!flags.ARMED && is_powered_set){

			// Checked each power output with rf meter and it is working
			CK_TIME_DelayMilliSec(VTX_WAIT);

			CK_SOFTSERIAL_SetOutput();

			CK_SMARTAUDIO_TX_Packet(SA_SET_POWER, power_table[POWER_V2][mW_25]);

			// VTX sends response after 1ms
			CK_SOFTSERIAL_SetInput();

			CK_SMARTAUDIO_RX_Packet(SA_SET_POWER, get_power_bytes);

			is_powered_set = false;
		}

		if(flags.ARMED && !is_powered_set){

			// Checked each power output with rf meter and it is working
			CK_TIME_DelayMilliSec(VTX_WAIT);

			CK_SOFTSERIAL_SetOutput();

			CK_SMARTAUDIO_TX_Packet(SA_SET_POWER, power_table[POWER_V2][selected_power]);

			// VTX sends response after 1ms
			CK_SOFTSERIAL_SetInput();

			CK_SMARTAUDIO_RX_Packet(SA_SET_POWER, get_power_bytes);

			is_powered_set = true;


		}

	}
}

void CK_SMARTAUDIO_TX_Packet(uint8_t cmd, uint16_t value){

	//here: length --> only payload, without CRC
	//here: CRC --> calculated for complete packet 0xAA ... payload

	tx_buffer[tx_buffer_idx++] = 0x00; // Needed to make line low, recommended on TBS pdf
	tx_buffer[tx_buffer_idx++] = 0xAA; // Header sync
	tx_buffer[tx_buffer_idx++] = 0x55; // Header sync
	tx_buffer[tx_buffer_idx++] = (cmd << 1) | 0x01; // cmd, Comman byte needed to be shifted 1 byte and LSB must be 1

	switch (cmd){

		case SA_GET_SETTINGS:

			tx_buffer[tx_buffer_idx++] = 0x00; // length
			tx_buffer[tx_buffer_idx++] = CK_SMARTAUDIO_Calculate_CRC8(tx_buffer, 1, 5);
			tx_buffer[tx_buffer_idx++] = 0x00;

			break;

		case SA_SET_POWER:

			tx_buffer[tx_buffer_idx++] = 0x01; // length
			tx_buffer[tx_buffer_idx++] = (smart_audio.vtx_version == SA_V1) ? power_table[POWER_V1][value] : power_table[POWER_V2][value];
			tx_buffer[tx_buffer_idx++] = CK_SMARTAUDIO_Calculate_CRC8(tx_buffer, 1, 6);
			tx_buffer[tx_buffer_idx++] = 0x00;

			break;

		case SA_SET_CHANNEL:

			tx_buffer[tx_buffer_idx++] = 0x01; // length
			tx_buffer[tx_buffer_idx++] = value;
			tx_buffer[tx_buffer_idx++] = CK_SMARTAUDIO_Calculate_CRC8(tx_buffer, 1, 6);
			tx_buffer[tx_buffer_idx++] = 0x00;

			break;

		case SA_SET_FREQUENCY:

			tx_buffer[tx_buffer_idx++] = 0x02;
			tx_buffer[tx_buffer_idx++] = (value>>8); // high byte first
			tx_buffer[tx_buffer_idx++] = value;
			tx_buffer[tx_buffer_idx++] = CK_SMARTAUDIO_Calculate_CRC8(tx_buffer, 1, 7);
			tx_buffer[tx_buffer_idx++] = 0x00;

			break;

		case SA_SET_MODE: // supported for V2 only: UNIFY HV and newer

			if (smart_audio.vtx_version == SA_V2){
				//TBD --> Pit mode
				tx_buffer[tx_buffer_idx++] = 0x01; // length
				tx_buffer[tx_buffer_idx++] = value;
				tx_buffer[tx_buffer_idx++] = CK_SMARTAUDIO_Calculate_CRC8(tx_buffer, 1, 6);
				tx_buffer[tx_buffer_idx++] = 0x00;

			}

			break;
	}

	for(int i = 0; i < tx_buffer_idx; i++){
		CK_SOFTSERIAL_Write(tx_buffer[i]);
	}

	tx_buffer_idx = 0;


}

bool CK_SMARTAUDIO_RX_Packet(uint8_t cmd, uint8_t size){

	bool result = false;
	for(int i = 0; i < size; i++){

		rx_buffer[rx_buffer_idx++] = CK_SOFTSERIAL_Read();

	}

	for(int i = 0; i < rx_buffer_idx; i++){
		CK_PRINTER_PrintInt(rx_buffer[i]);
		CK_PRINTER_PrintString("   ");
	}
	CK_PRINTER_PrintlnString("");

	if(CK_SMARTAUDIO_DecodePacket(cmd, rx_buffer_idx)){
		result = true;
		CK_PRINTER_PrintlnString("rx true");
	}
	else{
		CK_PRINTER_PrintlnString("rx false");
	}

	rx_buffer_idx = 0;

	return result;

}

bool CK_SMARTAUDIO_DecodePacket(uint8_t cmd, uint8_t size){

	switch(cmd){
	case SA_GET_SETTINGS:

		if(rx_buffer[0] == SA_SYNC && rx_buffer[1] == SA_HEADER){

			if(rx_buffer[2] == SA_GET_SETTINGS){
				smart_audio.vtx_version = SA_V1;
			}
			else if(rx_buffer[2] == SA_GET_SETTINGS_V2){
				smart_audio.vtx_version = SA_V2;
			}
			else{
				smart_audio.vtx_version = SA_NONE;
			}

			if(smart_audio.vtx_version != SA_NONE){

				// payload + 1 CRC
				uint8_t frame_length = rx_buffer[3];

				if(frame_length >= 2){

					smart_audio.channel = rx_buffer[4];

					smart_audio.powerLevel = rx_buffer[5];

					smart_audio.operation_mode = rx_buffer[6];

					smart_audio.frequency = rx_buffer[7] << 8 | rx_buffer[8];

					if(CK_SMARTAUDIO_Calculate_CRC8(rx_buffer, 2, size - 1) == rx_buffer[size - 1]){

						return true;
					}

				}

			}

		}
		break;

	case SA_SET_POWER:

		if(rx_buffer[0] == SA_SYNC && rx_buffer[1] == SA_HEADER){

			if(rx_buffer[2] == SA_SET_POWER){

				// payload + 1 CRC
				uint8_t frame_length = rx_buffer[3];

				if(frame_length == 2){

					smart_audio.powerLevel = rx_buffer[4];

					// reserved byte 0x01
					if(rx_buffer[5] == 0x01){
						if(CK_SMARTAUDIO_Calculate_CRC8(rx_buffer, 2, size - 1) == rx_buffer[size - 1]){

							return true;
						}
					}


				}

				return true;
			}

		}
		break;

	case SA_SET_CHANNEL:

		if(rx_buffer[0] == SA_SYNC && rx_buffer[1] == SA_HEADER){

			if(rx_buffer[2] == SA_SET_CHANNEL){

				// payload + 1 CRC
				uint8_t frame_length = rx_buffer[3];

				if(frame_length == 3){

					smart_audio.channel = rx_buffer[4];

					// reserved byte 0x01
					if(rx_buffer[5] == 0x01){
						if(CK_SMARTAUDIO_Calculate_CRC8(rx_buffer, 2, size - 1) == rx_buffer[size - 1]){

							return true;
						}
					}


				}

				return true;
			}

		}
		break;

	case SA_SET_FREQUENCY:

		if(rx_buffer[0] == SA_SYNC && rx_buffer[1] == SA_HEADER){

			if(rx_buffer[2] == SA_SET_FREQUENCY){

				// payload + 1 CRC
				uint8_t frame_length = rx_buffer[3];

				if(frame_length == 4){

					smart_audio.frequency = rx_buffer[4] << 8 | rx_buffer[5];

					// reserved byte 0x01
					if(rx_buffer[6] == 0x01){
						if(CK_SMARTAUDIO_Calculate_CRC8(rx_buffer, 2, size - 1) == rx_buffer[size - 1]){

							return true;
						}
					}


				}

				return true;
			}

		}
		break;

	default:
		break;
	}



	return false;

}

uint8_t CK_SMARTAUDIO_Calculate_CRC8(uint8_t* buffer, uint8_t start_idx, uint8_t end_idx){

	uint8_t crc = crc8_dvb_s2(0, buffer[start_idx]);

	for(int i = start_idx + 1; i < end_idx; i++){

		crc = crc8_dvb_s2(crc, buffer[i]);

	}

	return crc;
}

