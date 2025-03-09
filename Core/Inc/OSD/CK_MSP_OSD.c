/*
 * CK_MSP_OSD.c
 *
 *  Created on: Jun 12, 2023
 *      Author: ck
 */

#include "OSD/CK_MSP_OSD.h"
#include "OSD/CK_OSD.h"

#include "COMMUNICATION/CK_MSP.h"
#include "DRIVERS/CK_ADC.h"
#include "FLIGHT/CK_CRSF.h"
#include "FLIGHT/CK_RECEIVER.h"
msp_osd_config_t dji_osd;

// Display canvas mode hd: 0 start and 53 end
uint8_t current_font = 0x02;

int timer_plot_row 							= 18;
int timer_plot_column						= 4;
#define timer_parameters_len				6
int timer_plot_freq_ 						= TARGET_1HZ_US;
char timer_parameters[timer_parameters_len];

int rssi_dbm_plot_row 						= 18;
int rssi_dbm_plot_column					= 26;
#define rssi_dbm_parameters_len				5
int rssi_plot_freq_ 						= TARGET_10HZ_US;
char rssi_dbm_parameters[rssi_dbm_parameters_len];

int rssi_lq_plot_row 						= 17;
int rssi_lq_plot_column						= 40;
#define rssi_lq_parameters_len				4
int rssi_lq_plot_freq_ 						= TARGET_10HZ_US;
char rssi_lq_parameters[rssi_lq_parameters_len];

int core_temperature_plot_row 				= 17;
int core_temperature_plot_column			= 46;
#define core_temperature_parameters_len		3
int core_temperature_plot_freq_ 			= TARGET_1HZ_US;
char core_temperature_parameters[core_temperature_parameters_len];

int current_plot_row 						= 17;
int current_plot_column						= 35;
#define current_parameters_len				5
int current_plot_freq_ 						= TARGET_10HZ_US;
char current_parameters[current_parameters_len];

int fw_freq_plot_row 						= 17;
int fw_freq_plot_column						= 4;
#define fw_freq_parameters_len				7
int fw_freq_plot_freq_ 						= TARGET_10HZ_US;
char fw_freq_parameters[fw_freq_parameters_len];

int mah_plot_row 						= 18;
int mah_plot_column						= 16;
#define mah_parameters_len				5
int mah_plot_freq_ 						= TARGET_10HZ_US;
char mah_parameters[mah_parameters_len];

MSP_OSD_PACKET_s msp_osd_packet;

void CK_MSP_OSD_Init(uint32_t osdT, uint32_t mainT){

	dji_osd.sync.syncCounter 		= 0;

	dji_osd.sync.targetLoopTime 	= osdT;

	dji_osd.sync.syncRate 			= osdT / mainT;

	current_font = 0x00;

}


void CK_MSP_OSD_SetData(void){

	msp_osd_packet.cpu_core_temperature = (uint8_t)CK_ADC_GetTemperatureResult();

	msp_osd_packet.current = (uint16_t)(CK_ADC_GetCurrentResult() * 100.0f);

	msp_osd_packet.freqResult = osd_packet.freqResult;

	msp_osd_packet.isArmed = flags.ARMED;

	msp_osd_packet.rssi_dBm = CK_CRSF_GetRSSI_dBm();

	msp_osd_packet.rssi_link_quality = CK_CRSF_GetLinkQuality();

	msp_osd_packet.voltage = (uint16_t)(CK_ADC_GetLipoResult() * 100.0f);

	msp_osd_packet.system_percent = osd_packet.system_percent;

	msp_osd_packet.mainLoopTime = 0;

}

// 100 Hz
void CK_MSP_OSD_Update(uint32_t currentTime){

	CK_MSP_OSD_Timer(currentTime);

	CK_MSP_OSD_Current(currentTime);

	CK_MSP_OSD_RSSI(currentTime);

	CK_MSP_OSD_RSSILQ(currentTime);

	CK_MSP_OSD_CoreTemperature(currentTime);

	CK_MSP_OSD_FirmwareFreqPlot(currentTime);

	CK_MSP_OSD_MahPlot(currentTime);

}

void CK_MSP_OSD_Timer(uint32_t currentTime){

	static uint32_t timerCounter = 0;
	static uint8_t minutes, seconds;

	static uint32_t preTime = 0;
	uint32_t delta = currentTime - preTime;

	if(delta >= timer_plot_freq_){

		preTime = currentTime;

		timerCounter++;

		minutes = timerCounter / 60;
		seconds = timerCounter % 60;

		timer_parameters[0] = SYM_ON_M;
		timer_parameters[1] = 48 + (minutes / 10);
		timer_parameters[2] = 48 + (minutes % 10);
		timer_parameters[3] = ':';
		timer_parameters[4] = 48 + (seconds / 10);
		timer_parameters[5] = 48 + (seconds % 10);
	}

}

void CK_MSP_OSD_Current(uint32_t currentTime){

	static uint32_t preTime = 0;
	float delta = currentTime - preTime;

	if(delta >= current_plot_freq_){

		preTime = currentTime;

		float current = msp_osd_packet.current;
		current /= 100.0f; // osd multiplies with 100 for getting 2 decimal points

		// I = (Vout * 1000) / (0.5 x resistor)
		current = (current * 1000) / (CURRENT_RESISTOR * 0.5f); // Current Sens Vout = I*0.5m*105K/1K = I*52.5m

		// Add calibration multiply here
		current *= MAH_CALIBRATION_MULTIPLIER;

		/* Get Each Float Digit(1 Decimal After Point) */
		int current_temp = current * 10;

		if(current_temp >= 0 && current_temp <= 99){ // 0.0A to 9.9A

			current_parameters[0] = ' ';// Blank Space
			current_parameters[1] = ' ';// Blank Space

			current_parameters[2] = 48 + (current_temp / 10);
			current_parameters[3] = 48 + (current_temp % 10);

		}
		else if(current_temp > 99 && current_temp <= 999){ // 10.0A to 99.9A

			current_parameters[0] = ' ';// Blank Space
			current_parameters[1] = 48 + (current_temp / 100);
			current_temp = current_temp % 100;

			current_parameters[2] = 48 + (current_temp / 10);
			current_parameters[3] = 48 + (current_temp % 10);

		}
		else if(current_temp > 999 && current_temp <= 9999){ // 100.0A to 999.9A

			current_parameters[0] = 48 + (current_temp / 1000);
			current_temp = current_temp % 1000;

			current_parameters[1] = 48 + (current_temp / 100);
			current_temp = current_temp % 100;

			current_parameters[2] = 48 + (current_temp / 10);
			current_parameters[3] = 48 + (current_temp % 10);

		}

		current_parameters[4] = SYM_AMP;

	}
}

void CK_MSP_OSD_RSSI(uint32_t currentTime){

	static uint32_t preTime = 0;
	uint32_t delta = currentTime - preTime;

	if(delta >= rssi_plot_freq_){

		preTime = currentTime;

		int16_t rssi_temp = msp_osd_packet.rssi_dBm;

		rssi_dbm_parameters[0] = SYM_RSSI;

		if(rssi_temp < 0 && rssi_temp >= -9){
			rssi_dbm_parameters[1] = '-';
			rssi_dbm_parameters[2] = 48 + (rssi_temp * -1);// Blank Space
			rssi_dbm_parameters[3] = ' ';
			rssi_dbm_parameters[4] = ' ';
		}
		else if(rssi_temp < -9 && rssi_temp >= -99){
			rssi_dbm_parameters[1] = '-';// Blank Space
			rssi_dbm_parameters[2] = 48 + ((rssi_temp / 10) * -1);
			rssi_dbm_parameters[3] = 48 + ((rssi_temp % 10) * -1);
			rssi_dbm_parameters[4] = ' ';
		}
		else if(rssi_temp < -99 && rssi_temp >= -999){
			rssi_dbm_parameters[1] = '-';
			rssi_dbm_parameters[2] = 48 + ((rssi_temp / 100) * -1);
			rssi_dbm_parameters[3] = 48 + (((rssi_temp % 100) / 10) * -1);
			rssi_dbm_parameters[4] = 48 + (((rssi_temp % 100) % 10) * -1);
		}

	}
}

void CK_MSP_OSD_RSSILQ(uint32_t currentTime){

	static uint32_t preTime = 0;
	uint32_t delta = currentTime - preTime;

	if(delta >= rssi_lq_plot_freq_){

		preTime = currentTime;

		uint8_t rssi_temp = msp_osd_packet.rssi_link_quality;

		rssi_lq_parameters[0] = SYM_LINK_QUALITY;

		if(rssi_temp >= 0 && rssi_temp < 10){
			rssi_lq_parameters[1] = ' ';
			rssi_lq_parameters[2] = ' ';
			rssi_lq_parameters[3] = 48 + (rssi_temp % 10);
		}
		else if(rssi_temp >= 10 && rssi_temp < 100){
			rssi_lq_parameters[1] = ' ';
			rssi_lq_parameters[2] = 48 + (rssi_temp / 10);
			rssi_lq_parameters[3] = 48 + (rssi_temp % 10);
		}
		else if(rssi_temp >= 100 && rssi_temp < 1000){
			rssi_lq_parameters[1] = 48 + (rssi_temp / 100);
			rssi_temp = rssi_temp % 100;

			rssi_lq_parameters[2] = 48 + (rssi_temp / 10);
			rssi_lq_parameters[3] = 48 + (rssi_temp % 10);
		}
	}

}

void CK_MSP_OSD_CoreTemperature(uint32_t currentTime){

	static uint32_t preTime = 0;
	uint32_t delta = currentTime - preTime;

	if(delta >= core_temperature_plot_freq_){

		preTime = currentTime;

		int core_temperature_temp = msp_osd_packet.cpu_core_temperature;

		// First plot num of Sattelite than plot sattelite symbol
		if(core_temperature_temp >= 0 && core_temperature_temp < 10){
			core_temperature_parameters[0] = ' ';
			core_temperature_parameters[1] = 48 + (core_temperature_temp % 10);
		}
		else if(core_temperature_temp >= 10 && core_temperature_temp < 100){
			core_temperature_parameters[0] = 48 + (core_temperature_temp / 10);
			core_temperature_parameters[1] = 48 + (core_temperature_temp % 10);
		}

		core_temperature_parameters[2] = SYM_C;
	}

}

void CK_MSP_OSD_FirmwareFreqPlot(uint32_t currentTime){

	static uint32_t preTime = 0;
	float delta = currentTime - preTime;

	int rate_temp;

	if(delta >= fw_freq_plot_freq_){

		preTime = currentTime;

		// 8000 will be printed as 8.0 so divide with 100
		rate_temp = msp_osd_packet.freqResult / 100;

		if(rate_temp >= 0 && rate_temp < 10){
			fw_freq_parameters[0] = ' ';
			fw_freq_parameters[1] = 48;
			fw_freq_parameters[2] = '.';
			fw_freq_parameters[3] = 48 + (rate_temp % 10);
		}
		else if(rate_temp >= 10 && rate_temp < 100){
			fw_freq_parameters[0] = ' ';
			fw_freq_parameters[1] = 48 + (rate_temp / 10);
			fw_freq_parameters[2] = '.';
			fw_freq_parameters[3] = 48 + (rate_temp % 10);
		}
		else if(rate_temp >= 100 && rate_temp < 1000){
			fw_freq_parameters[0] = 48 + (rate_temp / 100);
			rate_temp = rate_temp % 100;

			fw_freq_parameters[1] = 48 + (rate_temp / 10);
			fw_freq_parameters[2] = '.';
			fw_freq_parameters[3] = 48 + (rate_temp % 10);
		}

		fw_freq_parameters[4] = 'K';
		fw_freq_parameters[5] = 'H';
		fw_freq_parameters[6] = 'Z';

	}

}

void CK_MSP_OSD_MahPlot(uint32_t currentTime){

	static uint32_t preTime = 0;
	float delta = currentTime - preTime;
	static float mahSum = 0.0f;

	float currentADC, mahAmpere;
	int mah_temp;
	const int mAhUpdateMs = 100;

	if(delta >= mah_plot_freq_){

		preTime = currentTime;

		currentADC = (float)(CK_ADC_GetCurrentResult() * 100.0f);
		currentADC /= 100.0f; // osd multiplies with 100 for getting 2 decimal point

		// I = (Vout * 1000) / (0.5 x resistor)
		mahAmpere = (currentADC * 1000) / (CURRENT_RESISTOR * 0.5f); // Current Sens Vout = I*0.5m*105K/1K = I*52.5m

		// Add calibration multiply here
		mahAmpere *= MAH_CALIBRATION_MULTIPLIER;

		/* Ampere to milliAmpere */
		mahAmpere *= 1000;

		/* Add mA per 100ms (10Hz) to sum */
		mahAmpere /= 3600;                  // mA per second
		mahAmpere /= (1000 / mAhUpdateMs);  // mA per 100ms

		mahSum += mahAmpere; // mAh Consumed

		mah_temp = (int)mahSum;

		if(mah_temp >= 0 && mah_temp <= 9){
			mah_parameters[0] = ' ';// Blank Space
			mah_parameters[1] = ' ';// Blank Space
			mah_parameters[2] = ' ';// Blank Space
			mah_parameters[3] = 48 + mah_temp;

		}
		else if(mah_temp > 9 && mah_temp <= 99){
			mah_parameters[0] = ' ';// Blank Space
			mah_parameters[1] = ' ';// Blank Space
			mah_parameters[2] = 48 + (mah_temp / 10);
			mah_parameters[3] = 48 + (mah_temp % 10);
		}
		else if(mah_temp > 99 && mah_temp <= 999){
			mah_parameters[0] = ' ';// Blank Space
			mah_parameters[1] = 48 + (mah_temp / 100);

			mah_temp = mah_temp % 100;

			mah_parameters[2] = 48 + (mah_temp / 10);
			mah_parameters[3] = 48 + (mah_temp % 10);

		}
		else if(mah_temp > 999 && mah_temp <= 9999){
			mah_parameters[0] = 48 + (mah_temp / 1000);
			mah_temp = mah_temp % 1000;

			mah_parameters[1] = 48 + (mah_temp / 100);
			mah_temp = mah_temp % 100;

			mah_parameters[2] = 48 + (mah_temp / 10);
			mah_parameters[3] = 48 + (mah_temp % 10);
		}

		mah_parameters[4] = SYM_MAH;

	}

}

uint16_t CK_MSP_OSD_PlotCommands(uint8_t command_id, uint8_t* buffer){

	uint16_t payloadSize = 0;
	uint8_t crc = 0;

	buffer[payloadSize++] = '$';
	buffer[payloadSize++] = 'M';
	buffer[payloadSize++] = '>';

	switch(command_id){

	// I write this for osd characters
	case MSP_DP_HEARTBEAT:

		CK_MSP_WriteBufferU8(buffer, 1, payloadSize++, &crc); //  byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_DISPLAYPORT, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, MSP_DP_HEARTBEAT, payloadSize++, &crc);

		buffer[payloadSize++] = crc;

		break;

	case MSP_DP_RELEASE:

		break;

	case MSP_DP_CLEAR_SCREEN:

		CK_MSP_WriteBufferU8(buffer, 1, payloadSize++, &crc); //  byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_DISPLAYPORT, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, MSP_DP_CLEAR_SCREEN, payloadSize++, &crc);

		buffer[payloadSize++] = crc;

		break;

	case MSP_DP_DRAW_SCREEN:

		CK_MSP_WriteBufferU8(buffer, 1, payloadSize++, &crc); //  byte payload
		CK_MSP_WriteBufferU8(buffer, MSP_DISPLAYPORT, payloadSize++, &crc);
		CK_MSP_WriteBufferU8(buffer, MSP_DP_DRAW_SCREEN, payloadSize++, &crc);

		buffer[payloadSize++] = crc;


		break;

	default:

		break;
	}

	return payloadSize;

}

uint16_t CK_MSP_OSD_WriteString(uint8_t row, uint8_t col, uint8_t* buffer, const char str[], uint8_t len){

	uint16_t payloadSize = 0;
	uint8_t crc = 0;

	buffer[payloadSize++] = '$';
	buffer[payloadSize++] = 'M';
	buffer[payloadSize++] = '>';

	// payload will be updated on osd file
	CK_MSP_WriteBufferU8(buffer, len + 4, payloadSize++, &crc); //  byte payload
	CK_MSP_WriteBufferU8(buffer, MSP_DISPLAYPORT, payloadSize++, &crc);
	CK_MSP_WriteBufferU8(buffer, MSP_DP_WRITE_STRING, payloadSize++, &crc);

	CK_MSP_WriteBufferU8(buffer, row, payloadSize++, &crc);
	CK_MSP_WriteBufferU8(buffer, col, payloadSize++, &crc);

	CK_MSP_WriteBufferU8(buffer, current_font, payloadSize++, &crc); // font 0

	for(int i = 0; i < len; i++){
		CK_MSP_WriteBufferU8(buffer, str[i], payloadSize++, &crc);
	}

	buffer[payloadSize++] = crc;

	return payloadSize;

}

uint16_t CK_MSP_OSD_PacketSequence(uint8_t* buffer){

	static int sequence = 0;
	uint16_t tx_buffer_size = 0;

	if(sequence == 0){
		tx_buffer_size = CK_MSP_FormPacket(MSP_FC_VERSION, buffer);
	}
	else if(sequence == 1){
		tx_buffer_size = CK_MSP_FormPacket(MSP_FC_VARIANT, buffer);
	}
	else if(sequence == 2){
		tx_buffer_size = CK_MSP_FormPacket(MSP_STATUS, buffer);
	}
	else if(sequence == 3){
		tx_buffer_size = CK_MSP_FormPacket(MSP_BATTERY_STATE, buffer);
	}
	else if(sequence == 4){
		tx_buffer_size = CK_MSP_FormPacket(MSP_ANALOG, buffer);
	}
	else if(sequence == 5){
		tx_buffer_size = CK_MSP_OSD_PlotCommands(MSP_DP_HEARTBEAT, buffer);
	}
	else if(sequence == 6){
		tx_buffer_size = CK_MSP_OSD_PlotCommands(MSP_DP_CLEAR_SCREEN, buffer);
	}
	else if(sequence == 7){
		tx_buffer_size = CK_MSP_OSD_WriteString(timer_plot_row, timer_plot_column, buffer, timer_parameters, timer_parameters_len);
	}
	else if(sequence == 8){
		tx_buffer_size = CK_MSP_OSD_WriteString(rssi_dbm_plot_row, rssi_dbm_plot_column, buffer, rssi_dbm_parameters, rssi_dbm_parameters_len);
	}
	else if(sequence == 9){
		tx_buffer_size = CK_MSP_OSD_WriteString(rssi_lq_plot_row, rssi_lq_plot_column, buffer, rssi_lq_parameters, rssi_lq_parameters_len);
	}
	else if(sequence == 10){
		tx_buffer_size = CK_MSP_OSD_WriteString(core_temperature_plot_row, core_temperature_plot_column, buffer, core_temperature_parameters, core_temperature_parameters_len);
	}
	else if(sequence == 11){
		tx_buffer_size = CK_MSP_OSD_WriteString(current_plot_row, current_plot_column, buffer, current_parameters, current_parameters_len);
	}
	else if(sequence == 12){
		tx_buffer_size = CK_MSP_OSD_WriteString(fw_freq_plot_row, fw_freq_plot_column, buffer, fw_freq_parameters, fw_freq_parameters_len);
	}
	else if(sequence == 13){
		tx_buffer_size = CK_MSP_OSD_WriteString(mah_plot_row, mah_plot_column, buffer, mah_parameters, mah_parameters_len);
	}
	else if(sequence == 14){
		tx_buffer_size = CK_MSP_OSD_PlotCommands(MSP_DP_DRAW_SCREEN, buffer);
	}
	sequence++;
	if(sequence == 15){
		sequence = 0;
	}

	return tx_buffer_size;

}

void CK_OSD_DJI_InitLocationParameters(void){

	uint8_t osd_pos_buffer_idx = 0;
	UNUSED(osd_pos_buffer_idx);

	dji_osd.osdflags						= 0;
	dji_osd.osdflags						|= (1u << 6) | (1u << 5) | (1u << 0);
	dji_osd.video_system					= 3;
	dji_osd.units							= 0; // 0 imperial unit, else 1
	dji_osd.rssi_alarm						= 0;
	dji_osd.cap_alarm						= 0;
	dji_osd.old_timer_alarm					= 0;
	dji_osd.osd_item_count					= 80;
	dji_osd.alt_alarm						= 0;

	/*
	dji_osd.osd_rssi_value_pos 						= osd_rssi_value_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rssi_value_pos;

	dji_osd.osd_main_batt_voltage_pos 				= osd_main_batt_voltage_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_main_batt_voltage_pos;

	dji_osd.osd_crosshairs_pos 						= osd_crosshairs_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_crosshairs_pos;

	dji_osd.osd_artificial_horizon_pos 				= osd_artificial_horizon_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_artificial_horizon_pos;

	dji_osd.osd_horizon_sidebars_pos 				= osd_horizon_sidebars_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_horizon_sidebars_pos;

	dji_osd.osd_item_timer_1_pos 					= osd_item_timer_1_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_item_timer_1_pos;

	dji_osd.osd_item_timer_2_pos 					= osd_item_timer_2_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_item_timer_2_pos;

	dji_osd.osd_flymode_pos 						= osd_flymode_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_flymode_pos;

	dji_osd.osd_craft_name_pos 						= osd_craft_name_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_craft_name_pos;

	dji_osd.osd_throttle_pos_pos 					= osd_throttle_pos_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_throttle_pos_pos;

	dji_osd.osd_vtx_channel_pos 					= osd_vtx_channel_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_vtx_channel_pos;

	dji_osd.osd_current_draw_pos 					= osd_current_draw_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_current_draw_pos;

	dji_osd.osd_mah_drawn_pos 						= osd_mah_drawn_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_mah_drawn_pos;

	dji_osd.osd_gps_speed_pos 						= osd_gps_speed_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_speed_pos;

	dji_osd.osd_gps_sats_pos 						= osd_gps_sats_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_sats_pos;

	dji_osd.osd_altitude_pos 						= osd_altitude_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_altitude_pos;

	dji_osd.osd_roll_pids_pos 						= osd_roll_pids_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_roll_pids_pos;

	dji_osd.osd_pitch_pids_pos 						= osd_pitch_pids_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_pitch_pids_pos;

	dji_osd.osd_yaw_pids_pos 						= osd_yaw_pids_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_yaw_pids_pos;

	dji_osd.osd_power_pos 							= osd_power_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_power_pos;

	dji_osd.osd_pidrate_profile_pos 				= osd_pidrate_profile_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_pidrate_profile_pos;

	dji_osd.osd_warnings_pos 						= osd_warnings_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_warnings_pos;

	dji_osd.osd_avg_cell_voltage_pos 				= osd_avg_cell_voltage_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++]	= osd_avg_cell_voltage_pos;

	dji_osd.osd_gps_lon_pos 						= osd_gps_lon_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_lon_pos;

	dji_osd.osd_gps_lat_pos 						= osd_gps_lat_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_lat_pos;

	dji_osd.osd_debug_pos 							= osd_debug_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_debug_pos;

	dji_osd.osd_pitch_angle_pos 					= osd_pitch_angle_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_pitch_angle_pos;

	dji_osd.osd_roll_angle_pos 						= osd_roll_angle_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_roll_angle_pos;

	dji_osd.osd_main_batt_usage_pos 				= osd_main_batt_usage_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_main_batt_usage_pos;

	dji_osd.osd_disarmed_pos 						= osd_disarmed_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_disarmed_pos;

	dji_osd.osd_home_dir_pos 						= osd_home_dir_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_home_dir_pos;

	dji_osd.osd_home_dist_pos 						= osd_home_dist_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_home_dist_pos;

	dji_osd.osd_numerical_heading_pos 				= osd_numerical_heading_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_numerical_heading_pos;

	dji_osd.osd_numerical_vario_pos 				= osd_numerical_vario_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_numerical_vario_pos;

	dji_osd.osd_compass_bar_pos 					= osd_compass_bar_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_compass_bar_pos;

	dji_osd.osd_esc_tmp_pos 						= osd_esc_tmp_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_esc_tmp_pos;

	dji_osd.osd_esc_rpm_pos 						= osd_esc_rpm_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_esc_rpm_pos;

	dji_osd.osd_remaining_time_estimate_pos 		= osd_remaining_time_estimate_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_remaining_time_estimate_pos;

	dji_osd.osd_rtc_datetime_pos 					= osd_rtc_datetime_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rtc_datetime_pos;

	dji_osd.osd_adjustment_range_pos 				= osd_adjustment_range_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_adjustment_range_pos;

	dji_osd.osd_core_temperature_pos 				= osd_core_temperature_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_core_temperature_pos;

	dji_osd.osd_anti_gravity_pos 					= osd_anti_gravity_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_anti_gravity_pos;

	dji_osd.osd_g_force_pos 						= osd_g_force_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_g_force_pos;

	dji_osd.osd_motor_diag_pos 						= osd_motor_diag_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_motor_diag_pos;

	dji_osd.osd_log_status_pos 						= osd_log_status_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_log_status_pos;

	dji_osd.osd_flip_arrow_pos 						= osd_flip_arrow_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_flip_arrow_pos;

	dji_osd.osd_link_quality_pos 					= osd_link_quality_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_link_quality_pos;

	dji_osd.osd_flight_dist_pos 					= osd_flight_dist_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_flight_dist_pos;

	dji_osd.osd_stick_overlay_left_pos 				= osd_stick_overlay_left_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_stick_overlay_left_pos;

	dji_osd.osd_stick_overlay_right_pos 			= osd_stick_overlay_right_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_stick_overlay_right_pos;

	dji_osd.osd_display_name_pos 					= osd_display_name_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_display_name_pos;

	dji_osd.osd_esc_rpm_freq_pos 					= osd_esc_rpm_freq_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_esc_rpm_freq_pos;

	dji_osd.osd_rate_profile_name_pos 				= osd_rate_profile_name_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rate_profile_name_pos;

	dji_osd.osd_pid_profile_name_pos 				= osd_pid_profile_name_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_pid_profile_name_pos;

	dji_osd.osd_profile_name_pos 					= osd_profile_name_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_profile_name_pos;

	dji_osd.osd_rssi_dbm_value_pos 					= osd_rssi_dbm_value_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rssi_dbm_value_pos;

	dji_osd.osd_rc_channels_pos 					= osd_rc_channels_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rc_channels_pos;

	dji_osd.osd_camera_frame_pos 					= osd_camera_frame_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_camera_frame_pos;

	dji_osd.osd_effiency_pos 						= osd_effiency_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_effiency_pos;

	dji_osd.osd_total_flights_pos 					= osd_total_flights_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_total_flights_pos;

	dji_osd.osd_up_down_ref_pos 					= osd_up_down_ref_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_up_down_ref_pos;

	dji_osd.osd_tx_uplink_power_pos 				= osd_tx_uplink_power_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_tx_uplink_power_pos;

	dji_osd.osd_watt_hours_drawn_pos 				= osd_watt_hours_drawn_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_watt_hours_drawn_pos;

	dji_osd.osd_aux_value_pos 						= osd_aux_value_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_aux_value_pos;

	dji_osd.osd_ready_mode_pos 						= osd_ready_mode_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_ready_mode_pos;

	dji_osd.osd_rsnr_value_pos 						= osd_rsnr_value_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_rsnr_value_pos;

	dji_osd.osd_sys_goggle_voltage_pos 				= osd_sys_goggle_voltage_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_goggle_voltage_pos;

	dji_osd.osd_sys_vtx_voltage_pos 				= osd_sys_vtx_voltage_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_vtx_voltage_pos;

	dji_osd.osd_sys_bitrate_pos 					= osd_sys_bitrate_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_bitrate_pos;

	dji_osd.osd_sys_delay_pos 						= osd_sys_delay_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_delay_pos;

	dji_osd.osd_sys_distance_pos 					= osd_sys_distance_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_distance_pos;

	dji_osd.osd_sys_lo_pos 							= osd_sys_lo_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_lo_pos;

	dji_osd.osd_sys_goggle_dvr_pos 					= osd_sys_goggle_dvr_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_goggle_dvr_pos;

	dji_osd.osd_sys_vtx_dvr_pos 					= osd_sys_vtx_dvr_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_vtx_dvr_pos;

	dji_osd.osd_sys_warnings_pos 					= osd_sys_warnings_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_warnings_pos;

	dji_osd.osd_sys_vtx_temp_pos 					= osd_sys_vtx_temp_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_vtx_temp_pos;

	dji_osd.osd_sys_fan_speed_pos 					= osd_sys_fan_speed_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_sys_fan_speed_pos;

	dji_osd.osd_gps_lap_time_current_pos 			= osd_gps_lap_time_current_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_lap_time_current_pos;

	dji_osd.osd_gps_lap_time_previous_pos 			= osd_gps_lap_time_previous_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_lap_time_previous_pos;

	dji_osd.osd_gps_lap_time_best3_pos 				= osd_gps_lap_time_best3_pos;
	dji_osd.osd_pos_buffer[osd_pos_buffer_idx++] 	= osd_gps_lap_time_best3_pos;
	*/

	dji_osd.osd_stat_count 					= 29;
	dji_osd.osd_timer_count 				= 2;
	dji_osd.osd_warning_count 				= 16;
	dji_osd.osd_profile_count 				= 1;
	dji_osd.osdprofileindex 				= 1;
	dji_osd.overlay_radio_mode 				= 0;

	dji_osd.camera_frame_height				= 24;
	dji_osd.camera_frame_width				= 11;

}
