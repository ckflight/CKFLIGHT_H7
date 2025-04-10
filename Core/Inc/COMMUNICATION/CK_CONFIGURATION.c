
#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"
#include "COMMUNICATION/CK_PRINTER.h"

#include "FLASH/CK_FLASH.h"

#include "FLIGHT/CK_PID.h"
#include "FLIGHT/pid_init.h"
#include "FLIGHT/CK_RC.h"

#include "MOTION/CK_ACC.h"

#include "DRIVERS/CK_TIME_HAL.h"

typedef struct{

	uint8_t is_check_completed;
	uint8_t is_eeprom_configured;

	uint8_t config_buffer[EEPROM_BUFFER_SIZE];

	uint8_t term_buffer[128];
	uint16_t term_index;

	uint8_t gui_buffer[128];
	uint16_t gui_index;

	uint8_t is_term_done;
	uint8_t is_term_input_done;

	uint8_t is_gui_done;
	uint8_t is_gui_input_done;

}config_t;

config_t config = {

	.is_check_completed 	= 0,
	.is_eeprom_configured 	= 0,

	.term_index = 0,

	.is_term_done = 0,
	.is_term_input_done = 0

};

typedef enum{

	GUI_PID_DATA 	= 0x01,
	GUI_RC_DATA 	= 0x02,
	GUI_IMU_DATA 	= 0x03,
	GUI_STATUS		= 0x04,
	GUI_CONFIG		= 0x05

}GUI_PacketType;

#define GUI_PID_DATA_LEN		8

/*
 * CK_CONFIGURATION_Init is responsible of configuring eeprom with default settings for once if eeprom is erased
 * Flags indicates if eeprom is loaded with data.
 * This method will be used only once to load hard coded default parameters to the eeprom
 *
 * After first initialisation possible ways of updating the eeprom
 * 1. Configurator's update method with commands. ( command C<$ )
 * 2. MSP protocol
 * 3. In-flight adjustment
 *
 *
 * Parameter current file format:
 *
 * 128 Bytes are allocated for parameters.
 * Currently indicator flag + pid set flag and pid numbers are configured.
 *
 * 20 bytes for flags.
 *
 * Byte 20 to 43 PID profile values
 * PID: Roll(p, i, d -> 3 Bytes), Pitch(3), Yaw(3), Altitude(3), Velocity(3), Navigation(3), Level(3), Horizon(3)
 *
 * Byte 44 to 47 ACC x, y calibration data. 16 bit is stored MSB first
 *
 * Byte 48 to 70 RC parameters data. 16 bit is stored MSB first
 *
 *
 */

void CK_CONFIGURATION_Init(void){

	// Configuration uses internal memory only.
	// Read the parameters to check if it is initialised
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config.config_buffer, CONFIG_ID_BYTES, CONFIG_ID_OFFSET);

	int state = 0;

	for(int i = 0; i < CONFIG_ID_BYTES; i++){

		uint8_t current_data = config.config_buffer[i];

		switch(state){

			case 0:
				if(current_data == CONFIG_ID[0]){
					state++;
					break;
				}
				state = 0;
				break;

			case 1:
				if(current_data == CONFIG_ID[1]){
					state++;
					break;
				}
				state = 0;
				break;

			case 2:
				if(current_data == CONFIG_ID[2]){
					state++;
					break;
				}
				state = 0;
				break;

			case 3:
				if(current_data == CONFIG_ID[3]){

					config.is_eeprom_configured = 1;

					state++;

					break;
				}
				state = 0;
				break;

			case 4:

				break;


			default:
				break;
		}

	}

	// If not configured, configure eeprom and set flags
	if(!config.is_eeprom_configured){

		// Clear buffer
		for(int i = 0; i < EEPROM_BUFFER_SIZE; i++){
			config.config_buffer[i] = 0;
		}

		CK_CONFIGURATION_LoadID();

		CK_CONFIGURATION_LoadParameters();

		CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE);

		CK_USBD_StringPrintln("Flash Parameters are updated");
		CK_USBD_Transmit();

	}

}

void CK_CONFIGURATION_DecodeInputStream(uint8_t* buffer, uint16_t buffer_size){

	// CK<$_T,G is the command

	if(buffer_size != 6){
		return;
	}

	int state = 0;
	uint8_t is_terminal_config_enabled 	= 0;
	uint8_t is_gui_config_enabled 		= 0;

	for(int i = 0; i < buffer_size; i++){

		uint8_t current_data = buffer[i];

		switch(state){
			case 0:
				if(current_data == 'C'){
					state++;
					break;
				}
				state = 0;
				break;

			case 1:
				if(current_data == 'K'){
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
				if(current_data == '$'){
					state++;
					break;
				}
				state = 0;
				break;

			case 4:
				if(current_data == '_'){
					state++;
					break;
				}
				state = 0;
				break;

			case 5:
				if(current_data == 'T'){
					is_terminal_config_enabled = 1;
					break;
				}
				if(current_data == 'G'){
					is_gui_config_enabled = 1;
					break;
				}
				state = 0;
				break;


			default:
				break;

		}
	}

	if(is_terminal_config_enabled) CK_CONFIGURATION_TerminalCMD();
	if(is_gui_config_enabled) CK_CONFIGURATION_GuiCMD();


}

void CK_CONFIGURATION_GuiCMD(void){

	uint8_t rx_data;
	uint8_t state = 0;

	/*
	 * Gui will read input data and save that data to eeprom.
	 * Then it will start listening new data until exit button is clicked gui
	 */
	while(!config.is_gui_done){

		switch(state){

		case 0:

			// Read user inputs
			while(CK_USBD_ReadData(&rx_data) == 1){

				config.term_buffer[config.term_index++] = rx_data;
			}

			// Define the byte number to be received
			// With header it sends around 69 bytes
			if(config.term_index){

				// Echo received data to gui which will print on terminal
				CK_USBD_StringPrint("Received data:");
				for(int i = 0; i < config.term_index; i++){
					CK_USBD_IntPrint(config.term_buffer[i]);
				}
				CK_USBD_StringPrintln("");
				CK_USBD_Transmit();

				state = 1;
			}

			break;

		case 1:

			// Decode data
			CK_CONFIGURATION_DecodeGUIData();
			state = 2;

			break;

		case 2:

			break;

		default:
			break;

		}

	}

}

bool CK_CONFIGURATION_DecodeGUIData(void){

	// Data structure: 2 byte packet header "CK" + 1 byte packet type code "PID, RC, etc" + 1 byte payload lenght + payload + 1 byte CRC
	// Example PID Data : CK + 0x01 + 0x08 + 8 byte data + CRC

	uint8_t state = 0;
	bool is_packet_valid = false;
	bool is_done = false;
	uint8_t crc = CK_CONFIGURATION_CalculateCRC();

	if(crc == config.term_buffer[config.term_index - 1]){
		while(!is_done){

			switch(state){
			case 0:

				if(config.term_buffer[0] == 'C' && config.term_buffer[1] == 'K'){
					state = 1;
				}
				else{
					is_done = true;
					is_packet_valid = false;
				}
				break;

			case 1:

				// Check packet type and payload len
				if(config.term_buffer[2] == GUI_PID_DATA && config.term_buffer[3] == GUI_PID_DATA_LEN){

					pidProfile.simplified_master_multiplier = config.term_buffer[4];
					pidProfile.simplified_pi_gain 			= config.term_buffer[5];
					pidProfile.simplified_feedforward_gain 	= config.term_buffer[6];
					pidProfile.simplified_roll_pitch_ratio 	= config.term_buffer[7];
					pidProfile.simplified_i_gain 			= config.term_buffer[8];
					pidProfile.simplified_d_gain 			= config.term_buffer[9];
					pidProfile.simplified_d_max_gain 		= config.term_buffer[10];
					pidProfile.simplified_pitch_pi_gain 	= config.term_buffer[11];

					state = 2;

				}
				else if(config.term_buffer[2] == GUI_RC_DATA){

					state = 2;
				}
				else if(config.term_buffer[2] == GUI_IMU_DATA){

					state = 2;
				}
				else if(config.term_buffer[2] == GUI_STATUS){

					state = 2;
				}
				else if(config.term_buffer[2] == GUI_CONFIG){

					state = 2;
				}
				else{
					is_done = true;
					is_packet_valid = false;
				}

			case 2:

				// Call save to eeprom here


				is_done = true;
				is_packet_valid = true;

				break;

			default:
				break;


			}
		}
	}
	return is_packet_valid;

}

uint8_t CK_CONFIGURATION_CalculateCRC(void){

	uint8_t crc = 0x00;
	for(int i = 0; i < config.term_index - 1; i++){
		crc += config.term_buffer[i];
	}

	return (crc & 0xFF); // return mod 256 of crc

}

void CK_CONFIGURATION_TerminalCMD(void){

	uint8_t rx_data;

	// Configuration selection format is:
	// For example to configure PID Level P,I,D to 70 10 10 type -> 1c10,10,10,
	// For example to configure TPA_Rate 40 type -> 1j40,
	// For example to configure RC Rates type -> 3a120,120,120,

	while(!config.is_term_done){

		CK_USBD_StringPrintln("1. PID Configuration");
		CK_USBD_StringPrint(" 1a. Roll, 1b.Pitch 1c.Yaw, 1d.Althold, 1e.Velocity, ");
		CK_USBD_StringPrint("1f.Navigation, 1g.Level, 1h.Horizon, 1i.Dmin, 1j.FeedForward, ");
		CK_USBD_StringPrintln("1k.TPA_Breakpoint, 1l.TPA_Rate");

		CK_USBD_StringPrintln("2. RC Configuration");
		CK_USBD_StringPrintln("	2a. RC_Idle_Drift, 2b.RC_Idle_Trim, 2c.Yaw_Deadband");

		CK_USBD_StringPrintln("3. Accelerometer Calibration");

		CK_USBD_StringPrintln("4. RC Rates Configuration");
		CK_USBD_StringPrintln("	4a. RC_Rate, 4b.RC_Expo, 4c.Rates");

		CK_USBD_StringPrintln("5. Exit");
		CK_USBD_StringPrintln("");
		CK_USBD_Transmit();

		while(!config.is_term_input_done){

			while(CK_USBD_ReadData(&rx_data) == 1){

				config.term_buffer[config.term_index++] = rx_data;
			}

			if(config.term_index){

				uint8_t resp = CK_CONFIGURATION_ConfigureParameters();

				config.term_index = 0;
				config.is_term_input_done = 1;

				if(resp){

					CK_USBD_StringPrintln("Parameters are updated");
					CK_USBD_StringPrintln("");
					CK_USBD_Transmit();

				}
				else{
					CK_USBD_StringPrintln("Incorrect!");
					CK_USBD_StringPrintln("");
					CK_USBD_Transmit();
				}

			}

			CK_TIME_DelayMilliSec(1);
		}

		config.is_term_input_done = 0;
	}

	config.is_term_done = 0;

	CK_USBD_StringPrintln("Configuration is Done. Nice Fligths!");
	CK_USBD_StringPrintln("Restarting the system.");
	CK_USBD_Transmit();
	CK_TIME_DelayMilliSec(10);
	NVIC_SystemReset();

}

uint8_t CK_CONFIGURATION_ConfigureParameters(void){

	uint8_t resp = 0;
	uint8_t menu_selection = config.term_buffer[0];

	uint16_t parameters_buffer[16];
	uint8_t parameters_index = 0;

	//******* MENU OPTION 1 **********
	if(menu_selection == '1'){

		// Min 4 max 14 characters
		if(config.term_index >= 4 && config.term_index <= 14){

			uint8_t pid_sub_menu = config.term_buffer[1];
			uint8_t pid_axis = pid_sub_menu - 97; // 'a' = 97
			char range1 = 'a';
			char range2 = 'l';
			if(pid_sub_menu >= range1 && pid_sub_menu <= range2){

				int start_index = 2;
				int end_index = 0;
				for(int i = 2; i < config.term_index; i++){

					if(config.term_buffer[i] == ','){

						end_index = i - 1;
						uint16_t num = CK_CONFIGURATION_AsciiToNumber(config.term_buffer, start_index, end_index);
						parameters_buffer[parameters_index++] = num;

						start_index = i + 1;

					}
				}

				CK_CONFIGURATION_SavePIDs(pid_axis, parameters_buffer, parameters_index);
				parameters_index = 0;

				if(pid_axis <= 9){

					CK_USBD_StringPrint("Axis: ");CK_USBD_IntPrint(pid_axis);
					CK_USBD_StringPrint(" P: ");CK_USBD_IntPrint(parameters_buffer[0]);
					CK_USBD_StringPrint(" I: ");CK_USBD_IntPrint(parameters_buffer[1]);
					CK_USBD_StringPrint(" D: ");CK_USBD_IntPrint(parameters_buffer[2]);
					CK_USBD_StringPrintln("");
					CK_USBD_Transmit();
				}
				else{
					CK_USBD_StringPrint("Axis: ");CK_USBD_IntPrint(pid_axis);
					CK_USBD_StringPrint(" , ");CK_USBD_IntPrint(parameters_buffer[0]);
					CK_USBD_StringPrintln("");
					CK_USBD_Transmit();

				}

				resp = 1;

			}
			else{
				resp = 0;
			}
		}
		else{
			resp = 0;
		}
	}

	//******* MENU OPTION 2 **********
	else if(menu_selection == '2'){

		// Min 4 max 6 characters
		if(config.term_index >= 4 && config.term_index <= 7){

			uint16_t num = 0;
			uint8_t rc_sub_menu = config.term_buffer[1];
			int option = rc_sub_menu - 97 + 1; // 'a' = 97
			char range1 = 'a';
			char range2 = 'c';
			if(rc_sub_menu >= range1 && rc_sub_menu <= range2){

				int start_index = 2;
				int end_index = 0;
				for(int i = 2; i < config.term_index; i++){

					if(config.term_buffer[i] == ','){

						end_index = i - 1;
						num = CK_CONFIGURATION_AsciiToNumber(config.term_buffer, start_index, end_index);

						start_index = i + 1;

					}
				}

				CK_CONFIGURATION_SaveRC(option, num);

				CK_USBD_StringPrint("Option: ");CK_USBD_IntPrint(option);
				CK_USBD_StringPrint(" , ");CK_USBD_IntPrint(num);
				CK_USBD_StringPrintln("");
				CK_USBD_Transmit();

				resp = 1;

			}
			else{
				resp = 0;
			}
		}
		else{
			resp = 0;
		}
	}

	//******* MENU OPTION 3 **********
	else if(menu_selection == '3'){

	    CK_PRINTER_PrintlnString("ACC Calibration");

	    int16_t acc_buffer[3];
		CK_ACC_PerformCalibration(acc_buffer);

		CK_CONFIGURATION_SaveAccCalibration(acc_buffer);
		CK_PRINTER_PrintlnString("");
		CK_PRINTER_PrintString("Acc X Axis:");CK_PRINTER_PrintlnInt(acc_buffer[X]);
		CK_PRINTER_PrintString("Acc Y Axis:");CK_PRINTER_PrintlnInt(acc_buffer[Y]);
		CK_PRINTER_PrintString("Acc Z Axis:");CK_PRINTER_PrintlnInt(acc_buffer[Z]);

		resp = 1;
	}

	//******* MENU OPTION 4 **********
	else if(menu_selection == '4'){

		// 4a. RC_Rate, 4b.RC_Expo, 4c.Rates
		// Min 8 max 14 characters
		if(config.term_index >= 8 && config.term_index <= 14){

			uint8_t rates_sub_menu = config.term_buffer[1];
			int option = rates_sub_menu - 97 + 1; // 'a' = 97
			char range1 = 'a';
			char range2 = 'h';
			if(rates_sub_menu >= range1 && rates_sub_menu <= range2){

				int start_index = 2;
				int end_index = 0;
				for(int i = 2; i < config.term_index; i++){

					if(config.term_buffer[i] == ','){

						end_index = i - 1;
						uint8_t num = (uint8_t)CK_CONFIGURATION_AsciiToNumber(config.term_buffer, start_index, end_index);
						parameters_buffer[parameters_index++] = num;

						start_index = i + 1;

					}
				}

				CK_CONFIGURATION_SaveRates(option, parameters_buffer, parameters_index);
				parameters_index = 0;

				CK_USBD_StringPrint("Option: ");CK_USBD_IntPrint(option);
				CK_USBD_StringPrint(" Roll: ");CK_USBD_IntPrint(parameters_buffer[0]);
				CK_USBD_StringPrint(" Pitch: ");CK_USBD_IntPrint(parameters_buffer[1]);
				CK_USBD_StringPrint(" Yaw: ");CK_USBD_IntPrint(parameters_buffer[2]);
				CK_USBD_StringPrintln("");
				CK_USBD_Transmit();

				resp = 1;

			}
			else{
				resp = 0;
			}
		}
		else{
			resp = 0;
		}

	}

	//******* MENU OPTION 5 **********
	else if(menu_selection == '5'){

		resp = 1;

		config.is_term_done = 1;

	}
	else{


	}

	return resp;

}

uint16_t CK_CONFIGURATION_AsciiToNumber(uint8_t* buffer, int start, int end){

	int digits = end - start + 1;
	uint16_t num = 0;

	if(digits == 4){
		num = 1000 * (int)(buffer[start] - 48);
		num += 100 * (int)(buffer[start+1] - 48);
		num += 10  * (int)(buffer[start+2] - 48);
		num += (int)(buffer[end] - 48);
	}
	if(digits == 3){
		num = 100 * (int)(buffer[start] - 48);
		num += 10 * (int)(buffer[start+1] - 48);
		num += (int)(buffer[end] - 48);
	}
	if(digits == 2){
		num += 10 * (int)(buffer[start] - 48);
		num += (int)(buffer[end] - 48);
	}
	if(digits == 1){
		num += (int)(buffer[end] - 48);
	}

	return num;


}

void CK_CONFIGURATION_LoadID(void){

	// Load id to buffer
	for(int i = 0; i < CONFIG_ID_BYTES; i++){
		config.config_buffer[i] = CONFIG_ID[i];
	}
}

// This method loads default parameters to buffer
void CK_CONFIGURATION_LoadParameters(void){

	uint8_t buffer[EEPROM_BUFFER_SIZE];

	//Copy each parameter from its library to buffer

	// PID Parameters
	/*
	 * Loads roll pitch yaw's p,i,d,ff,dmax
	 * and tpa breakpoint rate
	 */
	uint16_t buffer_size = pidGetDefaultProfile(buffer);

	// Load PID parameters to config buffer's related index
	for(int i = 0; i < buffer_size; i++){
		config.config_buffer[i + CONFIG_PID_OFFSET] = buffer[i];
	}

	// ACC default parameters are zero
	for(int i = 0; i < CONFIG_ACC_BYTES; i++){
		config.config_buffer[CONFIG_ACC_OFFSET + i] = 0;
	}

	// RC Parameters
	buffer_size = CK_RC_GetDefaultParameters(buffer);

	// Load rc to config buffer's related index
	for(int i = 0; i < buffer_size; i++){
		config.config_buffer[i + CONFIG_RC_OFFSET] = buffer[i];
	}


}

// This method saves new PID parameters to flash memory
void CK_CONFIGURATION_SavePIDs(int axis, uint16_t* pid_buffer, int pid_buffer_size){

	// Read the content of flash first
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	// Update axis parameters on its buffer location

	// PID parameters
	if(axis <= 9){
		for(int i = 0; i < pid_buffer_size; i++){
			config.config_buffer[CONFIG_PID_OFFSET + (axis * PID_ARRAY_COLUMN) + i] = pid_buffer[i];
		}
	}

	// TPA parameters
	else{
		if(axis == 10){
			config.config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN)] = (pid_buffer[0] >> 8) & 0xFF;
			config.config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN) + 1] = pid_buffer[0] & 0xFF;
		}
		if(axis == 11){
			config.config_buffer[CONFIG_PID_OFFSET + (PID_ARRAY_ROW * PID_ARRAY_COLUMN) + 2] = (uint8_t)pid_buffer[0];
		}

	}



	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE);

}

// This method saves acc calibration values to flash memory
void CK_CONFIGURATION_SaveAccCalibration(int16_t* acc_buffer){

	// Read the content of flash first
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	// Update axis parameters on its buffer location
	config.config_buffer[CONFIG_ACC_OFFSET] 	= (acc_buffer[0] >> 8) & 0xFF;
	config.config_buffer[CONFIG_ACC_OFFSET + 1] =  acc_buffer[0] & 0xFF;
	config.config_buffer[CONFIG_ACC_OFFSET + 2] = (acc_buffer[1] >> 8) & 0xFF;
	config.config_buffer[CONFIG_ACC_OFFSET + 3] =  acc_buffer[1] & 0xFF;
	config.config_buffer[CONFIG_ACC_OFFSET + 4] = (acc_buffer[2] >> 8) & 0xFF;
	config.config_buffer[CONFIG_ACC_OFFSET + 5] =  acc_buffer[2] & 0xFF;

	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE);

}

// This method saves new rc values to flash memory
void CK_CONFIGURATION_SaveRC(int option, uint16_t num){

	// Read the content of flash first
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	// Update axis parameters on its buffer location
	if(option == 1){
		config.config_buffer[CONFIG_RC_OFFSET] = (uint8_t)num;
	}
	if(option == 2){
		config.config_buffer[CONFIG_RC_OFFSET + 1] = (uint8_t)num;
	}
	if(option == 3){
		config.config_buffer[CONFIG_RC_OFFSET + 2] = (uint8_t)num;
	}

	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE);

}

// This method saves new rates values to flash memory
void CK_CONFIGURATION_SaveRates(int option, uint16_t* rate_buffer, int rate_buffer_size){

	// Read the content of flash first
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	// Update axis parameters on its buffer location
	// Rates start from 14th index of rc bytes
	for(int i = 0; i < rate_buffer_size; i++){
		config.config_buffer[CONFIG_RC_OFFSET + 11 + ((option-1) * 3) + i] = rate_buffer[i];
	}

	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, config.config_buffer, EEPROM_BUFFER_SIZE);

}



























