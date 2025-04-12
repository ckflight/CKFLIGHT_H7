
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"
#include "COMMUNICATION/CK_BLHELIPASS.h"
#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/CK_CONFIGURATION.h"

#include "DRIVERS/CK_ADC.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_MICROCARD.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_SBUS.h"
#include "FLIGHT/CK_ALTITUDE.h"
#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_NAVIGATION.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_ESC.h"
#include "FLIGHT/flight_monitor.h"

#include "MOTION/CK_IMU.h"
#include "MOTION/CK_BNO055.h"
#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_BAROMETER.h"
#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "FLASH/CK_FLASH.h"

#include "OSD/CK_OSD.h"
#include "OSD/CK_MAX7456.h"

int printerCounter = 0;
int loopTime;

char print_cmd = '.';// Default value to not run all if else statements

bool is_printer_motor_mode_enabled = false;

void CK_PRINTER_Init(uint32_t mainT){

	CK_PRINTER_PrintString("Firmware: CKFLIGHT ");
	CK_PRINTER_PrintInt(CURRENT_VERSION_MAJOR);
	CK_PRINTER_PrintString(".");
	CK_PRINTER_PrintInt(CURRENT_VERSION_MINOR);
	CK_PRINTER_PrintlnString(TARGET_MCU);

	CK_PRINTER_PrintString("Git Hash: ");
	CK_PRINTER_PrintlnString(COMMIT_HASH);

	CK_PRINTER_PrintlnString("Hardware: ");
	CK_PRINTER_PrintlnString(TARGET_BOARD);

	CK_PRINTER_PrintlnString("TARGET_MAIN_TIME US");
	CK_PRINTER_PrintlnInt(TARGET_MAIN_TIME_US);
	CK_PRINTER_PrintlnString("TARGET_GYRO_TIME US");
	CK_PRINTER_PrintlnInt(TARGET_GYRO_TIME_US);
	CK_PRINTER_PrintlnString("TARGET_ACC_TIME US");
	CK_PRINTER_PrintlnInt(TARGET_ACC_TIME_US);
	CK_PRINTER_PrintlnString("TARGET_FLASH");
	CK_PRINTER_PrintlnInt(TARGET_FLASH);

	if(card.is_Initialized){
		CK_PRINTER_PrintlnString("MicroSDCard is initialized");
	}
	else{
		CK_PRINTER_PrintlnString("MicroSDCard is not initialized:");
	}


	#if (GYRO1_SPI_|| GYRO2_SPI_) && (ACC1_SPI_ || ACC2_SPI_ || ACC_I2C_) && !(MAG_SPI_ || MAG_I2C_)
	CK_PRINTER_PrintlnString("IMU Source: GYRO + ACC");
	#else
		#if BNO055_
		CK_PRINTER_PrintlnString("Euler Source: BNO055");
	    #else
    	CK_PRINTER_PrintlnString("IMU Source: GYRO + ACC + MAG");
		#endif
	#endif

    CK_PRINTER_Transfer();

	loopTime = mainT;

	is_printer_motor_mode_enabled = false;
}

void CK_PRINTER_Update(CK_PRINT_TIMEx print_freq, uint32_t compT){

	if(print_cmd == '.'){
		return;
	}
	else if(print_cmd == '1' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintGyroADCRaw(print_freq, compT);
	}
	else if(print_cmd == '2' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintGyroADCZero(print_freq, compT);
	}
	else if(print_cmd == '3' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintGyroADCf(print_freq, compT);
	}
	else if(print_cmd == '4' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintAccADCRaw(print_freq, compT);
	}
	else if(print_cmd == '5' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintAccADCZero(print_freq, compT);
	}
	else if(print_cmd == '6' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintAccADCf(print_freq, compT);
	}
	else if(print_cmd == '7' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMagADCRaw(EVERY_50MS, compT);
	}
	else if(print_cmd == '8' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMagHardIron(EVERY_50MS, compT);
	}
	else if(print_cmd == '9' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMagADCf(EVERY_50MS, compT);
	}
	else if(print_cmd == '0' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintIMUAngles(EVERY_50MS, compT);
	}
	else if(print_cmd == '*' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintRCSetpoint(EVERY_50MS, compT);
	}
	else if(print_cmd == '-' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintRCData(EVERY_50MS, compT);
	}
	else if(print_cmd == 'q' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintRCCommand(EVERY_50MS, compT);
	}
	else if(print_cmd == 'w' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintFlags(EVERY_100MS, compT);
	}
	else if(print_cmd == 'b' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintAltitudeHold(EVERY_100MS, compT);
	}
	else if(print_cmd == 'l' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintLoopTime(compT);
	}
	else if(print_cmd == 'd' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintDebugTimes();
    }
	else if(print_cmd == 'c' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMotionCalibration(EVERY_50MS);
	}
	else if(print_cmd == 'j' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMotorFinalResults(print_freq, compT);
	}
	else if(print_cmd == 'k' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintMotorMixResults(print_freq, compT);
	}
	else if(print_cmd == 'g' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintGPS(EVERY_100MS, compT);
	}
	else if(print_cmd == 'n' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintNavigation(EVERY_100MS, compT);
	}
	else if(print_cmd == 's' && is_printer_motor_mode_enabled == false){
	    CK_GPS_SaveDestinationLocationAndStartHeight();
		CK_PRINTER_PrintlnString("Current location is saved as Destination");
		CK_PRINTER_PrintString("lat:");CK_PRINTER_PrintInt(gps.destination_lat);
		CK_PRINTER_PrintString("   lon:");CK_PRINTER_PrintlnInt(gps.destination_lon);
		CK_PRINTER_PrintString("           ");
		print_cmd = '.';// To not enter here again unless typed
	}
	else if(print_cmd == 'a' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintlnString("Accelerometer Calibration. Place on a flat surface");
		int16_t acc_buffer[3];
		CK_ACC_PerformCalibration(acc_buffer);
		CK_PRINTER_PrintlnString("Accelerometer is calibrated");
		CK_CONFIGURATION_SaveAccCalibration(acc_buffer);
		CK_PRINTER_PrintlnString("Accelerometer calibration parameters are flashed");
		print_cmd = '.';// To not enter here again unless typed
	}


	else if(print_cmd == 'm' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintlnString("Magnetometer Calibration");
		CK_MAGNETO_PerformCalibration();
		CK_PRINTER_PrintlnString("Magnetometer is calibrated");
		print_cmd = '.';// To not enter here again unless typed
	}
	else if(print_cmd == 'z' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintlnString("BNO055 Calibration");
		CK_BNO055_Calibrate();
		CK_PRINTER_PrintlnString("BNO055 is calibrated");
		print_cmd = '.';// To not enter here again unless typed
	}
	else if(print_cmd == 'p' && is_printer_motor_mode_enabled == false){
	    CK_PRINTER_PrintlnString("ESC BLHELI PASSTHROUG INITIALIZED");
		CK_BLHELIPASS_Init();
		print_cmd = '.';// To not enter here again unless typed
	}
	else if(print_cmd == 'h' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintPIDDefault();
		CK_PRINTER_PrintPID();
		CK_PRINTER_PrintPIDSliders();
		CK_PRINTER_PrintRC();
		CK_PRINTER_Transfer();
		print_cmd = '.';// To not enter here again unless typed
	}
	else if(print_cmd == 'r' && is_printer_motor_mode_enabled == false){//restart the system
		NVIC_SystemReset();
	}
	else if(print_cmd == 'i' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintADC(EVERY_250MS, compT);
	}
	else if(print_cmd == 'o' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintlnString("OSD CHAR UPDATE IS INITIALIZED");
		if(CK_MAX7456_CharUpdate()){
			CK_PRINTER_PrintlnString("OSD CHAR IS UPDATED.");
		}
		else{
			CK_PRINTER_PrintlnString("OSD CHAR UPDATE IS FAILED.");
		}

		print_cmd = '.';
	}
	else if(print_cmd == 'O' && is_printer_motor_mode_enabled == false){
		CK_MAX7456_ShowFonts();
	}
	else if(print_cmd == 'Q' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintRXRefrashRate(EVERY_10MS);
	}
	else if(print_cmd == 'A' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintFeedforward(EVERY_10MS);
	}
	else if(print_cmd == 'M' && is_printer_motor_mode_enabled == false){
		CK_PRINTER_PrintlnString("MOTOR TEST MODE, PROPS OFF!!");
		print_cmd = '.';
		is_printer_motor_mode_enabled = true;
	}
	else if(print_cmd == '1' && is_printer_motor_mode_enabled == true){
		CK_PRINTER_PrintlnString("MOTOR 1 is selected");
		CK_ESC_MOTOR_TEST_MODE_Enable(1);
		print_cmd = '.';
	}
	else if(print_cmd == '2' && is_printer_motor_mode_enabled == true){
		CK_PRINTER_PrintlnString("MOTOR 2 is selected");
		CK_ESC_MOTOR_TEST_MODE_Enable(2);
		print_cmd = '.';
	}
	else if(print_cmd == '3' && is_printer_motor_mode_enabled == true){
		CK_PRINTER_PrintlnString("MOTOR 3 is selected");
		CK_ESC_MOTOR_TEST_MODE_Enable(3);
		print_cmd = '.';
	}
	else if(print_cmd == '4' && is_printer_motor_mode_enabled == true){
		CK_PRINTER_PrintlnString("MOTOR 4 is selected");
		CK_ESC_MOTOR_TEST_MODE_Enable(4);
		print_cmd = '.';
	}
	else if(print_cmd == 'E' && is_printer_motor_mode_enabled == true){
		CK_PRINTER_PrintlnString("MOTOR TEST MODE IS DONE");
		CK_ESC_MOTOR_TEST_MODE_Disable();
		is_printer_motor_mode_enabled = false;
		print_cmd = '.';
	}
	else{
		return;
	}
}

void CK_PRINTER_DecodeInputStream(uint8_t* buffer, uint16_t size){

	if(size > 1){
		return;
	}
	// first byte is command and rest is data
	print_cmd = buffer[0];

}

// One Transfer sometimes not enough to print.
void CK_PRINTER_Transfer(void){

	for(int i = 0; i < 5; i++){
		CK_USBD_Transmit();
		CK_TIME_DelayMilliSec(1);
	}

}

void CK_PRINTER_PrintString(const char str[]){

	CK_USBD_StringPrint(str);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintlnString(const char str[]){

	CK_USBD_StringPrintln(str);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintInt(int32_t num){

	CK_USBD_IntPrint(num);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintlnInt(int32_t num){

	CK_USBD_IntPrintln(num);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintFloat(float num){

	CK_USBD_FloatPrint(num);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintlnFloat(float num){

	CK_USBD_FloatPrintln(num);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintFloatDecimal(float num, int dec){

	//CK_USBD_FloatDecimalPrint(num, dec);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintlnFloatDecimal(float num, int dec){

	//CK_USB_FloatDecimalPrintln(num, dec);

	CK_USBD_Transmit();
}

void CK_PRINTER_PrintADC(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("ADC_LIPO:    		");CK_USBD_FloatPrintln(CK_ADC_GetLipoResult());
		CK_USBD_StringPrint("LIPO VOLT:    		");CK_USBD_FloatPrintln(CK_ADC_GetLipoResult() * VOLT_CALIBRATION_MULTIPLIER);
		CK_USBD_StringPrint("ADC_CURRENT: 		");CK_USBD_FloatPrintln(CK_ADC_GetCurrentResult());
		CK_USBD_StringPrint("ADC_TEMPERATUR: 	");CK_USBD_FloatPrintln(CK_ADC_GetTemperatureResult());
		CK_USBD_StringPrint("SENSOR_TEMPERATUR: ");CK_USBD_FloatPrintln(monitor_get_gyroacc_temp());
		CK_USBD_StringPrintln("");

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintPID(void){
    CK_USBD_StringPrintln("");
    CK_USBD_StringPrintln("PID VALUES:");
    CK_USBD_StringPrintln("-------------------------------------------------");
    CK_USBD_StringPrintln("       P       I       D      FF     Dmax");

    CK_USBD_StringPrint("ROLL:  ");
    CK_USBD_IntPrint(pidProfile.pid[FD_ROLL].P); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_ROLL].I); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_ROLL].D); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_ROLL].F); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.d_max[FD_ROLL]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("PITCH: ");
    CK_USBD_IntPrint(pidProfile.pid[FD_PITCH].P); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_PITCH].I); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_PITCH].D); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_PITCH].F); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.d_max[FD_PITCH]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("YAW:   ");
    CK_USBD_IntPrint(pidProfile.pid[FD_YAW].P); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_YAW].I); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_YAW].D); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.pid[FD_YAW].F); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.d_max[FD_YAW]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("TPA_POINT: "); CK_USBD_IntPrintln(pidProfile.tpa_breakpoint);
    CK_USBD_StringPrint("TPA_RATE:  "); CK_USBD_IntPrintln(pidProfile.tpa_rate);

    CK_USBD_Transmit();
}

void CK_PRINTER_PrintPIDSliders(void){
    CK_USBD_StringPrintln("");
    CK_USBD_StringPrintln("PID SLIDER VALUES:");
    CK_USBD_StringPrintln("-------------------------------------------------");
    CK_USBD_StringPrintln("MM\tPI\tFF\tRPR\tI\tD\tDMAX\tPPI");

    CK_USBD_IntPrint(pidProfile.simplified_master_multiplier);	CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_pi_gain);			CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_feedforward_gain);	CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_roll_pitch_ratio);	CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_i_gain);				CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_d_gain);				CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_d_max_gain);			CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(pidProfile.simplified_pitch_pi_gain);		CK_USBD_StringPrintln("");

    CK_USBD_Transmit();

}

void CK_PRINTER_PrintPIDDefault(void){
    CK_USBD_StringPrintln("");
    CK_USBD_StringPrintln("PID DEFAULT VALUES:");
    CK_USBD_StringPrintln("-------------------------------------------------");
    CK_USBD_StringPrintln("       P       I       D      FF     Dmax");

    CK_USBD_StringPrint("ROLL:  ");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_ROLL][PID_P]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_ROLL][PID_I]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_ROLL][PID_D]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_ROLL][PID_FF]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_ROLL][PID_Dmax]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("PITCH: ");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_PITCH][PID_P]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_PITCH][PID_I]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_PITCH][PID_D]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_PITCH][PID_FF]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_PITCH][PID_Dmax]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("YAW:   ");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_YAW][PID_P]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_YAW][PID_I]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_YAW][PID_D]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_YAW][PID_FF]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(PID_SELECTED_PROFILE[FD_YAW][PID_Dmax]); CK_USBD_StringPrintln("");

    CK_USBD_StringPrint("TPA_POINT: "); CK_USBD_IntPrintln(pidProfile.tpa_breakpoint);
    CK_USBD_StringPrint("TPA_RATE:  "); CK_USBD_IntPrintln(pidProfile.tpa_rate);

    CK_USBD_Transmit();
}

void CK_PRINTER_PrintRC(void){
	CK_USBD_StringPrintln("");
    CK_USBD_StringPrintln("RC SETTINGS:");
    CK_USBD_StringPrintln("--------------------------------------");

    CK_USBD_StringPrint("ROLL PITCH DEADBAND: "); CK_USBD_IntPrintln(rc_config.deadband);
    CK_USBD_StringPrint("YAW_DEADBAND:        "); CK_USBD_IntPrintln(rc_config.yaw_deadband);

    CK_USBD_StringPrint("RC_RATE:    ");
    CK_USBD_IntPrint(rc_config.rcRates[FD_ROLL]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(rc_config.rcRates[FD_PITCH]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrintln(rc_config.rcRates[FD_YAW]);

    CK_USBD_StringPrint("RC_EXPO:    ");
    CK_USBD_IntPrint(rc_config.rcExpo[FD_ROLL]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(rc_config.rcExpo[FD_PITCH]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrintln(rc_config.rcExpo[FD_YAW]);

    CK_USBD_StringPrint("RATES:      ");
    CK_USBD_IntPrint(rc_config.rates[FD_ROLL]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrint(rc_config.rates[FD_PITCH]); CK_USBD_StringPrint("\t");
    CK_USBD_IntPrintln(rc_config.rates[FD_YAW]); CK_USBD_StringPrintln("");

    CK_USBD_Transmit();
}

void CK_PRINTER_PrintNavigation(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("GPSHeadingSetpoint: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetGPSHeadingSetpoint());

		CK_USBD_StringPrint(" GPSHeadingError: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetGPSHeadingError());

		CK_USBD_StringPrint(" GPSHeadingCorrection: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetGPSHeadingCorrection());

		CK_USBD_StringPrint(" GPSYawCommand: ");CK_USBD_IntPrintln(getRCCommand(YAW));


		CK_USBD_StringPrint(" MAGHeadingSetpoint: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetMAGHeadingSetpoint());

		CK_USBD_StringPrint(" MAGHeadingError: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetMAGHeadingError());

		CK_USBD_StringPrint(" MAGHeadingCorrection: ");CK_USBD_FloatPrint(CK_NAVIGATION_GetMAGHeadingCorrection());

		CK_USBD_StringPrint(" MAGYawCommand: ");CK_USBD_IntPrintln(getRCCommand(YAW));

		CK_USBD_StringPrint("   T:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintGPS(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("Cur.Lat: ");CK_USBD_IntPrintln(gps.current_lat);

		CK_USBD_StringPrint("Cur.Long: ");CK_USBD_IntPrintln(gps.current_lon);

		CK_USBD_StringPrint("Distance to Dest.: ");CK_USBD_IntPrint(gps.distanceToDestination);CK_USBD_StringPrintln(" cm");

		CK_USBD_StringPrint("Heading to Dest.: ");CK_USBD_IntPrint(gps.headingToDestination/100);CK_USBD_StringPrintln(" deg");

		CK_USBD_StringPrint("Heading of motion: ");CK_USBD_IntPrint(gps.groundCourse);CK_USBD_StringPrintln(" deg");

		CK_USBD_StringPrint("Gspeed: ");CK_USBD_IntPrint(gps.groundSpeed);CK_USBD_StringPrintln(" cm/s");

		CK_USBD_StringPrint("HeightSeaLevel: ");CK_USBD_IntPrint(gps.current_heightSeaLevel);CK_USBD_StringPrintln(" cm");

		CK_USBD_StringPrint("HeightGround: ");CK_USBD_IntPrint(gps.onGround_heightSeaLevel);CK_USBD_StringPrintln(" cm");

		CK_USBD_StringPrint("NumSat/Fix: ");CK_USBD_IntPrint(gps.numOfSattelite);CK_USBD_StringPrint("/");CK_USBD_IntPrintln(gps.satteliteFix);

		CK_USBD_StringPrintln(" ");

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintMotorFinalResults(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("MOTOR FINAL  ");CK_USBD_IntPrint(CK_MIXER_GetMotorFinalResult(1));

		CK_USBD_StringPrint("\t");CK_USBD_IntPrint(CK_MIXER_GetMotorFinalResult(2));

		CK_USBD_StringPrint("\t");CK_USBD_IntPrint(CK_MIXER_GetMotorFinalResult(3));

		CK_USBD_StringPrint("\t");CK_USBD_IntPrint(CK_MIXER_GetMotorFinalResult(4));

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintMotorMixResults(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("MOTOR MIX  ");CK_USBD_FloatPrint(CK_MIXER_GetMotorMixResult(1));

		CK_USBD_StringPrint("\t");CK_USBD_FloatPrint(CK_MIXER_GetMotorMixResult(2));

		CK_USBD_StringPrint("\t");CK_USBD_FloatPrint(CK_MIXER_GetMotorMixResult(3));

		CK_USBD_StringPrint("\t");CK_USBD_FloatPrint(CK_MIXER_GetMotorMixResult(4));

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintFlags(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("FS:");CK_USBD_IntPrint(flags.FAILSAFE);

		CK_USBD_StringPrint(" ARM:");CK_USBD_IntPrint(flags.ARMED);CK_USBD_StringPrint(" / ");

		CK_USBD_StringPrint(" ACRO:");CK_USBD_IntPrint(flags.ACRO_MODE);

		CK_USBD_StringPrint(" HORIZON:");CK_USBD_IntPrint(flags.HORIZON_MODE);

		CK_USBD_StringPrint(" ANGLE:");CK_USBD_IntPrint(flags.ANGLE_MODE);CK_USBD_StringPrint(" / ");

		CK_USBD_StringPrint(" BUZZER:");CK_USBD_IntPrint(flags.BUZZER);

		CK_USBD_StringPrint(" ALT:");CK_USBD_IntPrint(flags.ALTITUDE_HOLD);

		CK_USBD_StringPrint(" LAND:");CK_USBD_IntPrint(flags.LANDING);CK_USBD_StringPrint(" / ");

		CK_USBD_StringPrint(" GPS_RES:");CK_USBD_IntPrint(flags.GPS_RESCUE);

		CK_USBD_StringPrint(" GPS_POS:");CK_USBD_IntPrint(flags.GPS_POS_HOLD);

		CK_USBD_StringPrint(" MAG:");CK_USBD_IntPrint(flags.MAG_HOLD);

		CK_USBD_StringPrint("   T:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintRCSetpoint(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("RCSETPOINT ");CK_USBD_FloatPrint(getSetpointRate(FD_ROLL));

		CK_USBD_StringPrint("\t");CK_USBD_FloatPrint(getSetpointRate(FD_PITCH));

		CK_USBD_StringPrint("\t");CK_USBD_FloatPrint(getSetpointRate(FD_YAW));

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintRCCommand(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("RCCOMMAND ");CK_USBD_IntPrint(getRCCommand(ROLL));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCCommand(PITCH));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCCommand(YAW));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCCommand(THROTTLE));

		CK_USBD_StringPrint("     T:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintRCData(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("RCDATA ");CK_USBD_IntPrint(getRCDataRaw(ROLL));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(PITCH));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(YAW));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(THROTTLE));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX1));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX2));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX3));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX4));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX5));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX6));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX7));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(getRCDataRaw(AUX8));

		CK_USBD_StringPrint(" ");CK_USBD_IntPrint(flags.FAILSAFE);

		CK_USBD_StringPrint("  Time:");CK_USBD_IntPrint(t);CK_USBD_StringPrintln("usec");

		CK_USBD_Transmit();

	}

}

void CK_PRINTER_PrintGyroADCf(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("GyroADCf X:  ");CK_USBD_FloatPrint(gyro.gyroADCf[FD_ROLL]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_FloatPrint(gyro.gyroADCf[FD_PITCH]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_FloatPrint(gyro.gyroADCf[FD_YAW]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintGyroADCZero(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("GyroADCZero X:  ");CK_USBD_FloatPrint(gyro.gyroADCZero[FD_ROLL]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_FloatPrint(gyro.gyroADCZero[FD_PITCH]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_FloatPrint(gyro.gyroADCZero[FD_YAW]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();
	}

}

void CK_PRINTER_PrintGyroADCRaw(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("GyroADCRaw X:  ");CK_USBD_IntPrint(gyro.gyroADCRaw[FD_ROLL]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(gyro.gyroADCRaw[FD_PITCH]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(gyro.gyroADCRaw[FD_YAW]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}

}

void CK_PRINTER_PrintAccADCf(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("AccADCf X:  ");CK_USBD_FloatPrint(acc.accADCf[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_FloatPrint(acc.accADCf[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_FloatPrint(acc.accADCf[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintAccADCZero(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("AccADCfZero X:  ");CK_USBD_IntPrint(acc.accADCZero[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(acc.accADCZero[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(acc.accADCZero[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintAccADCRaw(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("AccADCRaw X:  ");CK_USBD_IntPrint(acc.accADCRaw[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(acc.accADCRaw[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(acc.accADCRaw[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintMagADCf(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("MagADCf X:  ");CK_USBD_IntPrint(mag.magADCf[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(mag.magADCf[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(mag.magADCf[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintMagHardIron(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("MagADCZero X:  ");CK_USBD_IntPrint(mag.magHardIron[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(mag.magHardIron[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(mag.magHardIron[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintMagADCRaw(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("MagADCRaw X:  ");CK_USBD_IntPrint(mag.magADCRaw[X]);

		CK_USBD_StringPrint("\tY:  ");CK_USBD_IntPrint(mag.magADCRaw[Y]);

		CK_USBD_StringPrint("\tZ:  ");CK_USBD_IntPrint(mag.magADCRaw[Z]);

		CK_USBD_StringPrint("\tT:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintAltitudeHold(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_IntPrint(barometer.temperature);CK_USBD_StringPrint("C  ");

		CK_USBD_IntPrint(barometer.pressure);CK_USBD_StringPrint(" Pa  ");

		CK_USBD_IntPrint(barometer.altitude);CK_USBD_StringPrint(" cm  ");

		CK_USBD_StringPrint(" | ");

		CK_USBD_StringPrint("  AltAdj:  ");CK_USBD_IntPrint(CK_ALTITUDE_GetThrottleAdjustment_AltitudeHold());

		CK_USBD_StringPrint("  LanAdj:  ");CK_USBD_IntPrint(CK_ALTITUDE_GetThrottleAdjustment_Landing());

		CK_USBD_StringPrint("  Hold:  ");CK_USBD_IntPrint(barometer.altitudeHold);

		CK_USBD_StringPrint("  FusedAlt: ");CK_USBD_IntPrint(CK_ALTITUDE_GetEstimatedAltitude());CK_USBD_StringPrint(" cm ");

		CK_USBD_StringPrint("  FusedVel:  ");CK_USBD_IntPrint(CK_ALTITUDE_GetEstimatedVelocity());CK_USBD_StringPrint(" cm/s ");

		CK_USBD_StringPrint("  Vel:  ");CK_USBD_IntPrint(CK_ALTITUDE_GetFailsafeVelocity());CK_USBD_StringPrintln(" cm/s ");

		CK_USBD_Transmit();

	}
}


void CK_PRINTER_PrintIMUAngles(CK_PRINT_TIMEx time, uint32_t t){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("IMU X:  ");CK_USBD_FloatPrint(attitude.values.roll);

		CK_USBD_StringPrint("  Y:  ");CK_USBD_FloatPrint(attitude.values.pitch);

		CK_USBD_StringPrint("  Z:  ");CK_USBD_FloatPrint(attitude.values.yaw);

		CK_USBD_StringPrint("  BNO X:  ");CK_USBD_FloatPrint(bno055.eulerAngles[FD_ROLL]);

		CK_USBD_StringPrint("  Y:  ");CK_USBD_FloatPrint(bno055.eulerAngles[FD_PITCH]);

		CK_USBD_StringPrint("  Z:  ");CK_USBD_FloatPrint(bno055.eulerAngles[FD_YAW]);

		CK_USBD_StringPrint("  Heading:  ");CK_USBD_FloatPrint(mag.magHeading);

		CK_USBD_StringPrint("  T:  ");CK_USBD_IntPrintln(t);

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintLoopTime(uint32_t time){

	if(time <= TARGET_MAIN_TIME_US){
		CK_USBD_StringPrint("Time:  ");CK_USBD_IntPrint(time);CK_USBD_StringPrintln(" usec");
	}
	else if(time > TARGET_MAIN_TIME_US){
			CK_USBD_StringPrint("Time:  ");CK_USBD_IntPrint(time);CK_USBD_StringPrintln(" usec********");
	}

    CK_USBD_Transmit();

}

void CK_PRINTER_PrintDebugTimes(void){

    CK_USBD_StringPrint("RX: ");CK_USBD_IntPrint(receiver_debug.update_time);
    CK_USBD_StringPrint(", RC1: ");CK_USBD_IntPrint(rc_debug.update_time);
    CK_USBD_StringPrint(", RC2: ");CK_USBD_IntPrint(rc_setpoint_debug.update_time);
    CK_USBD_StringPrint(", GYRO: ");CK_USBD_IntPrint(gyro_debug.update_time);
    CK_USBD_StringPrint(", ACC: ");CK_USBD_IntPrint(acc_debug.update_time);
    CK_USBD_StringPrint(", MAG: ");CK_USBD_IntPrint(mag_debug.update_time);
    CK_USBD_StringPrint(", BNO: ");CK_USBD_IntPrint(bno055_debug.update_time);
    CK_USBD_StringPrint(", BAR: ");CK_USBD_IntPrint(barometer_debug.update_time);
    CK_USBD_StringPrint(", GPS: ");CK_USBD_IntPrint(gps_debug.update_time);
    CK_USBD_StringPrint(", IMU: ");CK_USBD_IntPrint(imu_debug.update_time);
    CK_USBD_StringPrint(", NAV1: ");CK_USBD_IntPrint(navigation_gps_rescue_debug.update_time);
    CK_USBD_StringPrint(", NAV2: ");CK_USBD_IntPrint(navigation_gps_poshold_debug.update_time);
    CK_USBD_StringPrint(", ALT: ");CK_USBD_IntPrint(altitude_debug.update_time);
    CK_USBD_StringPrint(", PID: ");CK_USBD_IntPrint(pid_debug.update_time);
    CK_USBD_StringPrint(", MIXER: ");CK_USBD_IntPrint(mixer_debug.update_time);
    CK_USBD_StringPrint(", OSD: ");CK_USBD_IntPrint(osd_debug.update_time);
    CK_USBD_StringPrint(", MICRO: ");CK_USBD_IntPrint(micro_debug.update_time);

    CK_USBD_StringPrintln("");

    CK_USBD_Transmit();

}

void CK_PRINTER_PrintMotionCalibration(CK_PRINT_TIMEx time){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("Raw:");

		CK_USBD_IntPrint(acc.accADCRaw[X]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(acc.accADCRaw[Y]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(acc.accADCRaw[Z]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(gyro.gyroADCRaw[FD_ROLL]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(gyro.gyroADCRaw[FD_PITCH]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(gyro.gyroADCRaw[FD_YAW]);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(mag.magADCRaw[X]*0.5);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(mag.magADCRaw[Y]*0.5);CK_USBD_StringPrint(",");

		CK_USBD_IntPrint(mag.magADCRaw[Z]*0.5);CK_USBD_StringPrintln("\r");

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintRXRefrashRate(CK_PRINT_TIMEx time){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrint("RX Rate:");

		CK_USBD_IntPrintln(getCurrentRxRefreshRate());

		CK_USBD_Transmit();

	}
}

void CK_PRINTER_PrintFeedforward(CK_PRINT_TIMEx time){

	printerCounter++;

	if(printerCounter >= (time/loopTime)){

		printerCounter = 0;

		CK_USBD_StringPrintln("Feedforward:  ");

		CK_USBD_StringPrint("Roll:");CK_USBD_FloatPrint(pidData[FD_ROLL].F);CK_USBD_StringPrint(",");
		CK_USBD_StringPrint("Pitch:");CK_USBD_FloatPrint(pidData[FD_PITCH].F);CK_USBD_StringPrint(",");
		CK_USBD_StringPrint("Yaw:");CK_USBD_FloatPrint(pidData[FD_YAW].F);

		CK_USBD_Transmit();

	}
}
