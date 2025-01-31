#ifndef CK_PRINTER_H_
#define CK_PRINTER_H_

#include "CK_DEFINITIONS.h"

typedef enum{
	EVERY_125US		= 125,
	EVERY_250US		= 250,
	EVERY_500US		= 500,
	EVERY_1MS		= 1000,//in microsec
	EVERY_2MS		= 2000,
	EVERY_3MS		= 3000,
	EVERY_4MS		= 4000,
	EVERY_5MS		= 5000,
	EVERY_6MS		= 6000,
	EVERY_7MS		= 7000,
	EVERY_9MS		= 9000,
	EVERY_10MS		= 10000,
	EVERY_11MS		= 11000,
	EVERY_13MS		= 13000,
	EVERY_15MS		= 15000,
	EVERY_20MS		= 20000,
	EVERY_25MS		= 25000,
	EVERY_50MS		= 50000,
	EVERY_100MS		= 100000,
	EVERY_200MS		= 200000,
	EVERY_250MS		= 250000,
	EVERY_500MS     = 500000,
	EVERY_1000MS	= 1000000

}CK_PRINT_TIMEx;

typedef struct{

    uint32_t start_time;

    uint32_t update_time;

}DEBUG_TIME_t;

extern DEBUG_TIME_t rc_debug;
extern DEBUG_TIME_t rc_setpoint_debug;
extern DEBUG_TIME_t receiver_debug;
extern DEBUG_TIME_t gyro_debug;
extern DEBUG_TIME_t acc_debug;
extern DEBUG_TIME_t mag_debug;
extern DEBUG_TIME_t barometer_debug;
extern DEBUG_TIME_t gps_debug;
extern DEBUG_TIME_t bno055_debug;
extern DEBUG_TIME_t imu_debug;
extern DEBUG_TIME_t navigation_gps_rescue_debug;
extern DEBUG_TIME_t navigation_gps_poshold_debug;
extern DEBUG_TIME_t altitude_debug;
extern DEBUG_TIME_t pid_debug;
extern DEBUG_TIME_t mixer_debug;
extern DEBUG_TIME_t osd_debug;
extern DEBUG_TIME_t micro_debug;

void CK_PRINTER_Init(uint32_t mainT);

void CK_PRINTER_Update(CK_PRINT_TIMEx print_freq, uint32_t compT);

void CK_PRINTER_DecodeInputStream(uint8_t* buffer, uint16_t size);

void CK_PRINTER_Transfer(void);

void CK_PRINTER_PrintString(const char str[]);

void CK_PRINTER_PrintlnString(const char str[]);

void CK_PRINTER_PrintInt(int32_t num);

void CK_PRINTER_PrintlnInt(int32_t num);

void CK_PRINTER_PrintFloat(float num);

void CK_PRINTER_PrintlnFloat(float num);

void CK_PRINTER_PrintFloatDecimal(float num, int dec);

void CK_PRINTER_PrintlnFloatDecimal(float num, int dec);

void CK_PRINTER_PrintADC(void);

void CK_PRINTER_PrintPID(void);

void CK_PRINTER_PrintRC(void);

void CK_PRINTER_PrintNavigation(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGPS(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGPS2(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGPS3(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintMotorFinalResults(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintMotorMixResults(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintFlags(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintRCSetpoint(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintRCCommand(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintRCData(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGyroADCf(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGyroADCZero(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintGyroADCRaw(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintAccADCf(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintAccADCZero(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintAccADCRaw(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintMagADCf(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintMagHardIron(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintMagADCRaw(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintBarometer(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintAltitudeHold(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintIMUAngles(CK_PRINT_TIMEx time, uint32_t t);

void CK_PRINTER_PrintLoopTime(uint32_t time);

void CK_PRINTER_PrintDebugTimes(void);

void CK_PRINTER_PrintMotionCalibration(CK_PRINT_TIMEx time);

void CK_PRINTER_PrintRXRefrashRate(CK_PRINT_TIMEx time);

void CK_PRINTER_PrintFeedforward(CK_PRINT_TIMEx time);

#endif /* CK_PRINTER_H_ */
