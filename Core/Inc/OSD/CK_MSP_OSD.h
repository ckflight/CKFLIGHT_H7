/*
 * CK_MSP_OSD.h
 *
 *  Created on: Jun 12, 2023
 *      Author: ck
 */

#ifndef INC_OSD_CK_MSP_OSD_H_
#define INC_OSD_CK_MSP_OSD_H_

#include "CK_DEFINITIONS.h"

//Misc
#define SYM_NONE                    0x00
#define SYM_END_OF_FONT             0xFF
#define SYM_BLANK                   0x20
#define SYM_HYPHEN                  0x2D
#define SYM_BBLOG                   0x10
#define SYM_HOMEFLAG                0x11
//#define SYM_RPM                     0x12
#define SYM_ROLL                    0x14
#define SYM_PITCH                   0x15
#define SYM_TEMPERATURE             0x7A

// GPS and navigation
#define SYM_LAT                     0x89
#define SYM_LON                     0x98
#define SYM_ALTITUDE                0x7F
#define SYM_TOTAL_DISTANCE          0x71
#define SYM_OVER_HOME               0x05

// RSSI
#define SYM_RSSI                    0x01
#define SYM_LINK_QUALITY            0x7B

// Throttle Position (%)
#define SYM_THR                     0x04

// Unit Icons (Metric)
#define SYM_M                       0x0C
#define SYM_KM                      0x7D
#define SYM_C                       0x0E

// Unit Icons (Imperial)
#define SYM_FT                      0x0F
#define SYM_MILES                   0x7E
#define SYM_F                       0x0D

// Heading Graphics
#define SYM_HEADING_N               0x18
#define SYM_HEADING_S               0x19
#define SYM_HEADING_E               0x1A
#define SYM_HEADING_W               0x1B
#define SYM_HEADING_DIVIDED_LINE    0x1C
#define SYM_HEADING_LINE            0x1D

// AH Center screen Graphics
#define SYM_AH_CENTER_LINE          0x72
#define SYM_AH_CENTER               0x73
#define SYM_AH_CENTER_LINE_RIGHT    0x74
#define SYM_AH_RIGHT                0x02
#define SYM_AH_LEFT                 0x03
#define SYM_AH_DECORATION           0x13

// Satellite Graphics
#define SYM_SAT_L                   0x1E
#define SYM_SAT_R                   0x1F

// Direction arrows
#define SYM_ARROW_SOUTH             0x60
#define SYM_ARROW_2                 0x61
#define SYM_ARROW_3                 0x62
#define SYM_ARROW_4                 0x63
#define SYM_ARROW_EAST              0x64
#define SYM_ARROW_6                 0x65
#define SYM_ARROW_7                 0x66
#define SYM_ARROW_8                 0x67
#define SYM_ARROW_NORTH             0x68
#define SYM_ARROW_10                0x69
#define SYM_ARROW_11                0x6A
#define SYM_ARROW_12                0x6B
#define SYM_ARROW_WEST              0x6C
#define SYM_ARROW_14                0x6D
#define SYM_ARROW_15                0x6E
#define SYM_ARROW_16                0x6F

#define SYM_ARROW_SMALL_UP          0x75
#define SYM_ARROW_SMALL_DOWN        0x76

// AH Bars
#define SYM_AH_BAR9_0               0x80
#define SYM_AH_BAR9_1               0x81
#define SYM_AH_BAR9_2               0x82
#define SYM_AH_BAR9_3               0x83
#define SYM_AH_BAR9_4               0x84
#define SYM_AH_BAR9_5               0x85
#define SYM_AH_BAR9_6               0x86
#define SYM_AH_BAR9_7               0x87
#define SYM_AH_BAR9_8               0x88

// Progress bar
#define SYM_PB_START                0x8A
#define SYM_PB_FULL                 0x8B
#define SYM_PB_HALF                 0x8C
#define SYM_PB_EMPTY                0x8D
#define SYM_PB_END                  0x8E
#define SYM_PB_CLOSE                0x8F

// Batt evolution
#define SYM_BATT_FULL               0x90
#define SYM_BATT_5                  0x91
#define SYM_BATT_4                  0x92
#define SYM_BATT_3                  0x93
#define SYM_BATT_2                  0x94
#define SYM_BATT_1                  0x95
#define SYM_BATT_EMPTY              0x96

// Batt Icons
#define SYM_MAIN_BATT               0x97

// Voltage and amperage
#define SYM_VOLT                    0x06
#define SYM_AMP                     0x9A
#define SYM_MAH                     0x07
#define SYM_WATT                    0x57  // 0x57 is 'W'

// Time
#define SYM_ON_M                    0x9B
#define SYM_FLY_M                   0x9C

// Lap Timer
#define SYM_CHECKERED_FLAG          0x24
#define SYM_PREV_LAP_TIME           0x79

// Speed
#define SYM_SPEED                   0x70
#define SYM_KPH                     0x9E
#define SYM_MPH                     0x9D
#define SYM_MPS                     0x9F
#define SYM_FTPS                    0x99

// Menu cursor
#define SYM_CURSOR                  SYM_AH_LEFT

// Stick overlays
#define SYM_STICK_OVERLAY_SPRITE_HIGH 0x08
#define SYM_STICK_OVERLAY_SPRITE_MID  0x09
#define SYM_STICK_OVERLAY_SPRITE_LOW  0x0A
#define SYM_STICK_OVERLAY_CENTER      0x0B
#define SYM_STICK_OVERLAY_VERTICAL    0x16
#define SYM_STICK_OVERLAY_HORIZONTAL  0x17

// GPS degree/minute/second symbols
#define SYM_GPS_DEGREE              SYM_STICK_OVERLAY_SPRITE_HIGH  // kind of looks like the degree symbol
#define SYM_GPS_MINUTE              0x27 // '
#define SYM_GPS_SECOND              0x22 // "

typedef struct{
    uint8_t osdflags;
    uint8_t video_system;
    uint8_t units;
    uint8_t rssi_alarm;
    uint16_t cap_alarm;
    uint8_t old_timer_alarm;
    uint8_t osd_item_count;                     //80
    uint16_t alt_alarm;

    uint16_t osd_rssi_value_pos;
    uint16_t osd_main_batt_voltage_pos;
    uint16_t osd_crosshairs_pos;
    uint16_t osd_artificial_horizon_pos;
    uint16_t osd_horizon_sidebars_pos;
    uint16_t osd_item_timer_1_pos;
    uint16_t osd_item_timer_2_pos;
    uint16_t osd_flymode_pos;
    uint16_t osd_craft_name_pos;
    uint16_t osd_throttle_pos_pos;
    uint16_t osd_vtx_channel_pos;
    uint16_t osd_current_draw_pos;
    uint16_t osd_mah_drawn_pos;
    uint16_t osd_gps_speed_pos;
    uint16_t osd_gps_sats_pos;
    uint16_t osd_altitude_pos;
    uint16_t osd_roll_pids_pos;
    uint16_t osd_pitch_pids_pos;
    uint16_t osd_yaw_pids_pos;
    uint16_t osd_power_pos;
    uint16_t osd_pidrate_profile_pos;
    uint16_t osd_warnings_pos;
    uint16_t osd_avg_cell_voltage_pos;
    uint16_t osd_gps_lon_pos;
    uint16_t osd_gps_lat_pos;
    uint16_t osd_debug_pos;
    uint16_t osd_pitch_angle_pos;
    uint16_t osd_roll_angle_pos;
    uint16_t osd_main_batt_usage_pos;
    uint16_t osd_disarmed_pos;
    uint16_t osd_home_dir_pos;
    uint16_t osd_home_dist_pos;
    uint16_t osd_numerical_heading_pos;
    uint16_t osd_numerical_vario_pos;
    uint16_t osd_compass_bar_pos;
    uint16_t osd_esc_tmp_pos;
    uint16_t osd_esc_rpm_pos;
    uint16_t osd_remaining_time_estimate_pos;
    uint16_t osd_rtc_datetime_pos;
    uint16_t osd_adjustment_range_pos;
    uint16_t osd_core_temperature_pos;
    uint16_t osd_anti_gravity_pos;
    uint16_t osd_g_force_pos;
    uint16_t osd_motor_diag_pos;
    uint16_t osd_log_status_pos;
    uint16_t osd_flip_arrow_pos;
    uint16_t osd_link_quality_pos;
    uint16_t osd_flight_dist_pos;
    uint16_t osd_stick_overlay_left_pos;
    uint16_t osd_stick_overlay_right_pos;
    uint16_t osd_display_name_pos;
    uint16_t osd_esc_rpm_freq_pos;
    uint16_t osd_rate_profile_name_pos;
    uint16_t osd_pid_profile_name_pos;
    uint16_t osd_profile_name_pos;
    uint16_t osd_rssi_dbm_value_pos;
    uint16_t osd_rc_channels_pos;
    uint16_t osd_camera_frame_pos;
    uint16_t osd_effiency_pos;
    uint16_t osd_total_flights_pos;
    uint16_t osd_up_down_ref_pos;
    uint16_t osd_tx_uplink_power_pos;
    uint16_t osd_watt_hours_drawn_pos;
    uint16_t osd_aux_value_pos;
    uint16_t osd_ready_mode_pos;
    uint16_t osd_rsnr_value_pos;
    uint16_t osd_sys_goggle_voltage_pos;
    uint16_t osd_sys_vtx_voltage_pos;
    uint16_t osd_sys_bitrate_pos;
    uint16_t osd_sys_delay_pos;
    uint16_t osd_sys_distance_pos;
    uint16_t osd_sys_lo_pos;
    uint16_t osd_sys_goggle_dvr_pos;
    uint16_t osd_sys_vtx_dvr_pos;
    uint16_t osd_sys_warnings_pos;
    uint16_t osd_sys_vtx_temp_pos;
    uint16_t osd_sys_fan_speed_pos;
    uint16_t osd_gps_lap_time_current_pos;
    uint16_t osd_gps_lap_time_previous_pos;
    uint16_t osd_gps_lap_time_best3_pos;
    uint16_t osd_pos_buffer[80];

    uint8_t osd_stat_count;                     //29
    uint8_t osd_stat_rtc_date_time;
    uint8_t osd_stat_timer_1;
    uint8_t osd_stat_timer_2;
    uint8_t osd_stat_max_speed;
    uint8_t osd_stat_max_distance;
    uint8_t osd_stat_min_battery;
    uint8_t osd_stat_end_battery;
    uint8_t osd_stat_battery;
    uint8_t osd_stat_min_rssi;
    uint8_t osd_stat_max_current;
    uint8_t osd_stat_used_mah;
    uint8_t osd_stat_max_altitude;
    uint8_t osd_stat_blackbox;
    uint8_t osd_stat_blackbox_number;
    uint8_t osd_stat_max_g_force;
    uint8_t osd_stat_max_esc_temp;
    uint8_t osd_stat_max_esc_rpm;
    uint8_t osd_stat_min_link_quality;
    uint8_t osd_stat_flight_distance;
    uint8_t osd_stat_max_fft;
    uint8_t osd_stat_total_flights;
    uint8_t osd_stat_total_time;
    uint8_t osd_stat_total_dist;
    uint8_t osd_stat_min_rssi_dbm;
    uint8_t osd_stat_watt_hours_drawn;
    uint8_t osd_stat_min_rsnr;
    uint8_t osd_stat_best_3_consec_laps;
    uint8_t osd_stat_est_lap;

    uint16_t osd_timer_count;
    uint16_t osd_timer_1;
    uint16_t osd_timer_2;
    uint16_t enabledwarnings;

    uint8_t osd_warning_count;              // 16

    uint32_t enabledwarnings_1_41_plus;

    uint8_t osd_profile_count;              // 1
    uint8_t osdprofileindex;                // 1
    uint8_t overlay_radio_mode;             // 0

    uint8_t camera_frame_width;             // 24
    uint8_t camera_frame_height;            // 11

    syncTimer_t sync;

}msp_osd_config_t;

void CK_MSP_OSD_Init(uint32_t osdT, uint32_t mainT);

void CK_MSP_OSD_Update(uint32_t currentTime);

void CK_MSP_OSD_Timer(uint32_t currentTime);

void CK_MSP_OSD_Current(uint32_t currentTime);

void CK_MSP_OSD_RSSI(uint32_t currentTime);

void CK_MSP_OSD_RSSILQ(uint32_t currentTime);

void CK_MSP_OSD_CoreTemperature(uint32_t currentTime);

void CK_MSP_OSD_FirmwareFreqPlot(uint32_t currentTime);

void CK_MSP_OSD_MahPlot(uint32_t currentTime);

uint16_t CK_MSP_OSD_PlotCommands(uint8_t command_id, uint8_t* buffer);

uint16_t CK_MSP_OSD_WriteString(uint8_t row, uint8_t col, uint8_t* buffer, const char str[], uint8_t len);

uint16_t CK_MSP_OSD_PacketSequence(uint8_t* buffer);

void CK_OSD_DJI_InitLocationParameters(void);


#endif /* INC_OSD_CK_MSP_OSD_H_ */
