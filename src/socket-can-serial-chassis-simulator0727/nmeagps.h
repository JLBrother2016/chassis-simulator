#ifndef _NMEA_GPS_H
#define _NMEA_GPS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "utilconfig.h"

/* NMEA sentence max length, including \r\n (chars) */
#define MINMEA_MAX_LENGTH 128

static const double knot2kph = 1.852;

typedef struct measure_unit {
    int degrees;
    double moa;
} measure_unit_t;

typedef struct minmea_time {
    int hours;
    int minutes;
    int seconds;
    int microseconds;
} minmea_time_t;

typedef struct minmea_date {
    int day;
    int month;
    int year;
} minmea_date_t;

// 节和常用单位之间的换算公式
// 1 knot = 0.5144444 m/s = 1.152 MPH = 1.852 KPH
typedef struct shared_gps_data {
    minmea_date_t date;     //日期
    minmea_time_t time;     //时间
    char   fix_state;       //GPS的解算状态
    double latitude;        //经度(0~90), 北为正
    double longitude;       //纬度(0~180), 东为正
    double altitude;        //高度
    double hdop;            //水平位置精度因子
    double geoid_separation;//高度偏差
    char   valid_state;     //当前数据是否有效
    double ground_speed;    //相对地面的速度, 千米/时
    double ground_course;   //相对于地面的航向, 度
    double magnetic_declination; //磁偏角, 北为正
    char   mode_status;     //GPS的工作方式
    double heading_in_degrees; //GPS的方位角
} shared_gps_data_t;

// 经度latitude、纬度longitude、高度altitude
typedef struct gga_data {
    char cmd_id[5];
    double utc_time;
    double latitude;
    char NS_indicator;
    double longitude;
    char EW_indicator;
    unsigned char fix_indicator;
    unsigned char satellites_used;
    double hdop;
    double altitude;
    double geoid_separation;
    double diff_corr;
    int    station_id;
    char   checksum;
} gga_data_t;

typedef struct rmc_data {
    char cmd_id[5];
    double utc_time;
    char validity;
    double latitude;
    char NS_indicator;
    double longitude;
    char EW_indicator;
    double speed_over_ground;
    double course_over_gound;
    unsigned int date;
    double magnetic_variation_angle;
    char magnetic_variation_indicator;
    char mode_status;
    char   checksum;
} rmc_data_t;

typedef struct hdt_data {
    char cmd_id[5];
    double  heading_in_degrees;
    char checksum;
} hdt_data_t;

uint8_t minmea_checksum(const char *sentence);
bool minmea_check(const char *sentence);

int get_position(const gga_data_t* read_gga, 
                 const rmc_data_t* read_rmc,
                 const hdt_data_t* read_hdt,
                 shared_gps_data_t* gps_information);

int set_position(shared_gps_data_t* gps_information,
                 gga_data_t* read_gga, 
                 rmc_data_t* read_rmc,
                 hdt_data_t* read_hdt);

int read_gga_sentence(char const *sentence, gga_data_t* read_gga);
int read_rmc_sentence(char const *sentence, rmc_data_t* read_rmc);
int read_hdt_sentence(char const *sentence, hdt_data_t* read_hdt);

int read_gga2sentence(const gga_data_t* read_gga, char *sentence);
int read_rmc2sentence(const rmc_data_t* read_rmc, char *sentence);
int read_hdt2sentence(const hdt_data_t* read_hdt, char *sentence);

int printf_gga(gga_data_t* gga_data);
int printf_rmc(rmc_data_t* rmc_data);
int printf_hdt(hdt_data_t* hdt_data);

int time_caculation(minmea_date_t* date, minmea_time_t *time);

int get_gps_cmd(char *buffer, int size,
                gga_data_t* read_gga, 
                rmc_data_t* read_rmc,
                hdt_data_t* hdt_data);

#ifdef __cplusplus
}
#endif

#endif //_NMEA_GPS_H
