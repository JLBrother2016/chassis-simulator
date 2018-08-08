/************************************************************************
*
*   file name：nmeagps.c
*
*   file description：read, write NMEA GPS command.
*
*   creator： Jiang Long, 2018.08.07
*
*   version：0.0.1
*
*   Modify record：
*
************************************************************************/

#include "nmeagps.h"
#include "pthreadutil.h"

/*======================================================================
 * function name： hex2int
 * parameter：
 *      char c [IN] : a byte char data
 * descriptions:
 *      convert a hex to int
 * return：success --- the int type of the hex data, failure --- -1
 =====================================================================*/
static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;

    return -1;
}

/*======================================================================
 * function name： minmea_checksum
 * parameter：
 *      const char *sentence [IN] : a nmea gps command
 * descriptions:
 *      checksum is an XOR of all bytes between "$" and "*"
 * return：checksum of sentence
 =====================================================================*/
uint8_t minmea_checksum(const char *sentence)
{   // Support senteces with or without the starting dollar sign.
    if (*sentence == '$')
        sentence++;
    uint8_t checksum = 0x00;
    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

/*======================================================================
 * function name： minmea_check
 * parameter：
 *      const char *sentence [IN] : a nmea gps command
 * descriptions:
 *      check the a nmea gps command if it is valid
 * return：success --- true, failure --- false
 =====================================================================*/
bool minmea_check(const char *sentence)
{
    uint8_t checksum = 0x00;
    // Sequence length is limited.
    if (strlen(sentence) > MINMEA_MAX_LENGTH + 3)
        return false;
    // A valid sentence starts with "$".
    if (*sentence++ != '$')
        return false;
    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        checksum ^= *sentence++;
    // If checksum is present->..
    if (*sentence == '*') {
        // Extract checksum.
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;
        // Check for checksum mismatch.
        if (checksum != expected)
            return false;
    }

    // The only stuff allowed at this point is a newline.
    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}

/*======================================================================
 * function name： read_gga_sentence
 * parameter：
 *      char const *sentence [IN] : a nmea gps command
 *      gga_data_t* read_gga [OUT] : the GGA command data
 * descriptions:
 *      read and decoding a GGA command
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_gga_sentence(char const *sentence, gga_data_t* read_gga)
{
    if(minmea_check(sentence) == false) {
        debug("sentence is not valid!\n");
        return -1;
    }
    char cmd_id[5];
    memset(cmd_id, 0, sizeof(cmd_id));
    strncpy(cmd_id, sentence + 1, sizeof(cmd_id));
    cmd_id[5] = '\0';
    if(strcmp(cmd_id, "GPGGA") != 0) {
        debug("this is not GPGAA comand\n");
        debug("cmd_id:%s\n", cmd_id);
        return -1;
    }
    memset(read_gga->cmd_id, 0, sizeof(read_gga->cmd_id));
    strncpy(read_gga->cmd_id, cmd_id, sizeof(read_gga->cmd_id));
    read_gga->cmd_id[5] = '\0';
    int ret = sscanf(sentence, "$GPGGA,%lf,%lf,%c,%lf,%c,%hhu,%hhu,%lf,%lf,M,%lf,M,%lf,%d*%c\r\n", &read_gga->utc_time, 
                        &read_gga->latitude, 
                        &read_gga->NS_indicator, 
                        &read_gga->longitude, 
                        &read_gga->EW_indicator, 
                        &read_gga->fix_indicator, 
                        &read_gga->satellites_used, 
                        &read_gga->hdop, 
                        &read_gga->altitude, 
                        &read_gga->geoid_separation, 
                        &read_gga->diff_corr,
                        &read_gga->station_id, 
                        &read_gga->checksum);
    if(ret < 0) {
        debug("sscanf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： read_rmc_sentence
 * parameter：
 *      char const *sentence [IN] : a nmea gps command
 *      rmc_data_t* read_rmc [OUT] : the RMC command data
 * descriptions:
 *      read and decoding a RMC command
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_rmc_sentence(char const *sentence, rmc_data_t* read_rmc)
{
    if(minmea_check(sentence) == false) {
        debug("sentence is not valid!\n");
        return -1;
    }
    char cmd_id[5];
    memset(cmd_id, 0, sizeof(cmd_id));
    strncpy(cmd_id, sentence + 1, sizeof(cmd_id));
    cmd_id[5] = '\0';
    if(strcmp(cmd_id, "GPRMC") != 0) {
        debug("this is not GPRMC comand\n");
        return -1;
    }
    memset(read_rmc->cmd_id, 0, sizeof(read_rmc->cmd_id));
    strncpy(read_rmc->cmd_id, cmd_id, sizeof(cmd_id));
    read_rmc->cmd_id[5] = '\0';
    int ret = sscanf(sentence, "$GPRMC,%lf,%c,%lf,%c,%lf,%c,%lf,%lf,%u,%lf,%c,%c*%c\r\n", &read_rmc->utc_time,
                    &read_rmc->validity, 
                    &read_rmc->latitude, 
                    &read_rmc->NS_indicator, 
                    &read_rmc->longitude, 
                    &read_rmc->EW_indicator, 
                    &read_rmc->speed_over_ground, 
                    &read_rmc->course_over_gound, 
                    &read_rmc->date,
                    &read_rmc->magnetic_variation_angle,
                    &read_rmc->magnetic_variation_indicator, 
                    &read_rmc->mode_status,
                    &read_rmc->checksum);
    if(ret < 0) {
        debug("sscanf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： read_hdt_sentence
 * parameter：
 *      char const *sentence [IN] : a nmea gps command
 *      hdt_data_t* read_hdt [OUT] : the HDT command data
 * descriptions:
 *      read and decoding a HDT command
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_hdt_sentence(char const *sentence, hdt_data_t* read_hdt)
{
    if(minmea_check(sentence) == false) {
        debug("sentence is not valid!\n");
        return -1;
    }
    char cmd_id[5];
    memset(cmd_id, 0, sizeof(cmd_id));
    strncpy(cmd_id, sentence + 1, sizeof(cmd_id));
    cmd_id[5] = '\0';
    if(strcmp(cmd_id, "GPHDT") != 0) {
        debug("this is not GPHDT comand\n");
        return -1;
    }
    memset(read_hdt->cmd_id, 0, sizeof(read_hdt->cmd_id));
    strncpy(read_hdt->cmd_id, cmd_id, sizeof(cmd_id));
    read_hdt->cmd_id[5] = '\0';
    int ret = sscanf(sentence, "$GPHDT,%lf,T*%c\r\n",
                        &read_hdt->heading_in_degrees,
                        &read_hdt->checksum);
    if(ret < 0) {
        debug("sscanf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： read_gga2sentence
 * parameter：
 *      const gga_data_t* read_gga [IN] : the GGA command data
 *      char *sentence [OUT] : a nmea gps command
 * descriptions:
 *      encoding a GGA command from the GGA struct
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_gga2sentence(const gga_data_t* read_gga, char *sentence)
{
    if(read_gga == NULL || sentence == NULL)
        return -1;
    if(strcmp(read_gga->cmd_id, "GPGGA") != 0) {
        debug("read_gga->cmd_id: %s\n", read_gga->cmd_id);
        debug("this is not GPGAA comand\n");
        return -1;
    }
    int ret = sprintf(sentence, "$GPGGA,%.2lf,%.8lf,%c,%.8lf,%c,%d,%d,%g,%g,M,%g,M,%g,%d*%02X\r\n", read_gga->utc_time, 
                            read_gga->latitude, 
                            read_gga->NS_indicator, 
                            read_gga->longitude, 
                            read_gga->EW_indicator, 
                            read_gga->fix_indicator, 
                            read_gga->satellites_used, 
                            read_gga->hdop, 
                            read_gga->altitude, 
                            read_gga->geoid_separation, 
                            read_gga->diff_corr,
                            read_gga->station_id, 
                            read_gga->checksum);

    if(ret < 0) {
        debug("sprintf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： read_rmc2sentence
 * parameter：
 *      const rmc_data_t* read_rmc [IN] : the RMC command data
 *      char *sentence [OUT] : a nmea gps command
 * descriptions:
 *      encoding a RMC command from the RMC struct
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_rmc2sentence(const rmc_data_t* read_rmc, char *sentence)
{
    if(read_rmc == NULL || sentence == NULL)
        return -1;
    if(strncmp(read_rmc->cmd_id, "GPRMC", sizeof("GPRMC")) != 0) {
        debug("read_rmc->cmd_id:%s\n", read_rmc->cmd_id);
        debug("this is not GPRMC comand\n");
        return -1;
    }
    int ret = sprintf(sentence, "$GPRMC,%.2lf,%c,%.8lf,%c,%.8lf,%c,%g,%g,%d,%g,%c,%c*%02X\r\n", read_rmc->utc_time,
                         read_rmc->validity, 
                         read_rmc->latitude, 
                         read_rmc->NS_indicator, 
                         read_rmc->longitude, 
                         read_rmc->EW_indicator, 
                         read_rmc->speed_over_ground, 
                         read_rmc->course_over_gound, 
                         read_rmc->date,
                         read_rmc->magnetic_variation_angle,
                         read_rmc->magnetic_variation_indicator, 
                         read_rmc->mode_status,
                         read_rmc->checksum);
    if(ret < 0) {
        debug("sprintf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： read_hdt2sentence
 * parameter：
 *      const hdt_data_t* read_hdt [IN] : the HDT command data
 *      char *sentence [OUT] : a nmea gps command
 * descriptions:
 *      encoding a HDT command from the HDT struct
 * return：success --- 0, failure --- -1
 =====================================================================*/
int read_hdt2sentence(const hdt_data_t* read_hdt, char *sentence)
{
    if(read_hdt == NULL || sentence == NULL)
        return -1;
    if(strncmp(read_hdt->cmd_id, "GPHDT", sizeof("GPHDT")) != 0) {
        debug("read_hdt->cmd_id:%s\n", read_hdt->cmd_id);
        debug("this is not GPHDT comand\n");
        return -1;
    }
    int ret = sprintf(sentence, "$GPHDT,%.3lf,T*%02X\r\n",
                        read_hdt->heading_in_degrees,
                        read_hdt->checksum);
    if(ret < 0) {
        debug("sprintf sentence error\n");
        return -1;
    }

    return 0;
}

/*======================================================================
 * function name： printf_gga
 * parameter：
 *      gga_data_t* gga_data [IN] : the GGA command data
 * descriptions:
 *      printf a GGA command detail data.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int printf_gga(gga_data_t* gga_data)
{
    if(gga_data == NULL)
        return -1;
    debug("cmd_id = %s, \nutc_time = %f, \nlatitude = %.8lf, \nNS = %c,\
        \nlongitude = %.8lf, \nEW = %c, \nfix = %d, \nsate = %d, \nHDOP = %f,\
        \naltitude = %f, \ngeoid = %f, \ndiff_corr = %f, \nstation_id = %d,\
        \nchecksum = %02X\n", gga_data->cmd_id,
                            gga_data->utc_time, 
                            gga_data->latitude, 
                            gga_data->NS_indicator, 
                            gga_data->longitude,
                            gga_data->EW_indicator, 
                            gga_data->fix_indicator,
                            gga_data->satellites_used, 
                            gga_data->hdop, 
                            gga_data->altitude,
                            gga_data->geoid_separation, 
                            gga_data->diff_corr, 
                            gga_data->station_id,
                            gga_data->checksum);

    return 0;
}

/*======================================================================
 * function name： printf_rmc
 * parameter：
 *      rmc_data_t* rmc_data [IN] : the RMC command data
 * descriptions:
 *      printf a RMC command detail data.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int printf_rmc(rmc_data_t* rmc_data)
{
    if(rmc_data == NULL)
        return -1;
    debug("cmd_id = %s, \nutc_time = %f, \nV = %c, \nlatitude = %.8lf,\
        \nNS = %c, \nlongitude = %.8lf, \nEW = %c, \nspeed in knots = %f,\
        \ncourse = %f, \ndate = %d, \nvariation = %f, \nindicator = %c, \
        \nstatus = %c, \nchecksum = %02X\n", 
                      rmc_data->cmd_id,
                      rmc_data->utc_time, 
                      rmc_data->validity,
                      rmc_data->latitude, 
                      rmc_data->NS_indicator, 
                      rmc_data->longitude, 
                      rmc_data->EW_indicator, 
                      rmc_data->speed_over_ground, 
                      rmc_data->course_over_gound, 
                      rmc_data->date, 
                      rmc_data->magnetic_variation_angle,
                      rmc_data->magnetic_variation_indicator, 
                      rmc_data->mode_status, 
                      rmc_data->checksum);

    return  0;
}

/*======================================================================
 * function name： printf_hdt
 * parameter：
 *      hdt_data_t* hdt_data [IN] : the HDT command data
 * descriptions:
 *      printf a HDT command detail data.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int printf_hdt(hdt_data_t* hdt_data)
{
    if(hdt_data == NULL)
        return -1;
    debug("cmd_id = %s, \ndegrees = %f, checksum = %02X\n", 
            hdt_data->cmd_id,
            hdt_data->heading_in_degrees,
            hdt_data->checksum);

    return 0;
}

/*======================================================================
 * function name： get_position
 * parameter：
 *      const gga_data_t* read_gga [IN] : the GGA command data
 *      const rmc_data_t* read_rmc [IN] : the RMC command data
 *      const hdt_data_t* read_hdt [IN] : the HDT command data
 *      shared_gps_data_t* gps_information [OUT] : the shared gps information
 * descriptions:
 *      convert and abstract GGA, RMC, HDT data to the shared gps information.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int get_position(const gga_data_t* read_gga, 
                 const rmc_data_t* read_rmc,
                 const hdt_data_t* read_hdt,
                 shared_gps_data_t* gps_information)
{
    if(read_gga == NULL ||
       read_rmc == NULL ||
       read_hdt == NULL ||
       gps_information == NULL)
        return -1;

    gps_information->time.hours = ((int)read_gga->utc_time) / 10000;
    gps_information->time.minutes = ((int)read_gga->utc_time / 100) % 100;
    gps_information->time.seconds = (int)read_gga->utc_time % 100;
    gps_information->time.microseconds = (read_gga->utc_time  
                                      - (int)(read_gga->utc_time)) * 1000;

    gps_information->date.day = read_rmc->date % 100;
    gps_information->date.month = (read_rmc->date / 100) % 100;
    gps_information->date.year =  (read_rmc->date / 10000);

    gps_information->fix_state = read_gga->fix_indicator; //获取解算状态

    measure_unit_t tmp_unit;
    tmp_unit.degrees = (int)(read_gga->latitude / 100);
    tmp_unit.moa = read_gga->latitude - tmp_unit.degrees * 100;
    gps_information->latitude = (double)tmp_unit.degrees
                             + tmp_unit.moa / 60.0;
    if(read_gga->NS_indicator == 'S' || read_gga->NS_indicator == 's')
        gps_information->latitude = -gps_information->latitude;

    tmp_unit.degrees = (int)(read_gga->longitude / 100);
    tmp_unit.moa = read_gga->longitude - tmp_unit.degrees * 100;
    gps_information->longitude = (double)tmp_unit.degrees
                              + tmp_unit.moa / 60.0;
    if(read_gga->EW_indicator == 'W' || read_gga->EW_indicator == 'w')
        gps_information->longitude = -gps_information->longitude; //纬度取负   

    gps_information->altitude = read_gga->altitude;
    gps_information->hdop = read_gga->hdop;
    gps_information->geoid_separation = read_gga->geoid_separation;
    gps_information->valid_state = read_rmc->validity;
    gps_information->ground_speed = read_rmc->speed_over_ground * knot2kph;
    gps_information->ground_course = read_rmc->course_over_gound;
    gps_information->magnetic_declination = read_rmc->magnetic_variation_angle;
    if(read_rmc->magnetic_variation_indicator == 'S' || 
       read_rmc->magnetic_variation_indicator == 's')
        gps_information->magnetic_declination = -gps_information->magnetic_declination;
    gps_information->mode_status = read_rmc->mode_status;
    gps_information->heading_in_degrees = read_hdt->heading_in_degrees;

    return 0;
}

/*======================================================================
 * function name： set_position
 * parameter：
 *      shared_gps_data_t* gps_information [IN] : the shared gps information
 *      gga_data_t* read_gga [OUT] : the GGA command data
 *      rmc_data_t* read_rmc [OUT] : the RMC command data
 *      hdt_data_t* read_hdt [OUT] : the HDT command data
 * descriptions:
 *      convert the shared gps information to GGA, RMC, HDT data.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int set_position(shared_gps_data_t* gps_information,
                 gga_data_t* read_gga, 
                 rmc_data_t* read_rmc,
                 hdt_data_t* read_hdt)
{
    if(read_gga == NULL ||
       read_rmc == NULL ||
       read_hdt == NULL ||
       gps_information == NULL)
        return -1;

    read_gga->utc_time =  (double)(gps_information->time.hours * 10000
                       + gps_information->time.minutes * 100
                       + gps_information->time.seconds
                       + (double)gps_information->time.microseconds / 1000.0);
    read_rmc->date = gps_information->date.year 
                   + gps_information->date.month * 100
                   + gps_information->date.day * 10000;

    read_gga->fix_indicator = gps_information->fix_state;

    if(gps_information->latitude < 0) {
        read_gga->NS_indicator = 'S';
        gps_information->latitude = abs(gps_information->latitude);
    } else {
        read_gga->NS_indicator = 'N';
    }

    measure_unit_t tmp_unit;
    tmp_unit.degrees = (int)(gps_information->latitude);
    tmp_unit.moa = (gps_information->latitude - tmp_unit.degrees) * 60.0;
    read_gga->latitude = tmp_unit.degrees * 100 + tmp_unit.moa;

    if(gps_information->longitude < 0) {
        read_gga->EW_indicator = 'W';
        gps_information->longitude = abs(gps_information->longitude);
    } else {
        read_gga->EW_indicator = 'E';
    }

    tmp_unit.degrees = (int)(gps_information->longitude);
    tmp_unit.moa = (gps_information->longitude - tmp_unit.degrees) * 60.0;
    read_gga->longitude = tmp_unit.degrees * 100 + tmp_unit.moa;
    
    read_rmc->utc_time = read_gga->utc_time;
    read_rmc->latitude = read_gga->latitude;
    read_rmc->NS_indicator = read_gga->NS_indicator;
    read_rmc->longitude = read_gga->longitude;
    read_rmc->EW_indicator = read_gga->EW_indicator;

    read_gga->altitude = gps_information->altitude;
    read_gga->hdop = gps_information->hdop;
    read_gga->geoid_separation = gps_information->geoid_separation;
    read_rmc->validity = gps_information->valid_state;
    read_rmc->speed_over_ground = gps_information->ground_speed / knot2kph;
    read_rmc->course_over_gound = gps_information->ground_course;
    if(gps_information->magnetic_declination < 0) {
        gps_information->magnetic_declination = abs(gps_information->magnetic_declination);
        read_rmc->magnetic_variation_indicator = 'S';
    } else {
        read_rmc->magnetic_variation_indicator = 'N';
    }
    read_rmc->magnetic_variation_angle = gps_information->magnetic_declination;
    read_rmc->mode_status = gps_information->mode_status;
    read_hdt->heading_in_degrees = fmod(gps_information->heading_in_degrees, 
                                        360.0);

    return 0;
}

/*======================================================================
 * function name： time_caculation
 * parameter：
 *      minmea_date_t* date [OUT] : the NMEA date format
 *      minmea_time_t *time [OUT] : the NMEA time format
 * descriptions:
 *      convert localtime to the NMEA date and time.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int time_caculation(minmea_date_t* date, minmea_time_t *time)
{
    if(date == NULL || time == NULL)
        return -1;
    struct timeval tv;
    struct tm *ptm;

    gettimeofday(&tv, NULL);
    ptm = localtime(&tv.tv_sec);      //localtime convert local zone

    time->microseconds = (tv.tv_usec) / 1000;
    time->seconds = ptm->tm_sec;
    time->minutes = ptm->tm_min;
    time->hours = ptm->tm_hour - 8;   //local time minus 8 hours is green time

    date->day = ptm->tm_mday;
    date->month = ptm->tm_mon + 1;
    date->year = (ptm->tm_year + 1900) - 2000;

    return 0;
}

/*======================================================================
 * function name： time_caculation
 * parameter：
 *      char *buffer [OUT] : the buffer storing the three GPS command
 *      int size [IN] : the size of the buffer
 *      gga_data_t* read_gga [IN] : the GGA command data
 *      rmc_data_t* read_rmc [IN] : the RMC command data
 *      hdt_data_t* read_hdt [IN] : the HDT command data
 * descriptions:
 *      merge the three GPS command to a big buffer.
 * return：success --- 0, failure --- -1
 =====================================================================*/
int get_gps_cmd(char *buffer, int size,
                gga_data_t* read_gga, 
                rmc_data_t* read_rmc,
                hdt_data_t* read_hdt)
{
    if(buffer == NULL ||
       read_gga == NULL ||
       read_rmc == NULL ||
       read_hdt == NULL)
        return -1;
    // buffer storing the three GPS command
    char tmp[128] = {0};
    memset(buffer, 0, size);

    read_gga2sentence(read_gga, tmp);
    read_gga->checksum = minmea_checksum(tmp);
    read_gga2sentence(read_gga, tmp);
    strncat(buffer, tmp, strlen(tmp));

    memset(tmp, 0, sizeof(tmp));
    read_rmc2sentence(read_rmc, tmp);
    read_rmc->checksum = minmea_checksum(tmp);
    read_rmc2sentence(read_rmc, tmp);
    strncat(buffer, tmp, strlen(tmp));

    memset(tmp, 0, sizeof(tmp));
    read_hdt2sentence(read_hdt, tmp);
    read_hdt->checksum = minmea_checksum(tmp);
    read_hdt2sentence(read_hdt, tmp);
    strncat(buffer, tmp, strlen(tmp));

    tmp[strlen(tmp)] = '\0';

    return 0;
}



