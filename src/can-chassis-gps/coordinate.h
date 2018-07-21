#ifndef _COORDINATE_H
#define _COORDINATE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "utilconfig.h"

typedef struct UTMcoordinate {
    double x;
    double y;
} UTMcoordinate_t;

typedef struct WGS84coordinate
{
    double latitude;
    double longitude;
} WGS84coordinate_t;

static const double pi = 3.14159265358979;

static inline double degree2radian(double deg)
{
    return (deg / 180.0 * pi);
}

static inline double radian2degree(double rad)
{
    return (rad / pi * 180.0);
}

int WGS84_to_UTM(WGS84coordinate_t* latlon, UTMcoordinate_t* utmxy);
int UTM_to_WGS84(UTMcoordinate_t* utmxy, WGS84coordinate_t* latlon, 
                 WGS84coordinate_t* last_latlon);


#ifdef __cplusplus
}
#endif

#endif //_COORDINATE_H
