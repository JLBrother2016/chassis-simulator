#include "coordinate.h"

/* Ellipsoid model constants (actual values here are for WGS84) */
static const double sm_a = 6378137.0;       //单位为km
static const double sm_b = 6356752.314;
static const double sm_EccSquared = 6.69437999013e-03;
static const double UTMScaleFactor = 0.9996;

static double arc_of_deridian(double phi)
{
    double alpha, beta, gamma, delta, epsilon, n;
    double result;

    n = (sm_a - sm_b) / (sm_a + sm_b);
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0)
          + (pow(n, 4.0) / 64.0));
    beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0)
         + (-3.0 * pow(n, 5.0) / 32.0);
    gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    epsilon = (315.0 * pow(n, 4.0) / 512.0);
    result = alpha * (phi + (beta * sin(2.0 * phi)) 
           + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi))
           + (epsilon * sin(8.0 * phi)));

    return result;
}

// ellipsoidal coordinates 转换为椭圆坐标, 
// 输入为UTM的北极坐标y，输出为纬度
static double y2latitude(double y)
{
    double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
    double result;

    n = (sm_a - sm_b) / (sm_a + sm_b);
    alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (pow(n, 2.0) / 4) 
           + (pow(n, 4.0) / 64));
    y_ = y / alpha_;
    beta_ = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) 
          + (269.0 * pow(n, 5.0) / 512.0);
    gamma_ = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);
    delta_ = (151.0 * pow (n, 3.0) / 96.0)  + (-417.0 * pow (n, 5.0) / 128.0);
    epsilon_ = (1097.0 * pow(n, 4.0) / 512.0);
    result = y_ + (beta_ * sin(2.0 * y_)) + (gamma_ * sin(4.0 * y_))
           + (delta_ * sin(6.0 * y_)) + (epsilon_ * sin(8.0 * y_));

    return result;
}

static int lanlon2utmxy (double phi, double lambda, double lambda0, 
                         UTMcoordinate_t* utmxy)
{
    double N, nu2, ep2, t, t2, l;
    double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;

    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    nu2 = ep2 * pow(cos(phi), 2.0);
    N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    t = tan (phi);
    t2 = t * t;
    l = lambda - lambda0;
    /* Precalculate coefficients for l**n in the equations below
    so a normal human being can read the expressions for easting
    and northing
    -- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;
    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);
    /* Calculate easting (x) */
    utmxy->x = N * cos (phi) * l + (N / 6.0 * pow(cos(phi), 3.0)
             * l3coef * pow(l, 3.0))
             + (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0))
             + (N / 5040.0 * pow(cos (phi), 7.0) * l7coef * pow(l, 7.0));
    /* Calculate northing (y) */
    utmxy->y = arc_of_deridian(phi)   //phi是经度
             + (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0))
             + (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0))
             + (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0))
             + (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));

    return 0;
}


static int utmxy2lanlon (double x, double y, double lambda0,
                         WGS84coordinate_t* philambda)
{
    double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
    double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
    double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

    /* Get the value of phif, the footpoint latitude. */
    phif = y2latitude(y);
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    cf = cos (phif);
    nuf2 = ep2 * pow (cf, 2.0);
    Nf = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nuf2));
    Nfpow = Nf;
    tf = tan (phif);
    tf2 = tf * tf;
    tf4 = tf2 * tf2;
    x1frac = 1.0 / (Nfpow * cf);
    Nfpow *= Nf;   /* now equals Nf**2) */
    x2frac = tf / (2.0 * Nfpow);
    Nfpow *= Nf;   /* now equals Nf**3) */
    x3frac = 1.0 / (6.0 * Nfpow * cf);
    Nfpow *= Nf;   /* now equals Nf**4) */
    x4frac = tf / (24.0 * Nfpow);
    Nfpow *= Nf;   /* now equals Nf**5) */
    x5frac = 1.0 / (120.0 * Nfpow * cf);
    Nfpow *= Nf;   /* now equals Nf**6) */
    x6frac = tf / (720.0 * Nfpow);
    Nfpow *= Nf;   /* now equals Nf**7) */
    x7frac = 1.0 / (5040.0 * Nfpow * cf);
    Nfpow *= Nf;   /* now equals Nf**8) */
    x8frac = tf / (40320.0 * Nfpow);
    /* Precalculate polynomial coefficients for x**n.
    -- x**1 does not have a polynomial coefficient. */
    x2poly = -1.0 - nuf2;
    x3poly = -1.0 - 2 * tf2 - nuf2;
    x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 
           * (nuf2 *nuf2) - 9.0 * tf2 * (nuf2 * nuf2);
    x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;
    x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 
           + 162.0 * tf2 * nuf2;
    x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);
    x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);
    /* Calculate latitude */
    philambda->latitude = phif + x2frac * x2poly * (x * x) 
                        + x4frac * x4poly * pow(x, 4.0) 
                        + x6frac * x6poly * pow(x, 6.0) 
                        + x8frac * x8poly * pow(x, 8.0);
    /* Calculate longitude */
    philambda->longitude = lambda0 + x1frac * x 
                         + x3frac * x3poly * pow(x, 3.0) 
                         + x5frac * x5poly * pow(x, 5.0) 
                         + x7frac * x7poly * pow(x, 7.0);
    philambda->latitude = radian2degree(philambda->latitude); //转化为角度
    philambda->longitude = radian2degree(philambda->longitude);

    return 0;
}

int WGS84_to_UTM(WGS84coordinate_t* latlon, UTMcoordinate_t* utmxy)
{
    double latitude = degree2radian(latlon->latitude);
    double longitude = degree2radian(latlon->longitude);
    int zone = (int)((latlon->longitude + 180.0) / 6) + 1;
    double UTMcentral_meridian = degree2radian(zone * 6.0 - 183);

    lanlon2utmxy(latitude, longitude, UTMcentral_meridian, utmxy);

    /* Adjust easting and northing for UTM system. */
    utmxy->x = utmxy->x * UTMScaleFactor + 500000.0;
    utmxy->y = utmxy->y * UTMScaleFactor;
    if (utmxy->y < 0.0)
        utmxy->y += 10000000.0;

    return 0;
}

int UTM_to_WGS84(UTMcoordinate_t* utmxy, WGS84coordinate_t* latlon, 
                 WGS84coordinate_t* last_latlon)   //无法计算跨时区的计算
{
    double x = utmxy->x;
    double y = utmxy->y;
    double last_latitude = last_latlon->latitude;
    double last_longitude = last_latlon->longitude;
    int zone = (int)((last_longitude + 180.0) / 6) + 1;
    double cmeridian = degree2radian(zone * 6.0 - 183);

    x -= 500000.0;
    x /= UTMScaleFactor;
    if(last_latitude < 0)   //南半球
        y -= 10000000.0;
    y /= UTMScaleFactor;

    utmxy2lanlon(x, y, cmeridian, latlon);

    return 0;
}




