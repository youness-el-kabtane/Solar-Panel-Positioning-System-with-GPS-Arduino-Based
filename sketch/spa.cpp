#include "spa.h"
#include <math.h>

#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG 57.2957795

SunPosition calculateSunPosition(
  int year, int month, int day,
  int hour, int minute, int second,
  double latitude, double longitude,
  double altitude,
  int timezone_offset
) {
  SunPosition result;

  // Convert time to fractional hour
  double timeUTC = hour + minute / 60.0 + second / 3600.0 - timezone_offset;

  // Calculate day of year
  int N1 = floor(275 * month / 9);
  int N2 = floor((month + 9) / 12);
  int K = 1 + floor((year - 4 * floor(year / 4) + 2) / 3);
  int N = N1 - (N2 * K) + day - 30;

  // Declination angle (δ)
  double decl = 23.45 * sin(DEG_TO_RAD * ((360.0 / 365.0) * (N - 81)));

  // Solar time correction
  double B = 360.0 / 365.0 * (N - 81);
  double EoT = 9.87 * sin(2 * DEG_TO_RAD * B) - 7.53 * cos(DEG_TO_RAD * B) - 1.5 * sin(DEG_TO_RAD * B);
  double solarTime = timeUTC * 60 + 4 * longitude + EoT;  // in minutes
  double hourAngle = (solarTime / 4.0) - 180.0;

  // Elevation angle
  double elevation = asin(
    sin(DEG_TO_RAD * decl) * sin(DEG_TO_RAD * latitude) +
    cos(DEG_TO_RAD * decl) * cos(DEG_TO_RAD * latitude) * cos(DEG_TO_RAD * hourAngle)
  ) * RAD_TO_DEG;

  // Azimuth angle
  double azimuth = acos(
    (sin(DEG_TO_RAD * decl) * cos(DEG_TO_RAD * latitude) -
     cos(DEG_TO_RAD * decl) * sin(DEG_TO_RAD * latitude) * cos(DEG_TO_RAD * hourAngle)) /
    cos(DEG_TO_RAD * elevation)
  ) * RAD_TO_DEG;

  if (hourAngle > 0) azimuth = 360 - azimuth;

  result.azimuth = azimuth;
  result.elevation = elevation;

  return result;
}
