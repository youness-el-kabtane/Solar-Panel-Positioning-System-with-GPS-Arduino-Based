#ifndef SPA_H
#define SPA_H

struct SunPosition {
  double azimuth;
  double elevation;
};

SunPosition calculateSunPosition(
  int year, int month, int day,
  int hour, int minute, int second,
  double latitude, double longitude,
  double altitude,
  int timezone_offset
);

#endif
