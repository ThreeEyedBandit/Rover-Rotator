#pragma once

#include <stdint.h>

// Shared ESP-NOW payload. Display and BaseDriver must use this same layout.
// gs232Az/gs232El are raw controller angles (0 at the CCW hard limit).
// gs232Flags bit0 = azimuth valid, bit1 = elevation valid.
// resetAzHome is Display -> BaseDriver. azHomeEpoch increments when the
// base applies that reference reset so the display can drop the request.

#define GS232_FLAG_AZ 0x01
#define GS232_FLAG_EL 0x02

typedef struct dataPackage
{
  float latitude = -99.9999f;
  float longitude = -999.9999f;

  int dateYear = 0;
  int currentAltitude = 0;
  int currentAz = 0;
  int currentEl = 0;
  int gs232Az = 0;
  int gs232El = 0;

  uint8_t dateMonth = 0;
  uint8_t dateDay = 0;
  uint8_t timeHour = 0;
  uint8_t timeMinute = 0;
  uint8_t timeSecond = 0;
  uint8_t azSpeed = 0;
  uint8_t gs232Flags = 0;
  uint8_t azHomeEpoch = 0;

  bool left = false;
  bool right = false;
  bool resetAzHome = false;
}
dataPackage;
