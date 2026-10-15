#pragma once

// Mechanical azimuth for a 450° rotator.
// 0° is the counterclockwise hard limit (home). 450° is the clockwise hard
// limit. The extra 90° past 360° is real travel: 370° is not folded back to
// 10°. A heading in that overlap is a different stop position than the
// matching 0°..90° heading on the other side of north.

static const int AZ_HOME = 0;
static const int AZ_CW_LIMIT = 450;
static const int AZ_OVERLAP_DEGREES = 90;

struct AzimuthTracker
{
  long unwrapped;
  long homeOffset;
  int lastRaw;
  int mechanical;
  bool haveSample;
  bool homed;
};

inline int azimuthNormalize360(int deg)
{
  deg %= 360;
  if(deg < 0) deg += 360;
  return deg;
}

inline void azimuthTrackerInit(AzimuthTracker *tracker)
{
  tracker->unwrapped = 0;
  tracker->homeOffset = 0;
  tracker->lastRaw = 0;
  tracker->mechanical = AZ_HOME;
  tracker->haveSample = false;
  tracker->homed = false;
}

inline void azimuthPublish(AzimuthTracker *tracker)
{
  long mechanical = tracker->unwrapped - tracker->homeOffset;

  if(tracker->homed)
  {
    if(mechanical < AZ_HOME)
    {
      mechanical = AZ_HOME;
      tracker->unwrapped = tracker->homeOffset;
    }
    else if(mechanical > AZ_CW_LIMIT)
    {
      mechanical = AZ_CW_LIMIT;
      tracker->unwrapped = tracker->homeOffset + AZ_CW_LIMIT;
    }
  }

  if(mechanical < AZ_HOME) mechanical = AZ_HOME;
  else if(mechanical > AZ_CW_LIMIT) mechanical = AZ_CW_LIMIT;
  tracker->mechanical = (int)mechanical;
}

// Fold a 0..359 sensor sample into the 0..450 mechanical reading.
// The first sample sets the origin. Later samples accumulate the short
// delta so travel through north does not wrap back to 0.
inline void azimuthApplyRaw(AzimuthTracker *tracker, int raw)
{
  raw = azimuthNormalize360(raw);
  if(!tracker->haveSample)
  {
    tracker->lastRaw = raw;
    tracker->unwrapped = 0;
    tracker->homeOffset = 0;
    tracker->haveSample = true;
    tracker->mechanical = AZ_HOME;
    return;
  }

  int delta = raw - tracker->lastRaw;
  if(delta > 180) delta -= 360;
  else if(delta < -180) delta += 360;
  tracker->lastRaw = raw;
  tracker->unwrapped += delta;
  azimuthPublish(tracker);
}

// Reference reset: the position we are at now reads as home (0°).
// This does not move the rotator.
inline void azimuthResetToHome(AzimuthTracker *tracker)
{
  tracker->homeOffset = tracker->unwrapped;
  tracker->homed = true;
  tracker->mechanical = AZ_HOME;
}

// Controller angles are already mechanical (GS-232B 450° mode: 0 = CCW stop).
// zeroOffset is the raw angle captured by a home reset.
inline int azimuthFromAbsolute(int reported, int zeroOffset)
{
  long mechanical = (long)reported - (long)zeroOffset;
  if(mechanical < AZ_HOME) mechanical = AZ_HOME;
  if(mechanical > AZ_CW_LIMIT) mechanical = AZ_CW_LIMIT;
  return (int)mechanical;
}

inline bool azimuthBlockCcw(bool enforceLimits, int mechanical)
{
  return enforceLimits && mechanical <= AZ_HOME;
}

inline bool azimuthBlockCw(bool enforceLimits, int mechanical)
{
  return enforceLimits && mechanical >= AZ_CW_LIMIT;
}
