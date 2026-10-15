#pragma once

// 320x480 portrait UI. Azimuth and elevation values use FreeSerif 24pt at
// text size 2. Digit ink ends 70px under the top datum (ascent 33, digit
// height 33, yOffset -31: (33 - 31 + 33) * 2). The home control sits in the
// band directly under that elevation ink. Touch X already uses panel pixels
// (left of 160 / right of 160); touch Y uses the same pixel space.

static const int UI_SCREEN_W = 320;
static const int UI_SCREEN_H = 480;
static const int UI_MID_X = 160;

static const int UI_AZ_Y = 53;
static const int UI_EL_Y = 146;
static const int UI_EL_H = 70;
static const int UI_HOME_GAP = 8;
static const int UI_HOME_X = 16;
static const int UI_HOME_Y = UI_EL_Y + UI_EL_H + UI_HOME_GAP;
static const int UI_HOME_W = 288;
static const int UI_HOME_H = 44;

static const char UI_HOME_LABEL[] = "Reset azimuth to home";

enum TouchZone
{
  TOUCH_ZONE_NONE = 0,
  TOUCH_ZONE_LEFT,
  TOUCH_ZONE_RIGHT,
  TOUCH_ZONE_HOME
};

inline bool uiRectContains(int x, int y, int rx, int ry, int rw, int rh)
{
  return x >= rx && y >= ry && x < (rx + rw) && y < (ry + rh);
}

// Home wins over the full-screen left/right slew zones.
inline TouchZone uiTouchZone(int x, int y)
{
  if(uiRectContains(x, y, UI_HOME_X, UI_HOME_Y, UI_HOME_W, UI_HOME_H))
    return TOUCH_ZONE_HOME;
  if(x < UI_MID_X) return TOUCH_ZONE_LEFT;
  if(x > UI_MID_X) return TOUCH_ZONE_RIGHT;
  return TOUCH_ZONE_NONE;
}
