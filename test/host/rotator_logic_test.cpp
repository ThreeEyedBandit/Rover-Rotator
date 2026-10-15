#include "azimuth.h"
#include "gs232b.h"
#include "ui_layout.h"

#include <stdio.h>
#include <string.h>

static int gFailures = 0;

static void expectTrue(bool cond, const char *name)
{
  if(cond) return;
  printf("FAIL %s\n", name);
  gFailures++;
}

static void expectEq(int got, int want, const char *name)
{
  if(got == want) return;
  printf("FAIL %s got %d want %d\n", name, got, want);
  gFailures++;
}

static void decodeLine(const char *frame, Gs232bPosition *pos, bool *ok)
{
  Gs232bParser parser;
  gs232bParserReset(&parser);
  char line[GS232B_LINE_MAX];
  *ok = false;
  for(int i = 0; frame[i]; i++)
  {
    if(gs232bPushByte(&parser, frame[i], line, GS232B_LINE_MAX))
      *ok = gs232bDecodePosition(line, pos);
  }
}

static void testPositionFrames()
{
  Gs232bPosition pos;
  bool ok = false;

  decodeLine("+0000\r", &pos, &ok);
  expectTrue(ok && pos.hasAz && !pos.hasEl, "+0000 decoded");
  expectEq(pos.az, 0, "+0000 az");

  decodeLine("+0450\r\n", &pos, &ok);
  expectTrue(ok && pos.hasAz && !pos.hasEl, "+0450 decoded");
  expectEq(pos.az, 450, "+0450 az");

  decodeLine("+0180+0090\r", &pos, &ok);
  expectTrue(ok && pos.hasAz && pos.hasEl, "+0180+0090 decoded");
  expectEq(pos.az, 180, "c2 az");
  expectEq(pos.el, 90, "c2 el");

  decodeLine("+0360+0000\r", &pos, &ok);
  expectTrue(ok, "overlap frame");
  expectEq(pos.az, 360, "360 stays 360");

  decodeLine("+0450+0180\r", &pos, &ok);
  expectTrue(ok, "cw stop with el");
  expectEq(pos.az, 450, "cw stop az");
  expectEq(pos.el, 180, "el 180");

  decodeLine("+450\r", &pos, &ok);
  expectTrue(ok && pos.hasAz, "calibration +450");
  expectEq(pos.az, 450, "+450 value");

  decodeLine("AZ=113 EL=030\r\n", &pos, &ok);
  expectTrue(ok && pos.hasAz && pos.hasEl, "AZ= EL=");
  expectEq(pos.az, 113, "AZ=113");
  expectEq(pos.el, 30, "EL=030");

  decodeLine("AZ=110  EL=032\r", &pos, &ok);
  expectTrue(ok && pos.hasEl, "double space EL");
  expectEq(pos.az, 110, "AZ=110");
  expectEq(pos.el, 32, "EL=032");

  decodeLine("az=015 el=010\r", &pos, &ok);
  expectTrue(ok, "lowercase az el");
  expectEq(pos.az, 15, "az 15");
  expectEq(pos.el, 10, "el 10");

  decodeLine("EL=045\r", &pos, &ok);
  expectTrue(ok && !pos.hasAz && pos.hasEl, "EL only");
  expectEq(pos.el, 45, "el 45");

  decodeLine("AZ=370\r", &pos, &ok);
  expectTrue(ok && !pos.hasEl, "AZ only overlap");
  expectEq(pos.az, 370, "370 not folded");

  decodeLine("+0451\r", &pos, &ok);
  expectTrue(!ok, "reject az 451");

  decodeLine("EL=181\r", &pos, &ok);
  expectTrue(!ok, "reject el 181");

  decodeLine("\r\n", &pos, &ok);
  expectTrue(!ok, "empty cr is not a position");

  decodeLine("? >\r", &pos, &ok);
  expectTrue(!ok, "error prompt ignored");

  decodeLine("Connect OK.\r", &pos, &ok);
  expectTrue(!ok, "connect text ignored");
}

static void testPartialAndReconnect()
{
  Gs232bParser parser;
  gs232bParserReset(&parser);
  char line[GS232B_LINE_MAX];
  const char *first = "+018";
  bool completed = false;
  for(int i = 0; first[i]; i++)
    completed = gs232bPushByte(&parser, first[i], line, GS232B_LINE_MAX);
  expectTrue(!completed, "partial does not complete");

  const char *rest = "0+0045\r";
  Gs232bPosition pos;
  bool decoded = false;
  for(int i = 0; rest[i]; i++)
  {
    if(gs232bPushByte(&parser, rest[i], line, GS232B_LINE_MAX))
      decoded = gs232bDecodePosition(line, &pos);
  }
  expectTrue(decoded && pos.hasAz && pos.hasEl, "split c2 completes");
  expectEq(pos.az, 180, "split az");
  expectEq(pos.el, 45, "split el");

  gs232bParserReset(&parser);
  gs232bPushByte(&parser, '+', line, GS232B_LINE_MAX);
  gs232bPushByte(&parser, '0', line, GS232B_LINE_MAX);
  gs232bPushByte(&parser, '1', line, GS232B_LINE_MAX);
  gs232bParserReset(&parser);
  bool stale = false;
  const char *next = "+0020\r";
  for(int i = 0; next[i]; i++)
  {
    if(gs232bPushByte(&parser, next[i], line, GS232B_LINE_MAX))
      stale = gs232bDecodePosition(line, &pos);
  }
  expectTrue(stale, "frame after reset");
  expectEq(pos.az, 20, "reset drops the partial");

  gs232bParserReset(&parser);
  for(int i = 0; i < 200; i++)
    gs232bPushByte(&parser, 'X', line, GS232B_LINE_MAX);
  bool overflowed = false;
  const char *after = "\r+0007\r";
  int seen = 0;
  for(int i = 0; after[i]; i++)
  {
    if(!gs232bPushByte(&parser, after[i], line, GS232B_LINE_MAX)) continue;
    overflowed = true;
    if(gs232bDecodePosition(line, &pos) && pos.hasAz) seen = pos.az;
  }
  expectTrue(overflowed, "resync after overflow");
  expectEq(seen, 7, "next good frame after overflow");
}

static void testCommands()
{
  char frame[16];
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "C2"), "format C2");
  expectTrue(strcmp(frame, "C2\r") == 0, "C2 cr");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "c"), "format c");
  expectTrue(strcmp(frame, "C\r") == 0, "C upper cr");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "P45"), "P45");
  expectTrue(strcmp(frame, "P45\r") == 0, "P45 frame");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "L"), "L");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "R"), "R");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "A"), "A");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "M180"), "M180");
  expectTrue(strcmp(frame, "M180\r") == 0, "M frame");
  expectTrue(gs232bFormatCommand(frame, (int)sizeof(frame), "W180 045"), "W");
  expectTrue(!gs232bFormatCommand(frame, (int)sizeof(frame), "M45"), "M needs 3 digits");
  expectTrue(!gs232bFormatCommand(frame, (int)sizeof(frame), "GOTO"), "unknown rejected");
}

static void testAzimuthRangeAndHome()
{
  for(int az = 0; az <= AZ_CW_LIMIT; az++)
    expectEq(azimuthFromAbsolute(az, 0), az, "absolute pass-through");

  expectEq(azimuthFromAbsolute(10, 0), 10, "heading 10");
  expectEq(azimuthFromAbsolute(370, 0), 370, "overlap 370");
  expectTrue(azimuthFromAbsolute(10, 0) != azimuthFromAbsolute(370, 0), "overlap distinct");
  expectEq(azimuthFromAbsolute(370, 370), AZ_HOME, "absolute reset to home");
  expectEq(azimuthFromAbsolute(400, 370), 30, "travel after absolute home");
  expectEq(azimuthFromAbsolute(10, 370), AZ_HOME, "below home clamps");
  expectEq(azimuthFromAbsolute(900, 0), AZ_CW_LIMIT, "above cw clamps");

  AzimuthTracker tracker;
  azimuthTrackerInit(&tracker);
  azimuthApplyRaw(&tracker, 90);
  azimuthResetToHome(&tracker);
  expectEq(tracker.mechanical, AZ_HOME, "reset reads 0");
  expectTrue(tracker.homed, "reset marks homed");

  int sensor = 90;
  for(int step = 10; step <= 450; step += 10)
  {
    sensor = azimuthNormalize360(sensor + 10);
    azimuthApplyRaw(&tracker, sensor);
    expectEq(tracker.mechanical, step, "unwrapped travel");
  }
  expectEq(tracker.mechanical, 450, "cw hard stop");
  sensor = azimuthNormalize360(sensor + 10);
  azimuthApplyRaw(&tracker, sensor);
  expectEq(tracker.mechanical, 450, "no travel past 450");

  sensor = azimuthNormalize360(sensor - 10);
  azimuthApplyRaw(&tracker, sensor);
  expectEq(tracker.mechanical, 440, "reverse off the cw stop");

  expectTrue(!azimuthBlockCcw(false, AZ_HOME), "no ccw block before home");
  expectTrue(azimuthBlockCcw(true, AZ_HOME), "ccw blocked at home");
  expectTrue(!azimuthBlockCcw(true, 1), "ccw open above home");
  expectTrue(azimuthBlockCw(true, AZ_CW_LIMIT), "cw blocked at 450");
  expectTrue(!azimuthBlockCw(true, 400), "overlap is not a stop");
  expectTrue(!azimuthBlockCw(true, 360), "360 is not a stop");

  AzimuthTracker approach;
  azimuthTrackerInit(&approach);
  azimuthApplyRaw(&approach, 100);
  azimuthApplyRaw(&approach, 70);
  expectEq(approach.mechanical, AZ_HOME, "ccw before home stays at 0");
  expectTrue(!approach.homed, "limits wait for the reset");
  azimuthResetToHome(&approach);
  expectEq(approach.mechanical, AZ_HOME, "stop becomes home");
  azimuthApplyRaw(&approach, 100);
  expectEq(approach.mechanical, 30, "cw travel is measured from the stop");
}

static void testHomeButtonGeometry()
{
  expectTrue(UI_HOME_Y >= UI_EL_Y + UI_EL_H, "button is under elevation");
  expectTrue(UI_HOME_Y < UI_EL_Y + UI_EL_H + 24, "button is directly under elevation");
  expectTrue(UI_HOME_X >= 0 && UI_HOME_X + UI_HOME_W <= UI_SCREEN_W, "button on screen");
  expectTrue(UI_HOME_Y + UI_HOME_H < 400, "button above the grid line");
  expectTrue(strcmp(UI_HOME_LABEL, "Reset azimuth to home") == 0, "button label");

  expectEq((int)uiTouchZone(UI_HOME_X + 4, UI_HOME_Y + 4), (int)TOUCH_ZONE_HOME, "left side of button");
  expectEq((int)uiTouchZone(UI_HOME_X + UI_HOME_W - 2, UI_HOME_Y + UI_HOME_H / 2), (int)TOUCH_ZONE_HOME, "right side of button");
  expectEq((int)uiTouchZone(10, 100), (int)TOUCH_ZONE_LEFT, "left slew");
  expectEq((int)uiTouchZone(300, 80), (int)TOUCH_ZONE_RIGHT, "right slew");
  expectEq((int)uiTouchZone(UI_MID_X, 100), (int)TOUCH_ZONE_NONE, "center gap");
  expectTrue(uiTouchZone(20, UI_HOME_Y - 2) != TOUCH_ZONE_HOME, "just above button is not home");
}

int main()
{
  testPositionFrames();
  testPartialAndReconnect();
  testCommands();
  testAzimuthRangeAndHome();
  testHomeButtonGeometry();
  if(gFailures)
  {
    printf("%d failed\n", gFailures);
    return 1;
  }
  printf("ok\n");
  return 0;
}
