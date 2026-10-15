#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>
#include "esp_arduino_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FT6336U.h"
#include "Free_Fonts.h"
#include "data_package.h"
#include "azimuth.h"
#include "gs232b.h"
#include "ui_layout.h"

#define txPacketInterval 10
#define rxTimeoutInterval 200
#define displayUpdateInterval 50
#define buttonPressInterval 20
#define touchReleaseHoldMs 80
#define gs232PollMs 200
#define gs232StaleMs 1000

// Bluetooth serial peer. The ESP32 connects out to an adapter with this name
// (HC-05 default PIN 1234). If that inquiry fails, it also listens as
// "Rover-Rotator" so a master adapter can connect in.
#define GS232B_BT_NAME "GS232B"

#define I2C_SDA 18
#define I2C_SCL 19
#define RST_N_PIN 0
#define INT_N_PIN 0

uint8_t remoteAddress[] = {0x58, 0xCF, 0x79, 0xA5, 0xE3, 0xD8}; // BaseDriver

TFT_eSPI display = TFT_eSPI();
TFT_eSprite tempSprite = TFT_eSprite(&display);
FT6336U ft6336u(I2C_SDA, I2C_SCL, RST_N_PIN, INT_N_PIN);
TinyGPSPlus gps;
BluetoothSerial serialBt;
esp_now_peer_info_t peerInfo;

dataPackage localData;
dataPackage remoteData;

bool blnConnected = false;
bool blnTouched = false;
bool blnTouchReleasing = false;
bool blnHomeDown = false;
bool blnHomeResetPending = false;

uint8_t homeEpochSnapshot = 0;

unsigned long rxTimeoutStart = 0;
unsigned long buttonPressStart = 0;
unsigned long txPacketStart = 0;
unsigned long displayUpdateStart = 0;
unsigned long touchReleaseStart = 0;

String currentGrid = "******";

struct Gs232bSample
{
  int az;
  int el;
  uint8_t flags;
  unsigned long ms;
  int8_t wantDir;
};

static Gs232bSample gsSample;
static portMUX_TYPE gsMux = portMUX_INITIALIZER_UNLOCKED;

void updateDisplay();
void checkGPSInfo();
String toGrid(long tmpLat, long tmpLon);
static void gs232BtTask(void *arg);

static void requestAzHomeReset()
{
  if(blnHomeResetPending) return;
  blnHomeResetPending = true;
  homeEpochSnapshot = remoteData.azHomeEpoch;
  localData.resetAzHome = true;
}

static void gs232Publish(const Gs232bPosition *pos)
{
  portENTER_CRITICAL(&gsMux);
  if(pos->hasAz)
  {
    gsSample.az = pos->az;
    gsSample.flags = (uint8_t)(gsSample.flags | GS232_FLAG_AZ);
  }
  if(pos->hasEl)
  {
    gsSample.el = pos->el;
    gsSample.flags = (uint8_t)(gsSample.flags | GS232_FLAG_EL);
  }
  gsSample.ms = millis();
  portEXIT_CRITICAL(&gsMux);
}

static void gs232ClearLink()
{
  portENTER_CRITICAL(&gsMux);
  gsSample.flags = 0;
  portEXIT_CRITICAL(&gsMux);
}

static void gs232WriteCmd(const char *cmd)
{
  char frame[12];
  if(!gs232bFormatCommand(frame, (int)sizeof(frame), cmd)) return;
  serialBt.print(frame);
}

static bool gs232BtBegin(bool master)
{
  if(!serialBt.begin("Rover-Rotator", master)) return false;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  serialBt.setPin("1234", 4);
#else
  serialBt.setPin("1234");
#endif
  return true;
}

// Owns the SPP link, the CR reassembly, and reconnect. Blocking name inquiry
// stays on this task so the touch UI keeps running.
static void gs232BtTask(void *arg)
{
  (void)arg;
  Gs232bParser parser;
  gs232bParserReset(&parser);

  bool master = true;
  bool btUp = false;
  bool link = false;
  int misses = 0;
  int8_t sentDir = 0;
  unsigned long lastPoll = 0;
  unsigned long modeWaitUntil = 0;

  for(;;)
  {
    if(!btUp)
    {
      btUp = gs232BtBegin(master);
      if(!btUp)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    if(!serialBt.hasClient())
    {
      if(link)
      {
        link = false;
        gs232bParserReset(&parser);
        gs232ClearLink();
        sentDir = 0;
      }

      if(master)
      {
        if(!serialBt.connect(GS232B_BT_NAME))
        {
          misses++;
          if(misses >= 2)
          {
            serialBt.end();
            btUp = false;
            master = false;
            misses = 0;
            vTaskDelay(pdMS_TO_TICKS(300));
          }
          continue;
        }
        misses = 0;
      }
      else
      {
        unsigned long opened = millis();
        while(!serialBt.hasClient() && (millis() - opened < 20000))
          vTaskDelay(pdMS_TO_TICKS(50));
        if(!serialBt.hasClient())
        {
          serialBt.end();
          btUp = false;
          master = true;
          vTaskDelay(pdMS_TO_TICKS(300));
          continue;
        }
      }
    }

    if(!link)
    {
      link = true;
      misses = 0;
      gs232bParserReset(&parser);
      gs232ClearLink();
      // 450° mode keeps the 90° overlap. P45 is a mode switch, not a slew.
      gs232WriteCmd("P45");
      modeWaitUntil = millis() + 50;
      lastPoll = 0;
      sentDir = 99;
    }

    while(serialBt.available() > 0)
    {
      char line[GS232B_LINE_MAX];
      int raw = serialBt.read();
      if(raw < 0) break;
      if(!gs232bPushByte(&parser, raw, line, GS232B_LINE_MAX)) continue;
      Gs232bPosition pos;
      if(!gs232bDecodePosition(line, &pos)) continue;
      gs232Publish(&pos);
    }

    if((long)(millis() - modeWaitUntil) >= 0)
    {
      int8_t want;
      portENTER_CRITICAL(&gsMux);
      want = gsSample.wantDir;
      portEXIT_CRITICAL(&gsMux);

      if(want != sentDir)
      {
        if(want < 0) gs232WriteCmd("L");
        else if(want > 0) gs232WriteCmd("R");
        else gs232WriteCmd("A");
        sentDir = want;
      }

      if(millis() - lastPoll >= gs232PollMs)
      {
        gs232WriteCmd("C2");
        lastPoll = millis();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void gs232CopyToPacket()
{
  Gs232bSample sample;
  portENTER_CRITICAL(&gsMux);
  sample = gsSample;
  portEXIT_CRITICAL(&gsMux);

  if(sample.flags && (millis() - sample.ms > gs232StaleMs))
    sample.flags = 0;

  localData.gs232Az = sample.az;
  localData.gs232El = sample.el;
  localData.gs232Flags = sample.flags;

  int8_t dir = 0;
  if(localData.left && !localData.right) dir = -1;
  else if(localData.right && !localData.left) dir = 1;

  portENTER_CRITICAL(&gsMux);
  gsSample.wantDir = dir;
  portEXIT_CRITICAL(&gsMux);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  (void)info;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  (void)mac;
#endif
  if(len < (int)sizeof(remoteData)) return;
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  rxTimeoutStart = millis();
  localData.currentAz = remoteData.currentAz;
  localData.currentEl = remoteData.currentEl;
}

void setup()
{
  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(OnDataRecv);

  Serial2.begin(115200, SERIAL_8N1, 33, 14);

  display.init();
  display.setRotation(0);
  ft6336u.begin();
  updateDisplay();

  xTaskCreate(gs232BtTask, "gs232bt", 16384, NULL, 1, NULL);
}

void loop()
{
  if(millis() - buttonPressStart >= buttonPressInterval)
  {
    buttonPressStart = millis();

    if(ft6336u.read_touch1_event() == 0x02)
    {
      uint16_t touchX = ft6336u.read_touch1_x();
      uint16_t touchY = ft6336u.read_touch1_y();
      TouchZone zone = uiTouchZone((int)touchX, (int)touchY);
      blnTouched = true;
      blnTouchReleasing = false;

      if(zone == TOUCH_ZONE_HOME)
      {
        localData.left = false;
        localData.right = false;
        if(!blnHomeDown)
        {
          blnHomeDown = true;
          requestAzHomeReset();
        }
      }
      else
      {
        blnHomeDown = false;
        localData.left = (zone == TOUCH_ZONE_LEFT);
        localData.right = (zone == TOUCH_ZONE_RIGHT);
      }
    }
    else if(blnTouched)
    {
      if(!blnTouchReleasing)
      {
        blnTouchReleasing = true;
        touchReleaseStart = millis();
      }
      else if(millis() - touchReleaseStart >= touchReleaseHoldMs)
      {
        blnTouched = false;
        blnTouchReleasing = false;
        blnHomeDown = false;
        localData.left = false;
        localData.right = false;
      }
    }
  }

  while(Serial2.available() > 0)
  {
    if(gps.encode(Serial2.read()))
      checkGPSInfo();
  }

  blnConnected = (millis() - rxTimeoutStart < rxTimeoutInterval);

  if(blnHomeResetPending && remoteData.azHomeEpoch != homeEpochSnapshot)
  {
    blnHomeResetPending = false;
    localData.resetAzHome = false;
  }

  gs232CopyToPacket();

  if(millis() - txPacketStart >= txPacketInterval)
  {
    esp_now_send(remoteAddress, (uint8_t *)&localData, sizeof(localData));
    txPacketStart = millis();
  }

  if(millis() - displayUpdateStart >= displayUpdateInterval)
  {
    updateDisplay();
    displayUpdateStart = millis();
  }
}

static void drawHomeButton()
{
  int fill = blnHomeDown ? 2 : 0;
  int cx = UI_HOME_X + (UI_HOME_W / 2);
  int cy = UI_HOME_Y + (UI_HOME_H / 2);

  tempSprite.fillRoundRect(UI_HOME_X, UI_HOME_Y, UI_HOME_W, UI_HOME_H, 6, fill);
  tempSprite.drawRoundRect(UI_HOME_X, UI_HOME_Y, UI_HOME_W, UI_HOME_H, 6, 1);
  tempSprite.setTextSize(1);
  tempSprite.setFreeFont(FSS12);
  tempSprite.setTextDatum(MC_DATUM);
  tempSprite.setTextColor(1, fill);
  tempSprite.drawString(UI_HOME_LABEL, cx, cy, GFXFF);
}

void updateDisplay()
{
  uint16_t cmap[16] = {
    TFT_BLACK,
    TFT_WHITE,
    TFT_GREEN,
    TFT_RED,
    tempSprite.color565(0, remoteData.azSpeed, 0)
  };

  String tempDate;
  String tempTime;
  String tempAlt;
  int shownAz = blnHomeResetPending ? AZ_HOME : remoteData.currentAz;

  tempSprite.setColorDepth(4);
  tempSprite.createSprite(UI_SCREEN_W, UI_SCREEN_H);
  tempSprite.createPalette(cmap);
  tempSprite.fillSprite(0);

  tempSprite.setTextSize(2);
  tempSprite.setFreeFont(FS24);
  tempSprite.setTextDatum(TC_DATUM);
  tempSprite.setTextColor(1, 0);
  tempSprite.drawNumber(shownAz, UI_MID_X, UI_AZ_Y);
  tempSprite.drawNumber(remoteData.currentEl, UI_MID_X, UI_EL_Y);

  tempSprite.setFreeFont(FS18);
  tempSprite.setTextDatum(BC_DATUM);
  tempSprite.setTextColor(1, 0);
  tempSprite.drawString(currentGrid, UI_MID_X, 420, GFXFF);

  tempSprite.setTextSize(1);
  tempSprite.setFreeFont(FS18);
  tempSprite.setTextDatum(TL_DATUM);

  if(localData.dateMonth < 10) tempDate += "0";
  tempDate += String(localData.dateMonth) + "/";
  if(localData.dateDay < 10) tempDate += "0";
  tempDate += String(localData.dateDay) + "/";
  if(localData.dateYear < 10) tempDate += "0";
  tempDate += String(localData.dateYear);

  if(localData.timeHour < 10) tempTime += "0";
  tempTime += String(localData.timeHour) + ":";
  if(localData.timeMinute < 10) tempTime += "0";
  tempTime += String(localData.timeMinute) + ":";
  if(localData.timeSecond < 10) tempTime += "0";
  tempTime += String(localData.timeSecond) + "z";

  tempSprite.drawString(tempTime, 0, 0, GFXFF);
  tempSprite.drawString(tempDate, UI_SCREEN_W - tempSprite.textWidth(tempDate), 0, GFXFF);

  tempSprite.setTextDatum(BL_DATUM);
  tempAlt = String(localData.currentAltitude) + "'";
  tempSprite.drawString(tempAlt, 0, 450, GFXFF);
  tempSprite.drawFloat(localData.latitude, 4, 0, 484, GFXFF);

  tempSprite.setTextDatum(BR_DATUM);
  tempSprite.drawFloat(localData.longitude, 4, UI_SCREEN_W, 484, GFXFF);

  tempSprite.fillCircle(156, 466, 12, blnConnected ? 2 : 3);
  tempSprite.fillTriangle(0, 89, 45, 110, 45, 68,
    (remoteData.left && blnConnected && remoteData.azSpeed) ? 4 : 3);
  tempSprite.fillTriangle(UI_SCREEN_W, 89, 275, 110, 275, 68,
    (remoteData.right && blnConnected && remoteData.azSpeed) ? 4 : 3);

  drawHomeButton();
  tempSprite.pushSprite(0, 0);
  tempSprite.deleteSprite();
}

void checkGPSInfo()
{
  if(gps.date.isUpdated())
  {
    localData.dateMonth = gps.date.month();
    localData.dateDay = gps.date.day();
    localData.dateYear = gps.date.year() - 2000;
  }
  if(gps.time.isUpdated())
  {
    localData.timeHour = gps.time.hour();
    localData.timeMinute = gps.time.minute();
    localData.timeSecond = gps.time.second();
  }
  if(gps.altitude.isUpdated())
    localData.currentAltitude = gps.altitude.feet();
  if(gps.location.isUpdated())
  {
    localData.latitude = gps.location.lat();
    localData.longitude = gps.location.lng();
    currentGrid = toGrid(localData.latitude * 100000, localData.longitude * 100000);
  }
}

String toGrid(long tmpLat, long tmpLon)
{
  tmpLon += 18000000;
  tmpLat += 9000000;
  char GS[7] = {'A', 'A', '0', '0', 'a', 'a', '\0'};
  GS[0] += tmpLon / 2000000;
  GS[1] += tmpLat / 1000000;
  GS[2] += (tmpLon % 2000000) / 200000;
  GS[3] += (tmpLat % 1000000) / 100000;
  GS[4] += (tmpLon % 200000) / 8333;
  GS[5] += (tmpLat % 100000) / 4166;
  return String(GS);
}
