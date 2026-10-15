#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_now.h>
#include "FT6336U.h"
#include "Free_Fonts.h"

#define txPacketInterval 10
#define rxTimeoutInterval 200
#define displayUpdateInterval 50
#define buttonPressInterval 20
#define touchReleaseHoldMs 80

#define I2C_SDA 18
#define I2C_SCL 19
#define RST_N_PIN 0
#define INT_N_PIN 0

uint8_t remoteAddress[] = {0x58, 0xCF, 0x79, 0xA5, 0xE3, 0xD8}; // BaseDriver

TFT_eSPI display = TFT_eSPI();
TFT_eSprite tempSprite = TFT_eSprite(&display);
FT6336U ft6336u(I2C_SDA, I2C_SCL, RST_N_PIN, INT_N_PIN);
TinyGPSPlus gps;
esp_now_peer_info_t peerInfo;

typedef struct dataPackage
{
  float latitude = -99.9999;
  float longitude = -999.9999;

  int dateYear = 0;
  int currentAltitude = 0;
  int currentAz = 0;
  int currentEl = 0;

  byte dateMonth = 0;
  byte dateDay = 0;
  byte timeHour = 0;
  byte timeMinute = 0;
  byte timeSecond = 0;
  byte azSpeed = 0;

  bool left;
  bool right;
}
dataPackage;

dataPackage localData;
dataPackage remoteData;

bool blnConnected = false;
bool blnTouched = false;
bool blnTouchReleasing = false;

unsigned long rxTimeoutStart = 0;
unsigned long buttonPressStart = 0;
unsigned long txPacketStart = 0;
unsigned long displayUpdateStart = 0;
unsigned long touchReleaseStart = 0;

String currentGrid = "******";

void updateDisplay();
void checkGPSInfo();
String toGrid(long tmpLat, long tmpLon);

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
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
}

void loop()
{
  if(millis() - buttonPressStart >= buttonPressInterval)
  {
    buttonPressStart = millis();

    if(ft6336u.read_touch1_event() == 0x02)
    {
      uint16_t touchX = ft6336u.read_touch1_x();
      localData.left = (touchX < 160);
      localData.right = (touchX > 160);
      blnTouched = true;
      blnTouchReleasing = false;
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

  tempSprite.setColorDepth(4);
  tempSprite.createSprite(320, 480);
  tempSprite.createPalette(cmap);
  tempSprite.fillSprite(0);

  tempSprite.setTextSize(2);
  tempSprite.setFreeFont(FS24);
  tempSprite.setTextDatum(TC_DATUM);
  tempSprite.setTextColor(1, 0);
  tempSprite.drawNumber(remoteData.currentAz, 160, 53);
  tempSprite.drawNumber(remoteData.currentEl, 160, 146);

  tempSprite.setFreeFont(FS18);
  tempSprite.setTextDatum(BC_DATUM);
  tempSprite.drawString(currentGrid, 160, 420, GFXFF);

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
  tempSprite.drawString(tempDate, 320 - tempSprite.textWidth(tempDate), 0, GFXFF);

  tempSprite.setTextDatum(BL_DATUM);
  tempAlt = String(localData.currentAltitude) + "'";
  tempSprite.drawString(tempAlt, 0, 450, GFXFF);
  tempSprite.drawFloat(localData.latitude, 4, 0, 484, GFXFF);

  tempSprite.setTextDatum(BR_DATUM);
  tempSprite.drawFloat(localData.longitude, 4, 320, 484, GFXFF);

  tempSprite.fillCircle(156, 466, 12, blnConnected ? 2 : 3);
  tempSprite.fillTriangle(0, 89, 45, 110, 45, 68,
    (remoteData.left && blnConnected && remoteData.azSpeed) ? 4 : 3);
  tempSprite.fillTriangle(320, 89, 275, 110, 275, 68,
    (remoteData.right && blnConnected && remoteData.azSpeed) ? 4 : 3);

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
