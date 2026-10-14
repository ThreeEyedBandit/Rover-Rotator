//Battery Voltage and RSSI both ends

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <Wifi.h>
#include <esp_now.h>
#include "FT6336U.h"
#include "Free_Fonts.h"

#define txPacketInterval 10
#define rxTimeoutInterval 200
#define displayUpdateInterval 50
#define buttonPressInterval 20

#define I2C_SDA 18
#define I2C_SCL 19
#define RST_N_PIN 0
#define INT_N_PIN 0


//uint8_t remoteAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //All

//uint8_t remoteAddress[] = {0xE8, 0x31, 0xCD, 0x18, 0xDF, 0x98}; //Display
uint8_t remoteAddress[] = {0x58, 0xCF, 0x79, 0xA5, 0xE3, 0xD8}; //New Driver

TFT_eSPI display = TFT_eSPI();
TFT_eSprite tempSprite = TFT_eSprite(&display);

FT6336U ft6336u(I2C_SDA, I2C_SCL, RST_N_PIN, INT_N_PIN);

long backColor = TFT_BLACK; long foreColor = TFT_WHITE;

boolean blnTouched = false;

unsigned long rxTimeoutStart = millis();
unsigned long buttonPressStart = millis();
unsigned long txPacketStart = millis();
unsigned long displayUpdateStart = millis();
//unsigned long lastButtonPass = millis();

String currentGrid = "******";

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

void updateDisplay();void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);void checkGPSInfo();
String toGrid(long tmpLat, long tmpLon);

esp_now_peer_info_t peerInfo;
TinyGPSPlus gps;

void setup() {
  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 33, 14); //GPS

  Serial.print("MAC Address:  ");
  Serial.println(WiFi.macAddress());

  display.init();
  display.setRotation(0);

  //pinMode(34,INPUT);
  //pinMode(35,INPUT);

  ft6336u.begin();
  updateDisplay();
}
void loop() 
{  
  if(millis() - buttonPressStart >= buttonPressInterval){
    if(ft6336u.read_touch1_event() == 0x02){
      if(!blnTouched){
        if(ft6336u.read_touch1_x() < 160){
        localData.left = true;
        }
        else{
          localData.left = false;
        }

        if(ft6336u.read_touch1_x() > 160){
          localData.right = true;
        }
        else{
          localData.right = false;
        }
      blnTouched = true;   
      } 
    }
    else{
    blnTouched = false;
      if(localData.left)localData.left = false;
      if(localData.right)localData.right = false;
    }
  }

  /*
  int tempTime = millis() - lastButtonPass;
  lastButtonPass = millis();
  Serial.print(localData.left);
  Serial.print(" : ");
  Serial.print(localData.right);
  Serial.print (" : ");
  Serial.print(blnTouched);
  Serial.print(" : ");
  Serial.println(tempTime);*/

  while(Serial2.available() > 0) {char charTemp1 = Serial2.read();if(gps.encode(charTemp1)) checkGPSInfo();}

  if(millis() - rxTimeoutStart >= rxTimeoutInterval){blnConnected = false;}else{blnConnected = true;}  
  if(millis() - txPacketStart >= txPacketInterval)esp_now_send(remoteAddress, (uint8_t *) &localData, sizeof(localData));
  if(millis() - displayUpdateStart >= displayUpdateInterval){updateDisplay();displayUpdateStart = millis();}
}
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) 
{
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  rxTimeoutStart = millis();
  txPacketStart = millis();

  if(remoteData.currentAz != localData.currentAz)
  {
    localData.currentAz = remoteData.currentAz;
  }

  if(remoteData.currentEl != localData.currentEl)
  {
    localData.currentEl = remoteData.currentEl;
  }
}
void updateDisplay()
{
  uint16_t cmap[16];
  
  cmap[0]  = backColor;
  cmap[1]  = foreColor;
  cmap[2]  = TFT_GREEN;
  cmap[3]  = TFT_RED;
  cmap[4]  = tempSprite.color565(0,remoteData.azSpeed,0);
  cmap[5]  = TFT_PURPLE;
  cmap[6]  = TFT_PINK;
  cmap[7]  = TFT_LIGHTGREY;
  cmap[8]  = TFT_YELLOW;
  cmap[9]  = TFT_BLUE;
  cmap[10] = TFT_DARKCYAN;
  cmap[11] = TFT_CYAN;
  cmap[12] = TFT_DARKGREEN;
  cmap[13] = TFT_MAGENTA;
  cmap[14] = TFT_NAVY;
  cmap[15] = TFT_BLUE; 

  String tempDate;
  String tempTime;
  String tempAlt;

  tempSprite.setColorDepth(4);
  tempSprite.createSprite(320,480);
  tempSprite.createPalette(cmap);

  tempSprite.fillSprite(0);

  tempSprite.setTextSize(2);
  tempSprite.setFreeFont(FS24);
  tempSprite.setTextDatum(TC_DATUM);
  tempSprite.setTextColor(1, 0);
  tempSprite.drawNumber(remoteData.currentAz,160,53);
  tempSprite.drawNumber(remoteData.currentEl,160,146);

  tempSprite.setFreeFont(FS18);
  tempSprite.setTextDatum(BC_DATUM);
  tempSprite.drawString(currentGrid,160,420,GFXFF);

  tempSprite.setTextSize(1);
  tempSprite.setFreeFont(FS18);
  tempSprite.setTextDatum(TL_DATUM);
  if(localData.dateMonth < 10)
    tempDate = tempDate + "0";
  tempDate = tempDate + localData.dateMonth + "/";
  if(localData.dateDay < 10)
    tempDate = tempDate + "0";
  tempDate = tempDate + localData.dateDay + "/";
  if(localData.dateYear < 10)
    tempDate = tempDate + "0";
  tempDate = tempDate + localData.dateYear;

  if(localData.timeHour < 10)
    tempTime = tempTime + "0";
  tempTime = tempTime + localData.timeHour + ":";
  if(localData.timeMinute < 10)
    tempTime = tempTime + "0";
  tempTime = tempTime + localData.timeMinute + ":";
  if(localData.timeSecond < 10)
    tempTime = tempTime + "0";
  tempTime = tempTime + localData.timeSecond + "z";
  tempSprite.drawString(tempTime,0,0,GFXFF);
  tempSprite.drawString(tempDate,320-(tempSprite.textWidth(tempDate)),0,GFXFF);

  //tempSprite.setTextSize(2);
  //tempSprite.setTextDatum(TC_DATUM);
  //tempSprite.drawString(tempTime,160,290,GFXFF);


  //tempSprite.setTextSize(1);

  tempSprite.setTextDatum(BL_DATUM);
  tempAlt = tempAlt + localData.currentAltitude + "'";
  tempSprite.drawString(tempAlt,0,450,GFXFF);

  tempSprite.drawFloat(localData.latitude,4,0,484,GFXFF);

  tempSprite.setTextDatum(BR_DATUM);
  tempSprite.drawFloat(localData.longitude,4,320,484,GFXFF);

  if(blnConnected)
    tempSprite.fillCircle(156,466,12,2);
  else
    tempSprite.fillCircle(156,466,12,3);

  if(remoteData.left && blnConnected && remoteData.azSpeed != 0)
    tempSprite.fillTriangle(0,89,45,110,45,68,4);
  else
    tempSprite.fillTriangle(0,89,45,110,45,68,3);

  if(remoteData.right && blnConnected && remoteData.azSpeed != 0)
    tempSprite.fillTriangle(320,89,275,110,275,68,4);
  else
    tempSprite.fillTriangle(320,89,275,110,275,68,3);  

  tempSprite.pushSprite(0,0);
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
  {
    localData.currentAltitude = gps.altitude.feet();
  }
  if(gps.location.isUpdated())
  {
    localData.latitude = gps.location.lat();
    localData.longitude = gps.location.lng();
    currentGrid = toGrid(localData.latitude * 100000, localData.longitude * 100000);
  }
}
String toGrid(long tmpLat, long tmpLon)
{
  String strTemp;tmpLon = tmpLon + 18000000;
  tmpLat = tmpLat +  9000000;
  char GS[6] = {'A', 'A', '0', '0', 'a', 'a'  };
  GS[0] +=  tmpLon / 2000000;
  GS[1] +=  tmpLat / 1000000;
  GS[2] += (tmpLon % 2000000) / 200000;
  GS[3] += (tmpLat % 1000000) / 100000;
  GS[4] += (tmpLon %  200000) /   8333;
  GS[5] += (tmpLat %  100000) /   4166;
  int indx = 0;
  while (indx < 6)
  {
    strTemp += (GS[indx]);indx++;
  }
  return strTemp;
}