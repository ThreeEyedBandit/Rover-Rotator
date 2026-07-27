#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <analogWrite.h>
#include <JY901.h>
#include <Adafruit_NeoPixel.h>

#define txPacketInterval 100
#define rxTimeoutInterval 300

#define az_left_pin 7
#define az_right_pin 6

#define azSpeedTimeInterval 10
#define azSpeedMin 100
#define azSpeedMax 255
#define azSpeedAccelStep 1
#define azSpeedDecelStep -2

//uint8_t remoteAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t remoteAddress[] = {0xE8, 0x31, 0xCD, 0x18, 0xDF, 0x98}; //Display
//uint8_t remoteAddress[] = {0x10, 0x52, 0x1C, 0x5E, 0x2E, 0x20}; //Driver

void azStuff();
void checkAZEL();

byte azSpeed = azSpeedMin;
byte wasLeft = false;
byte wasRight = false;

unsigned long rxTimeoutStart = millis();
unsigned long txPacketStart = millis();
unsigned long azTimeStart = millis();

Adafruit_NeoPixel statusLED(1, PIN_NEOPIXEL);

typedef struct dataPackage
{
  float latitude;
  float longitude;

  int dateYear;
  int currentAltitude;
  int currentAz;
  int currentEl;

  byte dateMonth;
  byte dateDay;
  byte timeHour;
  byte timeMinute;
  byte timeSecond;
  byte azSpeed = azSpeedMin;

  bool left;
  bool right;
}
dataPackage;

dataPackage localData;
dataPackage remoteData;

bool blnConnected = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

esp_now_peer_info_t peerInfo;

void setup() 
{
  Serial.begin(19200);
  Serial1.begin(19200);

  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(OnDataRecv);

  analogWrite(az_left_pin, 0);
  analogWrite(az_right_pin, 0);

  Serial.print("MAC Address:  ");
  Serial.println(WiFi.macAddress());

  statusLED.begin();
  statusLED.setBrightness(25);

  statusLED.setPixelColor(0,statusLED.Color(255,0,0));
  statusLED.show();
}
void checkAZEL()
{
  int tempAz;
  int tempEl; 
  tempAz = ((float)JY901.stcAngle.Angle[2]/32768*180);
  tempEl = ((float)JY901.stcAngle.Angle[0]/32768*180);
  tempAz = map(tempAz,0,359,359,0);

  if(tempAz < 0)
  {
    tempAz += 360;
  }
  else if(tempAz >= 360)
  {
    tempAz -= 360;
  }
  if(tempEl > 90)
  {
    tempEl = 90;
  }
  else if(tempEl < 0)
  {
    tempEl = 0;
  }

  localData.currentAz = tempAz;
  localData.currentEl = tempEl;
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{ 
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  rxTimeoutStart = millis();
}
void azStuff()
{
  if(!blnConnected)
  {
    remoteData.left = false;
    remoteData.right = false;
  }
  
  if(remoteData.left && remoteData.right)
  {
    analogWrite(az_left_pin, 0);
    analogWrite(az_right_pin, 0);
  }
  else if(remoteData.left)
  {
    analogWrite(az_right_pin, 0);
    analogWrite(az_left_pin, azSpeed);
    localData.left = true;
    localData.right = false;
  }
  else if(remoteData.right)
  {
    analogWrite(az_left_pin, 0);
    analogWrite(az_right_pin, azSpeed);
    localData.left = false;
    localData.right = true;
  }
  else 
  {
    if(wasLeft)
    {
      analogWrite(az_right_pin, 0);
      analogWrite(az_left_pin, azSpeed);
    }
    else if(wasRight)
    {
      analogWrite(az_left_pin, 0);
      analogWrite(az_right_pin, azSpeed);
    }
    else
    {
      analogWrite(az_left_pin, 0);
      analogWrite(az_right_pin, 0);
      localData.left = false;
      localData.right = false;
    }
  }

  if(millis() - azTimeStart >= azSpeedTimeInterval)
  {
    if(remoteData.left || remoteData.right)
    {
      if(azSpeed == 0) azSpeed = azSpeedMin;

      if(remoteData.left)
      {
        wasLeft = true;
        wasRight= false;
      }
      else if(remoteData.right)
      {
        wasRight = true;
        wasLeft = false;
      }

      if(azSpeed + azSpeedAccelStep <= azSpeedMax) azSpeed += azSpeedAccelStep;
      else azSpeed = azSpeedMax;
    }

    else
    {
      if(azSpeed + azSpeedDecelStep >= azSpeedMin) azSpeed += azSpeedDecelStep;
      else azSpeed = 0;
    }
    azTimeStart = millis();
    localData.azSpeed = azSpeed;
  }
}
void loop() 
{
  azStuff();
  checkAZEL();

  while(Serial1.available() > 0) //Read AZ/EL
  {
    char charTemp = Serial1.read();
    Serial.write(charTemp);
    JY901.CopeSerialData(charTemp);
  }
  while(Serial.available() > 0)
  {
    char charTemp1 = Serial.read();
    Serial1.write(charTemp1);
  }

  if(millis() - rxTimeoutStart >= rxTimeoutInterval)//Timeout
  {
    if(blnConnected)
    {
      blnConnected = false;
      statusLED.setPixelColor(0,statusLED.Color(255,0,0));
      statusLED.show();
    }
    blnConnected = false;
  }
  else
  {
    if(!blnConnected)
    {
      blnConnected = true;
      statusLED.setPixelColor(0,statusLED.Color(0,255,0));
      statusLED.show();
    }
    blnConnected = true;
  }

  if(millis() - txPacketStart >= txPacketInterval)
  {
    esp_now_send(remoteAddress, (uint8_t *) &localData, sizeof(localData));
    txPacketStart = millis();
  }
}