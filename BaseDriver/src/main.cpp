#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <JY901.h>
#include <Adafruit_NeoPixel.h>

#define txPacketInterval 100
#define rxTimeoutInterval 300
#define touchHoldMs 100

#define az_left_pin 7
#define az_right_pin 6

#define AZ_DIR_NONE  0
#define AZ_DIR_LEFT  1
#define AZ_DIR_RIGHT 2

uint8_t remoteAddress[] = {0xE8, 0x31, 0xCD, 0x18, 0xDF, 0x98}; // Display

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
  byte azSpeed;

  bool left;
  bool right;
}
dataPackage;

dataPackage localData;
dataPackage remoteData;

bool blnConnected = false;
uint8_t azActiveDir = AZ_DIR_NONE;

unsigned long rxTimeoutStart = 0;
unsigned long txPacketStart = 0;

Adafruit_NeoPixel statusLED(1, PIN_NEOPIXEL);
esp_now_peer_info_t peerInfo;

void setAzOutputs(uint8_t dir)
{
  digitalWrite(az_left_pin, dir == AZ_DIR_LEFT ? HIGH : LOW);
  digitalWrite(az_right_pin, dir == AZ_DIR_RIGHT ? HIGH : LOW);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  rxTimeoutStart = millis();
}

void checkAZEL()
{
  int tempAz = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);
  int tempEl = ((float)JY901.stcAngle.Angle[0] / 32768 * 180);
  tempAz = map(tempAz, 0, 359, 359, 0);

  if(tempAz < 0) tempAz += 360;
  else if(tempAz >= 360) tempAz -= 360;

  if(tempEl > 90) tempEl = 90;
  else if(tempEl < 0) tempEl = 0;

  localData.currentAz = tempAz;
  localData.currentEl = tempEl;
}

void azStuff()
{
  static unsigned long lastLeftMs = 0;
  static unsigned long lastRightMs = 0;
  static bool leftHeld = false;
  static bool rightHeld = false;

  if(blnConnected)
  {
    if(remoteData.left && !remoteData.right)
    {
      lastLeftMs = millis();
      leftHeld = true;
      rightHeld = false;
    }
    else if(remoteData.right && !remoteData.left)
    {
      lastRightMs = millis();
      rightHeld = true;
      leftHeld = false;
    }
  }
  else
  {
    leftHeld = false;
    rightHeld = false;
  }

  if(leftHeld && (millis() - lastLeftMs >= touchHoldMs)) leftHeld = false;
  if(rightHeld && (millis() - lastRightMs >= touchHoldMs)) rightHeld = false;

  uint8_t wantDir = AZ_DIR_NONE;
  if(leftHeld && !rightHeld) wantDir = AZ_DIR_LEFT;
  else if(rightHeld && !leftHeld) wantDir = AZ_DIR_RIGHT;

  // Stop fully before allowing a direction change
  if(azActiveDir != AZ_DIR_NONE && wantDir != AZ_DIR_NONE && wantDir != azActiveDir)
    azActiveDir = AZ_DIR_NONE;
  else
    azActiveDir = wantDir;

  setAzOutputs(azActiveDir);
  localData.left = (azActiveDir == AZ_DIR_LEFT);
  localData.right = (azActiveDir == AZ_DIR_RIGHT);
  localData.azSpeed = (azActiveDir == AZ_DIR_NONE) ? 0 : 255;
}

void setup()
{
  Serial1.begin(19200);

  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(az_left_pin, OUTPUT);
  pinMode(az_right_pin, OUTPUT);
  setAzOutputs(AZ_DIR_NONE);

  statusLED.begin();
  statusLED.setBrightness(25);
  statusLED.setPixelColor(0, statusLED.Color(255, 0, 0));
  statusLED.show();
}

void loop()
{
  azStuff();
  checkAZEL();

  while(Serial1.available() > 0)
    JY901.CopeSerialData(Serial1.read());

  bool connected = (millis() - rxTimeoutStart < rxTimeoutInterval);
  if(connected != blnConnected)
  {
    blnConnected = connected;
    statusLED.setPixelColor(0, statusLED.Color(connected ? 0 : 255, connected ? 255 : 0, 0));
    statusLED.show();
  }

  if(millis() - txPacketStart >= txPacketInterval)
  {
    esp_now_send(remoteAddress, (uint8_t *)&localData, sizeof(localData));
    txPacketStart = millis();
  }
}
