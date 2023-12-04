#include <WiFi.h>
#include <esp_log.h>

#define TAG "main"

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/ma730/MagneticSensorMA730.h"

#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin
uint8_t pins[] = {2, 3, 10, 6, 7};
void setup()
 {
    delay(1000);
}

int indexOfCS   = 3;
int indexOfMISO = 4;
int indexOfMOSI = 0;
int indexOfCLK  = 2;

int pinOfCS   = 0;
int pinOfMISO = 0;
int pinOfMOSI = 0;
int pinOfCLK  = 0;

int numOfCS   = 1;
int numOfMISO = 1;
int numOfMOSI = 2;
int numOfCLK  = 1;

#include <math.h>

void showMA730(MagneticSensorMA730 *sensor1, int pinOfCLK, int pinOfMISO, int pinOfMOSI, int pinOfCS, int times = 5) {
  unsigned long prevMs = 0;
  int t = 0;
  while (true) {
    if (t >= times)
      break;

    if (millis() - prevMs < 10)
      continue;
    prevMs = millis();

    t++;

    sensor1->update();

    // get the angle, in radians, including full rotations
    float a1 = degrees(sensor1->getAngle());

    // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
    float v1 = degrees(sensor1->getVelocity());

    // get the angle, in radians, no full rotations
    float a2 = degrees(sensor1->getCurrentAngle());

    // get the raw 14 bit value
    uint16_t raw = sensor1->readRawAngle();

    // set pulses per turn for encoder mode
    sensor1->setPulsesPerTurn(999); // set to 999 if we want 1000 PPR == 4000 CPR
    if (a1 !=0 || v1 !=0 || a2 !=0 || raw !=0) {
      ESP_LOGI(TAG, "clkPin:%d misoPin:%d mosiPin:%d csPin:%d, Angle:%.2f, Velocity:%.2f, currAngle:%.2f, rawAngle:%d", pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS, a1, v1, a2, raw);
    }
  }
}

void findMA730FromSPI()
{
  for(int c=0; c < numOfCS; c++) {
    pinOfCS = pins[(indexOfCS+c)%5];
    for (int i = 0; i < numOfMISO; ++i) {
      pinOfMISO = pins[(indexOfMISO+i)%5];
      for (int j = 0; j < numOfMOSI; ++j) {
        pinOfMOSI = pins[(indexOfMOSI+j)%5];
        for (int k = 0; k < numOfCLK; ++k) {
          pinOfCLK = pins[(indexOfCLK+k)%5];
          MagneticSensorMA730 *sensor1 = new MagneticSensorMA730(pinOfCS);

          SPI.begin(pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS);
          
          sensor1->init(&SPI);
          sensor1->setRotationDirection(0);

          showMA730(sensor1, pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS, 20);

          sensor1->setRotationDirection(1);

          showMA730(sensor1, pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS, 20);

          SPI.end();
          digitalWrite(pinOfCS, LOW);
          ESP_LOGI(TAG, "");
        }
      }
    }
  }

  ESP_LOGI(TAG, "finished. restart");
  delay(1000);
  ESP.restart();
}

void testMa730()
{
  pinOfCS   = 6;
  pinOfMISO = 7;
  pinOfCLK  = 10;
  pinOfMOSI = 2;

  MagneticSensorMA730 *sensor1 = new MagneticSensorMA730(pinOfCS);

  SPI.begin(pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS);

  sensor1->init(&SPI);

  sensor1->update();

  sensor1->setRotationDirection(0);

  showMA730(sensor1, pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS, 1);

  sensor1->setRotationDirection(1);

  showMA730(sensor1, pinOfCLK, pinOfMISO, pinOfMOSI, pinOfCS, 1);

  SPI.end();
  digitalWrite(pinOfCS, LOW);
  ESP_LOGI(TAG, "");

  pinMode(3, INPUT);
  int ntcValue = analogRead(3);
  float ntcVoltage = analogReadMilliVolts(3);
  ESP_LOGI(TAG, "ntcValue:%d, ntcVoltage:%d", ntcValue, ntcVoltage);
}

unsigned long prevTime = 0;
unsigned long interval = 100;
;

void loop() 
{
  if (millis() - prevTime > interval)
  {
    prevTime = millis();

  #if 0
    findMA730FromSPI();
    delay(1000);
  #else
    testMa730();
    interval = 100;
  #endif
  }
}