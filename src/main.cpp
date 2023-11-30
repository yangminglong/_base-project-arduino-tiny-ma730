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

int indexOfCS = 0;
int indexOfMISO = 1;
int indexOfMOSI = 2;
int indexOfCLK = 3;
int csPin   = pins[0];
int misoPin = pins[1];
int mosiPin = pins[2];
int clkPin  = pins[3];



void findMA730FromSPI()
{
  for(int c=0; c < 5; c++) {
    csPin = pins[(indexOfCS+c)%5];
    for (int i = 0; i < 5; ++i) {
      misoPin = pins[(indexOfMISO+i)%5];
      for (int j = 0; j < 5; ++j) {
        mosiPin = pins[(indexOfMOSI+j)%5];
        for (int k = 0; k < 5; ++k) {
          clkPin = pins[(indexOfCLK+k)%5];
          MagneticSensorMA730 *sensor1 = new MagneticSensorMA730(csPin);

          SPI.begin(clkPin, misoPin, mosiPin, csPin);
          
          sensor1->init(&SPI);

          unsigned long prevMs = 0;
          int t = 0;
          while (true) {
            if (t > 5)
              break;

            if (millis() - prevMs < 10)
              continue;
            prevMs = millis();

            t++;

            sensor1->update();

            // get the angle, in radians, including full rotations
            float a1 = sensor1->getAngle();

            // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
            float v1 = sensor1->getVelocity();

            // get the angle, in radians, no full rotations
            float a2 = sensor1->getCurrentAngle();

            // get the raw 14 bit value
            uint16_t raw = sensor1->readRawAngle();

            // set pulses per turn for encoder mode
            sensor1->setPulsesPerTurn(999); // set to 999 if we want 1000 PPR == 4000 CPR
            if (a1 !=0 || v1 !=0 || a2 !=0 || raw !=0) {
              ESP_LOGI(TAG, "clkPin:%d misoPin:%d mosiPin:%d csPin:%d, Angle:%.2f, Velocity:%.2f, currAngle:%.2f, rawAngle:%d", clkPin, misoPin, mosiPin, csPin, a1, v1, a2, raw);
            }
          }
          SPI.end();
          digitalWrite(csPin, LOW);
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
  csPin   = 6;
  misoPin = 7;
  clkPin  = 10;
  mosiPin = 2;

  MagneticSensorMA730 *sensor1 = new MagneticSensorMA730(csPin);

  SPI.begin(clkPin, misoPin, mosiPin, csPin);

  sensor1->init(&SPI);

  sensor1->update();

  // get the angle, in radians, including full rotations
  float a1 = sensor1->getAngle();

  // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
  float v1 = sensor1->getVelocity();

  // get the angle, in radians, no full rotations
  float a2 = sensor1->getCurrentAngle();

  // get the raw 14 bit value
  uint16_t raw = sensor1->readRawAngle();

  // set pulses per turn for encoder mode
  sensor1->setPulsesPerTurn(999); // set to 999 if we want 1000 PPR == 4000 CPR
  if (a1 !=0 || v1 !=0 || a2 !=0 || raw !=0) {
    ESP_LOGI(TAG, "clkPin:%d misoPin:%d mosiPin:%d csPin:%d, Angle:%.2f, Velocity:%.2f, currAngle:%.2f, rawAngle:%d", clkPin, misoPin, mosiPin, csPin, a1, v1, a2, raw);
  }

  SPI.end();
  digitalWrite(csPin, LOW);
  ESP_LOGI(TAG, "");
}

unsigned long prevTime = 0;
unsigned long interval = 100;

void loop() 
{
  if (millis() - prevTime > interval)
  {
    prevTime = millis();

  #if 0
    findMA730FromSPI();
    interval = 1000;
  #else
    testMa730();
    interval = 100;
  #endif
  }
}