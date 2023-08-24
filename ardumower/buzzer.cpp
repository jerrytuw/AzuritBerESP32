#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
//#include "DueTimer.h"
//#define pinBuzzer 53
BuzzerClass Buzzer;
#define beepChannel 2

void BuzzerClass::begin()
{
  ledcSetup(beepChannel, 1000, 8);
  pinMode(pinBuzzer, OUTPUT);
  ledcWrite(beepChannel, 0);
  ledcAttachPin(pinBuzzer, beepChannel);
}

void BuzzerClass::tone( unsigned int  freq )
{
  ledcWriteTone(beepChannel, freq);
}

void BuzzerClass::noTone() {
  ledcWrite(beepChannel, 0);
}
