// Simple ping library replacement 07/30/2023, just needed ping_cm mostly
// can use interrupts or blocking pulsein
// *******************************************
//
// similar to: NewPing Library - v1.8 - 07/30/2016
//  Created by Tim Eckel - teckel@leethost.com
//  Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html

#include "SimplePing.h"

SimplePing::SimplePing(uint8_t pingPin, uint8_t echoPin, unsigned int max_cm_distance) { // attachInterrupt doesn't work in initializer...
  _echoPin = echoPin;
  _pingPin = pingPin;
  digitalWrite(_pingPin, LOW);
  pinMode(_pingPin, OUTPUT);
  pinMode(_echoPin, INPUT);
};

SimplePing::~SimplePing() {
#if !USE_PULSEIN
  if (_active) detachInterrupt(_echoPin);
#endif
};

unsigned long SimplePing::ping_cm(unsigned int max_cm_distance) { // trigger new ping and return last result
  ping_trigger();
#if USE_PULSEIN
  _duration = pulseIn(_echoPin, HIGH, MAX_SENSOR_DISTANCE * US_ROUNDTRIP_CM); // in case we want to wait...
#endif
  if (_duration > MAX_SENSOR_DISTANCE * US_ROUNDTRIP_CM) return NO_ECHO;
  else return _duration / US_ROUNDTRIP_CM;
}

boolean SimplePing::ping_trigger() {
  digitalWrite(_pingPin, LOW);
  delayMicroseconds(5);
  digitalWrite(_pingPin, HIGH);
  delayMicroseconds(20); // longer for JSN-SR04T
  digitalWrite(_pingPin, LOW);
  delayMicroseconds(5);
#if !USE_PULSEIN
  if (!_active)
  {
    attachInterrupt(_echoPin, std::bind(&SimplePing::isr, this), CHANGE);
    _active = true;
  }
#endif
  _measure = true;
  return true;
}

void IRAM_ATTR SimplePing::isr() { // Note: sometimes linker has problems to put this literal
  auto time = micros();
  if (!_measure) return;

  // check if we went from HIGH to LOW, or from LOW to HIGH
  switch (digitalRead(_echoPin)) {
    case HIGH:
      // store starttime when we just received ping response
      //    _______________.... (duration, longer pule = obstacle far away)
      //    |
      //____|
      _startTime = time;
      break;
    case LOW:
      // calculate ping duration for current sonar
      // ____
      //    |
      //    |______________.... (when signal goes LOW, calculate duration of pulse and store as one sample)
      _duration = (time - _startTime); // divide with 57 to get distance in centimeters from microseconds.
      _measure = false; // lock interrupt
  }
}
