// Simple ping library replacement 07/30/2023
// similar to: NewPing Library - v1.8 - 07/30/2016
//  Created by Tim Eckel - teckel@leethost.com
//  Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html

#ifndef SimplePing_h
#define SimplePing_h

#include <Arduino.h>
#include <FunctionalInterrupt.h> // for more flexible attachInterrupt() in class

#define MAX_SENSOR_DISTANCE 120 // Maximum sensor distance can be as high as 200cm, no reason to wait for ping longer than sound takes to travel this distance and back. Default=500
#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57
#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
#define PING_OVERHEAD 5         // Ping overhead in microseconds (uS). Default=5
#define USE_PULSEIN false       // use wait with pulseIn instead of interrupt. Defualt=false

class SimplePing
{
  public:
    SimplePing(uint8_t pingPin, uint8_t echoPin, unsigned int max_cm_distance = MAX_SENSOR_DISTANCE);
    ~SimplePing();
    unsigned long ping_cm(unsigned int max_cm_distance = 0);

  private:
    volatile bool _measure;
    bool _active = false;
    volatile unsigned long _startTime;
    volatile unsigned long _duration;
    uint8_t _pingPin;
    uint8_t _echoPin;
    boolean ping_trigger();
    void IRAM_ATTR isr();
};
#endif
