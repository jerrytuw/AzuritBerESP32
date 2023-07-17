#include "perimeter.h"
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "config.h"



//PerimeterClass Perimeter;

/*
  // developer test to be activated in mower.cpp:
  #ifdef USE_DEVELOPER_TEST
  // more motor driver friendly signal (receiver)
  int8_t sigcode_norm[] = {1, -1, 0, 0, 0, 1, -1, 0, 0, 0, -1, 1, 0, 0, 0, 1, -1, 0, 0, 0};
  #else
  // http://grauonline.de/alexwww/ardumower/filter/filter.html
  // "pseudonoise4_pw" signal
  // if using reconstructed sender signal, use this
  #if defined (SIGCODE_1) //for station area 1
*/

int8_t sigcode_diff[]        = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };
#define LARGE_MAG 2000

int8_t sigcode_norm1[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff1[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };

int8_t sigcode_norm2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff2[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };

int8_t sigcode_norm3[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff3[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };

int8_t sigcode_norm[128];
int16_t sigcode_size;

int16_t sumx[2][255];
int16_t posmin[2];
int16_t posmax[2];

PerimeterClass::PerimeterClass() {
  //useDifferentialPerimeterSignal = true;
  swapCoilPolarityLeft = false;
  swapCoilPolarityRight = false;
  timedOutIfBelowSmag = 300;
  timeOutSecIfNotInside = 15;
  callCounter = 0;
  mag[0] = mag[1] = 0;
  smoothMag[0] = smoothMag[1] = 0;
  filterQuality[0] = filterQuality[1] = 0;
  signalCounter[0] = signalCounter[1] = 0;
  lastInsideTime[0] = lastInsideTime[1] = 0;
}


void PerimeterClass::changeArea(byte areaInMowing) {
  Console.print("Change to Area : ");
  Console.println(areaInMowing);

  Console.println();

}


void PerimeterClass::begin(byte idx0Pin, byte idx1Pin) {
  idxPin[0] = idx0Pin;
  idxPin[1] = idx1Pin;

  subSample = 4;
  sigcode_size = sizeof sigcode_diff;
  // use max. 1024 samples and multiple of signalsize
  int adcSampleCount = sizeof sigcode_diff * subSample;
  numSamples = ((int)255 / adcSampleCount) * adcSampleCount;

  Console.print(F("matchSignal size="));
  Console.println(sigcode_size);
  Console.print(F("subSample="));
  Console.println((int)subSample);
  Console.print(F("capture size="));
  Console.println(numSamples);
}

void PerimeterClass::speedTest() {
  int loops = 0;
  unsigned long endTime = millis() + 1000;
  while (millis() < endTime) {
    matchedFilter(0);
    loops++;
  }
  Console.print("Read in 1 sec ");
  Console.println(loops);

}

const int8_t* PerimeterClass::getRawSignalSample(byte idx) {
  return &raw_buff[idx][0];
}

void PerimeterClass::run() {
  byte maxindex;
  //bb 2 coil read
  if (read2Coil) maxindex = 2;
  else maxindex = 1;

  for (int idx = 0; idx < maxindex; idx++) {
    ADCMan.run();
    //matchedFilter(idx);
  }
}

int PerimeterClass::getMagnitude(byte idx) {
  return mag[idx];
}

int PerimeterClass::getSmoothMagnitude(byte idx) {
  return smoothMag[idx];
}

void PerimeterClass::printADCMinMax(int8_t *samples) {
  int8_t vmax = SCHAR_MIN;
  int8_t vmin = SCHAR_MAX;
  for (byte i = 0; i < ADCMan.getSampleCount(idxPin[0]); i++) {
    vmax = max(vmax, samples[i]);
    vmin = min(vmin, samples[i]);
  }
  Console.print(F("perimter min,max="));
  Console.print((int)vmin);
  Console.print(F(","));
  Console.println((int)vmax);
}

// perimeter V2 uses a digital matched filter
void PerimeterClass::matchedFilter(byte idx) {

  // magnitude for tracking (fast but inaccurate)

  mag[idx] = corrFilter(idx, sigcode_diff, subSample, sizeof sigcode_diff, &raw_buff[idx][0], numSamples - (sizeof sigcode_diff) * subSample, filterQuality[idx]);

  if ((idx == 0) && swapCoilPolarityLeft) mag[idx] *= -1;
  if ((idx == 1) && swapCoilPolarityRight) mag[idx] *= -1;
  // smoothed magnitude used for signal-off detection change from 1 % to 5 % for faster detection and possible use on center big area to avoid in/out transition
  smoothMag[idx] = 0.95 * smoothMag[idx] + 0.05 * ((float)abs(mag[idx]));
  //smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * ((float)abs(mag[idx]));

  // perimeter inside/outside detection
  if (mag[idx] > 0) {
    signalCounter[idx] = min(signalCounter[idx] + 1, 5);
  } else {
    signalCounter[idx] = max(signalCounter[idx] - 1, -5);
  }
  if (mag[idx] < 0) {
    lastInsideTime[idx] = millis();
  }

  if (idx == 0) callCounter++;
}

void PerimeterClass::resetTimedOut() {
  lastInsideTime[0] = millis();
  lastInsideTime[1] = millis();
}

int16_t PerimeterClass::getSignalMin(byte idx) {
  return signalMin[idx];
}

int16_t PerimeterClass::getSignalMax(byte idx) {
  return signalMax[idx];
}

int16_t PerimeterClass::getSignalAvg(byte idx) {
  return signalAvg[idx];
}


float PerimeterClass::getFilterQuality(byte idx) {
  return filterQuality[idx];
}

int PerimeterClass::getSignalCounter(byte idx) {
  return signalCounter[idx];
}

boolean PerimeterClass::isInside() {

  return (isInside(IDX_LEFT));
  //return (isInside(IDX_LEFT) && isInside(IDX_RIGHT));
}

boolean PerimeterClass::isInside(byte idx) {
  if (abs(mag[idx]) > LARGE_MAG) {
    // Large signal, the in/out detection is reliable.
    // Using mag yields very fast in/out transition reporting.
    return (mag[idx] < 0);
  } else {
    // Low signal, use filtered value for increased reliability
    return (signalCounter[idx] < 0);
  }
}

boolean PerimeterClass::signalTimedOut() {

  return (signalTimedOut(IDX_LEFT) && signalTimedOut(IDX_RIGHT));
}


boolean PerimeterClass::signalTimedOut(byte idx) {
  if (getSmoothMagnitude(idx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[idx] > timeOutSecIfNotInside * 1000) return true;
  return false;
}


// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t PerimeterClass::corrFilter(byte idx, int8_t *signal, int8_t subsamples, int16_t signalLen, int8_t *data, int16_t nPts, float &quality) {

  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  int16_t subsampledSignalLen = signalLen * subsamples; // number of filter coeffs including subsampling

  posmin[idx] = posmax[idx] = 0;
  // compute sum of absolute filter coeffs
  int16_t Hsum = 0;
  for (int16_t i = 0; i < signalLen; i++) Hsum += abs(signal[i]);
  Hsum *= subsamples;

  // compute correlation
  // for each input value
  for (int16_t j = 0; j < nPts; j++) //192-4*24 = 96 = first half numsamples
  {
    int16_t sum = 0;
    int8_t *pSignal = signal;
    int8_t ss = 0;
    int8_t *pData = data;
    // for each filter coeffs
    for (int16_t i = 0; i < subsampledSignalLen; i++) //4*24
    {
      sum += ((int16_t)(*pSignal)) * ((int16_t)(*pData));
      ss++;
      if (ss == subsamples) {
        ss = 0;
        pSignal++; // next filter coeffs
      }
      pData++;
    }
    if (sum > sumMax)
    {
      sumMax = sum;
      posmax[idx] = j;
    }
    if (sum < sumMin)
    {
      sumMin = sum;
      posmin[idx] = j;
    }
    data++;
    sumx[idx][j] = sum;

  }
  for (int j = nPts; j < numSamples; j++)
    sumx[idx][j] = 0;
  // normalize to 4095
  sumMin = ((float)sumMin) / ((float)(Hsum * 127)) * 4095.0;
  sumMax = ((float)sumMax) / ((float)(Hsum * 127)) * 4095.0;

  // compute ratio min/max
  if (sumMax > -sumMin) {
    quality = ((float)sumMax) / ((float) - sumMin);
    return sumMax;
  } else {
    quality = ((float) - sumMin) / ((float)sumMax);
    return sumMin;
  }
}
