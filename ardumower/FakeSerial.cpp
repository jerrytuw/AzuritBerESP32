/*
 * this is a quick and dirty fake Serial port to be used over a WiFi connection e.g. for pfod app
 */
#include "FakeSerial.h"

void FakeSerialClass::begin(uint32_t&)
{
  writeptr = 0;
  readptr = 0;
  writeptrIn = 0;
  readptrIn = 0;
};
// Print

size_t FakeSerialClass::write(uint8_t c)
{
  //Serial.printf("%c", c);
  buf[writeptr++] = c;
  writeptr %= sizeof buf;
  return 1;
};

size_t FakeSerialClass::writeIn(uint8_t c)
{
  //Serial.printf("%c", c);
  buf[writeptrIn++] = c;
  writeptrIn %= sizeof bufIn;
  return 1;
};

size_t FakeSerialClass::write(const uint8_t* buffer, size_t size)
{
  //Serial.printf("write %d ", size);
  for (int i = 0; i < size; i++)
  {
    write(buffer[i]);
  }
  //Serial.println();
  return size;
};

size_t FakeSerialClass::writeIn(const uint8_t* buffer, size_t size)
{
  //Serial.printf("writeIn %d ", size);
  for (int i = 0; i < size; i++)
  {
    writeIn(buffer[i]);
  }
  //Serial.println();
  return size;
};

int FakeSerialClass::availableIn()
{
  int size = writeptr - readptr;
  if (size >= 0) return size;
  else return (writeptr + ((sizeof buf) - readptr));
};

int FakeSerialClass::available()
{
  int size = writeptrIn - readptrIn;
  if (size >= 0) return size;
  else return (writeptrIn + ((sizeof buf) - readptrIn));
};

int FakeSerialClass::readIn()
{
  if (writeptr == readptr) return -1;
  else
  {
    int c = buf[readptr++];
    readptr %= sizeof buf;
    return c;
  }
};

int FakeSerialClass::read()
{
  if (writeptrIn == readptrIn) return -1;
  else
  {
    int c = buf[readptrIn++];
    readptrIn %= sizeof bufIn;
    return c;
  }
};
