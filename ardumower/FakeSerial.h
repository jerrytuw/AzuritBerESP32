#ifndef FAKE_SERIAL
#define FAKE_SERIAL

#include <Arduino.h>

class FakeSerialClass : public Print {
  public:
    void begin(uint32_t&);
    size_t write(uint8_t c);
    size_t writeIn(uint8_t c);
    size_t write(const uint8_t* buffer, size_t size);
    size_t writeIn(const uint8_t* buffer, size_t size);
    int available();
    int availableIn();
    int read();
    int readIn();
  private:
    uint8_t buf[1024];
    uint8_t bufIn[1024];
    int readptr;
    int writeptr;
    int readptrIn;
    int writeptrIn;
};

extern FakeSerialClass FakeSerial;
#endif
