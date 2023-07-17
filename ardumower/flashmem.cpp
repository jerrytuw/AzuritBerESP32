#include "flashmem.h"
#include "drivers.h"
#include "config.h"

#if defined __AVR__ || defined ESP32
#include <EEPROM.h>
#endif

FlashClass Flash;

int eereadwriteString(boolean readflag, int &ee, String& value)
{
  unsigned int i;
  if (readflag) {
    value = "";
    char ch = Flash.read(ee++);
    while (ch) {
      value += ch;
      ch = Flash.read(ee++);
    }
  } else
  {
    for (i = 0; i < value.length(); i++) {
      Flash.write(ee++, value.charAt(i));
    }
    Flash.write(ee++, 0);
  }
  return i; // !
}

FlashClass::FlashClass() {
  verboseOutput = false;
#if defined __AVR__ || defined ESP32
#else
#endif
}

void FlashClass::test() {
  Serial.println(F("EEPROM test - Please wait..."));
  EEPROM.begin(EEPROM_SIZE);
  boolean success = true;

  byte savebuffer[EEPROM_SIZE];
  Serial.print("read current content... ");
  for (int i = 0; i < EEPROM_SIZE; i++) {// test 1024 addresses
    savebuffer[i] = EEPROM.readByte(i);  // read original value
    //if (i < 10) Serial.printf("%02x=%02x ", savebuffer[i], savebuffer[i]);
  }
  Serial.print("write test pattern... ");
  for (int i = 0; i < EEPROM_SIZE; i++) { // test 1024 addresses
    EEPROM.writeByte(i, (i & 0xff)); // write test value
    //if (i < 10) Serial.printf("%02x=%02x ", i & 0xff, EEPROM.readByte(i));
  }

  EEPROM.commit();

  Serial.print("committed... ");

  for (int i = 0; i < EEPROM_SIZE; i++) { // test 1024 addresses
    byte v = EEPROM.readByte(i); // get test value
    //if (i < 10) Serial.printf("w%02x=%02x | ", v, EEPROM.readByte(i));
    if (v != (i & 0xff)) { // incorrect read or write or both
      success = false;
      //if (i < 10) Serial.printf(" %02x=%02x @%d\n", v, EEPROM.readByte(i), i);
      break;
    }
  }
  for (int i = 0; i < EEPROM_SIZE; i++) { // test 1024 addresses
    EEPROM.writeByte(i, savebuffer[i]);  // write test value
  }
  EEPROM.commit();

  if (success) Serial.println(F("EEPROM test success!"));
  else Serial.println(F("EEPROM error - RTC module missing?"));
}

byte FlashClass::read(uint32_t address) {
#if defined __AVR__ || defined ESP32
  byte data = EEPROM.readByte(address);
  if (verboseOutput) {
    Console.print(F("#76,"));
    Console.print(address);
    Console.print(F(","));
    Console.print(data);
    Console.println();
  }
  return data;
#else
  return readAT24C32(address);
#endif
}

byte* FlashClass::readAddress(uint32_t address) {
#if defined __AVR__ || defined ESP32
  byte d = EEPROM.readByte(address);
  return &d;
#else
  byte d = readAT24C32(address);
  return &d;
#endif
}

void FlashClass::dump() {
  Console.println(F("EEPROM dump"));
  for (int i = 0; i < 1024; i++) {
    byte v = read(i);
    if (v != 0) {
      Console.print(i);
      Console.print(F("="));
      Console.print(v);
      Console.print(F(", "));
    }
  }
  Console.println();
}

boolean FlashClass::write(uint32_t address, byte value) {
  if (verboseOutput) {
    Console.print(F("!76,"));
    Console.print(address);
    Console.print(F(","));
    Console.print(value);
    Console.println();
  }
#if defined __AVR__ || defined ESP32
  EEPROM.writeByte(address, value);
  return true;
#else
  writeAT24C32(address, value);
  return true;
#endif
}


boolean FlashClass::write(uint32_t address, byte *data, uint32_t dataLength) {
  for (int i = 0; i < dataLength; i++) {
    write(address + i, data[i]);
  }
  return true;
}
