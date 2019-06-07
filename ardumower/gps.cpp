//This GPS class is only use to send the reveived data from Serial3 to raspberrypi port.
#include "robot.h"
#include "mower.h"
#include "gps.h"
//#define N_FLOATS 4




void GPS::init() {


  Console.println("------------------------------- GPS  Initialisation --------------------------------------------");
  GpsPort.begin(GPS_BAUDRATE);


  const char UBLOX_INIT[] PROGMEM =
  {
    // Disable NMEA not use sentence
    0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x00, 0xF9, 0x11, // <- UBX CFG,  Size  10,  'Config'
    // Disable or enable Gx... sentence
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off <- UBX CFG-MSG,  Size  16,  'Messages'
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,  // GxGGA on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
    //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x09, 0x54, // GxRMC on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

    // Rate
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, //1 each 3 second
    //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //1 each 1 second
    //speed change tp 38400 fail ??
    //0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x7E,  //speed change to 38400
    // 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22,  //

    0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 //<- UBX MON,  Size   8,  'Monitor'
  };
  // send configuration data in UBX protocol
  Console.println("Send config sentence");
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    GpsPort.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  robot.gpsReady=true;
  Console.println("End of Gps Config");
  /*
    //fail to change speed on M6n so stay at 9600 bps
    GpsPort.flush();
    Console.println("Restore the new baudRate to 38400 ");
    GpsPort.begin(38400);
  */
}

void GPS::run() {
  if (!GpsPort.available()) {
    return;
  }
  while (GpsPort.available()) {

    char c = GpsPort.read();
    buf[pos] = c;
    pos++;
    if (pos >= 120) //avoid a buffer overrun
    {
      //Console.print (buf);
      Console.println("----------------- warning >120 char received  from GPS ?????? ------------");
      memset(buf, '\0', 120);
      GpsPort.flush();
      pos = 0;
    }
    if (c == '\n') //linefeed
    {
        writePi(buf);
        memset(buf, '\0', 120);
        pos = 0;
     
    }
  }
}


void GPS::writePi(String stringLine) {
  if (robot.RaspberryPIUse) {
    RaspberryPIPort.print(stringLine); // no println becauce the cr lf is in the stringline
  }
  else
  {
    Console.println(stringLine);
  }


}
