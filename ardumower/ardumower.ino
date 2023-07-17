/* This is a crude mixture/stripdown of Ardumower Azurit/AzuritBer and other mower code snippets
 ************************************************************************************************
    with the aim to run a basic mower not on an AVR plus ESP32 but on a single ESP32,
    AKA "ESP32 all-in-one" with Arduino ESP32 libs. 
    Still a bit messy because of the quick compromises for adapting it to ESP32 and
    hoverboard motors with their unusual characteristics on ZS-X11H controllers.
    (Also the original hoverboard controller could be used over SoftwareSerial - not included here)
    Main modifications:
    - left out all extras considered not essential
    - Adcman version for ESP32 I2S DMA ADC including Perimeter mods
    - fixed I2Cdev for ESP32
    - Flashmem version for ESP32
    - Motor PWM and tick interface for ZS-X11H and hoverboard motors
    - FakeSerial over WiFi for Pfod
    - ElegantOTA for firmware updates over WiFi an mDNS name "mow.local"
 
*/

/*      DUE and ODOMETRY MANDATORY VERSION
        PCB1.3
        COMPATIBLE WITH ONE OF THIS IMU
        GY-88 CONNECTED ON I2C2 IMU
        OR GY-521 with compass deactivate CONNECTED ON I2C2 IMU
        OR GY-521 CONNECTED ON I2C2 IMU + GY-273 CONNECTED ON I2C3 DISPLAY with compass activate
        SO IT'S MPU6050 and HMC5883L or QMC5883L

        Connect DUE programming port to PC for programming and PC console
        Connect DUE Native USB port to Raspberry Pi

        During all test and dev set Enable_DueWatchdog to false
        or increase the delay in robot.cpp line : watchdogEnable(3000);// Watchdog trigger after  3 sec if not reseted.

        -------------------------- COMPASS TYPE --------------------------------------------------
        Into mower.h line 109 select correct configuration
        // ---------------- COMPASS Selection ---------------------------
        //#define COMPASS_IS HMC5883L
        #define COMPASS_IS QMC5883L
        -------------------------- BT or ESP8266 --------------------------------------------------
        Into mower.cpp line 255 select correct configuration
         bluetoothUse      = 1;      // use Bluetooth module? It's Impossible to use Bluetooth and esp8266 at same time
         esp8266Use        = 0;       // use ESP8266 Wifi module?

        ------------------------- RTC CHIP -------------------------------------------------------
        Into mower.h line 163 Select the correct RTC.
        #define AT24C32_ADDRESS B1010000 //0x50 //Standard PCB1.3 RTC ds1307 memory module
        //#define AT24C32_ADDRESS B1010111 //0x57 //Simple PCB RTC ds3231 memory module


        ------------------------- OLED SCREEN -------------------------------------------------------
        Into mower.h line 128 Enable_Screen if you want to connect a 128*64 OLED screen to mower
        Connect the screen to I2C1 DUE port
        #define Enable_Screen true
        //#define Enable_Screen false


        ------------------------------ RASPBERRY -------------------------------------------------
        If Raspberry PI is not connected you need to change into mower.h
        into mower.h line 121
            #define Console Serial
            //#define Console SerialUSB
        and into arduremote setting Raspberry set raspberryPiuse to NO
        then you have access to the Console on the PC

        If Raspberry PI is connected you need to change into mower.h
            //#define Console Serial
            #define Console SerialUSB
        and into arduremote setting R/C set raspberryPiuse to YES
        ------------------------------------------------------------------------------------------
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich

  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)


  Documentation:  http://wiki.ardumower.de

*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "FakeSerial.h"

FakeSerialClass FakeSerial; // for pfod

/**/
//WiFi code adapted from AzuritBer new passerelle for ESP32

#include <esp_wifi.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include <AsyncElegantOTA.h>
AsyncWebServer server(80);

#define PROTOCOL_TCP
#define bufferSize 1024

bool debug = true;

#define VERSION "1.00"

// For standard mode:
// ssid and password according your router
#include "credentials.h"

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;
#endif

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiClient TheClient;
WiFiServer TheServeur(8881);
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;
#endif

/* WiFi server for pfod app in connection with FakeSerial interface */
/* still contains rests of BT interface*/
void setuppfod() {

  if (debug) Serial.println("\n\n ESP32 BT and WiFi serial bridge V1.00");

#ifdef BLUETOOTH
  if (debug) Serial.println("Start Bluetooth Server");
  SerialBT.begin("Ardumower"); //Bluetooth device name
#endif


#ifdef PROTOCOL_TCP
  if (debug) Serial.println("Starting Server on port 8881");
  TheServeur.begin(); // start TCP server
  TheServeur.setNoDelay(true);
#endif

  // esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}

/* WiFi server for pfod app in connection with FakeSerial interface */
void looppf()
{
#ifdef BLUETOOTH
  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }
#endif

#ifdef PROTOCOL_TCP
  if (TheServeur.hasClient())
  {
    //find free/disconnected spot
    if (!TheClient || !TheClient.connected()) {
      if (TheClient) TheClient.stop();
      TheClient = TheServeur.available();
      if (debug) Serial.println("New WIFI client ");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TheServeur.available();
    TmpserverClient.stop();
  }
#endif
#ifdef PROTOCOL_TCP

  //if (Serial2 != NULL)
  {
    if (TheClient)
    {
      while (TheClient.available())
      {
        WIFIbuf[inWiFI] = TheClient.read(); // read char from client
        //Serial.printf("%c", WIFIbuf[inWiFI]);
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      if (inWiFI)
      {
        FakeSerial.writeIn(WIFIbuf, inWiFI); // now send to UART(2):
        //Serial.println();
        inWiFI = 0;
      }
    }
    if (FakeSerial.availableIn())
    {
      //Serial.printf("read ");
      while (FakeSerial.availableIn())
      {
        WIFIbuf[inWiFI] = FakeSerial.readIn(); // read char from UART(2)
        //Serial.printf("%c", WIFIbuf[inWiFI]);
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      if (TheClient && inWiFI)
      {
        TheClient.write(WIFIbuf, inWiFI);
        //Serial.printf(" pushed %d\n",inWiFI);
        inWiFI = 0;
      }

#ifdef BLUETOOTH
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);
#endif
      inWiFI = 0;
    }

  }
#endif
}
/**/

void setup()  {
  Serial.begin(115200);
  if (debug) Serial.println("Start ESP32 Station mode");
  // STATION mode (ESP connects to router)
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ESP32mower");
  WiFi.begin(ssid, pw);
  if (debug) Serial.print("try to Connect to the Wireless network: ");
  if (debug) Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    if (debug) Serial.print(".");
    ///need to count and stop wifi scanning if more than 20 secondes
  }
  if (debug) Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  
  Serial.println(WiFi.getHostname());

  if (!MDNS.begin("mow")) Serial.println("\nError starting mDNS");
  else
  {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("pfod", "tcp", 8881);
    Serial.println("\nAdvertized mDNS mow.local");
  }

  if (debug) Serial.println("\nat IP");
  if (debug) Serial.println(WiFi.localIP());
  if (debug) Serial.println("\nUse port 8881");

  setuppfod(); // also WIFI
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  robot.setup();
}

void loop()  {
  robot.loop();
}
