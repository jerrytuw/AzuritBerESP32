/*

  choose your robot type, PCB version, baud rates, pin definitions etc.
  Note: we adapt most from Due version and change where necessary for ESP32
  The ESP32 has plenty of ports but we still are limited so "55" is a virtual placeholder pin

*/

#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "robot.h"

#include "drivers.h"
//#include "bt.h"


/*  This software requires:
        Ardumower PCB v0.5/1.2/1.3  ( https://www.marotronics.de/Ardumower-Board-Prototyp )
        Arduino Mega or Arduino Due (Due requires PCB1.3)
        Ardumower Chassis Kit 1.0  ( http://wiki.ardumower.de/index.php?title=Ardumower_chassis ) or Ardumower Mini
*/


// ------- Choose one Ardumower PCB revision (1.2, 1.3 etc.) ------------------
//#define PCB_1_2
//#define PCB_1_3
#define ESP32_dev_board

// ------- Choose robot model (Ardumower or Ardumower Mini) -------------------
#define ROBOT_ARDUMOWER_ESP32
//#define ROBOT_MINI

// ------- Choose motor driver (MC33926 is recommended) -----------------------
#define DRIVER_ZSX11H
//#define DRIVER_L298N

#define IOREF 3.3   // I/O reference voltage for Due





// ------ pins---------------------------------------
// ***** pin 55 is dummy for unused pins

#define pinMotorLeftEnable  13       // EN motors enable - for hoverboard motor brake
#define pinMotorLeftPWM 26           // M1_IN1 left motor PWM pin - but we use DAC (only on 25, 26)
#define pinMotorLeftDir 5            // M1_IN2 left motor Dir pin
#define pinMotorLeftSense 34         // M1_FB left motor current sense
#define pinMotorLeftFault 55         // *M1_SF left motor fault - not used

#define pinMotorRightEnable  18      // EN motors enable - for hoverboard motor brake
#define pinMotorRightPWM  25         // M2_IN1 right motor PWM pin - but we use DAC (only on 25, 26)
#define pinMotorRightDir 16          // M2_IN2 right motor Dir pin
#define pinMotorRightSense 35        // M2_FB right motor current sense
#define pinMotorRightFault 55        // *M2_SF right motor fault - not used

#define pinMotorMowPWM 55           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 55           // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define pinMotorMowSense 55         // M1_FB  mower motor current sense  
#define pinMotorMowFault 55         // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowEnable 55        // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowRpm 55

#define pinBumperLeft 55            // bumper pins
#define pinBumperRight 55

#define pinDropLeft 55           // drop pins                                                                                          Dropsensor - Absturzsensor
#define pinDropRight 55          // drop pins                                                                                          Dropsensor - Absturzsensor

#define pinSonarCenterTrigger 15   // ultrasonic sensor pins
#define pinSonarCenterEcho 12

#define pinSonarRightTrigger 55
#define pinSonarRightEcho 55
#define pinSonarLeftTrigger 55
#define pinSonarLeftEcho 55

#define pinPerimeterRight 36       // perimeter
#define pinPerimeterLeft 39

#define pinPerimeterCenter 55


#define pinGreenLED 55              // DuoLED green
#define pinRedLED 55                // DuoLED red
#define pinLED 55                  // LED

#define pinBuzzer 19               // Buzzer

#define pinTilt 55                 // Tilt sensor (required for TC-G158 board)
#define pinButton 55               // digital ON/OFF button

#define pinBatteryVoltage 32       // battery voltage sensor

#define pinBatterySwitch 55         // battery-OFF switch   
#define pinChargeVoltage 55        // charging voltage sensor
#define pinChargeCurrent 55        // charge current sensor
#define pinChargeRelay 55          // charge relay
#define pinRemoteMow 55            // remote control mower motor
#define pinRemoteSteer 55          // remote control steering 
#define pinRemoteSpeed 55          // remote control speed
#define pinRemoteSwitch 55         // remote control switch
#define pinVoltageMeasurement 55   // test pin for your own voltage measurements

#define pinOdometryLeft 27     // left odometry sensor
#define pinOdometryLeft2 55    // left odometry sensor (optional two-wire)
#define pinOdometryRight 14    // right odometry sensor  
#define pinOdometryRight2 55   // right odometry sensor (optional two-wire)  

#define pinLawnFrontRecv 55        // lawn sensor front receive
#define pinLawnFrontSend 55        // lawn sensor front sender 
#define pinLawnBackRecv 55         // lawn sensor back receive
#define pinLawnBackSend 55         // lawn sensor back sender 
#define pinUserSwitch1 55          // user-defined switch 1
#define pinUserSwitch2 55          // user-defined switch 2
#define pinUserSwitch3 55          // user-defined switch 3
#define pinRain 55                 // rain sensor

// ---------------- COMPASS Selection ---------------------------
#define COMPASS_IS HMC5883L
//#define COMPASS_IS QMC5883L



// ------ serial ports for console, Bluetooth, ESP8266 -----------------------------

// Due has two serial ports: Native (SerialUSB) and Programming (Serial) -
// redirect 'Console' to -->'SerialUSB' so the Raspberry PI receive all message console data
// redirect 'Console' to -->'Serial' so the Raspberry PI receive all message console data
// Note: for all-in-one ESP32 we define a new FakeSerial port over WiFi


#define Console Serial
//#define Console SerialUSB
#define CONSOLE_BAUDRATE    115200       // baudrate used for Raspberry PI console

//#define Enable_DueWatchdog true
#define Enable_DueWatchdog false

//#define Enable_Screen true
#define Enable_Screen false

#define RaspberryPIPort Serial2  //The PI is connected on NATIVE USB port over USB cable

#define ESP8266port FakeSerial  //esp01 - we use a fake software serial port for pfod over WiFi
#define ESP8266_BAUDRATE    115200      // baudrate used for communication with esp8266 Wifi module

#define Bluetooth Serial2  // Ardumower default OK for ESP32 or HC05
#define BLUETOOTH_BAUDRATE  19200      // baudrate used for communication with Bluetooth module (Ardumower default: 19200)
#define BLUETOOTH_PIN       1234

#define GpsPort Serial2  // GPS do not forget workarround if PCB1.3 use

// ------- RTC  and EEPROM I2C addresses --------------------------------------------------------------
#define DS1307_ADDRESS B1101000
#define AT24C32_ADDRESS B1010000 //0x50 //Standard PCB1.3 RTC ds1307 memory module
//#define AT24C32_ADDRESS B1010111 //0x57 //Simple PCB RTC ds3231 memory module

// ---- choose only one perimeter signal code ----
#define SIGCODE_1  // Ardumower default perimeter signal
//#define SIGCODE_2  // Ardumower alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal

// ------- ultrasonic config ---------------------------------------------------------
#define NO_ECHO 0

/*
  Ardumower robot chassis
*/

class Mower : public Robot
{
  public:
    Mower();
    virtual void setup(void);
    // virtual void resetMotorFault();
    virtual int readSensor(char type);
    virtual void setActuator(char type, int value);
    virtual void configureBluetooth(boolean quick);
};


extern Mower robot;

#endif
