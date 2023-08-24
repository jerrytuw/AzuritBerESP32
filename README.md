This repo aims at a crude and messy modification for running the essential ardumower code on a **single ESP32**.

So far tested:
 - simple PCB
 - general (basic) running/state control for line move
 - drive motor control for 2 hoverboard motors on ZS-X11H (note: stop input just pulls 0-5V input low)
 - ESP32 adc(man) code, perimeter (2 channels - one used), battery, motor currents
 - 1 sonar simple code (interrupt based)
 - GY-521 IMU - actually with GY-87 - bypass mode for compass added
 - buzzer
 - pfod interface over WIFI pseudo serial
 - Elegant OTA update over WIFI
Not tested:
 - mow motor (but foreseen on PCB)
 - parameter optimizations and simplifications
Planned:
 - add MCP23008 I2C port expander on PCB
 
Notes:
Hoverboard motors are nice but have low torque at low enough speeds...
Also https://github.com/trycoon/liam-esp seems interesting.

Based mainly on:
 
# AzuritBer
Ardumower full odometry version
see https://www.ardumower.de  and https://wiki.ardumower.de/index.php?title=AzuritBer_Firmware_(English)# for more info.

Master branch need PCB1.3 or 1.4 ,ODOMETRY on drive motor and IMU : GY-521 / or GY-88  based on MPU6050 version



OPTIONAL:

        	RFID BOARD PN5180 AS RFID READER and ESP32 dev kit to manage SPI communication with PN5180 and WIFI/BT for arduremote
	
		RASPBERRY for vision or camera streaming:
	
See other branch for GY-87  or MPU-9250/9255 version

GY87 version use a GY87 IMU connected on I2C1 to avoid i2c adress conflit with RTC .

RL1000 version is the code for robomow rl model platform.
	The 3 big mow motor use 3 BTS7960 motor driver and 3 INA226 with R010 instead R100 to manage motor sense over I2C