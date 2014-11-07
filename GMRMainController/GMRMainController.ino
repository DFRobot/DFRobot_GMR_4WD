/**************************************************************************
 *  This program is free software: you can redistribute it and/or modify  *
 *  t under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or     *
 *  (at your option) any later version.                                   *
 *                                                                        *
 *  This program is distributed in the hope that it will be useful,       *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *  GNU General Public License for more details.                          *
 *                                                                        *
 *  You should have received a copy of the GNU General Public License     *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *************************************************************************/

/*
 *	Copyright (C) 2014 DFRobot
 *	author:	lisper <lisper.li@dfrobot.com> 
 *	board: mega2560
 */


#include <DFRobot_utility.h>
#include <utility/iic.h>
#include <hcr_4wd.h>

#define CMD_CTRL_MOTOR		0x03
#define CMD_REQUEST		0x55
#define CMD_REQUEST_STRING	0x56
#define CMD_RETURN		0x57

#define SerialPort Serial2

#define IIC_SLAVE		10

#define ID			0x10

#define IIC_SEND_SIZE		11
#define IIC_BACK_SIZE		11 
#define BACK_SIZE		16
#define SERIAL_MAX_SIZE		25
#define SERIAL_SIZE		11


uint8_t cmdBuffer[SERIAL_MAX_SIZE];

#define _debug

//
void setup() {
	Wire.begin(); // join i2c bus (address optional for master)
	/////////////////////////////////////
#ifdef _debug
	Serial.begin(9600);
#endif
	SerialPort.begin(9600);
}

//
void loop() {
	CommReader();
}

//read command from serial
void CommReader () {
	int cmdLength = serialRead (SerialPort, cmdBuffer, SERIAL_SIZE, 4);
	if (cmdLength <= 0)
		return ;
	if (cmdBuffer[0] != 0x55 || cmdBuffer[1] != 0xaa) {
		/////////////////////////////////////
#ifdef _debug
		Serial.println ("header error!");
#endif
		return ;  
	}
	if (cmdBuffer[2] != ID) {
		/////////////////////////////////////
#ifdef _debug
		Serial.println ("ID error!");
#endif
		return ;  
	}
	if (cmdLength != SERIAL_SIZE) {
		/////////////////////////////////////
#ifdef _debug
		Serial.println ("length error!");
#endif
		SerialPort.println ("length error!");
		return ;  
	}
	if (!checksum (cmdBuffer, SERIAL_MAX_SIZE)) {
		/////////////////////////////////////
#ifdef _debug
		Serial.println ("checksum error!");
#endif
		SerialPort.println ("checksum error!");
		return ; 
	}
#ifdef _debug
	Serial.print ("read serial ");	//for debug
	printHex (cmdBuffer, cmdLength);
#endif
	parseCmd (cmdBuffer, cmdLength);
}

//parse command that read from serial
void parseCmd (uint8_t *theCmdBuf, uint8_t theLeng) {  //feedback measure value
	switch (theCmdBuf[4]) {
		case CMD_CTRL_MOTOR:  //control motor
			iicWrite (IIC_SLAVE, theCmdBuf, theLeng);
			break;
		case CMD_REQUEST:  //request data from iic
			delay (5);
			receiveEvent (false);
			break;
		case CMD_REQUEST_STRING:
			delay (5);
			receiveEvent (true);
			break;
	}
}


/********************** I2C master receive ***********************/


//i2c receiveEvent
void receiveEvent (boolean isString) {
	uint8_t cmdBuf[IIC_BACK_SIZE];
	int length = iicRead (IIC_SLAVE, cmdBuf, IIC_BACK_SIZE);
	if (length != IIC_BACK_SIZE) {
		/////////////////////////////////////
#ifdef _debug
		Serial.println ("error! receive length error!");
#endif

		return;
	}

	/////////////////////////////////////
#ifdef _debug
	Serial.print ("receive ");
	printHex (cmdBuf, IIC_BACK_SIZE);
#endif

	if (isString) {

		/////////////////////////////////////
#ifdef _debug
		Serial.print ("left:");
		Serial.print (*(long*)(cmdBuf+2));
		Serial.print ("cm  ");
		Serial.print ("right:");
		Serial.print (*(long*)(cmdBuf+2+4));
		Serial.println ("cm");
#endif

		SerialPort.print ("left:");
		SerialPort.print (*(long*)(cmdBuf+2));
		SerialPort.print ("cm  ");
		SerialPort.print ("right:");
		SerialPort.print (*(long*)(cmdBuf+2+4));
		SerialPort.println ("cm");
	} else { 
		uint8_t backData[BACK_SIZE] = {0x55, 0xaa, ID, 0x08, CMD_RETURN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0d, 0x0a};
		for (int i=2; i<10; i++) {
			backData[i+3] = cmdBuf[i];
		}
		fillChecksum (backData);

		/////////////////////////////////////
#ifdef _debug
		Serial.println ("backData:");
		printHex (backData, BACK_SIZE);
#endif

		serial2Write (backData, BACK_SIZE);
	}
}


