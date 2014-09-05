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
 *	author:	lisper <lisper.li@dfrobot.com> 
 *	board: mega2560
 */
#include <Metro.h>
#include <DFRobot_utility.h>
#include <Wire.h>

#define SerialPort Serial2

#define IIC_SLAVE_ADDR 10


#define ID 0x10

#define IIC_SEND_SIZE  11
#define BACK_SIZE 16
#define SERIAL_MAX_SIZE 25
#define SERIAL_SIZE 11

uint8_t cmdBuffer[SERIAL_MAX_SIZE];


//
void setup() {
	Wire.begin(); // join i2c bus (address optional for master)
	SerialInit();
}

//
void loop() {
	CommReader();
}

//
void SerialInit() { 
	Serial.begin(9600);
	SerialPort.begin(9600);
}

//read command from serial
void CommReader () {
	int cmdLength = read_serial_with_timeout (SerialPort, cmdBuffer, SERIAL_SIZE, 4);
	if (cmdLength == SERIAL_SIZE) {
		if (!checksum (cmdBuffer, SERIAL_MAX_SIZE)) {
			Serial.println ("checksum error!");
			return ;  
		}
		Serial.print ("read serial ");
		serialHex (cmdBuffer, cmdLength);
		parseCmd (cmdBuffer, cmdLength);
	}
}

//parse command that read from serial
void parseCmd (uint8_t *theCmdBuf, uint8_t theLeng) {  //feedback measure value
	switch (theCmdBuf[4]) {
		case 0x03:  //control motor
			iicSend (IIC_SLAVE_ADDR, theCmdBuf, theLeng);
			break;
		case 0x55:  //request data from iic
			delay (10);
			Wire.requestFrom(IIC_SLAVE_ADDR, 11);
			receiveEvent ();
			break;
	}
}


/********************** I2C master receive ***********************/


//i2c receiveEvent
void receiveEvent () {
	uint8_t cmdBuf[11];
	int length = iicRead (cmdBuf);
	if (length != 11) {
		Serial.println ("error! receive length error!");
		return;
	}
	Serial.print ("receive ");
	serialHex (cmdBuf, 11);
	uint8_t backData[BACK_SIZE] = {0x55, 0xaa, ID, 0x08, 0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0d, 0x0a};
	for (int i=2; i<10; i++) {
		backData[i+3] = cmdBuf[i];
	}
	fillChecksum (backData);

	Serial.println ("backData:");
	serialHex (backData, BACK_SIZE);
//////////////////////////////////////////////
	Serial.print ("left:");
	Serial.println (*(long*)(cmdBuf+2));
	Serial.print ("right:");
	Serial.println (*(long*)(cmdBuf+2+4));
//////////////////////////////////////////////
	SerialPort.print ("left:");
	SerialPort.print (*(long*)(cmdBuf+2));
	SerialPort.print ("  ");
	SerialPort.print ("right:");
	SerialPort.println (*(long*)(cmdBuf+2+4));

	serial2Write (backData, BACK_SIZE);

}

//send data to serial2
void serial2Write (uint8_t *theBuf, uint8_t leng) {
	for (int i=0; i<leng; i++) {
		Serial2.write (theBuf[i]);
	}
}

//i2c master send data to slaver
void iicSend (uint8_t theSlave, uint8_t *theBuf, uint8_t leng) {
	Wire.beginTransmission(theSlave);
	for (int i=0; i<leng; i++) {
		Wire.write(theBuf[i]);
		delay (1);
	}
	Wire.endTransmission();
}

//
uint8_t iicRead (uint8_t *theBuf) {
	int leng = Wire.available ();  
	if (leng <= 0)
		return 0;
	for (int i=0; i<leng; i++) {
		theBuf[i] = Wire.read ();  
		//delay (1);
	}
	return leng;
}


//print data to PC in hex for test
void serialHex (uint8_t *thebuf, uint8_t leng) {
	Serial.print (leng);
	Serial.print (":");
	for (int i=0; i<leng; i++) {
		Serial.print (thebuf[i], HEX);
		Serial.print (" ");
	}
	Serial.println ();
}

//calc checksum and return it
uint8_t calcChecksum (uint8_t *theBuf) {
	int leng = theBuf[3] + 5;
	uint8_t sum = 0;
	for (int i=0; i<leng; i++) {
		sum += theBuf[i];
	}
	return sum;
}

//calc checksum and fill to rigth place
void fillChecksum (uint8_t *theBuf) {
	uint8_t sum = calcChecksum (theBuf);
	uint8_t sumsub = theBuf[3]+ 5;
	theBuf[sumsub] = sum;
}

//test checksum, ok return true, or return false 
boolean checksum (uint8_t *theBuf, uint8_t theMax) {
	uint8_t sum = calcChecksum (theBuf);
	uint8_t sumsub = theBuf[3]+ 5;
	if (sumsub >= theMax)
		return false;
	if (sum == theBuf[sumsub])
		return true;
	else 
		return false;
}


