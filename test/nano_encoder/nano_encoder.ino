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
 *	board: nano
 */

#define PIN_LEFT_I 3	//int0
#define PIN_LEFT_D 5

#define PIN_RIGHT_I 2	//int1
#define PIN_RIGHT_D 4

volatile int32_t countRight = 0; //
volatile int32_t countRight1 = 0; 
volatile int32_t countLeft = 0; //
volatile int32_t countLeft1 = 0; 


//
void setup() {
	Serial.begin(9600);

	pinMode(PIN_LEFT_I,INPUT_PULLUP);
	pinMode(PIN_RIGHT_I,INPUT_PULLUP);
	pinMode(PIN_LEFT_D,INPUT_PULLUP);
	pinMode(PIN_RIGHT_D,INPUT_PULLUP);

	attachInterrupt(0,funcLeft , CHANGE); 
	attachInterrupt(1,funcRight, CHANGE);
}

//
void loop() {
	//while count changed print it
	while (countRight1 != countRight || countLeft != countLeft1) {
		countRight1 = countRight;
		countLeft1 = countLeft;
		Serial.print (countLeft); 
		Serial.print ("  "); 
		Serial.println (countRight); 
	}
}

//int0
void funcRight () {
	int dataA = digitalRead (PIN_RIGHT_I);
	int dataAd = digitalRead (PIN_RIGHT_D);
	if	( dataA &&  dataAd)	countRight++;
	else if ( dataA && ~dataAd)	countRight--;
	else if (~dataA &&  dataAd)	countRight--;
	else if (~dataA && ~dataAd)	countRight++;
}


//int1
void funcLeft () {
	int dataB = digitalRead (PIN_LEFT_I);
	int dataBd = digitalRead (PIN_LEFT_D);
	if	( dataB &&  dataBd)	countLeft++;
	else if ( dataB && ~dataBd)	countLeft--;
	else if (~dataB &&  dataBd)	countLeft--;
	else if (~dataB && ~dataBd)	countLeft++;
}



