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
 *	board: leonardo
 */
 
#include <Wire.h>
#include <hcr_4wd.h>
 
//#define YAXIS A4
//#define XAXIS A5

#define YAXIS A2
#define XAXIS A3

#define CMD_SIZE 11
#define ID	0x10
int16_t x_value = 0;
int16_t y_value = 0;

int16_t l_speed = 0;
int16_t r_speed = 0;


uint8_t stop_sum = 0;

int speedMax = 255;
int speedMin = -255;

//
void setup () {
	Serial1.begin (9600);
	Serial.begin (9600);
	pinMode (XAXIS, INPUT);
	pinMode (YAXIS, INPUT);
	delay (2000);
}

//
void loop () {
	x_value = analogRead (XAXIS) ;
	y_value = analogRead (YAXIS) ;

	x_value = map (x_value, 0, 1023, speedMin, speedMax);
	y_value = map (y_value, 0, 1023, speedMin, speedMax);

		Serial.print (x_value);
		Serial.print ("  ");
		Serial.println (y_value);
		Serial.println ();

	if (abs (x_value) < 10)
		x_value = 0;
	if (abs (y_value) < 10)
		y_value = 0;

	l_speed = r_speed = y_value;

	if (x_value < 0) {
		if (l_speed > 0)
			l_speed -= -x_value;
		else if (l_speed < 0)
			l_speed += -x_value;
		else {
			l_speed  = x_value;
			r_speed  = -x_value;
		}
	}
	else if (x_value > 0) {
		if (r_speed > 0)
			r_speed -= x_value;
		else if (r_speed < 0)        
			r_speed += x_value;
		else {
			l_speed  = x_value;
			r_speed  = -x_value;
		}
	}

	if (abs (l_speed) < 40) l_speed = 0;
	if (abs (r_speed) < 40) r_speed = 0;

	if (l_speed == 0 && r_speed == 0) {
		stop_sum ++;
	}
	else {
		stop_sum = 0;
	}

	if (stop_sum < 3) {
		uint8_t buf[CMD_SIZE] = {0x55, 0xaa, ID, 0x03, 0x03, 0, 0, 0, 0, 0x0d, 0x0a};
		if (l_speed < 0) {
			l_speed = -l_speed;                  
			bitWrite (buf[5], 4, 1);
		}
		else {
			bitWrite (buf[5], 4, 0);
		}
		if (r_speed < 0) {
			r_speed = -r_speed;
			bitWrite (buf[5], 0, 1);
		} else {
			bitWrite (buf[5], 0, 0);
		}

		buf[6] = (uint8_t)l_speed;
		buf[7] = (uint8_t)r_speed;
		fillChecksum (buf);
		serial1Write (buf, CMD_SIZE);
		serialHex (buf, CMD_SIZE);
		delay (50);
	}
	delay (200);
}

