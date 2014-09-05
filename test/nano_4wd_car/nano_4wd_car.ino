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

#include <DFRobot_utility.h>

DFRobotCar mycar(8, 9, 11, 10);

void setup () {
	mycar.changeDir (true, false);
	pinMode (13, OUTPUT);
}


void loop () {
	mycar.control (100, 0);
	digitalWrite (13, HIGH);
	delay (1000);

	mycar.control (100, 100);
	digitalWrite (13, LOW);
	delay (1000);

	mycar.control (0, 0);
	delay (2000);
}


/*
 *	
 *	DFRobotCar (uint8_t left_en, uint8_t left_pwm, uint8_t right_en, uint8_t right_pwm);
 *	
 *	//switch left and right side direction
 *	void switchDir (bool left, bool right);
 *	
 *	//specify left and right side's speed, round is (-255 ~ 255)        
 *	void control (int16_t left, int16_t right); 
 *	
 */
