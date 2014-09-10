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
 *	hcr protocol checksum process library
 */

#include <Arduino.h>


//fill checksum to hcr protocol string
void fillChecksum (uint8_t *theBuf);

//calc checksum in hcr protocol string
uint8_t calcChecksum (uint8_t *theBuf); 

//test checksum in hcr protocol string
boolean checksum (uint8_t *theBuf, uint8_t theMax); 
