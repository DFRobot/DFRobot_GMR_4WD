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
#include <Servo.h>
#include <Metro.h>
#include <Wire.h>
#include <DFRobot_utility.h>

//Metro DataTrans = Metro(200,true);
Metro BehaviorInterval = Metro(25,true);

#define LF 0
#define RT 1
#define Pbn  11
#define IIC_READ_SIZE 11

int Lduration,Rduration;
boolean LcoderDir,RcoderDir;
const byte encoder0pinA = 2;
const byte encoder0pinB = 4;
byte encoder0PinALast;
const byte encoder1pinA = 3;
const byte encoder1pinB = 5;
byte encoder1PinALast;
int RotationCoder[2];
int pastCoder[2];

long totalCoder[2];

int _speedtarget[2];
double _lasterror[2];
double _preverror[2];

float _proportion;
float _integral;
float _derivative;
float _maximum;
float _minimum;

Servo MotorLeft;
Servo MotorRight;
float _speedleft;
float _speedright;
float _perimeterA;
float _FirmPulsePG;

byte wheelDir = 0x00;
int _CMDspeed[2] = {
	0,0};

DFRobotCar myCar (8, 9, 11, 10);

void setup() {
	delay(1000);
	Wire.begin(10);

	Wire.onRequest(requestEvent); // register event
	Wire.onReceive(receiveEvent); // register event
	myCar.changeDir (true, false);

	_speedtarget[LF] = 0;
	_speedtarget[RT] = 0;


	//  while(true);  

	Serial.begin(9600);
	CoderInit();

	_perimeterA = 43.6*1000;
	_FirmPulsePG = 4250;

	_proportion = 3;//4
	_integral = 0.5;//1
	_derivative = 0.6;//1
	_maximum = 500;
	_minimum = _maximum*(-1);

	//  Serial.print("Perimeter\t");
	//  Serial.println(_perimeterA);
	//  while(!Serial.available());
}

//
void loop() {
	//if(DataTrans.check()) {    
	//    Serial.print(_speedtarget[LF]);
	//    Serial.print(",");
	//    Serial.print(_speedtarget[RT]);
	//    Serial.print(" ");
	//    Serial.print(pastCoder[LF]);
	//    Serial.print(",");
	//    Serial.println(pastCoder[RT]);
	//
	//    Serial.print(Lduration);
	//    Serial.print(",");
	//    Serial.println(Rduration);
	//    Serial.print("\n");
	/*
	 */
	//} 
	if(BehaviorInterval.check()) {
		static int lastLspeed = 0;
		static int lastRspeed = 0;
		ResentSpeed();
		int limitMax = 300;
		int limitMin = -300;


		int _Loutput,_Routput;
		float Lpara,Rpara;
		Lpara = TVPIDcal(_speedleft,true);
		Rpara = TVPIDcal(_speedright,false);

		//    Serial.print(Lpara);

		// _Loutput = int(TVAffect(Lpara));
		// _Routput = int(TVAffect(Rpara));

		_Loutput = map (constrain (Lpara, limitMin, limitMax), -limitMin, limitMax,  -255, 255);
		_Routput = map (constrain (Rpara, limitMin, limitMax), -limitMin, limitMax,  -255, 255);
		//Serial.print (_Loutput);
		//Serial.print (" ");
		//Serial.println (_Routput);
		//myCar.control (_Loutput, _Routput);
		myCar.control (_speedtarget[LF], _speedtarget[RT]);
		//    lastLspeed = _Loutput;
		//    lastRspeed = _Routput;

		//		    Serial.print("LW:");
		//		    Serial.print(_speedleft);
		//		    Serial.print(",");
		//		    Serial.print(_speedtarget[LF]);
		//		    Serial.print(",");
		//		    Serial.print(Lpara);
		//		    Serial.print(",");
		//		    Serial.print(_Loutput);
		//    Serial.print("\tRW:");
		//    Serial.print(_speedright);
		//    Serial.print(",");
		//    Serial.print(_speedtarget[RT]);
		//    Serial.print(",");
		//    Serial.print(Rpara);
		//    Serial.print(",");
		//    Serial.println(_Routput);
	}
}

//
void CoderInit() {
	LcoderDir = true;
	RcoderDir = true;	

	//for(int i = 2;i<4; i ++)  pinMode(i,INPUT);

	pinMode (encoder0pinA, INPUT_PULLUP);
	pinMode (encoder1pinA, INPUT_PULLUP);
	pinMode (encoder0pinB, INPUT_PULLUP);
	pinMode (encoder1pinB, INPUT_PULLUP);
	attachInterrupt (LF, LwheelSpeed, CHANGE);
	attachInterrupt (RT, RwheelSpeed, CHANGE);
}


/************************************************* Motor Control ***********************************************/

/*
   void Motor(int value,byte whichwheel) {
   value = constrain(value,1000,2000);
//
// MotorLeft.write(90);
//MotorRight.write(90);
///
if(whichwheel == LF){
MotorLeft.writeMicroseconds(value);
//    MotorLeft.write(100);
}
else if(whichwheel == RT){
MotorRight.writeMicroseconds(value);
//    MotorRight.write(100);
}
}
 */

/************************************************ calculate speed (cm/s) ***********************************************/

float lastspeed (int longs,int diff) {
	double internum = _perimeterA;
	internum /= diff;
	float dist = float(internum*longs);
	dist /= _FirmPulsePG;
	return dist;
}

//
void ResentSpeed () {
	static int pasttime;
	static unsigned long lasttime;
	static unsigned long now;
	now = millis();
	pasttime = now - lasttime;
	lasttime = now;

	_speedleft = lastspeed (Lduration , pasttime);
	_speedright = lastspeed (Rduration , pasttime);

	//Serial.println(pasttime);
	// Serial.print("  ");
	//Serial.println(Lduration);
	//////////////////////////////////
	pastCoder[LF] += Lduration;
	pastCoder[RT] += Rduration;

	Lduration = 0;
	Rduration = 0;
}

/************************************************* PID Control ***********************************************/

//
float TVPIDcal (float prevspeed,boolean target) {
	static int sumerror[2];
	static int i;
	if (target)
		i = 0;
	else
		i = 1;
	int derror;
	int error = _speedtarget[i] - prevspeed;

	sumerror[i] += error;
	sumerror[i] = min(_maximum,sumerror[i]);//limit the range of intergral segment
	sumerror[i] = max(_minimum,sumerror[i]);

	derror = _lasterror[i] - _preverror[i];
	_preverror[i] = _lasterror[i];
	_lasterror[i] = error;

	return (_proportion*error+_integral*sumerror[i]+_derivative*derror);
}

/*
   int TVAffect(float pidpara) {
   float result = 500;
   float factor;

   if(pidpara>_maximum)
   factor = 1;
   else if (pidpara>0)
   factor = pidpara/_maximum;
   else if (pidpara<_minimum)
   factor = -1;
   else
   factor = pidpara/_maximum;

   result *= factor;
   result += 1500;

   return result;
   }
 */

/************************** Interrupt *****************************/


//
void LwheelSpeed() {
	int Lstate = digitalRead(encoder0pinA);
	if ((encoder0PinALast == LOW) && Lstate==HIGH) {
		int val = digitalRead(encoder0pinB);
		if (val == LOW && LcoderDir)  
			LcoderDir = false; //Lreverse
		else if (val == HIGH && !LcoderDir)  
			LcoderDir = true;  //Lforward
	}
	encoder0PinALast = Lstate;

	if(!LcoderDir)  
		Lduration++;
	else  
		Lduration--;

	if(true)  
		RotationCoder[LF] ++;
	else 
		RotationCoder[LF] = 0;
}

//
void RwheelSpeed()
{
	int Rstate = digitalRead(encoder1pinA);
	if((encoder1PinALast == LOW)&&Rstate==HIGH)
	{
		int val = digitalRead(encoder1pinB);
		if(val == LOW && RcoderDir)  
			RcoderDir = false; //Rreverse
		else if (val == HIGH && !RcoderDir)  
			RcoderDir = true;  //Rforward
	}
	encoder1PinALast = Rstate;

	if(!RcoderDir)  
		Rduration++;
	else  
		Rduration--;

	if(true)  
		RotationCoder[RT] ++;
	else 
		RotationCoder[RT] = 0;
}

/********************** I2C Slaver ***********************/

uint8_t iicReadBuf[IIC_READ_SIZE];

//
void receiveEvent (int HowMany) {
	if (iicRead (iicReadBuf) != IIC_READ_SIZE) {	//check length
		Serial.println ("error! command length error!");
		return;
	}
	if (iicReadBuf[0] != 0x55 || iicReadBuf[1] != 0xaa) {	//check header
		Serial.println ("error! command header error!");
		return;
	}
	if (!checksum (iicReadBuf, IIC_READ_SIZE)) {	//test checksum
		Serial.println ("error! checksum error!");
		return;
	}

	serialHex (iicReadBuf, IIC_READ_SIZE);

	switch (iicReadBuf[4]) {	//get command
		case 0x03:
			// controlDelayTime = millis ();
			if (iicReadBuf[5] & 0x01)
				_speedtarget[LF] = -iicReadBuf[6]; 
			else 
				_speedtarget[LF] = iicReadBuf[6]; 

			if (iicReadBuf[5] & 0x10)
				_speedtarget[RT] = -iicReadBuf[7]; 
			else 
				_speedtarget[RT] = iicReadBuf[7]; 
			break;
			//default:
			//Serial.println ("error! no this command!");
	}
}

//
void requestEvent() {  
	byte RequestString[Pbn] = {
		'F','E',0,0,0,0,0,0,0,0,10};

	RequestString[2] = 0x00;

	if(totalCoder[LF] < 0)  
		bitWrite (RequestString[2], 4, 1);
	if(totalCoder[RT] < 0)  
		bitWrite (RequestString[2], 0, 1);

	totalCoder[LF] += pastCoder[LF];
	totalCoder[RT] += pastCoder[RT];

	* (long*) (RequestString+2) = totalCoder[LF]; 
	* (long*) (RequestString+2+4) = totalCoder[RT]; 

	pastCoder[LF] = 0;
	pastCoder[RT] = 0;

	Wire.write (RequestString,Pbn);
}

//
uint8_t  iicRead (uint8_t *theBuf) {
	int leng = Wire.available ();  
	if (leng <= 0)
		return 0;
	for (int i=0; i<leng; i++) {
		theBuf[i] = Wire.read ();  
	}
	return leng;
}

//
void serialHex (uint8_t *thebuf, uint8_t leng) {
	Serial.print (leng);
	Serial.print (":");
	for (int i=0; i<leng; i++) {
		Serial.print (thebuf[i], HEX);
		Serial.print (" ");
	}
	Serial.println ();
}

//
void fillChecksum (uint8_t *theBuf) {
	int leng = theBuf[3] + 5;
	theBuf[leng] = getChecksum (theBuf);
}

//
uint8_t getChecksum (uint8_t *theBuf) {
	int leng = theBuf[3] + 5;
	uint8_t sum = 0;
	for (int i=0; i<leng; i++) {
		sum += theBuf[i];
	}
	return sum;
}

//
boolean checksum (uint8_t *theBuf, uint8_t theMax) {
	uint8_t sum = getChecksum (theBuf);
	uint8_t sumsub = theBuf[3]+ 5;
	if (sumsub >= theMax)
		return false;
	if (sum == theBuf[sumsub])
		return true;
	else 
		return false;
}


