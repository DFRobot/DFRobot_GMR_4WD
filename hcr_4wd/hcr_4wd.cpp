#include <Arduino.h>


//calc checksum in hcr protocol string
uint8_t calcChecksum (uint8_t *theBuf) {
	int leng = theBuf[3] + 5;
	uint8_t sum = 0;
	for (int i=0; i<leng; i++) {
		sum += theBuf[i];
	}
	return sum;
}

//fill checksum to hcr protocol string
void fillChecksum (uint8_t *theBuf) {
	int leng = theBuf[3] + 5;
	theBuf[leng] = calcChecksum (theBuf);
}

//test checksum in hcr protocol string
boolean checksum (uint8_t *theBuf, uint8_t theMax) {
	uint8_t sumsub = theBuf[3]+ 5;
	if (sumsub >= theMax)
		return false;

	uint8_t sum = calcChecksum (theBuf);
	if (sum == theBuf[sumsub])
		return true;
	else 
		return false;
}

