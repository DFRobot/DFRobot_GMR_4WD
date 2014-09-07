#include <Arduino.h>


//fill checksum to hcr protocol string
void fillChecksum (uint8_t *theBuf);

//calc checksum in hcr protocol string
uint8_t calcChecksum (uint8_t *theBuf); 

//test checksum in hcr protocol string
boolean checksum (uint8_t *theBuf, uint8_t theMax); 
