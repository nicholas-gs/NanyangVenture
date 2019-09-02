// 
// 
// 
#include "NV11DataSpeedo.h"
// parameter(CANbytes, stringChars)
// param1(2,4), param2(1,2)
NV11DataSpeedo::NV11DataSpeedo() :DataPoint("SM", 0x0A, 4)
{
	debug(F("DataPoint SM:\t0x0A\t4"));
}

void NV11DataSpeedo::insertData(float speed)
{
	timeStamp = millis();
	this->speed = speed;
}

float NV11DataSpeedo::getSpeed()
{
	return speed;
}

void NV11DataSpeedo::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// speed = 4bytes
#ifdef __AVR__
	char tmp[8];
	dtostrf(speed, 4, 1, tmp);
	sprintf(shiftedStr, "%s", tmp);

#elif defined _SAM3XA_
	sprintf(shiftedStr, "%4.1f", speed);
#endif
}

void NV11DataSpeedo::unpackString(char * str)
{
	//char* ptr = DataPoint::unpackStringDefault(str);
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	speed = atof(ptr);
}
