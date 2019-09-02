
#include "NV10CurrentSensor.h"
// parameter(CANbytes, stringChars)
// volts(1,2), ampCapIn(1,2), ampCapOut(1,2), ampMotor(1,2)
NV10CurrentSensor::NV10CurrentSensor():DataPoint("CS", 0x11, 8)
{
	debug(F("DataPoint CS:\t0x0C\t8"));
}

void NV10CurrentSensor::insertData(uint32_t volt, uint32_t ampCapIn, uint32_t ampCapOut, uint32_t ampMotor)
{
	timeStamp = millis();
	this->volt = volt;
	this->ampCapIn = ampCapIn;
	this->ampCapOut = ampCapOut;
	this->ampMotor = ampMotor;
}

void NV10CurrentSensor::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// v = 2, aCin = 2, aCout = 2, aMotor = 2
	sprintf(shiftedStr, "%02d\t%d\t%d\t%d", volt, ampCapIn, ampCapOut, ampMotor);
}

void NV10CurrentSensor::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	volt = atoi(ptr);

	ptr = strtok(NULL, "\t");
	ampCapIn = atoi(ptr);

	ptr = strtok(NULL, "\t");
	ampCapOut = atoi(ptr);

	ptr = strtok(NULL, "\t");
	ampMotor = atoi(ptr);
}

uint16_t NV10CurrentSensor::getVolt()
{
	return volt;
}

uint16_t NV10CurrentSensor::getAmpCapIn()
{
	return ampCapIn;
}

uint16_t NV10CurrentSensor::getAmpCapOut()
{
	return ampCapOut;
}

uint16_t NV10CurrentSensor::getAmpMotor()
{
	return ampMotor;
}


