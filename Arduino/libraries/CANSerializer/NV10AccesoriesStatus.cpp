// 
// 
// 
#include "NV10AccesoriesStatus.h"
// parameter(CANbytes, stringChars)
// param1(2,4), param2(1,2)
NV10AccesoriesStatus::NV10AccesoriesStatus() :DataPoint("ST", 0x10, 6)
{
	debug(F("DataPoint ST:\t0x10\t6"));
}

void NV10AccesoriesStatus::setLsig(uint8_t status)
{
	lsig = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::setRsig(uint8_t status)
{
	rsig = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::setWiper(uint8_t status)
{
	wiper = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::setHazard(uint8_t status)
{
	hazard = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::setHeadlights(uint8_t status)
{
	headlights = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::setBrake(uint8_t status)
{
	brake = status;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleLsig()
{
	lsig = !lsig;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleRsig()
{
	rsig = !rsig;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleWiper()
{
	wiper = !wiper;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleHazard()
{
	hazard = !hazard;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleHeadlights()
{
	headlights = !headlights;
	timeStamp = millis();
}

void NV10AccesoriesStatus::toggleBrake()
{
	brake = !brake;
	timeStamp = millis();
}

uint8_t NV10AccesoriesStatus::getLsig()
{
	return lsig;
}

uint8_t NV10AccesoriesStatus::getRsig()
{
	return rsig;
}

uint8_t NV10AccesoriesStatus::getWiper()
{
	return wiper;
}

uint8_t NV10AccesoriesStatus::getHazard()
{
	return hazard;
}

uint8_t NV10AccesoriesStatus::getHeadlights()
{
	return headlights;
}

uint8_t NV10AccesoriesStatus::getBrake()
{
	return brake;
}

void NV10AccesoriesStatus::insertData(uint8_t lsig, uint8_t rsig, uint8_t wiper, uint8_t hazard, uint8_t headlights, uint8_t brake)
{
	timeStamp = millis();
	this->lsig = lsig;
	this->rsig = rsig;
	this->wiper = wiper;
	this->hazard = hazard;
	this->headlights = headlights;
	this->brake = brake;
}

void NV10AccesoriesStatus::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// param1 = 4, param2 = 2
	sprintf(shiftedStr, "%d\t%d\t%d\t%d\t%d\t%d", lsig, rsig, hazard, headlights, brake, wiper);
}

void NV10AccesoriesStatus::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	lsig = atoi(ptr);

	ptr = strtok(NULL, "\t");
	rsig = atoi(ptr);

	ptr = strtok(NULL, "\t");
	hazard = atoi(ptr);

	ptr = strtok(NULL, "\t");
	headlights = atoi(ptr);

	ptr = strtok(NULL, "\t");
	brake = atoi(ptr);

	ptr = strtok(NULL, "\t");
	wiper = atoi(ptr);
}
