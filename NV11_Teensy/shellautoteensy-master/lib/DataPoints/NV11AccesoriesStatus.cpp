// 
// 
// 
#include "NV11AccesoriesStatus.h"
// parameter(CANbytes, stringChars)
// param1(2,4), param2(1,2)
NV11AccesoriesStatus::NV11AccesoriesStatus() :DataPoint("ST", 0x10, 8)
{
	debug(F("DataPoint ST:\t0x10\t8"));
}

void NV11AccesoriesStatus::setLsig(uint8_t status)
{
	lsig = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setRsig(uint8_t status)
{
	rsig = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setHazard(uint8_t status)
{
	hazard = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setHeadlights(uint8_t status)
{
	headlights = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setBrake(uint8_t status)
{
	brake = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setWiper(uint8_t status)
{
	wiper = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setFourWS(uint8_t status)
{
	fourWS = status;
	timeStamp = millis();
}

void NV11AccesoriesStatus::setRegen(uint8_t status)
{
	regen = status;
	timeStamp = millis();
}

uint8_t NV11AccesoriesStatus::getLsig()
{
	return lsig;
}

uint8_t NV11AccesoriesStatus::getRsig()
{
	return rsig;
}

uint8_t NV11AccesoriesStatus::getHazard()
{
	return hazard;
}

uint8_t NV11AccesoriesStatus::getHeadlights()
{
	return headlights;
}

uint8_t NV11AccesoriesStatus::getBrake()
{
	return brake;
}

uint8_t NV11AccesoriesStatus::getWiper()
{
	return wiper;
}

uint8_t NV11AccesoriesStatus::getFourWS()
{
	return fourWS;
}

uint8_t NV11AccesoriesStatus::getRegen()
{
	return regen;
}

void NV11AccesoriesStatus::insertData(uint8_t lsig, uint8_t rsig, uint8_t hazard, uint8_t headlights, uint8_t brake, uint8_t wiper, uint8_t fourWS, uint8_t regen)
{
	timeStamp = millis();
	this->lsig = lsig;
	this->rsig = rsig;
	this->hazard = hazard;
	this->headlights = headlights;
	this->brake = brake;
	this->wiper = wiper;
	this->fourWS = fourWS;
	this->regen = regen;
}

void NV11AccesoriesStatus::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// param1 = 4, param2 = 2
	sprintf(shiftedStr, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", lsig, rsig, hazard, headlights, brake, wiper, fourWS, regen);
}

void NV11AccesoriesStatus::unpackString(char * str)
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

	ptr = strtok(NULL, "\t");
	fourWS = atoi(ptr);

	ptr = strtok(NULL, "\t");
	regen = atoi(ptr);
}
void NV11AccesoriesStatus::printStatus()
{
	Serial.print("Lsig: "); Serial.println(lsig);
	Serial.print("Rsig: "); Serial.println(rsig);
	Serial.print("Hazard: "); Serial.println(hazard);
	Serial.print("Headlights: "); Serial.println(headlights);
	Serial.print("Brake: "); Serial.println(brake);
	Serial.print("Wiper: "); Serial.println(wiper);
	Serial.print("4WS: "); Serial.println(fourWS);
	Serial.print("Regen: "); Serial.println(regen);
	Serial.println(F("----------------------"));
}