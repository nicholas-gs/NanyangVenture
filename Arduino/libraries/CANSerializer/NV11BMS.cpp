// 
// 
// 
#include "NV11BMS.h"

NV11BMS::NV11BMS():DataPoint("BM", 0x11, 8)
{
}

void NV11BMS::insertData(float volt, float amp, float temperature, float minCellVolt)
{
	this->volt = volt;
	this->amp = amp;
	this->temperature = temperature;
	this->minCellVolt = minCellVolt;
}

uint16_t NV11BMS::getVolt()
{
	return volt;
}
uint16_t NV11BMS::getAmp()
{
	return amp;
}
uint16_t NV11BMS::getTemperature()
{
	return temperature;
}

void NV11BMS::unpackCAN(const CANFrame * f)
{
	DataPoint::unpackCAN(f);
	volt = volt_ / 10.0;
	amp = amp_ / 10.0;
	minCellVolt = minCellVolt_ / 10000.0;
}

void NV11BMS::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);


#ifdef __AVR__
	//Since AVR does not support float printing, have to use dtostrf
	char voltStr[5], ampStr[5], minCellVoltStr[5];
	dtostrf(volt, 4, 1, voltStr);
	dtostrf(amp, 4, 1, ampStr);
	dtostrf(minCellVolt, 4, 2, minCellVoltStr);
	sprintf(shiftedStr, "%s\t%s\t%d\t%s", voltStr, ampStr, temperature, minCellVoltStr);

#elif defined _SAM3XA_
	sprintf(shiftedStr, "%4.1f\t%4.1f\t%d\t%4.2f", volt, amp, temperature, minCellVolt);
#endif
}

void NV11BMS::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	volt = strtod(ptr, NULL);

	ptr = strtok(NULL, "\t");
	amp = strtod(ptr, NULL);

	ptr = strtok(NULL, "\t");
	temperature = atoi(ptr);

	ptr = strtok(NULL, "\t");
	minCellVolt = strtod(ptr, NULL);
}
