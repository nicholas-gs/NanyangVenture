// 
// 
// 
#include "NV10CurrentSensorStats.h"
const uint32_t MS_PER_HOUR = 3600000;
const float SHUNT_100A75mV_SCALE = 0.0078125 * (100.0 / 75);
const float SHUNT_200A75mV_SCALE = 0.0078125 * (200.0 / 75);
const float VOLTAGEDIVIDER_SCALE = 1;
// parameter(CANbytes, stringChars)
// wattHour(2,4), ampPeak(1,2)
NV10CurrentSensorStats::NV10CurrentSensorStats() :DataPoint("cs", 0x12, 8)
{
	debug(F("DataPoint cs:\t0x0D\t8"));
}

void NV10CurrentSensorStats::insertData(uint32_t volt, uint32_t ampMotor)
{
	timeStamp = millis();
	
	uint32_t wattMsDelta = volt * ampMotor * getTimeDiff();
	wattMs += wattMsDelta;
	ampPeak = max(ampPeak, ampMotor);
}

uint16_t NV10CurrentSensorStats::getWattHours()
{
	return wattMs/MS_PER_HOUR;
}

float NV10CurrentSensorStats::getAmpPeak()
{
	return ampPeak;
}

void NV10CurrentSensorStats::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// wattHour = 4, aPeak = 2
	sprintf(shiftedStr, "%04d\t%d", getWattHours(), ampPeak);
}

void NV10CurrentSensorStats::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	wattMs = atoi(ptr) * MS_PER_HOUR;

	ptr = strtok(NULL, "\t");
	ampPeak = strtod(ptr, NULL);
}

void NV10CurrentSensorStats::syncTime()
{
	getTimeDiff();
}
uint32_t NV10CurrentSensorStats::getTimeDiff()
{
	uint32_t timeDiff = millis() - lastTime;
	lastTime = millis();
	return timeDiff;
}

