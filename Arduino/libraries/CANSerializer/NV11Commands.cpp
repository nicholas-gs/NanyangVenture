// 
// 
// 
#include "NV11Commands.h"

NV11Commands::NV11Commands() :DataPoint("CM", 0x12, 4)
{
}

void NV11Commands::insertData(uint8_t horn)
{
	this->horn = horn;
}

void NV11Commands::clearActivationHistory()
{
	horn = lapCounter = lapCounterReset = shutdownPi = notTriggered;
}

void NV11Commands::triggerHorn()
{
	horn = triggered;
}

void NV11Commands::triggerLaps()
{
	lapCounter = triggered;
}

void NV11Commands::triggerLapsReset()
{
	lapCounterReset = triggered;
}

void NV11Commands::setLapTime(uint8_t time)
{
	lapTime = time;
}

void NV11Commands::triggerShutdownRPi()
{
	shutdownPi = triggered;
}

uint8_t NV11Commands::getHorn()
{
	return horn;
}
uint8_t NV11Commands::getLapTrig()
{
	return lapCounter;
}

uint8_t NV11Commands::getLapTime()
{
	return lapTime;
}

void NV11Commands::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// horn = 1
	sprintf(shiftedStr, "%d\t%d\t%d\t%d", horn, lapCounter, lapCounterReset, shutdownPi);
}

void NV11Commands::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	horn = atoi(ptr);
	ptr = strtok(NULL, "\t");
	lapCounter = atoi(ptr);
	ptr = strtok(NULL, "\t");
	lapCounterReset = atoi(ptr);
	ptr = strtok(NULL, "\t");
	shutdownPi = atoi(ptr);
}

bool NV11Commands::dataHasChanged()
{
	for (int i = 0; i < CANLength; i++)
	{
		if (data.Byte[i] == triggered)
			return true;
	}
	return false;
}
