// 
// 
// 
#include "DataPointTemplate.h"
// parameter(CANbytes, stringChars)
// param1(2,4), param2(1,2)
DataPointTemplate::DataPointTemplate(uint8_t CANId) :DataPoint("XX", CANId, 8)
{
}

void DataPointTemplate::insertData(uint32_t param1, uint32_t param2)
{
	this->param1 = param1;
	this->param2 = param2;
}

uint16_t DataPointTemplate::getParam1()
{
	return param1;
}

float DataPointTemplate::getParam2()
{
	return param2;
}

void DataPointTemplate::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// param1 = 4, param2 = 2
	sprintf(shiftedStr, "%04d\t%02d", param1, param2);
}

void DataPointTemplate::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	param1 = atoi(ptr);

	ptr = strtok(NULL, "\t");
	param2 = strtod(ptr, NULL);
}
