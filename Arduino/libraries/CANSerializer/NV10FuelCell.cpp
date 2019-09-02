// 
// 
// 

#include "NV10FuelCell.h"
// parameter(CANbytes, stringChars)
// Watts(2,4), pressure(4,4), temperature(1,2), status(1,2)
NV10FuelCell::NV10FuelCell():DataPoint("FC", 0x13, 8)
{
	debug(F("DataPoint FC:\t0x0B\t8"));
}

float NV10FuelCell::getPressure()
{
	return pressure / 10.0;
}

uint8_t NV10FuelCell::getVolts()
{
	return volts;
}
uint8_t NV10FuelCell::getAmps()
{
	return amps;
}

uint8_t NV10FuelCell::getTemperature()
{
	return temperature;
}

const char * NV10FuelCell::getStatus()
{
	return statusTxt;
}

void NV10FuelCell::insertData(char* str)
{
	// slice up the string
	if (strlen(str) >= 77)
	{
		char* readPtr = strtok(str, ">");	// str + 1 to skip the first char '>', strtok will find the 2nd one
		if (readPtr == NULL)
			return;

		//updated = true;
		timeStamp = millis();
		// update respective variables

		readPtr = strtok(readPtr, " ");
		volts = atoi(readPtr);
		//loggedParams[volts] = atof(readPtr);

		readPtr = strtok(NULL, " ");
		amps = atoi(readPtr);
		//loggedParams[amps] = atof(readPtr);

		readPtr = strtok(NULL, " ");
		//loggedParams[watts] = atof(readPtr);

		readPtr = strtok(NULL, " ");
		//loggedParams[energy] = atof(readPtr);

		// take max stack temperature out of 4 readings
		float thisMaxTemp = 0;
		for (int i = 0; i < 4; i++)
		{
			readPtr = strtok(NULL, " ");
			thisMaxTemp = max(atof(readPtr), thisMaxTemp);
		}
		//loggedParams[maxTemperature] = thisMaxTemp;
		temperature = thisMaxTemp;
		readPtr = strtok(NULL, " ");
		//loggedParams[pressure] = atof(readPtr);
		pressure = atof(readPtr) * 10;
		readPtr = strtok(NULL, " ");
		//loggedParams[capacitorVolts] = atof(readPtr);

		readPtr = strtok(NULL, " ");
		// skip average temperature

		readPtr = strtok(NULL, " ");
		strcpy(statusTxt, readPtr);

		if (strcmp(readPtr, "OP") == 0)
			status = OP;	// alive
		else if (strcmp(readPtr, "SD") == 0)
			status = SD;	// SHUTDOWN! 
		else if (strcmp(readPtr, "IN") == 0)
			status = IN; // initializing
		else
			status = UNKNOWN;

		//>>00.0V 00.0A 0000W 00000Wh 021.1C 028.3C 028.5C 031.6C 0.90B 59.0V 028.0C IN 00.0C 00 0000
		//  ^   * ^   * ^   * ^    *  ^    * ^    * ^    * ^    * ^   * ^   *        ^ *
		//  ".........:.........:.........:.........:.........".........:.........:.........:.........:........."
	}
	// TODO: remember to terminate the string in the main code!
}

void NV10FuelCell::unpackCAN(const CANFrame* f)
{
	DataPoint::unpackCAN(f);
	strcpy(statusTxt, cStatus[status]); // status takes on enum value, match with constant Status array string
}

void NV10FuelCell::packString(char * str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// p = 4.2, t = 2, w = 4, st = 2
#ifdef __AVR__
	char pressureString[8];
	dtostrf(pressure/10.0, 3, 1, pressureString);
	sprintf(shiftedStr, "%d\t%d\t%s\t%2d\t%s", volts, amps, pressureString, temperature, statusTxt);

#elif defined _SAM3XA_
	sprintf(shiftedStr, "%d\t%d\t%4.1f\t%2d\t%s", volts, amps, pressure/10.0, temperature, statusTxt);
#endif
}

void NV10FuelCell::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	ptr = strtok(NULL, "\t");
	volts = atoi(ptr);

	ptr = strtok(NULL, "\t");
	amps = atoi(ptr);

	ptr = strtok(NULL, "\t");
	pressure = strtod(ptr, NULL) * 10;

	ptr = strtok(NULL, "\t");
	temperature = atoi(ptr);

	ptr = strtok(NULL, "\t");
	strcpy(statusTxt, ptr);
	//eStatus statusCode = (eStatus)atoi(ptr);
	//switch (statusCode)
	//{
	//case OP:
	//	strcpy(statusTxt, "OP");
	//	break;
	//case IN:
	//	strcpy(statusTxt, "IN");
	//	break;
	//case SD:
	//	strcpy(statusTxt, "SD");
	//	break;
	//default:
	//	strcpy(statusTxt, "--");
	//	break;
	//}

}

