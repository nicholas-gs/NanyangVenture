/*
 Name:		CANSerializer.h
 Created:	2/15/2019 3:38:38 PM
 Author:	MX
 Editor:	http://www.visualmicro.com
*/

#ifndef _CANSERIALIZER_h
#define _CANSERIALIZER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <mcp_can.h>
#include <DataPoint.h>
class CANSerializer
{
private:
	MCP_CAN* CAN;
	//DataPoint* dataPoints[1]; // TODO: implement convertion to raw CAN bytes only inside CANSerializer
public:
	bool init(uint8_t cs, uint16_t speed = 1000);
	bool sendCanFrame(CANFrame*);
	// Populates Packet only if data is available.
	bool receiveCanFrame(CANFrame*);
	bool checkNoError();
};
#endif