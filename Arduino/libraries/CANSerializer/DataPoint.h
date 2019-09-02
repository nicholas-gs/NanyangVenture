// DataPoint.h

#ifndef _DATAPOINT_h
#define _DATAPOINT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#define DEBUG 0
#if DEBUG
#define debug_(str) Serial.print(str)
#define debug(str)  Serial.println(str)
#else
#define debug_(...)
#define debug(...)
#endif

// TODO: add in comments to explain rationale behind every child class variable being a reference to element of union "data".
// -> they are actually messages to be sent onto CAN
// -> they are constrained to add up to 8 bytes max. Multi-frame transmission currently not supported.

/* Usage flow for send: 
	call packCAN / packString with supplied CANFrame* / char* 
	call CAN->sendMsgBuf / Serial.print
Usage flow for receive:
	call checkMatchCAN / checkMatchString on every dataPoint listening on an input
	if find match:
		call CAN->readMsgBuf / Serial.read
		call unpackCAN / unpackString
	else:
		report stray message & suggest hardware filter for CAN

*/

extern const char* STRING_HEADER[];// Fuel Cell, Current Sensor, current sensor stats, Speedometer, Hydrogen Tank Bar, Status of lights, Commands, Heartbeat
void debugPrint(char* toPrint, int len);
void print(char * toPrint);
class CANFrame
{
public:
	unsigned long id;
	byte length;
	byte payload[8]; // TODO: extend payload byte length, then perform slicing in CANSerializer
};
class DataPoint
{
public:
	/// <summary>
	/// Provide a standard CAN ID for each DataPoint
	/// </summary>
	/// <param name="id">value between 0 - 0x7FF</param>
	//virtual void insertData() = 0; cancelled due to different function signatures in different implementations
	bool checkMatchCAN(const CANFrame * f);
	bool checkMatchString(char * str);
	virtual void packCAN(CANFrame*);
	virtual void unpackCAN(const CANFrame*);
	virtual void packString(char * str);
	virtual void unpackString(char * str);
	// by default, data requires broadcast when changed since last call
	bool dataRequiresBroadcast();
	void printRaw();
protected:
	DataPoint(const char* strHeader, uint8_t CANId, const uint8_t CANLength);
	char strHeader[3];
	unsigned long timeStamp;
	const uint8_t CANLength;
	uint8_t CANId;
	union uCanPayload{
		char String[8];
		byte Byte[8];
		uint8_t Int8[8];
		uint16_t UInt[4];
		uint32_t Long[2];
		float Float[2];
	};
	union uCanPayload data;
	void setCanId(uint8_t id);
	uint8_t getCanId();
	const char* getStringHeader();
	char* packStringDefault(char * str);
	char * unpackStringDefault(char * str);
private:
	union uCanPayload oldData;
	//eEncodingPreset presets[8];
	//byte errorFlag = 0;
	// TODO: error flag appropriate?
};

#endif

