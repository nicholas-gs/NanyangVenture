// NV10FuelCell.h

#ifndef _NV10FUELCELL_h
#define _NV10FUELCELL_h

#include <DataPoint.h>
class NV10FuelCell:public DataPoint
{
 protected:

	uint8_t& volts = data.Byte[3];
	uint8_t& amps = data.Byte[4];
	uint8_t& pressure = data.Byte[5];
	uint8_t& temperature = data.Byte[6];
	uint8_t& status = data.Byte[7];
	
	char statusTxt[3];
private:
	enum eStatus
	{
		SD,
		OP,
		IN,
		UNKNOWN
	};
	const char* cStatus[4] = { "SD", "OP", "IN", "UN" };
 public:
	NV10FuelCell();

	float getPressure();
	uint8_t getVolts();
	uint8_t getAmps();
	uint8_t getTemperature();
	const char* getStatus();

	void unpackCAN(const CANFrame*);
	void insertData(char* str);
	void packString(char*);
	void unpackString(char * str);
};

#endif

