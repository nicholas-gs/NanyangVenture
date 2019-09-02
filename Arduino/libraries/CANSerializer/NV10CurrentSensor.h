// NV10CurrentSensor.h

#ifndef _NV10CURRENTSENSOR_h
#define _NV10CURRENTSENSOR_h

#include <DataPoint.h>
class NV10CurrentSensor:public DataPoint
{
 protected:
	 uint16_t& volt = data.UInt[0];
	 uint16_t& ampCapIn = data.UInt[1];
	 uint16_t& ampCapOut = data.UInt[2];
	 uint16_t& ampMotor = data.UInt[3];

 public:
	NV10CurrentSensor();

	void insertData(uint32_t volt, uint32_t ampCapIn, uint32_t ampCapOut, uint32_t ampMotor);

	void packString(char*);
	void unpackString(char * str);

	uint16_t getVolt();
	uint16_t getAmpCapIn();
	uint16_t getAmpCapOut();
	uint16_t getAmpMotor();
};

#endif

