// NV10CurrentSensorStats.h

#ifndef _DATAPOINTTEMPLATE_h
#define _DATAPOINTTEMPLATE_h

#include <DataPoint.h>
class DataPointTemplate:public DataPoint
{
 protected:
	 uint32_t& param1 = data.Long[0];
	 float& param2 = data.Float[1]; // params can be higher resolution than their packed variants (this float is packed as uint16_t in .cpp file)

 public:
	DataPointTemplate(uint8_t CANId);

	void insertData(uint32_t param1, uint32_t param2);
	uint16_t getParam1();
	float getParam2();
	void packString(char*);
	void unpackString(char * str);
};

#endif

