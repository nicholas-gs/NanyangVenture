// NV10CurrentSensorStats.h

#ifndef _NV11DATASPEEDO_h
#define _NV11DATASPEEDO_h

#include <DataPoint.h>
class NV11DataSpeedo:public DataPoint
{
 protected:
	 float& speed = data.Float[0];

 public:
	NV11DataSpeedo();
	// implement parent class
	void insertData(float speed);
	float getSpeed();
	void packString(char*);
	void unpackString(char * str);
};
#endif

