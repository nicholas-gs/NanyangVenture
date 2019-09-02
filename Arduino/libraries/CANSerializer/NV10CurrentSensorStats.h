// NV10CurrentSensorStats.h

#ifndef _NV10CURRENTSENSORSTATS_h
#define _NV10CURRENTSENSORSTATS_h

#include <NV10CurrentSensor.h>
class NV10CurrentSensorStats:public DataPoint
{
 protected:
	 uint32_t& wattMs = data.Long[0];
	 float& ampPeak = data.Float[1];

 public:
	NV10CurrentSensorStats();

	void insertData(uint32_t volt, uint32_t ampMotor);
	uint16_t getWattHours();
	float getAmpPeak();
	void packString(char*);
	void unpackString(char * str);
	void syncTime();
private:
	uint32_t lastTime;
	uint32_t getTimeDiff();
};

#endif

