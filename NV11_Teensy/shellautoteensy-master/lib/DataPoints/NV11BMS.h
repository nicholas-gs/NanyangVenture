// NV10CurrentSensorStats.h

#ifndef _NV11BMS_h
#define _NV11BMS_h

#include <DataPoint.h>
class NV11BMS:public DataPoint
{
 protected:
	 // received "Pack Summed Voltage" must divide by 100, "Pack Instant Voltage" must divide by 10
	 uint16_t& volt_ = data.UInt[0];
	 // received amps must divide by 100
	 uint16_t& amp_ = data.UInt[1];
	 // received temperature is as it is
	 uint16_t& temperature = data.UInt[2];
	 uint16_t& minCellVolt_ = data.UInt[3];
	 float volt, amp, minCellVolt;

 public:
	NV11BMS();
	void insertData(float volt, float amp, float temperature, float minCellVolt);
	uint16_t getVolt();
	uint16_t getAmp();
	uint16_t getTemperature();
	void unpackCAN(const CANFrame *f);
	void packString(char*);
	void unpackString(char * str);
};

#endif

