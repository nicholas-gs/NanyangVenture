// NV10CurrentSensorStats.h

#ifndef _NV11ACCESSORIESSTATUS_h
#define _NV11ACCESSORIESSTATUS_h

#define STATE_EN 1
#define STATE_DS 0
#include <DataPoint.h>
class NV11AccesoriesStatus:public DataPoint
{
 protected:
	uint8_t& lsig = data.Byte[0];
	uint8_t& rsig = data.Byte[1];
	uint8_t& hazard = data.Byte[2];
	uint8_t& headlights = data.Byte[3];
	uint8_t& brake = data.Byte[4];
	uint8_t& wiper = data.Byte[5];
	uint8_t& fourWS = data.Byte[6];
	uint8_t& regen = data.Byte[7];
	
public:
	NV11AccesoriesStatus();

	void setLsig(uint8_t status);
	void setRsig(uint8_t status);
	void setHazard(uint8_t status);
	void setHeadlights(uint8_t status);
	void setBrake(uint8_t status);
	void setWiper(uint8_t status);
	void setFourWS(uint8_t status);
	void setRegen(uint8_t status);
	uint8_t getLsig();
	uint8_t getRsig();
	uint8_t getHazard();
	uint8_t getHeadlights();
	uint8_t getBrake();
	uint8_t getWiper();
	uint8_t getFourWS();
	uint8_t getRegen();
	void insertData(uint8_t lsig, uint8_t rsig, uint8_t hazard, uint8_t headlights, uint8_t brake, uint8_t wiper, uint8_t fourWS, uint8_t regen);
	void packString(char*);
	void unpackString(char * str);
	void printStatus();
	enum eGeneralStates {
		disable,
		enable
	};
	enum eWiperStates { // left, middle, right
		wiperOff,
		wiperSlow,
		wiperFast
	};
	enum eFourWSStates { // left, middle, right
		fourWScounterPhase,
		fourWSfrontOnly,
		fourWSinPhase
	};
};

#endif

