#ifndef _NV11DATAULTRASONIC_h
#define _NV11DATAULTRASONIC_h

#include <DataPoint.h>
class NV11DataUltrasonic :public DataPoint
{
protected:
	uint8_t rightFront = data.Int8[0];
	uint8_t rightSide = data.Int8[1];
	uint8_t rightBack = data.Int8[2];
	uint8_t leftFront = data.Int8[3];
	uint8_t leftSide = data.Int8[4];
	uint8_t leftBack = data.Int8[5];
	uint8_t front = data.Int8[6];

public:
	NV11DataUltrasonic();
	// implement parent class
	void insertData(uint16_t right1, uint16_t right2, uint16_t right3, uint16_t left1, uint16_t left2, uint16_t left3, uint16_t front);
	uint16_t getRightFront();
	uint16_t getRightSide();
	uint16_t getRightBack();
	uint16_t getLeftFront();
	uint16_t getLeftSide();
	uint16_t getLeftBack();
	uint16_t getFront();
	
	void packString(char* str);
	void unpackString(char* str);
};
#endif