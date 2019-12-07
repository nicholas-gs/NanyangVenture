#ifndef _NV11DATAULTRASONIC_h
#define _NV11DATAULTRASONIC_h

#include <DataPoint.h>
class NV11DataUltrasonic :public DataPoint
{
protected:
	int rightFront = data.Int8[0];
	int rightSide = data.Int8[1];
	int rightBack = data.Int8[2];
	int leftFront = data.Int8[3];
	int leftSide = data.Int8[4];
	int leftBack = data.Int8[5];
	int front = data.Int8[6];

public:
	NV11DataUltrasonic();
	// implement parent class
	void insertData(int right1, int right2, int right3, int left1, int left2, int left3, int front);
	int getRightFront();
	int getRightSide();
	int getRightBack();
	int getLeftFront();
	int getLeftSide();
	int getLeftBack();
	int getFront();
	/*
	void packString(char*);
	void unpackString(char* str);*/
};
#endif