#include "NV11DataUltrasonic.h"

NV11DataUltrasonic::NV11DataUltrasonic() : DataPoint("US", 0x0A, 32)
{
	debug(F("DataPoint US:\t0x0A\t32"));
}

void NV11DataUltrasonic::insertData(int right1, int right2, int right3, int left1, int left2, int left3, int front)
{
	timeStamp = millis();
	this->rightFront = right1;
	this->rightSide = right2;
	this->rightBack = right3;
	this->leftFront = left1;
	this->leftSide = left2;
	this->leftBack = left3;
	this->front = front;
}

int NV11DataUltrasonic::getRightFront()
{
	return rightFront;
}

int NV11DataUltrasonic::getRightSide()
{
	return rightSide;
}

int NV11DataUltrasonic::getRightBack()
{
	return rightBack;
}

int NV11DataUltrasonic::getLeftFront()
{
	return leftFront;
}

int NV11DataUltrasonic::getLeftSide()
{
	return leftSide;
}

int NV11DataUltrasonic::getLeftBack()
{
	return leftBack;
}

int NV11DataUltrasonic::getFront() {
	return front;
}

void NV11DataUltrasonic::packString(char* str) {
	char* shiftedStr = DataPoint::packStringDefault(str);
	sprintf(shiftedStr, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", rightFront, rightSide, rightBack, leftFront, leftSide, leftFront, front);
}

void NV11DataUltrasonic::unpackString(char* str) {
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	rightFront = atoi(ptr);

	ptr = strtok(NULL, "\t");
	rightSide = atoi(ptr);

	ptr = strtok(NULL, "\t");
	rightBack = atoi(ptr);

	ptr = strtok(NULL, "\t");
	leftFront = atoi(ptr);

	ptr = strtok(NULL, "\t");
	leftSide = atoi(ptr);

	ptr = strtok(NULL, "\t");
	leftFront = atoi(ptr);

	ptr = strtok(NULL, "\t");
	front = atoi(ptr);

}