#include "NV11DataUltrasonic.h"

NV11DataUltrasonic::NV11DataUltrasonic() : DataPoint("US", 0x0A, 8)
{
	debug(F("DataPoint US:\t0x0A\t32"));
}

/*
The HC-SR04 ultrasonic sensor has a range of up to 400cm. That can be represented using uint16_t (2 bytes). 
But to send all readings through the CAN bus in a single CAN frame, we can only represent each distance using 1 byte.
Hence we divide each distance arguement by 2 before assigning to is respective class attribute.
*/
void NV11DataUltrasonic::insertData(uint16_t right1, uint16_t right2, uint16_t right3, uint16_t left1, 
	uint16_t left2, uint16_t left3, uint16_t front)
{
	timeStamp = millis();
	this->rightFront = (right1/2);
	this->rightSide = (right2/2);
	this->rightBack = (right3/2);
	this->leftFront = (left1/2);
	this->leftSide = (left2/2);
	this->leftBack = (left3/2);
	this->front = (front/2);
}

/*
	In the teensy, we need to convert the CAN frame into a NV11DataUltrasonic object. But the values in the CAN frame are already
	divided by 2. So we assign them to the class attributes as is.
*/
void NV11DataUltrasonic::insertDataTeensy(uint8_t right1, uint8_t right2, uint8_t right3, uint8_t left1,
	uint8_t left2, uint8_t left3, uint8_t front) {
	timeStamp = millis();
	this->rightFront = right1;
	this->rightSide = right2 ;
	this->rightBack = right3;
	this->leftFront = left1 ;
	this->leftSide = left2 ;
	this->leftBack = left3 ;
	this->front = front ;
}

uint16_t NV11DataUltrasonic::getRightFront()
{
	return (rightFront*2);
}

uint16_t NV11DataUltrasonic::getRightSide()
{
	return (rightSide*2);
}

uint16_t NV11DataUltrasonic::getRightBack()
{
	return (rightBack*2);
}

uint16_t NV11DataUltrasonic::getLeftFront()
{
	return (leftFront*2);
}

uint16_t NV11DataUltrasonic::getLeftSide()
{
	return (leftSide*2);
}

uint16_t NV11DataUltrasonic::getLeftBack()
{
	return (leftBack*2);
}

uint16_t NV11DataUltrasonic::getFront() {
	return (front*2);
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