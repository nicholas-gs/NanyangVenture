/*
    Name:       NV11_CANtoSerial_UNO.ino
    Created:	8/3/2019 12:06:26 AM
    Author:     DESKTOP-GV1MS6E\MX
*/
#define CAN_CS 4
#define CAN_INT 3
#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <NV11BMS.h>
#include <NV11DataSpeedo.h>
#include <NV11Commands.h>
#include <NV11DataUltrasonic.h>
CANSerializer serializer;
NV11DataSpeedo dataSpeedo;
NV11AccesoriesStatus dataAccesories;
NV11BMS dataBMS;
NV11Commands dataCommands;
NV11DataUltrasonic dataUltrasonic;

DataPoint* canListenList[] = { &dataSpeedo, &dataBMS, &dataCommands, &dataAccesories, &dataUltrasonic }; // incoming messages: speedo BMS
DataPoint* serialListenList[] = { &dataAccesories }; // outgoing messages: accessories
void setup()
{
	Serial.begin(9600);
	Serial.setTimeout(500);
	while (!Serial);
	serializer.init(CAN_CS);
	attachInterrupt(digitalPinToInterrupt(CAN_INT), CAN_ISR, FALLING);
}

void loop()
{
	CANFrame f;
	char toSerialPort[100];
	char fromSerialPort[100];
	if (serializer.receiveCanFrame(&f))
	{
		Serial.println("Receiving CAN frame"); // Debug purpose
		for (int i = 0; i < sizeof(canListenList) / sizeof(canListenList[0]); i++)
		{
			DataPoint& dataPoint = *canListenList[i];
			if (dataPoint.checkMatchCAN(&f))
			{
				Serial.println("Match Can Frame"); // Debug purpose
				dataPoint.unpackCAN(&f);
				dataPoint.packString(toSerialPort);
				Serial.println(toSerialPort);
				break;
			}
		}
	}
	if (Serial.available())
	{
		int bytesRead = Serial.readBytesUntil('\n', fromSerialPort, 100); 
		fromSerialPort[bytesRead-1] = '\0'; // "\r\n" should be the terminator, replace '\r' with '\0'
		for (int i = 0; i < sizeof(serialListenList) / sizeof(serialListenList[0]); i++)
		{
			DataPoint& dataPoint = *serialListenList[i];
			if (dataPoint.checkMatchString(fromSerialPort))
			{
				dataPoint.unpackString(fromSerialPort);
				dataPoint.packCAN(&f);
				serializer.sendCanFrame(&f);
				break;
			}
		}
	}
}
void CAN_ISR()
{

}