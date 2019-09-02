/*
Name:		CANtoSerial_UNO.ino
Created:	6/24/2018 11:37:49 PM
Author:	MX
*/
#include <CANSerializer.h>
#include <NV10AccesoriesStatus.h>
#include <NV10CurrentSensor.h>
#include <NV10CurrentSensorStats.h>
#include <NV10FuelCell.h>
#include <NV11DataSpeedo.h>
#include <NV11Commands.h>

//#define SEEDSTUDIO_CAN_SHIELD
// seedstudio CAN shield v1.2
#ifdef SEEDSTUDIO_CAN_SHIELD
#define CAN_CS 10
#define CAN_INT 2
#define CANSPEED 500
#else
#define CAN_CS 4
#define CAN_INT 3
#define CANSPEED 1000
#endif

NV10CurrentSensor dataCS;
NV10CurrentSensorStats dataCSStats;
NV10AccesoriesStatus dataAccessory;
NV10FuelCell dataFC;
NV11DataSpeedo dataSpeedo;
NV11Commands dataCommands;

DataPoint* dpRecv[] = { &dataFC, &dataCS, &dataCSStats, &dataSpeedo };
DataPoint* dpSend[] = { &dataAccessory, &dataCommands };

CANSerializer serializer;
void setup()
{
	Serial.begin(9600);
	pinMode(CAN_INT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CAN_INT), CAN_ISR, FALLING);
	if (!serializer.init(CAN_CS, CANSPEED))
	{
		delay(1000);
		asm volatile ("  jmp 0");
	}
}

void loop()
{
	CANFrame f;
	char str[100];
	if (Serial.available())
	{
		int bytesRead = Serial.readBytesUntil('\n', str, 100);
		str[bytesRead - 1] = '\0';
		for (int i = 0; i < sizeof(dpSend) / sizeof(dpSend[0]); i++)
		{
			if (dpSend[i]->checkMatchString(str))
			{
				dpSend[i]->unpackString(str);
				dpSend[i]->packCAN(&f);
				serializer.sendCanFrame(&f);
			}
		}
	}
	if (serializer.receiveCanFrame(&f))
	{
		for (int i = 0; i < sizeof(dpRecv) / sizeof(dpRecv[0]); i++)
		{
			if(dpRecv[i]->checkMatchCAN(&f))
			{
				dpRecv[i]->unpackCAN(&f);
				dpRecv[i]->packString(str);
				Serial.println(str);
			}
		}
	}
}
void CAN_ISR()
{

}