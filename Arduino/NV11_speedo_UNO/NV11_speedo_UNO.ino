/*
 Name:		NV11_speedo_UNO.ino
 Created:	2/18/2019 3:14:55 PM
 Author:	MX
*/
#include <ILI9488.h>
#include <CANSerializer.h>
#include <NV11DataSpeedo.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>

#include "Speedometer.h"
#include "Pins_speedo.h"


CANSerializer serializer;
NV11DataSpeedo dataSpeedo;

Speedometer speedoBL = Speedometer(SPEEDO_BL_A, SPEEDO_BL_B, 545, 500, true);
Speedometer speedoBR = Speedometer(SPEEDO_BR_A, SPEEDO_BR_B, 545, 500, false);

bool CAN_avail;
//ros::NodeHandle nh;//_<ArduinoHardware, 10, 10, 100, 105, ros::DefaultReadOutBuffer_>
//std_msgs::Float32MultiArray speedData;
//std_msgs::UInt32MultiArray distData;
//ros::Publisher speedPublisher("speedKmh", &speedData), distPublisher("distMm", &distData);
//ros::Subscriber 

void setup() {
	delay(500);
	Serial.begin(9600);
	pinMode(CANENABLE_OUTPUT, OUTPUT);
	CAN_avail = serializer.init(CAN_CS);
	if (CAN_avail)
	{
		digitalWrite(CANENABLE_OUTPUT, HIGH);
		Serial.println("CAN init");
	}
	else
	{
		digitalWrite(CANENABLE_OUTPUT, LOW);
		Serial.println("CAN fail");
	}
	pinMode(SPEEDO_BL_A, INPUT_PULLUP);
	pinMode(SPEEDO_BL_B, INPUT_PULLUP);
	pinMode(SPEEDO_BR_A, INPUT_PULLUP);
	pinMode(SPEEDO_BR_B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(SPEEDO_BL_A), tickL, FALLING);
	attachInterrupt(digitalPinToInterrupt(SPEEDO_BR_A), tickR, FALLING);

	//Serial.println("NV11 Speedo");
	
	//nh.getHardware()->setBaud(9600);

	//nh.initNode();
	//nh.advertise(speedPublisher);
	//nh.advertise(distPublisher);
	// in RosSerial AVR port, only a length and a pointer is supplied. Pointer should point towards a pre-allocated array
	//speedData.data_length = 2;
	//distData.data_length = 2;
	//nh.subscribe(&dummyScr);
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint8_t counter = 0;
	static float speed[2];
	static uint32_t dist[2];
	counter++;

	//Serial.print(speedoBL.getSpeedKmh()); Serial.print("\t");
	//Serial.print(speedoBL.getTotalDistTravelled()); Serial.print("\t");
	//Serial.print(speedoBR.getSpeedKmh()); Serial.print("\t");
	//Serial.print(speedoBR.getTotalDistTravelled()); Serial.println();

	speed[0] = speedoBL.getSpeedKmh();
	speed[1] = speedoBR.getSpeedKmh();

	// ------------------ ROS SERIAL thingy ---------------------
	//speedData.data = speed;
	//speedPublisher.publish(&speedData);
	//dist[0] = speedoBL.getTotalDistTravelled();
	//dist[1] = speedoBR.getTotalDistTravelled();
	//distData.data = dist;
	//
	//speedPublisher.publish(&speedData);
	//distPublisher.publish(&distData);
	//nh.spinOnce();

	if (counter % 4 == 0)
	{
		CANFrame f;
		dataSpeedo.insertData((speed[0]+speed[1])/2);
		dataSpeedo.packCAN(&f);
		serializer.sendCanFrame(&f);
		CAN_avail = serializer.checkNoError();
	}
	digitalWrite(CANENABLE_OUTPUT, HIGH^CAN_avail);
	delay(100);
	digitalWrite(CANENABLE_OUTPUT, LOW^CAN_avail);
	delay(100);
}

void tickL()
{
	speedoBL.trip();
}
void tickR()
{
	speedoBR.trip();
}