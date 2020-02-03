/*
 * Test Node for Arduino that is connected to Nvidia Jetson. Subscribes to node and sends data through CAN bus.
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <CANSerializer.h>
#include <NV11DataSpeedo.h>

#define CAN_CS 10
#define LedPin 13

ros::NodeHandle nh;
CANSerializer serializer;
NV11DataSpeedo speedData;

void twistMessage(const std_msgs::Float32& twist_msg) {
	std_msgs::Float32 twist = twist_msg;
	// Write CAN code here
	CANFrame f;
	speedData.insertData(twist.data);
	speedData.packCAN(&f);
	bool sent = serializer.sendCanFrame(&f);

	// Turn LED on Arduino ON when CAN frame is sent successfully
	if (sent) {
		digitalWrite(LedPin, HIGH);
	}
	else {
		digitalWrite(LedPin, LOW);
	}
	delay(1000);
	digitalWrite(LedPin, LOW);
}

ros::Subscriber<std_msgs::Float32> sub("nv11/steering_angle", &twistMessage);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	pinMode(LedPin, OUTPUT);
	// Init CAN bus
	serializer.init(CAN_CS);
}

void loop() {
	nh.spinOnce();
}