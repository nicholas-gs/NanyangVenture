/*
 Name:		Teensy_Jetson_Serial.ino
 Created:	2/11/2020 3:07:13 PM
 Author:	Nicho
*/

/*
	This code is for testing the Teensy that is connected to Jetson.
	1) Subscribe to ros topic

*/

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;


/*
	Function called when msg received from ROS
*/
void twistMessage(const std_msgs::Float32& twist_msg) {
	
	std_msgs::Float32 twist = twist_msg;
	int x = (int)twist.data;

	if (x < 0) {
		digitalWrite(13, HIGH);
		delay(250);
		digitalWrite(13, LOW);
	}
	
	Wire.beginTransmission(9);
	Wire.write(x);
	Wire.endTransmission();
}

void accelMessage(const std_msgs::Float32& accel_msg) {

	std_msgs::Float32 twist = accel_msg;
	int x = (int)twist.data;

	Wire.beginTransmission(9);
	Wire.write(x);
	Wire.endTransmission();
}

ros::Subscriber<std_msgs::Float32> sub("nv11/steering_angle", &twistMessage);
// ros::Subscriber<std_msgs::Float32> sub2("nv11/accel", &accelMessage);

// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	// Init Serial4
	// Serial4.begin(9600);
	Wire.begin();

	nh.initNode();
	nh.subscribe(sub);
  //nh.subscribe(sub2);
}

// the loop function runs over and over again until power down or reset
void loop() {
	nh.spinOnce();
}
