/*
 Name:		NV11_autonomous_MEGA.ino
 Created:	10/18/2019 2:11:25 PM
 Author:	Laksh
*/

#include <ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

void driveMessage(const ackermann_msgs::AckermannDriveStamped& drive_msg) {
	Drive = drive_msg.drive;
	Steering.data = Drive.steering_angle;
	Accel.data = Drive.acceleration;
	Brake.data = Drive.jerk;
	Accel.data = steering_angle;
	Serial.print("Ackermann Data: ");
	Serial.print("Steering angle - ");
	Serial.print(steering_angle);
	Serial.print(" Speed - ");
	Serial.print(speed);
	Serial.print(" Jerk - ");
	Serial.println(jerk);
}

// Define ROS variables
std_msgs::Float64 Distance;
std_msgs::Float64 Accel;
std_msgs::Float64 Brake;
std_msgs::Float64 Steering;
ackermann_msgs::AckermannDriveStamped drive_msg;
ackermann_msgs::AckermannDrive Drive;
ros::NodeHandle nh;
ros::Publisher chatter("chatter", &Distance);
ros::Publisher accel("Accel", &Accel);
ros::Publisher brake("Brake", &Brake);
ros::Publisher steering("Steering", &Steering);
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub("arduino_cmd_topic", &driveMessage);

// Define pins numbers
const int trigPin = 9;
const int echoPin = 10;

// Define variables
long duration;
float distance;

float steering_angle;
float speed;
float jerk;

void setup() {
	nh.initNode();
	nh.advertise(chatter);
	nh.subscribe(sub);
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	Serial.begin(57600);
}

void loop() {
	// publish data
	Distance.data = distance;
	chatter.publish(&Distance);
	accel.publish(&Accel);
	brake.publish(&Brake);
	steering.publish(&Steering);
	nh.spinOnce();
	delay(1);
}

void getUltrasonic() {
	// Clear the trigPin
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	// Read the echoPin, returns the sound wave travel time in microseconds
	duration = pulseIn(echoPin, HIGH);

	// Calculate the distance
	distance = duration * 0.034 / 2;
}