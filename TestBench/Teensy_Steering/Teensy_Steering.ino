/*
 Name:		Teensy_Steering.ino
 Created:	2/25/2020 4:37:43 PM
 Author:	Nicho
*/

/*
	Test bench for the Teensy controlling the steering module.

	1. Teensy has to subscribe to ros topic to get the steering information
	2. Teensy has to read the current steering angle from the absolute encoder
	3. Teensy has to control the HSS57 Hybrid Servo driver based on 1 and 2
*/

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <AccelStepper.h>

// Stepper pin configurations
#define STEERDPIN 24
#define STEERPPIN 25

// Removes Jitter in the steps, this is the range where the motor will ignore step commands
#define JITTERZONE 3

// Rotary Encoder I2C address
#define ROTARY_ENCODER_ADDRESS 0x12

// Assign the steering angle as defined by ROS in the subscriber handler
static int targetSteeringAngle = 0;
static int targetSteeringStep = 0;
static int stepSLast = 0;

// Assign the steering angle read from the Rotary Encoder
static int currentSteeringAngle = 0;

// Steering stepper motor control object
AccelStepper steering(1, STEERPPIN, STEERDPIN);

// ROS Node handler
ros::NodeHandle nh;

void steeringHandler(const std_msgs::Float32& twist_msg);
// ros::Subscriber<std_msgs::Float32> sub("nv11/steering_angle", &steeringHandler);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	Wire.begin();

	/*
		When AccelStepper starts it has no idea where the motors are, hence the need to home 
		the motors and set the homed position to 0 (or some reference position).
	*/
	steering.setCurrentPosition(0);
}

// the loop function runs over and over again until power down or reset
void loop() {
	// RequestRotaryData();
	targetSteeringStep = targetSteeringAngle;
	if (targetSteeringAngle > stepSLast + JITTERZONE || targetSteeringAngle < stepSLast - JITTERZONE) {
		steering.move(targetSteeringStep);
		stepSLast = targetSteeringAngle;
	}
	steering.run();
}

/*
	Handler for steering message from ROS topic
*/
void steeringHandler(const std_msgs::Float32& twist_msg) {
	std_msgs::Float32 twist = twist_msg;
	targetSteeringAngle = (int)twist.data;
}

/*
	Get absolute encoder value
*/
void RequestRotaryData() {
	Wire.requestFrom(ROTARY_ENCODER_ADDRESS, 4);
	while (Wire.available()) {

	}
}