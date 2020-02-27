/*
 Name:		NV11_Ultrasonic_UNO.ino
 Created:	12/31/2019 9:43:10 AM
 Author:	Nicho
*/

/*
	The PING))) 28015 ultrasonic sensor has a range of 2cm to 3m.
*/

#include <Wire.h>

// For calculating distance in cm from signal pin for ultrasonic sensor
#define DISTANCE_FACTOR 58.138
#define NO_OF_SENSORS 7
#define SLAVE_ADDRESS 0x10

// ID & array positions for the calculated distances and signal pins
const uint8_t FRONT_ID = 0;
const uint8_t RIGHTFRONT_ID = 1;
const uint8_t RIGHTSIDE_ID = 2;
const uint8_t RIGHTBACK_ID = 3;
const uint8_t LEFTFRONT_ID = 4;
const uint8_t LEFTSIDE_ID = 5;
const uint8_t LEFTBACK_ID = 6;

//Pins for ultrasonic sensors
static uint8_t SIG_PINS[NO_OF_SENSORS] = { 8, 2, 3, 4, 5, 6, 7 };

// Array to store the calculated distances
static uint16_t distanceBuf[NO_OF_SENSORS];

// the setup function runs once when you press reset or power the board
void setup() {
	ultrasonicInit();
	Serial.begin(9600);

	// Init I2C. Join as slave with address of 0x10
	Wire.begin(SLAVE_ADDRESS);
	// Register request event -- The master will request distance data
	Wire.onRequest(requestEvent);

	delay(50);
}

// the loop function runs over and over again until power down or reset
void loop() {
	// Read in the distance from all ultrasonic sensors and store them in the distance buffer
	for (int i = 0; i < NO_OF_SENSORS; i++) {
		uint16_t d = read_ultrasonic(SIG_PINS[i]);
		distanceBuf[i] = d;

		// Debug purpose
		Serial.println(distanceBuf[i]);
	}

	Serial.println("**********************");
}

/*
	Init ultrasonic sensor pins
*/
void ultrasonicInit() {
	//Init the ultrasonic sensor pins
	pinMode(SIG_PINS[RIGHTFRONT_ID], OUTPUT); // Right front
	digitalWrite(SIG_PINS[RIGHTFRONT_ID], LOW);

	pinMode(SIG_PINS[RIGHTSIDE_ID], OUTPUT); // Right side
	digitalWrite(SIG_PINS[RIGHTSIDE_ID], LOW);

	pinMode(SIG_PINS[RIGHTBACK_ID], OUTPUT); // Right back
	digitalWrite(SIG_PINS[RIGHTBACK_ID], LOW);

	pinMode(SIG_PINS[LEFTFRONT_ID], OUTPUT); // Left front
	digitalWrite(SIG_PINS[LEFTFRONT_ID], LOW);

	pinMode(SIG_PINS[LEFTSIDE_ID], OUTPUT); // Left side
	digitalWrite(SIG_PINS[LEFTSIDE_ID], LOW);

	pinMode(SIG_PINS[LEFTBACK_ID], OUTPUT); // Left back
	digitalWrite(SIG_PINS[LEFTBACK_ID], LOW);

	pinMode(SIG_PINS[FRONT_ID], OUTPUT); // Front
	digitalWrite(SIG_PINS[FRONT_ID], LOW);
}

/*
	Returns the distance reading from a single ultrasonic sensor
*/
uint16_t read_ultrasonic(uint8_t sig_pin) {
	uint16_t distance;
	pinMode(sig_pin, OUTPUT);
	digitalWrite(sig_pin, LOW);
	delayMicroseconds(2);
	// Send logic high to trigger the ultrasonic sensor
	digitalWrite(sig_pin, HIGH);
	delayMicroseconds(5);
	// Turn off trigger signal
	digitalWrite(sig_pin, LOW);
	// Config signal pin to input
	pinMode(sig_pin, INPUT);
	// Read in time the sig_pin remains high
	distance = pulseIn(sig_pin, HIGH);

	distance = distance / DISTANCE_FACTOR;
	if (distance > 300) {
		distance = 300;
	}
	return distance;
}

/*
	I2C master request data from the slave.
	For each ultrasonic data, first byte will be the ID of the sensor, the second byte is the actual data.
	REMEMBER TO MULTIPLY THE DATA BY 2 AT THE RECEIVER SIDE !!!!
*/
void requestEvent() {
	for (int i = 0; i < NO_OF_SENSORS; i++) {
		Wire.write((uint8_t)i); // Send the ID of the ultrasonic data
		Wire.write((uint8_t)(distanceBuf[i] / 2)); // Divide the data by 2 such that it can be transmitted as 1 byte
	}
}
