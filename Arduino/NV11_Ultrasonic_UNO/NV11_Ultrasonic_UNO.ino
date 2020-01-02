/*
 Name:		NV11_Ultrasonic_UNO.ino
 Created:	12/31/2019 9:43:10 AM
 Author:	Nicho
*/

#include <CANSerializer.h>
#include <NV11DataUltrasonic.h>

// For calculating distance in cm from signal pin for ultrasonic sensor
#define DISTANCE_FACTOR 58.138
#define CAN_SPI_CS 10

CANSerializer serializer;
NV11DataUltrasonic dataUltra;

//Pins for ultrasonic sensors
const uint8_t sigPin1 = 2;
const uint8_t sigPin2 = 4;
const uint8_t sigPin3 = 6;
const uint8_t sigPin4 = 8;
const uint8_t sigPin5 = 10;
const uint8_t sigPin6 = 12;
const uint8_t sigPin7 = 24;

// Calculated ultrasonic distances
uint16_t distance, RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front;

// the setup function runs once when you press reset or power the board
void setup() {
	serializer.init(CAN_SPI_CS);
	ultrasonicInit();
	Serial.begin(9600);
	delay(50);
}

// the loop function runs over and over again until power down or reset
void loop() {
	// Get distance from all ultrasonic sensors
	RightFront = read_ultrasonic(sigPin1);
	RightSide = read_ultrasonic(sigPin2);
	RightBack = read_ultrasonic(sigPin3);
	LeftFront = read_ultrasonic(sigPin4);
	LeftSide = read_ultrasonic(sigPin5);
	LeftBack = read_ultrasonic(sigPin6);
	Front = read_ultrasonic(sigPin7);

	// Send distance readings through the CAN bus
	CANFrame f;
	dataUltra.insertData(RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front);
	dataUltra.packCAN(&f);
	bool sent = serializer.sendCanFrame(&f);

	// Debug purposes
	if (sent) {
		Serial.println("CAN Frame sent");
	}
	else {
		Serial.println("Cannot send CANframe");
	}

	delay(200);
}

/*
	Init ultrasonic sensor pins
*/
void ultrasonicInit() {
	//Init the ultrasonic sensor pins
	pinMode(sigPin1, OUTPUT); // Right front
	digitalWrite(sigPin1, LOW);

	pinMode(sigPin2, OUTPUT); // Right side
	digitalWrite(sigPin2, LOW);

	pinMode(sigPin3, OUTPUT); // Right back
	digitalWrite(sigPin3, LOW);

	pinMode(sigPin4, OUTPUT); // Left front
	digitalWrite(sigPin4, LOW);

	pinMode(sigPin5, OUTPUT); // Left side
	digitalWrite(sigPin5, LOW);

	pinMode(sigPin6, OUTPUT); // Left back
	digitalWrite(sigPin6, LOW);

	pinMode(sigPin7, OUTPUT); // Front
	digitalWrite(sigPin7, LOW);
}

/*
	Returns the distance reading from a single ultrasonic sensor
*/
uint16_t read_ultrasonic(uint8_t sig_pin) {
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

	return distance;
}
