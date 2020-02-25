/*
 Name:		Teensy_Ultrasonic.ino
 Created:	2/25/2020 4:05:50 PM
 Author:	Nicho
*/

/*
	Test bench for the Teensy requesting data from the ultrasonic data over I2C.

	ID of the respective ultrasonic sensors, which also acts as the array position in the buffer to store the reading
	***************************************************
	FRONT_ID = 0;
	RIGHTFRONT_ID = 1;
	RIGHTSIDE_ID = 2;
	RIGHTBACK_ID = 3;
	LEFTFRONT_ID = 4;
	LEFTSIDE_ID = 5;
	LEFTBACK_ID = 6;
	***************************************************
*/

#include <Wire.h>

#define ULTRASONIC_SLAVE_ADDRESS 0x10

// the setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();
	Serial.begin(9600);
}

const uint8_t NO_OF_SENSORS = 7;

// Buffer to store the ultrasonic data
static uint16_t distanceBuf[NO_OF_SENSORS];

// the loop function runs over and over again until power down or reset
void loop() {
	Wire.requestFrom(ULTRASONIC_SLAVE_ADDRESS, (NO_OF_SENSORS * 2));
	while (Wire.available()) {
		uint8_t id = Wire.read();
		uint16_t temp = Wire.read() * 2;
		distanceBuf[id] = temp;
	}

	// Debug Purpose
	for (int i = 0; i < NO_OF_SENSORS; i++) {
		Serial.print("ID : "); Serial.print(i); Serial.print(" : "); Serial.println(distanceBuf[i]);
	}
	Serial.println("***************************");

	// The master should not request ultrasonic data reading from the slave too often
	// as it takes some time for the arduino uno to read in the distances from all the sensors
	delay(500);
}
