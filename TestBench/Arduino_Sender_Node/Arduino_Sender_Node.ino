/*
 Name:		Sender_Node.ino
 Created:	1/20/2020 9:50:33 PM
 Author:	Nicho
*/

// This CAN Node acts as the sender for test bench testing purposes only. For the receiver node, use NV11_CANtoSerial_UNO.

#include <CANSerializer.h>
#include <NV11DataUltrasonic.h>
#include <NV11DataSpeedo.h>

#define CAN_CS 10

CANSerializer canSerializer;
NV11DataUltrasonic dataUltra;
NV11DataSpeedo dataSpeedo;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	bool setup = canSerializer.init(CAN_CS);
	// initUltrasonic();
	if (setup) {
		Serial.println("CAN set up");
	}
	else {
		Serial.println("Error setting up CAN!");
	}
}

// the loop function runs over and over again until power down or reset
void loop() {

	CANFrame frame;
	dataUltra.insertData(50, 150, 200, 250, 300, 350, 400);
	dataUltra.packCAN(&frame);
	bool sent = canSerializer.sendCanFrame(&frame);
	if (sent) {
		Serial.println("dataUltra CAN frame sent");
	}
	else {
		Serial.println("dataUltra CAN frame cannot be sent");
	}

	delay(250);

	CANFrame frame2;
	dataSpeedo.insertData(27.9);
	dataSpeedo.packCAN(&frame2);
	bool sent2 = canSerializer.sendCanFrame(&frame2);
	if (sent2) {
		Serial.println("dataSpeedo CAN frame sent");
	}
	else {
		Serial.println("dataSpeedo CAN frame cannot be sent");
	}

	delay(250);
}