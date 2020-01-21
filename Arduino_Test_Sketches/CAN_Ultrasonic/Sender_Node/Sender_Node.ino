/*
 Name:		Sender_Node.ino
 Created:	1/20/2020 9:50:33 PM
 Author:	Nicho
*/

// This CAN Node acts as the sender for test bench testing purposes only. For the receiver node, use NV11_CANtoSerial_UNO.

#include <CANSerializer.h>
#include <NV11DataUltrasonic.h>
#include <NV11DataSpeedo.h>

#define CAN_CS 9

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
	dataUltra.insertData(400, 400, 400, 400, 400, 400, 400);
	dataUltra.packCAN(&frame);
	bool sent = canSerializer.sendCanFrame(&frame);
	if (sent) {
    Serial.println("CAN frame sent");
	}
	else {
    Serial.println("Cannot send CAN frame");
	}

/*
  
	CANFrame frame;
	dataSpeedo.insertData(23.2);
	dataSpeedo.packCAN(&frame);
	bool sent = canSerializer.sendCanFrame(&frame);
  Serial.println("Tried to send");
	if (sent) {
		Serial.println("CAN frame sent");
	}
	else {
		Serial.println("Cannot send CAN frame");
	}
 */
 delay(500);
} 


// Init the pins for the ultrasonic sensors

void initUltrasonic() {

}
