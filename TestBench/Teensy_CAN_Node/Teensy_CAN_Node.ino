/*
 Name:		Teensy_CAN_Node.ino
 Created:	1/29/2020 8:47:57 PM
 Author:	Nicho
*/

// Teensy Node for testbench the CAN bus with the Teensy

#include <FlexCAN.h>

void hexDump(uint8_t len, uint8_t* buf) {
	for (uint8_t i = 0; i < 8; i++) {
		Serial.print(buf[i]); Serial.print(" ,");
	}
	Serial.println("\n");
}

// the setup function runs once when you press reset or power the board
void setup() {
	delay(1000);
	Serial.begin(9600);
	Can0.begin(1000000);
	delay(500);
}

// the loop function runs over and over again until power down or reset
void loop() {

	// Polling
	CAN_message_t inMsg;
	while (Can0.available()) {
		Can0.read(inMsg);
		Serial.println("CAN message received ... ");
		Serial.print("ID: "); Serial.println(inMsg.id);
		Serial.print("Length: "); Serial.println(inMsg.len);
		hexDump(inMsg.len, inMsg.buf);
	}
	Serial.println("End of loop");
	delay(20);
}
